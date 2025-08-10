#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <istream>
#include <optional>
#include <pcl/impl/point_types.hpp>
#include <vector>

//
#include <fmt/core.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <CLI/CLI.hpp>

//
#include "sdk_client/inno_lidar_packet_v1_adapt.h"
#include "sdk_common/inno_lidar_api.h"
#include "sdk_common/inno_lidar_other_api.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"

namespace fs = std::filesystem;

// Get all files in a directory with a specific extension
std::vector<fs::path> GetAllFiles(const fs::path& dir,
                                  const std::string_view ext = {},
                                  bool sort = true) {
  std::vector<fs::path> files;

  for (const auto& entry : fs::directory_iterator(dir)) {
    if (fs::is_regular_file(entry.path()) && (ext.empty() || entry.path().extension() == ext)) {
      files.push_back(entry.path());
    }
  }

  if (sort) {
    std::sort(files.begin(), files.end());
  }

  return files;
}

void FixInnoDataPacketV1(InnoDataPacket* p) {
  p->common.size += 16;
  p->common.version.major_version = InnoPacketV1Adapt::kInnoProtocolMajorV2;
  p->common.version.minor_version = InnoPacketV1Adapt::kInnoProtocolMinorV2;
  InnoPacketReader::set_packet_crc32(&p->common);
}

class InnoLidarParser {
 public:
  bool ProcessData(const std::vector<char>& data) {
    // Cast to header
    const auto* header = reinterpret_cast<const InnoCommonHeader*>(data.data());
    if (header->version.magic_number != kInnoMagicNumberDataPacket) {
      LOG(WARNING) << "Not data packet";
      return false;
    }

    const auto* packet = reinterpret_cast<const InnoDataPacket*>(data.data());
    FixInnoDataPacketV1(const_cast<InnoDataPacket*>(packet));
    CHECK(inno_lidar_check_data_packet(packet, 0));

    VLOG(1) << fmt::format(
        "idx: {:3d}, sub_idx: {:3d}, sub_seq: {:5d}, item: {:3d}, first: {}, last: {}",
        packet->idx,
        packet->sub_idx,
        packet->sub_seq,
        packet->item_number,
        packet->is_first_sub_frame,
        packet->is_last_sub_frame);

    DecodePayload(*packet);

    // Update prev packet
    prev_packet_ = *packet;
    return true;
  }

  void DecodePayload(const InnoDataPacket& packet) {
    if (CHECK_SPHERE_POINTCLOUD_DATA(packet.type)) {
      DecodePayloadSphere(packet);
    } else if (CHECK_XYZ_POINTCLOUD_DATA(packet.type)) {
      LOG(INFO) << "Processing XYZ_POINTCLOUD";
    }
  }

  void DecodePayloadSphere(const InnoDataPacket& packet) {
    if (packet.type != INNO_ITEM_TYPE_SPHERE_POINTCLOUD) {
      return;
    }

    uint32_t block_size{};
    uint32_t num_return{};  // Number of return, should be 1
    InnoDataPacketUtils::get_block_size_and_number_return(packet, &block_size, &num_return);

    const auto item_type = InnoItemType(packet.type);

    for (uint32_t i = 0; i < packet.item_number; ++i) {
      const auto* block = reinterpret_cast<const InnoBlock*>(packet.payload + block_size * i);
      //  LOG(INFO) << fmt::format("block: {:3d}, scan_id: {:3d}, scan_idx: {:3d}",
      //                          i,
      //                          block->header.scan_id,
      //                          block->header.scan_idx);

      InnoBlockFullAngles full_angles;
      InnoDataPacketUtils::get_block_full_angles(&full_angles, block->header, item_type);

      for (uint32_t chan = 0; chan < kInnoChannelNumber; chan++) {
        for (uint32_t ret = 0; ret < num_return; ret++) {
          const InnoChannelPoint& chan_pt = block->points[innoblock_get_idx(chan, ret)];
          InnoXyzrD xyzr;

          if (chan_pt.radius > 0) {
            InnoDataPacketUtils::get_xyzr_meter(
                full_angles.angles[chan], chan_pt.radius, chan, &xyzr);
            pcl::PointXYZ pcl_pt;
            pcl_pt.x = xyzr.x;
            pcl_pt.y = xyzr.y;
            pcl_pt.z = xyzr.z;
            cloud_.push_back(pcl_pt);
          }
        }
      }
    }
  }

  const auto& cloud() const { return cloud_; }

 private:
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  std::optional<InnoDataPacket> prev_packet_;
};

int main(int argc, char** argv) {
  CLI::App app{"Binary to PCD Converter"};
  argv = app.ensure_utf8(argv);

  std::string input_dir;
  app.add_option("-i,--input", input_dir, "Input directory")->required();

  std::string output_dir;
  app.add_option("-o,--output", output_dir, "Output directory")->required();

  int num_files;
  app.add_option("-n,--num-files", num_files, "Number of files to process (default: 100)")
      ->default_val(100);

  CLI11_PARSE(app, argc, argv);

  LOG(INFO) << input_dir;
  LOG(INFO) << output_dir;

  std::filesystem::create_directories(output_dir);

  const auto bin_files = GetAllFiles(input_dir, ".bin");
  LOG(INFO) << "Number of bin files: " << bin_files.size();

  InnoLidarParser parser;

  for (size_t i = 0; i < num_files && i < bin_files.size(); ++i) {
    const auto& bin_file = bin_files[i];

    std::ifstream ifs(bin_file, std::ios::binary);
    if (!ifs) {
      LOG(ERROR) << "Failed to open file: " << bin_file;
      continue;
    }

    // Read the binary data to a vector of bytes
    std::vector<char> data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    data.resize(data.size() + 16);

    parser.ProcessData(data);

    pcl::io::savePCDFile(fmt::format("{}/test.pcd", output_dir), parser.cloud());
  }

  return EXIT_SUCCESS;
}