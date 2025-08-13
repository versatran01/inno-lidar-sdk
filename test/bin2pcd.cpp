#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <vector>

//
#include <fmt/core.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>

#include <CLI/CLI.hpp>

//
#include "sdk_client/inno_lidar_packet_v1_adapt.h"
#include "sdk_common/inno_lidar_other_api.h"
#include "sdk_common/inno_lidar_packet.h"
#include "sdk_common/inno_lidar_packet_utils.h"

namespace fs = std::filesystem;

struct PointInnoLidar {
  PCL_ADD_POINT4D;  // Adds x, y, z, and a float for padding
  union EIGEN_ALIGN16 {
    struct {
      uint8_t refl;   // 1b
      uint8_t elon;   // 1b
      uint8_t chan;   // 1b
      uint8_t scan;   // 1b
      uint16_t fire;  // 2b
      uint16_t beam;  // 2b
      int16_t elev;   // 2b
    };
  };
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Ensure correct memory alignment
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointInnoLidar,
                                  (float, x, x)           //
                                  (float, y, y)           //
                                  (float, z, z)           //
                                  (uint8_t, refl, refl)   //
                                  (uint8_t, elon, elon)   //
                                  (uint8_t, chan, chan)   //
                                  (uint8_t, scan, scan)   //
                                  (uint16_t, fire, fire)  //
                                  (uint16_t, beam, beam)  //
                                  (int16_t, elev, elev)   //
)

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

void FixInnoDataPacketV1(InnoDataPacket* p, int pad) {
  p->common.size += pad;
  p->common.version.major_version = InnoPacketV1Adapt::kInnoProtocolMajorV2;
  p->common.version.minor_version = InnoPacketV1Adapt::kInnoProtocolMinorV2;
  InnoPacketReader::set_packet_crc32(&p->common);
}

class RingIdMapper {
 public:
  int GetRingId(const int beam) const {
    CHECK_LE(0, beam);
    CHECK_LT(beam, static_cast<int>(beam2ring_.size()));
    return beam2ring_.at(beam);
  }

  bool Initialized() const { return !beam2ring_.empty(); }

  void AddBeamElevation(int beam, uint16_t elevation) {
    beam_elevations_[beam].push_back(elevation);
  }

 private:
  std::map<int, std::vector<uint16_t>> beam_elevations_;
  std::vector<int> beam2ring_;
};

class InnoLidarParser {
 public:
  bool scan_complete() const { return scan_complete_; }

  void ProcessData(const std::vector<char>& data) {
    // Cast to header
    const auto* header = reinterpret_cast<const InnoCommonHeader*>(data.data());
    if (header->version.magic_number != kInnoMagicNumberDataPacket) {
      LOG(WARNING) << "Not data packet";
      return;
    }

    // Handle V1 and V2 data
    const InnoDataPacket* packet{nullptr};
    if (header->version.major_version == InnoPacketV1Adapt::kInnoProtocolMajorV1) {
      constexpr int pad = InnoPacketV1Adapt::kMemorryFrontGap;
      fixed_.assign(data.size() + pad, 0);
      // Copy first sizeof(InnoDataPacketV1) bytes to fixed
      auto v1_size = sizeof(InnoDataPacketV1);
      std::copy(data.data(), data.data() + v1_size, fixed_.data());
      // Copy the reset of data into fixed with pad bytes offset from sizeof(InnoDataPacketV1)
      std::copy(data.data() + v1_size, data.data() + data.size(), fixed_.data() + v1_size + pad);
      FixInnoDataPacketV1(reinterpret_cast<InnoDataPacket*>(fixed_.data()), pad);
      packet = reinterpret_cast<const InnoDataPacket*>(fixed_.data());
    } else {
      packet = reinterpret_cast<const InnoDataPacket*>(data.data());
    }
    CHECK(inno_lidar_check_data_packet(packet, 0));

    // Wait for first packet
    if (!got_first_frame_) {
      if (packet->is_first_sub_frame) {
        got_first_frame_ = true;
        LOG(INFO) << "Got first frame idx: " << packet->idx;
      } else {
        LOG(INFO) << "Waiting for first frame, current idx: " << packet->idx;
        return;
      }
    }

    if (packet->is_first_sub_frame) {
      // Clear frame if it is the first packet
      cloud_.clear();
      LOG(INFO) << "Got first frame for: " << packet->idx;
    }

    if (packet->is_last_sub_frame) {
      scan_complete_ = true;
      LOG(INFO) << "Got last frame for: " << packet->idx;
    } else {
      scan_complete_ = false;
    }

    // Add packet to frame
    DecodePayload(*packet);
  }

  const auto& cloud() const { return cloud_; }

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
    CHECK_EQ(block_size, packet.item_size);

    // scanner_direction: 1 (bottom to top)
    // use_reflectance: 0
    // long_distance_mode: 0
    // multi_return_mode: 1 (single return)
    // confidence_level: 3 (highest)
    VLOG(1) << fmt::format(
        "[P] idx: {:3d}, sub_idx: {:3d}, sub_seq: {:5d}, item: {:3d}, first: {}, last: {}, conf: "
        "{}, ts_start: {}",
        packet.idx,
        packet.sub_idx,
        packet.sub_seq,
        packet.item_number,
        packet.is_first_sub_frame,
        packet.is_last_sub_frame,
        packet.confidence_level,
        packet.common.ts_start_us);

    const auto item_type = InnoItemType(packet.type);

    for (uint32_t i = 0; i < packet.item_number; i++) {
      const auto* pblock = reinterpret_cast<const InnoBlock*>(packet.payload + block_size * i);
      const auto& block = *pblock;

      InnoBlockFullAngles full_angles;
      InnoDataPacketUtils::get_block_full_angles(&full_angles, block.header, item_type);

      VLOG(2) << fmt::format("  [B] block: {:03d}, ts: {}, scan_idx: {}, scan_id: {}, facet: {}",
                             i,
                             block.header.ts_10us,
                             block.header.scan_idx,
                             block.header.scan_id,
                             block.header.facet);

      for (uint32_t chan = 0; chan < kInnoChannelNumber; chan++) {
        const auto beam = block.header.scan_id * kInnoChannelNumber + chan;

        for (uint32_t ret = 0; ret < num_return; ret++) {
          const InnoChannelPoint& pt = block.points[innoblock_get_idx(chan, ret)];
          VLOG(3) << fmt::format(
              "    [T] chan: {}, type: {}, refl: {}, radius: {}, elongation: {}, elev: {:3d}",
              chan,
              pt.type,
              pt.refl,
              pt.radius,
              pt.elongation,
              full_angles.angles[chan].v_angle);
          InnoXyzrD xyzr{};

          if (pt.radius > 0) {
            InnoDataPacketUtils::get_xyzr_meter(full_angles.angles[chan], pt.radius, chan, &xyzr);
            PointInnoLidar pcl_pt;
            pcl_pt.x = xyzr.x;
            pcl_pt.y = xyzr.y;
            pcl_pt.z = xyzr.z;
            pcl_pt.refl = pt.refl;
            pcl_pt.chan = static_cast<uint8_t>(chan);
            pcl_pt.elon = pt.elongation;
            pcl_pt.scan = block.header.scan_id;
            pcl_pt.fire = block.header.scan_idx;
            pcl_pt.beam = static_cast<uint16_t>(beam);
            pcl_pt.elev = full_angles.angles[chan].v_angle;
            cloud_.push_back(pcl_pt);
          }
        }
      }
    }
  }

 private:
  bool got_first_frame_{false};
  bool scan_complete_{false};

  std::vector<char> fixed_;
  RingIdMapper ring_mapper_;
  pcl::PointCloud<PointInnoLidar> cloud_;
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
      ->default_val(-1);

  CLI11_PARSE(app, argc, argv);

  LOG(INFO) << input_dir;
  LOG(INFO) << output_dir;

  std::filesystem::create_directories(output_dir);

  const auto bin_files = GetAllFiles(input_dir, ".bin");
  LOG(INFO) << "Number of bin files: " << bin_files.size();

  InnoLidarParser parser;

  int n = 0;

  size_t total_files = bin_files.size();
  if (num_files > 0) {
    total_files = std::min<size_t>(total_files, num_files);
  }

  for (size_t i = 0; i < total_files; ++i) {
    const auto& bin_file = bin_files[i];

    std::ifstream ifs(bin_file, std::ios::binary);
    if (!ifs) {
      LOG(ERROR) << "Failed to open file: " << bin_file;
      continue;
    }

    // Read the binary data to a vector of bytes
    std::vector<char> data((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());

    parser.ProcessData(data);

    if (parser.scan_complete()) {
      const auto pcd_file = fmt::format("{}/{:05d}.pcd", output_dir, n++);
      pcl::io::savePCDFile(pcd_file, parser.cloud());
      LOG(INFO) << "Saved PCD file: " << pcd_file;
    }
  }

  return EXIT_SUCCESS;
}