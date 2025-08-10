#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <istream>
#include <optional>
#include <vector>

//
#include <fmt/core.h>
#include <glog/logging.h>

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

class InnoLidarParser {
 public:
  bool ProcessData(std::vector<char> data) {
    // Cast to header
    auto* header = reinterpret_cast<InnoCommonHeader*>(data.data());

    if (header->version.magic_number == kInnoMagicNumberStatusPacket) {
      LOG(INFO) << "Status packet";
      return false;
    }

    if (header->version.magic_number != kInnoMagicNumberDataPacket) {
      LOG(WARNING) << "Not data packet";
      return false;
    }

    const auto* packet = reinterpret_cast<const InnoDataPacket*>(data.data());
    if (!inno_lidar_check_data_packet(packet, 0)) {
      LOG(INFO) << "Bad packet size";
    }

    if (header->version.major_version == InnoPacketV1Adapt::kInnoProtocolMajorV1) {
      LOG(INFO) << "Convert V1";
      int gap = InnoPacketV1Adapt::kMemorryFrontGap;
      char* ptr = data.data() + gap;
      if (!InnoPacketV1Adapt::check_data_packet_v1_and_convert_packet(&ptr, 0, gap)) {
        LOG(WARNING) << "Got bad message";
      }
    }

    // char *ptr = reinterpret_cast<char *>(data.data()) +
    //             InnoPacketV1Adapt::kMemorryFrontGap;
    // if ((packet->common.version.major_version ==
    //      InnoPacketV1Adapt::kInnoProtocolMajorV1) &&
    //     InnoPacketV1Adapt::check_data_packet_v1_and_convert_packet(&ptr, 0,
    //                                                                gap_)) {
    // }

    // LOG(INFO) << fmt::format("idx: {:3d}, sub_idx: {:3d}, sub_seq: {:5d}, "
    //                          "item: {:3d}, first: {}, last: {}",
    //                          packet->idx, packet->sub_idx, packet->sub_seq,
    //                          packet->item_number, packet->is_first_sub_frame,
    //                          packet->is_last_sub_frame);

    // const int frame_id = packet->idx;

    // if (packet->is_first_sub_frame) {
    //   start_new_frame_ = true;
    //   frame_data_.clear();
    //   LOG(INFO) << "Start new frame: " << frame_id;
    // }

    // if (start_new_frame_) {
    //   frame_data_.push_back(*packet);
    // }

    // Update prev packet
    // prev_packet_ = *packet;
    return true;
  }

 private:
  bool start_new_frame_{false};
  int gap_{InnoPacketV1Adapt::kMemorryFrontGap};
  std::vector<InnoDataPacket> frame_data_;
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

    parser.ProcessData(data);
  }

  return EXIT_SUCCESS;
}