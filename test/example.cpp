void PcapProcessor::process_data_cpoint_(const InnoDataPacket& pkt) {
  uint32_t return_number;
  uint32_t unit_size;
  // summary date package frame and sub frame
  summary_package_.summary_data_package(pkt);
  InnoDataPacketUtils::get_block_size_and_number_return(pkt, &unit_size, &return_number);
  double frame_timestamp_sec = pkt.common.ts_start_us / kUsInSecond;
  const InnoBlock* block = reinterpret_cast<const InnoBlock*>(&pkt.inno_block1s[0]);
  for (uint32_t i = 0; i < pkt.item_number; i++,
                block = reinterpret_cast<const InnoBlock*>(reinterpret_cast<const char*>(block) +
                                                           unit_size)) {
    // calculate (x,y,z) cartesian coordinate from spherical coordinate
    // for each point in the block
    // 1. use get_full_angles() to restore angle for each channel
    // 2. use get_xyzr_meter() to calculate (x,y,z)
    InnoBlockFullAngles full_angles;
    InnoDataPacketUtils::get_block_full_angles(&full_angles, block->header);
    for (uint32_t channel = 0; channel < kInnoChannelNumber; channel++) {
      for (uint32_t m = 0; m < return_number; m++) {
        const InnoChannelPoint& pt = block->points[InnoBlock2::get_idx(channel, m)];

        InnoXyzrD xyzr;
        if (pt.radius > 0) {
          InnoDataPacketUtils::get_xyzr_meter(
              full_angles.angles[channel], pt.radius, channel, &xyzr);

          add_point_to_pcd_recorder_(pkt.common.ts_start_us,
                                     pkt.idx,
                                     xyzr.x,
                                     xyzr.y,
                                     xyzr.z,
                                     pt.refl,
                                     channel,
                                     block->header.in_roi,
                                     block->header.facet,
                                     m,
                                     pkt.confidence_level,
                                     pt.type,
                                     pt.elongation,
                                     frame_timestamp_sec + block->header.ts_10us / k10UsInSecond,
                                     block->header.scan_id,
                                     block->header.scan_idx,
                                     pkt.use_reflectance);
        }
      }
    }
  }

  return;
}