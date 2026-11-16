#pragma once

#include <cstddef>
#include <vector>

namespace camera_driver_ov2710 {

// UVC 缓冲区可能在 JPEG EOI 后附带对齐字节；只将 JPEG 部分传给解码器。
// SOI 或 EOI 缺失时仍拒绝该帧，不人为补造结束标记。
inline bool trimJpegPadding(std::vector<unsigned char>* data) {
  if (data == nullptr || data->size() < 4 ||
      (*data)[0] != 0xff || (*data)[1] != 0xd8) {
    return false;
  }
  for (std::size_t end = data->size(); end >= 4; --end) {
    if ((*data)[end - 2] == 0xff && (*data)[end - 1] == 0xd9) {
      data->resize(end);
      return true;
    }
  }
  return false;
}

}  // namespace camera_driver_ov2710
