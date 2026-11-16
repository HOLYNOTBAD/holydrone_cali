#include <camera_driver_ov2710/jpeg_frame.h>
#include <gtest/gtest.h>

using camera_driver_ov2710::trimJpegPadding;

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

// These are marker-level fixtures; full decoding is verified on camera frames.
TEST(JpegFrame, KeepsUnpaddedFrame) {
  std::vector<unsigned char> data{0xff, 0xd8, 0x12, 0x34, 0xff, 0xd9};
  const auto original = data;
  EXPECT_TRUE(trimJpegPadding(&data));
  EXPECT_EQ(original, data);
}

TEST(JpegFrame, TrimsVariableLengthPadding) {
  const std::vector<unsigned char> jpeg{0xff, 0xd8, 0x12, 0x34, 0xff, 0xd9};
  for (std::size_t count = 1; count <= 16; ++count) {
    auto data = jpeg;
    data.insert(data.end(), count, 0);
    EXPECT_TRUE(trimJpegPadding(&data));
    EXPECT_EQ(jpeg, data);
  }
}

TEST(JpegFrame, TrimsNonzeroPadding) {
  std::vector<unsigned char> data{0xff, 0xd8, 0x12, 0xff, 0xd9, 0x80, 0xff};
  EXPECT_TRUE(trimJpegPadding(&data));
  EXPECT_EQ(5u, data.size());
}

TEST(JpegFrame, RejectsTruncatedOrInvalidFrames) {
  for (auto data : std::vector<std::vector<unsigned char>>{
      {}, {0xff}, {0xff, 0xd8}, {0xff, 0xd8, 0, 0},
      {0, 0, 0xff, 0xd9}}) {
    const auto original = data;
    EXPECT_FALSE(trimJpegPadding(&data));
    EXPECT_EQ(original, data);
  }
  EXPECT_FALSE(trimJpegPadding(nullptr));
}
