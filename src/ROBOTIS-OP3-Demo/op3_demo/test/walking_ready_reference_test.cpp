// Acuan init pose page 2 yang dipakai demo D60: loader berkasnya, pembaca page
// dari .bin, dan -- dengan berkas SUNGGUHAN di repo -- bahwa acuan masih dari
// page 2 yang sekarang dan semua berkas versi sudah digeser ke acuan itu.
// Gagal di sini = jalankan scripts/walking_ready_fk.py --write.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "op3_demo/studio_walking_versions.h"

using robotis_op::loadStudioWalkingFile;
using robotis_op::loadWalkingPoseReference;
using robotis_op::readActionPageLegRaw;

namespace
{

const std::string kSource = OP3_DEMO_SOURCE_DIR;
const std::string kStudioData = kSource + "/../../bascorro_studio/data";
// actionFile di berkas acuan relatif terhadap akar workspace (tempat src/).
const std::string kWorkspace = kSource + "/../../..";
const std::vector<std::string> kFields = {
  "init_x_offset", "init_z_offset", "init_roll_offset",
  "init_pitch_offset", "init_yaw_offset", "hip_pitch_offset",
};

std::string writeTemp(const std::string &name, const std::string &text)
{
  const std::string path = ::testing::TempDir() + name;
  std::ofstream(path, std::ios::binary) << text;
  return path;
}

}  // namespace

TEST(WalkingReadyReference, RepositoryReferenceIsFromTheCurrentPage2)
{
  const auto reference = loadWalkingPoseReference(kStudioData + "/walking_ready_reference.json", kFields);
  ASSERT_TRUE(reference.ok) << reference.error;
  EXPECT_EQ(reference.page, 2);
  ASSERT_EQ(reference.leg_raw.size(), 12u);
  for (const std::string &field : kFields)
    EXPECT_TRUE(reference.offsets.count(field)) << field;

  std::map<std::string, int> now_raw;
  std::string error;
  ASSERT_TRUE(readActionPageLegRaw(kWorkspace + "/" + reference.action_file, reference.page, now_raw, error))
      << error;
  EXPECT_EQ(now_raw, reference.leg_raw) << "page 2 sudah di-teach ulang; acuan basi";
}

TEST(WalkingReadyReference, RepositoryVersionFilesCarryTheReferenceTag)
{
  const auto reference = loadWalkingPoseReference(kStudioData + "/walking_ready_reference.json", kFields);
  ASSERT_TRUE(reference.ok) << reference.error;
  for (const std::string &path : {kStudioData + "/walking_versions.json",
                                  kSource + "/config/d60_jalan_baru_r95.json"})
  {
    const auto file = loadStudioWalkingFile(path);
    ASSERT_TRUE(file.ok) << path << ": " << file.error;
    EXPECT_EQ(file.pose_reference_id, reference.id) << path;
  }
}

TEST(WalkingReadyReference, MissingOffsetOrWrongFormatIsRejected)
{
  const auto missing = loadWalkingPoseReference(
      writeTemp("ref_missing.json",
                R"({"format": "bascorro-studio/walking-ready-reference", "id": "uji",
                    "offsets": {"init_x_offset": 0.01, "init_z_offset": 0.04, "init_roll_offset": 0,
                                "init_yaw_offset": 0, "hip_pitch_offset": 0.14}})"),
      kFields);
  EXPECT_FALSE(missing.ok);
  EXPECT_NE(missing.error.find("init_pitch_offset"), std::string::npos) << missing.error;

  const auto foreign = loadWalkingPoseReference(
      writeTemp("ref_foreign.json", R"({"format": "bascorro-studio/walking-versions", "versions": []})"), kFields);
  EXPECT_FALSE(foreign.ok);
}

TEST(WalkingReadyReference, ReadsLegJointsOfStepZeroFromActionFile)
{
  std::string data(3 * 512, '\0');
  const std::map<std::string, int> ids = {
    {"r_hip_yaw", 7}, {"l_hip_yaw", 8}, {"r_hip_roll", 9}, {"l_hip_roll", 10}, {"r_hip_pitch", 11},
    {"l_hip_pitch", 12}, {"r_knee", 13}, {"l_knee", 14}, {"r_ank_pitch", 15}, {"l_ank_pitch", 16},
    {"r_ank_roll", 17}, {"l_ank_roll", 18},
  };
  std::map<std::string, int> expected;
  for (const auto &item : ids)
  {
    const int value = 2000 + 37 * item.second;
    const size_t offset = 2 * 512 + 64 + 2 * item.second;
    data[offset] = static_cast<char>(value & 0xFF);
    data[offset + 1] = static_cast<char>(value >> 8);
    expected[item.first] = value;
  }
  const std::string path = writeTemp("pages.bin", data);

  std::map<std::string, int> raw;
  std::string error;
  ASSERT_TRUE(readActionPageLegRaw(path, 2, raw, error)) << error;
  EXPECT_EQ(raw, expected);
  EXPECT_FALSE(readActionPageLegRaw(path, 5, raw, error));
}

TEST(WalkingReadyReference, VersionFileWithoutTagHasEmptyReferenceId)
{
  const auto file = loadStudioWalkingFile(
      writeTemp("versions_untagged.json", R"({"format": "bascorro-studio/walking-versions", "versions": []})"));
  ASSERT_TRUE(file.ok) << file.error;
  EXPECT_TRUE(file.pose_reference_id.empty());
}
