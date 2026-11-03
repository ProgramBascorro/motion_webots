#include "op3_demo/studio_walking_versions.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <utility>

namespace robotis_op
{

namespace
{

std::string lowerAscii(std::string text)
{
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

}  // namespace

const std::vector<std::string> &studioWalkingNumberFields()
{
  static const std::vector<std::string> fields = {
    "init_x_offset", "init_y_offset", "init_z_offset",
    "init_roll_offset", "init_pitch_offset", "init_yaw_offset",
    "period_time", "dsp_ratio", "step_fb_ratio", "soft_start_time",
    "x_move_amplitude", "y_move_amplitude", "z_move_amplitude",
    "angle_move_amplitude", "angle_move_amplitude_turn_in",
    "balance_hip_roll_gain", "balance_knee_gain",
    "balance_ankle_roll_gain", "balance_ankle_pitch_gain",
    "y_swap_amplitude", "z_swap_amplitude", "arm_swing_gain",
    "pelvis_offset", "hip_pitch_offset",
    "p_gain", "i_gain", "d_gain",
  };
  return fields;
}

const std::vector<std::string> &studioWalkingFlagFields()
{
  static const std::vector<std::string> fields = {"move_aim_on", "balance_enable"};
  return fields;
}

StudioWalkingFile loadStudioWalkingFile(const std::string &path)
{
  StudioWalkingFile file;
  file.path = path;

  // JSON adalah YAML yang sah untuk yaml-cpp (bentuk flow), jadi tidak perlu
  // pustaka JSON tambahan. Berkas Studio memakai spasi, bukan tab.
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path);
  }
  catch (const std::exception &e)
  {
    file.error = std::string("tidak bisa dibaca: ") + e.what();
    return file;
  }

  try
  {
    if (!doc.IsMap() || !doc["versions"] || !doc["versions"].IsSequence())
    {
      file.error = "bukan berkas versi walking Bascorro Studio (tidak ada daftar 'versions')";
      return file;
    }
    if (doc["format"] && doc["format"].as<std::string>() != "bascorro-studio/walking-versions")
    {
      file.error = "format berkas tidak dikenal: " + doc["format"].as<std::string>();
      return file;
    }
    if (doc["revision"])
      file.revision = doc["revision"].as<int>();
    if (doc["savedAt"])
      file.saved_at = doc["savedAt"].as<std::string>();
    if (doc["poseReferenceId"] && doc["poseReferenceId"].IsScalar())
      file.pose_reference_id = doc["poseReferenceId"].as<std::string>();

    for (const YAML::Node &item : doc["versions"])
    {
      if (!item.IsMap() || !item["name"] || !item["params"] || !item["params"].IsMap())
        continue;
      StudioWalkingVersion version;
      version.name = item["name"].as<std::string>();
      if (item["id"])
        version.id = item["id"].as<std::string>();

      const YAML::Node params = item["params"];
      for (const std::string &field : studioWalkingNumberFields())
      {
        if (!params[field])
          continue;
        try
        {
          version.numbers[field] = params[field].as<double>();
        }
        catch (const std::exception &)
        {
          // Field yang bukan angka dianggap tidak ada; studioWalkingVersionComplete()
          // yang melaporkannya.
        }
      }
      for (const std::string &field : studioWalkingFlagFields())
      {
        if (!params[field])
          continue;
        try
        {
          version.flags[field] = params[field].as<bool>();
        }
        catch (const std::exception &)
        {
        }
      }
      file.versions.push_back(version);
    }
  }
  catch (const std::exception &e)
  {
    file.error = std::string("isi berkas rusak: ") + e.what();
    file.versions.clear();
    return file;
  }

  file.ok = true;
  return file;
}

// ID servo sendi kaki (op3_manager/config/OP3.robot).
static const std::vector<std::pair<std::string, int>> &legJointIds()
{
  static const std::vector<std::pair<std::string, int>> ids = {
    {"r_hip_yaw", 7}, {"r_hip_roll", 9}, {"r_hip_pitch", 11}, {"r_knee", 13}, {"r_ank_pitch", 15}, {"r_ank_roll", 17},
    {"l_hip_yaw", 8}, {"l_hip_roll", 10}, {"l_hip_pitch", 12}, {"l_knee", 14}, {"l_ank_pitch", 16}, {"l_ank_roll", 18},
  };
  return ids;
}

WalkingPoseReference loadWalkingPoseReference(const std::string &path, const std::vector<std::string> &fields)
{
  WalkingPoseReference reference;
  reference.path = path;
  try
  {
    const YAML::Node doc = YAML::LoadFile(path);
    if (!doc.IsMap() || !doc["format"] || doc["format"].as<std::string>() != "bascorro-studio/walking-ready-reference")
    {
      reference.error = "bukan berkas acuan page 2 (format bascorro-studio/walking-ready-reference)";
      return reference;
    }
    if (!doc["id"] || !doc["offsets"] || !doc["offsets"].IsMap())
    {
      reference.error = "id atau offsets tidak ada";
      return reference;
    }
    reference.id = doc["id"].as<std::string>();
    if (doc["generatedAt"])
      reference.generated_at = doc["generatedAt"].as<std::string>();
    const YAML::Node source = doc["source"];
    if (source && source.IsMap())
    {
      if (source["actionFile"])
        reference.action_file = source["actionFile"].as<std::string>();
      if (source["page"])
        reference.page = source["page"].as<int>();
      if (source["legRaw"] && source["legRaw"].IsMap())
        for (const auto &item : legJointIds())
          if (source["legRaw"][item.first])
            reference.leg_raw[item.first] = source["legRaw"][item.first].as<int>();
    }
    for (const std::string &field : fields)
    {
      const YAML::Node value = doc["offsets"][field];
      double number = std::nan("");
      if (value)
        number = value.as<double>();
      if (!std::isfinite(number))
      {
        // Acuan yang diam-diam jadi 0 berarti tiap preset berdiri belasan
        // derajat dari page 2 -- lebih baik mode ini menolak mulai.
        reference.error = "offset " + field + " tidak ada atau bukan angka";
        return reference;
      }
      reference.offsets[field] = number;
    }
  }
  catch (const std::exception &e)
  {
    reference.error = std::string("tidak bisa dibaca: ") + e.what();
    return reference;
  }
  reference.ok = true;
  return reference;
}

bool readActionPageLegRaw(const std::string &path, int page, std::map<std::string, int> &raw, std::string &error)
{
  constexpr std::streamoff kPageSize = 512;
  constexpr std::streamoff kStepOffset = 64;
  std::ifstream in(path, std::ios::binary);
  if (!in)
  {
    error = "tidak bisa dibuka";
    return false;
  }
  unsigned char step[62];
  in.seekg(kPageSize * page + kStepOffset);
  if (!in.read(reinterpret_cast<char *>(step), sizeof(step)))
  {
    error = "halaman " + std::to_string(page) + " tidak ada";
    return false;
  }
  raw.clear();
  for (const auto &item : legJointIds())
    raw[item.first] = static_cast<int>(step[2 * item.second]) | (static_cast<int>(step[2 * item.second + 1]) << 8);
  return true;
}

const StudioWalkingVersion *findStudioWalkingVersion(const StudioWalkingFile &file,
                                                     const std::string &exact_name,
                                                     std::vector<std::string> *similar_out)
{
  const StudioWalkingVersion *found = nullptr;
  const std::string wanted_lower = lowerAscii(exact_name);
  for (const StudioWalkingVersion &version : file.versions)
  {
    if (version.name == exact_name)
    {
      // Studio menolak nama ganda (planWalkingSave), jadi yang pertama cukup.
      if (found == nullptr)
        found = &version;
    }
    else if (similar_out != nullptr && lowerAscii(version.name) == wanted_lower)
    {
      similar_out->push_back(version.name);
    }
  }
  return found;
}

bool studioWalkingVersionComplete(const StudioWalkingVersion &version,
                                  std::vector<std::string> *missing_out)
{
  bool complete = true;
  for (const std::string &field : studioWalkingNumberFields())
  {
    const auto it = version.numbers.find(field);
    if (it == version.numbers.end() || !std::isfinite(it->second))
    {
      complete = false;
      if (missing_out != nullptr)
        missing_out->push_back(field);
    }
  }
  for (const std::string &field : studioWalkingFlagFields())
  {
    if (version.flags.find(field) == version.flags.end())
    {
      complete = false;
      if (missing_out != nullptr)
        missing_out->push_back(field);
    }
  }
  return complete;
}

}  // namespace robotis_op
