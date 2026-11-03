#ifndef OP3_DEMO_STUDIO_WALKING_VERSIONS_H_
#define OP3_DEMO_STUDIO_WALKING_VERSIONS_H_

#include <map>
#include <string>
#include <vector>

namespace robotis_op
{

// Versi walking yang TERSIMPAN di Bascorro Studio, dibaca langsung dari berkas
// yang ditulis Studio sendiri: src/bascorro_studio/data/walking_versions.json
// (lihat web/server/walkingVersionFile.js).
//
// Kenapa bukan config/preset_walking.yaml: berkas itu SALINAN tangan, dan
// pada 15 Sep 2026 keenam bagiannya sudah berbeda dari yang
// tersimpan di Studio (operator menyetel ulang 14 Sep). Membaca berkas Studio
// berarti mode ini selalu memakai angka yang benar-benar disimpan operator.
//
// Angka di berkas itu sudah dalam satuan WalkingParam.msg (detik, meter,
// radian) -- Studio menyimpan "nilai asli ROS", bukan tampilan Delta page 2.
struct StudioWalkingVersion
{
  std::string id;
  std::string name;
  std::map<std::string, double> numbers;  // 27 field float/int
  std::map<std::string, bool> flags;      // move_aim_on, balance_enable
};

struct StudioWalkingFile
{
  bool ok = false;
  std::string error;
  std::string path;
  int revision = -1;
  std::string saved_at;
  // Acuan page 2 tempat angka init pose versi-versi ini berlaku (ditulis
  // scripts/walking_ready_fk.py). Kosong = belum pernah digeser/ditandai.
  std::string pose_reference_id;
  std::vector<StudioWalkingVersion> versions;
};

// Acuan Init Pose page 2: src/bascorro_studio/data/walking_ready_reference.json,
// ditulis scripts/walking_ready_fk.py dari page 2 WALKING_READY di ALPHONSE.bin
// (FK lewat IK walking_module). Berkas yang SAMA dibaca Bascorro Studio
// (web/src/walkingPoseReference.js), jadi acuan yang dipatok mode ini dan titik
// nol "Relatif page 2" di Studio tidak bisa berbeda.
struct WalkingPoseReference
{
  bool ok = false;
  std::string error;
  std::string path;
  std::string id;
  std::string generated_at;
  std::string action_file;                 // relatif akar workspace
  int page = -1;
  std::map<std::string, double> offsets;   // field bias, satuan WalkingParam
  std::map<std::string, int> leg_raw;      // 12 sendi kaki page saat acuan dibuat
};

// `fields` = offset yang wajib ada dan berhingga; yang kurang = gagal.
WalkingPoseReference loadWalkingPoseReference(const std::string &path, const std::vector<std::string> &fields);

// 12 sendi kaki step 0 halaman `page` di berkas action op3_action_module
// (512 B per halaman, step 0 di +64, 31 x uint16 LE, indeks = ID servo).
bool readActionPageLegRaw(const std::string &path, int page, std::map<std::string, int> &raw,
                          std::string &error);

// 29 field WalkingParam, urutan sama dengan berkas .msg.
const std::vector<std::string> &studioWalkingNumberFields();
const std::vector<std::string> &studioWalkingFlagFields();

StudioWalkingFile loadStudioWalkingFile(const std::string &path);

// Pencarian nama PERSIS (peka huruf besar/kecil, tanpa trim). Mengembalikan
// nullptr kalau tidak ada. Nama yang hanya berbeda kapital sengaja TIDAK
// dianggap sama; kalau ada, namanya dikembalikan di similar_out supaya log bisa
// menyebutnya tanpa memakainya.
const StudioWalkingVersion *findStudioWalkingVersion(const StudioWalkingFile &file,
                                                     const std::string &exact_name,
                                                     std::vector<std::string> *similar_out = nullptr);

// true kalau ke-29 field ada dan berhingga. missing_out berisi yang kurang.
bool studioWalkingVersionComplete(const StudioWalkingVersion &version,
                                  std::vector<std::string> *missing_out = nullptr);

}  // namespace robotis_op

#endif  // OP3_DEMO_STUDIO_WALKING_VERSIONS_H_
