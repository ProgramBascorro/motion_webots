// Satu pemegang robot untuk tab training. Semua tab Studio tetap ter-mount
// (hanya disembunyikan), jadi tanpa kunci ini operator bisa memulai percobaan
// Action Training di satu tab selagi percobaan RL Training masih berjalan di tab
// lain -- dua pemberi perintah ke action/walking module yang saling timpa.

let holder = null;

// null = berhasil dikunci; selain itu nama pemegang yang sedang memakai robot.
export function lockRobot(name) {
  if (holder && holder !== name) return holder;
  holder = name;
  return null;
}

export function unlockRobot(name) {
  if (holder === name) holder = null;
}
