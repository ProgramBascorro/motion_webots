/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef OP_DEMO_H_
#define OP_DEMO_H_

#include "rclcpp/rclcpp.hpp"

namespace robotis_op
{

class OPDemo
{
 public:
  // Nomor halaman aksi di ALPHONSE.bin. INI SUMBER UTAMANYA: soccer_demo,
  // vision_demo dan demo_node memakai nama-nama di bawah, bukan angka, jadi
  // cukup satu tempat ini yang diubah kalau .bin-nya di-teach ulang.
  //
  // Pemakai lain yang TIDAK bisa meng-include berkas ini menyimpan angka yang
  // sama sebagai parameter/enum sendiri, dan harus ikut disamakan:
  //   op3_gui_demo/include/op3_gui_demo/main_window.hpp   (enum Motion_Index)
  //   op3_ball_localization/config/ball_localizer.yaml    (kick_*/getup_*_page)
  //   op3_joy_teleop/config/op3_joy_teleop.yaml           (left/right_kick_page, getup_*)
  //   plus default declare_parameter() di node python masing-masing.
  enum Motion_Index
  {
    InitPose = 1,
    WalkingReady = 2,
    GetUpFront = 85,
    GetUpBack = 86,
    // Riwayat nomor tendangan di robot ini, disimpan supaya gampang dikembalikan:
    // kanan 32 (inti) / 218 (regional) / 220 (nasional) / 6 (smpg)
    // kiri  104 (inti) / 219 (regional) / 221 (nasional) / 7 (smpg)
    RightKick = 105,
    LeftKick = 104,
    Ceremony = 17,
    ForGrass = 21,
  };


  OPDemo()
  : enable_(false)
  {
  }
  virtual ~OPDemo()
  {
  }

  virtual void setDemoEnable() { enable_ = true; }
  virtual void setDemoDisable() { enable_ = false; }

  bool isDemoEnabled() { return enable_; }

 protected:
  bool enable_;
};

} /* namespace robotis_op */

#endif /* OP_DEMO_H_ */
