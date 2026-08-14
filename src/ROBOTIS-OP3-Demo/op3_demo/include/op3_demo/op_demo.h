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
  enum Motion_Index
  {
    InitPose = 2,
    WalkingReady = 2,   // this robot's ready pose lives on page 2 (WALKING_READY) only
    GetUpFront = 85,
    GetUpBack = 86,
    RightKick = 218, //32 inti  //218 regional //220 nasional // smpg kanan : 6 
    LeftKick = 104, //104 inti //219 regional //221 nasional // smpg kiri : 7 
    Ceremony = 27, //
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
