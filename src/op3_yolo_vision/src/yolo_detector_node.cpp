// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include "op3_yolo_vision/yolo_detector.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Use multi-threaded executor for better performance
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

  auto node = std::make_shared<op3_yolo_vision::YoloDetector>();
  executor.add_node(node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
