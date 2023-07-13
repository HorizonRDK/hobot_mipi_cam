// Copyright (c) 2022，Horizon Robotics.
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

#include <memory>
#include <string>
#include <fstream>
#include "hobot_mipi_cap.hpp"
#include "hobot_mipi_cap_iml.hpp"
#include <rclcpp/rclcpp.hpp>

namespace mipi_cam {
std::shared_ptr<HobotMipiCap> createMipiCap(const std::string &dev_name) {
  std::shared_ptr<HobotMipiCap> cap_ptr;
  if (dev_name == "RDKX3") {
    cap_ptr = std::make_shared<HobotMipiCapImlRDKX3>();
  } else if (dev_name == "x3sdb") {
    cap_ptr = std::make_shared<HobotMipiCapImlSDB>();
  } else if (dev_name == "RDKX3_m") {
    cap_ptr = std::make_shared<HobotMipiCapImlRDKX3_m>();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_factory"),
    "This is't support device type, only support x3sdb RDKX3 and RDKX3_m.\n");
  }
  return cap_ptr;
}

std::string getBoardType() {
  int board_type = 0;
  bool auto_detect = false;
  std::ifstream som_name("/sys/class/socinfo/som_name");
  if (som_name.is_open()) {
    som_name >> board_type;
    auto_detect = true;
  }
  std::string board_type_str;
  if (auto_detect) {
    switch (board_type) {
      case 3:
      case 4:
        board_type_str = "x3sdb";
        break;
      case 5:
      case 6:
      case 8:
      case 9:
        board_type_str = "RDKX3";
        break;      
      case 11:
      case 12:
      case 13:
      case 14:
      case 15:
      case 16:
        board_type_str = "RDKX3_m";
        break;
      default:
        break;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_factory"),
    "board_type %s\n", board_type_str.c_str());
  return board_type_str;
}

}  // namespace mipi_cam
