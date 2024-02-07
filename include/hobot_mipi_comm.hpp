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

#ifndef MIPI_MIPI_COMM_HPP_
#define MIPI_MIPI_COMM_HPP_

#include <string>

extern "C" {
#include <string.h>
#include <stdlib.h>
}

namespace mipi_cam
{

struct NodePara {
  std::string config_path_;
  std::string video_device_name_;
  int channel_;
  std::string camera_info_url_;
  std::string camera_calibration_file_path_;
  std::string out_format_name_;
  int image_width_;
  int image_height_;
  int framerate_;
  std::string gdc_file_path_;
  int rotate_degree_;
};

typedef struct {
  std::string config_path;
  std::string sensor_type;
  std::string out_format_name;
  int width;
  int height;
  int fps;
  int channel_;
  int need_gdc_;
	int rotate_degree_;
  char gdc_file_path_[256];
} MIPI_CAP_INFO_ST;

typedef struct sensor_id {
  int i2c_bus;           // sensor挂在哪条总线上
  int i2c_dev_addr;      // sensor i2c设备地址
  int i2c_addr_width;    // 总线地址宽（1/2字节）
  int det_reg;           // 读取的寄存器地址
  char sensor_name[10];  // sensor名字
} SENSOR_ID_T;

typedef struct {
  bool reset_flag;
  int reset_gpio;
  int reset_level;
  int i2c_bus;
  int mipi_host;
} BOARD_CONFIG_ST;

#define I2C_ADDR_8    1
#define I2C_ADDR_16   2

#define ARRAY_SIZE(a) ((sizeof(a) / sizeof(a[0])))


// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char *res, int max);


}  // namespace mipi_cam
#endif  // MIPI_MIPI_COMM_HPP_
