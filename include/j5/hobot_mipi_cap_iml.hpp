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

#ifndef HOBOT_MIPI_CAP_IML_HPP_
#define HOBOT_MIPI_CAP_IML_HPP_
#include <vector>
#include <string>
#include "hobot_mipi_cap.hpp"

namespace mipi_cam {

class HobotMipiCapIml : public HobotMipiCap {
 public:
  HobotMipiCapIml() {}
  ~HobotMipiCapIml() {}

  // 初始化设备环境，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  virtual int initEnv(std::string sensor);

  // 复位sensor和时钟，如X3的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  virtual int resetSensor(std::string sensor);

  // 初始化相关sensor的VIO pipeline；
  // 输入参数：info--sensor的配置参数。
  // 返回值：0，初始化成功；-1，初始化失败。
  int init(MIPI_CAP_INFO_ST &info);

  // 反初始化相关sensor的VIO pipeline ；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  int deInit();

  // 启动相关sensor的VIO pipeline的码流；
  // 返回值：0，启动成功；-1，启动失败。
  int start();

  // 停止相关sensor的VIO pipeline的码流；
  // 返回值：0，停止成功；-1，停止失败。
  int stop();

  // 遍历设备连接的sensor
  virtual std::vector<std::string> listSensor();

  // 如果有 vps ，就 输出vps 的分层数据
  int getFrame(int nChnID, int* nVOutW, int* nVOutH,
        void* buf, unsigned int bufsize, unsigned int*, uint64_t&);

  int parseConfig(std::string config_path, std::string sensor_name,
                  int w, int h, int fps);

  int UpdateConfig(MIPI_CAP_INFO_ST &info);

  // 检测对应的pipeline是否已经打开；
  // 输入参数：pipeline_idx pipeline的group ID。
  // 返回值：true，已经打开；false，没有打开。
  bool checkPipelineOpened(int pipeline_idx);

  // 获取cap的info信息；
  // 输入输出参数：MIPI_CAP_INFO_ST的结构信息。
  // 返回值：0，初始化成功；-1，初始化失败。
  int getCapInfo(MIPI_CAP_INFO_ST &info);


 protected:
  // virtual int checkConfig(std::string sensor_name, int w, int h, int fps);
  bool started_ = false;
  std::string vio_cfg_file_;
  std::string cam_cfg_file_;
  int cam_cfg_index_;
  bool vio_inited_ = false;
  bool cam_inited_ = false;
  bool use_ds_roi_ = false;
  int pipeline_idx_;
  int data_layer_ = 0xff;
  int ds_pym_layer_ = 0;
  u_int32_t src_width_;
  u_int32_t src_height_;
  MIPI_CAP_INFO_ST cap_info_;
};

class HobotMipiCapImlRDKJ5 : public HobotMipiCapIml {
 public:
  HobotMipiCapImlRDKJ5() {}
  ~HobotMipiCapImlRDKJ5() {}

  // 初始化设备环境，如J5的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  int initEnv(std::string sensor);

  // 复位sensor和时钟，如J5的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  int resetSensor(std::string sensor);

  // 判断设备是否支持遍历设备连接的sensor
  // 返回值：true,支持；false，不支持
  bool hasListSensor();

  // 遍历设备连接的sensor
  std::vector<std::string> listSensor();

  // 确认支持的分辨率和帧率。
  // int checkConfig(std::string sensor_name, int w, int h, int fps);
};


class HobotMipiCapImlJ5Evm : public HobotMipiCapIml {
 public:
  HobotMipiCapImlJ5Evm() {}
  ~HobotMipiCapImlJ5Evm() {}

  // 初始化设备环境，如J5的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  int initEnv(std::string sensor);

  // 复位sensor和时钟，如J5的sensor GPIO配置和时钟配置
  // 返回值：0，成功；-1，配置失败
  int resetSensor(std::string sensor);

  // 判断设备是否支持遍历设备连接的sensor
  // 返回值：true,支持；false，不支持
  bool hasListSensor();

  // 遍历设备连接的sensor
  std::vector<std::string> listSensor();

};


}  // namespace mipi_cam


#endif  // HOBOT_MIPI_CAP_IML_HPP_
