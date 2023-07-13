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

#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cap_iml.hpp"
#include "vio/hb_vio_interface.h"
#include "cam/hb_cam_interface.h"
#include "sensor_msgs/msg/camera_info.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "x3_sdk_wrap.h"
#include "x3_vio_bind.h"
#include "x3_vio_vin.h"
#include "x3_vio_vp.h"
#include "x3_vio_vps.h"
#include "x3_preparam.h"

#include <rclcpp/rclcpp.hpp>

namespace mipi_cam {

int HobotMipiCapIml::initEnv() {
  return 0;
}

int HobotMipiCapIml::init(MIPI_CAP_INFO_ST &info) {
  int ret = 0;
  cap_info_ = info;

  parseConfig(cap_info_.sensor_type, cap_info_.width, cap_info_.height, cap_info_.fps);
  int pipeline_id = 0;
  for (; pipeline_id < 8; pipeline_id++) {
    if (!checkPipelineOpened(pipeline_id)) {
      break;
    }
  }
  if (pipeline_id >= 8) {
    goto vp_err;
  }
  vin_info_.pipe_id = pipeline_id;

  ret = x3_vp_init();
  if (ret) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->hb_vp_init failed, ret: %d.\n", __func__, ret);
    goto vp_err;
  }
  // 初始化算法模块
  // 1. 初始化
  // vin，此处调用失败，大概率是因为sensor没有接，或者没有接好，或者sensor库用的版本不配套
  ret = x3_vin_init(&vin_info_);
  // {  // check分辨率
    // int sensor_w =vin_info.mipi_attr.mipi_host_cfg.width;
    // int sensor_h =vin_info.mipi_attr.mipi_host_cfg.height;
    // int input_w = m_oCamInfo.width;
    // int input_h = m_oCamInfo.height;
    // if (!checkParams(sensor_w, sensor_h, input_w, input_h)) {
    //  x3_vp_deinit();
    //  return -3;
    // }

  if (ret) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x3_vin_init failed: %d!\n", ret);
    if (HB_ERR_VIN_SIF_INIT_FAIL == abs(ret)) {  //重复打开
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "Cannot open '%s'! You may have started mipi_cam repeatedly?",
        cap_info_.sensor_type);
    }
    x3_vp_deinit();
    return -2;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_vin_init ok!\n");
  // 2. 初始化 vps，创建 group
  if (vps_enable_) {
    ret = x3_vps_init_wrap(&vps_infos_.m_vps_info[0]);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "x3_vps_init failed = %d.\n", ret);
      goto vps_err;
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_vps_init_wrap ok!\n");
  }
  // 4 vin bind vps
  if (vin_enable_ && vps_enable_) {
    ret = x3_vin_bind_vps(
        vin_info_.pipe_id,
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vin_info_.vin_vps_mode);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "[%s]x3_vin_bind_vps failed, %d.\n", __func__, ret);
      goto vps_bind_err;
    }
  }
  HB_ISP_GetSetInit();
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "x3 camera init success.\n");
  return ret;
vps_bind_err:
  if (vps_enable_) {
    x3_vps_uninit_wrap(&vps_infos_.m_vps_info[0]);
  }
vps_err:
  if (vin_enable_) {
    x3_vin_deinit(&vin_info_);
  }
vin_err:
  x3_vp_deinit();
vp_err:
  RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
    "[%s]->vp create err.\n", __func__);
  return -1;
}

int HobotMipiCapIml::deInit() {
  int i = 0;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "x3_cam_deinit start.\n");
  if (vin_enable_ && vps_enable_) {
    x3_vin_unbind_vps(
        vin_info_.pipe_id,
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        vin_info_.vin_vps_mode);
  }
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++)
      x3_vps_uninit_wrap(&vps_infos_.m_vps_info[i]);
  }
  if (vin_enable_) {
    x3_vin_deinit(&vin_info_);
  }
  x3_vp_deinit();
  HB_ISP_GetSetExit();
  return 0;
}

int HobotMipiCapIml::start() {
  int i = 0, ret = 0;
  // 使能 vps
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++) {
      ret = x3_vps_start(
          vps_infos_.m_vps_info[i].m_vps_grp_id);
      if (ret) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "x3_vps_start failed, %d", ret);
        return -3;
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_vps_start ok!");
  }
  if (vin_enable_) {
    ret = x3_vin_start(&vin_info_);
    if (ret) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "x3_vin_start failed, %d", ret);
      return -3003;
    }
  }
  started_ = true;
  return 0;
}

int HobotMipiCapIml::stop() {
  int i = 0, ret = 0;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x3 camera isn't started");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_mipi_cam_stop start.\n");
  if (vin_enable_) {
    x3_vin_stop(&vin_info_);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_mipi_cam_stop groupNum=%d.\n",
             vps_infos_.m_group_num);
  // stop vps
  if (vps_enable_) {
    for (i = 0; i < vps_infos_.m_group_num; i++) {
      x3_vps_stop(vps_infos_.m_vps_info[i].m_vps_grp_id);
      if (ret) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "x3_vps_stop failed, %d", ret);
        return -1;
      }
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"), "x3_mipi_cam_stop end.\n");
  return 0;
}

std::vector<std::string> HobotMipiCapIml::listSensor() {
  std::vector<std::string> device;
  return device;
}

int HobotMipiCapIml::getFrame(int nChnID, int* nVOutW, int* nVOutH,
        void* frame_buf, unsigned int bufsize, unsigned int* len,
        uint64_t &timestamp) {
  int size = -1, ret = 0;
  struct timeval select_timeout = {0};
  hb_vio_buffer_t vOut;
  if (!started_) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "x3 camera isn't started");
    return -1;
  }
  int i = 0;
  int stride = 0, width = 0, height = 0;
  int nTry = 0;
  do {
    ret = HB_VPS_GetChnFrame(
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        nChnID,
        &vOut,
        1000);
    if (ret < 0) {
      if (++nTry > 3) {
        if (ret == -268696577 || ret == -268696579 ||
            ret == -268696580) {  // group不存在或非法
           RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
            "Invalid groupID: %d! Please check if this group has been "
            "closed in unexpected places!",
            vps_infos_.m_vps_info[0].m_vps_grp_id);
          return -1;
        } else if (ret == -268696581 ||
                   -268696587 == ret) {  // 通道未使能或不存在
           RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
            "Invalid chn:%d in groupID: %d! Please check if this channel "
            "has been closed in unexpected places!",
            nChnID,
            vps_infos_.m_vps_info[0].m_vps_grp_id);
          return -1;
        }
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "[GetVpsFrame]->HB_VPS_GetChnFrame chn=%d,goupID=%d,error =%d "
          "!!!\n",
          nChnID,
          vps_infos_.m_vps_info[0].m_vps_grp_id,
          ret);
        return -1;
      }
      usleep(10 * 1000);
      continue;
    }
    timestamp = vOut.img_info.tv.tv_sec * 1e9 + vOut.img_info.tv.tv_usec * 1e3;
    size = vOut.img_addr.stride_size * vOut.img_addr.height;
    stride = vOut.img_addr.stride_size;
    width = vOut.img_addr.width;
    height = vOut.img_addr.height;
    *nVOutW = width;
    *nVOutH = height;
    *len = width * height * 3 / 2;
    if (bufsize < *len) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
        "buf size(%d) < frame size(%d)", bufsize, *len);
      return -1;
    }
    ISP_AE_PARAM_S stAeParam;
    stAeParam.GainOpType = static_cast<ISP_OP_TYPE_E>(0);
    stAeParam.IntegrationOpType = static_cast<ISP_OP_TYPE_E>(0);
    ret = HB_ISP_GetAeParam(vin_info_.pipe_id, &stAeParam);
    if (ret == 0) {
      timestamp += stAeParam.u32IntegrationTime / 2 * 1e3;
    }
    if (stride == width) {
      memcpy(frame_buf, vOut.img_addr.addr[0], width * height);
      memcpy(frame_buf + width * height,
             vOut.img_addr.addr[1],
             width * height / 2);
    } else {
      // jump over stride - width Y
      for (i = 0; i < height; i++) {
        memcpy(
            frame_buf + i * width, vOut.img_addr.addr[0] + i * stride, width);
      }
      // jump over stride - width UV
      for (i = 0; i < height / 2; i++) {
        memcpy(frame_buf + width * height + i * width,
               vOut.img_addr.addr[1] + i * stride,
               width);
      }
    }
    HB_VPS_ReleaseChnFrame(
        vps_infos_.m_vps_info[0].m_vps_grp_id,
        nChnID,
        &vOut);
    break;
  } while (1);
  return 0;
}

int HobotMipiCapIml::parseConfig(std::string sensor_name,
                                   int w, int h, int fps) {
  int ret = 0;
  if ((sensor_name == "IMX415") || (sensor_name == "imx415")) {
    imx415_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "F37") || (sensor_name == "f37")) {
    f37_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "GC4663") || (sensor_name == "gc4663")) {
    gc4663_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "IMX586") || (sensor_name == "imx586")) {
    // enable, sdb3.0
    std::vector<std::string> sys_cmds{
        "echo 111 > /sys/class/gpio/export",
        "echo out > /sys/class/gpio/gpio111/direction",
        "echo 1 > /sys/class/gpio/gpio111/value",
    };
    for (const auto& sys_cmd : sys_cmds) {
      system(sys_cmd.data());
    }
    imx586_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "GC4C33") || (sensor_name == "gc4c33")) {
    gc4c33_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "IMX219") || (sensor_name == "imx219")) {
    imx219_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "IMX477") || (sensor_name == "imx477")) {
    imx477_linear_vin_param_init(&vin_info_);
  } else if ((sensor_name == "OV5647") || (sensor_name == "ov5647")) {
    ov5647_linear_vin_param_init(&vin_info_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->sensor name not found(%s).\n", __func__, sensor_name.c_str());
    // m_oX3UsbCam.m_infos.m_vin_enable = 0;
    return -1;
  }
  auto sensor_bus = getSensorBus(sensor_name);
  if (sensor_bus != 0xff)
    vin_info_.snsinfo.sensorInfo.bus_num = sensor_bus;

    // 减少ddr带宽使用量
  vin_info_.vin_vps_mode = VIN_ONLINE_VPS_OFFLINE;
  // 2. 根据vin中的分辨率配置vps
  int width = vin_info_.mipi_attr.mipi_host_cfg.width;
  int height = vin_info_.mipi_attr.mipi_host_cfg.height;
  int mipi_fps = vin_info_.mipi_attr.mipi_host_cfg.fps;
  // vin_info_.vin_vps_mode = VIN_ONLINE_VPS_ONLINE;
  vps_enable_ = 1;
  vps_infos_.m_group_num = 1;
  // 配置group的输入
  ret |= vps_grp_param_init(
      &vps_infos_.m_vps_info[0], width, height);
  // 以下是配置group的每一个通道的参数
  vps_infos_.m_vps_info[0].m_chn_num = 1;
  ret |= vps_chn_param_init(
      &vps_infos_.m_vps_info[0].m_vps_chn_attrs[0],
      2, w, h, fps);
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "[%s]-> w:h=%d:%d ,fps=%d sucess.\n", __func__, w, h, fps);
}

bool HobotMipiCapIml::checkPipelineOpened(int pipeline_idx) {
  std::string cfg_info;
  // 若mipi camera已打开，sif_info返回“pipeid”
  std::ifstream sif_info("/sys/devices/platform/soc/a4001000.sif/cfg_info");
  if (sif_info.is_open()) {
    while (getline(sif_info, cfg_info)) {
      std::string pipe_line_info =
            "pipe " + std::to_string(pipeline_idx) + " not inited";
      if (!cfg_info.compare(pipe_line_info)) {
        RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "mipi camera the pipeline %d is idle.\n", pipeline_idx);
        return false;
      }
    }
  }
  return true;
}

int HobotMipiCapIml::getCapInfo(MIPI_CAP_INFO_ST &info) {
  info = cap_info_;
}

int HobotMipiCapIml::getSensorBus(std::string &sensor_name) {
  return 0xff;
}

void HobotMipiCapIml::listMipiHost(std::vector<int> &mipi_hosts, 
    std::vector<int> &started, std::vector<int> &stoped) {
  std::vector<int> host;
  std::string board_type_str = "";
  for (int num : mipi_hosts) {
    std::string mipi_host = "/sys/class/vps/mipi_host" + std::to_string(num) + "/status/cfg";
    std::ifstream mipi_host_fd(mipi_host);
    board_type_str = "";
    if (mipi_host_fd.is_open()) {
      std::getline(mipi_host_fd, board_type_str);
      if (board_type_str == "not inited") {
        stoped.push_back(num);
      } else {
        started.push_back(num);
      }
      mipi_host_fd.close();
    }
  }
}

bool HobotMipiCapIml::detectSensor(SENSOR_ID_T &sensor_info) {
  char cmd[256];
  char result[1024];
  memset(cmd, '\0', sizeof(cmd));
  memset(result, '\0', sizeof(result));
  if (sensor_info.i2c_addr_width == I2C_ADDR_8) {
    sprintf(cmd, "i2ctransfer -y -f %d w1@0x%x 0x%x r1 2>&1",
            sensor_info.i2c_bus,
            sensor_info.i2c_dev_addr,
            sensor_info.det_reg);
  } else if (sensor_info.i2c_addr_width == I2C_ADDR_16) {
    sprintf(cmd,
            "i2ctransfer -y -f %d w2@0x%x 0x%x 0x%x r1 2>&1",
            sensor_info.i2c_bus,
            sensor_info.i2c_dev_addr,
            sensor_info.det_reg >> 8,
            sensor_info.det_reg & 0xFF);
  } else {
    return false;
  }
  exec_cmd_ex(cmd, result, sizeof(result));
  if (strstr(result, "Error") == NULL && strstr(result, "error") == NULL) {
    // 返回结果中不带Error, 说明sensor找到了
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
          "match sensor:%s\n", sensor_info.sensor_name);
    return true;
  }
  return false;
}



int HobotMipiCapImlRDKX3::initEnv() {
  std::vector<int> mipi_hosts = {0};
  std::vector<int> mipi_started;
  std::vector<int> mipi_stoped;
  listMipiHost(mipi_hosts, mipi_started, mipi_stoped);
  if (mipi_stoped.size() == 0) {
    return -1;
  }
  std::vector<std::string> sys_cmds;
  for (int num : mipi_stoped) {
    switch (num) {
      case 0:
        sys_cmds.push_back("echo 19 > /sys/class/gpio/export");
        sys_cmds.push_back("echo out > /sys/class/gpio/gpio19/direction");
        sys_cmds.push_back("echo 0 > /sys/class/gpio/gpio19/value");
        sys_cmds.push_back("sleep 0.2");
        sys_cmds.push_back("echo 1 > /sys/class/gpio/gpio19/value");
        sys_cmds.push_back("echo 19 > /sys/class/gpio/unexport");
        sys_cmds.push_back("echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq");
        sys_cmds.push_back("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
        break;
      default:
        break;
    }
  }
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }
  return 0;
}

std::vector<std::string> HobotMipiCapImlRDKX3::listSensor() {

  // mipi sensor的信息数组
  SENSOR_ID_T sensor_id_list[] = {
    {2, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {1, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {1, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {2, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {2, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {3, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {2, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {3, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {3, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
    {1, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
  };
  std::vector<std::string> device;

  /* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功的 */
  HB_MIPI_EnableSensorClock(0);
  HB_MIPI_EnableSensorClock(1);
  // HB_MIPI_EnableSensorClock(2); // 需要修改内核dts使能mipihost2的mclk

  for (auto sensor_id : sensor_id_list) {
    if (detectSensor(sensor_id)) {
      device.push_back(sensor_id.sensor_name);
    }
  }
  return device;
}

int HobotMipiCapImlRDKX3::getSensorBus(std::string &sensor_name) {
  int ret = 0xff;
  if ((sensor_name == "IMX415") || (sensor_name == "imx415")) {
    ret = 1;
  } else if ((sensor_name == "F37") || (sensor_name == "f37")) {
     ret = 1;
  } else if ((sensor_name == "GC4663") || (sensor_name == "gc4663")) {
     ret = 1;
  } else if ((sensor_name == "IMX586") || (sensor_name == "imx586")) {
     ret = 1;
  } else if ((sensor_name == "GC4C33") || (sensor_name == "gc4c33")) {
     ret = 1;
  } else if ((sensor_name == "IMX219") || (sensor_name == "imx219")) {
     ret = 1;
  } else if ((sensor_name == "IMX477") || (sensor_name == "imx477")) {
     ret = 1;
  } else if ((sensor_name == "OV5647") || (sensor_name == "ov5647")) {
     ret = 1;
  }
  return ret;
}




int HobotMipiCapImlRDKX3_m::initEnv() {
  std::vector<int> mipi_hosts = {0,1,2,3};
  std::vector<int> mipi_started;
  std::vector<int> mipi_stoped;
  listMipiHost(mipi_hosts, mipi_started, mipi_stoped);
  if (mipi_stoped.size() == 0) {
    return -1;
  }
  std::vector<std::string> sys_cmds;
  for (int num : mipi_stoped) {
    switch (num) {
      case 0:
        sys_cmds.push_back("echo 19 > /sys/class/gpio/export");
        sys_cmds.push_back("echo out > /sys/class/gpio/gpio19/direction");
        sys_cmds.push_back("echo 0 > /sys/class/gpio/gpio19/value");
        sys_cmds.push_back("sleep 0.2");
        sys_cmds.push_back("echo 1 > /sys/class/gpio/gpio19/value");
        sys_cmds.push_back("echo 19 > /sys/class/gpio/unexport");
        sys_cmds.push_back("echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq");
        sys_cmds.push_back("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
        break;
      default:
        break;
    }
  }
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }
  return 0;
}

std::vector<std::string> HobotMipiCapImlRDKX3_m::listSensor() {

  // mipi sensor的信息数组
  SENSOR_ID_T sensor_id_list[] = {
    {2, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {1, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {2, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {2, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {2, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {3, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {2, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {3, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {3, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {2, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
    {1, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
  };
  std::vector<std::string> device;

  /* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功的 */
  HB_MIPI_EnableSensorClock(0);
  HB_MIPI_EnableSensorClock(1);
  // HB_MIPI_EnableSensorClock(2); // 需要修改内核dts使能mipihost2的mclk

  for (auto sensor_id : sensor_id_list) {
    if (detectSensor(sensor_id)) {
      device.push_back(sensor_id.sensor_name);
    }
  }
  return device;
}

int HobotMipiCapImlRDKX3_m::getSensorBus(std::string &sensor_name) {
  int ret = 0xff;
  if ((sensor_name == "IMX415") || (sensor_name == "imx415")) {
    ret = 1;
  } else if ((sensor_name == "F37") || (sensor_name == "f37")) {
     ret = 1;
  } else if ((sensor_name == "GC4663") || (sensor_name == "gc4663")) {
     ret = 1;
  } else if ((sensor_name == "IMX586") || (sensor_name == "imx586")) {
     ret = 1;
  } else if ((sensor_name == "GC4C33") || (sensor_name == "gc4c33")) {
     ret = 1;
  } else if ((sensor_name == "IMX219") || (sensor_name == "imx219")) {
     ret = 1;
  } else if ((sensor_name == "IMX477") || (sensor_name == "imx477")) {
     ret = 1;
  } else if ((sensor_name == "OV5647") || (sensor_name == "ov5647")) {
     ret = 1;
  }
  return ret;
}


int HobotMipiCapImlSDB::initEnv() {
  std::vector<int> mipi_hosts = {0,1,2,3};
  std::vector<int> mipi_started;
  std::vector<int> mipi_stoped;
  listMipiHost(mipi_hosts, mipi_started, mipi_stoped);
  if (mipi_stoped.size() == 0) {
    return -1;
  }
  std::vector<std::string> sys_cmds;
  for (int num : mipi_stoped) {
    switch (num) {
      case 0:
        sys_cmds.push_back("echo 19 > /sys/class/gpio/export");
        sys_cmds.push_back("echo out > /sys/class/gpio/gpio19/direction");
        sys_cmds.push_back("echo 0 > /sys/class/gpio/gpio19/value");
        sys_cmds.push_back("sleep 0.2");
        sys_cmds.push_back("echo 1 > /sys/class/gpio/gpio19/value");
        sys_cmds.push_back("echo 19 > /sys/class/gpio/unexport");
        sys_cmds.push_back("echo 24000000 > /sys/class/vps/mipi_host0/param/snrclk_freq");
        sys_cmds.push_back("echo 1 > /sys/class/vps/mipi_host0/param/stop_check_instart");
        break;
      default:
        break;
    }
  }
  for (const auto& sys_cmd : sys_cmds) {
    system(sys_cmd.data());
  }
  return 0;
}

std::vector<std::string> HobotMipiCapImlSDB::listSensor() {

  // mipi sensor的信息数组
  SENSOR_ID_T sensor_id_list[] = {
    {2, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {1, 0x40, I2C_ADDR_8, 0x0B, "F37"},        // F37
    {2, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx415"},  // imx415
    {1, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {2, 0x29, I2C_ADDR_16, 0x03f0, "GC4663"},  // GC4663
    {1, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {2, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {3, 0x10, I2C_ADDR_16, 0x0000, "imx219"},  // imx219 for x3-pi
    {1, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {2, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {3, 0x1a, I2C_ADDR_16, 0x0200, "imx477"},  // imx477 for x3-pi
    {1, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {3, 0x36, I2C_ADDR_16, 0x300A, "ov5647"},  // ov5647 for x3-pi
    {2, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {1, 0x1a, I2C_ADDR_16, 0x0000, "imx586"},  // imx586
    {2, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
    {1, 0x29, I2C_ADDR_16, 0x0000, "gc4c33"},  // gc4c33
  };
  std::vector<std::string> device;

  /* sdb 生态开发板  ，使能sensor       mclk, 否则i2c 通信不会成功的 */
  HB_MIPI_EnableSensorClock(0);
  HB_MIPI_EnableSensorClock(1);
  // HB_MIPI_EnableSensorClock(2); // 需要修改内核dts使能mipihost2的mclk

  for (auto sensor_id : sensor_id_list) {
    if (detectSensor(sensor_id)) {
      device.push_back(sensor_id.sensor_name);
    }
  }
  return device;
}

int HobotMipiCapImlSDB::getSensorBus(std::string &sensor_name) {
  int ret = 0xff;
  if ((sensor_name == "IMX415") || (sensor_name == "imx415")) {
    ret = 1;
  } else if ((sensor_name == "F37") || (sensor_name == "f37")) {
     ret = 1;
  } else if ((sensor_name == "GC4663") || (sensor_name == "gc4663")) {
     ret = 1;
  } else if ((sensor_name == "IMX586") || (sensor_name == "imx586")) {
     ret = 1;
  } else if ((sensor_name == "GC4C33") || (sensor_name == "gc4c33")) {
     ret = 1;
  } else if ((sensor_name == "IMX219") || (sensor_name == "imx219")) {
     ret = 1;
  } else if ((sensor_name == "IMX477") || (sensor_name == "imx477")) {
     ret = 1;
  } else if ((sensor_name == "OV5647") || (sensor_name == "ov5647")) {
     ret = 1;
  }
  return ret;
}


}  // namespace mipi_cam
