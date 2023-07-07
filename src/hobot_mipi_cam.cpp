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

#define __STDC_CONSTANT_MACROS
#include "hobot_mipi_comm.hpp"
#include "hobot_mipi_cam.hpp"
#include "hobot_mipi_factory.hpp"

#include "sensor_msgs/distortion_models.hpp"
#include <rclcpp/rclcpp.hpp>

#include <errno.h>
#include <malloc.h>
#include <unistd.h>

#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace mipi_cam {

// popen运行cmd，并获取cmd返回结果
int exec_cmd_ex(const char *cmd, char* res, int max) {
  if (cmd == NULL || res == NULL || max <= 0)
    return -1;
  FILE *pp = popen(cmd, "r");
  if (!pp) {
   RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
     "error, cannot popen cmd: %s\n", cmd);
    return -1;
  }
  int length;
  char tmp[1024] = {0};
  length = max;
  if (max > 1024)
    length = 1024;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "[%s]->cmd %s, fp=0x%x, len=%d.\n", __func__, cmd, pp, max);
  while (fgets(tmp, length, pp) != NULL) {
    // printf("exec_cmd_ex -- tmp:%s\n", tmp);
    sscanf(tmp, "%s", res);
  }
  pclose(pp);
  return strlen(res);
}

class MipiCamIml : public MipiCam {
 public:
  MipiCamIml();
  ~MipiCamIml();

  // 初始化摄像机
  // 输入参数：para是node传入的参数，包括sensor类型、名称，图像的宽、高等等。
  // 返回值：0，初始化成功，-1，初始化失败。
  int init(struct NodePara &para);

  // 反初始化摄像机；
  // 返回值：0，反初始化成功；-1，反初始化失败。
  int deInit();

  // 启动摄像机的码流；
  // 返回值：0，启动成功；-1，启动失败。
  int start();

  // 停止摄像机的码流；
  // 返回值：0，停止成功；-1，停止失败。
  int stop();

  // grabs a new image from the camera
  bool getImage(
    builtin_interfaces::msg::Time & stamp,
    std::string & encoding,
    uint32_t & height, uint32_t & width,
    uint32_t & step, std::vector<uint8_t> & data);

  // grabs a new hbmem's image hbmem from the camera
  bool getImageMem(
    builtin_interfaces::msg::Time & stamp,
    std::array<uint8_t, 12> & encoding,
    uint32_t & height, uint32_t & width, uint32_t & step,
    std::array<uint8_t, 6220800> & data, uint32_t & data_size);

  // gen camera calibration
  bool getCamCalibration(sensor_msgs::msg::CameraInfo& cam_info,
                           const std::string &file_path);

  bool isCapturing();

 private:
  bool lsInit_;
  bool is_capturing_;
  std::shared_ptr<HobotMipiCap> mipiCap_ptr_;
  struct NodePara nodePare_;
};

std::shared_ptr<MipiCam> MipiCam::create_mipicam() {
  return std::make_shared<MipiCamIml>();
}

MipiCamIml::MipiCamIml()
    : lsInit_(false),
      is_capturing_(false) {
}

MipiCamIml::~MipiCamIml() {
  stop();
  deInit();
}



int MipiCamIml::init(struct NodePara &para) {
  if (lsInit_) {
    return 0;
  }
  memcpy(&nodePare_, &para, sizeof(nodePare_));
  auto board_type = getBoardType();

  mipiCap_ptr_ = createMipiCap(board_type);
  if (!mipiCap_ptr_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->cap %s create capture failture.\r\n",
      __func__, board_type);
    return -1;
  }
  MIPI_CAP_INFO_ST cap_info;
  cap_info.config_path = nodePare_.config_path_;
  cap_info.sensor_type = nodePare_.video_device_name_;
  cap_info.width = nodePare_.image_width_;
  cap_info.height = nodePare_.image_height_;
  cap_info.fps = nodePare_.framerate_;

  mipiCap_ptr_->initEnv(nodePare_.video_device_name_);
  if (nodePare_.video_device_name_.length() == 0) {
    bool detect_device = false;
    auto mipicap_v = mipiCap_ptr_->listSensor();
    if (mipicap_v.size() <= 0) {
      RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "[%s] No camera detected!"
          " Please check if camera is connected.\r\n",
          __func__);
        return -2;
    }
    cap_info.sensor_type = mipicap_v[0];
  }
  if (mipiCap_ptr_->init(cap_info) != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->cap capture init failture.\r\n", __func__);
    return -5;
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "[%s]->cap %s init success.\r\n", __func__, nodePare_.video_device_name_);
  lsInit_ = true;
  return 0;
}

int MipiCamIml::deInit() {
  if (isCapturing()) {
    stop();
  }
  auto ret = mipiCap_ptr_->deInit();
  lsInit_ = false;
  return ret;
}

int MipiCamIml::start() {
  if (!lsInit_ || is_capturing_) {
    return -1;
  }
  int ret = 0;
  if (mipiCap_ptr_->start()) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s]->cap capture start failture.\r\n", __func__);
  }

  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
              "[%s]->w:h=%d:%d.\n",
              __func__,
              nodePare_.image_width_,
              nodePare_.image_height_);
  is_capturing_ = true;
  return ret;
}

int MipiCamIml::stop() {
  int ret = 0;
  if (isCapturing()) {
    ret = mipiCap_ptr_->stop();
  }
  is_capturing_ = false;
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
    "mipi_cam is stoped");
  return ret;
}

bool MipiCamIml::isCapturing() { return is_capturing_; }

bool MipiCamIml::getImage(builtin_interfaces::msg::Time &stamp,
                        std::string &encoding,
                        uint32_t &height,
                        uint32_t &width,
                        uint32_t &step,
                        std::vector<uint8_t> &data) {
  if (!is_capturing_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s][%-%d] Camera isn't captureing", __FILE__, __func__, __LINE__);
    return false;
  }
  if ((nodePare_.image_width_ == 0) || (nodePare_.image_height_ == 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Invalid publish width:%d height: %d! Please check the image_width "
      "and image_height parameters!",
      nodePare_.image_width_,
      nodePare_.image_height_);
    return false;
  }
  struct timespec time_start = {0, 0};
  int64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }

  int data_size = nodePare_.image_width_ * nodePare_.image_height_ * 1.5;

  data.resize(data_size);  // step * height);

  if (mipiCap_ptr_->getFrame(
          2,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(&data[0]),
          data_size,
          reinterpret_cast<unsigned int *>(&data_size)))
    return false;
  encoding = "nv12";
  clock_gettime(CLOCK_REALTIME, &time_start);
  stamp.sec = time_start.tv_sec;
  stamp.nanosec = time_start.tv_nsec;
  step = width;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
             "[%s]->enc=%s,step=%d, w:h=%d:%d,sz=%d,start %ld->laps=%ld ms.\n",
              __func__,
              encoding.c_str(),
              step,
              width,
              height,
              data_size,
              msStart,
              msEnd - msStart);
  return true;
}

bool MipiCamIml::getImageMem(
    builtin_interfaces::msg::Time &stamp,
    std::array<uint8_t, 12> &encoding,
    uint32_t &height,
    uint32_t &width,
    uint32_t &step,
    std::array<uint8_t, 6220800> &data,
    uint32_t &data_size) {
  if (!is_capturing_) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "[%s][%-%d] Camera isn't captureing", __FILE__, __func__, __LINE__);
    return false;
  }
  if ((nodePare_.image_width_ == 0) || (nodePare_.image_height_ == 0)) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Invalid publish width:%d height: %d! Please check the image_width "
      "and image_height parameters!",
      nodePare_.image_width_,
      nodePare_.image_height_);
    return false;
  }
  // get the image
  struct timespec time_start = {0, 0};
  uint64_t msStart = 0, msEnd = 0;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msStart = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  if (mipiCap_ptr_->getFrame(
          2,
          reinterpret_cast<int *>(&width),
          reinterpret_cast<int *>(&height),
          reinterpret_cast<void *>(data.data()),
          6220800,
          reinterpret_cast<unsigned int *>(&data_size)))
    return false;
  memcpy(encoding.data(), "nv12", strlen("nv12"));
  clock_gettime(CLOCK_REALTIME, &time_start);
  stamp.sec = time_start.tv_sec;
  stamp.nanosec = time_start.tv_nsec;
  step = width;
  {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    msEnd = (ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
  }
  RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
             "[%s]->hbmem enc=%s,step=%d,sz=%d,start %ld->laps=%ld ms.\n",
              __func__,
              encoding.data(),
              step,
              data_size,
              msStart,
              msEnd - msStart);
  return true;
}

bool MipiCamIml::getCamCalibration(sensor_msgs::msg::CameraInfo &cam_info,
                                  const std::string &file_path) {
  try {
    std::string camera_name;
    std::ifstream fin(file_path.c_str());
    if (!fin) {
     RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
          "Camera calibration file: %s not exist! Please make sure the "
          "calibration file path is correct and the calibration file exists!",
          file_path.c_str());
      return false;
    }
    YAML::Node calibration_doc = YAML::Load(fin);
    if (calibration_doc["camera_name"]) {
      camera_name = calibration_doc["camera_name"].as<std::string>();
    } else {
      camera_name = "unknown";
    }
    cam_info.width = calibration_doc["image_width"].as<int>();
    cam_info.height = calibration_doc["image_height"].as<int>();

    const YAML::Node &camera_matrix = calibration_doc["camera_matrix"];
    const YAML::Node &camera_matrix_data = camera_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.k[i] = camera_matrix_data[i].as<double>();
    }
    const YAML::Node &rectification_matrix =
        calibration_doc["rectification_matrix"];
    const YAML::Node &rectification_matrix_data = rectification_matrix["data"];
    for (int i = 0; i < 9; i++) {
      cam_info.r[i] = rectification_matrix_data[i].as<double>();
    }
    const YAML::Node &projection_matrix = calibration_doc["projection_matrix"];
    const YAML::Node &projection_matrix_data = projection_matrix["data"];
    for (int i = 0; i < 12; i++) {
      cam_info.p[i] = projection_matrix_data[i].as<double>();
    }

    if (calibration_doc["distortion_model"]) {
      cam_info.distortion_model =
          calibration_doc["distortion_model"].as<std::string>();
    } else {
      cam_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
      RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
                 "Camera calibration file did not specify distortion model, "
                  "assuming plumb bob");
    }
    const YAML::Node &distortion_coefficients =
        calibration_doc["distortion_coefficients"];
    int d_rows, d_cols;
    d_rows = distortion_coefficients["rows"].as<int>();
    d_cols = distortion_coefficients["cols"].as<int>();
    const YAML::Node &distortion_coefficients_data =
        distortion_coefficients["data"];
    cam_info.d.resize(d_rows * d_cols);
    for (int i = 0; i < d_rows * d_cols; ++i) {
      cam_info.d[i] = distortion_coefficients_data[i].as<double>();
    }
    RCLCPP_INFO(rclcpp::get_logger("mipi_cam"),
      "[getCamCalibration]->parse calibration file successfully");
    return true;
  } catch (YAML::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("mipi_cam"),
      "Unable to parse camera calibration file normally:%s",
      e.what());
    return false;
  }
}

}  // namespace mipi_cam
