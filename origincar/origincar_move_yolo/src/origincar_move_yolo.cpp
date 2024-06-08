/***********************************************************************
Copyright (c) 2022, www.guyuehome.com

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/


#include "origincar_move_yolo/origincar_move_yolo.h"
const float LINE_REAL_MIDX = 306.0;
const float STD_LINEAR_X = 0.30;
const double IMAGE_WIDTH = 640.0;
const double IMAGE_HEIGHT = 480.0;

const int _1L_OBS_TIME = 40;
const int _1R_OBS_TIME = 45;
const int _2L_OBS_TIME = 60;
const int _2R_OBS_TIME = 45;
const int _1L_START_INDEX = 15;   
const int _1R_START_INDEX = 15;  
const int _2L_START_INDEX = 35;  
const int _2R_START_INDEX = 15; 
const int _1L_BACK_START_INDEX = 0;   
const int _1R_BACK_START_INDEX = 23;   
const int _2L_BACK_START_INDEX = 1;   
const int _2R_BACK_START_INDEX = 1;   
const float _1L_START_ANGULAR_Z = 1.3;
const float _1R_START_ANGULAR_Z = 1.8;
const float _2L_START_ANGULAR_Z = 1.0;
const float _2R_START_ANGULAR_Z = 1.5;
const float _1L_dz = _1L_START_ANGULAR_Z / _1L_OBS_TIME;
const float _1R_dz = _1R_START_ANGULAR_Z / _1R_OBS_TIME - 0.01;
const float _2L_dz = _2L_START_ANGULAR_Z / _2L_OBS_TIME;
const float _2R_dz = _2R_START_ANGULAR_Z / _2R_OBS_TIME;

std::vector<float> _1L_angularz;
std::vector<float> _1R_angularz;
std::vector<float> _2L_angularz;
std::vector<float> _2R_angularz;
int _1L_angularz_index = _1L_START_INDEX - 2;
int _1R_angularz_index = _1R_START_INDEX - 2;
int _2L_angularz_index = _2L_START_INDEX - 2;
int _2R_angularz_index = _2R_START_INDEX - 2;

/* 设置全局变量用于状态控制 */
bool is_exe_obs = false;
bool is_second = false;    
bool is_left = false;
bool is_goback = false;
bool is_slowdown = false;


/* 设置全局变量用于巡线 */
float line_x = 0, line_y = 0;
float deceleration_rate1 = 0.003;     // 每步减少的速度
float deceleration_rate2 = 0.010;     // 每步减小的速度
float now_linear_x = 0.30;
float target_x = 0;//障碍物中点
float closest_line_x = 0;//距离障碍物最近的line
int PostProcess_cnt = 80;
auto cmd_vel_message = geometry_msgs::msg::Twist();

using hobot::dnn_node::DNNTensor;
namespace hobot {
namespace dnn_node {
namespace playfootball_node {
// 算法输出解析参数
struct PTQYolo5Config {
  std::vector<int> strides;
  std::vector<std::vector<std::pair<double, double>>> anchors_table;
  int class_num;
  std::vector<std::string> class_names;
};

float score_threshold_ = 0.6;
float nms_threshold_ = 0.5;
int nms_top_k_ = 5000;

PTQYolo5Config yolo5_config_ = {
    {8, 16, 32},
    {{{10, 13}, {16, 30}, {33, 23}},
    {{30, 61}, {62, 45}, {59, 119}},
    {{116, 90}, {156, 198}, {373, 326}}},
    2,
    {"hat", "line"}
};

void ParseTensor(std::shared_ptr<DNNTensor> tensor,
                 int layer,
                 std::vector<YoloV5Result> &results) {
  hbSysFlushMem(&(tensor->sysMem[0]), HB_SYS_MEM_CACHE_INVALIDATE);
  int num_classes = yolo5_config_.class_num;
  int stride = yolo5_config_.strides[layer];
  int num_pred = yolo5_config_.class_num + 4 + 1;

  std::vector<float> class_pred(yolo5_config_.class_num, 0.0);
  std::vector<std::pair<double, double>> &anchors = yolo5_config_.anchors_table[layer];

  //  int *shape = tensor->data_shape.d;
  int height, width;
  auto ret = get_tensor_hw(tensor, &height, &width);
  if (ret != 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                 "get_tensor_hw failed");
  }

  int anchor_num = anchors.size();
  auto *data = reinterpret_cast<float *>(tensor->sysMem[0].virAddr);
  for (int h = 0; h < height; h++) {
    for (int w = 0; w < width; w++) {
      for (int k = 0; k < anchor_num; k++) {
        double anchor_x = anchors[k].first;
        double anchor_y = anchors[k].second;
        float *cur_data = data + k * num_pred;
        float objness = cur_data[4];

        int id = argmax(cur_data + 5, cur_data + 5 + num_classes);
        double x1 = 1 / (1 + std::exp(-objness)) * 1;
        double x2 = 1 / (1 + std::exp(-cur_data[id + 5]));
        double confidence = x1 * x2;

        if (confidence < score_threshold_) {
          continue;
        }

        float center_x = cur_data[0];
        float center_y = cur_data[1];
        float scale_x = cur_data[2];
        float scale_y = cur_data[3];

        double box_center_x =
            ((1.0 / (1.0 + std::exp(-center_x))) * 2 - 0.5 + w) * stride;
        double box_center_y =
            ((1.0 / (1.0 + std::exp(-center_y))) * 2 - 0.5 + h) * stride;

        double box_scale_x =
            std::pow((1.0 / (1.0 + std::exp(-scale_x))) * 2, 2) * anchor_x;
        double box_scale_y =
            std::pow((1.0 / (1.0 + std::exp(-scale_y))) * 2, 2) * anchor_y;

        double xmin = (box_center_x - box_scale_x / 2.0);
        double ymin = (box_center_y - box_scale_y / 2.0);
        double xmax = (box_center_x + box_scale_x / 2.0);
        double ymax = (box_center_y + box_scale_y / 2.0);

        if (xmax <= 0 || ymax <= 0) {
          continue;
        }

        if (xmin > xmax || ymin > ymax) {
          continue;
        }

        results.emplace_back(
            YoloV5Result(static_cast<int>(id),
                         xmin,
                         ymin,
                         xmax,
                         ymax,
                         confidence,
                         yolo5_config_.class_names[static_cast<int>(id)]));
      }
      data = data + num_pred * anchors.size();
    }
  }
}

void yolo5_nms(std::vector<YoloV5Result> &input,
               float iou_threshold,
               int top_k,
               std::vector<std::shared_ptr<YoloV5Result>> &result,
               bool suppress) {
  // sort order by score desc
  std::stable_sort(input.begin(), input.end(), std::greater<YoloV5Result>());

  std::vector<bool> skip(input.size(), false);

  // pre-calculate boxes area
  std::vector<float> areas;
  areas.reserve(input.size());
  for (size_t i = 0; i < input.size(); i++) {
    float width = input[i].xmax - input[i].xmin;
    float height = input[i].ymax - input[i].ymin;
    float area = width * height;
    areas.push_back(area);
    // RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Area %d: %f", i, area);
  }

  int count = 0;
  for (size_t i = 0; count < top_k && i < skip.size(); i++) {
    if (skip[i]) {
      continue;
    }
    skip[i] = true;
    ++count;

    for (size_t j = i + 1; j < skip.size(); ++j) {
      if (skip[j]) {
        continue;
      }
      if (suppress == false) {
        if (input[i].id != input[j].id) {
          continue;
        }
      }

      // intersection area
      float xx1 = std::max(input[i].xmin, input[j].xmin);
      float yy1 = std::max(input[i].ymin, input[j].ymin);
      float xx2 = std::min(input[i].xmax, input[j].xmax);
      float yy2 = std::min(input[i].ymax, input[j].ymax);

      if (xx2 > xx1 && yy2 > yy1) {
        float area_intersection = (xx2 - xx1) * (yy2 - yy1);
        float iou_ratio =
            area_intersection / (areas[j] + areas[i] - area_intersection);
        if (iou_ratio > iou_threshold) {
          skip[j] = true;
        }
      }
    }

    auto yolo_res = std::make_shared<YoloV5Result>(input[i].id,
                                                   input[i].xmin,
                                                   input[i].ymin,
                                                   input[i].xmax,
                                                   input[i].ymax,
                                                   input[i].score,
                                                   input[i].class_name);
    if (!yolo_res) {
      RCLCPP_ERROR(rclcpp::get_logger("Yolo5_detection_parser"),
                   "invalid yolo_res");
    }

    result.push_back(yolo_res);
  }
}

int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width) {
  int h_index = 0;
  int w_index = 0;
  if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NHWC) {
    h_index = 1;
    w_index = 2;
  } else if (tensor->properties.tensorLayout == HB_DNN_LAYOUT_NCHW) {
    h_index = 2;
    w_index = 3;
  } else {
    return -1;
  }
  *height = tensor->properties.validShape.dimensionSize[h_index];
  *width = tensor->properties.validShape.dimensionSize[w_index];
  return 0;
}

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::vector<std::shared_ptr<YoloV5Result>> &results) {
  std::vector<YoloV5Result> parse_results;
  for (size_t i = 0; i < node_output->output_tensors.size(); i++) {
    ParseTensor(
        node_output->output_tensors[i], static_cast<int>(i), parse_results);
  }

  yolo5_nms(parse_results, nms_threshold_, nms_top_k_, results, false);

  return 0;
}

} 
} 
}  


// 使用hobotcv resize nv12格式图片，固定图片宽高比
int ResizeNV12Img(const char* in_img_data,
                  const int& in_img_height,
                  const int& in_img_width,
                  const int& scaled_img_height,
                  const int& scaled_img_width,
                  cv::Mat& out_img,
                  float& ratio) {
  // RCLCPP_INFO(rclcpp::get_logger("ResizeNV12Img"), "Scaled Image Height: %d, Scaled Image Width: %d", scaled_img_height, scaled_img_width);
  cv::Mat src(
      in_img_height * 3 / 2, in_img_width, CV_8UC1, (void*)(in_img_data));
  float ratio_w =
      static_cast<float>(in_img_width) / static_cast<float>(scaled_img_width);
  float ratio_h =
      static_cast<float>(in_img_height) / static_cast<float>(scaled_img_height);
  float dst_ratio = std::max(ratio_w, ratio_h);
  int resized_width, resized_height;
  if (dst_ratio == ratio_w) {
    resized_width = scaled_img_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  } else if (dst_ratio == ratio_h) {
    resized_width = static_cast<float>(in_img_width) / dst_ratio;
    resized_height = scaled_img_height;
  }

  // hobot_cv要求输出宽度为16的倍数
  int remain = resized_width % 16;
  if (remain != 0) {
    //向下取16倍数，重新计算缩放系数
    resized_width -= remain;
    dst_ratio = static_cast<float>(in_img_width) / resized_width;
    resized_height = static_cast<float>(in_img_height) / dst_ratio;
  }
  //高度向下取偶数
  resized_height =
      resized_height % 2 == 0 ? resized_height : resized_height - 1;
  ratio = dst_ratio;

  return hobot_cv::hobotcv_resize(
      src, in_img_height, in_img_width, out_img, resized_height, resized_width);
}

struct Football_output : public hobot::dnn_node::DnnNodeOutput {
  // 缩放比例系数，原图和模型输入分辨率的比例。
  float ratio = 1.0;
};

void Football_node::restart_subscription() {
  if (!ros_img_subscription_)
  {
    ros_img_subscription_ = this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
        "hbmem_img",
        10,
        std::bind(&Football_node::FeedImg,
        this,
        std::placeholders::_1)); 
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Subscription restarted");
  }
}

void Football_node::stop_subscription(int choice) {
  if (ros_img_subscription_ && choice == 0)
  {
    ros_img_subscription_.reset();
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Subscription topic hbmem stopped");
  }
  else if (choice == 1)
  {
    subscriber_switch.reset();
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Subscription topic sign_switch stopped");
  }
  else if (choice == 2)
  {
    subscriber_return.reset();
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Subscription topic sign4return stopped");
  }
}

Football_node::Football_node(const std::string& node_name,
                             const rclcpp::NodeOptions& options)
    : hobot::dnn_node::DnnNode(node_name, options) {
  if (Init() != 0 ||
      GetModelInputSize(0, model_input_width_, model_input_height_) < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"), "Node init fail!");
    rclcpp::shutdown();
  }
  ros_img_subscription_ =
      this->create_subscription_hbmem<hbm_img_msgs::msg::HbmMsg1080P>(
          "/hbmem_img",
          10,
          std::bind(&Football_node::FeedImg, this, std::placeholders::_1));
  msg_publisher_ = 
    this->create_publisher<ai_msgs::msg::PerceptionTargets>("/playfootball_node", 10);
  cmd_vel_publisher_ =
    this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 5);

  subscriber_return = this->create_subscription<std_msgs::msg::Int32>(
    "/sign4return", 10,
    std::bind(&Football_node::subscription_callback_return, this, std::placeholders::_1)); 

  subscriber_switch = this->create_subscription<origincar_msg::msg::Sign>(
    "/sign_switch", 10,
    std::bind(&Football_node::subscription_callback_switch, this, std::placeholders::_1));
}

int Football_node::SetNodePara() {
  if (!dnn_node_para_ptr_) return -1;
  dnn_node_para_ptr_->model_file =
      "/root/dev_ws/src/origincar/origincar_move_yolo/config/play_football.bin";
  dnn_node_para_ptr_->model_task_type =
      hobot::dnn_node::ModelTaskType::ModelInferType;
  dnn_node_para_ptr_->task_num = 6;
  return 0;
}

void Football_node::subscription_callback_return(
  const std_msgs::msg::Int32::SharedPtr msg) {
  // 如果按下按钮"C区出口结束遥测"，会被调用，重启订阅者hbmem，关闭订阅者sign4return
  if (msg->data == 6) {
    restart_subscription(); // 重新打开订阅者hbmem
    stop_subscription(2);   // 关闭订阅者sign4return
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Topic sign4return did not receive msg.data equals 6!");
  }
}

void Football_node::subscription_callback_switch(
  const origincar_msg::msg::Sign::SharedPtr msg) {
  // 如果扫描到二维码，会被调用，关闭hbmem和sign_switch的订阅
  if (msg->sign_data == 3 || msg->sign_data == 4) {
    stop_subscription(0);    // 关闭订阅者hbmem_img
    stop_subscription(1);    // 关闭订阅者sign_switch
    cmd_vel_message.linear.x = 0.0, cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0, cmd_vel_message.angular.z = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_message);
    is_exe_obs = false, is_goback = false, is_slowdown = false, is_left = false, is_second = true;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Topic sign_switch did not receive msg equals 3 or 4!");
  }
}

void Football_node::FeedImg(
    const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr img_msg) {
  if (!rclcpp::ok() || !img_msg) {
    return;
  }

  // 1 对订阅到的图片消息进行验证
  if ("nv12" !=
      std::string(reinterpret_cast<const char*>(img_msg->encoding.data()))) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                 "Please use nv12 img encoding!.");
    return;
  }

  // 2 创建算法输出数据，填充消息头信息，用于推理完成后AI结果的发布
  auto dnn_output = std::make_shared<Football_output>();
  dnn_output->msg_header = std::make_shared<std_msgs::msg::Header>();
  dnn_output->msg_header->set__frame_id(std::to_string(img_msg->index));
  dnn_output->msg_header->set__stamp(img_msg->time_stamp);

  // 3 算法前处理，即创建算法输入数据
  std::shared_ptr<hobot::dnn_node::NV12PyramidInput> pyramid = nullptr;
  if (img_msg->height != static_cast<uint32_t>(model_input_height_) ||
      img_msg->width != static_cast<uint32_t>(model_input_width_)) {
    // 3.1 订阅到的图片和算法输入分辨率不一致，需要做resize处理
    cv::Mat out_img;
    if (ResizeNV12Img(reinterpret_cast<const char*>(img_msg->data.data()),
                      img_msg->height,
                      img_msg->width,
                      model_input_height_,
                      model_input_width_,
                      out_img,
                      dnn_output->ratio) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                   "Resize nv12 img fail!");
      return;
    }

    uint32_t out_img_width = out_img.cols;
    uint32_t out_img_height = out_img.rows * 2 / 3;
    // 3.2 根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(out_img.data),
        out_img_height,
        out_img_width,
        model_input_height_,
        model_input_width_);
  } else {
    // 3.3不需要进行resize，直接根据算法输入图片分辨率，使用hobot_dnn中提供的方法创建算法输入数据
    pyramid = hobot::dnn_node::ImageProc::GetNV12PyramidFromNV12Img(
        reinterpret_cast<const char*>(img_msg->data.data()),
        img_msg->height,
        img_msg->width,
        model_input_height_,
        model_input_width_);
  }
  // 3.4 校验算法输入数据
  if (!pyramid) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"), "Get pym fail");
    return;
  }
  // 3.5 将算法输入数据转成dnn node推理输入的格式
  auto inputs =
      std::vector<std::shared_ptr<hobot::dnn_node::DNNInput>>{pyramid};


  // 4 使用创建的算法输入和输出数据，以异步模式运行推理，推理结果通过PostProcess接口回调返回
  if (Run(inputs, dnn_output, nullptr, false) < 0) {
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Run predict fail!");
  }
}

int Football_node::PostProcess(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>& node_output) {
  if (!rclcpp::ok()) {
    return 0;
  }
  /* ====== !!!避障的逻辑控制!!! ======  */
  if(is_exe_obs){ // 进入避障状态
    cmd_vel_message.linear.x  = STD_LINEAR_X;
    // RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "temp_angularz_index: %d", temp_angularz_index);
    if(!is_second){ // 第一条巡线
      if(!is_goback) {// 避障的前一半状态
        if (is_left) { // 障碍物在左侧，先右转
          _1L_angularz_index += 1;
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_1L_angularz_index: %d", _1L_angularz_index);
          cmd_vel_message.angular.z = (float)(-1.0) * _1L_angularz[_1L_angularz_index];
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
          _1L_angularz.push_back(_1L_angularz[_1L_angularz_index] - _1L_dz);  
          if(_1L_angularz_index >= _1L_OBS_TIME) { // 右转时间到，开始返回
            is_goback = true;
            _1L_angularz_index = _1L_OBS_TIME;   // 数值从大到0，从0到大
          }
        } else {       // 障碍物在右侧，先左转
          _1R_angularz_index += 1;
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_1R_angularz_index: %d", _1R_angularz_index);
          cmd_vel_message.angular.z = (float)( 1.0) * _1R_angularz[_1R_angularz_index];
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
          _1R_angularz.push_back(_1R_angularz[_1R_angularz_index] - _1R_dz);  
          if(_1R_angularz_index >= _1R_OBS_TIME) { // 右转时间到，开始返回
            is_goback = true;
            _1R_angularz_index = _1R_OBS_TIME;   // 关键，控制反转时的起始数值（从大到0，从大到0）
          }
        }
      } else { // 避障的后一半状态
        if (is_left) { // 障碍物在左侧
          if(_1L_angularz_index <= _1L_BACK_START_INDEX) { // 执行完避障的后半段
            cmd_vel_message.angular.z = 0.0;
            is_exe_obs = false, is_goback = false, is_slowdown = false, is_left = false;
            _1L_angularz_index = _1L_START_INDEX - 2;
            _1L_angularz.clear();
            for(int i=0; i < _1L_START_INDEX; i++)
              _1L_angularz.push_back(_1L_START_ANGULAR_Z);
          } else { // 正在执行避障的后半段，返回时左转
            _1L_angularz_index -= 1;
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_1L_angularz_index: %d", _1L_angularz_index);
            cmd_vel_message.angular.z = (float)( 1.0) * (_1L_angularz[_1L_angularz_index] + 0.2);
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
          }
        } else {       // 障碍物在右侧
          if(_1R_angularz_index <= _1R_BACK_START_INDEX) { // 执行完避障的后半段
            cmd_vel_message.angular.z = 0.0;
            is_exe_obs = false, is_goback = false, is_slowdown = false, is_left = false;
            _1R_angularz_index = _1R_START_INDEX - 2;
            _1R_angularz.clear();
            for(int i=0; i < _1R_START_INDEX; i++)
              _1R_angularz.push_back(_1R_START_ANGULAR_Z);
          } else { // 正在执行避障的后半段，返回时右转
            _1R_angularz_index -= 1;
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_1R_angularz_index: %d", _1R_angularz_index);
            cmd_vel_message.angular.z = (float)(-1.0) * (std::max(_1R_angularz[_1R_angularz_index] - 0.6, 0.0));
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
          }
        }
      }
    } else { //第二条巡线过程中的避障，现在重点问题还是极限处左右不分
      if(!is_goback) {
        if (is_left) { // 障碍物在左侧
          _2L_angularz_index += 1;
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_2L_angularz_index: %d", _2L_angularz_index);
          cmd_vel_message.angular.z = (float)(-1.0) * _2L_angularz[_2L_angularz_index];
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
          _2L_angularz.push_back(_2L_angularz[_2L_angularz_index] - _2L_dz);  
          if(_2L_angularz_index >= _2L_OBS_TIME) { // 右转时间到，开始返回
            is_goback = true;
            _2L_angularz_index = _2L_BACK_START_INDEX;   // 关键，数值应该是从大到0，再从大到0
          }       
        } else {       // 障碍物在右侧
          _2R_angularz_index += 1;
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_2R_angularz_index: %d", _2R_angularz_index);          
          cmd_vel_message.angular.z = (float)( 1.0) * _2R_angularz[_2R_angularz_index];
          RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
          _2R_angularz.push_back(_2R_angularz[_2R_angularz_index] - _2R_dz);  
          if(_2R_angularz_index >= _2R_OBS_TIME) { // 右转时间到，开始返回
            is_goback = true;
            _2R_angularz_index = _2R_BACK_START_INDEX;   // 关键，控制反转时的起始数值（从大到0，从大到0）
          }
        }
      } else { // 避障的后一半状态
        if(is_left) { // 障碍物在左侧
          if(_2L_angularz_index >= _2L_OBS_TIME) { // 执行完避障的后半段
            cmd_vel_message.angular.z = 0.0;
            is_exe_obs = false, is_goback = false, is_slowdown = false, is_left = false;
            _2L_angularz_index = _2L_START_INDEX - 2;
            _2L_angularz.clear();
            for(int i=0; i < _2L_START_INDEX; i++)
              _2L_angularz.push_back(_2L_START_ANGULAR_Z);
          } else { // 正在执行避障的后半段，返回时左转
            if(_2L_angularz[_2L_angularz_index]){
              cmd_vel_message.angular.z = (float)( 1.0) * (_2L_angularz[_2L_angularz_index] + 0.5);
            } else {
              cmd_vel_message.angular.z = 0.35;
            }
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_2L_angularz_index: %d", _2L_angularz_index);
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
            _2L_angularz_index += 1;
          }
        } else {     // 障碍物在右侧
          if(_2R_angularz_index >= _2R_OBS_TIME) { // 执行完避障的后半段
            cmd_vel_message.angular.z = 0.0;
            is_exe_obs = false, is_goback = false, is_slowdown = false, is_left = false;
            _2R_angularz_index = _2R_START_INDEX - 2;
            _2R_angularz.clear();
            for(int i=0; i < _2R_START_INDEX; i++)
              _2R_angularz.push_back(_2R_START_ANGULAR_Z);
          } else { // 正在执行避障的后半段，返回时右转
            if(_2R_angularz[_2R_angularz_index]){
              cmd_vel_message.angular.z = (float)(-1.0) * _2R_angularz[_2R_angularz_index];
            } else {
              cmd_vel_message.angular.z = -0.05;
            }
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "_2R_angularz_index: %d", _2R_angularz_index);
            RCLCPP_WARN(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);            
            _2R_angularz_index += 1;
          }
        }
      }
    }
    RCLCPP_INFO(rclcpp::get_logger("playfootball_node_exe_obs"), "message.angular.z: %f", cmd_vel_message.angular.z);
    cmd_vel_message.linear.y = 0.0;
    cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0;
    cmd_vel_message.angular.y = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_message);
    return 0;
  }

  auto tp_start = std::chrono::system_clock::now();

  // 1 创建用于发布推理结果的ROS Msg
  ai_msgs::msg::PerceptionTargets::UniquePtr pub_data(new ai_msgs::msg::PerceptionTargets());

  int exist_result = 0;
  // 2 将推理输出对应图片的消息头填充到ROS Msg
  pub_data->set__header(*node_output->msg_header);

  // 3创建解析输出数据，输出YoloV5Result是自定义的算法输出数据类型，results的维度等于检测出来的目标数
  std::vector<std::shared_ptr<hobot::dnn_node::playfootball_node::YoloV5Result>> results;

  if (hobot::dnn_node::playfootball_node::Parse(node_output, results) < 0) { 
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                 "Parse node_output fail!");
    return -1;
  }

  for (auto& rect : results) {
    if (!rect) {
      exist_result=0;
      continue;
    }

    if (rect->xmin < 0) rect->xmin = 0;
    if (rect->ymin < 0) rect->ymin = 0;
    if (rect->xmax >= model_input_width_) {
      rect->xmax = model_input_width_ - 1;
    }
    if (rect->ymax >= model_input_height_) {
      rect->ymax = model_input_height_ - 1;
    }
    if(rect->score >= hobot::dnn_node::playfootball_node::score_threshold_) exist_result=1;

    /* 发布的是坐标转换前的rect的信息 */
    // std::stringstream ss;
    // ss << "roi rect: " << rect->xmin << " " << rect->ymin << " " << rect->xmax
    //    << " " << rect->ymax << ", roi type: " << rect->class_name
    //    << ", score:" << rect->score;
    // RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "%s", ss.str().c_str());

    /* 填充roi消息的相关参数，后续发布到ai_msgs中 */
    ai_msgs::msg::Roi roi;
    roi.rect.set__x_offset(rect->xmin);
    roi.rect.set__y_offset(rect->ymin);
    roi.rect.set__width(rect->xmax - rect->xmin);
    roi.rect.set__height(rect->ymax - rect->ymin);
    roi.set__confidence(rect->score);

    ai_msgs::msg::Target target;
    target.set__type(rect->class_name);
    target.rois.emplace_back(roi);
    pub_data->targets.emplace_back(std::move(target));
  }

  // 4 坐标映射后得到相对于输入图片分辨率的坐标
  auto sample_node_output =
      std::dynamic_pointer_cast<Football_output>(node_output);
  if (!sample_node_output) {
    RCLCPP_ERROR(rclcpp::get_logger("playfootball_node"),
                 "Cast dnn node output fail!");
    return -1;
  }
  if (sample_node_output->ratio != 1.0) {
    for (auto& target : pub_data->targets) {
      for (auto& roi : target.rois) {
        roi.rect.x_offset *= sample_node_output->ratio;
        roi.rect.y_offset *= sample_node_output->ratio;
        roi.rect.width *= sample_node_output->ratio;
        roi.rect.height *= sample_node_output->ratio;
        // std::stringstream ss;
        // ss << "raw roi rect: " << roi.rect.x_offset << " " <<roi.rect.y_offset
        //    << " " << roi.rect.width << " " << roi.rect.height;
        // RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "%s", ss.str().c_str());
        // target_x = roi.rect.x_offset + 0.5 * roi.rect.width;
      }
    }
  }

  // 5. 根据坐标转化后的结果分别计算hat和line的相关参数 ---PlanA，根据type将坐标转化后的结果，划分为results_hat和results_line
  std::vector<std::shared_ptr<hobot::dnn_node::playfootball_node::YoloV5Result>> results_hat;
  std::vector<std::shared_ptr<hobot::dnn_node::playfootball_node::YoloV5Result>> results_line;
  for (auto& target : pub_data->targets){
    if(target.type == "hat"){
      for (auto& roi : target.rois){
        auto temp_yolo_res = std::make_shared<hobot::dnn_node::playfootball_node::YoloV5Result>
        (
          0,
          roi.rect.x_offset,
          roi.rect.y_offset,
          roi.rect.x_offset + roi.rect.width,
          roi.rect.y_offset + roi.rect.height,
          roi.confidence,
          "hat"
        );
        results_hat.push_back(temp_yolo_res);
      }
    } else {
      for (auto& roi : target.rois){
        auto temp_yolo_res = std::make_shared<hobot::dnn_node::playfootball_node::YoloV5Result>
        (
          1,
          roi.rect.x_offset,
          roi.rect.y_offset,
          roi.rect.x_offset + roi.rect.width,
          roi.rect.y_offset + roi.rect.height,
          roi.confidence,
          "line"
        );
        results_line.push_back(temp_yolo_res);
      }
    }
  }

  std::vector<float> rect_area; // rect的面积
  float max_rect_area = 0.0, now_rect_area = 0.0, hat_bottom_y = 0.0; 
  for(auto& rect : results_hat){
    if (rect->xmax < 80 || rect->xmin > 560) // xmax小于150或者xmin大于540, ymin大于140，不再考虑
      continue;
    now_rect_area = (rect->xmax - rect->xmin) * (rect->ymax - rect->ymin);
    if (max_rect_area < now_rect_area){
      target_x = (rect->xmin + rect->xmax) / 2;
      hat_bottom_y = rect->ymax;
      max_rect_area = now_rect_area;
    }
  } 

  /* 巡线PlanA，进行范围的一定限制，然后取所有范围内line_x的平均值 */
  float sum_line_x = 0, dis_hatline_y = 1000.0;
  int cnt_line = 0;
  float max_line_y = 0.0;
  if(is_second){
    //第二条巡线过程中的巡线目标点的确定：取范围限制内的line平均值
    for (auto& rect : results_line){
      // if((rect->ymax > 240.0) && (rect->ymin < 420.0) && (rect->xmin > 50.0) && (rect->xmax < 600.0)) {

      // }
      line_y = (rect->ymin + rect->ymax) / 2;
      line_x = (rect->xmin + rect->xmax) / 2;
      if(fabs(hat_bottom_y - line_y)< dis_hatline_y){
        closest_line_x = line_x;
        dis_hatline_y = fabs(hat_bottom_y - line_y);
      }
      sum_line_x += line_x;
      ++cnt_line;
    }
    if(cnt_line != 0)
      line_x = sum_line_x / cnt_line; 
    else
      line_x = LINE_REAL_MIDX;
  } else {
    //第一条巡线过程中的巡线目标点的确定：取范围限制内的最近line点
    for (auto& rect : results_line){
      line_y = (rect->ymin + rect->ymax) / 2;
      if(fabs(hat_bottom_y - line_y)< dis_hatline_y){
        closest_line_x = (rect->xmin + rect->xmax) / 2;
        dis_hatline_y = fabs(hat_bottom_y - line_y);
      }
      if(rect->ymin > max_line_y && rect->ymin < 400){
        max_line_y = rect->ymin;
        line_x = (rect->xmin + rect->xmax) / 2;
      }
    }
  }
  

  /* 巡线PlanB，进行范围的一定限制，然后将范围内的line按照line_y进行排序，取倒数第二个line_x */


  //5. 根据坐标转化后的结果分别计算hat和line的相关参数 ---PlanB，不提前划分results_line和results_hat
  // std::vector<int> rect_area; // rect的面积
  // int max_rect_area = 0, now_rect_area = 0, hat_bottom_y = 0; 
  // float sum_line_x = 0, dis_hatline_y = 1000.0;
  // int cnt = 0;
  // for (auto& target : pub_data->targets){
  //   if(target.type == "hat"){         // 如果target的种类是hat，则计算所有hat的面积最大值
  //     for (auto& roi : target.rois){
  //       if (roi.rect.x_offset + roi.rect.width < 100 || roi.rect.x_offset > 500) // xmax小于100或者xmin大于500, ymin大于200，不再考虑,|| roi.rect.y_offset > 200
  //         continue;
  //       now_rect_area = roi.rect.width * roi.rect.height;
  //       if (max_rect_area < now_rect_area){
  //         target_x = roi.rect.x_offset + 0.5 * roi.rect.width;
  //         hat_bottom_y = roi.rect.y_offset + roi.rect.height;
  //         max_rect_area = now_rect_area;
  //       }
  //     }     
  //   }                   
  // }

  // for (auto& target : pub_data->targets){
  //   if(target.type == "line"){   // 如果target的种类是line，则将y坐标最大的line作为巡线line
  //     for(auto& roi : target.rois){
  //       if((roi.rect.y_offset < 420.0)&&(roi.rect.x_offset > 50.0)&&((roi.rect.x_offset + roi.rect.width) < 600.0)){
  //         line_y = roi.rect.y_offset + 0.5 * roi.rect.height;
  //         line_x = roi.rect.x_offset + 0.5 * roi.rect.width;
  //         if(fabs(hat_bottom_y - line_y) < dis_hatline_y){
  //           closest_line_x = line_x;
  //           dis_hatline_y = fabs(hat_bottom_y - line_y);
  //         }
  //         sum_line_x = sum_line_x + line_x;
  //         cnt++;
  //       }
  //     }
  //   }
  // }

  // if (cnt == 0){
  //   line_x = LINE_REAL_MIDX;
  // } else {
  //   line_x = sum_line_x / cnt;
  // }

  // 6. 判断是否执行避障，根据面积的比例 + 锥底部的y坐标和当前line的y坐标的差值 + target_x和line_x的相对位置
  double area_ratio = max_rect_area / (IMAGE_WIDTH * IMAGE_HEIGHT);
  // 如果target_x和line_x相距小于150，并且hat_bottom_y和line_y相距小于150，再进行避障
  //if (fabs(target_x - line_x) <= 250.0 && fabs(hat_bottom_y - line_y) <= 200.0){ 
  //if (fabs(target_x - closest_line_x) <= 250.0)
  RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "target_x: %f", target_x);
  RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "closest_line_x: %f", closest_line_x);
  if (fabs(target_x - closest_line_x) <= 250.0){    
    if ((area_ratio >= 0.10) && !is_second) {
      is_exe_obs = true;  // 
    } else if(is_second && ((area_ratio >= 0.13 && fabs(target_x - LINE_REAL_MIDX) < 50.0) || (area_ratio >= 0.16))) {
      RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "target_x: %f", target_x);
      RCLCPP_WARN(rclcpp::get_logger("playfootball_node"), "area_ratio: %f", area_ratio);
      is_exe_obs = true;  // 如果是第二阶段，适当增大面积比例，因为障碍物在两侧时面积会增大
    } else if(area_ratio >= 0.05) {
      is_slowdown = true; // 0.07<=面积占比<0.12，被视为提前减速的避障点，先减速
    } else {
      exist_result = 0, is_slowdown = false;
    }                 
  } else {
    exist_result = 0, is_slowdown = false;
  }
  RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "area_ratio: %2f", area_ratio);

  RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "Exist Result: %d, Target Point: %f", exist_result, target_x);

  if (is_exe_obs && (target_x <= closest_line_x)) { // 需要开始避障，并且target_x<=closest_line_x，则障碍物在左侧
    is_left = true;
  }
  // else{
  //   if(is_exe_obs && (target_x > closest_line_x)){
  //     BACK_LIMIT = START_INDEX+5;
  //   }
  // }

  // 7. 巡线运动控制
  RCLCPP_INFO(rclcpp::get_logger("playfootball_node"), "post coor x: %f    y:%f", line_x, line_y);
  if (is_slowdown) { // slowdown的控制逻辑
    cmd_vel_message.angular.z = -1.0 * (line_x - LINE_REAL_MIDX) / 300.0; // 将305视为中点，因为摄像头像素点偏差
    cmd_vel_message.linear.x = STD_LINEAR_X;
    cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
    cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
    cmd_vel_publisher_->publish(cmd_vel_message);
  } else {           // 没有slowdown的控制逻辑
    if (!is_second) {
      // // 计算线速度，匀减速运动，目前纯纯匀速运动，不需要打开此注释
      // if (PostProcess_cnt>=0){
      //   now_linear_x = now_linear_x - deceleration_rate1;
      // }
      // else{
      //   now_linear_x = now_linear_x - deceleration_rate2;
      // }
      // if (now_linear_x < 0.30) 
      //   now_linear_x = 0.30; // 确保速度不会低于下限
      // PostProcess_cnt -= 1;
      cmd_vel_message.angular.z = -1.0 * (line_x - LINE_REAL_MIDX) / 400.0;
      cmd_vel_message.linear.x = STD_LINEAR_X;
      cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
      cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_message);
    } else {
      // 线速度0.35匀速运动返回
      cmd_vel_message.angular.z = -1.0 * (line_x - LINE_REAL_MIDX) / 300.0;
      cmd_vel_message.linear.x = STD_LINEAR_X;
      cmd_vel_message.linear.y = 0.0, cmd_vel_message.linear.z = 0.0;
      cmd_vel_message.angular.x = 0.0, cmd_vel_message.angular.y = 0.0;
      cmd_vel_publisher_->publish(cmd_vel_message);
    }
  }

  // 8. 将算法推理输出帧率填充到ROS Msg
  if (node_output->rt_stat) {
    pub_data->set__fps(round(node_output->rt_stat->output_fps));
    // 如果算法推理统计有更新，输出算法输入和输出的帧率统计、推理耗时
    if (node_output->rt_stat->fps_updated) {
      auto tp_now = std::chrono::system_clock::now();
      auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(
                          tp_now - tp_start)
                          .count();
      RCLCPP_WARN(rclcpp::get_logger("playfootball_node"),
                  "input fps: %.2f, out fps: %.2f, infer time ms: %d, "
                  "post process time ms: %d",
                  node_output->rt_stat->input_fps,
                  node_output->rt_stat->output_fps,
                  node_output->rt_stat->infer_time_ms,
                  interval);
    }
  }

  // 9. 发布ROS Msg
  msg_publisher_->publish(std::move(pub_data));

  return 0;
}

int main(int argc, char** argv) {
  for(int i=0; i < _1L_START_INDEX; i++){
    _1L_angularz.push_back(_1L_START_ANGULAR_Z);
  }
  for(int i=0; i < _1R_START_INDEX; i++){
    _1R_angularz.push_back(_1R_START_ANGULAR_Z);
  }
  for(int i=0; i < _2L_START_INDEX; i++){
    _2L_angularz.push_back(_2L_START_ANGULAR_Z);
  }
  for(int i=0; i < _2R_START_INDEX; i++){
    _2R_angularz.push_back(_2R_START_ANGULAR_Z);
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Football_node>());
  rclcpp::shutdown();
  return 0;
}
