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
#include "dnn_node/dnn_node_data.h"
#include <rclcpp_action/rclcpp_action.hpp>
#include "ai_msgs/msg/perception_targets.hpp"
#include "dnn_node/dnn_node.h"
#include "dnn_node/util/image_proc.h"
#include "hbm_img_msgs/msg/hbm_msg1080_p.hpp"
#include "hobot_cv/hobotcv_imgproc.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "origincar_msg/msg/sign.hpp"

namespace hobot {
namespace dnn_node {
namespace playfootball_node {
// 定义算法输出数据类型
struct YoloV5Result {
  // 目标类别ID
  int id;
  // 目标检测框
  float xmin;
  float ymin;
  float xmax;
  float ymax;
  // 检测结果的置信度
  float score;
  // 目标类别
  std::string class_name;

  YoloV5Result(int id_,
               float xmin_,
               float ymin_,
               float xmax_,
               float ymax_,
               float score_,
               std::string class_name_)
      : id(id_),
        xmin(xmin_),
        ymin(ymin_),
        xmax(xmax_),
        ymax(ymax_),
        score(score_),
        class_name(class_name_) {}

  friend bool operator>(const YoloV5Result &lhs, const YoloV5Result &rhs) {
    return (lhs.score > rhs.score);
  }
};

int32_t Parse(
    const std::shared_ptr<hobot::dnn_node::DnnNodeOutput> &node_output,
    std::vector<std::shared_ptr<YoloV5Result>> &results);


void ParseTensor(std::shared_ptr<DNNTensor> tensor,
                 int layer,
                 std::vector<YoloV5Result> &results);

void yolo5_nms(std::vector<YoloV5Result> &input,
               float iou_threshold,
               int top_k,
               std::vector<std::shared_ptr<YoloV5Result>> &result,
               bool suppress);

int get_tensor_hw(std::shared_ptr<DNNTensor> tensor, int *height, int *width);

template <class ForwardIterator>
inline size_t argmax(ForwardIterator first, ForwardIterator last) {
  return std::distance(first, std::max_element(first, last));
}

}  // namespace playfootball_node
}  // namespace dnn_node
}  // namespace hobot


// 继承DnnNode虚基类，创建算法推理节点
class Football_node : public hobot::dnn_node::DnnNode {
 public:
  Football_node(const std::string& node_name = "playfootball_node",
                const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 protected:
  int SetNodePara() override;
  int PostProcess(const std::shared_ptr<hobot::dnn_node::DnnNodeOutput>&
                      node_output) override;

 private:
  // 算法输入图片数据的宽和高
  int model_input_width_ = -1;
  int model_input_height_ = -1;
  
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriber_return;          // sign4return的订阅者
  rclcpp::Subscription<origincar_msg::msg::Sign>::SharedPtr subscriber_switch;      // sign_switch的订阅者

  // 图片消息订阅者
  rclcpp::SubscriptionHbmem<hbm_img_msgs::msg::HbmMsg1080P>::ConstSharedPtr ros_img_subscription_ = nullptr;
  // 算法推理结果消息发布者
  rclcpp::Publisher<ai_msgs::msg::PerceptionTargets>::SharedPtr msg_publisher_ = nullptr;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // 图片消息订阅回调
  void FeedImg(const hbm_img_msgs::msg::HbmMsg1080P::ConstSharedPtr msg);

  void restart_subscription();
  void stop_subscription(int choice);

  void subscription_callback_return(const std_msgs::msg::Int32::SharedPtr msg);     // sign4return的回调函数
  void subscription_callback_switch(const origincar_msg::msg::Sign::SharedPtr msg); // sign_switch的回调函数
};

struct dovecar_yolo{
  int xmin;
  int xmax;
  int ymin;
  int ymax;
};
