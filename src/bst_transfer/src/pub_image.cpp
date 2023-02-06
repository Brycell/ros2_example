// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <memory>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
static void LoadImageNames(std::string const &filename,
                    std::vector<std::string> &images) {
  images.clear();

  /*Check if path is a valid directory path. */
  FILE *fp = fopen(filename.c_str(), "r");
  if (NULL == fp) {
    fprintf(stdout, "open file: %s  error\n", filename.c_str());
    exit(1);
  }

  char buffer[256] = {0};
  while (fgets(buffer, 256, fp) != NULL) {
    int n = strlen(buffer);
    buffer[n - 1] = '\0';
    std::string name = buffer;
    images.push_back(name);
  }

  fclose(fp);
}


class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher(const std::string & topic_name): Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);
    timer_ = this->create_wall_timer(40ms, std::bind(&MinimalPublisher::timer_callback, this));
    LoadImageNames(ws_name_+"/left_images_list.txt", left_names_);
    img_num = left_names_.size();
  }

private:
  void timer_callback() {
    std::string left_name = left_names_[count_];
    if (count_ == img_num - 1) {
        count_ = 0;
    } else {
        count_++;
    }
    // std::cout<<ws_name_+"/"+left_name<<std::endl;
    RCLCPP_INFO(this->get_logger(), "'%s'/%s'", ws_name_.c_str(), left_name.c_str());
    cv::Mat left_img = cv::imread(ws_name_+"/"+left_name);
    // std::cout<<"left_img cols:"<<left_img.cols<<" left_img rows:"<<left_img.rows<<" left_img channels:"<<left_img.channels()<<std::endl;
    // RCLCPP_INFO(this->get_logger(), "left_img cols:'%s' left_img rows:'%s' left_img channels:'%s'", 
    //             std::string(left_img.cols).c_str(), std::string(left_img.rows).c_str(), std::string(left_img.channels()).c_str());
    cv_bridge::CvImage cvi_rgb;
    cvi_rgb.header.stamp = this->get_clock()->now();
    cvi_rgb.header.frame_id = "dabai_rgb";
    cvi_rgb.encoding = "bgr8";
    cvi_rgb.image = left_img;
    sensor_msgs::msg::Image im_rgb;
    cvi_rgb.toImageMsg(im_rgb);

    // auto message = std_msgs::msg::String();
    // message.data = "Hello, world! " + std::to_string(count_++);
    // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(im_rgb);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  size_t count_, img_num;
  std::vector<std::string> left_names_;
  std::string ws_name_ = "/root/ros2_ws/ros2_example";
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto topic_name = std::string(argv[1]);
  rclcpp::spin(std::make_shared<MinimalPublisher>(topic_name));
  rclcpp::shutdown();
  return 0;
}
