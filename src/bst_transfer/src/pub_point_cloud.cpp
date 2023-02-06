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
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>


#include "pcl_conversions/pcl_conversions.h"

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


class PointCloud2Publisher : public rclcpp::Node {
public:
  PointCloud2Publisher(const std::string & topic_name): Node("pointcloud2_publisher"), count_(0) {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 10);
    timer_ = this->create_wall_timer(40ms, std::bind(&PointCloud2Publisher::timer_callback, this));
    LoadImageNames(ws_name_+"/disp_images_list.txt", disp_names_);
    img_num = disp_names_.size();
  }

private:
  void timer_callback() {
    std::string disp_name = disp_names_[count_];
    if (count_ == img_num - 1) {
        count_ = 0;
    } else {
        count_++;
    }
    // std::cout<<ws_name_+"/"+left_name<<std::endl;
    RCLCPP_INFO(this->get_logger(), "'%s'/%s'", ws_name_.c_str(), disp_name.c_str());
    cv::Mat disp_img = cv::imread(ws_name_ + "/" + disp_name, cv::IMREAD_ANYDEPTH);
    // left_img = left_img(cv::Rect(0, 0, 1920, 1200));
    // std::cout<<"disp_img cols:"<<disp_img.cols<<" disp_img rows:"<<disp_img.rows<<" disp_img channels:"<<disp_img.channels()<<std::endl;
    
    int max_disp = 64;
    int min_disp = 0;
    int disp_range = max_disp - min_disp;
    float dr,db,dg,v;
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    int num_points = disp_img.rows * disp_img.cols;
    cloud.points.resize(num_points);
    for (int i = 0; i < disp_img.rows; i++) {
      for (int j = 0; j< disp_img.cols; j++) {
        float disp = (float)disp_img.at<short>(i,j) /32;
        int index = disp_img.cols * i + j;
        
        if (fabs(disp) < 0.1) {
            cloud.points[index].x = 10000000.f;
            cloud.points[index].y = 10000000.f;
            cloud.points[index].z = 10000000.f;
        } else {
            cloud.points[index].x = (float)((160 * (j - 960))/disp)/1000;//depth * tx/1000;
            cloud.points[index].y = (float)((160 * (i - 600))/disp)/1000;//depth * ty/1000;
            cloud.points[index].z = (float)(320/disp);//depth/1000;
            // std::cout<<cloud.points[index].x << " " << cloud.points[index].y <<" " <<cloud.points[index].z<<std::endl;
        }

        if (disp > max_disp) {
          v = 1.0;
        } else {
          v = (disp*1.f - min_disp) / disp_range;
        }
            
        if (v < 0.1242f) {
          dr = 0.f;
          db = 0.504f + ((1.f - 0.504f) / 0.1242f) * v;
          dg = 0.f;
        } else if (v <0.3747f) {
          dr = 0.f;
          dg = (v - 0.1242f) * (1.f / (0.3747f - 0.1242f));
          db = 1.f;
        } else if (v < 0.6253f) {
          dr = (v - 0.3747f) * (1.f / (0.6253f -0.3747f));
          dg = 1.f;
          db = (0.6253 - v) * (1.f / (0.6253f -0.3747f));
        } else if (v < 0.8758f) {
          dr = 1.f;
          dg = (0.8758f - v) * (1.f / (0.8758f - 0.6253f));
          db = 0.f;
        } else {
          dr = 1.f - (v - 0.8758f) * ((1.f - 0.504f) / (1.f - 0.8758f));
          dg = 0.f;
          db = 0.f;
        }


        cloud.points[index].r = dr *255;
        cloud.points[index].g = dg *255;
        cloud.points[index].b = db *255;
                
      }
    }
    // std::cout<<max_disp<<" "<<min_disp<<std::endl;
 
    //将pcl点云转换到ros消息对象
    pcl::toROSMsg(cloud, output_msg);
    
    // 发布的点云坐标系
    output_msg.header.frame_id="mpv";
    output_msg.header.stamp = this->get_clock()->now();
    publisher_->publish(output_msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  size_t count_, img_num;
  std::vector<std::string> disp_names_;
  std::string ws_name_ = "/root/ros2_ws/ros2_example";
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto topic_name = std::string(argv[1]);
  rclcpp::spin(std::make_shared<PointCloud2Publisher>(topic_name));
  rclcpp::shutdown();
  return 0;
}
