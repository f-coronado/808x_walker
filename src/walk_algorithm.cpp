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

/**
* @file walk_algorithm.cpp
* @author f-coronado
* @brief Walkser script
* @date 11/26/2023
*
* @copyright Copyright (c) 2023
*
*/

#include <array>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/qos.hpp>

using twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using lzrScan = sensor_msgs::msg::LaserScan;

class walker : public rclcpp::Node {
 public:
        /**
        * @brief Create walker node
        */
        walker() : Node("walker"), qos(10) {
            RCLCPP_INFO(this->get_logger(), "Initializing walker node");


            qos = rclcpp::QoS(10);

            // auto cb = std::bind(&walker::laserData_callback, this, _1);
            laser_scan_subscriber =
                this->create_subscription<lzrScan>("scan", qos, std::bind(
                    &walker::scan_cb, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "Subscriber loaded successfully");

            scan_data_[0] = 0.0;
            scan_data_[1] = 0.0;
            scan_data_[2] = 0.0;

            /**
             * @brief Create a publisher to the topic cmd_vel topic
             * 
             */
            cmdVel_publisher = this->create_publisher<twist>("cmd_vel", qos);
            RCLCPP_INFO(this->get_logger(), "Publisher loaded successfully");
        }

 private:
        rclcpp::QoS qos;
        std::array<float, 3>scan_data_;

        /**
         * @brief Declaring the publisher as a shared pointer of type twist
         * 
         */
        rclcpp::Publisher<twist>::SharedPtr cmdVel_publisher;
        /**
         * @brief Declaring the subscriber as a shared pointer of type lzrScan
         * 
         */
        rclcpp::Subscription<lzrScan>::SharedPtr laser_scan_subscriber;

        /**
         * @brief Method to move the robot
         * 
         * @param vx 
         * @param ang_z 
         */
        void move(double vx, double ang_z) {
            auto twist_vel = twist();
            twist_vel.linear.x = vx;
            twist_vel.angular.z = ang_z;

            cmdVel_publisher->publish(twist_vel);
            RCLCPP_INFO(this->get_logger(), "Move method called!");
        }

        /**
         * @brief Callback which guides the robot movement
         * 
         * @param msg
         */
        void scan_cb(const lzrScan& msg){
            // Check Right side peripheral vision
            for (float range : std::vector<float>(msg.ranges.begin(),
                msg.ranges.begin() + 45)) {
                if (range < 1) {
                    // Obstacle detected on the right side
                    move(0.0, 0.1);
                } else {
                    move(0.1, 0.0);
                }
            }

            // Check Left side peripheral vision
            for (float range : std::vector<float>(msg.ranges.begin() + 325,
                msg.ranges.end())) {
                if (range < 1) {
                    // Obstacle detected on the left side
                    move(0.0, -0.1);
                } else {
                    move(0.1, 0.0);
                }
            }
        }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<walker>());
    rclcpp::shutdown();
    return 0;
}
