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

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <rclcpp/qos.hpp>
#include <array>

using twist = geometry_msgs::msg::Twist;
using std::placeholders::_1;
using lzrScan = sensor_msgs::msg::LaserScan;

class walker : public rclcpp::Node{

    public:
        /**
        * @brief Create walker node
        */
        walker() : Node("walker"), qos(10){
            RCLCPP_INFO(this->get_logger(), "Initializing walker node");


            qos = rclcpp::QoS(10);

            // auto cb = std::bind(&walker::laserData_callback, this, _1);
            laser_scan_subscriber = 
                this->create_subscription<lzrScan>("scan", qos, std::bind(
                    &walker::scan_cb, this, std::placeholders::_1
                ));
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
        void move(double vx, double ang_z){
            auto twist_vel = twist();
            twist_vel.linear.x = vx;
            twist_vel.angular.z = ang_z;

            cmdVel_publisher->publish(twist_vel);
            RCLCPP_INFO(this->get_logger(), "Robot is moving");

        }


        /**
        * @brief 
        * 
        * @param lzrData 
        */
        void scan_cb(const lzrScan& lzrData){

            auto lzrScan_ranges = lzrData.ranges;

            for (int i = 330; i < 330 + 60; i++) {
                if (lzrScan_ranges[i % 360] < 0.8) {
                    move(0.0, 0.1);
                } else {
                    move(0.1, 0.0);
                }
            }

            // for (int num = 0; num < 3; num++){
            //     if(std::isinf(lzrData->ranges.at(lzrScan_ranges[num]))){
            //         scan_data_[num] = lzrData->range_max;
            //         move(0.1, 0.0);
            //     } else{
            //         scan_data_[num] = lzrData->ranges.at(lzrScan_ranges[num]);
            //         move(0.0, 0.1);
            //     }
            // }

            RCLCPP_INFO(this->get_logger(), "scan_cb");

        }

        /**
         * @brief Turn the robot is less than a predefined distance from obstacle
         * 
         * @param scanMsg 
         */
        // void turn_robot(const lzrScan scanMsg){

        //     if (scanMsg->distance[0] < 1){

        //         twist_vel.angular.z = 0.1;
        //         twist_vel.linear.x = 0.0;

        //     }

        // }
};

int main(int argc, char ** argv){

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<walker>());
    rclcpp::shutdown();
    return 0;

}