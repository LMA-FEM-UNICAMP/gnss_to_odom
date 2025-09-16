/*
 * gnss_to_odom.hpp
 *
 *  Created on: Apr 16, 2025
 *
 *  Author: Gabriel Toffanetto França da Rocha
 *
 *  Laboratory of Autonomous Mobility (LMA)
 *  School of Mechanical Engineering (FEM)
 *  University of Campinas (Unicamp)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef gnss_to_odom__gnss_to_odom_HPP_
#define gnss_to_odom__gnss_to_odom_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "gnss_to_odom/ll_to_utm_transform.h"


class GnssToOdom : public rclcpp::Node
{
public:
    GnssToOdom();

private:

    bool configure;

    clap_b7::LlToUtmTransform ll_to_utm_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    void gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};
#endif  // gnss_to_odom__gnss_to_odom_HPP_
