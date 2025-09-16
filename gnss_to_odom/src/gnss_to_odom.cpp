/*
 * gnss_to_odom.cpp
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

#include "gnss_to_odom/gnss_to_odom.hpp"
#include <array>

GnssToOdom::GnssToOdom() : Node("gnss_to_odom")
{
    using std::placeholders::_1;

    gnss_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gnss", 1, std::bind(&GnssToOdom::gnss_callback, this, _1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/gnss/odom", 1);

    configure = false;
}

void GnssToOdom::gnss_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{

    if (!configure)
    {
        configure = true;

        ll_to_utm_.initUTM(msg->latitude, msg->longitude, msg->altitude);
    }

    nav_msgs::msg::Odometry odom;

    odom.header = msg->header;

    ll_to_utm_.transform_global(msg->latitude,
                                msg->longitude,
                                msg->altitude,
                                odom.pose.pose.position.x,
                                odom.pose.pose.position.y, 
                                odom.pose.pose.position.z);

    std::array<double, 36> cov;

    cov.fill(0.0);

    cov[0] = msg->position_covariance[0];
    cov[7] = msg->position_covariance[4];
    cov[14] = msg->position_covariance[8];

    odom.pose.covariance = cov;

    odom_pub_->publish(odom);
}