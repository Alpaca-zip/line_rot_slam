/**
 * Copyright (C) 2023  Alpaca-zip
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
 */

#pragma once

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <actionlib/client/simple_action_client.h>
#include <line_rot_slam/OptimizationAction.h>
#include <termios.h>

typedef actionlib::SimpleActionClient<line_rot_slam::OptimizationAction> Client;

class TwistKeyboardControl
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher twist_pub_;
  ros::Timer timer_;
  geometry_msgs::Twist twist_;
  line_rot_slam::OptimizationGoal goal_;
  float linear_velocity_;
  float angular_velocity_;
  boost::shared_ptr<Client> client_;

public:
  TwistKeyboardControl();
  void twistTimerCallback(const ros::TimerEvent& event);
  void controlLoop();
  void displayInstructions();
  int getch();
};
