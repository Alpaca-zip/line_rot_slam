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
#include <termios.h>
#include <fcntl.h>

#define REPRINT_INTERVAL 5

enum State
{
  MOVE,
  TURN,
  STOP
};

class TwistKeyboardControl
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber optimization_end_flag_sub_;
  ros::Publisher optimization_start_flag_pub_;
  ros::Publisher twist_pub_;
  enum State previous_state_;
  enum State current_state_;
  float linear_velocity_;
  float angular_velocity_;

public:
  TwistKeyboardControl();
  void optimizationEndCallback(const std_msgs::Bool& flag_msg);
  void controlLoop();
  void displayInstructions();
  int getch();
};
