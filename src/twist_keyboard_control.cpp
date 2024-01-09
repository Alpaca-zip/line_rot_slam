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

#include "twist_keyboard_control/twist_keyboard_control.h"

TwistKeyboardControl::TwistKeyboardControl() : pnh_("~")
{
  pnh_.param<float>("linear_velocity", linear_velocity_, 0.5);
  pnh_.param<float>("angular_velocity", angular_velocity_, 0.5);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  timer_ = nh_.createTimer(ros::Duration(0.1), &TwistKeyboardControl::twistTimerCallback, this);

  client_.reset(new Client("optimization", true));
  goal_.state = "STOP";

  displayInstructions();
}

void TwistKeyboardControl::twistTimerCallback(const ros::TimerEvent& event)
{
  twist_pub_.publish(twist_);
}

void TwistKeyboardControl::controlLoop()
{
  if (!client_->waitForServer(ros::Duration(1.0)))
  {
    ROS_INFO("Waiting for the optimization action server...");
    return;
  }

  int c = getch();

  if (c == 'w' && goal_.state == "STOP")
  {
    std::cout << "\033[33m **MOVE FORWARD**\033[0m" << std::endl;
    twist_.linear.x = linear_velocity_;
    goal_.state = "MOVE";
  }
  else if (c == 's' && goal_.state == "STOP")
  {
    std::cout << "\033[33m **MOVE BACKWARD**\033[0m" << std::endl;
    twist_.linear.x = -linear_velocity_;
    goal_.state = "MOVE";
  }
  else if (c == 'a' && goal_.state == "STOP")
  {
    std::cout << "\033[33m **TURN LEFT**\033[0m" << std::endl;
    twist_.angular.z = angular_velocity_;
    goal_.state = "TURN";
  }
  else if (c == 'd' && goal_.state == "STOP")
  {
    std::cout << "\033[33m **TURN RIGHT**\033[0m" << std::endl;
    twist_.angular.z = -angular_velocity_;
    goal_.state = "TURN";
  }
  else if (c == ' ' && goal_.state != "STOP")
  {
    std::cout << "\033[31m **STOP**\033[0m" << std::endl;
    twist_.linear.x = 0.0;
    twist_.angular.z = 0.0;
    twist_pub_.publish(twist_);
    client_->sendGoal(goal_);
    client_->waitForResult(ros::Duration(10.0));
    goal_.state = "STOP";
  }
}

void TwistKeyboardControl::displayInstructions()
{
  std::cout << "\033[32m---------------------------" << std::endl;
  std::cout << "Twist Keyboard Control:" << std::endl;
  std::cout << std::endl;
  std::cout << "         w" << std::endl;
  std::cout << std::endl;
  std::cout << "   a     +     d" << std::endl;
  std::cout << std::endl;
  std::cout << "         s" << std::endl;
  std::cout << std::endl;
  std::cout << "w/s : move forward/backward" << std::endl;
  std::cout << "a/d : turn left/right" << std::endl;
  std::cout << "SPACE : stop" << std::endl;
  std::cout << "CTRL-C to quit" << std::endl;
  std::cout << "---------------------------\033[0m" << std::endl;
}

int TwistKeyboardControl::getch()
{
  static struct termios oldt, newt;
  int ch;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_keyboard_control_node");
  TwistKeyboardControl node;
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    node.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
