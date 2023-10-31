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

  optimization_end_flag_sub_ =
      nh_.subscribe("optimization_end_flag", 10, &TwistKeyboardControl::optimizationEndCallback, this);
  optimization_start_flag_pub_ = nh_.advertise<std_msgs::Bool>("optimization_start_flag", 10);
  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  previous_state_ = State::STOP;
  current_state_ = State::MOVE;

  displayInstructions();
  std::cout << "\033[32m **START MOVING**\033[0m" << std::endl;
}

void TwistKeyboardControl::optimizationEndCallback(const std_msgs::Bool& flag_msg)
{
  if (!flag_msg.data)
  {
    return;
  }
  else if (previous_state_ == State::MOVE)
  {
    previous_state_ = current_state_;
    current_state_ = State::TURN;
    std::cout << "\033[32m **START TURNING**\033[0m" << std::endl;
  }
  else if (previous_state_ == State::TURN)
  {
    previous_state_ = current_state_;
    current_state_ = State::MOVE;
    std::cout << "\033[32m **START MOVING**\033[0m" << std::endl;
  }
}

void TwistKeyboardControl::controlLoop()
{
  std_msgs::Bool msg;
  geometry_msgs::Twist twist;

  int c = getch();

  if (current_state_ == State::STOP)
  {
    msg.data = false;
  }
  else if (current_state_ == State::MOVE)
  {
    if (c == -1 && previous_state_ == State::STOP)
    {
      msg.data = false;
    }
    else if (c == 'w' && previous_state_ == State::STOP)
    {
      std::cout << "\033[32m **MOVE FORWARD**\033[0m" << std::endl;
      twist.linear.x = linear_velocity_;
      twist_pub_.publish(twist);
      previous_state_ = current_state_;
      msg.data = false;
    }
    else if (c == 's' && previous_state_ == State::STOP)
    {
      std::cout << "\033[32m **MOVE BACKWARD**\033[0m" << std::endl;
      twist.linear.x = -linear_velocity_;
      twist_pub_.publish(twist);
      previous_state_ = current_state_;
      msg.data = false;
    }
    else if (c == ' ' && previous_state_ != State::STOP)
    {
      std::cout << "\033[32m **STOP MOVING**\033[0m" << std::endl;
      twist_pub_.publish(twist);
      current_state_ = State::STOP;
      msg.data = true;
    }
  }
  else if (current_state_ == State::TURN)
  {
    if (c == -1 && previous_state_ == State::STOP)
    {
      msg.data = false;
    }
    else if (c == 'a' && previous_state_ == State::STOP)
    {
      std::cout << "\033[32m **TURN LEFT**\033[0m" << std::endl;
      twist.angular.z = angular_velocity_;
      twist_pub_.publish(twist);
      previous_state_ = current_state_;
      msg.data = false;
    }
    else if (c == 'd' && previous_state_ == State::STOP)
    {
      std::cout << "\033[32m **TURN RIGHT**\033[0m" << std::endl;
      twist.angular.z = -angular_velocity_;
      twist_pub_.publish(twist);
      previous_state_ = current_state_;
      msg.data = false;
    }
    else if (c == ' ' && previous_state_ != State::STOP)
    {
      std::cout << "\033[32m **STOP TURNING**\033[0m" << std::endl;
      twist_pub_.publish(twist);
      current_state_ = State::STOP;
      msg.data = true;
    }
  }

  optimization_start_flag_pub_.publish(msg);
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
  int oldf;

  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

  ch = getchar();

  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);

  if (ch != EOF)
  {
    return ch;
  }

  return -1;
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
