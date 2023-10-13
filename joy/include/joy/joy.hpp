/*
 * Copyright (c) 2020, Open Source Robotics Foundation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <SDL.h>

#include <future>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

namespace joy
{

class Joy final : public rclcpp::Node
{
public:
  explicit Joy(const rclcpp::NodeOptions & options);
  Joy(Joy && c) = delete;
  Joy & operator=(Joy && c) = delete;
  Joy(const Joy & c) = delete;
  Joy & operator=(const Joy & c) = delete;

  ~Joy() override;

private:
  void eventThread();
  bool handleJoyAxis(const SDL_Event & e);
  bool handleJoyButtonDown(const SDL_Event & e);
  bool handleJoyButtonUp(const SDL_Event & e);
  bool handleJoyHatMotion(const SDL_Event & e);
  void handleJoyDeviceAdded(const SDL_Event & e);
  void handleJoyDeviceRemoved(const SDL_Event & e);
  float convertRawAxisValueToROS(int16_t val);
  void feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg);

  struct Parameter {
    int dev_id{0};
    std::string dev_name;
    int32_t joystick_instance_id{0};
    double scaled_deadzone{0.0};
    double unscaled_deadzone{0.0};
    double scale{0.0};
    bool sticky_buttons{false};
    rclcpp::Duration publishing_interval = rclcpp::Duration(0, 10000000); // default: 10ms
    std::string frame_id_ = "joy";
  } parameter_;

  SDL_Joystick * joystick_{nullptr};
  SDL_Haptic * haptic_{nullptr};

  std::thread event_thread_;
  std::shared_future<void> future_;
  std::promise<void> exit_signal_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_sub_;

  sensor_msgs::msg::Joy joy_msg_;
};

}  // namespace joy
