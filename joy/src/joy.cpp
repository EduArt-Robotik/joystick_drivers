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

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include <SDL.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>

#include "joy/joy.hpp"

namespace joy
{

Joy::Joy(const rclcpp::NodeOptions & options)
: rclcpp::Node("joy_node", options)
{
  parameter_.dev_id = static_cast<int>(this->declare_parameter("device_id", 0));

  parameter_.dev_name = this->declare_parameter("device_name", std::string(""));

  // The user specifies the deadzone to us in the range of 0.0 to 1.0.  Later on
  // we'll convert that to the range of 0 to 32767.  Note also that negatives
  // are not allowed, as this is a +/- value.
  parameter_.scaled_deadzone = this->declare_parameter("deadzone", 0.05);
  if (parameter_.scaled_deadzone < 0.0 || parameter_.scaled_deadzone > 1.0) {
    throw std::runtime_error("Deadzone must be between 0.0 and 1.0");
  }
  parameter_.unscaled_deadzone = 32767.0 * parameter_.scaled_deadzone;
  // According to the SDL documentation, this always returns a value between
  // -32768 and 32767.  However, we want to report a value between -1.0 and 1.0,
  // hence the "scale" dividing by 32767.  Also note that SDL returns the axes
  // with "forward" and "left" as negative.  This is opposite to the ROS
  // conventionof "forward" and "left" as positive, so we invert the axes here
  // as well.  Finally, we take into account the amount of deadzone so we truly
  // do get value between -1.0 and 1.0 (and not -deadzone to +deadzone).
  parameter_.scale = static_cast<float>(-1.0 / (1.0 - parameter_.scaled_deadzone) / 32767.0);

  const double publishing_rate = this->declare_parameter("rate", 20.0);
  if (publishing_rate < 0.0) {
    throw std::runtime_error("rate must be >= 0.0");
  } else if (publishing_rate > 1000.0) {
    throw std::runtime_error("rate must be <= 1000.0");
  } else {
    parameter_.publishing_interval = rclcpp::Duration(0, (1.0 / publishing_rate) * 1000 * 1000 * 1000);
  }

  parameter_.sticky_buttons = this->declare_parameter("sticky_buttons", false);

  pub_ = create_publisher<sensor_msgs::msg::Joy>("joy", 10);

  feedback_sub_ = this->create_subscription<sensor_msgs::msg::JoyFeedback>(
    "joy/set_feedback", rclcpp::QoS(10), std::bind(
      &Joy::feedbackCb, this,
      std::placeholders::_1));

  future_ = exit_signal_.get_future();

  if (SDL_Init(SDL_INIT_JOYSTICK | SDL_INIT_HAPTIC) < 0) {
    throw std::runtime_error("SDL could not be initialized: " + std::string(SDL_GetError()));
  }
  // In theory we could do this with just a timer, which would simplify the code
  // a bit.  But then we couldn't react to "immediate" events, so we stick with
  // the thread.
  event_thread_ = std::thread(&Joy::eventThread, this);
}

Joy::~Joy()
{
  exit_signal_.set_value();
  event_thread_.join();
  if (haptic_ != nullptr) {
    SDL_HapticClose(haptic_);
  }
  if (joystick_ != nullptr) {
    SDL_JoystickClose(joystick_);
  }
  SDL_Quit();
}

void Joy::feedbackCb(const std::shared_ptr<sensor_msgs::msg::JoyFeedback> msg)
{
  if (haptic_ == nullptr) {
    // No ability to do feedback, so ignore.
    return;
  }

  if (msg->type != sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE) {
    // We only support rumble
    return;
  }

  if (msg->id != 0) {
    // There can be only one (rumble)
    return;
  }

  if (msg->intensity < 0.0 || msg->intensity > 1.0) {
    // We only accept intensities between 0 and 1.
    return;
  }

  // We purposely ignore the return value; if it fails, what can we do?
  SDL_HapticRumblePlay(haptic_, msg->intensity, 1000);
}

float Joy::convertRawAxisValueToROS(int16_t val)
{
  // SDL reports axis values between -32768 and 32767.  To make sure
  // we report out scaled value between -1.0 and 1.0, we add one to
  // the value iff it is exactly -32768.  This makes all of the math
  // below work properly.
  if (val == -32768) {
    val = -32767;
  }

  // Note that we do all of the math in double space below.  This ensures
  // that the values stay between -1.0 and 1.0.
  double double_val = static_cast<double>(val);
  // Apply the deadzone semantic here.  This allows the deadzone
  // to be "smooth".
  if (double_val > parameter_.unscaled_deadzone) {
    double_val -= parameter_.unscaled_deadzone;
  } else if (double_val < -parameter_.unscaled_deadzone) {
    double_val += parameter_.unscaled_deadzone;
  } else {
    double_val = 0.0;
  }

  return static_cast<float>(double_val * parameter_.scale);
}

bool Joy::handleJoyAxis(const SDL_Event & e)
{
  if (e.jaxis.which != parameter_.joystick_instance_id) {
    return false;
  }

  if (e.jaxis.axis >= joy_msg_.axes.size()) {
    RCLCPP_WARN(get_logger(), "Saw axes too large for this device, ignoring");
    return false;
  }

  float last_axis_value = joy_msg_.axes.at(e.jaxis.axis);
  joy_msg_.axes.at(e.jaxis.axis) = convertRawAxisValueToROS(e.jaxis.value);

  return last_axis_value != joy_msg_.axes.at(e.jaxis.axis);
}

bool Joy::handleJoyButtonDown(const SDL_Event & e)
{
  if (e.jbutton.which != parameter_.joystick_instance_id) {
    return false;
  }

  if (e.jbutton.button >= joy_msg_.buttons.size()) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return false;
  }

  if (parameter_.sticky_buttons) {
    // For sticky buttons, invert 0 -> 1 or 1 -> 0
    joy_msg_.buttons.at(e.jbutton.button) = 1 - joy_msg_.buttons.at(e.jbutton.button);
  } else {
    joy_msg_.buttons.at(e.jbutton.button) = 1;
  }

  return true;
}

bool Joy::handleJoyButtonUp(const SDL_Event & e)
{
  if (e.jbutton.which != parameter_.joystick_instance_id) {
    return false;
  }

  if (e.jbutton.button >= joy_msg_.buttons.size()) {
    RCLCPP_WARN(get_logger(), "Saw button too large for this device, ignoring");
    return false;
  }

  if (!parameter_.sticky_buttons) {
    joy_msg_.buttons.at(e.jbutton.button) = 0;
    return true;
  }

  return false;
}

bool Joy::handleJoyHatMotion(const SDL_Event & e)
{
  if (e.jhat.which != parameter_.joystick_instance_id) {
    return false;
  }

  // The hats are the last axes in the axes list.  There are two axes per hat;
  // the first of the pair is for left (positive) and right (negative), while
  // the second of the pair is for up (positive) and down (negative).

  // Determine which pair we are based on e.jhat.hat
  const int num_axes = SDL_JoystickNumAxes(joystick_);
  if (num_axes < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get axes: %s", SDL_GetError());
    return false;
  }
  size_t axes_start_index = num_axes + e.jhat.hat * 2;
  // Note that we check axes_start_index + 1 here to ensure that we can write to
  // either the left/right axis or the up/down axis that corresponds to this hat.
  if ((axes_start_index + 1) >= joy_msg_.axes.size()) {
    RCLCPP_WARN(get_logger(), "Saw hat too large for this device, ignoring");
    return false;
  }

  if (e.jhat.value & SDL_HAT_LEFT) {
    joy_msg_.axes.at(axes_start_index) = 1.0;
  }
  if (e.jhat.value & SDL_HAT_RIGHT) {
    joy_msg_.axes.at(axes_start_index) = -1.0;
  }
  if (e.jhat.value & SDL_HAT_UP) {
    joy_msg_.axes.at(axes_start_index + 1) = 1.0;
  }
  if (e.jhat.value & SDL_HAT_DOWN) {
    joy_msg_.axes.at(axes_start_index + 1) = -1.0;
  }
  if (e.jhat.value == SDL_HAT_CENTERED) {
    joy_msg_.axes.at(axes_start_index) = 0.0;
    joy_msg_.axes.at(axes_start_index + 1) = 0.0;
  }

  return true;
}

void Joy::handleJoyDeviceAdded(const SDL_Event & e)
{
  if (parameter_.dev_name.empty() == false) {
    int num_joysticks = SDL_NumJoysticks();
    if (num_joysticks < 0) {
      RCLCPP_WARN(get_logger(), "Failed to get the number of joysticks: %s", SDL_GetError());
      return;
    }
    bool matching_device_found = false;
    for (int i = 0; i < num_joysticks; ++i) {
      const char * name = SDL_JoystickNameForIndex(i);
      if (name == nullptr) {
        RCLCPP_WARN(get_logger(), "Could not get joystick name: %s", SDL_GetError());
        continue;
      }
      if (std::string(name) == parameter_.dev_name) {
        // We found it!
        matching_device_found = true;
        parameter_.dev_id = i;
        break;
      }
    }
    if (!matching_device_found) {
      RCLCPP_WARN(
        get_logger(), "Could not get joystick with name %s: %s",
        parameter_.dev_name.c_str(), SDL_GetError());
      return;
    }
  }

  if (e.jdevice.which != parameter_.dev_id) {
    return;
  }

  joystick_ = SDL_JoystickOpen(parameter_.dev_id);
  if (joystick_ == nullptr) {
    RCLCPP_WARN(get_logger(), "Unable to open joystick %d: %s", parameter_.dev_id, SDL_GetError());
    return;
  }

  // We need to hold onto this so that we can properly remove it on a
  // remove event.
  parameter_.joystick_instance_id = SDL_JoystickGetDeviceInstanceID(parameter_.dev_id);
  if (parameter_.joystick_instance_id < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get instance ID for joystick: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }

  int num_buttons = SDL_JoystickNumButtons(joystick_);
  if (num_buttons < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get number of buttons: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  joy_msg_.buttons.resize(num_buttons);

  int num_axes = SDL_JoystickNumAxes(joystick_);
  if (num_axes < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get number of axes: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  int num_hats = SDL_JoystickNumHats(joystick_);
  if (num_hats < 0) {
    RCLCPP_WARN(get_logger(), "Failed to get number of hats: %s", SDL_GetError());
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
    return;
  }
  joy_msg_.axes.resize(num_axes + num_hats * 2);

  // Get the initial state for each of the axes
  for (int i = 0; i < num_axes; ++i) {
    int16_t state;
    if (SDL_JoystickGetAxisInitialState(joystick_, i, &state)) {
      joy_msg_.axes.at(i) = convertRawAxisValueToROS(state);
    }
  }

  haptic_ = SDL_HapticOpenFromJoystick(joystick_);
  if (haptic_ != nullptr) {
    if (SDL_HapticRumbleInit(haptic_) < 0) {
      // Failed to init haptic.  Clean up haptic_.
      SDL_HapticClose(haptic_);
      haptic_ = nullptr;
    }
  } else {
    RCLCPP_INFO(get_logger(), "No haptic (rumble) available, skipping initialization");
  }

  RCLCPP_INFO(
    get_logger(), "Opened joystick: %s.  deadzone: %f",
    SDL_JoystickName(joystick_), parameter_.scaled_deadzone);
}

void Joy::handleJoyDeviceRemoved(const SDL_Event & e)
{
  if (e.jdevice.which != parameter_.joystick_instance_id) {
    return;
  }

  joy_msg_.buttons.resize(0);
  joy_msg_.axes.resize(0);
  if (haptic_ != nullptr) {
    SDL_HapticClose(haptic_);
    haptic_ = nullptr;
  }
  if (joystick_ != nullptr) {
    SDL_JoystickClose(joystick_);
    joystick_ = nullptr;
  }
}

void Joy::eventThread()
{
  std::future_status status;
  rclcpp::Time last_publish = this->now();

  do {
    SDL_Event e;

    if (SDL_WaitEventTimeout(&e, (parameter_.publishing_interval.nanoseconds() / 1000000) / 10)) {
      // Succeeded getting an event
      if (e.type == SDL_JOYAXISMOTION) {
        handleJoyAxis(e);
      } else if (e.type == SDL_JOYBUTTONDOWN) {
        handleJoyButtonDown(e);
      } else if (e.type == SDL_JOYBUTTONUP) {
        handleJoyButtonUp(e);
      } else if (e.type == SDL_JOYHATMOTION) {
        handleJoyHatMotion(e);
      } else if (e.type == SDL_JOYDEVICEADDED) {
        handleJoyDeviceAdded(e);
      } else if (e.type == SDL_JOYDEVICEREMOVED) {
        handleJoyDeviceRemoved(e);
      } else {
        RCLCPP_INFO(get_logger(), "Unknown event type %d", e.type);
      }
    }

    const auto now = get_clock()->now();
    const auto dt = now - last_publish;

    if (joystick_ != nullptr && dt >= parameter_.publishing_interval) {
      joy_msg_.header.frame_id = parameter_.frame_id_;
      joy_msg_.header.stamp = now;

      pub_->publish(joy_msg_);
      last_publish = now;
    }

    status = future_.wait_for(std::chrono::seconds(0));
  } while (status == std::future_status::timeout);
}

}  // namespace joy

RCLCPP_COMPONENTS_REGISTER_NODE(joy::Joy)
