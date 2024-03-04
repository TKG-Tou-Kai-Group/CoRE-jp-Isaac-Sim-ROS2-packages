/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2014, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef TELEOP_TWIST_JOY__TELEOP_TWIST_JOY_HPP_
#define TELEOP_TWIST_JOY__TELEOP_TWIST_JOY_HPP_

#include <rclcpp/rclcpp.hpp>
#include "teleop_twist_joy_for_sample_robot/teleop_twist_joy_for_sample_robot_export.h"

namespace teleop_twist_joy_for_sample_robot
{

  /**
   * Class implementing a basic Joy -> Twist translation.
   */
  class TELEOP_TWIST_JOY_FOR_SAMPLE_ROBOT_EXPORT TeleopTwistJoy : public rclcpp::Node
  {
  public:
    explicit TeleopTwistJoy(const rclcpp::NodeOptions &options);

    virtual ~TeleopTwistJoy();

  private:
    struct Impl;
    Impl *pimpl_;
    OnSetParametersCallbackHandle::SharedPtr callback_handle;
  };

} // namespace teleop_twist_joy

#endif // TELEOP_TWIST_JOY__TELEOP_TWIST_JOY_HPP_
