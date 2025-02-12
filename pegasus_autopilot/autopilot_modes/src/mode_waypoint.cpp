/*****************************************************************************
 * 
 *   Author: Marcelo Jacinto <marcelo.jacinto@tecnico.ulisboa.pt>
 *   Copyright (c) 2024, Marcelo Jacinto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright 
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in 
 * the documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this 
 * software must display the following acknowledgement: This product 
 * includes software developed by Project Pegasus.
 * 4. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived 
 * from this software without specific prior written permission.
 *
 * Additional Restrictions:
 * 4. The Software shall be used for non-commercial purposes only. 
 * This includes, but is not limited to, academic research, personal 
 * projects, and non-profit organizations. Any commercial use of the 
 * Software is strictly prohibited without prior written permission 
 * from the copyright holders.
 * 5. The Software shall not be used, directly or indirectly, for 
 * military purposes, including but not limited to the development 
 * of weapons, military simulations, or any other military applications. 
 * Any military use of the Software is strictly prohibited without 
 * prior written permission from the copyright holders.
 * 6. The Software may be utilized for academic research purposes, 
 * with the condition that proper acknowledgment is given in all 
 * corresponding publications.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/
#include "autopilot_modes/mode_waypoint.hpp"
#include <pegasus_utils/rotations.hpp>


namespace autopilot {

WaypointMode::~WaypointMode() {
    // Terminate the waypoint service
    this->waypoint_service_.reset();
}

void WaypointMode::initialize() {

    // Create the waypoint service server
    node_->declare_parameter<std::string>("autopilot.WaypointMode.set_waypoint_service", "set_waypoint"); 
    this->waypoint_service_ = this->node_->create_service<pegasus_msgs::srv::Waypoint>(node_->get_parameter("autopilot.WaypointMode.set_waypoint_service").as_string(), std::bind(&WaypointMode::waypoint_callback, this, std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->node_->get_logger(), "WaypointMode initialized");
    attitude_target_publisher_ = this->node_->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude_target", 10);

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    this->mass_ = vehicle_constansts.mass;
}

bool WaypointMode::enter() {

    // Check if the waypoint was already set - if not, then do not enter the waypoint mode
    if (!this->waypoint_set_) {
        RCLCPP_ERROR(this->node_->get_logger(), "Waypoint not set - cannot enter waypoint mode.");
        return false;
    }

    // Reset the waypoint flag (to make sure we do not enter twice in this mode without setting a new waypoint)
    this->waypoint_set_ = false;

    // Return true to indicate that the mode has been entered successfully
    return true;
}

bool WaypointMode::exit() {
    
    // Nothing to do here
    return true;   // Return true to indicate that the mode has been exited successfully
}

void WaypointMode::update(double dt) {
    if (t < 5.0) {
        this->controller_->set_position({0.0, 0.0, -1.5}, 0.0, dt);
        // Set the vector values (roll, pitch, yaw)
        // Set the header (timestamp and frame_id)
        euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

        euler_msg.vector.x = 0; // roll
        euler_msg.vector.y = 0; // pitch
        euler_msg.vector.z = 0; // yaw

        // Publish the Vector3Stamped message
        attitude_target_publisher_->publish(euler_msg);
    } else if (t < 6.0) {
        Eigen::Vector3d attitude_target = compute_attitude(t);
        double T = 9.83 * this->mass_ / std::abs(std::cos(attitude_target[0] * M_PI / 180 + attitude_target[1] * M_PI / 180));

        this->controller_->set_attitude(attitude_target, T, dt);

        // Set the header (timestamp and frame_id)
        euler_msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        euler_msg.header.frame_id = "base_link"; // Set appropriate frame_id

        // Set the vector values (roll, pitch, yaw)
        euler_msg.vector.x = attitude_target[0]; // roll
        euler_msg.vector.y = attitude_target[1]; // pitch
        euler_msg.vector.z = attitude_target[2]; // yaw

        // Publish the Vector3Stamped message
        attitude_target_publisher_->publish(euler_msg);
    }
    else {
        this->controller_->set_position({0.0, 0.0, -1.5}, 0.0, dt);
    }

    this->t += dt; // Increment time
}

// Funtion to compute the attitude target to be sent to the controabs(ller based on a sinusoidal
Eigen::Vector3d WaypointMode::compute_attitude(double t) {

    // Initialize the unit vector
    Eigen::Vector3d attitude = {0.0, 0.0, 0.0};       // Set the z component to 1.0 (to ensure hover mode, cos(~=0) = 1)

    // Calculate the sinusoidal attitude for the specified axis
    // double sinValue = std::sin(2*M_PI*this->frequency * t);

    // Original: attitude[this->axis] = sinValue;
    // attitude[0] = 0.173 * this->axis[0] * sinValue ;
    attitude[0] = 50 * this->axis[0];
    attitude[1] = 50 * this->axis[1];
    return attitude;
}

void WaypointMode::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
    // Set the waypoint
    this->axis[0] = request->position[0];
    this->axis[1] = request->position[1];
    this->frequency = request->position[2];
    this->yaw = request->yaw;
    this->t = 0.0;          // Reset the time to 0.0


    // Set the waypoint flag
    this->waypoint_set_ = true;

    // Return true to indicate that the waypoint has been set successfully
    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Axis oscilation direction set to (%f, %f, %f) with frequency %f", this->axis[0], this->axis[1], this->axis[2], this->frequency);
}



} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::WaypointMode, autopilot::Mode)