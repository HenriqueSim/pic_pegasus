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

    // Get the mass of the vehicle (used to get the thrust from the acceleration)
    VehicleConstants vehicle_constansts = get_vehicle_constants();
    mass_ = vehicle_constansts.mass;
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

    // Get the current state of the vehicle
    State state = this->get_vehicle_state();

    // Calculate the position error
    Eigen::Vector3d position_error = this->target_pos - state.position;

    // Calculate the velocity error using the previous position error
    Eigen::Vector3d velocity_error = (position_error - this->prev_pos_error_) / dt;

    //Update the previous position error
    this->prev_pos_error_ = position_error;
    // Compute the desired control output acceleration for each controller
    Eigen::Vector3d u;
    const Eigen::Vector3d g(0.0, 0.0, 9.81);
    for(unsigned int i=0; i < 3; i++) u[i] = compute_output(position_error[i], velocity_error[i], 0.0, dt, i); // (acceleration[i] - g[i])* mass_
    
    u[2] = u[2] - g(2);

    // Convert the acceleration to attitude and thrust
    Eigen::Vector4d attitude_thrust = get_attitude_thrust_from_acceleration(u, mass_, Pegasus::Rotations::deg_to_rad(this->target_yaw));

    // Set the control output
    Eigen::Vector3d attitude_target = Eigen::Vector3d(
        Pegasus::Rotations::rad_to_deg(attitude_thrust[0]),
        Pegasus::Rotations::rad_to_deg(attitude_thrust[1]),
        Pegasus::Rotations::rad_to_deg(attitude_thrust[2]));

    // Send the attitude and thrust to the attitude controller
    this->controller_->set_attitude(attitude_target, attitude_thrust[3]);

    // // Update and publish the PID statistics
    // update_statistics(position);
    // statistics_pub_->publish(pid_statistics_msg_);
}

Eigen::Vector4d WaypointMode::get_attitude_thrust_from_acceleration(const Eigen::Vector3d & u, double mass, double yaw) {

    Eigen::Matrix3d RzT;
    Eigen::Vector3d r3d;
    Eigen::Vector4d attitude_thrust;

    /* Compute the normalized thrust and r3d vector */
    double T = mass * u.norm();

    /* Compute the rotation matrix about the Z-axis */
    RzT << cos(yaw), sin(yaw), 0.0,
          -sin(yaw), cos(yaw), 0.0,
                0.0,      0.0, 1.0;

    /* Compute the normalized rotation */
    r3d = -RzT * u / u.norm();

    // Compute the actual attitude and setup the desired thrust to apply to the vehicle
    attitude_thrust << asin(-r3d[1]), atan2(r3d[0], r3d[2]), yaw, T;
    return attitude_thrust;
}

double WaypointMode::compute_output(double error_p, double error_d, double feed_forward_ref, double dt, unsigned int i) {

    // Compute the PID terms
    double p_term = kp_[i] * error_p;
    double d_term = kd_[i] * error_d;
    double ff_term = kff_[i] * feed_forward_ref;

    // Compute the output and saturate it
    double output = p_term + d_term + ff_term;      //add the integral term later
    double saturated_ouput = std::max(min_output_, std::min(output, max_output_));

    // // Update the statistics structure used for extracting the performance of the control loop
    // stats_.dt = dt;
    // stats_.error_p = error_p;
    // stats_.error_d = error_d;
    // stats_.integral = error_i_;
    // stats_.ff_ref = feed_forward_ref;
    // stats_.p_term = p_term;
    // stats_.d_term = d_term;
    // stats_.i_term = i_term;
    // stats_.ff_term = ff_term;
    // stats_.output_pre_sat = output;
    // stats_.output = saturated_ouput;

    return saturated_ouput;
}


void WaypointMode::waypoint_callback(const pegasus_msgs::srv::Waypoint::Request::SharedPtr request, const pegasus_msgs::srv::Waypoint::Response::SharedPtr response) {
    
    // Set the waypoint
    this->target_pos[0] = request->position[0];
    this->target_pos[1] = request->position[1];
    this->target_pos[2] = request->position[2];
    this->target_yaw = request->yaw;

    // Set the waypoint flag
    this->waypoint_set_ = true;

    // Return true to indicate that the waypoint has been set successfully
    response->success = true;
    RCLCPP_WARN(this->node_->get_logger(), "Waypoint set to (%f, %f, %f) with yaw %f", this->target_pos[0], this->target_pos[1], this->target_pos[2], this->target_yaw);
}

// void WaypointMode::update_statistics(const Eigen::Vector3d & position_ref) {
    
//     // For each PID control [x, y, z]
//     for(unsigned int i = 0; i < 3; i++) {

//         // Get the statistics from the controller object
//         Pegasus::Pid::Statistics stats = controllers_[i]->get_statistics();

//         pid_statistics_msg_.statistics[i].dt = stats.dt;
//         pid_statistics_msg_.statistics[i].reference = position_ref[i];
//         // Fill the feedback errors
//         pid_statistics_msg_.statistics[i].error_p = stats.error_p;
//         pid_statistics_msg_.statistics[i].error_d = stats.error_d;
//         pid_statistics_msg_.statistics[i].integral = stats.integral;
//         pid_statistics_msg_.statistics[i].ff_ref = stats.ff_ref;

//         // Fill the errors scaled by the gains
//         pid_statistics_msg_.statistics[i].p_term = stats.p_term;
//         pid_statistics_msg_.statistics[i].d_term = stats.d_term;
//         pid_statistics_msg_.statistics[i].i_term = stats.i_term;
//         pid_statistics_msg_.statistics[i].ff_term = stats.ff_term;

//         // Fill the outputs of the controller
//         pid_statistics_msg_.statistics[i].anti_windup_discharge = stats.anti_windup_discharge;
//         pid_statistics_msg_.statistics[i].output_pre_sat = stats.output_pre_sat;
//         pid_statistics_msg_.statistics[i].output = stats.output;
//     }
// }

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::WaypointMode, autopilot::Mode)