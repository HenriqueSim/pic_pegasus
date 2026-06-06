#pragma once

// =====================================================================
//  ChirpIdentification — single-axis frequency-sweep (chirp) excitation
//  for inertia / rotational-dynamics identification.
//
//  Built to sit alongside your existing SystemIdentification mode and to
//  reuse the same plugin/service/controller pattern.
//
//  >>> VERIFY: mirror the includes of your existing mode_systemID.hpp.
//      The base class autopilot::Mode and the State / VehicleConstants
//      types come from the autopilot core header your SystemID mode uses
//      (path shown below is the usual Pegasus location — confirm it).
// =====================================================================
#include "autopilot/mode.hpp"          // <-- VERIFY this path (same base header as mode_systemID.hpp)

#include <Eigen/Dense>
#include <geometry_msgs/msg/vector3.hpp>
#include "pegasus_utils/frames.hpp"
#include <pegasus_utils/rotations.hpp>
#include "pegasus_msgs/srv/system_id.hpp"   // reuse the SystemID service (axis[3], amplitude, frequency)

namespace autopilot {

class ChirpIdentification : public Mode {

public:
    ~ChirpIdentification();

    void initialize() override;
    bool enter() override;
    bool exit() override;
    void update(double dt) override;

private:
    // --- chirp helpers (tau = time since the sweep started) ---
    double chirp_frequency(double tau) const;   // instantaneous frequency [Hz]
    double chirp_phase(double tau) const;        // integrated phase [rad]
    double altitude_hold_thrust(const State& state, double roll_rad, double pitch_rad) const;
    void   publish_debug(double roll_deg, double pitch_deg, double freq_hz);

    // --- service ---
    void chirp_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request,
                        const pegasus_msgs::srv::SystemID::Response::SharedPtr response);
    rclcpp::Service<pegasus_msgs::srv::SystemID>::SharedPtr chirp_service_;

    // --- debug publishers ---
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr attitude_cmd_pub_;  // commanded (roll,pitch,0) [deg]
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr chirp_state_pub_;    // (freq_hz, amplitude_deg, t)

    // --- state ---
    double mass_{1.0};
    double t_{0.0};
    bool   settings_set_{false};

    // --- excitation configuration ---
    Eigen::Vector3d axis_{1.0, 0.0, 0.0};  // (roll, pitch, -) selector: (1,0,0)=roll, (0,1,0)=pitch
    double amplitude_deg_{10.0};           // attitude amplitude [deg]
    double f_start_{0.5};                  // sweep start frequency [Hz]
    double f_end_{4.0};                    // sweep end frequency [Hz]
    double sweep_duration_{25.0};          // sweep length [s]
    double t_hover_{3.0};                  // settle/hover time before the sweep [s]
    double hover_height_{1.5};             // hover altitude [m] (NED target z = -hover_height_)
    double kp_z_{6.0};                     // altitude-hold proportional gain
    double kd_z_{4.0};                     // altitude-hold derivative gain
    double max_tilt_deg_{35.0};            // tilt clamp for thrust compensation
    bool   use_log_chirp_{true};           // true = logarithmic sweep, false = linear
};

} // namespace autopilot