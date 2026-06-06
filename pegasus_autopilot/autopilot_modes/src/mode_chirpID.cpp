#include "autopilot_modes/mode_chirp_id.hpp"   // <-- VERIFY path matches where you place the header
#include <pegasus_utils/rotations.hpp>
#include <algorithm>
#include <cmath>

namespace autopilot {

static constexpr double RAD2DEG = 180.0 / M_PI;
static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double G       = 9.81;

ChirpIdentification::~ChirpIdentification() {
    this->chirp_service_.reset();
}

void ChirpIdentification::initialize() {

    // Trigger/configuration service. Reuses the SystemID srv (axis[3], amplitude, frequency).
    // NOTE: if the SystemIdentification mode is also loaded, give this a DISTINCT name to avoid a
    // service-name clash (e.g. "chirp/set_settings") and match it in the mission file.
    node_->declare_parameter<std::string>("autopilot.ChirpIdentification.set_settings_service", "set_settings");
    chirp_service_ = node_->create_service<pegasus_msgs::srv::SystemID>(
        node_->get_parameter("autopilot.ChirpIdentification.set_settings_service").as_string(),
        std::bind(&ChirpIdentification::chirp_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Sweep parameters not carried by the srv (f_end and duration) are exposed as ROS parameters.
    node_->declare_parameter<double>("autopilot.ChirpIdentification.f_start_hz", 0.5);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.f_end_hz", 2.0);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.sweep_duration_s", 25.0);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.hover_time_s", 3.0);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.hover_height_m", 1.5);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.amplitude_deg", 10.0);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.kp_z", 7.0);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.kd_z", 4.0);
    node_->declare_parameter<double>("autopilot.ChirpIdentification.max_tilt_deg", 80.0);
    node_->declare_parameter<bool>("autopilot.ChirpIdentification.use_log_chirp", true);

    f_start_        = node_->get_parameter("autopilot.ChirpIdentification.f_start_hz").as_double();
    f_end_          = node_->get_parameter("autopilot.ChirpIdentification.f_end_hz").as_double();
    sweep_duration_ = node_->get_parameter("autopilot.ChirpIdentification.sweep_duration_s").as_double();
    t_hover_        = node_->get_parameter("autopilot.ChirpIdentification.hover_time_s").as_double();
    hover_height_   = node_->get_parameter("autopilot.ChirpIdentification.hover_height_m").as_double();
    amplitude_deg_  = node_->get_parameter("autopilot.ChirpIdentification.amplitude_deg").as_double();
    kp_z_           = node_->get_parameter("autopilot.ChirpIdentification.kp_z").as_double();
    kd_z_           = node_->get_parameter("autopilot.ChirpIdentification.kd_z").as_double();
    max_tilt_deg_   = node_->get_parameter("autopilot.ChirpIdentification.max_tilt_deg").as_double();
    use_log_chirp_  = node_->get_parameter("autopilot.ChirpIdentification.use_log_chirp").as_bool();

    attitude_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Vector3>("/chirp_id/debug/attitude_cmd", 10);
    chirp_state_pub_  = node_->create_publisher<geometry_msgs::msg::Vector3>("/chirp_id/debug/chirp_state", 10);

    VehicleConstants vc = get_vehicle_constants();
    mass_ = vc.mass;

    RCLCPP_INFO(node_->get_logger(), "ChirpIdentification initialized");
}

bool ChirpIdentification::enter() {
    if (!settings_set_) {
        RCLCPP_ERROR(node_->get_logger(), "Settings not set - cannot enter ChirpIdentification mode.");
        return false;
    }
    settings_set_ = false;   // require fresh settings before re-entering
    t_ = 0.0;
    return true;
}

bool ChirpIdentification::exit() {
    return true;
}


// Tilt-compensated altitude-hold thrust. Same sign convention as the SystemID mode (NED).
double ChirpIdentification::altitude_hold_thrust(const State& state, double roll_rad, double pitch_rad) const {
    const double target_z = -hover_height_;                 // NED target altitude
    const double e_z  = state.position.z() - target_z;      // >0 if below target -> add thrust
    const double e_vz = state.velocity.z() - 0.0;           // damping on vertical velocity
    double c = std::cos(roll_rad) * std::cos(pitch_rad);    // vertical thrust fraction
    c = std::max(c, std::cos(max_tilt_deg_ * DEG2RAD));     // clamp so thrust does not blow up at large tilt
    const double T = (kp_z_ * e_z + kd_z_ * e_vz + G) * mass_ / c;
    return std::max(T, 0.0);
}

void ChirpIdentification::update(double dt) {
    State state = get_vehicle_state();

    if (t_ < t_hover_) {
        this->controller_->set_position({state.position.x(), state.position.y(), -hover_height_}, 90.0, dt);
    }
    else if (t_ < t_hover_ + sweep_duration_) {
        const double tau = t_ - t_hover_;
        const double omega = 2.0 * M_PI * f_start_;

        const double roll_rate_deg_s  = axis_[0] * (amplitude_deg_ * omega * std::cos(omega * tau));
        const double pitch_rate_deg_s = axis_[1] * (amplitude_deg_ * omega * std::cos(omega * tau));

        Eigen::Vector3d attitude_rate_deg(roll_rate_deg_s, pitch_rate_deg_s, 0.0);

        Eigen::Vector3d current_euler = Pegasus::Rotations::quaternion_to_euler(state.attitude);
        const double T = altitude_hold_thrust(state, current_euler(0), current_euler(1));

        this->controller_->set_attitude_rate(attitude_rate_deg, T, dt);
    }
    else {
        this->controller_->set_position({state.position.x(), state.position.y(), -hover_height_}, 90.0, dt);
    }

    t_ += dt;
}

void ChirpIdentification::publish_debug(double roll_deg, double pitch_deg, double freq_hz) {
    geometry_msgs::msg::Vector3 att, st;
    att.x = roll_deg;          // commanded roll  [deg]
    att.y = pitch_deg;         // commanded pitch [deg]
    att.z = 0.0;               // yaw fixed
    st.x  = freq_hz;           // instantaneous excitation frequency [Hz] (use to segment the bag)
    st.y  = amplitude_deg_;    // commanded amplitude [deg]
    st.z  = t_;                // mode time [s]
    attitude_cmd_pub_->publish(att);
    chirp_state_pub_->publish(st);
}

void ChirpIdentification::chirp_callback(const pegasus_msgs::srv::SystemID::Request::SharedPtr request,
                                         const pegasus_msgs::srv::SystemID::Response::SharedPtr response) {
    // axis selects the excitation axis: (1,0,0) = roll, (0,1,0) = pitch.
    axis_ = Eigen::Vector3d(request->axis[0], request->axis[1], request->axis[2]);
    if (request->amplitude > 0.0) amplitude_deg_ = request->amplitude;  // attitude amplitude [deg]
    if (request->frequency > 0.0) f_start_       = request->frequency;  // sweep start frequency [Hz]
    t_ = 0.0;
    settings_set_ = true;

    response->success = true;
    RCLCPP_WARN(node_->get_logger(),
        "Chirp set: axis=(%.0f,%.0f,%.0f), amp=%.1f deg, f=%.2f -> %.2f Hz over %.1f s",
        axis_[0], axis_[1], axis_[2], amplitude_deg_, f_start_, f_end_, sweep_duration_);
}

} // namespace autopilot

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(autopilot::ChirpIdentification, autopilot::Mode)