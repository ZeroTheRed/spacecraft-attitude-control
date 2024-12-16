#include <iostream>
#include <fstream>
#include <cstdlib>
#include <stdlib.h>
#include <vector>
#include <math.h>
#include <chrono>
#include <thread>
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <iomanip>

class AttitudeController {
private:
    Eigen::Vector3f integral_error;
    Eigen::Vector3f previous_error;
    float Kp, Ki, Kd;
    float max_integral;
    float max_torque;
    float dt;

    // Enforce a maximum limit on the control torque
    Eigen::Vector3f saturate(const Eigen::Vector3f& v, float limit) {
        if (v.norm() > limit) {
            return v.normalized() * limit;
        }
        return v;
    }

    Eigen::Vector3f unwrap_error(const Eigen::Vector3f& error) {
        return error.array() - 2 * M_PI * (error.array() / (2 * M_PI)).round();
    }

public:
    AttitudeController(float kp, float ki, float kd, float max_i, float max_t, float delta_t)
        : Kp(kp), Ki(ki), Kd(kd), max_integral(max_i), max_torque(max_t), dt(delta_t) {
        integral_error.setZero();
        previous_error.setZero();
    }

    Eigen::Vector3f PID_controller(const Eigen::Quaternionf& q, const Eigen::Quaternionf& q_desired, const Eigen::Vector3f& omega) {
        Eigen::Quaternionf q_error = q_desired * q.inverse();
        Eigen::Vector3f error = unwrap_error(q_error.vec() * 2.0f * std::copysign(1.0f, q_error.w()));

        // Adaptive gains based on error magnitude
        float error_mag = error.norm();
        float adaptive_Kp = Kp * (1 + 10 * std::exp(-0.5 * -error_mag));
        float adaptive_Ki = Ki * std::exp(-0.5 * -error_mag);
        float adaptive_Kd = Kd * (1 + 5 * std::exp(-0.5 * -error_mag));

        // PID control with anti-windup
        Eigen::Vector3f p_term = adaptive_Kp * error;
        integral_error += error * dt;
        integral_error = saturate(integral_error, max_integral);
        Eigen::Vector3f i_term = adaptive_Ki * integral_error;
        Eigen::Vector3f d_term = adaptive_Kd * (error - previous_error) / dt;

        Eigen::Vector3f nonlinear_term = 5.0 * error.normalized() * std::tanh(error.norm());

        Eigen::Vector3f control = p_term + i_term + d_term - adaptive_Kd * omega;
        previous_error = error;

        return saturate(control, max_torque);
    }
};

// Moment of inertia of spacecraft
Eigen::Matrix3f I = (Eigen::Matrix3f() << 7.0, 0.0, 0.0, 0.0, 8.0, 0.0, 0.0, 0.0, 9.0).finished();

// External torques 
Eigen::Vector3f disturbance_torque(0.0, 0.0, 0.0);

Eigen::Vector3f update_angular_velocity(const Eigen::Vector3f& omega, const Eigen::Vector3f& torque, float dt) {
    Eigen::Vector3f omega_dot = I.inverse() * (torque - omega.cross(I * omega));
    return omega + omega_dot*dt;
}

Eigen::Quaternionf update_orientation(Eigen::Quaternionf& q, const Eigen::Vector3f& omega, float dt) {
    Eigen::Quaternionf omega_quat(0, omega(0)*0.5, omega(1)*0.5, omega(2)*0.5);

    Eigen::Quaternionf q_dot = q * omega_quat;

    Eigen::Quaternionf q_new = Eigen::Quaternionf(
        q.w() + q_dot.w() * dt,
        q.x() + q_dot.x() * dt,
        q.y() + q_dot.y() * dt,
        q.z() + q_dot.z() * dt
    ).normalized();

    return q_new;
}

int main() {
    std::vector<float> qd_w, qd_x, qd_y, qd_z, q_w, q_x, q_y, q_z;

    // Initial angular velocity
    Eigen::Vector3f omega(0.1, 0.1, 0.1);  

    // Initial and desired angle
    float angle = 0.0 * M_PI/180.0f, angle_desired = 90.0 * M_PI/180.0f;
    Eigen::Vector3f axis_initial(1, 0, 0), axis_desired(0, 1, 0);

    // 1 + 0i + 0j + 0k <- Initial orientation in quaternion form
    Eigen::Quaternionf orientation(Eigen::AngleAxisf(angle, axis_initial));

    // 0.707 + 0.707j <- Desired quaternion for 90°
    Eigen::Quaternionf desired_orientation(Eigen::AngleAxisf(angle_desired, axis_desired));

    float dt = 0.01;  
    int steps = 1000;  

    std::cout << "Initial Orientation: " << orientation.coeffs().transpose() << std::endl;
    std::cout << "Desired Orientation: " << desired_orientation.coeffs().transpose() << std::endl;

    AttitudeController controller(50.0, 0.1, 100.0, 5.0, 10.0, dt);

    std::ofstream output_file("quaternion_data.dat", std::ofstream::out | std::ofstream::trunc);

    for (int i = 0; i < steps; ++i) {
        Eigen::Vector3f control_torque = controller.PID_controller(orientation, desired_orientation, omega);
        Eigen::Vector3f net_torque = disturbance_torque + control_torque;

        // Update angular velocity and orientation based on external torques
        omega = update_angular_velocity(omega, net_torque, dt);
        orientation = update_orientation(orientation, omega, dt);

        // Extract error as angle for logging
        Eigen::Quaternionf quat_error = desired_orientation * orientation.inverse(); 
        float error_angle = 2 * acos(abs(quat_error.w())) * 180.0 / M_PI;

        // std::cout << "Step: " << std::setw(3) << i 
        //           << " | Orient: " << std::setprecision(4) << orientation.coeffs().transpose()
        //           << " | Omega: " << std::setprecision(4) << omega.transpose()
        //           << " | Error (deg): " << std::setprecision(4) << error_angle << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10));

        qd_w.push_back(desired_orientation.w());
        qd_x.push_back(desired_orientation.x());
        qd_y.push_back(desired_orientation.y());
        qd_z.push_back(desired_orientation.z());

        q_w.push_back(orientation.w());
        q_x.push_back(orientation.x());
        q_y.push_back(orientation.y());
        q_z.push_back(orientation.z());

        output_file << i << " "
                    << desired_orientation.w() << " "
                    << desired_orientation.x() << " "
                    << desired_orientation.y() << " "
                    << desired_orientation.z() << " "
                    << orientation.w() << " "
                    << orientation.x() << " "
                    << orientation.y() << " "
                    << orientation.z() << " " << std::endl;

        if (error_angle < 1.0f) {
            std::cout << "Reached desired orientation within 1° tolerance" << std::endl;
            break;
        }
    }

    output_file.close();
    system("gnuplot -persist plot_script.gp");
    
    return 0;
}