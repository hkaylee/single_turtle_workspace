/*
 * @Author: Ryoma Liu -- ROBOLAND 
 * @Date: 2021-11-27 15:44:32 
 * @Last Modified by: Ryoma Liu
 * @Last Modified time: 2022-02-02 17:54:19
 */

#ifndef PID_SPEED_CONTROLLER_H_
#define PID_SPEED_CONTROLLER_H_

#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include "controller/basecontroller.h"
#include "proxy/control_data.h"
#include "rclcpp/rclcpp.hpp"

namespace turtle_namespace{
namespace control{
class PID_speed_controller : public BaseController
{

public:
    PID_speed_controller() = default;

    virtual ~PID_speed_controller() = default;

    void Init() override;

    void ComputeControlCommand(turtle &) override;

    void Reset() override;

    void Stop() override;

private:
    bool is_init = false;
    float t = 0;
    float theta1;
    float gamma1;
    float beta1;
    float theta2;
    float gamma2;
    float beta2;
    float kp = 10;
    float ki = 0;
    float kd = 2;
    float current_theta;
    float current_gamma;
    float target_speed = 2;
    float theta_error;
    float gamma_error;
    float theta_last_error;
    float gamma_last_error;
    float output;
    float integral_theta;
    float integral_gamma;

};
} // namespace control
} // namespace turtle_namespace

#endif