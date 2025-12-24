#ifndef ARRC_nakamura_hand_H_
#define ARRC_nakamura_hand_H_

#include "PwmOut.h"
#include "mbed.h"
#include "motor.hpp"
#include "config.hpp"
#include "robomaster_can.hpp"
#include "pid.hpp"

class Hand {
    private:
        PwmOut* ring;
        PwmOut* scaffold;
        robomaster::Robomaster_ESC* esc;
        DigitalIn* lim_forward;
        DigitalIn* lim_backward;
        PID pid;

        bool rotate_direction;

        double now_angle;       // [rad]
        double tar_omega;
        double now_omega;       // [rad/s]

        int16_t current;
        const int16_t default_current = 3000;
        const int16_t CURRENT_MAX = 8000;

        const double robomas_gear_ratio = 36.0;
        const double robomas_pulse_per_rev = 8192.0;
        const double gear_ratio = 2.5; // 手首のギア比
        const double angle_coeff = 0.9;

        const double tar_omega_max = 360.0; // [rad/s]
        const double tar_omega_coeff = 3;
        const double up_angle = 120.0;
        const double down_angle = 0;

        const int ring_hold_angle = 2500;
        const int scaffold_hold_angle = 1500;
        const int ring_leave_angle = 500;
        const int scaffold_leave_angle = 500;
        
        const int servo_period = 20;

        void robomas_move(double tar_angle, bool direction){    //direction   true : up  false : down
            tar_omega = (tar_angle - now_angle) * tar_omega_coeff; //目標角速度
            if(abs(tar_omega) > tar_omega_max){
                tar_omega = tar_omega_max * (tar_omega > 0 ? 1 : -1);
            }

            pid.Input(tar_omega, now_omega);
            current = static_cast<int16_t>(pid.Output()) + default_current * cos((now_angle * M_PI / 180.0) * angle_coeff) * (rotate_direction ? -1 : 1);

            // if(direction){
            //     pid_up.Input(tar_angle, now_angle);
            //     current = static_cast<int16_t>(pid_up.Output());
            // } else {
            //     pid_down.Input(tar_angle, now_angle);
            //     current = static_cast<int16_t>(pid_down.Output());
            // }

            if(abs(current) > CURRENT_MAX){
                current = current > 0 ? CURRENT_MAX : -CURRENT_MAX;
            }

            if(!lim_forward->read() && (rotate_direction ? current > 0 : current < 0)){
                current = 0;
            }

            if(!lim_backward->read() && (rotate_direction ? current < 0 : current > 0)){
                current = 0;
            }

            esc->set_current(current);
        }

    public:
        Hand(PwmOut* ring, PwmOut* scaffold, robomaster::Robomaster_ESC* esc, DigitalIn* lim_forward, DigitalIn* lim_backward, bool rotate_direction, double kp = 11.0, double ki = 0.1, double kd = 0)
            : ring(ring), scaffold(scaffold), esc(esc), lim_forward(lim_forward), lim_backward(lim_backward), rotate_direction(rotate_direction), pid(kp, ki, kd) {
                ring->period_ms(servo_period);
                scaffold->period_ms(servo_period);
            }

        void update(){
            now_angle = (esc->get_continuous_angle() * 360.0) / (robomas_pulse_per_rev * robomas_gear_ratio) / gear_ratio;
            now_omega = esc->get_rpm() * 360.0 / (60.0 * 36.0);
        }
    
        void ring_hold() {
            ring->pulsewidth_us(ring_hold_angle);
        }

        void scaffold_hold() {
            scaffold->pulsewidth_us(scaffold_hold_angle);
        }

        void ring_leave() {
            ring->pulsewidth_us(ring_leave_angle);
        }

        void scaffold_leave() {
            scaffold->pulsewidth_us(scaffold_leave_angle);
        }

        void up(){
            robomas_move(rotate_direction ? -up_angle : up_angle, true);
        }

        void down(){
            robomas_move(down_angle, false);
        }

        double get_now_angle(){
            return now_angle;
        }

        double get_now_omega(){
            return now_omega;
        }

        double get_tar_omega(){
            return tar_omega;
        }

        int16_t get_current(){
            return esc->get_current();
        }

        int16_t get_program_current(){
            return current;
        }
};

#endif
