#ifndef ROBOMASTER_nakamura_hand_HPP
#define ROBOMASTER_nakamura_hand_HPP
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
        DigitalIn* lim;
        PID pid;
        double now_angle;
        int16_t current;
        const int ring_hold_angle = 2500;
        const int scaffold_hold_angle = 2500;
        const int ring_leave_angle = 500;
        const int scaffold_leave_angle = 1500;
        const int servo_period = 20;
        const double tar_angle = -1800.0;
        const int CURRENT_MAX = 8000;
    public:
        Hand(PwmOut* ring, PwmOut* scaffold, robomaster::Robomaster_ESC* esc, DigitalIn* lim, 
                   double kp = 2.5, double ki = 0.25, double kd = 0.0002)
            : ring(ring), scaffold(scaffold), esc(esc), lim(lim), pid(kp, ki, kd) {
                ring->period_ms(servo_period);
                scaffold->period_ms(servo_period);
            }

        void update(){
            now_angle = esc->get_continuous_angle() / 36.0;
            pid.Input(tar_angle, now_angle);
            current = static_cast<int16_t>(pid.Output());

            if(abs(current) > CURRENT_MAX){
                current = current > 0 ? CURRENT_MAX : -CURRENT_MAX;
            }

            if(!lim->read() && current > 0){
                current = 0;
            }
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
            esc->set_current(current);
        }

        void down(){
            esc->set_current(0);
        }

        double get_now_angle(){
            return now_angle;
        }

        int16_t get_current(){
            return esc->get_current();
        }
};

// main() runs in its own thread in the OS
int main()
{
    Serial pc(USBTX, USBRX, 115200);

    const int bitrate = 1000000;
    const int id = 0;
    robomaster::Robomaster_Array array(PB_5, PB_6, bitrate);
    robomaster::Robomaster_ESC esc(id);
    array.add_ESC(&esc);

    PwmOut servo[2]{
        {(PB_10)},  //ring
        {(PB_2)}    //scaffold
    };

    DigitalIn lim(PA_9,PullUp);

    Hand arm(&servo[0], &servo[1], &esc, &lim);

    Timer timer;
    timer.start();
    double t = timer.read();

    while (true) {
        arm.update();

        // arm.up();
        // arm.down();

        // arm.ring_hold();
        // arm.scaffold_hold();

        // arm.ring_leave();
        // arm.scaffold_leave();

        array.send();

        while(timer.read() - t < dt);
        t = timer.read();
    }
}
#endif
