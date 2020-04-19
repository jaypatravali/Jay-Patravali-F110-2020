#include <iostream>
#include <math.h>


// pi
const double PI = 3.1415;


void update(const CarState start, double accel_ ,double steer_angle_vel_, CarParams p,
        double dt)
{


    double thresh = .5; // cut off to avoid singular behavior
    double err = .03; // deadband to avoid flip flop
    if (!start.st_dyn)
        thresh += err;

    // if velocity is low or negative, use normal Kinematic Single Track dynamics
    if (start.velocity < thresh) {
        return update_k(
                    start,
                    accel,
                    steer_angle_vel,
                    p,
                    dt);
    }


    CarState end;

    double g = 9.81; // m/s^2

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta + start.slip_angle);
    double y_dot = start.velocity * std::sin(start.theta + start.slip_angle);
    double v_dot = accel;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.angular_velocity;

    // for eases of next two calculations
    double rear_val = g * p.l_r - accel * p.h_cg;
    double front_val = g * p.l_f + accel * p.h_cg;

    // in case velocity is 0
    double vel_ratio, first_term;
    if (start.velocity == 0) {
        vel_ratio = 0;
        first_term = 0;
    }
    else {
        vel_ratio = start.angular_velocity / start.velocity;
        first_term = p.friction_coeff / (start.velocity * (p.l_r + p.l_f));
    }

    double theta_double_dot = (p.friction_coeff * p.mass / (p.I_z * p.wheelbase)) *
            (p.l_f * p.cs_f * start.steer_angle * (rear_val) +
             start.slip_angle * (p.l_r * p.cs_r * (front_val) - p.l_f * p.cs_f * (rear_val)) -
             vel_ratio * (std::pow(p.l_f, 2) * p.cs_f * (rear_val) + std::pow(p.l_r, 2) * p.cs_r * (front_val)));\

    double slip_angle_dot = (first_term) *
            (p.cs_f * start.steer_angle * (rear_val) -
             start.slip_angle * (p.cs_r * (front_val) + p.cs_f * (rear_val)) +
             vel_ratio * (p.cs_r * p.l_r * (front_val) - p.cs_f * p.l_f * (rear_val))) -
            start.angular_velocity;


    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = start.velocity + v_dot * dt;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = start.angular_velocity + theta_double_dot * dt;
    end.slip_angle = start.slip_angle + slip_angle_dot * dt;
    end.st_dyn = true;

}


CarState STKinematics::update_k(
        const CarState start,
        double accel,
        double steer_angle_vel,
        CarParams p,
        double dt) {

    CarState end;

    // compute first derivatives of state
    double x_dot = start.velocity * std::cos(start.theta);
    double y_dot = start.velocity * std::sin(start.theta);
    double v_dot = accel;
    double steer_angle_dot = steer_angle_vel;
    double theta_dot = start.velocity / p.wheelbase * std::tan(start.steer_angle);
    double theta_double_dot = accel / p.wheelbase * std::tan(start.steer_angle) +
            start.velocity * steer_angle_vel / (p.wheelbase * std::pow(std::cos(start.steer_angle), 2));
    double slip_angle_dot = 0;

    // update state
    end.x = start.x + x_dot * dt;
    end.y = start.y + y_dot * dt;
    end.theta = start.theta + theta_dot * dt;
    end.velocity = start.velocity + v_dot * dt;
    end.steer_angle = start.steer_angle + steer_angle_dot * dt;
    end.angular_velocity = 0; //start.angular_velocity + theta_double_dot * dt;
    end.slip_angle = 0; //start.slip_angle + slip_angle_dot * dt;
    end.st_dyn = false;


    return end;

}



void update_pose(const CarState start, double accel_ ,double steer_angle_vel_, CarParams p,
        double dt) {

    state = update(
        state,
        accel,
        steer_angle_vel,
        params,
        dt);
    state.velocity = std::min(std::max(state.velocity, -max_speed), max_speed);
    state.steer_angle = std::min(std::max(state.steer_angle, -max_steering_angle), max_steering_angle);

} 

 

void set_accel(double accel_) {
    accel = std::min(std::max(accel_, -max_accel), max_accel);
}

void set_steer_angle_vel(double steer_angle_vel_) {
    steer_angle_vel = std::min(std::max(steer_angle_vel_, -max_steering_vel), max_steering_vel);
}



double compute_steer_vel(double desired_angle) {
    // get difference between current and desired
    double dif = (desired_angle - state.steer_angle);

    // calculate velocity
    double steer_vel;
    if (std::abs(dif) > .0001)  // if the difference is not trivial
        steer_vel = dif / std::abs(dif) * max_steering_vel;
    else {
        steer_vel = 0;
    }

    return steer_vel;
}

void compute_accel(double desired_velocity) {
    // get difference between current and desired
    double dif = (desired_velocity - state.velocity);

    if (state.velocity > 0) {
        if (dif > 0) {
            // accelerate
            double kp = 2.0 * max_accel / max_speed;
            set_accel(kp * dif);
        } else {
            // brake
            accel = -max_decel; 
        }    
    } else {
        if (dif > 0) {
            // brake
            accel = max_decel;

        } else {
            // accelerate
            double kp = 2.0 * max_accel / max_speed;
            set_accel(kp * dif);
        }   
    }
}


// Initialize car state and driving commands
state = {.x=0, .y=0, .theta=0, .velocity=0, .steer_angle=0.0, .angular_velocity=0.0, .slip_angle=0.0, .st_dyn=false};
accel = 0.0;
steer_angle_vel = 0.0;
desired_speed = 0.0;
desired_steer_ang = 0.0;


struct CarState {
    double x; // x position
    double y; // y position
    double theta; // orientation
    double velocity;
    double steer_angle;
    double angular_velocity;
    double slip_angle;
    bool st_dyn;
};


int main(int argc, char ** argv) 
{
    desired_speed, desired_steer_ang, dt = argv;
    accel =  compute_accel(desired_speed);
    steering_vel = set_steer_angle_vel(compute_steer_vel(desired_steer_ang));
    update_pose(state, accel, steering_vel, dt)
    return 0;
}
