#include "myrobot.h"

#include <iostream>
#include <fstream>
using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    base_actuation = false;

    // set use_joystick as true if you want to command robot with joystick
    use_joystick = true;
    max_stride = 0.085;
    max_sway   = 0.085;
    max_turn   = 0.05;

    stairSwitch = false;
    stairTime   = 0.0;
    dstairTime  = 0.0;
    staircount  = 0;

    jumpSwitch = false;
    jumpTime = 0.0;
    djumpTime = 0.0;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 50.0;
	param.com_height =  0.70;
	param.gravity    =  9.8;
    
    // kinematic parameters
    param.base_to_shoulder[0] = Vector3(0.0, -0.1,  0.3);
    param.base_to_shoulder[1] = Vector3(0.0,  0.1,  0.3);
    param.base_to_hip     [0] = Vector3(0.0, -0.1, -0.1);
    param.base_to_hip     [1] = Vector3(0.0,  0.1, -0.1);
    param.wrist_to_hand   [0] = Vector3(0.0,  0.0, -0.1);
    param.wrist_to_hand   [1] = Vector3(0.0,  0.0, -0.1);
    param.ankle_to_foot   [0] = Vector3(0.0,  0.0, -0.05);
    param.ankle_to_foot   [1] = Vector3(0.0,  0.0, -0.05);
    param.arm_joint_index [0] =  4;
    param.arm_joint_index [1] = 11;
    param.leg_joint_index [0] = 18;
    param.leg_joint_index [1] = 24;
    param.upper_arm_length = 0.2;
    param.lower_arm_length = 0.2;
    param.upper_leg_length = 0.3;
    param.lower_leg_length = 0.4;

    param.trunk_mass = 24.0;
    param.trunk_com = Vector3(0.0, 0.0, 0.166);

    param.arm_mass[0] = 0.5;
    param.arm_mass[1] = 0.5;
    param.arm_mass[2] = 1.0;
    param.arm_mass[3] = 0.5;
    param.arm_mass[4] = 1.0;
    param.arm_mass[5] = 0.5;
    param.arm_mass[6] = 0.5;
    param.arm_com[0] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[1] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[2] = Vector3(0.0, 0.0, -0.1);
    param.arm_com[3] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[4] = Vector3(0.0, 0.0, -0.1);
    param.arm_com[5] = Vector3(0.0, 0.0,  0.0);
    param.arm_com[6] = Vector3(0.0, 0.0,  0.0);

    param.leg_mass[0] = 0.5;
    param.leg_mass[1] = 0.5;
    param.leg_mass[2] = 1.5;
    param.leg_mass[3] = 1.5;
    param.leg_mass[4] = 0.5;
    param.leg_mass[5] = 0.5;
    param.leg_com[0] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[1] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[2] = Vector3(0.0, 0.0, -0.15);
    param.leg_com[3] = Vector3(0.0, 0.0, -0.20);
    param.leg_com[4] = Vector3(0.0, 0.0,  0.0);
    param.leg_com[5] = Vector3(0.0, 0.0,  0.0);

    // stabilizer uses z-movement of zmp to stabilize com height
    // so certain admissible range in z-direction is needed
    param.zmp_min = Vector3(-0.1, -0.05, -0.1);
    param.zmp_max = Vector3( 0.1,  0.05,  0.1);

    param.Init();

    // two hands and two feet
    foot.resize(2);
    hand.resize(2);

    // 30 joints
    joint.resize(30);
    joint[ 0].Set(1000.0, 200.0, 100.0);
    joint[ 1].Set(1000.0, 200.0, 100.0);
    joint[ 2].Set(1000.0, 200.0, 100.0);
    joint[ 3].Set(1000.0, 200.0, 100.0);
    joint[ 4].Set(1000.0, 200.0, 100.0);
    joint[ 5].Set(1000.0, 200.0, 100.0);
    joint[ 6].Set(1000.0, 200.0, 100.0);
    joint[ 7].Set(1000.0, 200.0, 100.0);
    joint[ 8].Set(1000.0, 200.0, 100.0);
    joint[ 9].Set(1000.0, 200.0, 100.0);
    joint[10].Set(1000.0, 200.0, 100.0);
    joint[11].Set(1000.0, 200.0, 100.0);
    joint[12].Set(1000.0, 200.0, 100.0);
    joint[13].Set(1000.0, 200.0, 100.0);
    joint[14].Set(1000.0, 200.0, 100.0);
    joint[15].Set(1000.0, 200.0, 100.0);
    joint[16].Set(1000.0, 200.0, 100.0);
    joint[17].Set(1000.0, 200.0, 100.0);
    joint[18].Set(1000.0, 200.0, 100.0);
    joint[19].Set(1000.0, 200.0, 100.0);
    joint[20].Set(1000.0, 200.0, 100.0);
    joint[21].Set(1000.0, 200.0, 100.0);
    joint[22].Set(100.0, 20.0, 100.0);
    joint[23].Set(100.0, 20.0, 100.0);
    joint[24].Set(1000.0, 200.0, 100.0);
    joint[25].Set(1000.0, 200.0, 100.0);
    joint[26].Set(1000.0, 200.0, 100.0);
    joint[27].Set(1000.0, 200.0, 100.0);
    joint[28].Set(100.0, 20.0, 100.0);
    joint[29].Set(100.0, 20.0, 100.0);
    
    // init hardware (simulator interface)
	Robot::Init(io, timer, joint);

    // set initial state
    centroid.com_pos_ref = Vector3(0.0, 0.0, param.com_height);
    centroid.dcm_ref     = Vector3(0.0, 0.0, param.com_height);
    foot[0].pos_ref = Vector3(0.0, -0.2/2.0, 0.0);
    foot[1].pos_ref = Vector3(0.0,  0.2/2.0, 0.0);

    // init footsteps
    footstep.steps.push_back(Step(0.0, 0.0, 0.2, 0.0, 0.0, 0.5, 0));
    footstep.steps.push_back(Step(0.0, 0.0, 0.2, 0.0, 0.0, 0.5, 1));
    // foot placement and DCM of the initial step must be specified
    footstep.steps[0].foot_pos[0] = foot[0].pos_ref;
    footstep.steps[0].foot_pos[1] = foot[1].pos_ref;
    footstep.steps[0].dcm = centroid.dcm_ref;
    footstep_planner.Plan(param, footstep);
    footstep_planner.GenerateDCM(param, footstep);

    footstep_buffer.steps.push_back(footstep.steps[0]);
    footstep_buffer.steps.push_back(footstep.steps[1]);

    // init stepping controller
    stepping_controller.swing_height = 0.05;
    stepping_controller.swing_tilt   = 0.0;
    stepping_controller.dsp_duration = 0.05;
    
    // init stabilizer
    stabilizer.orientation_ctrl_gain_p = 100.0;
    stabilizer.orientation_ctrl_gain_d = 10.0;
    stabilizer.dcm_ctrl_gain           = 2.0;
    stabilizer.base_tilt_rate          = 2.0;
    stabilizer.base_tilt_damping_p     = 100.0;
    stabilizer.base_tilt_damping_d     = 50.0;
}

void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);

    // calc FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);

	if(timer.count % 10 == 0){
        if(use_joystick){
		    // read joystick
		    joystick.readCurrentState();

		    /* Xbox controller mapping:
			    L_STICK_H_AXIS -> L stick right
			    L_STICK_V_AXIS -> L stick down
			    R_STICK_H_AXIS -> L trigger - R trigger
			    R_STICK_V_AXIS -> R stick down
			    A_BUTTON -> A
			    B_BUTTON -> B
			    X_BUTTON -> X
			    Y_BUTTON -> Y
			    L_BUTTON -> L
			    R_BUTTON -> R
		        */
		    /*
            cout <<  joystick.getPosition(Joystick::L_STICK_H_AXIS) << " " 
			     << joystick.getPosition(Joystick::L_STICK_V_AXIS) << " " 
			     << joystick.getPosition(Joystick::R_STICK_H_AXIS) << " " 
			     << joystick.getPosition(Joystick::R_STICK_V_AXIS) << " " 
			     << joystick.getButtonState(Joystick::A_BUTTON) << " "
			     << joystick.getButtonState(Joystick::B_BUTTON) << " "
			     << joystick.getButtonState(Joystick::X_BUTTON) << " "
			     << joystick.getButtonState(Joystick::Y_BUTTON) << " "
			     << joystick.getButtonState(Joystick::L_BUTTON) << " "
			     << joystick.getButtonState(Joystick::R_BUTTON) << endl;
             */
        }
		
		// erase current footsteps
		while(footstep.steps.size() > 2)
			footstep.steps.pop_back();

        // generate footsteps
		Step step;
        step.stride     = 0.0;
        step.sway       = 0.0;
        step.climb      = 0.0;
        step.turn       = 0.0;
        step.duration   = 0.235;
        // step.duration = 0.22;
        step.spacing    = 0.2;
        // max_stride = 0.09;

        if(use_joystick){
            step.stride   = -max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
            if(joystick.getButtonState(Joystick::B_BUTTON)){
                step.stride   = step.stride / 3;
            }
            step.sway     = -max_sway  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
            step.turn     = -max_turn  *(joystick.getButtonState(Joystick::R_BUTTON) - joystick.getButtonState(Joystick::L_BUTTON));
        }
        else{
            step.stride = max_stride;
        }

        if (!stairSwitch && joystick.getButtonState(Joystick::A_BUTTON)){
            stairSwitch = true;
            stairTime   = timer.time;
        }

        if (stairSwitch){
            dstairTime      = timer.time - stairTime;
            step.duration   = 0.8;
            step.spacing    = 0.12;


            if(staircount == 0){
                // go up bridge
                if(dstairTime < 0.5 + 1.0){
                    step.duration = 0.23;
                }
                else if(dstairTime < 0.7 + 1.0){
                    step.stride = 0.3;
                    step.climb  = 0.11;
                }
                else if(dstairTime < 2.0 + 1.0){
                    step.stride = 0.0;
                    step.climb  = 0.02;
                }
                else if(dstairTime < 5.0){
                    step.duration = 0.15;
                }
                else{
                stairSwitch = false;
                staircount += 1;
                }  
            }
            else if(staircount == 1){
                // go down bridge
                if(dstairTime < 0.5 + 1.0){
                    step.duration = 0.23;
                }
                else if(dstairTime < 0.7 + 1.0){
                    step.stride = 0.3;
                    step.climb  = -0.11;
                }
                else if(dstairTime < 2.0 + 1.0){
                    step.stride = 0.0;
                    step.climb  = 0.0;
                }
                else if(dstairTime < 5.0){
                    step.duration = 0.15;
                }
                else{
                stairSwitch = false;
                staircount += 1;
                }  
            }
            else{
                if(dstairTime < 0.5 + 1.0){
                    step.duration = 0.23;
                }
                else if(dstairTime < 0.7 + 1.0){
                    step.stride = 0.3;
                    step.climb  = 0.11;
                }
                else if(dstairTime < 2.0 + 1.0){
                    step.stride = 0.0;
                    step.climb  = 0.02;
                }
                else if(dstairTime < 5.0){
                    step.duration = 0.15;
                }
                else{
                stairSwitch = false;
                staircount = 0;
                }  
            }
            printf("%d",staircount);
            
            // else if(dstairTime < 0.7 + 1.0){
            //     step.stride = 0.23;
            //     step.climb  = -0.09;
            // }
            // else if(dstairTime < 2.0 + 1.0){
            //     step.stride = 0.23;
            //     step.climb  = -0.18;
            // }
            // // stop at the lowest ground to stabilize
            // else if(dstairTime < 4.7 + 1.0){
            //     step.stride = 0.0;
            //     step.climb  = 0.0;
            // }
            // // go back to get a running start
            // else if(dstairTime < 5.3 + 1.0){
            //     step.stride     = -0.09;
            //     step.duration   = 0.5;
            // }
            // else if(dstairTime < 6.5 + 1.0){
            //     step.stride = 0.0;
            // }
            // else if(dstairTime < 6.5 + 0.8 + 1.0){
            //     step.stride = 0.15;
            //     step.duration = 0.5;
            // }
            // // go up the stairs
            // else if(dstairTime < 14.0 + 0.8 + 1.0){
            //     step.stride   = 0.238;
            //     step.climb    = 0.20;
            //     step.duration = 0.80;
            // }
            // else if(dstairTime < 15.0 + 0.8 + 1.0){
            //     step.stride   = 0.00;
            //     step.climb    = 0.00;
            // } 
            // else if(dstairTime < 17.0 + 0.8 + 1.0){
            //     step.stride = 0.20;
            //     step.duration = 0.30;
            // }
            

            
        }

        if (!jumpSwitch && joystick.getButtonState(Joystick::X_BUTTON)){
            jumpSwitch = true;
            jumpTime   = timer.time;
            com_pos_tmp_jump = centroid.com_pos_ref;
            base_yaw_jump = base.angle_ref.z();
            printf("button");
        }

        if (jumpSwitch){
            djumpTime = timer.time - jumpTime;
            double tau_take = 0.5 ;
            double tau_fly = 0.4425846526766571 ;
            double tau_land =  0.5 ;
            if(tau_take + tau_fly + tau_land > djumpTime){
                MyRobot::Jump(djumpTime,base_yaw_jump,com_pos_tmp_jump);
            }else{
                jumpSwitch = false;
                printf("buttonfalse");
                    joint[18].Set(1000.0, 200.0, 100.0);
                    joint[19].Set(1000.0, 200.0, 100.0);
                    joint[20].Set(1000.0, 200.0, 100.0);
                    joint[21].Set(1000.0, 200.0, 100.0);
                    joint[22].Set(100.0, 20.0, 100.0);
                    joint[23].Set(100.0, 20.0, 100.0);
                    joint[24].Set(1000.0, 200.0, 100.0);
                    joint[25].Set(1000.0, 200.0, 100.0);
                    joint[26].Set(1000.0, 200.0, 100.0);
                    joint[27].Set(1000.0, 200.0, 100.0);
                    joint[28].Set(100.0, 20.0, 100.0);
                    joint[29].Set(100.0, 20.0, 100.0);
            }
            

        }

		footstep.steps.push_back(step);
		footstep.steps.push_back(step);
		footstep.steps.push_back(step);
		step.stride = 0.0;
		step.turn   = 0.0;
		footstep.steps.push_back(step);
		
		footstep_planner.Plan(param, footstep);
        footstep_planner.GenerateDCM(param, footstep);
	}

    // stepping controller generates swing foot trajectory 
    // it also performs landing position adaptation
    stepping_controller.Update(timer, param, footstep, footstep_buffer, centroid, base, foot);
    
    // stabilizer performs balance feedback
    stabilizer         .Update(timer, param, footstep_buffer, centroid, base, foot);
    
    // step timing adaptation
    //Centroid centroid_pred = centroid;
    //stabilizer.Predict(timer, param, footstep_buffer, base, centroid_pred);
    //stepping_controller.AdjustTiming(timer, param, centroid_pred, footstep, footstep_buffer);

    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;

    // calc CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

	Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


double MyRobot::analysis_solution_p(double t,double p_0,double v_0, double T_0, double lam, double a, double b, double c, double g){
    double ha = -a*T_0*T_0/lam/lam - T_0*T_0*g;
    double hb = 2*a*T_0*T_0/lam/lam - b*T_0/lam;
    double hc = -a*T_0*T_0/lam/lam + b*T_0/lam - c;
    double T_t = lam*t + T_0;
    double alpha = (lam + std::sqrt(lam*lam + 4))/2;
    double beta = (lam - std::sqrt(lam*lam + 4))/2;
    double A_1 = ha*(2*lam-beta)/(2*lam*lam-1) - (lam-beta)*hb + beta*hc - T_0*v_0 + beta*p_0;
    double A_2 = ha*(2*lam-alpha)/(2*lam*lam-1) - (lam-alpha)*hb + alpha*hc - T_0*v_0 + alpha*p_0;
    double p_t=ha/(2*lam*lam-1)*std::pow(T_t/T_0,2) - hb*(T_t/T_0) - hc + std::pow(T_t/T_0,alpha/lam)*A_1/(beta-alpha) - std::pow(T_t/T_0,beta/lam)*A_2/(beta-alpha);
    return p_t;
}
double MyRobot::analysis_solution_v(double t,double p_0,double v_0,double T_0, double lam, double a, double b, double c, double g){
    double ha = -a*T_0*T_0/lam/lam - T_0*T_0*g;
    double hb = 2*a*T_0*T_0/lam/lam - b*T_0/lam;
    double hc = -a*T_0*T_0/lam/lam + b*T_0/lam - c;
    double T_t = lam*t + T_0;
    double alpha = (lam + std::sqrt(lam*lam + 4))/2;
    double beta = (lam - std::sqrt(lam*lam + 4))/2;
    double A_1 = ha*(2*lam-beta)/(2*lam*lam-1) - (lam-beta)*hb + beta*hc - T_0*v_0 + beta*p_0;
    double A_2 = ha*(2*lam-alpha)/(2*lam*lam-1) - (lam-alpha)*hb + alpha*hc - T_0*v_0 + alpha*p_0;
    double v_t=(2*ha/(2*lam*lam-1))*(lam*T_t/(T_0*T_0)) - hb*lam/T_0 + alpha/(beta-alpha)/T_0*std::pow(T_t/T_0,alpha/lam - 1)*A_1 - beta/(beta-alpha)/T_0*std::pow(T_t/T_0,beta/lam - 1)*A_2;
    return v_t;
}

void MyRobot::Jump(double t, double yaw, Vector3 com_pos_tmp_jump)
{
    //const parameter

    double l = 0.55;
    double m = 50;
    double g = 9.8;
    double z_0 = 0.7;
    double h_0 = 0.7;
    double x_0 = 0.0169717;
    double vx0 = 0;
    double vz0 = 0;

    double z_t = z_0;
    double x_t = x_0;
    double vzt = vz0;
    double vxt = vx0;

    double fzt = 0;
    double fxt = 0;
    double f_t = 0;
    double mu_t = 0;
    double c_t = 0;
    double x_l = 0;
    double z_l = 0;
    
    double foot_height = 0.1;
    double foot_lastheight = 0.05;
 
    //fluctional parameter
    double Fmax =  3000 ;
    double tau_1 =  0.25 ;
    double tau_2 =  0.25 ;
    double tau_3 =  0.25 ;
    double tau_4 =  0.25 ;
    double tau_jump =  0.5 ;
    double T_1 =  0.7644505163781621 ;
    double T_2 =  0.10801234497346433 ;
    double T_3 =  0.10871116994750962 ;
    double T_4 =  0.7430526844897645 ;
    double c_0 =  -0.14999999999999974 ;
    double c_1 =  -0.1499999999999942 ;
    double c_2 =  0.031620996039634586 ;
    double c_3 =  4.203318337422 ;
    double c_4 =  4.130247458738874 ;
    double c_5 =  4.057176580055748 ;
    double z_bend =  0.428778144945948 ;
    double x_bend =  0.008093003606325212 ;
    double vxbend =  0.06532003468639781 ;
    double vzbend =  -2.1890785969740043 ;
    double c_bend =  -0.1499999999999942 ;
    double z_takeoff =  0.5328470896215136 ;
    double x_takeoff =  0.47690928157739737 ;
    double vztakeoff =  3.3660360433839163 ;
    double vxtakeoff =  4.643870694746253 ;
    double t_takeoff =  0.499 ;
    double t_fly =  1.18694613130284 ;
    double z_landing =  0.5360274153828803 ;
    double x_landing =  3.662604578173327 ;
    double vzlanding =  -3.356763956616084 ;
    double vxlanding =  4.643870694746253 ;
    double t_landing =  1.186 ;
    double z_bear =  0.4308447952460521 ;
    double x_bear =  4.055790170239874 ;
    double vzbear =  2.1735154026362835 ;
    double vxbear =  0.016699676631070304 ;
    double t_bear =  1.4359999999999726 ;
    double z_stand =  0.6999999999991149 ;
    double x_stand =  4.057176580055785 ;
    double vzstand =  -8.356426661748628e-12 ;
    double vxstand =  1.6259216195635418e-13 ;
    double t_stand =  1.685999999999945 ;

    //code
    if (t <= tau_1) {
        z_t = g * std::pow(T_1, 2)
              + (h_0 - g * std::pow(T_1, 2)) * std::cosh(t / T_1);
        x_t = c_0 + (c_1 - c_0) / tau_1 * t + (x_0 - c_0) * std::cosh(t / T_1)
              + T_1 * (-(c_1 - c_0) / tau_1) * std::sinh(t / T_1);
        vzt = 1 / T_1 * (h_0 - g * T_1 * T_1) * std::sinh(t / T_1);
        vxt = (c_1 - c_0) / tau_1 + 1 / T_1 * (x_0 - c_0) * std::sinh(t / T_1)
              + (vx0 - (c_1 - c_0) / tau_1) * std::cosh(t / T_1);
        c_t = c_0 + (c_1 - c_0) / tau_1 * t;

        fzt = m * (1 / std::pow(T_1, 2) * z_t);
        fxt = m
                     * (1 / std::pow(T_1, 2)
                        * (x_t - c_0 - (c_1 - c_0) / tau_1 * t));
        f_t = std::sqrt(fzt * fzt + fxt * fxt);
        mu_t = fxt / fzt;
        centroid.justbeforeR = foot[0].pos;
        centroid.justbeforeL = foot[1].pos;
        /////////////////////////////////////////////////////////////////
        double pgain = 500;
        double dgain = 20;
        double limit = 1000000;
        joint[18].Set(pgain * 10, dgain * 10, limit);
        joint[19].Set(pgain * 10, dgain * 10, limit);
        joint[20].Set(pgain * 10, dgain * 10, limit);
        joint[21].Set(pgain * 10, dgain * 10, limit);
        joint[22].Set(pgain, dgain, limit);
        joint[23].Set(pgain, dgain, limit);
        joint[24].Set(pgain * 10, dgain * 10, limit);
        joint[25].Set(pgain * 10, dgain * 10, limit);
        joint[26].Set(pgain * 10, dgain * 10, limit);
        joint[27].Set(pgain * 10, dgain * 10, limit);
        joint[28].Set(pgain, dgain, limit);
        joint[29].Set(pgain, dgain, limit);
        /////////////////////////////////////////////////////////////////
    } else if (tau_1 <= t && t < tau_jump) {
        z_t = g * std::pow(T_2, 2)
              + (z_bend - g * std::pow(T_2, 2)) * std::cosh((t - tau_1) / T_2)
              + T_2 * vzbend * std::sinh((t - tau_1) / T_2);
        x_t = c_1 + (c_2 - c_1) / tau_2 * (t - tau_1)
              + (x_bend - c_1) * std::cosh((t - tau_1) / T_2)
              + T_2 * (vxbend - (c_2 - c_1) / tau_1)
                    * std::sinh((t - tau_1) / T_2);
        vzt = 1 / T_2 * (z_bend - g * T_2 * T_2) * std::sinh((t - tau_1) / T_2)
              + vzbend * std::cosh((t - tau_1) / T_2);
        vxt = (c_2 - c_1) / tau_2
              + 1 / T_2 * (x_bend - c_1) * std::sinh((t - tau_1) / T_2)
              + (vxbend - (c_2 - c_1) / tau_2) * std::cosh((t - tau_1) / T_2);

        fzt = m * (1 / std::pow(T_2, 2) * z_t);
        fxt = m
                     * (1 / std::pow(T_2, 2)
                        * (x_t - c_1 - (c_2 - c_1) / tau_2 * (t - tau_1)));
        f_t = std::sqrt(fzt * fzt + fxt * fxt);
        mu_t = fxt / fzt;

        c_t = c_1 + (c_2 - c_1) / tau_2 * (t - tau_1);

        centroid.justbeforeR = foot[0].pos;
        centroid.justbeforeL = foot[1].pos;
        /////////////////////////////////////////////////////////////////
        double pgain = 200;
        double dgain = 50;
        double limit = 1000000;
        joint[18].Set(pgain * 10, dgain * 10, limit);
        joint[19].Set(pgain * 10, dgain * 10, limit);
        joint[20].Set(pgain * 10, dgain * 10, limit);
        joint[21].Set(pgain * 10, dgain * 10, limit);
        joint[22].Set(pgain, dgain, limit);
        joint[23].Set(pgain, dgain, limit);
        joint[24].Set(pgain * 10, dgain * 10, limit);
        joint[25].Set(pgain * 10, dgain * 10, limit);
        joint[26].Set(pgain * 10, dgain * 10, limit);
        joint[27].Set(pgain * 10, dgain * 10, limit);
        joint[28].Set(pgain, dgain, limit);
        joint[29].Set(pgain, dgain, limit);
        /////////////////////////////////////////////////////////////////
    } else if (tau_jump <= t && t < t_fly) {
        z_t = z_takeoff + vztakeoff * (t - tau_jump)
              - 0.5 * g * (t - tau_jump) * (t - tau_jump);
        x_t = x_takeoff + vxtakeoff * (t - tau_jump);
        vzt = vztakeoff - g * (t - tau_jump);
        vxt = vxtakeoff;
        fzt = 0;
        fxt = 0;
        x_l = x_stand
              * (2 * M_PI * (t - tau_jump) / (t_landing - tau_jump)
                 - sin(2 * M_PI * (t - tau_jump) / (t_landing - tau_jump)))
              / (2 * M_PI);

        z_l = foot_height
                  * (1 - cos(2 * M_PI * (t - tau_jump) / (t_landing - tau_jump)))
                  / 2
              + (x_l * foot_lastheight / x_stand);
        /////////////////////////////////////////////////////////////////
        double pgain = 100;
        double dgain = 20;
        double limit = 500;
        if (t < tau_jump + 0.05) {
            pgain = 200;
            dgain = 50;
        }
        if (t_fly - 0.05 < t) {
            pgain = 10;
            dgain = 10;
            limit = 10;
            //foot[0].pos_ref = foot[0].pos;
            //foot[1].pos_ref = foot[1].pos;
            //foot[0].foot_landing = foot[0].pos[0];
            //foot[1].foot_landing = foot[1].pos[0];
        }
        joint[18].Set(pgain * 10, dgain * 10, limit);
        joint[19].Set(pgain * 10, dgain * 10, limit);
        joint[20].Set(pgain * 10, dgain * 10, limit);
        joint[21].Set(pgain * 10, dgain * 10, limit);
        joint[22].Set(1000, 100, limit);
        joint[23].Set(1000, 100, limit);
        //joint[22].Set(pgain * 10, dgain, limit);
        //joint[23].Set(pgain * 10, dgain, limit);
        joint[24].Set(pgain * 10, dgain * 10, limit);
        joint[25].Set(pgain * 10, dgain * 10, limit);
        joint[26].Set(pgain * 10, dgain * 10, limit);
        joint[27].Set(pgain * 10, dgain * 10, limit);
        joint[28].Set(1000, 100, limit);
        joint[29].Set(1000, 100, limit);
        //joint[28].Set(pgain * 10, dgain, limit);
        //joint[29].Set(pgain * 10, dgain, limit);
        /////////////////////////////////////////////////////////////////
    } else if (t_landing < t && t < (t_landing + tau_3)) {
        z_t = g * pow(T_3, 2)
              + (z_landing - g * pow(T_3, 2)) * cosh((t - t_landing) / T_3)
              + T_3 * vzlanding * sinh((t - t_landing) / T_3);
        x_t = c_3 + (c_4 - c_3) / tau_3 * (t - t_landing)
              + (x_landing - c_3) * cosh((t - t_landing) / T_3)
              + T_3 * (vxlanding - (c_4 - c_3) / tau_3)
                    * sinh((t - t_landing) / T_3);
        vzt = 1 / T_3 * (z_landing - g * T_3 * T_3)
                  * sinh((t - t_landing) / T_3)
              + vzlanding * cosh((t - t_landing) / T_3);
        vxt = (c_4 - c_3) / tau_3
              + 1 / T_3 * (x_landing - c_3) * sinh((t - t_landing) / T_3)
              + (vxlanding - (c_4 - c_3) / tau_3) * cosh((t - t_landing) / T_3);
        fzt = m * 1 / pow(T_3, 2) * z_t;
        fxt = m * 1 / pow(T_3, 2)
                     * (x_t - c_3 - (c_4 - c_3) / tau_3 * (t - t_landing));
        f_t = sqrt(fzt * fzt + fxt * fxt);
        mu_t = fxt / fzt;
        c_t = c_3 + (c_4 - c_3) / tau_3 * (t - t_landing);
        x_l = x_stand;
        /////////////////////////////////////////////////////////////////
        

        double pgain = 300;
        double dgain = 20;
        double limit = 250;
        double minlimit = 10;
        double maxlimit = 500;
        double anklelimit = (maxlimit - minlimit) / tau_3 * (t - t_landing);
        joint[18].Set(pgain * 10, dgain * 10, limit);
        joint[19].Set(pgain * 10, dgain * 10, limit);
        joint[20].Set(pgain * 10, dgain * 10, limit);
        joint[21].Set(pgain * 10, dgain * 10, anklelimit);
        joint[22].Set(pgain * 10*3, dgain * 10, anklelimit);
        joint[23].Set(pgain * 10*3, dgain * 10, anklelimit);
        joint[24].Set(pgain * 10, dgain * 10, limit);
        joint[25].Set(pgain * 10, dgain * 10, limit);
        joint[26].Set(pgain * 10, dgain * 10, limit);
        joint[27].Set(pgain * 10, dgain * 10, anklelimit);
        joint[28].Set(pgain * 10*3, dgain * 10, anklelimit);
        joint[29].Set(pgain * 10*3, dgain * 10, anklelimit);
        if (t - t_landing < 0.025) {
            pgain = 100;
            dgain = 5;
            limit = 50;
            joint[18].Set(pgain * 10, dgain * 10, limit);
            joint[19].Set(pgain * 10, dgain * 10, limit);
            joint[20].Set(pgain * 10, dgain * 10, limit);
            joint[21].Set(pgain * 10, dgain * 10, limit);
            joint[22].Set(pgain, dgain, 10);
            joint[23].Set(pgain, dgain, 10);
            joint[24].Set(pgain * 10, dgain * 10, limit);
            joint[25].Set(pgain * 10, dgain * 10, limit);
            joint[26].Set(pgain * 10, dgain * 10, limit);
            joint[27].Set(pgain * 10, dgain * 10, limit);
            joint[28].Set(pgain, dgain, 10);
            joint[29].Set(pgain, dgain, 10);
        } 
        /*else if ((t - t_landing < 0.15) && (0.025 <= t - t_landing)) {
            pgain = 200;
            dgain = 20;
            limit = 250;
            joint[18].Set(pgain * 10, dgain * 10, limit);
            joint[19].Set(pgain * 10, dgain * 10, limit);
            joint[20].Set(pgain * 10, dgain * 10, limit);
            joint[21].Set(pgain * 10, dgain * 10, limit);
            joint[22].Set(pgain, dgain, 10);
            joint[23].Set(pgain, dgain, 10);
            joint[24].Set(pgain * 10, dgain * 10, limit);
            joint[25].Set(pgain * 10, dgain * 10, limit);
            joint[26].Set(pgain * 10, dgain * 10, limit);
            joint[27].Set(pgain * 10, dgain * 10, limit);
            joint[28].Set(pgain, dgain, 10);
            joint[29].Set(pgain, dgain, 10);
        }*/
        /////////////////////////////////////////////////////////////////


    } else if ((t_landing + tau_3) <= t && t < (t_landing + tau_3 + tau_4)) {
        z_t = g * pow(T_4, 2)
              + (z_bear - g * pow(T_4, 2)) * cosh((t - t_bear) / T_4)
              + T_4 * vzbear * sinh((t - t_bear) / T_4);
        x_t = c_4 + (c_5 - c_4) / tau_4 * (t - t_bear)
              + (x_bear - c_4) * cosh((t - t_bear) / T_4)
              + T_4 * (vxbear - (c_5 - c_4) / tau_4) * sinh((t - t_bear) / T_4);
        vzt = 1 / T_4 * (z_bear - g * T_4 * T_4)
                  * sinh((t - t_landing - tau_1) / T_4)
              + vzbear * cosh((t - t_landing - tau_3) / T_4);
        vxt = (c_5 - c_4) / tau_4
              + 1 / T_4 * (x_bear - c_4) * sinh((t - t_bear) / T_4)
              + (vxbear - (c_5 - c_4) / tau_4) * cosh((t - t_bear) / T_4);

        fzt = m * 1 / pow(T_4, 2) * z_t;
        fxt = m * 1 / pow(T_4, 2)
                     * (x_t - c_4 - (c_5 - c_4) / tau_4 * (t - t_bear));
        f_t = sqrt(fzt * fzt + fxt * fxt);
        mu_t = fxt / fzt;
        c_t = c_4 + (c_5 - c_4) / tau_4 * (t - t_bear);
        x_l = x_stand;
        /////////////////////////////////////////////////////////////////
        double pgain =300;
        double dgain = 20;
        double limit = 10000;
        joint[18].Set(pgain * 10, dgain * 10, limit);
        joint[19].Set(pgain * 10, dgain * 10, limit);
        joint[20].Set(pgain * 10, dgain * 10, limit);
        joint[21].Set(pgain * 10, dgain * 10, limit);
        joint[22].Set(pgain * 30, dgain * 10, limit);
        joint[23].Set(pgain * 30, dgain * 10, limit);
        joint[24].Set(pgain * 10, dgain * 10, limit);
        joint[25].Set(pgain * 10, dgain * 10, limit);
        joint[26].Set(pgain * 10, dgain * 10, limit);
        joint[27].Set(pgain * 10, dgain * 10, limit);
        joint[28].Set(pgain * 30, dgain * 10, limit);
        joint[29].Set(pgain * 30, dgain * 10, limit);
        /////////////////////////////////////////////////////////////////
    } else {
        z_t = param.com_height;
        x_t = x_stand;
        fzt = m * g;
        fxt = 0;
        c_t = x_stand;
        x_l = x_stand;
        z_l = 0;
        stepping_controller
        .Update(timer, param, footstep, footstep_buffer, centroid, base, foot);
        stabilizer.Update(timer, param, footstep_buffer, centroid, base, foot);
    }

    // centroid.com_pos_ref = Vector3(x_t*std::cos(yaw), x_t*std::sin(yaw), z_t) + com_pos_tmp_jump - Vector3(0.0, 0.0, z_0);
    centroid.com_pos_ref =
    com_pos_tmp_jump                                           // ← 開始時の基準位置
    + Vector3( (x_t - x_0)*std::cos(yaw),                      // ← x の増分を yaw で回転
               (x_t - x_0)*std::sin(yaw),
               (z_t - z_0) );                                  // ← z も増分だけ足す
    foot[0].force_ref = Vector3(fxt*std::cos(yaw) / 2, fxt*std::sin(yaw)/2 , fzt / 2);
    foot[1].force_ref = Vector3(fxt*std::cos(yaw) / 2, fxt*std::sin(yaw)/2 , fzt / 2);
    foot[0].moment_ref = Vector3(fzt * c_t / 2 * std::sin(yaw), -fzt * c_t / 2 * std::cos(yaw), 0.0);
    foot[1].moment_ref = Vector3(fzt * c_t / 2 * std::sin(yaw), -fzt * c_t / 2 * std::cos(yaw), 0.0);
    foot[0].pos_ref = centroid.justbeforeR + Vector3(x_l*std::cos(yaw), x_l*std::sin(yaw), z_l);
    foot[1].pos_ref = centroid.justbeforeL + Vector3(x_l*std::cos(yaw), x_l*std::sin(yaw), z_l);
    //foot[0].ori_ref = base.ori;
    //foot[1].ori_ref = base.ori;
    foot[0].ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);
    foot[1].ori_ref = Quaternion(1.0, 0.0, 0.0, 0.0);
    //foot[0].zmp_ref = Vector3(c_t, -1.0, 0.0);
    //foot[0].zmp_ref = Vector3(c_t, 1.0, 0.0);
    // centroid.zmp_ref = Vector3(c_t*std::cos(yaw) + (centroid.justbeforeL[0] + centroid.justbeforeR[1]) / 2, c_t*std::sin(yaw) + (centroid.justbeforeL[0] + centroid.justbeforeR[1]) / 2, 0.0);
    //if (0 <= t && t < t_landing) {
    //    base.ori_ref = base.ori;
    //}
    ////    base.ori_ref = base.ori;
    base.ori_ref = base.ori;


 }


}
}