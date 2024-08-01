#include "myrobot.h"
#include <iostream>

using namespace std;

namespace cnoid{
namespace vnoid{

MyRobot::MyRobot(){
    base_actuation = false;
}

void MyRobot::Init(SimpleControllerIO* io){
    // init params
    //  dynamical parameters
	param.total_mass = 50.0;
	param.com_height =  0.75;
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
    stabilizer.dcm_ctrl_gain           = 5.0;
    stabilizer.base_tilt_rate          = 5.0;
    stabilizer.base_tilt_damping_p     = 100.0;
    stabilizer.base_tilt_damping_d     = 50.0;

    flagCamera = false;
    flagStairStep = false;

    max_stride = 0.085;
    max_sway = 0.085;
    max_turn = 0.05;

    stairSwitch = 0;
    stairCount = 0;
}

void MyRobot::Camera(MyCamera* camera){
    points_convex.clear();
    camera->GroundScan(points_convex);

    ground_rectangle.clear();
    if (points_convex.size() != 0){
        ground_rectangle = fk_solver.FootToGroundFK(param, joint, base, foot, points_convex);
    }else {
        ground_rectangle.push_back(Vector3(0, 0, 0));
        ground_rectangle.push_back(Vector3(0, 0, 0));
        ground_rectangle.push_back(Vector3(0, 0, 0));
        ground_rectangle.push_back(Vector3(0, 0, 0));
    }
    
    int i = 0;
    for(Vector3& p : ground_rectangle){
        printf("id%d: %lf, %lf, %lf\n", i, p.x(), p.y(), p.z());
        i++;
    }

    // flagStairStep = true;       // 分離用
}

void MyRobot::Control(MyCamera* camera){
    Robot::Sense(timer, base, foot, joint);

    // calc FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);

	if(timer.count % 10 == 0){
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
		
		// std::cout << joystick.getPosition(Joystick::L_STICK_H_AXIS) << " " 
		// 	    << joystick.getPosition(Joystick::L_STICK_V_AXIS) << " " 
		// 	    << joystick.getPosition(Joystick::R_STICK_H_AXIS) << " " 
		// 	    << joystick.getPosition(Joystick::R_STICK_V_AXIS) << " " 
		// 	    << joystick.getButtonState(Joystick::A_BUTTON) << " "
		// 	    << joystick.getButtonState(Joystick::B_BUTTON) << " "
		// 	    << joystick.getButtonState(Joystick::X_BUTTON) << " "
		// 	    << joystick.getButtonState(Joystick::Y_BUTTON) << " "
		// 	    << joystick.getButtonState(Joystick7::L_BUTTON) << " "
		// 	    << joystick.getButtonState(Joystick::R_BUTTON) << " "
        //      << joystick.getPosition(Joystick::DIRECTIONAL_PAD_V_AXIS) << " " 
        //      << joystick.getPosition(Joystick::DIRECTIONAL_PAD_H_AXIS) << " " <<std::endl;

		// erase current footsteps
		while(footstep.steps.size() > 2)
			footstep.steps.pop_back();

        // planning the desire landing potion and orientation by joystick input
        // Robot::Operation(footstep.steps, base);

        Step step;
        stairSwitch	  = joystick.getButtonState(Joystick::X_BUTTON);
        step.stride   = - max_stride * joystick.getPosition(Joystick::L_STICK_V_AXIS);
        step.sway     = - max_sway   * joystick.getPosition(Joystick::L_STICK_H_AXIS);
        step.turn     = - max_turn   * (joystick.getButtonState(Joystick::R_BUTTON) - joystick.getButtonState(Joystick::L_BUTTON));	

        step.spacing  = 0.20;
        step.climb    = 0.0;
        step.duration = 0.235;

        if (stairSwitch){
            step.spacing  = 0.10;
            step.duration = 0.8;
        }

        // get point cloud
        if (joystick.getButtonState(Joystick::A_BUTTON)){
            MyRobot::Camera(camera);
            if (std::fabs(ground_rectangle[0].z()) >= 0.05){
                flagCamera = true;
            }
        }

        // 階段の連続昇降
        /*
        概要　：階段昇降の連続化．撮影→歩行→撮影→...を階段の上り，下りが続く分だけ繰り返す．
        使用法：階段前で位置調整を行い，Aボタン（1ボタン）を一度押すだけ．
        */
        // if (flagCamera){
        //     step.spacing  = 0.10;
        //     step.duration = 0.8;

        //     stairCount++;
        //     if (stairCount == 80){
        //         flagStairStep = true;
        //     }else if (stairCount == 160){
        //         flagStairStep = false;
        //     }else if (stairCount == 400){
        //         stairCount = 0;
        //         flagCamera = false;
        //         double pre_height = ground_rectangle[0].z();
        //         MyRobot::Camera(camera);
        //         if (std::fabs(ground_rectangle[0].z()) >= 0.05 & pre_height * ground_rectangle[0].z() > 0){
        //             flagCamera = true;
        //         }
        //     }
        // }

        // 階段昇降時の撮影，踏み出しの一連化
        /*
        概要　：階段昇降1歩分の一連化．撮影→歩行を行う．
        使用法：階段前で位置調整を行い，Xボタン（3ボタン）を押しっぱなしにしつつAボタン（1ボタン）を押す．
        */
        if (flagCamera){
            stairCount++;
            flagStairStep = true;
            if (stairCount == 80){
                flagCamera    = false;
                flagStairStep = false;
                stairCount = 0;
            }
        }

        if (flagStairStep){                     // 連続化，一連化
        // if (flagStairStep && step.stride > 0){   // 分離
            step.climb    = ground_rectangle[0].z();

            // Vector2 A = Vector2(ground_rectangle[0].x(), ground_rectangle[0].y());
            // Vector2 B = Vector2(ground_rectangle[1].x(), ground_rectangle[1].y());
            Vector2 C = Vector2(ground_rectangle[2].x(), ground_rectangle[2].y());
            Vector2 D = Vector2(ground_rectangle[3].x(), ground_rectangle[3].y());
            Vector2 P = Vector2(0.0, 0.0);

            // double P2AB = std::fabs((B - A).x()*(P - A).y() - (B - A).y()*(P - A).x()) / (B - A).norm();
            double P2CD = std::fabs((D - C).x()*(P - C).y() - (D - C).y()*(P - C).x()) / (D - C).norm();
            
            if (step.climb < 0){
                step.stride = P2CD + 0.17;
            }else if (step.climb < 0.15){
                step.stride = P2CD + 0.12;	 	// 第１ステージ右用
            }else{
                step.stride = P2CD + 0.07;		// 階段上り用
            }
        }

        footstep.steps.push_back(step);
        footstep.steps.push_back(step);
        footstep.steps.push_back(step);

        step.stride   = 0.0;
        step.turn 	  = 0.0;
        step.sway 	  = 0.0;

        footstep.steps.push_back(step);

        //// old landing planner
        // double max_stride = 2.0;
        // double max_turn   = M_PI / 4;
    	// double max_sway   = 0.20;
        // Step step;
        // step.stride   = 0.0 -max_stride*joystick.getPosition(Joystick::L_STICK_V_AXIS);
        // step.turn     = 0.0 -max_turn  *joystick.getPosition(Joystick::R_STICK_H_AXIS);
        // step.sway     = 0.0 -max_sway  *joystick.getPosition(Joystick::L_STICK_H_AXIS);
        // step.spacing  = 0.20;
        // step.climb    = 0.0;
        // step.duration = 0.5;
        // footstep.steps.push_back(step);
        // footstep.steps.push_back(step);
        // footstep.steps.push_back(step);
        // step.stride = 0.0;
        // step.turn   = 0.0;
        // step.sway   = 0.0;
        // footstep.steps.push_back(step);
    
		footstep_planner.Plan(param, footstep);
        footstep_planner.GenerateDCM(param, footstep);
	}

    // stepping controller generates swing foot trajectory 
    // it also performs landing position adaptation
    stepping_controller.Update(timer, param, footstep, footstep_buffer, centroid, base, foot, flagStairStep, ground_rectangle);
    
    // stabilizer performs balance feedback
    stabilizer         .Update(timer, param, footstep_buffer, centroid, base, foot);
    
    // step timing adaptation
    // Centroid centroid_pred = centroid;
    // stabilizer.Predict(timer, param, footstep_buffer, base, centroid_pred);
    // stepping_controller.AdjustTiming(timer, param, centroid_pred, footstep, footstep_buffer);

    hand[0].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0, -0.25, -0.1);
    hand[0].ori_ref = base.ori_ref;
    hand[1].pos_ref = centroid.com_pos_ref + base.ori_ref*Vector3(0.0,  0.25, -0.1);
    hand[1].ori_ref = base.ori_ref;

    // calc CoM IK
    ik_solver.Comp(&fk_solver, param, centroid, base, hand, foot, joint);

	Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
