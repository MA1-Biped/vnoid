#include "robot.h"
#include "footstep.h"
#include "iksolver.h"
#include "rollpitchyaw.h"
#include <iostream>

namespace cnoid{
namespace vnoid{

///////////////////////////////////////////////////////////////////////////////////////////////////

Joint::Joint(){
	pgain  = 0.0;
	dgain  = 0.0;
	ulimit = 0.0;

	q      = 0.0;
	dq     = 0.0;
	q_ref  = 0.0;
	dq_ref = 0.0;
	u      = 0.0;
	u_ref  = 0.0;
}

void Joint::Set(double _pgain, double _dgain, double _ulimit){
    pgain  = _pgain;
    dgain  = _dgain;
    ulimit = _ulimit;
}

void Joint::CalcTorque(){
	u = u_ref + pgain*(q_ref - q) + dgain*(dq_ref - dq);
	u = std::min(std::max(-ulimit, u), ulimit);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Centroid::Centroid(){
	force_ref   = Vector3(0.0, 0.0, 0.0);
	moment_ref  = Vector3(0.0, 0.0, 0.0);
	zmp         = Vector3(0.0, 0.0, 0.0);
	zmp_ref     = Vector3(0.0, 0.0, 0.0);
	dcm         = Vector3(0.0, 0.0, 0.0);
	dcm_ref     = Vector3(0.0, 0.0, 0.0);
	com_pos     = Vector3(0.0, 0.0, 0.0);
	com_pos_ref = Vector3(0.0, 0.0, 0.0);
	com_vel_ref = Vector3(0.0, 0.0, 0.0);
	com_acc_ref = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Base::Base(){
	pos        = Vector3(0.0, 0.0, 0.0);
	pos_ref    = Vector3(0.0, 0.0, 0.0);
	angle      = Vector3(0.0, 0.0, 0.0);
	angle_ref  = Vector3(0.0, 0.0, 0.0);
	ori        = Quaternion(1.0, 0.0, 0.0, 0.0);
	ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
	vel_ref    = Vector3(0.0, 0.0, 0.0);
	angvel     = Vector3(0.0, 0.0, 0.0);
	angvel_ref = Vector3(0.0, 0.0, 0.0);
	acc_ref    = Vector3(0.0, 0.0, 0.0);
	angacc_ref = Vector3(0.0, 0.0, 0.0);	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Hand::Hand(){
	pos        = Vector3(0.0, 0.0, 0.0);
	pos_ref    = Vector3(0.0, 0.0, 0.0);
	vel_ref    = Vector3(0.0, 0.0, 0.0);
	acc_ref    = Vector3(0.0, 0.0, 0.0);
	ori        = Quaternion(1.0, 0.0, 0.0, 0.0);
	ori_ref    = Quaternion(1.0, 0.0, 0.0, 0.0);
	angle      = Vector3(0.0, 0.0, 0.0);
	angle_ref  = Vector3(0.0, 0.0, 0.0);
	angvel_ref = Vector3(0.0, 0.0, 0.0);
	angacc_ref = Vector3(0.0, 0.0, 0.0);
	arm_twist  = 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Foot::Foot(){
    contact          = false;
	contact_ref      = false;
	balance          = 0.0;
	balance_ref      = 0.0;
	pos              = Vector3(0.0, 0.0, 0.0);
	pos_ref          = Vector3(0.0, 0.0, 0.0);
	ori              = Quaternion(1.0, 0.0, 0.0, 0.0);
	ori_ref          = Quaternion(1.0, 0.0, 0.0, 0.0);
	angle            = Vector3(0.0, 0.0, 0.0);
	angle_ref        = Vector3(0.0, 0.0, 0.0);
	vel_ref          = Vector3(0.0, 0.0, 0.0);
	angvel_ref       = Vector3(0.0, 0.0, 0.0);
	acc_ref          = Vector3(0.0, 0.0, 0.0);
	angacc_ref       = Vector3(0.0, 0.0, 0.0);
	force            = Vector3(0.0, 0.0, 0.0);
	force_ref        = Vector3(0.0, 0.0, 0.0);
	moment           = Vector3(0.0, 0.0, 0.0);
	moment_ref       = Vector3(0.0, 0.0, 0.0);
	zmp              = Vector3(0.0, 0.0, 0.0);
	zmp_ref          = Vector3(0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Ground::Ground(){
	angle = Vector3(0.0, 0.0, 0.0);
	ori   = Quaternion(1.0, 0.0, 0.0, 0.0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Param::Param(){
    total_mass       = 50.0;
	nominal_inertia  = Vector3(20.0, 20.0, 5.0);
	com_height       = 1.0;
	gravity          = 9.8;
    
    base_to_shoulder[0] = Vector3(0.0, 0.0, 0.0);
    base_to_shoulder[1] = Vector3(0.0, 0.0, 0.0);
    base_to_hip     [0] = Vector3(0.0, 0.0, 0.0);
    base_to_hip     [1] = Vector3(0.0, 0.0, 0.0);
    arm_joint_index [0] = 0;
    arm_joint_index [1] = 0;
    leg_joint_index [0] = 0;
    leg_joint_index [1] = 0;
    
    upper_arm_length = 0.2;
    lower_arm_length = 0.2;
    upper_leg_length = 0.3;
    lower_leg_length = 0.4;

	trunk_mass = 1.0;
	trunk_com  = Vector3(0.0, 0.0, 0.0);
	
	for (int i = 0; i < 7; i++) {
        arm_mass[i] = 0.0;
        arm_com[i] = Vector3(0.0, 0.0, 0.0);
    }
    for (int i = 0; i < 6; i++) {
        leg_mass[i] = 0.0;
        leg_com[i] = Vector3(0.0, 0.0, 0.0);
    }

    Init();
}

void Param::Init(){
    T = sqrt(com_height/gravity);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Timer::Timer(){
	count = 0;
    time  = 0.0;
    dt    = 0.001;
}

void Timer::Countup(){
	count++;
	time += dt;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Robot::Robot(){
    base_acc_sensor_name    = "gsensor"  ;
	base_gyro_sensor_name   = "gyrometer";
	right_force_sensor_name = "rfsensor" ;
	left_force_sensor_name  = "lfsensor" ;

    gyro_axis_x = Vector3(1.0, 0.0, 0.0);
    gyro_axis_y = Vector3(0.0, 1.0, 0.0);
    gyro_axis_z = Vector3(0.0, 0.0, 1.0);
    
	base_actuation            = false;
    gyro_filter_cutoff        = 20.0;
    acc_filter_cutoff         = 20.0;
    foot_force_filter_cutoff  = 20.0;
    foot_moment_filter_cutoff = 20.0;
    joint_pos_filter_cutoff   = 10.0;

	max_stride = 0.085;
	max_sway = 0.085;
	max_turn = 0.05;

	stairSwitch = 0;
	stairCount = 0;
}

void Robot::Init(SimpleControllerIO* io, Timer& timer, vector<Joint>& joint){
	io_body = io->body();
	
	if(base_actuation){
        io_body->link(0)->setActuationMode(cnoid::Link::LinkPosition);
        io->enableIO(io_body->link(0));
    }
	io->enableInput (io_body->link(0), cnoid::Link::LinkPosition);

    joint_pos_filter.resize(joint.size());
	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);
		
        jnt->setActuationMode(cnoid::Link::JointTorque);
		io->enableIO(jnt);
		io->enableInput(jnt, cnoid::Link::JointVelocity);
		
		joint[i].q_ref  = joint[i].q  = jnt->q ();
		joint[i].dq_ref = joint[i].dq = jnt->dq();
		joint[i].u      = 0.0;

        joint_pos_filter[i].SetCutoff(joint_pos_filter_cutoff);
	}
		
	{
        accel_sensor = io_body->findDevice<AccelerationSensor>(base_acc_sensor_name  );
		gyro_sensor  = io_body->findDevice<RateGyroSensor    >(base_gyro_sensor_name );

        if(accel_sensor) io->enableInput(accel_sensor);
        if(gyro_sensor ) io->enableInput(gyro_sensor );

        for(int j = 0; j < 3; j++){
            acc_filter [j].SetCutoff(acc_filter_cutoff );
            gyro_filter[j].SetCutoff(gyro_filter_cutoff);
        }
	}

	for(int i = 0; i < 2; i++){
		foot_force_sensor[i] = io_body->findDevice<ForceSensor>(i == 0 ? right_force_sensor_name : left_force_sensor_name);
        if(foot_force_sensor[i])
            io->enableInput(foot_force_sensor[i]);

        for(int j = 0; j < 3; j++){
            foot_force_filter [i][j].SetCutoff(foot_force_filter_cutoff );
            foot_moment_filter[i][j].SetCutoff(foot_moment_filter_cutoff);
        }
	}
	
	timer.dt = io->timeStep();

}

void Robot::Sense(Timer& timer, Base& base, vector<Joint>& joint){
	// store absolute position of base link
	{
        Link* lnk = io_body->link(0);
        base.pos = lnk->p();
    }

	if(accel_sensor){
		Vector3 a = accel_sensor->dv();
		base.acc[0] = acc_filter[0](gyro_axis_x.dot(a), timer.dt);
		base.acc[1] = acc_filter[1](gyro_axis_y.dot(a), timer.dt);
		base.acc[2] = acc_filter[2](gyro_axis_z.dot(a), timer.dt);
	}
	if(gyro_sensor){
        Vector3 w = gyro_sensor->w();
        base.angvel[0] = gyro_filter[0](gyro_axis_x.dot(w), timer.dt);
        base.angvel[1] = gyro_filter[1](gyro_axis_y.dot(w), timer.dt);
        base.angvel[2] = gyro_filter[2](gyro_axis_z.dot(w), timer.dt);
    }

	const double g = 9.8;
	const double angle_correction_gain = 0.01;
	base.angle [0] += (base.angvel[0] + angle_correction_gain*(  base.acc.y() - g*base.angle.x() ))*timer.dt;
	base.angle [1] += (base.angvel[1] + angle_correction_gain*( -base.acc.x() - g*base.angle.y() ))*timer.dt;
	base.angle [2] +=  base.angvel[2]*timer.dt;

	base.ori = FromRollPitchYaw(base.angle);

	for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);

        // get position and velocity of each joint
		joint[i].q  = jnt->q ();
		joint[i].dq = jnt->dq();
	}
}

void Robot::Sense(Timer& timer, Base& base, vector<Foot>& foot, vector<Joint>& joint){
	Sense(timer, base, joint);

	for(int i = 0; i < 2; i++){
		// get force/moment from force sensor
        if(foot_force_sensor[i]){
            Vector3 f = foot_force_sensor[i]->F().segment<3>(0);
            Vector3 m = foot_force_sensor[i]->F().segment<3>(3);
            for(int j = 0; j < 3; j++){
		        foot[i].force [j] = foot_force_filter [i][j](f[j], timer.dt);
		        foot[i].moment[j] = foot_moment_filter[i][j](m[j], timer.dt);
            }
        }
    }
}

void Robot::Actuate(Timer& timer, Base& base, vector<Joint>& joint){
    // if base actuation is enabled, directly specify the position and orientation of the base link
    if(base_actuation){
        Link* lnk = io_body->link(0);
        lnk->p() = base.pos_ref;
        lnk->R() = base.ori_ref.matrix();
    }

    for (int i = 0; i < joint.size(); ++i) {
		cnoid::Link* jnt = io_body->joint(i);

        // filter position reference
        joint[i].q_ref  = joint_pos_filter[i](joint[i].q_ref, timer.dt);
        // velocity reference if taken from the time derivative of the filter output
        joint[i].dq_ref = joint_pos_filter[i].yd;
		
		// determine joint torque by PD control
        joint[i].CalcTorque();
		jnt->u() = joint[i].u;
	}
}

// add Robot::Operartion function to control the robot by using joystick
// add sway movement: 2024/01/15: Tanaka
void Robot::Operation(deque<Step>& steps, Base& base){ // [向井] 修正予定
	joystick.readCurrentState();

	// std::cout << base.angle.z() << std::endl; //正面０で反時計回りせいで単位がradだった

	Step step;
	stairSwitch	  = joystick.getButtonState(Joystick::X_BUTTON);
	step.stride   = - max_stride * joystick.getPosition(Joystick::L_STICK_V_AXIS);
	step.sway     = - max_sway   * joystick.getPosition(Joystick::L_STICK_H_AXIS);
	step.turn     = - max_turn   * (joystick.getButtonState(Joystick::R_BUTTON) - joystick.getButtonState(Joystick::L_BUTTON));	

	step.spacing  = 0.20;
	step.climb    = 0.0;
	step.duration = 0.235;
	
	if(stairSwitch == 1){
		step.duration = 0.8;
		step.spacing  = 0.10;
	}

	// Landing position planner on stairs: 2024/06/04: Tanaka
	/*  
		概要　　：階段昇降時における着地位置修正．階段の手前のエッジ (CD) から一定距離離れた位置 (Q) に着地位置を修正する．
				　PからCDまでの距離P2CDを点と直線の距離の公式をもとに計算し，調整を加えてstep.strideを決定する．
		座標
			A：階段平面の右上座標
			B：階段平面の左上座標
			C：階段平面の左下座標
			D：階段平面の右下座標
			P：支持足位置座標 (0, 0)
			Q：着地位置座標

		B ---------------------	A
		|						|
		|			Q			|
		|						|
		C ---------------------	D

					P
	*/

	// Combine getting point cloud and stepping one step: 2024/07/22: Tanaka
	/*
		概要　　：階段の次の段の点群を取得した後，自動的に一歩踏み出すことで階段昇降を一連化する．
				　階段昇降モードの切り替えフラグであるcompStairStepをfalseにするタイミングを調整することで，一歩の切り出しを行っている．
				　環境（PC?）によって適切な切り替えタイミング異なるため，使用者はそれぞれの設定を探すこと（田中の環境では80カウント）．
				　※一連化しない場合は，if(compStairStep)以下をコメントアウトし，if(step.stride > 0 && compStairStep)を利用すること．
		細かい話：入力は歩行３歩＋停止１歩の４歩を１セットとしている．
				　この入力が複数セット繰り返されることでロボットが動くらしい．
				　そこで，１歩分に必要なセット数を見つけることができれば，一連化につながる．
				　以下のstairCount == 80 を調整する．小さすぎると反応せず，大きすぎると２歩になるため丁度いい塩梅を見つけること．
	*/

	// 一連化
	// ---------------
	// if(compStairStep){
	if (flagCamera){
		stairCount++;
		if (stairCount == 80){
			flagStairStep = false;
		}else if (stairCount == 500){
			flagCamera = false;
			flag = true;
			stairCount = 0;
		}
	}
	// ---------------

	// 分離
	// if(step.stride > 0 && compStairStep){

	if (flagStairStep){
		step.climb    = ground_rectangle[0].z();		
		if (std::fabs(step.climb) >= 0.05) {
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

			// if(fabs(base.angle.z()) > 0.01) {
			// 	step.turn = - std::pow(base.angle.z(), 2) * base.angle.z() / std::fabs(base.angle.z());
			// }

		}else {
			step.climb = 0.0;
		}
		}
	steps.push_back(step);
	steps.push_back(step);
	steps.push_back(step);

	step.stride   = 0.0;
	step.turn 	  = 0.0;
	step.sway 	  = 0.0;

	steps.push_back(step);
}

}
}
