#pragma once

#include "robot.h"
#include "iksolver.h"
#include "fksolver.h"
#include "footstep.h"
#include "footstep_planner.h"
#include "stepping_controller.h"
#include "stabilizer.h"

#include <cnoid/Joystick>

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
    double    standby_period;      ///< period of initial standby mode
	double    standby_com_height;  ///< com height in standby mode

    int       plan_cycle;
    bool      use_joystick;
    double    max_stride;
    double    max_turn;
    double    max_sway;
    Joystick  joystick;

    bool      stairSwitch;
    double    stairTime;
    double    dstairTime;

    bool      jumpSwitch;
    double    jumpTime;
    double    djumpTime;

    double base_yaw_jump;

    Vector3 com_pos_tmp_jump;

	Timer            timer;
    Param            param;
    Centroid         centroid;
    Base             base;
    vector<Hand>     hand;
    vector<Foot>     foot;
    vector<Joint>    joint;
    Footstep         footstep;    
    Footstep         footstep_buffer;

    FootstepPlanner     footstep_planner;
    SteppingController  stepping_controller;
    Stabilizer          stabilizer;
    FkSolver            fk_solver;
    IkSolver            ik_solver;

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();

    
	
	MyRobot();
    

    double analysis_solution_p(double t,double p_0,double v_0, double T_0, double lam, double a, double b, double c, double g);
    double analysis_solution_v(double t,double p_0,double v_0,double T_0, double lam, double a, double b, double c, double g);
    // void Jump(double t, double yaw);
    void Jump(double t, double yaw, Vector3 com_pos_tmp_jump);

};

}
}