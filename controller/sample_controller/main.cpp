#include <cnoid/SimpleController>
#include <cnoid/Body>
#include <cnoid/Camera>
#include <cnoid/RangeCamera>
#include <cnoid/Joystick>

#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

#include "myrobot.h"
#include "mycamera.h"

using namespace cnoid;
using namespace cnoid::vnoid;

class VnoidSampleController : public SimpleController{
public:
	MyRobot*  robot;
    MyCamera* camera;
    FkSolver* fk_solver;
    Joystick joystick;

    //vector<Vector3> ground_rectangle;
    bool PreAButtonState;
    bool PreBButtonState;  // 新しく追加
    bool PreYButtonState;  // 新しく追加
    int count;

public:
    virtual bool configure(SimpleControllerConfig* config){
        return true;
    }

	virtual bool initialize(SimpleControllerIO* io){
        camera = new MyCamera();
        camera->Init(io);
        count = 0;

		robot = new MyRobot();
		robot->Init(io);

        PreAButtonState = false;
        PreBButtonState = false;
        PreYButtonState = false;

		return true;
	}

	virtual bool control()	{
        joystick.readCurrentState();
        bool AButtonState = joystick.getButtonState(Joystick::A_BUTTON);
        if (AButtonState && !PreAButtonState) {
            robot->points_convex.clear();
            printf("push A_BUTTON\n");
            camera->GroundScan(robot->points_convex);
            robot->compStairStep = true;
            //ground_rectangle = fk_solver->FootToGroundFK(robot);
            //int i;
            //for(Vector3& p : ground_rectangle){
            //    printf("id%d: %lf, %lf, %lf\n", i, p.x(), p.y(), p.z());
            //    i++;
            //}
        }
        PreAButtonState = AButtonState;

        // 新しく追加：Y_BUTTON処理（うつ伏せ起き上がり）
        bool YButtonState = joystick.getButtonState(Joystick::Y_BUTTON);
        if (YButtonState && !PreYButtonState) {
            printf("=== Y_BUTTON PRESSED ===\n");
            printf("Starting FACE_DOWN getup sequence\n");
            robot->startManualGetup(MyRobot::FallDirection::FACE_DOWN);
        }
        PreYButtonState = YButtonState;
        
        // 新しく追加：B_BUTTON処理（仰向け起き上がり）
        bool BButtonState = joystick.getButtonState(Joystick::B_BUTTON);
        if (BButtonState && !PreBButtonState) {
            printf("=== B_BUTTON PRESSED ===\n");
            printf("Starting FACE_UP getup sequence\n");
            robot->startManualGetup(MyRobot::FallDirection::FACE_UP);
        }
        PreBButtonState = BButtonState;
        
		robot->Control();
        count++;
		return true;
	}
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(VnoidSampleController)
