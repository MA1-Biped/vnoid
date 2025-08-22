#pragma once

#include "robot.h"
#include "iksolver.h"
#include "fksolver.h"
#include "footstep.h"
#include "footstep_planner.h"
#include "stepping_controller.h"
#include "stabilizer.h"
#include <vector> // vectorを使うために追加

namespace cnoid{
namespace vnoid{

class MyRobot : public Robot{
public:
	// [ADD] 起き上がり制御の状態を定義
    enum class GetupState {
        INACTIVE,         // 通常状態 (歩行など)
        CHECK_POSE,       // 転倒を検知し、姿勢をチェックする初期段階
        TUCK_UP,          // うつ伏せから手足を縮める
        PUSH_UP,          // 腕立て伏せのように体を持ち上げる
        KNEEL_UP,         // 膝立ち姿勢に移行する
        FINISHED          // 起き上がり完了

        /*TRANSITION_TO_STABILIZER,  // 新しく追加：stabilizerへの移行
        STAND_UP,         // 膝立ちから立ち上がる
        STABILIZER_ACTIVE, // stabilizerで制御中*/
    };

    // 簡略化された倒れ方向
    enum class FallDirection {
        UNKNOWN,
        FACE_DOWN,    // うつ伏せ
        FACE_UP       // 仰向け
    };
    
    double    standby_period;      ///< period of initial standby mode
	double    standby_com_height;  ///< com height in standby mode

    int       plan_cycle;
    bool      use_joystick;

	Timer            timer;
    Param            param;
    Centroid         centroid;
    Base             base;
    vector<Hand>     hand;
    vector<Foot>     foot;
    vector<Joint>    joint;
    Footstep         footstep;    
    Footstep         footstep_buffer;
    
    // [ADD] 階段昇降用の変数を追加（元のコードから推測）
    bool compStairStep;
    vector<Vector3> ground_rectangle;
    vector<Vector3> points_convex;

    FootstepPlanner     footstep_planner;
    SteppingController  stepping_controller;
    Stabilizer          stabilizer;
    FkSolver            fk_solver;
    IkSolver            ik_solver;

    bool    PreAButtonState;

    void startManualGetup(FallDirection direction);

public:
	virtual void  Init   (SimpleControllerIO* io);
	virtual void  Control();
    
	MyRobot();
	
private:
    // [ADD] 起き上がり機能用のプライベートメンバー
    GetupState getup_state_;
    double motion_timer_;
    FallDirection fall_direction_;

    bool stabilizer_initialized_;
    double stabilizer_start_time_;
    double kneel_com_height_; 

    // 元のキーフレームデータ（書き換えない）
    std::vector<double> q_initial_;

    //std::vector<double> q_tuck_facedown_, q_pushup_facedown_, q_kneel_facedown_, q_standup_facedown_;
    //std::vector<double> q_tuck_faceup_, q_pushup_faceup_, q_kneel_faceup_, q_standup_faceup_;

    std::vector<double> q_tuck_facedown_, q_pushup_facedown_, q_kneel_facedown_;
    std::vector<double> q_tuck_faceup_, q_pushup_faceup_, q_kneel_faceup_;

    // 参照ポインタ（動的に切り替える）
    const std::vector<double>* current_q_tuck_;
    const std::vector<double>* current_q_pushup_;
    const std::vector<double>* current_q_kneel_;
    //const std::vector<double>* current_q_standup_;

    // [ADD] 起き上がり制御用のプライベート関数
    void updateGetupController();
    FallDirection detectFallDirection();
    void initializeKeyframes();
    //void createFaceUpKeyframes();  // うつ伏せから仰向けキーフレームを生成
    void selectKeyframesForDirection(FallDirection direction);


    // stabilizerへの移行関数
    //void transitionToStabilizer();
    void initializeStabilizerAfterKneel();
    void updateStabilizerControl();
    //bool isStabilizerReady();

};

}
}
