#include "myrobot.h"
#include <iostream>
#include "rollpitchyaw.h"

using namespace std;

namespace cnoid{
namespace vnoid{

// [ADD] 2つのキーフレーム間を滑らかに補間するヘルパー関数
void interpolate(vector<double>& q_out, const vector<double>& q_start, const vector<double>& q_end, double t) {
    // t を 0.0〜1.0の範囲に収める
    t = std::max(0.0, std::min(1.0, t));

    // 動き始めと終わりが滑らかになるコサインカーブで補間
    double smooth_t = (1.0 - cos(t * M_PI)) / 2.0;

    for (size_t i = 0; i < q_out.size(); ++i) {
        q_out[i] = q_start[i] + (q_end[i] - q_start[i]) * smooth_t;
    }
}

MyRobot::MyRobot(){
    use_joystick = true;

    base_actuation = false;
    // [ADD] 状態とタイマーを初期化
    getup_state_ = MyRobot::GetupState::INACTIVE;
    fall_direction_ = MyRobot::FallDirection::UNKNOWN;
    motion_timer_ = 0.0;

    // 参照ポインタを初期化（nullptrで初期化）
    current_q_tuck_ = nullptr;
    current_q_pushup_ = nullptr;
    current_q_kneel_ = nullptr;
    current_q_standup_ = nullptr;
}

void MyRobot::Init(SimpleControllerIO* io){
   
    
    // **最初に起き上がり状態を確実にINACTIVEに設定**
    getup_state_ = MyRobot::GetupState::INACTIVE;
    motion_timer_ = 0.0;
    
    
    
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


    // キーフレームの初期化
    initializeKeyframes();

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

    compStairStep = false;


    cout << "Final getup_state_ check: " << (int)getup_state_ << " (should be 0)" << endl;
    cout << "=== MyRobot::Init END ===" << endl;

}

void MyRobot::initializeKeyframes(){
    // キーフレームをリサイズ
    q_initial_.resize(30, 0.0);
    q_tuck_facedown_.resize(30, 0.0);
    q_pushup_facedown_.resize(30, 0.0);
    q_kneel_facedown_.resize(30, 0.0);
    q_standup_facedown_.resize(30, 0.0);

    // === うつ伏せ用キーフレーム（既存のもの） ===
    // キーフレーム1: 手足を縮める (Tuck)
    q_tuck_facedown_[param.arm_joint_index[0] + 0] = 0.0; q_tuck_facedown_[param.arm_joint_index[1] + 0] = 0.0; // Shoulder Pitch
    q_tuck_facedown_[param.arm_joint_index[0] + 3] =  2.0; q_tuck_facedown_[param.arm_joint_index[1] + 3] =  2.0; // Elbow Pitch
    q_tuck_facedown_[param.leg_joint_index[0] + 2] =  2.5; q_tuck_facedown_[param.leg_joint_index[1] + 2] =  2.5; // Hip Pitch
    q_tuck_facedown_[param.leg_joint_index[0] + 3] = -2.5; q_tuck_facedown_[param.leg_joint_index[1] + 3] = -2.5; // Knee Pitch
    q_tuck_facedown_[param.leg_joint_index[0] + 4] =  1.0; q_tuck_facedown_[param.leg_joint_index[1] + 4] =  1.0; // Ankle Pitch

    // キーフレーム2: 体を起こす (Push-up)
    q_pushup_facedown_ = q_tuck_facedown_;
    q_pushup_facedown_[param.arm_joint_index[0] + 0] = 1.2; q_pushup_facedown_[param.arm_joint_index[1] + 0] = 1.2; // Shoulder Pitch
    q_pushup_facedown_[param.arm_joint_index[0] + 3] =  0.5; q_pushup_facedown_[param.arm_joint_index[1] + 3] =  0.5; // Elbow Pitch
    q_pushup_facedown_[param.leg_joint_index[0] + 4] =  2.0; q_pushup_facedown_[param.leg_joint_index[1] + 4] =  2.0; // Ankle Pitch
    
    // キーフレーム3: 膝立ち (Kneel)
    q_kneel_facedown_ = q_pushup_facedown_;
    q_kneel_facedown_[param.leg_joint_index[0] + 2] =  1.0; q_kneel_facedown_[param.leg_joint_index[1] + 2] =  1.0; // Hip Pitch
    q_kneel_facedown_[param.leg_joint_index[0] + 3] = -2.0; q_kneel_facedown_[param.leg_joint_index[1] + 3] = -2.0; // Knee Pitch
    q_kneel_facedown_[param.leg_joint_index[0] + 4] =  1.0; q_kneel_facedown_[param.leg_joint_index[1] + 4] =  1.0; // Ankle Pitch
    q_kneel_facedown_[param.arm_joint_index[0] + 0] =  1.2; q_kneel_facedown_[param.arm_joint_index[1] + 0] =  1.2; // Shoulder Pitch
    q_kneel_facedown_[param.arm_joint_index[0] + 3] =  0.5; q_kneel_facedown_[param.arm_joint_index[1] + 3] =  0.5; // Elbow Pitch

    // キーフレーム4: 立ち上がり (Stand-up)
    q_standup_facedown_[param.leg_joint_index[0] + 2] = -0.1; q_standup_facedown_[param.leg_joint_index[1] + 2] = -0.1; // Hip Pitch
    q_standup_facedown_[param.leg_joint_index[0] + 3] =  0.2; q_standup_facedown_[param.leg_joint_index[1] + 3] =  0.2; // Knee Pitch
    q_standup_facedown_[param.leg_joint_index[0] + 4] = -0.1; q_standup_facedown_[param.leg_joint_index[1] + 4] = -0.1; // Ankle Pitch

    // 仰向け用キーフレームを生成（うつ伏せの反転）

     // 仰向け用キーフレームをリサイズ
    q_tuck_faceup_.resize(30, 0.0);
    q_pushup_faceup_.resize(30, 0.0);
    q_kneel_faceup_.resize(30, 0.0);
    q_standup_faceup_.resize(30, 0.0);

    // うつ伏せキーフレームを反転して仰向けキーフレームを作成
    // 基本ルール: Pitch軸の動作を反転、他の軸は調整
    
    // === 仰向け Tuck: うつ伏せTuckの反転 ===
    for(int i = 0; i < 30; i++) {
        q_tuck_faceup_[i] = 0.0;  // 初期化
    }
    
    // 腕: Shoulder Pitchを反転（上向きに）
    q_tuck_faceup_[param.arm_joint_index[0] + 0] =  0.0; q_tuck_faceup_[param.arm_joint_index[1] + 0] =  0.0; // 反転
    q_tuck_faceup_[param.arm_joint_index[0] + 3] = -2.0; q_tuck_faceup_[param.arm_joint_index[1] + 3] = -2.0; // Elbow 反転
    
    // 脚: Hip Pitchを反転（腹筋のように）
    q_tuck_faceup_[param.leg_joint_index[0] + 2] = -2.5; q_tuck_faceup_[param.leg_joint_index[1] + 2] = -2.5; // 反転
    q_tuck_faceup_[param.leg_joint_index[0] + 3] =  2.5; q_tuck_faceup_[param.leg_joint_index[1] + 3] =  2.5; // Knee 反転
    q_tuck_faceup_[param.leg_joint_index[0] + 4] = -1.0; q_tuck_faceup_[param.leg_joint_index[1] + 4] = -1.0; // Ankle 反転

    // === 仰向け Push-up: うつ伏せPush-upの反転 ===
    q_pushup_faceup_ = q_tuck_faceup_;
    q_pushup_faceup_[param.arm_joint_index[0] + 0] =  -1.2; q_pushup_faceup_[param.arm_joint_index[1] + 0] =  -1.2; // 反転
    q_pushup_faceup_[param.arm_joint_index[0] + 3] = -0.5; q_pushup_faceup_[param.arm_joint_index[1] + 3] = -0.5; // 反転
    q_pushup_facedown_[param.leg_joint_index[0] + 4] =  -2.0; q_pushup_facedown_[param.leg_joint_index[1] + 4] =  -2.0; // Ankle Pitch
    
    // === 仰向け Kneel: うつ伏せKneelの反転 ===
    q_kneel_faceup_ = q_pushup_faceup_;
    q_kneel_faceup_[param.leg_joint_index[0] + 2] = -1.0; q_kneel_faceup_[param.leg_joint_index[1] + 2] = -1.0; // 反転
    q_kneel_faceup_[param.leg_joint_index[0] + 3] =  2.0; q_kneel_faceup_[param.leg_joint_index[1] + 3] =  2.0; // 反転
    q_kneel_faceup_[param.leg_joint_index[0] + 4] = -1.0; q_kneel_faceup_[param.leg_joint_index[1] + 4] = -1.0; // 反転
    q_kneel_faceup_[param.arm_joint_index[0] + 0] =  -1.2; q_kneel_faceup_[param.arm_joint_index[1] + 0] =  -1.2; // 同じ
    q_kneel_faceup_[param.arm_joint_index[0] + 3] = -0.5; q_kneel_faceup_[param.arm_joint_index[1] + 3] = -0.5; // 反転

    // === 仰向け Stand-up: 最終的には同じ立位姿勢 ===
    q_standup_faceup_[param.leg_joint_index[0] + 2] = -0.1; q_standup_faceup_[param.leg_joint_index[1] + 2] = -0.1;
    q_standup_faceup_[param.leg_joint_index[0] + 3] =  0.2; q_standup_faceup_[param.leg_joint_index[1] + 3] =  0.2;
    q_standup_faceup_[param.leg_joint_index[0] + 4] = -0.1; q_standup_faceup_[param.leg_joint_index[1] + 4] = -0.1;
    
}

MyRobot::FallDirection MyRobot::detectFallDirection(){
    Vector3 rpy = ToRollPitchYaw(base.ori);
    double pitch_deg = rpy.y() * 180.0 / M_PI;
   
    
    cout << "Fall direction detection - Pitch: " << pitch_deg << " deg" << endl;
    

    
        // ピッチの方が大きい場合
        if (pitch_deg < 0) {
            cout << "Direction: FACE_DOWN (forward fall, pitch dominant)" << endl;
            return FallDirection::FACE_DOWN;
        } else {
            cout << "Direction: FACE_UP (backward fall, pitch dominant)" << endl;
            return FallDirection::FACE_UP;
        }
    
}

void MyRobot::selectKeyframesForDirection(FallDirection direction){
    switch(direction) {
        case FallDirection::FACE_DOWN:
            current_q_tuck_ = &q_tuck_facedown_;
            current_q_pushup_ = &q_pushup_facedown_;
            current_q_kneel_ = &q_kneel_facedown_;
            current_q_standup_ = &q_standup_facedown_;
            cout << "Selected FACE_DOWN keyframes (references set)" << endl;
            break;
            
        case FallDirection::FACE_UP:
            current_q_tuck_ = &q_tuck_faceup_;
            current_q_pushup_ = &q_pushup_faceup_;
            current_q_kneel_ = &q_kneel_faceup_;
            current_q_standup_ = &q_standup_faceup_;
            cout << "Selected FACE_UP keyframes (references set)" << endl;
            break;
            
        case FallDirection::UNKNOWN:
        default:
            // デフォルトはうつ伏せ
            current_q_tuck_ = &q_tuck_facedown_;
            current_q_pushup_ = &q_pushup_facedown_;
            current_q_kneel_ = &q_kneel_facedown_;
            current_q_standup_ = &q_standup_facedown_;
            cout << "Selected default FACE_DOWN keyframes" << endl;
            break;
    }
}


// [ADD] 新設した起き上がり制御関数
void MyRobot::updateGetupController(){
    vector<double> q_target(30);
    double duration = 2.0; // 各モーションの基本時間

    switch(getup_state_){
        case MyRobot::GetupState::CHECK_POSE:
            for(int i=0; i<30; ++i) q_initial_[i] = joint[i].q;

            // 倒れた向きを検知
            fall_direction_ = detectFallDirection();
            
            // 向きに応じたキーフレームを選択
            selectKeyframesForDirection(fall_direction_);


            getup_state_ = MyRobot::GetupState::TUCK_UP;
            motion_timer_ = 0.0;
            cout << "CHECK_POSE -> TUCK_UP" << endl;
            break;
        case MyRobot::GetupState::TUCK_UP:
            duration = 2.0;
            if(current_q_tuck_ != nullptr) {
                interpolate(q_target, q_initial_, *current_q_tuck_, motion_timer_ / duration);
            }
            if(motion_timer_ > duration){
                getup_state_ = MyRobot::GetupState::PUSH_UP;
                motion_timer_ = 0.0;
                cout << "TUCK_UP completed" << endl;
            }
            break;
        case MyRobot::GetupState::PUSH_UP:
            duration = 1.5;
            if(current_q_pushup_ != nullptr && current_q_tuck_ != nullptr) {
                interpolate(q_target, *current_q_tuck_, *current_q_pushup_, motion_timer_ / duration);
            }
            if(motion_timer_ > duration){
                getup_state_ = MyRobot::GetupState::KNEEL_UP;
                motion_timer_ = 0.0;
                cout << "PUSH_UP completed" << endl;
            }
            break;
        case MyRobot::GetupState::KNEEL_UP:
            duration = 3.0;
            if(current_q_kneel_ != nullptr && current_q_pushup_ != nullptr) {
                interpolate(q_target, *current_q_pushup_, *current_q_kneel_, motion_timer_ / duration);
            }
            if(motion_timer_ > duration){
                getup_state_ = MyRobot::GetupState::STAND_UP;
                motion_timer_ = 0.0;
                cout << "KNEEL_UP completed" << endl;
            }

            break;
        case MyRobot::GetupState::STAND_UP:
            duration = 3.0;
            if(current_q_standup_ != nullptr && current_q_kneel_ != nullptr) {
                interpolate(q_target, *current_q_kneel_, *current_q_standup_, motion_timer_ / duration);
            }
            if(motion_timer_ > duration){
                getup_state_ = MyRobot::GetupState::FINISHED;
                motion_timer_ = 0.0;
                cout << "STAND_UP completed" << endl;
            }
            break;
        case MyRobot::GetupState::FINISHED:
           cout << "=== GETUP SEQUENCE COMPLETED ===" << endl;
            getup_state_ = MyRobot::GetupState::INACTIVE;
            fall_direction_ = FallDirection::UNKNOWN;
            
            // 参照をクリア（次回のために）
            current_q_tuck_ = nullptr;
            current_q_pushup_ = nullptr;
            current_q_kneel_ = nullptr;
            current_q_standup_ = nullptr;
            return;
        default:
            return;
    }

    for(int i=0; i<30; ++i){
        joint[i].q_ref = q_target[i];
    }

    motion_timer_ += timer.dt;
}

void MyRobot::startManualGetup(FallDirection direction){
    if(getup_state_ != MyRobot::GetupState::INACTIVE) {
        cout << "Getup already in progress, ignoring manual command." << endl;
        return;
    }
    
    cout << "=== MANUAL GETUP INITIATED ===" << endl;
    cout << "Direction: " << (direction == FallDirection::FACE_DOWN ? "FACE_DOWN" : "FACE_UP") << endl;
    
    // 手動で方向を設定
    fall_direction_ = direction;
    
    // 現在の関節角度を記録
    for(int i=0; i<30; ++i) {
        q_initial_[i] = joint[i].q;
    }
    
    // 指定された方向のキーフレームを選択
    selectKeyframesForDirection(fall_direction_);
    
    // 起き上がりシーケンス開始
    getup_state_ = MyRobot::GetupState::TUCK_UP;  // CHECK_POSEをスキップして直接TUCK_UPへ
    motion_timer_ = 0.0;
    
    cout << "Manual getup sequence started." << endl;
}


void MyRobot::Control(){
    Robot::Sense(timer, base, foot, joint);



    // calc FK
    fk_solver.Comp(param, joint, base, centroid, hand, foot);

    
    /*
        // [ADD] 転倒検知と制御の切り替えロジック
    Vector3 rpy = ToRollPitchYaw(base.ori);
    double pitch_angle = rpy.y() * 180.0 / M_PI;
     double roll_angle = rpy.x() * 180.0 / M_PI;


      // より敏感な転倒検知（ピッチとロール両方を考慮）
    const double fall_threshold = 60.0;  
    bool is_fallen = (std::abs(pitch_angle) > fall_threshold || 
                      std::abs(roll_angle) > fall_threshold);

    if (is_fallen && getup_state_ == MyRobot::GetupState::INACTIVE) {
        cout << "=== FALL DETECTED ===" << endl;
        cout << "Pitch: " << pitch_angle << " deg, Roll: " << roll_angle << " deg" << endl;
        cout << "Starting getup sequence..." << endl;
        getup_state_ = MyRobot::GetupState::CHECK_POSE;
    }
        // デバッグ用：現在の角度を定期的に表示
    static int debug_counter = 0;
    if(debug_counter % 100 == 0) {  // 100フレームに1回表示
        cout << "Current angles - Pitch: " << pitch_angle 
             << " deg, Roll: " << roll_angle 
             << " deg, State: " << (int)getup_state_ << endl;
    }
    debug_counter++;

    */
    
     


   

    // [ADD] 状態に応じた制御の分岐
    if (getup_state_ != MyRobot::GetupState::INACTIVE) {
        // --- 起き上がりモード ---
        updateGetupController();
        // 起き上がり中はIKソルバーを呼ばず、直接関節角度(q_ref)を指令する
    } else {
        // --- 通常モード (元のコードの処理) ---
        if (compStairStep && !PreAButtonState) {
            ground_rectangle.clear();
            ground_rectangle = fk_solver.FootToGroundFK(param, joint, base, foot, points_convex);
            int i = 0;
            for(Vector3& p : ground_rectangle){
                printf("id%d: %lf, %lf, %lf\n", i, p.x(), p.y(), p.z());
                i++;
            }
        }
        cout << "normal control" << endl;
        PreAButtonState = compStairStep;


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
		// 	    << joystick.getButtonState(Joystick::L_BUTTON) << " "
		// 	    << joystick.getButtonState(Joystick::R_BUTTON) << " "
        //         << joystick.getPosition(Joystick::DIRECTIONAL_PAD_V_AXIS) << " " 
        //         << joystick.getPosition(Joystick::DIRECTIONAL_PAD_H_AXIS) << " " <<std::endl;
	
		// erase current footsteps
		while(footstep.steps.size() > 2)
			footstep.steps.pop_back();

        // planning the desire landing potion and orientation by joystick input
        Robot::Operation(footstep.steps);

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
    stepping_controller.Update(timer, param, footstep, footstep_buffer, centroid, base, foot, compStairStep, ground_rectangle);
    
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
        
    }

	
    

	Robot::Actuate(timer, base, joint);
	
	timer.Countup();
}


}
}
