#include "stepping_controller.h"

#include "robot.h"
#include "footstep.h"
#include "rollpitchyaw.h"

namespace cnoid{
namespace vnoid{

const double pi = 3.14159265358979;

SteppingController::SteppingController(){
    swing_height        = 0.05;
    swing_tilt          = 0.0;
    dsp_duration        = 0.05;
    descend_duration    = 0.0;
    descend_depth       = 0.0;
    landing_adjust_rate = 0.0;

    buffer_ready = false;
}

bool SteppingController::CheckLanding(const Timer& timer, Step& step, vector<Foot>& foot){
	// step duration elapsed
    double t = timer.time;
    if(t >= step.tbegin + step.duration){
        return true;
    }
    
    // force on landing foot exceeded threshold
    //if( t >= step.tbegin + step.duration - 0.1 && foot[!step.side].contact ){
    //    return true;
    //}
    
    return false;
}

// Quintic interpolate for swing trajectory on stairs: 2024/06: Tanaka
/*
    概要　　：階段昇降における遊脚軌道のｚ成分の計算．始点・終点に2つの経由点を加えた計4点における境界条件をもとに遊脚軌道を補完する．
            　境界条件として，始点・終点での位置・速度，各経由点での位置を与えるため，必要十分な次数を用いて5次補完を行う．
            　補完は連立６元１次方程式を解くことで行うことが可能であるが，始点における境界条件により，０次および１次項の係数が 0 となるため，連立４元１次方程式を解くことになる．
            　係数行列の掃き出し法によって連立n元１次方程式を解くプログラムを実装．
    補完式　：z = a0 + a1t + a2t^2 + a3t^3 + a4t^4 + a5t^5
    境界条件
        始点：位置 = 0, 速度 = 0
    経由点１：位置 = h1
    経由点２：位置 = h2
        終点：位置 = climb, 速度 = 0
    変数
         sm1：経由点１の通過時間
         sm2：経由点２の通過時間
          h1：経由点１の位置
          h2：経由点２の位置
*/

double SteppingController::QuinticInterpolate(double s, double sf, double climb){
    double sm1, sm2, h1, h2;
    int n = 4;
    if (climb < 0){
        sm1 = 0.2;
        sm2 = 0.5;
        h1  = 0.05;
        h2  = 0.05;
    }else{
        sm1 = 0.3;
        sm2 = 0.5;
        h1  = climb + 0.05;
        h2  = climb + 0.05;
    }

    double a[4][5] = {{  std::pow(sm1, 2),   std::pow(sm1, 3),   std::pow(sm1, 4),   std::pow(sm1, 5),    h1},
                      {  std::pow(sm2, 2),   std::pow(sm2, 3),   std::pow(sm2, 4),   std::pow(sm2, 5),    h2},
                      {  std::pow(sf, 2) ,   std::pow(sf, 3) ,   std::pow(sf, 4) ,   std::pow(sf, 5) , climb},
                      {2*std::pow(sf, 1) , 3*std::pow(sf, 2) , 4*std::pow(sf, 3) , 5*std::pow(sf, 4) ,     0}};

    for(int i=0;i<n;i++){
        int maxLine=i;
        for(int j=i; j<n; j++) if(abs(a[maxLine][i])<abs(a[j][i])) maxLine = j;
        for(int j=i; j<=n; j++) std::swap(a[i][j], a[maxLine][j]);

        long double beginNum = a[i][i];
        for(int j=i; j<=n; j++) a[i][j] /= beginNum;

        for(int j=i+1; j<n; j++){
            double beginDelete = a[j][i];
            for(int k=i; k<=n; k++) a[j][k] -= beginDelete * a[i][k];
        }
    }
    for(int i=n-1; i>=0; i--){
        for(int j=i+1; j<n; j++)a[i][n] -= a[j][n]*a[i][j];
    }

    double z = a[0][4] * std::pow(s, 2) + a[1][4] * std::pow(s, 3) + a[2][4] * std::pow(s, 4) + a[3][4] * std::pow(s, 5);
    return z;
}

void SteppingController::Update(const Timer& timer, const Param& param, Footstep& footstep, Footstep& footstep_buffer, Centroid& centroid, Base& base, vector<Foot>& foot, bool& compStairStep, vector<Vector3>& ground_rectangle){
    if(CheckLanding(timer, footstep_buffer.steps[0], foot)){
        if(footstep.steps.size() > 1){
            // pop step just completed from the footsteps
            footstep.steps.pop_front();
            if(footstep.steps.size() == 1){
		        printf("end of footstep reached\n");
                return;
	        }
        }

        footstep_buffer.steps[1].dcm = footstep_buffer.steps[0].dcm;
        footstep_buffer.steps.pop_front();
        footstep_buffer.steps.push_back(Step());
        footstep_buffer.steps[0].tbegin = timer.time;

        buffer_ready = false;
    }

    Step& st0  = footstep.steps[0];
    Step& st1  = footstep.steps[1];
    Step& stb0 = footstep_buffer.steps[0];
    Step& stb1 = footstep_buffer.steps[1];
    int sup =  st0.side;
    int swg = !st0.side;
	
    double T  = param.T;
    Vector3 offset(0.0, 0.0, param.com_height);
    
    if(!buffer_ready){
        // update support, lift-off, and landing positions
        stb0.side = st0.side;
        stb1.side = st1.side;

        stb0.stepping = st0.stepping;
        stb0.duration = st0.duration;

        stb0.foot_pos  [sup] = foot[sup].pos_ref;
        stb0.foot_angle[sup] = Vector3(0.0, 0.0, foot[sup].angle_ref.z());
        stb0.foot_ori  [sup] = FromRollPitchYaw(stb0.foot_angle[sup]);
        stb0.foot_pos  [swg] = foot[swg].pos_ref;
        stb0.foot_angle[swg] = Vector3(0.0, 0.0, foot[swg].angle_ref.z());
        stb0.foot_ori  [swg] = FromRollPitchYaw(stb0.foot_angle[swg]);
        stb0.dcm = centroid.dcm_ref;
    
        // landing position relative to support foot, taken from footsteps
        Quaternion ori_rel = st0.foot_ori[sup].conjugate()* st1.foot_ori[swg];
        Vector3    pos_rel = st0.foot_ori[sup].conjugate()*(st1.foot_pos[swg] - st0.foot_pos[sup]);
        Vector3    dcm_rel = st0.foot_ori[sup].conjugate()*(st1.dcm - st0.foot_pos[sup]);

        // calc absolute landing position
        stb1.foot_pos  [sup] = stb0.foot_pos  [sup];
        stb1.foot_ori  [sup] = stb0.foot_ori  [sup];
        stb1.foot_angle[sup] = stb0.foot_angle[sup];
        stb1.foot_pos  [swg] = stb0.foot_pos[sup] + stb0.foot_ori[sup]*pos_rel;
        stb1.foot_ori  [swg] = stb0.foot_ori[sup]*ori_rel;
        stb1.foot_angle[swg] = ToRollPitchYaw(stb1.foot_ori[swg]);
        stb1.dcm = stb0.foot_pos[sup] + stb0.foot_ori[sup]*dcm_rel;

        // calc zmp
        double alpha = exp(-stb0.duration/T);
        stb0.zmp = (1.0/(1.0 - alpha))*(stb0.dcm - alpha*stb1.dcm) - offset;

        buffer_ready = true;
    }

    if(footstep.steps.size() < 2){
		return;
	}

    // time to landing
    double ttl = stb0.tbegin + stb0.duration - timer.time;
	double alpha = exp(-ttl/T);
    
	stb0.dcm = (1.0-alpha)*(stb0.zmp + offset) + alpha*stb1.dcm;
    
    // landing adjustment based on dcm
    Vector3 land_rel = (st1.foot_pos[swg] - st0.foot_pos[sup]) - (stb0.dcm - stb0.foot_pos[sup]) + 0.0*(stb0.zmp - stb0.foot_pos[sup]); //centroid.dcm_ref
	stb1.foot_pos[swg].x() = centroid.dcm_ref.x() + land_rel.x();
	stb1.foot_pos[swg].y() = centroid.dcm_ref.y() + land_rel.y();

    // reference base orientation is set as the middle of feet orientation
    double angle_diff = foot[1].angle_ref.z() - foot[0].angle_ref.z();
    while(angle_diff >  pi) angle_diff -= 2.0*pi;
    while(angle_diff < -pi) angle_diff += 2.0*pi;
	base.angle_ref.z() = foot[0].angle_ref.z() + angle_diff/2.0;

    base.ori_ref   = FromRollPitchYaw(base.angle_ref);

    // set support foot position
    foot[sup].pos_ref     = stb0.foot_pos[sup];
    foot[sup].angle_ref   = stb0.foot_angle[sup];
    foot[sup].ori_ref     = FromRollPitchYaw(foot[sup].angle_ref);
    foot[sup].contact_ref = true;

    // set swing foot position
    if(!stb0.stepping || (timer.time - stb0.tbegin) < dsp_duration)
    {
        foot[swg].pos_ref     = stb0.foot_pos  [swg];
        foot[swg].angle_ref   = stb0.foot_angle[swg];
        foot[swg].ori_ref     = stb0.foot_ori  [swg];
        foot[swg].contact_ref = true;
    }
    else{
        double ts   = timer.time - (stb0.tbegin + dsp_duration);            //< time elapsed in ssp
        double tauv = stb0.duration - dsp_duration; //< duration of vertical movement
        double tauh = tauv - descend_duration;     //< duration of horizontal movement

        // cycloid swing profile
        double sv     = ts/tauv;
        double sh     = ts/tauh;
        double thetav = 2.0*pi*sv;
        double thetah = 2.0*pi*sh;
        double ch     = (sh < 1.0 ? (thetah - sin(thetah))/(2.0*pi) : 1.0);
        double cv     = (1.0 - cos(thetav))/2.0;
        double cv2    = (1.0 - cos(thetav/2.0))/2.0;
        double cw     = sin(thetah);

        // foot turning
        Vector3 turn = stb1.foot_angle[swg] - stb0.foot_angle[swg];
        while(turn.z() >  pi) turn.z() -= 2.0*pi;
        while(turn.z() < -pi) turn.z() += 2.0*pi;

        // foot tilting
        Vector3 tilt = stb0.foot_ori[swg]*Vector3(0.0, swing_tilt, 0.0);

        // calc horizontal component of the swing foot trajectory
        foot[swg].pos_ref.x()      = (1.0 - ch)*stb0.foot_pos[swg].x() + ch*stb1.foot_pos[swg].x();
        foot[swg].pos_ref.y()      = (1.0 - ch)*stb0.foot_pos[swg].y() + ch*stb1.foot_pos[swg].y();

        // calc vertical component of the swing foot trajectory
        double climb = stb1.foot_pos[swg].z() - stb0.foot_pos[swg].z();
        if (std::abs(climb) > 1.0e-02){  // for walking on stairs
            foot[swg].pos_ref.z()  = stb0.foot_pos[swg].z() + QuinticInterpolate(ts, tauv, climb);   // Quintic interpolation
            if (ts >= tauv - 0.005){
                compStairStep = false;  // 階段歩行が一歩完了したらfalseにする
            }
        } else { // for walking on horizontal surfaces or slightly uneven terrain
            foot[swg].pos_ref.z()  = (1.0 - ch)*stb0.foot_pos[swg].z() + ch*stb1.foot_pos[swg].z();   // cycloid
            foot[swg].pos_ref.z() += (cv*(swing_height + 0.5*descend_depth) - cv2*descend_depth);
        }
        
        foot[swg].angle_ref    = stb0.foot_angle[swg] + ch*turn + cw*tilt;
        foot[swg].ori_ref      = FromRollPitchYaw(foot[swg].angle_ref);
        foot[swg].contact_ref  = false;

        // adjust swing foot considering base link inclination
        Quaternion qrel = 
            FromRollPitchYaw(Vector3(base.angle_ref.x(), base.angle_ref.y(), base.angle_ref.z()))*
            FromRollPitchYaw(Vector3(base.angle    .x(), base.angle    .y(), base.angle_ref.z())).conjugate();
        Vector3 pivot   = centroid.zmp_ref;
        foot[swg].pos_ref   = qrel*(foot[swg].pos_ref - pivot) + pivot;
        foot[swg].ori_ref   = qrel* foot[swg].ori_ref;
        foot[swg].angle_ref = ToRollPitchYaw(foot[swg].ori_ref);

        /*
        // base link orientation error
        Quaternion base_rot = 
            FromRollPitchYaw(Vector3(base.angle_ref.x(), base.angle_ref.y(), base.angle_ref.z()))*
            FromRollPitchYaw(Vector3(base.angle    .x(), base.angle    .y(), base.angle_ref.z())).conjugate();

        // modify swing foot pose so that
        // relative position from support foot is rotated as much as base link orientation error
        foot[swg].pos_ref   = base_rot*(foot[swg].pos_ref - foot[sup].pos_ref) + foot[sup].pos_ref;
        foot[swg].ori_ref   = base_rot* foot[swg].ori_ref;
        foot[swg].angle_ref = ToRollPitchYaw(foot[swg].ori_ref);
        */
    }	
}

void SteppingController::AdjustTiming(const Timer& timer, const Param& param, const Centroid& centroid_pred, const Footstep& footstep, Footstep& footstep_buffer){
    const Step& st0  = footstep.steps[0];
    const Step& st1  = footstep.steps[1];
    Step& stb0 = footstep_buffer.steps[0];
    Step& stb1 = footstep_buffer.steps[1];
    int sup =  st0.side;
    int swg = !st0.side;

    // no adjustment during dsp or foot is not lifted
    if(!stb0.stepping || timer.time <= stb0.tbegin + dsp_duration){
        return;
    }
    // no adjustment right before landing
    if(timer.time > stb0.tbegin + stb0.duration - 0.1){
        return;
    }
	
    Vector3 land_min(-0.3, (swg == 0 ? -0.5 : 0.1), 0.0);
    Vector3 land_max( 0.3, (swg == 0 ? -0.1 : 0.5), 0.0);
    Vector3 dcm_offset = st0.foot_ori[sup].conjugate()*(st1.dcm - st1.foot_pos[swg]);
    Vector3 dcm_min = land_min + dcm_offset;
    Vector3 dcm_max = land_max + dcm_offset;

    // check if predicted DCM at landing is inside feasible region
    Vector3 dcm_local = st0.foot_ori[sup].conjugate()*(centroid_pred.dcm_ref - st0.foot_pos[sup]);
    bool inside = true;
    for(int j = 0; j < 2; j++){
        inside &= (dcm_min[j] <= dcm_local[j] && dcm_local[j] <= dcm_max[j]);
    }
    
    if(inside){
        stb0.duration = 0.5*(stb0.duration + st0.duration);
    }
    else{
        stb0.duration = std::max(stb0.duration*0.99, 0.3);
    }
    stb0.duration = std::max(stb0.duration, timer.time - stb0.tbegin);
}

}
}
