#ifndef MJPC_TASKS_QUADRUPED_RFMPC_3_DOF_H
#define MJPC_TASKS_QUADRUPED_RFMPC_3_DOF_H

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include <mutex> // Include the mutex header
#include <thread>

namespace mjpc {

class RFMPC_3_DOF : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
    public:
      explicit ResidualFn(const RFMPC_3_DOF* task)
        : mjpc::BaseResidualFn(task) {}
      ResidualFn(const ResidualFn&) = default;
      void Residual(const mjModel* model, const mjData* data,
                    double* residual) const override;

    private:
      friend class RFMPC_3_DOF;

      //  ============  enums  ============
      // modes
      enum RFMPC_3_DOFMode {
        kModeQuadruped = 0,
        kModeWalk,
        kModePath,
        kNumMode
      };

      // feet
      enum RFMPC_3_DOFFoot {
        kFootFL  = 0,
        kFootHL,
        kFootFR,
        kFootHR,
        kNumFoot
      };

      // gaits
      enum RFMPC_3_DOFGait {
        kGaitStand = 0,
        kGaitWalk,
        kGaitTrot,
        kGaitCanter,
        kGaitGallop,
        kNumGait
      };

      //  ============  constants  ============
      constexpr static RFMPC_3_DOFFoot kFootAll[kNumFoot] = {kFootFL, kFootHL,
                                                    kFootFR, kFootHR};
      constexpr static RFMPC_3_DOFFoot kFootHind[2] = {kFootHL, kFootHR};
      constexpr static RFMPC_3_DOFGait kGaitAll[kNumGait] = { kGaitStand, kGaitWalk, kGaitTrot};

      // gait phase signature (normalized)
      constexpr static double kGaitPhase[kNumGait][kNumFoot] =
      {
      // FL     HL     FR     HR
        {0,     0,     0,     0   },   // stand
        {0,     0.75,  0.5,   0.25},   // walk
        {0,     0.5,   0.5,   0   },   // trot
        {0,     0.33,  0.33,  0.66},   // canter
        {0,     0.4,   0.05,  0.35}    // gallop
      };

      // gait parameters, set when switching into gait
      constexpr static double kGaitParam[kNumGait][8] =
      {
      // duty ratio  cadence  amplitude  balance   upright   height   posture     consistency
      // unitless    Hz       meter      unitless  unitless  unitless unitless    unitless
        {1,          1,       0,         0.03,     1,        1,       1,        0.04},        // stand
        {0.75,       1,       0.015,     0.15,     1,        1,       0.2,        0.1},        // walk
        {0.45,       2,       0.015,     0.2,      1,        1,       0.2,        0.1},        // trot
        {0.4,        4,       0.025,     0.03,     0.5,      0.2,     0.2,        0.05},        // canter
        {0.3,        3.5,     0.05,      0.03,     0.2,      0.1,     0.2,        0.02}         // gallop
      };

      // velocity ranges for automatic gait switching, meter/second
      constexpr static double kGaitAuto[kNumGait] =
      {
        0,     // stand
        0.02,  // walk
        0.02,  // trot
        0.25,   // canter
        0.5,     // gallop
      };

      // notes:
      // - walk is never triggered by auto-gait
      // - canter actually has a wider range than gallop

      // automatic gait switching: time constant for com speed filter
      constexpr static double kAutoGaitFilter = 0.2;    // second

      // automatic gait switching: minimum time between switches
      constexpr static double kAutoGaitMinTime = 1;     // second

      // target torso height over feet when quadrupedal
      constexpr static double kHeightQuadruped = 0.22;  // meter

      // radius of foot geoms
      constexpr static double kFootRadius = 0.02;       // meter

      // below this target yaw velocity, walk straight
      constexpr static double kMinAngvel = 0.01;        // radian/second

      // posture gain factors for abduction, hip, knee
      constexpr static double kJointPostureGain[3] = {1, 1, 1};  // unitless

      //  ============  methods  ============
      // return internal phase clock
      double GetPhase(double time) const;

      // return current gait
      RFMPC_3_DOFGait GetGait() const;

      // compute average foot position, depending on mode
      void AverageFootPos(double avg_foot_pos[3],
                          double* foot_pos[kNumFoot]) const;

      // return normalized target step height
      double StepHeight(double time, double footphase, double duty_ratio) const;

      // compute target step height for all feet
      void FootStep(double step[kNumFoot], double time, RFMPC_3_DOFGait gait) const;

      // walk horizontal position given time
      void Walk(double pos[2], double time) const;

      // print residual
      void PrintResidual(double* residual, int size) const;

      //  ============  task state variables, managed by Transition  ============
      RFMPC_3_DOFMode current_mode_ = kModeQuadruped;
      double last_transition_time_ = -1;

      // common mode states
      double mode_start_time_  = 0;
      double position_[3]       = {0};

      // walk states
      double heading_[2]        = {0};
      double speed_             = 0;
      double angvel_            = 0;

      // // backflip states
      // double ground_            = 0;
      // double orientation_[4]    = {0};
      // double save_gait_switch_  = 0;
      // std::vector<double> save_weight_;

      // gait-related states
      double current_gait_      = 0;
      double phase_start_       = 0;
      double phase_start_time_  = 0;
      double phase_velocity_    = 0;
      double com_vel_[2]        = {0};
      double gait_switch_time_  = 0;

      // posture
      double current_posture_ = 0;

      // previous actuator values
      constexpr static int kNumActuator = 12;
      double prev_actuator_[kNumActuator] = {0};

      // path mode states
      int current_path_stage_ = 0;
      int num_path_stages_ = 15;

      //  ============  constants, computed in Reset()  ============
      int torso_body_id_        = -1;
      int head_site_id_         = -1;
      int goal_mocap_id_        = -1;
      int gait_param_id_        = -1;
      int gait_switch_param_id_ = -1;
      int cadence_param_id_     = -1;
      int amplitude_param_id_   = -1;
      int duty_param_id_        = -1;
      int path_stage_param_id_  = -1;
      int upright_cost_id_      = -1;
      int balance_cost_id_      = -1;
      int height_cost_id_       = -1;
      int posture_cost_id_       = -1;
      int consistency_cost_id_       = -1;

      int foot_geom_id_[kNumFoot];
      int shoulder_body_id_[kNumFoot];

      // // derived kinematic quantities describing flip trajectory
      double gravity_           = 9.81;
      // double jump_vel_          = 0;
      // double flight_time_       = 0;
      // double jump_acc_          = 0;
      // double crouch_time_       = 0;
      // double leap_time_         = 0;
      // double jump_time_         = 0;
      // double crouch_vel_        = 0;
      // double land_time_         = 0;
      // double land_acc_          = 0;
      // double flight_rot_vel_    = 0;
      // double jump_rot_vel_      = 0;
      // double jump_rot_acc_      = 0;
      // double land_rot_acc_      = 0;

  }; // class ResidualFn

  RFMPC_3_DOF() : residual_(this) {}

  void TransitionLocked(mjModel* model, mjData* data) override;

  // call base-class Reset, save task-related ids
  void ResetLocked(const mjModel* model) override;

  // draw task-related geometry in the scene
  void ModifyScene(const mjModel* model, const mjData* data,
                   mjvScene* scene) const override;

protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override {
    return std::make_unique<ResidualFn>(residual_);
  }

  ResidualFn* InternalResidual() override { return &residual_; }

private:
  friend class ResidualFn;
  ResidualFn residual_;

};

}  // namespace mjpc

#endif  // MJPC_TASKS_QUADRUPED_RFMPC_3_DOF_H_
