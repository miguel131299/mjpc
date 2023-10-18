#ifndef MJPC_TASKS_QUADRUPED_MULTISPINE_H
#define MJPC_TASKS_QUADRUPED_MULTISPINE_H

#include <string>
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include <mutex> // Include the mutex header
#include <thread>
#include <string_view>

namespace mjpc {

class MULTISPINE : public Task {
 public:
  std::string Name() const override;
  std::string XmlPath() const override;

  class ResidualFn : public mjpc::BaseResidualFn {
    public:
      explicit ResidualFn(const MULTISPINE* task)
        : mjpc::BaseResidualFn(task) {}

      ResidualFn(const ResidualFn&) = default;
      
      void Residual(const mjModel* model, const mjData* data,
                    double* residual) const override;

    private:
      friend class MULTISPINE;

      //  ============  enums  ============
      // modes
      enum MULTISPINEMode {
        kModeQuadruped = 0,
        kModeWalk,
        kModePath,
        kNumMode
      };

      // feet
      enum MULTISPINEFoot {
        kFootFL  = 0,
        kFootHL,
        kFootFR,
        kFootHR,
        kNumFoot
      };

      // gaits
      enum MULTISPINEGait {
        kGaitStand = 0,
        kGaitWalk,
        kGaitTrot,
        kGaitCanter,
        kGaitGallop,
        kNumGait
      };

            // posture names
      enum RFMPC_POSTURE {
        kPostureNormal = 0,
        kPostureLow,
        kPostureHigh,
        kPostureLeft,
        kPostureRight,
        kNumPosture
      };


      //  ============  constants  ============
      constexpr static MULTISPINEFoot kFootAll[kNumFoot] = {kFootFL, kFootHL,
                                                    kFootFR, kFootHR};
      constexpr static MULTISPINEFoot kFootHind[2] = {kFootHL, kFootHR};
      constexpr static MULTISPINEGait kGaitAll[kNumGait] = { kGaitStand, kGaitWalk, kGaitTrot};

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
        {1,          1,       0,         0.03,     1,        1,       1,        0.02},        // stand
        {0.75,       1,       0.015,     0.15,     1,        1,       0.2,        0.05},        // walk
        {0.45,       2,       0.015,     0.2,      1,        1,       0.2,        0.05},        // trot
        {0.4,        4,       0.025,     0.03,     0.5,      0.2,     0.2,        0.025},        // canter
        {0.3,        3.5,     0.05,      0.03,     0.2,      0.1,     0.2,        0.01}         // gallop
      };

      // velocity ranges for automatic gait switching, meter/second
      constexpr static double kGaitAuto[kNumGait] =
      {
        0,     // stand
        0.01,  // walk
        0.01,  // trot
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
      constexpr static double kHeightQuadruped[kNumPosture] = 
      {
        0.1885,   // low
        0.20,   // normal
        0.19,      // high
        0.20,   // left
        0.20    // right
      };  // meter
      // constexpr static double kHeightQuadruped = 0.20;  // meter

      // radius of foot geoms
      constexpr static double kFootRadius = 0.02;       // meter

      // below this target yaw velocity, walk straight
      constexpr static double kMinAngvel = 0.01;        // radian/second

      // posture gain factors for abduction, hip, knee
      constexpr static double kJointPostureGain[3] = {1, 1, 1};  // unitless

      constexpr static const char* kPostureNames[kNumPosture] = {
        "low", 
        "home", 
        "high",
        "left",
        "right"
        };

      constexpr static double kMinHeight = 0.15;
      constexpr static double kMediumHeight = 0.2;
      constexpr static double kMaxHeight = 0.25;

      //  ============  methods  ============
      // return internal phase clock
      double GetPhase(double time) const;

      // return current gait
      MULTISPINEGait GetGait() const;

      // return current posture
      const char* GetPosture() const;

      // compute average foot position, depending on mode
      void AverageFootPos(double avg_foot_pos[3],
                          double* foot_pos[kNumFoot]) const;

      // return normalized target step height
      double StepHeight(double time, double footphase, double duty_ratio) const;

      // compute target step height for all feet
      void FootStep(double step[kNumFoot], double time, MULTISPINEGait gait) const;

      // walk horizontal position given time
      void Walk(double pos[2], double time) const;

      // print residual
      void PrintResidual(double* residual, int size) const;

      double map(double value, double inMin, double inMax, double outMin, double outMax) const;

      //  ============  task state variables, managed by Transition  ============
      MULTISPINEMode current_mode_ = kModeQuadruped;
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

      // spinal angle
      double spinal_angle_ = 0;

      // previous actuator values
      constexpr static int kNumActuator = 20;
      double prev_actuator_[kNumActuator] = {0};

      // path mode states
      int current_path_stage_ = 0;
      int num_path_stages_ = 15;

      enum MULTISPINEJointIDs {
        kFrontLimbZHinge = 7,
        kFrontLimbYHinge,
        kAbductionFL,
        kHipFL,
        kKneeFL,
        kAbductionFR,
        kHipFR,
        kKneeFR,
        kHindLimbYHinge,
        kAbductionRL,
        kHipRL,
        kKneeRL,
        kAbductionRR,
        kHipRR,
        kKneeRR,
      };

      struct MULTISPINEJointValues {
        double min_value;
        double max_value;
      };

      MULTISPINEJointValues getJointValues(enum MULTISPINEJointIDs joint_id) const 
      {
        switch (joint_id)
        {
          case kFrontLimbYHinge:
            return {-0.42, 0.42};
          case kHindLimbYHinge:
            return {0.42, -0.42};
          // case kHipFL:
            return {0.45, 0};
          case kHipFR:
            return {0.45, 0};
          case kHipRL:
            return {-0.45, 0.7};
          case kHipRR:
            return {-0.45, 0.7};
          default:
            printf("ERROR: joint id shouldnt get interpolated\n");
            return {-1, 1};
        }
      }

      //  ============  constants, computed in Reset()  ============
      int torso_body_id_        = -1;
      int head_site_id_         = -1;
      int goal_mocap_id_        = -1;
      int gait_param_id_        = -1;
      int gait_switch_param_id_ = -1;
      int cadence_param_id_     = -1;
      int amplitude_param_id_   = -1;
      int duty_param_id_        = -1;
      int posture_param_id_     = -1;
      int spinal_angle_z_param_id_ = -1;
      int spinal_angle_y_param_id_ = -1;
      int chest_height_param_id_ = -1;
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

  MULTISPINE() : residual_(this) {}

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

#endif  // MJPC_TASKS_QUADRUPED_MULTISPINE_H_