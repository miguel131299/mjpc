#include "mjpc/tasks/rfmpc/rfmpc_3_dof.h"

#include <string>
#include <thread>
#include <fstream>
#include <mutex> // Include the mutex header

#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/utilities.h"

namespace mjpc {

std::string RFMPC_3_DOF::XmlPath() const {
  return GetModelPath("rfmpc/task_3_dof.xml");
}
std::string RFMPC_3_DOF::Name() const { return "3 DOF RFMPC"; }

void RFMPC_3_DOF::ResidualFn::Residual(const mjModel* model,
                                         const mjData* data,
                                         double* residual) const {

  // printf("Residual\n");

  // std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // start counter
  int counter = 0;
  
  // get foot positions
  double* foot_pos[kNumFoot];
  for (RFMPC_3_DOFFoot foot : kFootAll) {
    foot_pos[foot] = data->geom_xpos + 3 * foot_geom_id_[foot];
  }  

  // average foot position
  double avg_foot_pos[3];
  AverageFootPos(avg_foot_pos, foot_pos);

  // xmat = cartesian orientation of body frame
  double* torso_xmat = data->xmat + 9 * torso_body_id_;
  double* goal_pos = data->mocap_pos + 3 * goal_mocap_id_;
  double* compos = SensorByName(model, data, "torso_subtreecom");

  // ------------ Upright --------------
  // printf("Upright\n");
  residual[counter++] = torso_xmat[8] - 1;
  residual[counter++] = 0;
  residual[counter++] = 0;

  // -------------- Height ---------------
  // height of torso over feet
  // xipos = cartesian position of body com
  // printf("Height\n");
  double* torso_pos = data->xipos + 3 * torso_body_id_;
  double height_goal = kHeightQuadruped;
  residual[counter++] = (torso_pos[2] - avg_foot_pos[2]) - height_goal;

  // -------------- Position ---------------
  // printf("Position\n");
  double* head = data->site_xpos + 3 * head_site_id_;
  double target[3];
  if (current_mode_ == ResidualFn::kModeWalk)
  {
    // follow prescribed Walk trajectory
    double mode_time = data->time - mode_start_time_;
    Walk(target, mode_time);
  }
  else {
    // go to the goal mocap body
    target[0] = goal_pos[0];
    target[1] = goal_pos[1];
    target[2] = goal_pos[2];
  }
  residual[counter++] = head[0] - target[0];
  residual[counter++] = head[1] - target[1];
  residual[counter++] = 0;

  // -------------- Gait ---------------

  // printf("Gait\n");

  RFMPC_3_DOFGait gait = GetGait();
  double step[kNumFoot];
  FootStep(step, GetPhase(data->time), gait);
  
  for (RFMPC_3_DOFFoot foot : kFootAll) {
    double query[3] = {foot_pos[foot][0], foot_pos[foot][1], foot_pos[foot][2]};
    double ground_height = Ground(model, data, query);
    double height_target = ground_height + kFootRadius + step[foot];
    double height_difference = foot_pos[foot][2] - height_target;

    residual[counter++] = step[foot] ? height_difference : 0;
  }

  // -------------- Balance ---------------
  // printf("Balance\n");
  double* comvel = SensorByName(model, data, "torso_subtreelinvel");
  double capture_point[3];
  double fall_time = mju_sqrt(2*height_goal/gravity_);
  mju_addScl3(capture_point, compos, comvel, fall_time);
  residual[counter++] = capture_point[0] - avg_foot_pos[0];
  residual[counter++] = capture_point[1] - avg_foot_pos[1];

  // -------------- Effort ---------------
  // nu = number of actuators
  // printf("Effort\n");
  mju_scl(residual + counter, data->actuator_force, 2e-2, model->nu);
  counter += model->nu;

  // -------------- Posture ---------------
  // printf("Posture\n");
  double* home = KeyQPosByName(model, data, "home");

  // + 7 because we only care about the joint positions
  mju_sub(residual + counter, data->qpos + 7, home + 7, model->nu);

  // TODO: Check dimensions of data->qpos and keyframe
  // set all joints to a certain value and then check MJData
  for (RFMPC_3_DOFFoot foot: kFootAll)
  {
    // robot has 3 joints per leg
    for (int joint = 0; joint < 3; joint++)
    {
      residual[counter + 2*foot + joint] *= kJointPostureGain[joint];
    }
  }
  counter += model->nu;

  // -------------- Yaw/Orientiation ---------------
  // printf("Yaw\n");
  if (current_mode_ == ResidualFn::kModePath)
  {
    double* goal_orientation = data->mocap_quat + 4 * goal_mocap_id_;
    double* orientation = SensorByName(model, data, "orientation");
    double geodesic_distance = 1.0 - mju_abs(mju_dot(goal_orientation, orientation, 4));
    residual[counter++] = geodesic_distance;
    residual[counter++] = geodesic_distance;
  } else {
    double torso_heading[2] = {torso_xmat[0], torso_xmat[3]};

    mju_normalize(torso_heading, 2);
    double heading_goal = parameters_[ParameterIndex(model, "Heading")];
    residual[counter++] = torso_heading[0] - mju_cos(heading_goal);
    residual[counter++] = torso_heading[1] - mju_sin(heading_goal);
  }
  
  // ------------- Angular momentum --------------
  // printf("Angular momentum\n");
  mju_copy3(residual + counter, SensorByName(model, data, "torso_angmom"));
  counter += 3;

  // ------------ Consistency ----------
  double actuator_sum = 0;
  for (int i = 0; i < model->nu; i++)
  {
    // for each actuator, calculate the difference between the current and previous actuator value
    // square it and add it to the residual
    double actuator_difference = data->ctrl[i] - prev_actuator_[i];
    actuator_sum += actuator_difference * actuator_difference;
  }

  residual[counter++] = actuator_sum * 1e-2;


  CheckSensorDim(model, counter);

  // printf("Residual end\n");

  // PrintResidual(residual, counter);
}

//  ============  transition  ============
void RFMPC_3_DOF::TransitionLocked(mjModel* model, mjData* data) {

  // printf("Transition\n");


  // ----------- handle mjData reset -------------------
  if (data->time < residual_.last_transition_time_ || 
      residual_.last_transition_time_ == -1) {
    if (mode != ResidualFn::kModeQuadruped) {
      mode = ResidualFn::kModeQuadruped;    // mode is stateful, switch to Quadruped
    }
    residual_.last_transition_time_ = residual_.phase_start_time_ 
        =residual_. phase_start_ = data->time;
  }  

  // ----------- prevent forbiddden mode transitions -------------------
  // switching mode, not from quadruped
  if (mode != residual_.current_mode_ && 
      residual_.current_mode_ != ResidualFn::kModeQuadruped)
  {
    // switch into stateful mode only allowed from Quadruped
    if (mode == ResidualFn::kModeWalk)
    {
      mode = ResidualFn::kModeQuadruped;
    }
  }

  // ---------------- handle phase velocity change --------------------
  double phase_velocity = 2 * mjPI * parameters[residual_.cadence_param_id_];
  if (phase_velocity != residual_.phase_velocity_) {
    // phase velocity changed, update phase start time
    residual_.phase_start_ = residual_.GetPhase(data->time);
    residual_.phase_start_time_ = data->time;
    residual_.phase_velocity_ = phase_velocity;
  }

  
  // ----------------- automatic gait switching ----------------
  double* comvel = SensorByName(model, data, "torso_subtreelinvel");
  double beta = mju_exp(-(data->time - residual_.last_transition_time_) /
                        ResidualFn::kAutoGaitFilter);
  residual_.com_vel_[0] = beta * residual_.com_vel_[0] + (1 - beta) * comvel[0];
  residual_.com_vel_[1] = beta * residual_.com_vel_[1] + (1 - beta) * comvel[1];

  int auto_swich = ReinterpretAsInt(parameters[residual_.gait_switch_param_id_]);
  if (auto_swich)
  {
    double com_speed = mju_norm(residual_.com_vel_, 2);
    for (int64_t gait: ResidualFn::kGaitAll)
    {
      bool lower = com_speed > ResidualFn::kGaitAuto[gait];
      bool upper = gait == ResidualFn::kGaitGallop ||
                   com_speed <= ResidualFn::kGaitAuto[gait + 1];
      bool wait = mju_abs(residual_.gait_switch_time_ - data->time) >
                  ResidualFn::kAutoGaitMinTime;

      if (lower && upper && wait) {
        parameters[residual_.gait_param_id_] = ReinterpretAsDouble(gait);
        residual_.gait_switch_time_ = data->time;
      }
    }
  }
  
  // ---------------- handle gait switch, manual or auto -------
  double gait_selection = parameters[residual_.gait_param_id_];
  if (gait_selection != residual_.current_gait_) {
    residual_.current_gait_ = gait_selection;
    ResidualFn::RFMPC_3_DOFGait gait = residual_.GetGait();
    parameters[residual_.duty_param_id_] = ResidualFn::kGaitParam[gait][0];
    parameters[residual_.cadence_param_id_] = ResidualFn::kGaitParam[gait][1];
    parameters[residual_.amplitude_param_id_] = ResidualFn::kGaitParam[gait][2];
    weight[residual_.balance_cost_id_] = ResidualFn::kGaitParam[gait][3];
    weight[residual_.upright_cost_id_] = ResidualFn::kGaitParam[gait][4];
    weight[residual_.height_cost_id_] = ResidualFn::kGaitParam[gait][5];
    weight[residual_.posture_cost_id_] = ResidualFn::kGaitParam[gait][6];
    weight[residual_.consistency_cost_id_] = ResidualFn::kGaitParam[gait][7];
  }

  // ------------- Walk ----------------
  double* goal_pos = data->mocap_pos + 3 * residual_.goal_mocap_id_;
  if (mode == ResidualFn::kModeWalk)
  {
    double angvel = parameters[ParameterIndex(model, "Walk turn")];
    double speed = parameters[ParameterIndex(model, "Walk speed")];

    // current torso direction
    double* torso_xmat = data->xmat + 9 * residual_.torso_body_id_;
    double forward[2] = {torso_xmat[0], torso_xmat[3]};
    mju_normalize(forward, 2);
    double leftward[2] = {-forward[1], forward[0]};

    // switching into Walk or parameters changed, reset task state
    if (mode != residual_.current_mode_ || residual_.angvel_ != angvel || 
        residual_.speed_ != speed)
    {
      // save time
      residual_.mode_start_time_ = data->time;

      // save current speed and angvel
      residual_.speed_ = speed;
      residual_.angvel_ = angvel;

      // compute and save rotation axis / walk origin
      double axis[2] = {data->xpos[3*residual_.torso_body_id_],
                        data->xpos[3*residual_.torso_body_id_ + 1]};
      
      if (mju_abs(angvel) > ResidualFn::kMinAngvel) {
        // don't allow turning with very small angvel
        double d = speed / angvel;
        axis[0] += d * leftward[0];
        axis[1] += d * leftward[1];
      }
      residual_.position_[0] = axis[0];
      residual_.position_[1] = axis[1];

      // save vector from axis to initial goal position
      residual_.heading_[0] = goal_pos[0] - axis[0];
      residual_.heading_[1] = goal_pos[1] - axis[1];
    }

    // move goal
    double time = data->time - residual_.mode_start_time_;
    residual_.Walk(goal_pos, time);
  }

  // ------------ Path ----------------
  if (mode == ResidualFn::kModePath) 
  {

    int path_stage = ReinterpretAsInt(parameters[residual_.path_stage_param_id_]);

    if (path_stage > 0)
    {
      residual_.current_path_stage_ = path_stage - 1;
    } else {

      // goal orientation
      const double* goal_orientation = data->mocap_quat + 3 * residual_.goal_mocap_id_;

      // robot position
      double* head_pos = data->site_xpos + 3 * residual_.head_site_id_;

      // robot orientation
      double* orientation = SensorByName(model, data, "orientation");

      // position error
      double position_error[3];
      mju_sub3(position_error, head_pos, goal_pos);
      double position_error_norm = mju_norm3(position_error);

      // orientation error
      double geodesic_distance =
          1.0 - mju_abs(mju_dot(goal_orientation, orientation, 4));


      // ---------- Check tolerance ----------
      double tolerance = 1.5e-1;
      if (position_error_norm <= tolerance && geodesic_distance <= tolerance) {
        // update task state
        residual_.current_path_stage_ += 1;
        if (residual_.current_path_stage_ == residual_.num_path_stages_) {
          residual_.current_path_stage_ = 0;
        }
      }
    }
    
    // ---------- Set goal ----------
    mju_copy3(data->mocap_pos, model->key_mpos + 3 * residual_.current_path_stage_);
    mju_copy4(data->mocap_quat, model->key_mquat + 4 * residual_.current_path_stage_);
  }

  // save current mode
  residual_.current_mode_ = static_cast<ResidualFn::RFMPC_3_DOFMode>(mode);
  residual_.last_transition_time_ = data->time;
  
}

// colors of visualisation elements drawn in ModifyScene()
constexpr float kStepRgba[4] = {0.6, 0.8, 0.2, 1};  // step-height cylinders
constexpr float kHullRgba[4] = {0.4, 0.2, 0.8, 1};  // convex hull
constexpr float kAvgRgba[4] = {0.4, 0.2, 0.8, 1};   // average foot position
constexpr float kCapRgba[4] = {0.3, 0.3, 0.8, 1};   // capture point
constexpr float kPcpRgba[4] = {0.5, 0.5, 0.2, 1};   // projected capture point

// draw task-related geometry in the scene
void RFMPC_3_DOF::ModifyScene(const mjModel* model, const mjData* data,
                           mjvScene* scene) const {
  // printf("ModifyScene\n");

  // current foot positions
  double* foot_pos[ResidualFn::kNumFoot];
  for (ResidualFn::RFMPC_3_DOFFoot foot : ResidualFn::kFootAll) {
    foot_pos[foot] = data->geom_xpos + 3 * residual_.foot_geom_id_[foot];
  }

  // stance and flight positions
  double flight_pos[ResidualFn::kNumFoot][3];
  double stance_pos[ResidualFn::kNumFoot][3];
  // set to foot horizontal position:
  for (ResidualFn::RFMPC_3_DOFFoot foot : ResidualFn::kFootAll) 
  {
    flight_pos[foot][0] = stance_pos[foot][0] = foot_pos[foot][0];
    flight_pos[foot][1] = stance_pos[foot][1] = foot_pos[foot][1];
  }

  // ground height below feet
  double ground[ResidualFn::kNumFoot];
  for (ResidualFn::RFMPC_3_DOFFoot foot : ResidualFn::kFootAll) {
    ground[foot] = Ground(model, data, foot_pos[foot]);
  }

  // step heights
  ResidualFn::RFMPC_3_DOFGait gait = residual_.GetGait();
  double step[ResidualFn::kNumFoot];
  residual_.FootStep(step, residual_.GetPhase(data->time), gait);

  // draw step height
  for (ResidualFn::RFMPC_3_DOFFoot foot : ResidualFn::kFootAll) {
    stance_pos[foot][2] = ResidualFn::kFootRadius + ground[foot];

    if (step[foot])
    {
      flight_pos[foot][2] =ResidualFn:: kFootRadius + step[foot] + ground[foot];
      AddConnector(scene, mjGEOM_CYLINDER, ResidualFn::kFootRadius, 
                    stance_pos[foot], flight_pos[foot], kStepRgba);
    }
  }

  // support polygon (currently unused for cost)
  double polygon[2*ResidualFn::kNumFoot];
  for (ResidualFn::RFMPC_3_DOFFoot foot : ResidualFn::kFootAll) {
    polygon[2*foot] = foot_pos[foot][0];
    polygon[2*foot + 1] = foot_pos[foot][1];
  }
  int hull[ResidualFn::kNumFoot];
  int num_hull = Hull2D(hull, ResidualFn::kNumFoot, polygon);
  for (int i=0; i < num_hull; i++) {
    int j = (i + 1) % num_hull;
    AddConnector(scene, mjGEOM_CAPSULE, ResidualFn::kFootRadius/2,
                 stance_pos[hull[i]], stance_pos[hull[j]], kHullRgba);
  }

  // capture point
  double height_goal = ResidualFn::kHeightQuadruped;
  double fall_time = mju_sqrt(2*height_goal/residual_.gravity_);
  double capture[3];
  double* compos = SensorByName(model, data, "torso_subtreecom");
  double* comvel = SensorByName(model, data, "torso_subtreelinvel");
  mju_addScl3(capture, compos, comvel, fall_time);

  // ground under CoM
  double com_ground = Ground(model, data, compos);

  // average foot position
  double feet_pos[3];
  residual_.AverageFootPos(feet_pos, foot_pos);
  feet_pos[2] = com_ground;

  double foot_size[3] = {ResidualFn::kFootRadius, 0, 0};

  // average foot position
  AddGeom(scene, mjGEOM_SPHERE, foot_size, feet_pos, /*mat=*/ nullptr, kAvgRgba);

  // capture point
  capture[2] = com_ground;
  AddGeom(scene, mjGEOM_SPHERE, foot_size, capture, /*mat=*/ nullptr, kCapRgba);

  // capture point, projected onto hull
  double pcp2[2];
  NearestInHull(pcp2, capture, polygon, hull, num_hull);
  double pcp[3] = {pcp2[0], pcp2[1], com_ground};
  AddGeom(scene, mjGEOM_SPHERE, foot_size, pcp, /*mat=*/ nullptr, kPcpRgba);
}

void RFMPC_3_DOF::ResetLocked(const mjModel* model) {
  // printf("Reset\n");
  // call method from base class

  // ----------- task identifiers ---------------
  residual_.gait_param_id_ = ParameterIndex(model, "select_Gait");

  residual_.gait_switch_param_id_ = ParameterIndex(model, "select_Gait switch");
  residual_.cadence_param_id_ = ParameterIndex(model, "Cadence");
  residual_.amplitude_param_id_ = ParameterIndex(model, "Amplitude");
  residual_.duty_param_id_ = ParameterIndex(model, "Duty ratio");
  residual_.path_stage_param_id_ = ParameterIndex(model, "select_Path stage");

  
  residual_.balance_cost_id_ = CostTermByName(model, "Balance");
  residual_.upright_cost_id_ = CostTermByName(model, "Upright");
  residual_.height_cost_id_ = CostTermByName(model, "Height");
  residual_.consistency_cost_id_ = CostTermByName(model, "Consistency");

  // ------------ model identifiers ---------------
  residual_.torso_body_id_ = mj_name2id(model, mjOBJ_XBODY, "main_body");
  if (residual_.torso_body_id_ < 0) mju_error("body 'main_body' not found");

  residual_.head_site_id_ = mj_name2id(model, mjOBJ_SITE, "head");
  if (residual_.head_site_id_ < 0) mju_error("site 'head' not found");

  int goal_id = mj_name2id(model, mjOBJ_BODY, "goal");
  if (goal_id < 0) mju_error("body 'goal' not found");

  residual_.goal_mocap_id_ = model->body_mocapid[goal_id];
  if (residual_.goal_mocap_id_ < 0) mju_error("body 'goal' is not mocap");

  // foot geom ids
  int foot_index = 0;
  for (const char* footname: {"Foot_FL", "Foot_RL", "Foot_FR", "Foot_RR"}) {
    int foot_id = mj_name2id(model, mjOBJ_GEOM, footname);
    if (foot_id < 0) mju_error("geom '%s' not found", footname);
    residual_.foot_geom_id_[foot_index] = foot_id;
    foot_index++;
  }

  // shoulder body ids
  int shoulder_index = 0;
  for (const char* shouldername: {"FL", "RL", "FR", "RR"}) {
    int shoulder_id = mj_name2id(model, mjOBJ_BODY, shouldername);
    if (shoulder_id < 0) mju_error("body '%s' not found", shouldername);
    residual_.shoulder_body_id_[shoulder_index] = shoulder_id;
    shoulder_index++;
  }
}

// compute average foot position, depending on mode
void RFMPC_3_DOF::ResidualFn::AverageFootPos(double avg_foot_pos[3],
                              double* foot_pos[kNumFoot]) const {
  mju_add3(avg_foot_pos, foot_pos[kFootHL], foot_pos[kFootHR]);
  mju_addTo3(avg_foot_pos, foot_pos[kFootFL]);
  mju_addTo3(avg_foot_pos, foot_pos[kFootFR]);
  mju_scl3(avg_foot_pos, avg_foot_pos, 0.25);
  
}

// return phase as a function of time
double RFMPC_3_DOF::ResidualFn::GetPhase(double time) const {
  return phase_start_ + (time - phase_start_time_) * phase_velocity_;
}

// horizontal Walk trajectory
void RFMPC_3_DOF::ResidualFn::Walk(double pos[2], double time) const {
  if (mju_abs(angvel_) < kMinAngvel) {
    // no rotation, go in straight line
    double forward[2] = {heading_[0], heading_[1]};
    mju_normalize(forward, 2);
    pos[0] = position_[0] + heading_[0] + time*speed_*forward[0];
    pos[1] = position_[1] + heading_[1] + time*speed_*forward[1];
  } else {
    // walk on a circle
    double angle = time * angvel_;
    double mat[4] = {mju_cos(angle), -mju_sin(angle),
                     mju_sin(angle),  mju_cos(angle)};
    mju_mulMatVec(pos, mat, heading_, 2, 2);
    pos[0] += position_[0];
    pos[1] += position_[1];
  }
}

// get gait
RFMPC_3_DOF::ResidualFn::RFMPC_3_DOFGait RFMPC_3_DOF::ResidualFn::GetGait() const {
  return static_cast<RFMPC_3_DOFGait>(ReinterpretAsInt(current_gait_));
}

// return normalized target step height
double RFMPC_3_DOF::ResidualFn::StepHeight(double time, double footphase,
                                 double duty_ratio) const {
  double foot_angle = fmod(time + mjPI - footphase, 2*mjPI) - mjPI;
  double step_height = 0;
  if (duty_ratio < 1) {
    foot_angle *= 0.5 / (1 - duty_ratio);
    step_height = mju_cos(mju_clip(foot_angle, -mjPI/2, mjPI/2));
  }

  return mju_abs(step_height) < 1e-6 ? 0.0 : step_height;
}

// compute target step height for all feet
void RFMPC_3_DOF::ResidualFn::FootStep(double target_step_height[kNumFoot], double time,
                             RFMPC_3_DOFGait gait) const {
  double amplitude = parameters_[amplitude_param_id_];
  double duty_ratio = parameters_[duty_param_id_];
  for (RFMPC_3_DOFFoot foot : kFootAll) {
    double footphase = 2*mjPI*kGaitPhase[gait][foot];
    target_step_height[foot] = amplitude * StepHeight(time, footphase, duty_ratio);
  }
}

}  // namespace mjpc