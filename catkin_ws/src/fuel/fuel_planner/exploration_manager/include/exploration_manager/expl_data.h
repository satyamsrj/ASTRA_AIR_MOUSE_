#ifndef _EXPL_DATA_H_
#define _EXPL_DATA_H_

#include <Eigen/Eigen>
#include <vector>
#include <bspline/Bspline.h>

using std::vector;
using Eigen::Vector3d;

namespace fast_planner {
struct FSMData {
  // FSM data
  bool trigger_, have_odom_, static_state_;
  vector<string> state_str_;

  Eigen::Vector3d odom_pos_, odom_vel_;  // odometry state
  Eigen::Quaterniond odom_orient_;
  double odom_yaw_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
  vector<Eigen::Vector3d> start_poss;
  bspline::Bspline newest_traj_;
};

struct FSMParam {
  double replan_thresh1_;
  double replan_thresh2_;
  double replan_thresh3_;
  double replan_time_;  // second
};

struct ExplorationData {
  vector<vector<Vector3d>> frontiers_;
  vector<vector<Vector3d>> dead_frontiers_;
  vector<pair<Vector3d, Vector3d>> frontier_boxes_;
  vector<Vector3d> points_;
  vector<Vector3d> averages_;
  vector<Vector3d> views_;
  vector<double> yaws_;
  vector<Vector3d> global_tour_;

  vector<int> refined_ids_;
  vector<vector<Vector3d>> n_points_;
  vector<Vector3d> unrefined_points_;
  vector<Vector3d> refined_points_;
  vector<Vector3d> refined_views_;  // points + dir(yaw)
  vector<Vector3d> refined_views1_, refined_views2_;
  vector<Vector3d> refined_tour_;

  Vector3d next_goal_;
  vector<Vector3d> path_next_goal_;

  // Target hysteresis state. planExploreMotion() is re-entered on every replan
  // (~0.2-1.5 s) and upstream recomputes the next viewpoint from scratch with no
  // memory of what it picked last time, so near-tied frontiers make the commanded
  // target flip continuously. These carry the previous choice across replans so it
  // can be defended against a merely-marginal challenger. See
  // PLANNING_DOCS/fuel_repeat_scan_root_cause_2026-09-04.md Sec 6.3.
  Vector3d last_next_pos_;
  double last_next_yaw_ = 0.0;
  bool has_last_target_ = false;
  // Consecutive "No path to next viewpoint" failures against the CURRENTLY held target. A
  // single failure is not evidence the target is bad -- the map is still filling in and A*
  // can miss on one cycle and succeed on the next -- so the hold is only dropped after a run
  // of them. See the no-path branch in planExploreMotion().
  int target_fail_streak_ = 0;

  // viewpoint planning
  // vector<Vector4d> views_;
  vector<Vector3d> views_vis1_, views_vis2_;
  vector<Vector3d> centers_, scales_;
};

struct ExplorationParam {
  // params
  bool refine_local_;
  int refined_num_;
  double refined_radius_;
  int top_view_num_;
  double max_decay_;
  string tsp_dir_;  // resource dir of tsp solver
  double relax_time_;

  // Target hysteresis (see ExplorationData above). A freshly computed viewpoint must
  // beat the previously committed one by more than target_switch_margin_ seconds of
  // ViewNode::computeCost before the vehicle is allowed to abandon its current target.
  // Setting the margin to 0.0 restores upstream's stateless behaviour exactly.
  double target_switch_margin_;
  // Consecutive no-path failures tolerated against a held target before it is released.
  int target_fail_limit_;
  double target_reached_dist_;
  double target_match_dist_;
};

}  // namespace fast_planner

#endif