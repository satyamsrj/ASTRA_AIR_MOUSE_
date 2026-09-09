#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::vector;
using std::shared_ptr;
using std::unique_ptr;
using std::string;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH };

class FastExplorationFSM {
private:
  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  bool classic_;

  // FINISH-state recovery. Upstream FUEL treats FINISH as terminal: a single NO_FRONTIER
  // result ends the mission forever. That assumes the planner is first triggered inside an
  // already-partially-mapped space. This mission triggers it the instant the vehicle crosses
  // the arena threshold, when the interior is still unmapped, so the very first plan reliably
  // returns NO_FRONTIER and exploration never starts (observed 2026-09-05 08:42: PLAN_TRAJ ->
  // FINISH at t+34, then 82 consecutive FINISH iterations while the frontier finder was
  // simultaneously reporting 3 frontiers to visit).
  // These re-attempt planning a bounded number of times before accepting that exploration is
  // genuinely complete.
  double finish_recheck_interval_;
  int finish_recheck_max_;
  int finish_recheck_count_;
  // Latched once the FINISH re-checks are exhausted, i.e. exploration is genuinely over rather
  // than momentarily frontier-less. The mission layer (EDM) waits on /exploration_completed to
  // start the return leg; upstream FUEL never published anything, so RETURN was unreachable.
  bool exploration_completed_sent_;
  // Set by /mission/stop_exploration. The mission layer may decide exploration is
  // over before FUEL's own frontier logic does (coverage plateau); this is the
  // handshake that lets it stop us WITHOUT both of us driving /planning/pos_cmd.
  bool stop_requested_;
  ros::Time finish_last_recheck_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, stop_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_, completed_pub_;
  // The viewpoint this replan actually committed to. Published purely so a run can be analysed
  // afterwards: "Next view:" already went to std::cout, but with no timestamp, which makes it
  // impossible to line targets up against coverage or against where the vehicle actually was -
  // the exact correlation needed to tell a productive transit through mapped space from FUEL
  // re-targeting somewhere it has already finished.
  ros::Publisher next_view_pub_;

  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  // Clamp a measured (noisy) velocity to what the kinodynamic search will accept as a start
  // state; see the definition for the measurements behind it.
  Eigen::Vector3d clampStartVel(const Eigen::Vector3d& v) const;
  // Vertical half of the same guard. Needs the position, because the limit depends on how much
  // room is left above/below inside the planning box. See the definition.
  Eigen::Vector3d clampStartVelZ(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const;

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const nav_msgs::PathConstPtr& msg);
  void stopExplorationCallback(const std_msgs::BoolConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void visualize();
  void clearVisMarker();

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace fast_planner

#endif