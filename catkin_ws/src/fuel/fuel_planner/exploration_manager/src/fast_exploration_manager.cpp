// #include <fstream>
#include <exploration_manager/fast_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <lkh_tsp_solver/lkh_interface.h>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <active_perception/frontier_finder.h>
#include <plan_manage/planner_manager.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FastExplorationManager::FastExplorationManager() {
}

FastExplorationManager::~FastExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FastExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/target_switch_margin", ep_->target_switch_margin_, 1.0);
  nh.param("exploration/target_fail_limit", ep_->target_fail_limit_, 5);
  nh.param("exploration/target_fail_min_seconds", ep_->target_fail_min_seconds_, 3.0);
  nh.param("exploration/target_reached_dist", ep_->target_reached_dist_, 0.4);
  nh.param("exploration/target_match_dist", ep_->target_match_dist_, 2.0);
  nh.param("exploration/target_stale_seconds", ep_->target_stale_seconds_, 20.0);
  nh.param("exploration/target_progress_eps", ep_->target_progress_eps_, 0.15);
  nh.param("exploration/global_stale_seconds", ep_->global_stale_seconds_, 40.0);
  ed_->has_last_target_ = false;

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/single.par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ << "/single.tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE =" << ep_->tsp_dir_ << "/single.txt\n";
  par_file << "RUNS = 1\n";

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FastExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();
  ed_->global_tour_.clear();

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();

  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all frontier clusters and get visible ones' info
  frontier_finder_->computeFrontiersToVisit();
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    ed_->has_last_target_ = false;  // nothing left to commit to; drop any held target
    return NO_FRONTIER;
  }
  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  double view_time = (ros::Time::now() - t1).toSec();
  ROS_WARN(
      "Frontier: %d, t: %lf, viewpoint: %d, t: %lf", ed_->frontiers_.size(), frontier_time,
      ed_->points_.size(), view_time);

  // Do global and local tour planning and retrieve the next viewpoint
  Vector3d next_pos;
  double next_yaw;
  if (ed_->points_.size() > 1) {
    // Find the global tour passing through all viewpoints
    // Create TSP and solve by LKH
    // Optimal tour is returned as indices of frontier
    vector<int> indices;
    findGlobalTour(pos, vel, yaw, indices);

    if (ep_->refine_local_) {
      // Do refinement for the next few viewpoints in the global tour
      // Idx of the first K frontier in optimal tour
      t1 = ros::Time::now();

      ed_->refined_ids_.clear();
      ed_->unrefined_points_.clear();
      int knum = min(int(indices.size()), ep_->refined_num_);
      for (int i = 0; i < knum; ++i) {
        auto tmp = ed_->points_[indices[i]];
        ed_->unrefined_points_.push_back(tmp);
        ed_->refined_ids_.push_back(indices[i]);
        if ((tmp - pos).norm() > ep_->refined_radius_ && ed_->refined_ids_.size() >= 2) break;
      }

      // Get top N viewpoints for the next K frontiers
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, ed_->refined_ids_, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      ed_->refined_points_.clear();
      ed_->refined_views_.clear();
      vector<double> refined_yaws;
      refineLocalTour(pos, vel, yaw, ed_->n_points_, n_yaws, ed_->refined_points_, refined_yaws);
      next_pos = ed_->refined_points_[0];
      next_yaw = refined_yaws[0];

      // Get marker for view visualization
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        Vector3d view =
            ed_->refined_points_[i] + 2.0 * Vector3d(cos(refined_yaws[i]), sin(refined_yaws[i]), 0);
        ed_->refined_views_.push_back(view);
      }
      ed_->refined_views1_.clear();
      ed_->refined_views2_.clear();
      for (int i = 0; i < ed_->refined_points_.size(); ++i) {
        vector<Vector3d> v1, v2;
        frontier_finder_->percep_utils_->setPose(ed_->refined_points_[i], refined_yaws[i]);
        frontier_finder_->percep_utils_->getFOV(v1, v2);
        ed_->refined_views1_.insert(ed_->refined_views1_.end(), v1.begin(), v1.end());
        ed_->refined_views2_.insert(ed_->refined_views2_.end(), v2.begin(), v2.end());
      }
      double local_time = (ros::Time::now() - t1).toSec();
      ROS_WARN("Local refine time: %lf", local_time);

    } else {
      // Choose the next viewpoint from global tour
      next_pos = ed_->points_[indices[0]];
      next_yaw = ed_->yaws_[indices[0]];
    }
  } else if (ed_->points_.size() == 1) {
    // Only 1 destination, no need to find global tour through TSP
    frontier_finder_->updateFrontierCostMatrix();
    ed_->global_tour_ = { pos, ed_->points_[0] };
    ed_->refined_tour_.clear();
    ed_->refined_views1_.clear();
    ed_->refined_views2_.clear();

    if (ep_->refine_local_) {
      // Find the min cost viewpoint for next frontier
      ed_->refined_ids_ = { 0 };
      ed_->unrefined_points_ = { ed_->points_[0] };
      ed_->n_points_.clear();
      vector<vector<double>> n_yaws;
      frontier_finder_->getViewpointsInfo(
          pos, { 0 }, ep_->top_view_num_, ep_->max_decay_, ed_->n_points_, n_yaws);

      double min_cost = 100000;
      int min_cost_id = -1;
      vector<Vector3d> tmp_path;
      for (int i = 0; i < ed_->n_points_[0].size(); ++i) {
        auto tmp_cost = ViewNode::computeCost(
            pos, ed_->n_points_[0][i], yaw[0], n_yaws[0][i], vel, yaw[1], tmp_path);
        if (tmp_cost < min_cost) {
          min_cost = tmp_cost;
          min_cost_id = i;
        }
      }
      next_pos = ed_->n_points_[0][min_cost_id];
      next_yaw = n_yaws[0][min_cost_id];
      ed_->refined_points_ = { next_pos };
      ed_->refined_views_ = { next_pos + 2.0 * Vector3d(cos(next_yaw), sin(next_yaw), 0) };
    } else {
      next_pos = ed_->points_[0];
      next_yaw = ed_->yaws_[0];
    }
  } else
    ROS_ERROR("Empty destination.");

  // ---- Target hysteresis -------------------------------------------------------------
  // Everything above recomputes the next viewpoint from scratch on every replan and simply
  // takes whichever candidate is cheapest right now. Once the map is mostly explored the
  // surviving frontiers have near-identical cost, so millimetre-scale changes in pos/vel flip
  // which one wins and the vehicle chases a new heading every cycle instead of flying to any
  // of them. Measured on a clean 482 s flight: the commanded target changed on 12 of 12
  // consecutive 5 s samples, mean 1.1 m and 99 deg of yaw per change, yielding a persistent
  // ~5 s-period 1.1-1.6 deg roll/pitch oscillation and 15.7 deg mean yaw tracking error.
  //
  // Defend the committed target: only abandon it for a challenger that is better by more than
  // target_switch_margin_ seconds of cost. computeCost() returns seconds
  // (max(path_len/vm, yaw_diff/yd)), so the default 1.0 s is worth ~0.6 m of path or ~60 deg
  // of yaw at the configured limits. The hold releases on its own in three ways:
  //   1. the target is reached (within target_reached_dist_),
  //   2. no active viewpoint sits near it any more -- its frontier was cleared, so continuing
  //      would fly to a spot with nothing left to see,
  //   3. a genuinely better target appears (beats the margin).
  // Note the hold is self-reinforcing in the right way: as the vehicle turns toward the held
  // target its yaw cost falls, so a committed target naturally gets cheaper to keep.
  if (ep_->target_switch_margin_ > 0.0 && ed_->has_last_target_) {
    const bool reached = (ed_->last_next_pos_ - pos).norm() < ep_->target_reached_dist_;

    // Ask the frontier finder about EVERY sampled viewpoint, not ed_->points_. points_ holds
    // only the best-ranked viewpoint per frontier and getTopViewpointsInfo() re-picks those
    // relative to the CURRENT position, so they slide as the vehicle flies. Scanning that list
    // made a still-valid committed target look inactive once the vehicle had moved far enough
    // for every representative to drift past target_match_dist_, releasing the hold and
    // freeing the planner to switch -- which moved the vehicle further and released it again.
    // Measured 2026-09-07: from t=150 s the vehicle never again closed within 2.1 m of its
    // commanded target (median gap 7-8.6 m) while A* was still returning paths (only 11
    // no-path failures), churn 12.1, and coverage sat at 52.7% for 168 s.
    const bool still_active =
        frontier_finder_->hasViewpointNear(ed_->last_next_pos_, ep_->target_match_dist_);

    if (!reached && still_active) {
      vector<Vector3d> tmp_path;
      const double cost_last = ViewNode::computeCost(
          pos, ed_->last_next_pos_, yaw[0], ed_->last_next_yaw_, vel, yaw[1], tmp_path);
      const double cost_new =
          ViewNode::computeCost(pos, next_pos, yaw[0], next_yaw, vel, yaw[1], tmp_path);

      if (cost_new > cost_last - ep_->target_switch_margin_) {
        // Challenger is not decisively better -- keep flying to the committed target.
        next_pos = ed_->last_next_pos_;
        next_yaw = ed_->last_next_yaw_;
      }
    }
  }
  // ---- progress watchdog on the held target -------------------------------------------
  // The failure this catches is NOT a planning failure: A* returns a path every cycle, the
  // trajectory executes, and the vehicle flies -- it just never gets close enough to the
  // viewpoint to observe the frontier, so the cluster is never covered, never cleared, and the
  // planner re-selects it forever. Retirement therefore has to key off closing progress rather
  // than off no-path, which is what target_fail_streak_ already covers.
  //
  // A real traverse shrinks the distance every cycle, so its timer resets continuously and it
  // can run as long as it likes; only a target the vehicle cannot actually reach lets the clock
  // expire. Measured 20260907_213348: the target sat fixed for 220 s while the gap oscillated
  // 0.26-1.85 m and never improved.
  {
    const double dist_now = (next_pos - pos).norm();
    const bool same_target = ed_->has_last_target_ &&
        (next_pos - ed_->last_next_pos_).norm() < ep_->target_match_dist_;

    if (!same_target) {
      ed_->target_best_dist_ = dist_now;
      ed_->target_progress_time_ = ros::Time::now();
    } else if (dist_now < ed_->target_best_dist_ - ep_->target_progress_eps_) {
      ed_->target_best_dist_ = dist_now;
      ed_->target_progress_time_ = ros::Time::now();
    } else if (!ed_->target_progress_time_.isZero()) {
      const double stale_s = (ros::Time::now() - ed_->target_progress_time_).toSec();
      if (stale_s >= ep_->target_stale_seconds_) {
        ROS_WARN("[FSM] target held %.1f s without closing (best %.2f m, now %.2f m); "
                 "retiring it so the planner moves on.", stale_s, ed_->target_best_dist_,
                 dist_now);
        frontier_finder_->requestRetireFrontierNear(next_pos);
        ed_->target_progress_time_ = ros::Time::now();
        ed_->target_best_dist_ = 1e9;
      }
    }

    // Global stall clock. The per-target test above resets whenever the planner switches to a
    // target further than target_match_dist_ away, so a planner thrashing between several
    // unreachable viewpoints escapes it entirely. This clock ignores target identity and asks
    // only whether exploration advanced at all -- it is reset in the FSM when a cluster is
    // actually covered. Whatever target is held when it expires is the one blocking progress.
    if (ed_->last_progress_time_.isZero()) {
      ed_->last_progress_time_ = ros::Time::now();
    } else {
      const double idle_s = (ros::Time::now() - ed_->last_progress_time_).toSec();
      if (idle_s >= ep_->global_stale_seconds_) {
        ROS_WARN("[FSM] no frontier cluster covered for %.1f s; retiring the held target (%.2f "
                 "%.2f %.2f) and moving on.", idle_s, next_pos(0), next_pos(1), next_pos(2));
        frontier_finder_->requestRetireFrontierNear(next_pos);
        ed_->last_progress_time_ = ros::Time::now();
        ed_->target_progress_time_ = ros::Time::now();
        ed_->target_best_dist_ = 1e9;
      }
    }
  }

  ed_->last_next_pos_ = next_pos;
  ed_->last_next_yaw_ = next_yaw;
  ed_->has_last_target_ = true;
  // ---- end target hysteresis ---------------------------------------------------------

  std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

  // Plan trajectory (position and yaw) to the next viewpoint
  t1 = ros::Time::now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff = fabs(next_yaw - yaw[0]);
  double time_lb = min(diff, 2 * M_PI - diff) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos) != Astar::REACH_END) {
    ROS_ERROR("No path to next viewpoint");
    // Release the hysteresis hold before bailing out -- but NOT on the first failure. A
    // committed target that has genuinely become unreachable would otherwise be re-selected
    // and re-fail forever, so the hold has to break eventually; the original code broke it
    // immediately, which is too eager. Measured 2026-09-05: 1141 no-path failures in one
    // flight, each one disarming the anti-thrash hysteresis, so 26 of 95 target changes were
    // returns to targets already abandoned -- the vehicle ping-ponged between two attractors.
    // A* legitimately misses on one cycle and succeeds on the next while the map fills in, so
    // require a run of failures against the SAME target before giving up on it.
    if (ed_->target_fail_streak_ == 0) ed_->target_fail_streak_start_ = ros::Time::now();
    ++ed_->target_fail_streak_;
    const double streak_s = (ros::Time::now() - ed_->target_fail_streak_start_).toSec();
    if (ed_->target_fail_streak_ >= ep_->target_fail_limit_ &&
        streak_s >= ep_->target_fail_min_seconds_) {
      ROS_WARN("[FSM] target unreachable %d cycles over %.1f s; releasing the held target.",
               ed_->target_fail_streak_, streak_s);
      // Releasing the hold alone does not stop this: nothing removed the frontier from
      // frontiers_, so the very next ATSP cycle costs and re-picks the same viewpoint and A*
      // fails again -- measured 2026-09-07: one such frontier was targeted, abandoned by the
      // hysteresis-fail path above, and re-targeted in 52 separate episodes across a single
      // 1300 s run, most of it spent oscillating in place while coverage stalled at 91.2%.
      // Retiring the cluster to dormant here is what actually stops the loop; the existing
      // dormant re-check on map change resurrects it if it later becomes reachable.
      // Gated on the run of failures spanning target_fail_min_seconds_ of real time, not on
      // the cycle count alone. Replanning runs at tens of hertz, so a count-only test is met
      // in milliseconds whenever the vehicle simply cannot fly -- and a vehicle that cannot
      // fly fails A* to EVERY frontier at once, which retires the whole list and reports the
      // mission finished. Requiring the failures to persist over seconds keeps retirement a
      // statement about the frontier rather than about the airframe.
      frontier_finder_->requestRetireFrontierNear(next_pos);
      ed_->has_last_target_ = false;
      ed_->target_fail_streak_ = 0;
    }
    return FAIL;
  }
  // Reached here means A* found a route to the held target, so the failure run is over.
  ed_->target_fail_streak_ = 0;
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);

  const double radius_far = 5.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;

  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
    // if (!planner_manager_->kinodynamicReplan(
    //         pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
    //   return FAIL;
    // ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.1)
    ROS_ERROR("Lower bound not satified!");

  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);

  double traj_plan_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);
  double total = (ros::Time::now() - t2).toSec();
  ROS_WARN("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

void FastExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FastExplorationManager::findGlobalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
    vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->updateFrontierCostMatrix();
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/single.tsp");
  // Problem specification part, follow the format of TSPLIB

  string prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
      "\nEDGE_WEIGHT_TYPE : "
      "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";

  // string prob_spec = "NAME : single\nTYPE : TSP\nDIMENSION : " + to_string(dimension) +
  //     "\nEDGE_WEIGHT_TYPE : "
  //     "EXPLICIT\nEDGE_WEIGHT_FORMAT : LOWER_ROW\nEDGE_WEIGHT_SECTION\n";

  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  if (false) {
    // Use symmetric TSP
    for (int i = 1; i < dimension; ++i) {
      for (int j = 0; j < i; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }

  } else {
    // Use Asymmetric TSP
    for (int i = 0; i < dimension; ++i) {
      for (int j = 0; j < dimension; ++j) {
        int int_cost = cost_mat(i, j) * scale;
        prob_file << int_cost << " ";
      }
      prob_file << "\n";
    }
  }

  prob_file << "EOF";
  prob_file.close();

  // Call LKH TSP solver
  solveTSPLKH((ep_->tsp_dir_ + "/single.par").c_str());

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/single.txt");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  if (false) {
    // Read path for Symmetric TSP formulation
    getline(res_file, res);  // Skip current pose
    getline(res_file, res);
    int id = stoi(res);
    bool rev = (id == dimension);  // The next node is virutal depot?

    while (id != -1) {
      indices.push_back(id - 2);
      getline(res_file, res);
      id = stoi(res);
    }
    if (rev) reverse(indices.begin(), indices.end());
    indices.pop_back();  // Remove the depot

  } else {
    // Read path for ATSP formulation
    while (getline(res_file, res)) {
      // Read indices of frontiers in optimal tour
      int id = stoi(res);
      if (id == 1)  // Ignore the current state
        continue;
      if (id == -1) break;
      indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
    }
  }

  res_file.close();

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->global_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_WARN("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);
}

void FastExplorationManager::refineLocalTour(
    const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
    const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
    vector<Vector3d>& refined_pts, vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local tour graph: ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group)
        g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

}  // namespace fast_planner
