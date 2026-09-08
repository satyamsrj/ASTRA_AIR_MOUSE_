#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

using Eigen::Vector3d;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;
using std::list;
using std::pair;

class RayCaster;

namespace fast_planner {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<Vector3d> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;
  // When the progress watchdog retired this cluster to the dormant list, or zero if it went
  // dormant the ordinary way (no viewpoint found). The dormant re-check resurrects any cluster
  // whose cells overlap the update box and have changed -- and the vehicle is parked right
  // beside the cluster it just gave up on, so that fires within a cycle and the cluster comes
  // straight back as a new id. Measured 20260908_091850: dormant oscillated 3->4->2->0 and the
  // same frontier was retired twice. This stamp holds a watchdog-retired cluster down for
  // retire_cooldown_ seconds so the planner actually moves on to a different one.
  ros::Time retired_at_;
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);
  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
                            vector<Vector3d>& averages);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
                         const double& max_decay, vector<vector<Vector3d>>& points,
                         vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
                         Eigen::MatrixXd& mat);
  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  // REQUEST that the frontier owning the viewpoint nearest to viewpoint_pos be retired to the
  // dormant list. Called after A* has failed a run of consecutive times to reach a COMMITTED
  // target (fast_exploration_manager.cpp target_fail_streak_): the normal dormancy path in
  // computeFrontiersToVisit only retires a cluster when NO viewpoint at all can be sampled for
  // it, so a cluster whose viewpoint is sampled but genuinely unreachable (behind a pinch A*
  // cannot cross) stays in frontiers_ and gets re-costed and re-targeted forever.
  //
  // This only RECORDS the request; the retirement itself happens at the top of the next
  // searchFrontiers(). That indirection is mandatory, not stylistic: every Frontier carries
  // costs_/paths_ lists holding one positional entry per OTHER frontier, and the only code that
  // may shrink frontiers_ is searchFrontiers(), because it records each removal's index in
  // removed_ids_ for updateFrontierCostMatrix() to purge the matching entry everywhere else.
  // Erasing from frontiers_ directly leaves those lists one element too long and the next
  // positional read runs off the end -- measured 2026-09-07: doing exactly that killed
  // exploration_node with SIGABRT 0.2 s after the first retirement fired.
  void requestRetireFrontierNear(const Vector3d& viewpoint_pos);
  // Is ANY viewpoint of ANY live frontier within tol of pos? Used by the target hysteresis to
  // ask "does my committed target still have something to see?" -- which must be answered
  // against every sampled viewpoint, not against the one-representative-per-frontier list the
  // caller happens to hold, because that list is re-picked relative to the vehicle's CURRENT
  // position and therefore slides as the vehicle flies.
  bool hasViewpointNear(const Vector3d& pos, const double& tol);
  void wrapYaw(double& yaw);

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(const Vector3d& min1, const Vector3d& max1, const Vector3d& min2,
                   const Vector3d& max2);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(const vector<Vector3d>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);

  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_;
  // Viewpoint positions whose owning frontier is to be retired at the next searchFrontiers().
  vector<Vector3d> pending_retire_;
  // Retirement is rate limited. Frontiers become unreachable one at a time as the map fills
  // in; many going unreachable at the same instant is a statement about the VEHICLE, not the
  // map, and must not be allowed to empty the frontier list and end the mission. See
  // retire_min_interval_.
  ros::Time last_retire_time_;
  double retire_min_interval_;
  // How long a watchdog-retired cluster stays dormant before the ordinary re-check may
  // resurrect it. Bounded rather than permanent: if the map genuinely opens a route in later,
  // the cluster should come back.
  double retire_cooldown_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  double min_candidate_z_, max_candidate_z_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

}  // namespace fast_planner
#endif