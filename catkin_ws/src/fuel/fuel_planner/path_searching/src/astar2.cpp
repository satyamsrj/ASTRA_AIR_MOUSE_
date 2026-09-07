#include <path_searching/astar2.h>
#include <sstream>
#include <plan_env/sdf_map.h>
#include <algorithm>
#include <cmath>
#include <limits>

using namespace std;
using namespace Eigen;

namespace fast_planner {
Astar::Astar() {
}

Astar::~Astar() {
  for (int i = 0; i < allocate_num_; i++)
    delete path_node_pool_[i];
}

void Astar::init(ros::NodeHandle& nh, const EDTEnvironment::Ptr& env) {
  nh.param("astar/resolution_astar", resolution_, -1.0);
  nh.param("astar/lambda_heu", lambda_heu_, -1.0);
  nh.param("astar/max_search_time", max_search_time_, -1.0);
  nh.param("astar/allocate_num", allocate_num_, -1);
  // Default sized to just over one inflation radius: enough to step out of the inflated band
  // around a wall, not enough to jump the vehicle through one.
  nh.param("astar/escape_radius", escape_radius_, 0.6);

  tie_breaker_ = 1.0 + 1.0 / 1000;

  this->edt_env_ = env;

  /* ---------- map params ---------- */
  this->inv_resolution_ = 1.0 / resolution_;
  edt_env_->sdf_map_->getRegion(origin_, map_size_3d_);
  cout << "origin_: " << origin_.transpose() << endl;
  cout << "map size: " << map_size_3d_.transpose() << endl;

  path_node_pool_.resize(allocate_num_);
  for (int i = 0; i < allocate_num_; i++) {
    path_node_pool_[i] = new Node;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
  early_terminate_cost_ = 0.0;
}

void Astar::setResolution(const double& res) {
  resolution_ = res;
  this->inv_resolution_ = 1.0 / resolution_;
}

bool Astar::isAdmissible(const Eigen::Vector3d& pt) {
  return edt_env_->sdf_map_->isInBox(pt) &&
      edt_env_->sdf_map_->getInflateOccupancy(pt) != 1 &&
      edt_env_->sdf_map_->getOccupancy(pt) != SDFMap::UNKNOWN;
}

bool Astar::findNearestAdmissible(const Eigen::Vector3d& from, Eigen::Vector3d& out) {
  // Expanding shells at the search resolution. Deliberately capped: if nothing admissible is
  // within escape_radius_ the vehicle is genuinely lost and silently teleporting the plan
  // origin several metres would be worse than reporting no path.
  const int max_ring = std::max(1, int(std::ceil(escape_radius_ / resolution_)));
  double best_d = std::numeric_limits<double>::max();
  bool found = false;
  for (int r = 1; r <= max_ring; ++r) {
    for (int i = -r; i <= r; ++i)
      for (int j = -r; j <= r; ++j)
        for (int k = -r; k <= r; ++k) {
          // shell only -- interior rings were covered by earlier iterations
          if (std::max({ std::abs(i), std::abs(j), std::abs(k) }) != r) continue;
          Eigen::Vector3d cand = from + resolution_ * Eigen::Vector3d(i, j, k);
          if (!isAdmissible(cand)) continue;
          double d = (cand - from).norm();
          if (d < best_d) { best_d = d; out = cand; found = true; }
        }
    if (found) return true;   // nearest shell containing anything admissible wins
  }
  return false;
}

int Astar::search(const Eigen::Vector3d& start_pt, const Eigen::Vector3d& end_pt) {
  // A* only ever tests NEIGHBOURS for admissibility, never the start itself. When the vehicle
  // is parked inside the inflated obstacle band every neighbour is rejected, the open set
  // empties immediately and this returns NO_PATH on every single replan -- with no log line
  // distinguishing it from a genuinely disconnected goal.
  //
  // 2026-09-05, measured: after entering this arena the vehicle sat 0.340 m from a wall (inside
  // its own 0.384 m radius, in a 0.63 m corridor) and produced 33320 consecutive "No path to
  // next viewpoint" against 2 successful plans. Nudging the plan origin to the nearest
  // admissible cell is the general fix; tuning where the entry module stops the vehicle is not,
  // because it re-breaks whenever the arena geometry changes.
  //
  // The true start is prepended to the path afterwards, so the returned trajectory still begins
  // at the vehicle and the caller sees no discontinuity.
  Eigen::Vector3d search_start = start_pt;
  const bool nudged = !isAdmissible(start_pt);
  if (nudged && !findNearestAdmissible(start_pt, search_start)) {
    ROS_WARN_THROTTLE(2.0,
                      "[Astar] start (%.2f %.2f %.2f) is blocked and no admissible cell within "
                      "%.2f m; cannot plan.",
                      start_pt.x(), start_pt.y(), start_pt.z(), escape_radius_);
    return NO_PATH;
  }
  if (nudged) {
    ROS_WARN_THROTTLE(2.0,
                      "[Astar] start (%.2f %.2f %.2f) was inside the inflated band; planning "
                      "from nearest free cell (%.2f %.2f %.2f), %.2f m away.",
                      start_pt.x(), start_pt.y(), start_pt.z(), search_start.x(),
                      search_start.y(), search_start.z(), (search_start - start_pt).norm());
  }

  NodePtr cur_node = path_node_pool_[0];
  cur_node->parent = NULL;
  cur_node->position = search_start;
  posToIndex(search_start, cur_node->index);
  cur_node->g_score = 0.0;
  cur_node->f_score = lambda_heu_ * getDiagHeu(cur_node->position, end_pt);

  Eigen::Vector3i end_index;
  posToIndex(end_pt, end_index);

  open_set_.push(cur_node);
  open_set_map_.insert(make_pair(cur_node->index, cur_node));
  use_node_num_ += 1;

  const auto t1 = ros::Time::now();

  /* ---------- search loop ---------- */
  while (!open_set_.empty()) {
    cur_node = open_set_.top();
    bool reach_end = abs(cur_node->index(0) - end_index(0)) <= 1 &&
        abs(cur_node->index(1) - end_index(1)) <= 1 && abs(cur_node->index(2) - end_index(2)) <= 1;
    if (reach_end) {
      backtrack(cur_node, end_pt);
      // Re-attach the vehicle's real position so the trajectory starts where it actually is.
      if (nudged) path_nodes_.insert(path_nodes_.begin(), start_pt);
      return REACH_END;
    }

    // Early termination if time up
    if ((ros::Time::now() - t1).toSec() > max_search_time_) {
      // std::cout << "early";
      early_terminate_cost_ = cur_node->g_score + getDiagHeu(cur_node->position, end_pt);
      return NO_PATH;
    }

    open_set_.pop();
    open_set_map_.erase(cur_node->index);
    close_set_map_.insert(make_pair(cur_node->index, 1));
    iter_num_ += 1;

    Eigen::Vector3d cur_pos = cur_node->position;
    Eigen::Vector3d nbr_pos;
    Eigen::Vector3d step;

    for (double dx = -resolution_; dx <= resolution_ + 1e-3; dx += resolution_)
      for (double dy = -resolution_; dy <= resolution_ + 1e-3; dy += resolution_)
        for (double dz = -resolution_; dz <= resolution_ + 1e-3; dz += resolution_) {
          step << dx, dy, dz;
          if (step.norm() < 1e-3) continue;
          nbr_pos = cur_pos + step;
          // Check safety
          if (!edt_env_->sdf_map_->isInBox(nbr_pos)) continue;
          if (edt_env_->sdf_map_->getInflateOccupancy(nbr_pos) == 1 ||
              edt_env_->sdf_map_->getOccupancy(nbr_pos) == SDFMap::UNKNOWN)
            continue;

          bool safe = true;
          Vector3d dir = nbr_pos - cur_pos;
          double len = dir.norm();
          dir.normalize();
          for (double l = 0.1; l < len; l += 0.1) {
            Vector3d ckpt = cur_pos + l * dir;
            if (edt_env_->sdf_map_->getInflateOccupancy(ckpt) == 1 ||
                edt_env_->sdf_map_->getOccupancy(ckpt) == SDFMap::UNKNOWN) {
              safe = false;
              break;
            }
          }
          if (!safe) continue;

          // Check not in close set
          Eigen::Vector3i nbr_idx;
          posToIndex(nbr_pos, nbr_idx);
          if (close_set_map_.find(nbr_idx) != close_set_map_.end()) continue;

          NodePtr neighbor;
          double tmp_g_score = step.norm() + cur_node->g_score;
          auto node_iter = open_set_map_.find(nbr_idx);
          if (node_iter == open_set_map_.end()) {
            neighbor = path_node_pool_[use_node_num_];
            use_node_num_ += 1;
            if (use_node_num_ == allocate_num_) {
              cout << "run out of node pool." << endl;
              return NO_PATH;
            }
            neighbor->index = nbr_idx;
            neighbor->position = nbr_pos;
          } else if (tmp_g_score < node_iter->second->g_score) {
            neighbor = node_iter->second;
          } else
            continue;

          neighbor->parent = cur_node;
          neighbor->g_score = tmp_g_score;
          neighbor->f_score = tmp_g_score + lambda_heu_ * getDiagHeu(nbr_pos, end_pt);
          open_set_.push(neighbor);
          open_set_map_[nbr_idx] = neighbor;
        }
  }
  // cout << "open set empty, no path!" << endl;
  // cout << "use node num: " << use_node_num_ << endl;
  // cout << "iter num: " << iter_num_ << endl;
  return NO_PATH;
}

double Astar::getEarlyTerminateCost() {
  return early_terminate_cost_;
}

void Astar::reset() {
  open_set_map_.clear();
  close_set_map_.clear();
  path_nodes_.clear();

  std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> empty_queue;
  open_set_.swap(empty_queue);
  for (int i = 0; i < use_node_num_; i++) {
    path_node_pool_[i]->parent = NULL;
  }
  use_node_num_ = 0;
  iter_num_ = 0;
}

double Astar::pathLength(const vector<Eigen::Vector3d>& path) {
  double length = 0.0;
  if (path.size() < 2) return length;
  for (int i = 0; i < path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

void Astar::backtrack(const NodePtr& end_node, const Eigen::Vector3d& end) {
  path_nodes_.push_back(end);
  path_nodes_.push_back(end_node->position);
  NodePtr cur_node = end_node;
  while (cur_node->parent != NULL) {
    cur_node = cur_node->parent;
    path_nodes_.push_back(cur_node->position);
  }
  reverse(path_nodes_.begin(), path_nodes_.end());
}

std::vector<Eigen::Vector3d> Astar::getPath() {
  return path_nodes_;
}

double Astar::getDiagHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  double h;
  double diag = min(min(dx, dy), dz);
  dx -= diag;
  dy -= diag;
  dz -= diag;

  if (dx < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
  }
  if (dy < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
  }
  if (dz < 1e-4) {
    h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
  }
  return tie_breaker_ * h;
}

double Astar::getManhHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  double dx = fabs(x1(0) - x2(0));
  double dy = fabs(x1(1) - x2(1));
  double dz = fabs(x1(2) - x2(2));
  return tie_breaker_ * (dx + dy + dz);
}

double Astar::getEuclHeu(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2) {
  return tie_breaker_ * (x2 - x1).norm();
}

std::vector<Eigen::Vector3d> Astar::getVisited() {
  vector<Eigen::Vector3d> visited;
  for (int i = 0; i < use_node_num_; ++i)
    visited.push_back(path_node_pool_[i]->position);
  return visited;
}

void Astar::posToIndex(const Eigen::Vector3d& pt, Eigen::Vector3i& idx) {
  idx = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
}

}  // namespace fast_planner
