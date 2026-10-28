#include <set>
#include "plan_env/grid_map.h"
#include "plan_env/ground_height.hpp"
#include <rclcpp/create_timer.hpp>
#include <cmath>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{
template <typename T>
void load_parameter(rclcpp::Node *node, const std::string &name, T &value, const T &default_value)
{
  if (!node->has_parameter(name))
    node->declare_parameter<T>(name, default_value);
  node->get_parameter(name, value);
}
}  // namespace

plan_env::PoseLookupResult plan_env::lookupTimedPose(
    const std::deque<TimedPose>& history, int64_t stamp_ns,
    int64_t max_interpolation_gap_ns, int64_t max_nearest_age_ns,
    TimedPose& result)
{
  if (history.empty())
    return PoseLookupResult::EMPTY;

  const auto after = std::lower_bound(
      history.begin(), history.end(), stamp_ns,
      [](const TimedPose& pose, int64_t stamp) {
        return pose.stamp_ns < stamp;
      });
  if (after != history.end() && after->stamp_ns == stamp_ns)
  {
    result = *after;
    return PoseLookupResult::EXACT;
  }
  if (after == history.begin())
  {
    if (after->stamp_ns - stamp_ns <= max_nearest_age_ns)
    {
      result = *after;
      result.stamp_ns = stamp_ns;
      return PoseLookupResult::NEAREST;
    }
    return PoseLookupResult::TOO_OLD;
  }
  if (after == history.end())
  {
    const TimedPose& nearest = history.back();
    if (stamp_ns - nearest.stamp_ns <= max_nearest_age_ns)
    {
      result = nearest;
      result.stamp_ns = stamp_ns;
      return PoseLookupResult::NEAREST;
    }
    return PoseLookupResult::TOO_NEW;
  }

  const TimedPose& before = *std::prev(after);
  const int64_t gap_ns = after->stamp_ns - before.stamp_ns;
  if (gap_ns <= 0 || gap_ns > max_interpolation_gap_ns)
    return PoseLookupResult::GAP_TOO_LARGE;

  const double alpha = static_cast<double>(stamp_ns - before.stamp_ns) /
                       static_cast<double>(gap_ns);
  result.stamp_ns = stamp_ns;
  result.position = before.position + alpha * (after->position - before.position);
  result.orientation = before.orientation.slerp(alpha, after->orientation).normalized();
  return PoseLookupResult::INTERPOLATED;
}

double plan_env::updateHeightReference(
    double current, double measurement, double dt_seconds,
    double filter_tau_seconds, double max_rate)
{
  if (!std::isfinite(measurement) || dt_seconds <= 0.0 ||
      filter_tau_seconds <= 0.0 || max_rate <= 0.0)
    return current;

  const double alpha = 1.0 - std::exp(-dt_seconds / filter_tau_seconds);
  const double filtered_step = alpha * (measurement - current);
  const double max_step = max_rate * dt_seconds;
  return current + std::clamp(filtered_step, -max_step, max_step);
}

bool plan_env::pointInsideDoubleCylinder(
    const Eigen::Vector3d& point, const Eigen::Vector3d& center,
    const Eigen::Quaterniond& orientation, double radius, double offset,
    double z_down, double z_up)
{
  const Eigen::Vector3d relative = point - center;
  if (relative.z() < -z_down || relative.z() > z_up)
    return false;

  const Eigen::Vector3d heading = orientation * Eigen::Vector3d::UnitX();
  Eigen::Vector2d heading_xy = heading.head<2>();
  if (heading_xy.squaredNorm() < 1e-8)
    heading_xy = Eigen::Vector2d::UnitX();
  else
    heading_xy.normalize();

  const Eigen::Vector2d point_xy = point.head<2>();
  const Eigen::Vector2d center_xy = center.head<2>();
  const Eigen::Vector2d front = center_xy + offset * heading_xy;
  const Eigen::Vector2d rear = center_xy - offset * heading_xy;
  const double radius_squared = radius * radius;
  return (point_xy - front).squaredNorm() <= radius_squared ||
         (point_xy - rear).squaredNorm() <= radius_squared;
}

int plan_env::endpointObservation(bool self_filter_enabled, bool inside_self_filter)
{
  return self_filter_enabled && inside_self_filter ? 0 : 1;
}

bool plan_env::lowReturnOutsideBodyBand(const Eigen::Vector3d& point,
    const Eigen::Vector3d& body_position, bool separate_low_returns,
    double below_body)
{
  return separate_low_returns && point.allFinite() && body_position.allFinite() &&
      std::isfinite(below_body) && below_body > 0. &&
      point.z() < body_position.z() - below_body;
}

bool plan_env::highReturnOutsideBodyBand(const Eigen::Vector3d& point,
    const Eigen::Vector3d& body_position, bool separate_high_returns,
    double above_body)
{
  return separate_high_returns && point.allFinite() && body_position.allFinite() &&
      std::isfinite(above_body) && above_body > 0. &&
      point.z() > body_position.z() + above_body;
}

plan_env::LaserRangeEvidence plan_env::laserRangeEvidence(double measured, double minimum,
    double maximum, double no_return_range) {
  if(!std::isfinite(minimum) || !std::isfinite(maximum) || minimum<0. || maximum<=minimum ||
      !std::isfinite(no_return_range) || no_return_range<=0. || std::isnan(measured) ||
      measured<minimum || (std::isfinite(measured) && measured>maximum)) return {};
  if(std::isfinite(measured) && measured<maximum) return {true,true,measured};
  return {true,false,std::min(maximum,no_return_range)}; // +inf or explicit range limit
}

bool plan_env::rayTraversalStopsAtOccupied(bool historically_occupied)
{
  return historically_occupied;
}

bool plan_env::occupiedMissHasEnoughConfirmation(
    uint8_t consecutive_miss_frames, int required_frames)
{
  return required_frames > 0 &&
         static_cast<int>(consecutive_miss_frames) >= required_frames;
}

int8_t plan_env::localPatchOccupancy(bool observed, bool inflated_occupied)
{
  // Inflation is positive obstacle evidence even without a hit at this height.
  return inflated_occupied ? 100 : (observed ? 0 : -1);
}

bool plan_env::observationRayMayReach(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end, double resolution,
    const std::function<int(const Eigen::Vector3d&)>& query)
{
  if (!start.allFinite() || !end.allFinite() || !std::isfinite(resolution) ||
      resolution <= 0 || query(start) != 0 || query(end) < 0)
    return false;
  RayCaster ray;
  // The endpoint is the observation object, not an admitted motion waypoint.
  if (!ray.setInput(start / resolution, end / resolution))
    return true;
  Eigen::Vector3d cell;
  while (ray.step(cell))
    if (query((cell + Eigen::Vector3d::Constant(0.5)) * resolution) != 0)
      return false;
  return true;
}

bool GridMap::observationRayMayReach(
    const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  return plan_env::observationRayMayReach(start, end, mp_.resolution_,
      [this](const Eigen::Vector3d& point) { return getOccupancy(point); });
}

thread_local const GridMap* GridMap::tls_snapshot_owner_ = nullptr;
thread_local GridMap::InflatedOccupancySnapshotPtr GridMap::tls_snapshot_ = nullptr;
thread_local const GridMap* GridMap::tls_clearance_owner_ = nullptr;
thread_local double GridMap::tls_planning_clearance_margin_ = 0.0;
thread_local std::vector<Eigen::Vector2d> GridMap::tls_clearance_offsets_;

int GridMap::InflatedOccupancySnapshot::getCylinderOccupancy(const Eigen::Vector3d& point) const
{
    Eigen::Vector3i id;
    for (int dim = 0; dim < 3; ++dim)
      id(dim) = static_cast<int>(std::floor(point(dim) * resolution_inv));
    if ((id.array() < bound_min.array()).any() ||
        (id.array() > bound_max.array()).any())
      return -1;

    Eigen::Vector3i local;
    for (int dim = 0; dim < 3; ++dim)
    {
      local(dim) = id(dim) % voxel_num(dim);
      if (local(dim) < 0)
        local(dim) += voxel_num(dim);
    }
    const int address = local(0) * voxel_num(1) * voxel_num(2) +
                        local(1) * voxel_num(2) + local(2);
    return static_cast<int>(buffer[address]);
}

std::shared_ptr<const scan_planner::PlanarClearanceField>
GridMap::InflatedOccupancySnapshot::planarClearance(double z) const
{
  if (!std::isfinite(z) || resolution_inv<=0 ||
      (bound_max.array()<bound_min.array()).any()) return {};
  const int slice=static_cast<int>(std::floor(z*resolution_inv));
  if(slice<bound_min.z() || slice>bound_max.z()) return {};
  std::lock_guard<std::mutex> lock(clearance_mutex);
  const auto found=clearance_slices.find(slice);
  if(found!=clearance_slices.end()) return found->second;
  const double resolution=1.0/resolution_inv;
  const Eigen::Vector2d origin=(bound_min.head<2>().cast<double>()-Eigen::Vector2d::Ones())*resolution;
  auto field=std::make_shared<scan_planner::PlanarClearanceField>(
      bound_max.x()-bound_min.x()+3,bound_max.y()-bound_min.y()+3,resolution,origin,
      [&](int x,int y) {
        const Eigen::Vector2d p=origin+resolution*Eigen::Vector2d(x+0.5,y+0.5);
        return getCylinderOccupancy({p.x(),p.y(),(slice+0.5)*resolution})==0;
      });
  // Bound memory when the body moves across many vertical slices.
  if(clearance_slices.size()>=3) clearance_slices.erase(clearance_slices.begin());
  clearance_slices.emplace(slice,field);
  return field;
}

std::shared_ptr<const scan_planner::PlanarClearanceField>
GridMap::capturePlanarClearance(double z) const
{
  const auto snapshot=(tls_snapshot_owner_==this && tls_snapshot_)
      ? tls_snapshot_ : captureInflatedOccupancySnapshot();
  return snapshot ? snapshot->planarClearance(z) : nullptr;
}

int GridMap::InflatedOccupancySnapshot::getInflateOccupancy(
    const Eigen::Vector3d& pos, double yaw,
    const std::vector<Eigen::Vector2d>& clearance_offsets) const
{
  auto query=[this](const Eigen::Vector3d& point) {return getCylinderOccupancy(point);};
  const auto query_with_margin = [&query, &clearance_offsets](const Eigen::Vector3d& center) {
    if(clearance_offsets.empty()) return query(center);
    for(const auto& offset:clearance_offsets) {
      Eigen::Vector3d sample=center; sample.head<2>()+=offset;
      const int occupancy=query(sample);
      if(occupancy!=0) return occupancy;
    }
    return 0;
  };

  const Eigen::Vector3d heading(std::cos(yaw), std::sin(yaw), 0.0);
  const int front = query_with_margin(pos + body_offset * heading);
  return front != 0
             ? front
             : query_with_margin(pos - body_offset * heading);
}

plan_env::MotionAssessment GridMap::InflatedOccupancySnapshot::assessMotion(
    const Eigen::Vector3d& pos, double yaw) const
{
  plan_env::MotionAssessment result;
  if (!pos.allFinite() || !std::isfinite(yaw) || resolution_inv<=0) {
    result.outside_map=true;return result;
  }
  const Eigen::Vector3d heading(std::cos(yaw),std::sin(yaw),0.);
  for (double side : {-1.,1.}) {
    const Eigen::Vector3d center=pos+side*body_offset*heading;
    const Eigen::Vector3i query=(center*resolution_inv).array().floor().cast<int>();
    const int occupied=getCylinderOccupancy(center);
    if (occupied<0) {result.outside_map=true;continue;}
    bool explained=false;
    const int xmin=query.x()-inflation_max.x(), xmax=query.x()-inflation_min.x();
    auto source=sources.lower_bound({xmin,std::numeric_limits<int>::min(),std::numeric_limits<int>::min()});
    for (;source!=sources.end() && source->first[0]<=xmax;++source) {
      const auto& key=source->first;
      if (!inflation_offsets.count({query.x()-key[0],query.y()-key[1],query.z()-key[2]})) continue;
      explained=true;
      const Eigen::Vector2d source_xy((key[0]+.5)/resolution_inv,(key[1]+.5)/resolution_inv);
      // One voxel diagonal accounts for the discrete inflation kernel. This is
      // deliberately more conservative than a point-to-cylinder distance.
      const double penetration=std::max(0.,body_radius+std::sqrt(2.)/resolution_inv-
          (center.head<2>()-source_xy).norm());
      auto found=result.contacts.find(key);
      if(found==result.contacts.end() || penetration>found->second.penetration)
        result.contacts[key]={key,source->second.layers,penetration,source->second};
    }
    if (occupied>0 && !explained) result.unexplained_inflation=true;
  }
  return result;
}

void GridMap::setTerrainMotionPolicy(const plan_env::TerrainMotionPolicy& policy)
{
  policy.validate();
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  terrain_motion_policy_=policy;
}

GridMap::InflatedOccupancySnapshotPtr GridMap::captureInflatedOccupancySnapshot() const
{
  auto snapshot = std::atomic_load(&latest_inflated_snapshot_);
  if (snapshot)
    return snapshot;

  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  snapshot = buildInflatedOccupancySnapshotLocked();
  std::atomic_store(&latest_inflated_snapshot_, snapshot);
  return snapshot;
}

GridMap::InflatedOccupancySnapshotPtr
GridMap::buildInflatedOccupancySnapshotLocked() const
{
  auto snapshot = std::make_shared<InflatedOccupancySnapshot>();
  snapshot->buffer = md_.occupancy_buffer_inflate_;
  snapshot->voxel_num = mp_.map_voxel_num_;
  snapshot->bound_min = mp_.map_bound_min_idx_;
  snapshot->bound_max = mp_.map_bound_max_idx_;
  snapshot->resolution_inv = mp_.resolution_inv_;
  snapshot->body_offset = mp_.double_cylinder_offset_;
  snapshot->revision = map_revision_.load();
  snapshot->body_radius=mp_.double_cylinder_radius_;
  snapshot->sources=md_.occupancy_sources_;
  snapshot->terrain_facts=md_.terrain_facts_;
  for(const auto& offset:md_.inflate_offsets_) {
    snapshot->inflation_offsets.insert({offset.x(),offset.y(),offset.z()});
    snapshot->inflation_min=snapshot->inflation_min.cwiseMin(offset);
    snapshot->inflation_max=snapshot->inflation_max.cwiseMax(offset);
  }
  return snapshot;
}

void GridMap::useInflatedOccupancySnapshotForCurrentThread(
    InflatedOccupancySnapshotPtr snapshot) const
{
  tls_snapshot_owner_ = this;
  tls_snapshot_ = std::move(snapshot);
}

void GridMap::clearInflatedOccupancySnapshotForCurrentThread() const
{
  if (tls_snapshot_owner_ == this)
  {
    tls_snapshot_.reset();
    tls_snapshot_owner_ = nullptr;
  }
}

double GridMap::getMapAgeSeconds() const
{
  const int64_t update_ns = last_map_update_ns_.load();
  if (!node_ || update_ns <= 0)
    return std::numeric_limits<double>::infinity();
  return std::max(0.0, (node_->now().nanoseconds() - update_ns) * 1e-9);
}

void GridMap::initMap(
    rclcpp::Node *node,
    rclcpp::CallbackGroup::SharedPtr update_callback_group,
    rclcpp::CallbackGroup::SharedPtr visualization_callback_group)
{
  node_ = node;

  /* get parameter */
  double x_size, y_size, z_size;
  load_parameter(node_, "grid_map.resolution", mp_.resolution_, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_x", x_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_y", y_size, -1.0);
  load_parameter(node_, "grid_map.sliding_map_size_z", z_size, -1.0);
  load_parameter(node_, "grid_map.local_update_range_x", mp_.local_update_range_(0), x_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_y", mp_.local_update_range_(1), y_size / 2.0);
  load_parameter(node_, "grid_map.local_update_range_z", mp_.local_update_range_(2), z_size / 2.0);
  
  load_parameter(node_, "grid_map.obstacles_inflation_z_up", mp_.obstacles_inflation_z_up, -1.0);
  load_parameter(node_, "grid_map.obstacles_inflation_z_down", mp_.obstacles_inflation_z_down, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_radius", mp_.double_cylinder_radius_, -1.0);
  load_parameter(node_, "grid_map.double_cylinder_offset", mp_.double_cylinder_offset_, 0.0);
  load_parameter(node_, "grid_map.self_filter_enabled", mp_.self_filter_enabled_, true);
  load_parameter(node_, "grid_map.self_filter_z_down", mp_.self_filter_z_down_, 0.45);
  load_parameter(node_, "grid_map.self_filter_z_up", mp_.self_filter_z_up_, 0.20);
  if (mp_.self_filter_enabled_ &&
      (mp_.double_cylinder_radius_ <= 0.0 || mp_.double_cylinder_offset_ < 0.0 ||
       mp_.self_filter_z_down_ < 0.0 || mp_.self_filter_z_up_ < 0.0))
  {
    throw std::invalid_argument(
        "grid_map self-filter requires a positive radius and non-negative offsets and z bounds");
  }
  load_parameter(node_, "grid_map.separate_low_returns", mp_.separate_low_returns_, false);
  load_parameter(node_, "grid_map.low_return_z_below_body", mp_.low_return_z_below_body_, 0.25);
  if (!std::isfinite(mp_.low_return_z_below_body_) || mp_.low_return_z_below_body_ <= 0.)
    throw std::invalid_argument("grid_map.low_return_z_below_body must be finite and positive");
  // Upper half of the same platform band. Enabled alongside the low side by
  // the identified-ray launch: a 3-D lidar always sees overhead structure, and
  // treating it as occupancy blocks every cell the beam passes over.
  load_parameter(node_, "grid_map.separate_high_returns", mp_.separate_high_returns_, false);
  load_parameter(node_, "grid_map.high_return_z_above_body", mp_.high_return_z_above_body_, 0.25);
  if (!std::isfinite(mp_.high_return_z_above_body_) || mp_.high_return_z_above_body_ <= 0.)
    throw std::invalid_argument("grid_map.high_return_z_above_body must be finite and positive");
  load_parameter(node_, "grid_map.map_sliding_en", mp_.map_sliding_en_, true);
  load_parameter(node_, "grid_map.map_sliding_thresh", mp_.map_sliding_thresh_, mp_.resolution_);

  load_parameter(node_, "grid_map.fx", mp_.fx_, -1.0);
  load_parameter(node_, "grid_map.fy", mp_.fy_, -1.0);
  load_parameter(node_, "grid_map.cx", mp_.cx_, -1.0);
  load_parameter(node_, "grid_map.cy", mp_.cy_, -1.0);

  load_parameter(node_, "grid_map.depth_filter_maxdist", mp_.depth_filter_maxdist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_mindist", mp_.depth_filter_mindist_, -1.0);
  load_parameter(node_, "grid_map.depth_filter_margin", mp_.depth_filter_margin_, -1);
  load_parameter(node_, "grid_map.k_depth_scaling_factor", mp_.k_depth_scaling_factor_, -1.0);
  load_parameter(node_, "grid_map.skip_pixel", mp_.skip_pixel_, -1);

  load_parameter(node_, "grid_map.p_hit", mp_.p_hit_, -1.0);
  load_parameter(node_, "grid_map.p_miss", mp_.p_miss_, -1.0);
  load_parameter(node_, "grid_map.p_min", mp_.p_min_, -1.0);
  load_parameter(node_, "grid_map.p_max", mp_.p_max_, -1.0);
  load_parameter(node_, "grid_map.p_occ", mp_.p_occ_, -1.0);
  load_parameter(node_, "grid_map.max_ray_length", mp_.max_ray_length_, -0.1);
  load_parameter(node_, "grid_map.occupied_miss_confirmation_frames",
                 mp_.occupied_miss_confirmation_frames_, 3);
  if (mp_.occupied_miss_confirmation_frames_ <= 0 ||
      mp_.occupied_miss_confirmation_frames_ > 255)
    throw std::invalid_argument(
        "grid_map.occupied_miss_confirmation_frames must be in [1, 255]");
  // A terrain projection is released when its cell stops being observed as a
  // hazard for this many scans: the robot moved or the surface became
  // occluded, and a stale block must not outlive the evidence that made it.
  load_parameter(node_, "grid_map.terrain_projection_timeout_frames",
                 mp_.terrain_projection_timeout_frames_, 20);
  if (mp_.terrain_projection_timeout_frames_ < 2 ||
      mp_.terrain_projection_timeout_frames_ > 100000)
    throw std::invalid_argument(
        "grid_map.terrain_projection_timeout_frames must be in [2, 100000]");
  // Ground-relative clearance volume for height-carrying measured returns.
  // These are robot geometry plus capability statements, not tuning: the legs
  // handle terrain below obstacle_min_height, and obstacle_max_height is where
  // structure stops being able to hit the body. Returns without a height keep
  // using the body-relative band below.
  load_parameter(node_, "grid_map.terrain_floor_tolerance",
                 mp_.terrain_floor_tolerance_, 0.035);
  load_parameter(node_, "grid_map.obstacle_min_height", mp_.obstacle_min_height_, 0.22);
  load_parameter(node_, "grid_map.obstacle_max_height", mp_.obstacle_max_height_, 0.55);
  if (!(mp_.terrain_floor_tolerance_ > 0. &&
        mp_.terrain_floor_tolerance_ < mp_.obstacle_min_height_ &&
        mp_.obstacle_min_height_ < mp_.obstacle_max_height_))
    throw std::invalid_argument(
        "grid_map terrain heights must satisfy 0 < terrain_floor_tolerance < "
        "obstacle_min_height < obstacle_max_height");

  terrain_motion_policy_={mp_.terrain_floor_tolerance_,mp_.obstacle_min_height_,
      mp_.obstacle_min_height_,mp_.obstacle_max_height_};
  terrain_motion_policy_.validate();
  load_parameter(node_, "grid_map.vis_height", mp_.vis_height_, 0.3);
  // Display separation only. Neither fusion nor collision queries use this.
  load_parameter(node_, "grid_map.body_obstacle_z_below", mp_.body_obstacle_z_below_, 0.12);
  if (!std::isfinite(mp_.body_obstacle_z_below_) || mp_.body_obstacle_z_below_ < 0)
    throw std::invalid_argument("grid_map.body_obstacle_z_below must be finite and nonnegative");
  load_parameter(node_, "grid_map.show_occ_time", mp_.show_occ_time_, false);

  load_parameter(node_, "grid_map.frame_id", mp_.frame_id_, string("world"));
  load_parameter(node_, "grid_map.sliding_map_frame_id", mp_.sliding_map_frame_id_, string("sliding_map"));
  load_parameter(node_, "grid_map.ground_height", mp_.ground_height_, 0.0);
  load_parameter(node_, "grid_map.local_patch_enabled", local_patch_enabled_, true);
  load_parameter(node_, "grid_map.local_patch_period", local_patch_period_, 0.25);
  load_parameter(node_, "grid_map.local_patch_resolution", local_patch_resolution_, 0.2);
  load_parameter(node_, "grid_map.local_patch_z", local_patch_z_, mp_.ground_height_ + 0.3);
  load_parameter(node_, "grid_map.local_patch_follow_body_z", local_patch_follow_body_z_, true);
  load_parameter(node_, "grid_map.local_patch_z_filter_tau", local_patch_z_filter_tau_, 0.75);
  load_parameter(node_, "grid_map.local_patch_z_max_rate", local_patch_z_max_rate_, 0.5);
  load_parameter(node_, "grid_map.local_patch_terrain_tile_size",
                 local_patch_terrain_tile_size_, 2.0);
  if (!std::isfinite(local_patch_period_) || local_patch_period_ <= 0.0 ||
      !std::isfinite(local_patch_resolution_) || local_patch_resolution_ < mp_.resolution_)
    throw std::invalid_argument(
        "grid_map local patch period must be positive and resolution must not be finer than the voxel map");
  if (!std::isfinite(local_patch_terrain_tile_size_) ||
      local_patch_terrain_tile_size_ < local_patch_resolution_)
    throw std::invalid_argument(
        "grid_map local patch terrain tile size must be at least the patch resolution");
  if (local_patch_z_filter_tau_ <= 0.0 || local_patch_z_max_rate_ <= 0.0)
    throw std::invalid_argument(
        "grid_map local patch height filter parameters must be positive");
  local_patch_reference_z_ = local_patch_z_;

  load_parameter(node_, "grid_map.sensor_type", mp_.sensor_type_, string("lidar"));
  load_parameter(node_, "grid_map.use_laser_scan", mp_.use_laser_scan_, false);
  load_parameter(node_, "grid_map.use_ray_cloud", mp_.use_ray_cloud_, false);
  if (mp_.use_ray_cloud_ && mp_.use_laser_scan_)
    throw std::invalid_argument("choose one identified ray input, not both");
  load_parameter(node_, "grid_map.no_return_range", mp_.no_return_range_, 3.0);
  if(!std::isfinite(mp_.no_return_range_) || mp_.no_return_range_<=0.)
    throw std::invalid_argument("grid_map.no_return_range must be finite and positive");
  load_parameter(node_, "grid_map.cloud_is_world", mp_.cloud_is_world_, true);
  load_parameter(node_, "grid_map.need_extrinsic", mp_.need_extrinsic_, true);

  load_parameter(node_, "grid_map.pose_history_seconds", mp_.pose_history_seconds_, 0.5);
  load_parameter(node_, "grid_map.max_interpolation_gap", mp_.max_interpolation_gap_seconds_, 0.05);
  load_parameter(node_, "grid_map.max_nearest_pose_age", mp_.max_nearest_pose_age_seconds_, 0.02);
  if (mp_.pose_history_seconds_ <= 0.0 || mp_.max_interpolation_gap_seconds_ <= 0.0 ||
      mp_.max_nearest_pose_age_seconds_ < 0.0)
    throw std::invalid_argument("grid_map pose history timing parameters are invalid");

  double lidar_x = -0.01100;
  double lidar_y = -0.02329;
  double lidar_z = 0.04412;
  double lidar_roll = 0.0;
  double lidar_pitch = 0.0;
  double lidar_yaw = 0.0;
  load_parameter(node_, "grid_map.lidar_extrinsic_x", lidar_x, lidar_x);
  load_parameter(node_, "grid_map.lidar_extrinsic_y", lidar_y, lidar_y);
  load_parameter(node_, "grid_map.lidar_extrinsic_z", lidar_z, lidar_z);
  load_parameter(node_, "grid_map.lidar_extrinsic_roll", lidar_roll, lidar_roll);
  load_parameter(node_, "grid_map.lidar_extrinsic_pitch", lidar_pitch, lidar_pitch);
  load_parameter(node_, "grid_map.lidar_extrinsic_yaw", lidar_yaw, lidar_yaw);

  mp_.lidar_extrinsic_.setIdentity();
  mp_.lidar_extrinsic_.block<3, 3>(0, 0) =
      (Eigen::AngleAxisd(lidar_yaw, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(lidar_pitch, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(lidar_roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
  mp_.lidar_extrinsic_.block<3, 1>(0, 3) = Eigen::Vector3d(lidar_x, lidar_y, lidar_z);
  RCLCPP_INFO(
      node_->get_logger(),
      "[SENSOR_CONTRACT] cloud_is_world=%s need_extrinsic=%s lidar_xyz=(%.3f,%.3f,%.3f) "
      "pose_history=%.3fs interpolation_gap=%.3fs nearest_age=%.3fs",
      mp_.cloud_is_world_ ? "true" : "false",
      mp_.need_extrinsic_ ? "true" : "false", lidar_x, lidar_y, lidar_z,
      mp_.pose_history_seconds_, mp_.max_interpolation_gap_seconds_,
      mp_.max_nearest_pose_age_seconds_);
  RCLCPP_INFO(
      node_->get_logger(),
      "[LOCAL_PATCH_CONTRACT] follow_body_z=%s fallback_z=%.3f filter_tau=%.3fs "
      "max_z_rate=%.3fm/s",
      local_patch_follow_body_z_ ? "true" : "false", local_patch_z_,
      local_patch_z_filter_tau_, local_patch_z_max_rate_);

  mp_.depth_extrinsic_ <<
      0.0,  0.707107, 0.707107, -0.15170,
     -1.0,  0.000000, 0.000000,  0.00000,
      0.0, -0.707107, 0.707107,  0.07510,
      0.0,  0.000000, 0.000000,  1.00000;
      
  if (mp_.sensor_type_ != "lidar" && mp_.sensor_type_ != "depth")
  {
    RCLCPP_ERROR(node_->get_logger(), "[GridMap] invalid grid_map.sensor_type: %s; falling back to lidar",
                 mp_.sensor_type_.c_str());
    mp_.sensor_type_ = "lidar";
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-x_size / 2.0, -y_size / 2.0, mp_.ground_height_);
  mp_.map_size_ = Eigen::Vector3d(x_size, y_size, z_size);

  mp_.prob_hit_log_ = logit(mp_.p_hit_);
  mp_.prob_miss_log_ = logit(mp_.p_miss_);
  mp_.clamp_min_log_ = logit(mp_.p_min_);
  mp_.clamp_max_log_ = logit(mp_.p_max_);
  mp_.min_occupancy_log_ = logit(mp_.p_occ_);
  mp_.unknown_flag_ = 0.01;
  mp_.map_sliding_thresh_vox_ = std::max(1, static_cast<int>(std::ceil(mp_.map_sliding_thresh_ * mp_.resolution_inv_)));

  cout << "hit: " << mp_.prob_hit_log_ << endl;
  cout << "miss: " << mp_.prob_miss_log_ << endl;
  cout << "min log: " << mp_.clamp_min_log_ << endl;
  cout << "max: " << mp_.clamp_max_log_ << endl;
  cout << "thresh log: " << mp_.min_occupancy_log_ << endl;

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
  posToIndex(mp_.map_origin_, mp_.map_bound_min_idx_);
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();
  mp_.map_origin_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ / 2;
  updateMapBoundaryFromIndex();

  // initialize data buffers

  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);

  md_.occupancy_buffer_ = vector<double>(buffer_size, mp_.clamp_min_log_ - mp_.unknown_flag_);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
  md_.terrain_projection_buffer_ = vector<char>(buffer_size, 0);
  md_.occupancy_buffer_inflate_cnt_ = vector<int>(buffer_size, 0);
  md_.occupied_miss_confirmation_count_ = vector<uint8_t>(buffer_size, 0);
  rebuildInflationOffsets();

  md_.count_hit_and_miss_ = vector<short>(buffer_size, 0);
  md_.count_hit_ = vector<short>(buffer_size, 0);
  md_.flag_rayend_ = vector<char>(buffer_size, -1);
  md_.flag_traverse_ = vector<char>(buffer_size, -1);

  md_.raycast_num_ = 0;

  md_.proj_points_.resize(640 * 480 / mp_.skip_pixel_ / mp_.skip_pixel_);
  md_.proj_points_cnt = 0;

  /* init callback */
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*node_);

  if (mp_.sensor_type_ == "depth")
  {
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>();
    depth_pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>();
    depth_sub_->subscribe(node_, "depth", rmw_qos_profile_sensor_data);
    depth_pose_sub_->subscribe(node_, "sensor_pose", rmw_qos_profile_sensor_data);

    sync_image_pose_.reset(new message_filters::Synchronizer<SyncPolicyImagePose>(
        SyncPolicyImagePose(100), *depth_sub_, *depth_pose_sub_));
    sync_image_pose_->registerCallback(
        std::bind(&GridMap::depthPoseCallback, this, std::placeholders::_1, std::placeholders::_2));
  }
  else if (mp_.sensor_type_ == "lidar")
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = update_callback_group;
    lidar_pose_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "sensor_pose", rclcpp::SensorDataQoS(),
        std::bind(&GridMap::sensorPoseCallback, this, std::placeholders::_1), options);
    if(mp_.use_laser_scan_)
      scan_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
          "scan", rclcpp::SensorDataQoS(),
          std::bind(&GridMap::scanCallback, this, std::placeholders::_1), options);
    else
      cloud_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          "cloud", rclcpp::SensorDataQoS(),
          std::bind(&GridMap::cloudCallback, this, std::placeholders::_1), options);
  }

  rclcpp::SubscriptionOptions body_pose_options;
  body_pose_options.callback_group = update_callback_group;
  sliding_map_frame_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "body_pose", rclcpp::SensorDataQoS(),
      std::bind(&GridMap::slidingMapFrameCallback, this, std::placeholders::_1),
      body_pose_options);
  locomotion_state_sub_ = node_->create_subscription<scan_planner_msgs::msg::LocomotionState>(
      "/robot/locomotion_state", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&GridMap::locomotionStateCallback, this, std::placeholders::_1));

  occ_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                        std::bind(&GridMap::updateOccupancyCallback, this),
                                        update_callback_group);
  // Use the same ROS clock as Explorer. A wall timer would run at a different
  // effective rate when Gazebo is paused or its real-time factor changes.
  const auto spatial_period = rclcpp::Duration::from_seconds(local_patch_period_);
  local_patch_timer_ = rclcpp::create_timer(node_, node_->get_clock(), spatial_period,
                                        std::bind(&GridMap::publishLocalMapPatch, this),
                                        update_callback_group);
  vis_timer_ = rclcpp::create_timer(node_, node_->get_clock(), spatial_period,
                                        std::bind(&GridMap::visCallback, this),
                                        visualization_callback_group);

  // These are derived map frames, not raw sensor streams. Reliable latest-only
  // delivery prevents large inflated clouds losing UDP fragments in RViz.
  const auto map_display_qos = rclcpp::QoS(1).reliable().durability_volatile();
  map_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy", map_display_qos);
  map_inf_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/occupancy_inflate", map_display_qos);
  low_surface_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/low_surface_returns", map_display_qos);
  high_surface_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/high_surface_returns", map_display_qos);
  body_obstacle_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/body_obstacles", map_display_qos);
  terrain_evidence_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/terrain_evidence", map_display_qos);
  sliding_map_bbox_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>("grid_map/sliding_map_bbox", 10);

  unknown_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/unknown", rclcpp::SensorDataQoS());
  depth_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("grid_map/depth_cloud", rclcpp::SensorDataQoS());
  extrinsic_pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("grid_map/sensor_pose_extrinsic", 10);
  local_patch_pub_ = node_->create_publisher<scan_planner_msgs::msg::LocalMapPatch>(
      "grid_map/local_map_patch", rclcpp::QoS(1).best_effort().durability_volatile());

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  md_.has_first_depth_ = false;
  md_.has_ray_pose_ = false;
  md_.has_cloud_ = false;
  md_.image_cnt_ = 0;
  md_.ray_pos_.setZero();
  md_.sliding_map_frame_pos_.setZero();
  md_.ray_q_ = Eigen::Quaterniond::Identity();

  md_.fuse_time_ = 0.0;
  md_.update_num_ = 0;
  md_.max_fuse_time_ = 0.0;
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;

  // rand_noise_ = uniform_real_distribution<double>(-0.2, 0.2);
  // rand_noise2_ = normal_distribution<double>(0, 0.2);
  // random_device rd;
  // eng_ = default_random_engine(rd());
}

void GridMap::updateMapBoundaryFromIndex()
{
  mp_.map_bound_min_idx_ = mp_.map_origin_idx_ - mp_.map_voxel_num_ / 2;
  mp_.map_bound_max_idx_ = mp_.map_bound_min_idx_ + mp_.map_voxel_num_ - Eigen::Vector3i::Ones();

  mp_.map_min_boundary_ = mp_.map_bound_min_idx_.cast<double>() * mp_.resolution_;
  mp_.map_max_boundary_ = (mp_.map_bound_max_idx_.cast<double>() + Eigen::Vector3d::Ones()) * mp_.resolution_;
  mp_.map_origin_ = mp_.map_min_boundary_;
}

void GridMap::rebuildInflationOffsets()
{
  const double double_radius = std::max(0.0, mp_.double_cylinder_radius_);
  const int inf_step_xy = ceil(double_radius / mp_.resolution_);
  const int inf_step_z_up = ceil(mp_.obstacles_inflation_z_up / mp_.resolution_);
  const int inf_step_z_down = ceil(mp_.obstacles_inflation_z_down / mp_.resolution_);

  md_.inflate_offsets_.clear();
  for (int x = -inf_step_xy; x <= inf_step_xy; ++x)
    for (int y = -inf_step_xy; y <= inf_step_xy; ++y)
    {
      Eigen::Vector2d offset_xy(x * mp_.resolution_, y * mp_.resolution_);
      if (offset_xy.norm() >= double_radius)
        continue;

      for (int z = -inf_step_z_down; z <= inf_step_z_up; ++z)
        md_.inflate_offsets_.push_back(Eigen::Vector3i(x, y, z));
    }
}

void GridMap::resetAllMapData()
{
  md_.low_surface_returns_.clear();
  md_.high_surface_returns_.clear();
  md_.terrain_projections_.clear();
  md_.occupancy_sources_.clear();
  md_.terrain_facts_.clear();
  md_.terrain_scan_ = 0;
  std::fill(md_.terrain_projection_buffer_.begin(),md_.terrain_projection_buffer_.end(),0);
  std::fill(md_.occupancy_buffer_.begin(), md_.occupancy_buffer_.end(), mp_.clamp_min_log_ - mp_.unknown_flag_);
  std::fill(md_.occupancy_buffer_inflate_.begin(), md_.occupancy_buffer_inflate_.end(), 0);
  std::fill(md_.occupancy_buffer_inflate_cnt_.begin(), md_.occupancy_buffer_inflate_cnt_.end(), 0);
  std::fill(md_.occupied_miss_confirmation_count_.begin(),
            md_.occupied_miss_confirmation_count_.end(), 0);
  std::fill(md_.count_hit_and_miss_.begin(), md_.count_hit_and_miss_.end(), 0);
  std::fill(md_.count_hit_.begin(), md_.count_hit_.end(), 0);
  std::fill(md_.flag_rayend_.begin(), md_.flag_rayend_.end(), -1);
  std::fill(md_.flag_traverse_.begin(), md_.flag_traverse_.end(), -1);
  std::queue<Eigen::Vector3i> empty;
  std::swap(md_.cache_voxel_, empty);
}

void GridMap::hashIdToGlobalIndex(int addr, Eigen::Vector3i& id_g) const
{
  Eigen::Vector3i id_l;
  id_l(0) = addr / (mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2));
  id_l(1) = (addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2)) / mp_.map_voxel_num_(2);
  id_l(2) = addr - id_l(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) -
            id_l(1) * mp_.map_voxel_num_(2);

  for (int i = 0; i < 3; ++i)
  {
    const int min_l = getLocalIndex(mp_.map_bound_min_idx_(i), i);
    int dist = id_l(i) - min_l;
    if (dist < 0)
      dist += mp_.map_voxel_num_(i);
    id_g(i) = mp_.map_bound_min_idx_(i) + dist;
  }
}

void GridMap::updateInflationLayer(const Eigen::Vector3i& id, int delta,
                                   const vector<Eigen::Vector3i>& offsets,
                                   std::vector<int>& cnt_buffer,
                                   std::vector<char>& flag_buffer,
                                   const std::vector<char>* ignore_mask)
{
  for (const auto& offset : offsets)
  {
    const Eigen::Vector3i inf_id = id + offset;
    if (!isInMap(inf_id))
      continue;

    const int addr = toAddress(inf_id);
    if (ignore_mask && (*ignore_mask)[addr])
      continue;

    cnt_buffer[addr] += delta;
    if (cnt_buffer[addr] < 0)
      cnt_buffer[addr] = 0;
    flag_buffer[addr] = cnt_buffer[addr] > 0 ? 1 : 0;
  }
}

void GridMap::updateInflation(const Eigen::Vector3i& id, int delta, const std::vector<char>* ignore_mask)
{
  updateInflationLayer(id, delta, md_.inflate_offsets_, md_.occupancy_buffer_inflate_cnt_,
                       md_.occupancy_buffer_inflate_, ignore_mask);
}

void GridMap::setTerrainProjection(const Eigen::Vector3i& id, bool occupied)
{
  if (!isInMap(id)) return;
  const int address=toAddress(id);
  const bool raw=md_.occupancy_buffer_[address] > mp_.min_occupancy_log_;
  const bool was=raw || md_.terrain_projection_buffer_[address];
  md_.terrain_projection_buffer_[address]=occupied;
  const plan_env::SourceKey key{id.x(),id.y(),id.z()};
  if(occupied) md_.occupancy_sources_[key].layers|=plan_env::OccupancyEvidence::TERRAIN_CONSTRAINT;
  else {
    auto source=md_.occupancy_sources_.find(key);
    if(source!=md_.occupancy_sources_.end()) {
      source->second.layers&=~plan_env::OccupancyEvidence::TERRAIN_CONSTRAINT;
      if(!source->second.layers) md_.occupancy_sources_.erase(source);
    }
  }
  const bool now=raw || occupied;
  if(was!=now) updateInflation(id,now ? 1 : -1);
}

void GridMap::applyOccupancyUpdate(const Eigen::Vector3i& id, double new_log_odds)
{
  if (!isInMap(id))
    return;

  const int addr = toAddress(id);
  const bool was_occ = md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_ ||
      md_.terrain_projection_buffer_[addr];
  const bool now_occ = new_log_odds > mp_.min_occupancy_log_ ||
      md_.terrain_projection_buffer_[addr];

  const plan_env::SourceKey key{id.x(),id.y(),id.z()};
  if(new_log_odds>mp_.min_occupancy_log_) {
    auto& source=md_.occupancy_sources_[key];
    source.layers|=plan_env::OccupancyEvidence::MEASURED;
    // Free evidence changes confidence, not the time of the last real hit.
    if(new_log_odds>md_.occupancy_buffer_[addr]) source.measured_scan=md_.terrain_scan_;
  } else {
    auto source=md_.occupancy_sources_.find(key);
    if(source!=md_.occupancy_sources_.end()) {
      source->second.layers&=~plan_env::OccupancyEvidence::MEASURED;
      if(!source->second.layers) md_.occupancy_sources_.erase(source);
    }
  }
  md_.occupancy_buffer_[addr] = new_log_odds;
  if (was_occ != now_occ)
    updateInflation(id, now_occ ? 1 : -1);
}

void GridMap::resetCellByAddress(int addr)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_ || md_.terrain_projection_buffer_[addr])
    updateInflation(id_g, -1);

  md_.terrain_projection_buffer_[addr]=0;
  md_.occupancy_sources_.erase({id_g.x(),id_g.y(),id_g.z()});

  md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
  md_.occupied_miss_confirmation_count_[addr] = 0;
  md_.count_hit_[addr] = 0;
  md_.count_hit_and_miss_[addr] = 0;
  md_.flag_rayend_[addr] = -1;
  md_.flag_traverse_[addr] = -1;
}

void GridMap::resetCellByAddressForSliding(int addr, const std::vector<char>& clear_mask)
{
  Eigen::Vector3i id_g;
  hashIdToGlobalIndex(addr, id_g);
  if (md_.occupancy_buffer_[addr] > mp_.min_occupancy_log_ || md_.terrain_projection_buffer_[addr])
    updateInflation(id_g, -1, &clear_mask);
}

void GridMap::updateSlidingMap(const Eigen::Vector3d& center)
{
  if (!mp_.map_sliding_en_)
    return;

  Eigen::Vector3i new_origin_idx;
  posToIndex(center, new_origin_idx);
  const Eigen::Vector3i shift_num = new_origin_idx - mp_.map_origin_idx_;
  if (shift_num.cwiseAbs().maxCoeff() < mp_.map_sliding_thresh_vox_)
    return;

  if ((shift_num.cwiseAbs().array() >= mp_.map_voxel_num_.array()).any())
  {
    resetAllMapData();
    mp_.map_origin_idx_ = new_origin_idx;
    updateMapBoundaryFromIndex();
    md_.local_bound_min_ = mp_.map_bound_min_idx_;
    md_.local_bound_max_ = mp_.map_bound_max_idx_;
    return;
  }

  const int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  std::vector<char> clear_mask(buffer_size, 0);
  std::vector<int> clear_addrs;
  clear_addrs.reserve(buffer_size / 8);

  auto add_clear_addr = [&](const Eigen::Vector3i& id_l) {
    const int addr = toAddressLocal(id_l);
    if (!clear_mask[addr])
    {
      clear_mask[addr] = 1;
      clear_addrs.push_back(addr);
    }
  };

  for (int dim = 0; dim < 3; ++dim)
  {
    const int shift = shift_num(dim);
    if (shift == 0)
      continue;

    if (shift > 0)
    {
      for (int k = 0; k < shift; ++k)
      {
        const int clear_g = mp_.map_bound_min_idx_(dim) + k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
    else
    {
      for (int k = 0; k < -shift; ++k)
      {
        const int clear_g = mp_.map_bound_max_idx_(dim) - k;
        const int clear_l = getLocalIndex(clear_g, dim);

        for (int a = 0; a < mp_.map_voxel_num_((dim + 1) % 3); ++a)
          for (int b = 0; b < mp_.map_voxel_num_((dim + 2) % 3); ++b)
          {
            Eigen::Vector3i id_l;
            id_l(dim) = clear_l;
            id_l((dim + 1) % 3) = a;
            id_l((dim + 2) % 3) = b;
            add_clear_addr(id_l);
          }
      }
    }
  }

  for (int addr : clear_addrs)
    resetCellByAddressForSliding(addr, clear_mask);

  for (int addr : clear_addrs)
  {
    md_.occupancy_buffer_[addr] = mp_.clamp_min_log_ - mp_.unknown_flag_;
    md_.terrain_projection_buffer_[addr] = 0;
    Eigen::Vector3i old_id; hashIdToGlobalIndex(addr,old_id);
    md_.occupancy_sources_.erase({old_id.x(),old_id.y(),old_id.z()});
    md_.occupancy_buffer_inflate_cnt_[addr] = 0;
    md_.occupancy_buffer_inflate_[addr] = 0;
    md_.occupied_miss_confirmation_count_[addr] = 0;
    md_.count_hit_[addr] = 0;
    md_.count_hit_and_miss_[addr] = 0;
    md_.flag_rayend_[addr] = -1;
    md_.flag_traverse_[addr] = -1;
  }

  mp_.map_origin_idx_ = new_origin_idx;
  updateMapBoundaryFromIndex();
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);
}

void GridMap::resetBuffer()
{
  resetAllMapData();
  md_.local_bound_min_ = mp_.map_bound_min_idx_;
  md_.local_bound_max_ = mp_.map_bound_max_idx_;
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{
  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
    {
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        resetCellByAddress(toAddress(x, y, z));
      }
    }
}

int GridMap::setCacheOccupancy(Eigen::Vector3d pos, int occ)
{
  if (occ != 1 && occ != 0)
    return INVALID_IDX;

  Eigen::Vector3i id;
  posToIndex(pos, id);
  if (!isInMap(id))
    return INVALID_IDX;

  int idx_ctns = toAddress(id);

  md_.count_hit_and_miss_[idx_ctns] += 1;

  if (md_.count_hit_and_miss_[idx_ctns] == 1)
  {
    md_.cache_voxel_.push(id);
  }

  if (occ == 1)
    md_.count_hit_[idx_ctns] += 1;

  return idx_ctns;
}

void GridMap::projectDepthImage()
{
  // md_.proj_points_.clear();
  md_.proj_points_cnt = 0;

  uint16_t *row_ptr;
  // int cols = current_img_.cols, rows = current_img_.rows;
  int cols = md_.depth_image_.cols;
  int rows = md_.depth_image_.rows;

  double depth;

  Eigen::Matrix3d sensor_r = md_.ray_q_.toRotationMatrix();

  // cout << "rotate: " << md_.ray_q_.toRotationMatrix() << endl;
  // std::cout << "pos in proj: " << md_.ray_pos_ << std::endl;

  if (!md_.has_first_depth_)
  {
    md_.has_first_depth_ = true;
    return;
  }

  Eigen::Vector3d pt_cur, pt_world;
  const double inv_factor = 1.0 / mp_.k_depth_scaling_factor_;

  for (int v = mp_.depth_filter_margin_; v < rows - mp_.depth_filter_margin_; v += mp_.skip_pixel_)
  {
    row_ptr = md_.depth_image_.ptr<uint16_t>(v) + mp_.depth_filter_margin_;

    for (int u = mp_.depth_filter_margin_; u < cols - mp_.depth_filter_margin_; u += mp_.skip_pixel_)
    {
      const uint16_t raw_depth = *row_ptr;
      depth = raw_depth * inv_factor;
      row_ptr = row_ptr + mp_.skip_pixel_;

      // filter depth
      // depth += rand_noise_(eng_);
      // if (depth > 0.01) depth += rand_noise2_(eng_);

      if (raw_depth == 0)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }
      else if (depth < mp_.depth_filter_mindist_)
      {
        continue;
      }
      else if (depth > mp_.depth_filter_maxdist_)
      {
        depth = mp_.max_ray_length_ + 0.1;
      }

      // project to world frame
      pt_cur(0) = (u - mp_.cx_) * depth / mp_.fx_;
      pt_cur(1) = (v - mp_.cy_) * depth / mp_.fy_;
      pt_cur(2) = depth;

      pt_world = sensor_r * pt_cur + md_.ray_pos_;
      // if (!isInMap(pt_world)) {
      //   pt_world = closetPointInMap(pt_world, md_.ray_pos_);
      // }

      md_.proj_points_[md_.proj_points_cnt++] = pt_world;
    }
  }
}

bool GridMap::isInsideSelfFilter(const Eigen::Vector3d& point) const
{
  if (!mp_.self_filter_enabled_)
    return false;

  return plan_env::pointInsideDoubleCylinder(
      point, md_.ray_body_pos_, md_.ray_body_q_, mp_.double_cylinder_radius_,
      mp_.double_cylinder_offset_, mp_.self_filter_z_down_, mp_.self_filter_z_up_);
}

void GridMap::raycastProcess()
{
  // if (md_.proj_points_.size() == 0)
  if (md_.proj_points_cnt == 0)
    return;

  updateSlidingMap(md_.ray_pos_);

  md_.raycast_num_ += 1;
  md_.terrain_scan_ += 1;
  // Retain physical XYZ for support, slopes, depressions and overhead returns.
  // Sliding drops facts outside the window; lack of a return never says FREE.
  for(auto it=md_.terrain_facts_.begin();it!=md_.terrain_facts_.end();) {
    if(it->first[0]<mp_.map_bound_min_idx_.x() || it->first[0]>mp_.map_bound_max_idx_.x() ||
        it->first[1]<mp_.map_bound_min_idx_.y() || it->first[1]>mp_.map_bound_max_idx_.y())
      it=md_.terrain_facts_.erase(it);
    else ++it;
  }
  md_.low_surface_returns_.clear();
  md_.high_surface_returns_.clear();
  size_t self_endpoints=0, low_endpoints=0, high_endpoints=0, obstacle_endpoints=0;
  // One typed observation per XY cell per scan. Hazard wins over support in
  // the same scan; only repeated explicit support can retract old projections.
  std::map<std::pair<int,int>, std::pair<Eigen::Vector3i,plan_env::TerrainEvidence>> terrain_hazards;
  std::set<std::pair<int,int>> terrain_support;
  size_t projections_live=0, projections_active=0;
  for(auto it=md_.terrain_projections_.begin();it!=md_.terrain_projections_.end();) {
    if(!isInMap(it->second.id)) {it=md_.terrain_projections_.erase(it);continue;}
    // Expire an action constraint, not a measured physical obstacle. Lack of
    // returns does not constitute a free-space measurement. Sources keep their
    // kind so expiry can never erase the independent measured layer.
    if(md_.terrain_scan_-it->second.last_scan>=
       static_cast<uint32_t>(mp_.terrain_projection_timeout_frames_)) {
      setTerrainProjection(it->second.id,false);
      it=md_.terrain_projections_.erase(it);
      continue;
    }
    ++projections_live;
    if(it->second.hits>=2) ++projections_active;
    ++it;
  }

  int vox_idx;
  double length;

  // bounding box of updated region
  double min_x = mp_.map_max_boundary_(0);
  double min_y = mp_.map_max_boundary_(1);
  double min_z = mp_.map_max_boundary_(2);

  double max_x = mp_.map_min_boundary_(0);
  double max_y = mp_.map_min_boundary_(1);
  double max_z = mp_.map_min_boundary_(2);

  RayCaster raycaster;
  Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
  Eigen::Vector3d ray_pt, pt_w;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt_w = md_.proj_points_[i];

    // set flag for projected point

    if (!isInMap(pt_w))
    {
      pt_w = closetPointInMap(pt_w, md_.ray_pos_);

      length = (pt_w - md_.ray_pos_).norm();
      if (length > mp_.max_ray_length_)
      {
        pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
      }
      vox_idx = INVALID_IDX;
    }
    else
    {
      length = (pt_w - md_.ray_pos_).norm();

      const bool clearing_only = i < static_cast<int>(md_.proj_hit_flags_.size()) &&
          md_.proj_hit_flags_[i] == 0;
      if (length > mp_.max_ray_length_ || clearing_only)
      {
        if(length > mp_.max_ray_length_)
          pt_w = (pt_w - md_.ray_pos_) / length * mp_.max_ray_length_ + md_.ray_pos_;
        vox_idx = INVALID_IDX; // no invented obstacle at a valid miss endpoint
      }
      else
      {
        const uint8_t kind=i<static_cast<int>(md_.proj_hit_flags_.size()) ? md_.proj_hit_flags_[i] : 1;
        const bool inside_self_filter = isInsideSelfFilter(pt_w);
        if (inside_self_filter) {
          ++self_endpoints;
          vox_idx=setCacheOccupancy(pt_w,0);
        } else if(kind>=plan_env::kHeightFirst) {
          // Keep the measured terrain fact. Local's explicit motion profile
          // produces a separate capability constraint, never a fabricated hit.
          const double height=plan_env::decodeGroundHeight(kind);
          const plan_env::TerrainEvidence fact{pt_w,height,md_.ray_body_pos_.z(),md_.terrain_scan_};
          Eigen::Vector3i terrain_id;posToIndex(pt_w,terrain_id);
          md_.terrain_facts_[{terrain_id.x(),terrain_id.y()}].observe(fact);
          const auto verdict=terrain_motion_policy_.assess(fact);
          if(verdict==plan_env::TerrainVerdict::SUPPORT) {
            ++low_endpoints;
            md_.low_surface_returns_.push_back(pt_w);
            // Measured support revokes old floor hits at the actual endpoint.
            vox_idx=setCacheOccupancy(pt_w,0);
            Eigen::Vector3i support_id;posToIndex(pt_w,support_id);
            terrain_support.insert({support_id.x(),support_id.y()});
          } else if(verdict==plan_env::TerrainVerdict::BODY_HEIGHT_RISE ||
                    verdict==plan_env::TerrainVerdict::DROP) {
            ++obstacle_endpoints;
            md_.low_surface_returns_.push_back(pt_w);
            // XYZ remains measured; the obstruction blocks the body plane.
            Eigen::Vector3d hazard=pt_w;hazard.z()=md_.ray_body_pos_.z();
            Eigen::Vector3i hazard_id;posToIndex(hazard,hazard_id);
            if(isInMap(hazard_id)) terrain_hazards[{hazard_id.x(),hazard_id.y()}]={hazard_id,fact};
            vox_idx=INVALID_IDX;
          } else if(verdict==plan_env::TerrainVerdict::OVERHEAD) {
            ++high_endpoints;
            md_.high_surface_returns_.push_back(pt_w);
            vox_idx=setCacheOccupancy(pt_w,0);
          } else {
            // Leg-height terrain: measured, cleared at its own Z, and never a
            // body-plane obstruction.
            ++low_endpoints;
            md_.low_surface_returns_.push_back(pt_w);
            vox_idx=setCacheOccupancy(pt_w,0);
          }
        } else if (plan_env::lowReturnOutsideBodyBand(
                       pt_w,md_.ray_body_pos_,mp_.separate_low_returns_,
                       mp_.low_return_z_below_body_)) {
          ++low_endpoints;
          md_.low_surface_returns_.push_back(pt_w);
          // Reclassification also challenges an old obstacle at this exact
          // endpoint. Keep measured XYZ in its channel; never clear beyond it.
          vox_idx=setCacheOccupancy(pt_w,0);
        } else if (plan_env::highReturnOutsideBodyBand(
                       pt_w,md_.ray_body_pos_,mp_.separate_high_returns_,
                       mp_.high_return_z_above_body_)) {
          ++high_endpoints;
          md_.high_surface_returns_.push_back(pt_w);
          // An old body-band hit may now be explicitly overhead. Repeated
          // endpoint misses revoke that stale source and its inflation too.
          vox_idx=setCacheOccupancy(pt_w,0);
        } else {
          ++obstacle_endpoints;
          vox_idx=setCacheOccupancy(pt_w,1);
        }
      }
    }

    max_x = max(max_x, pt_w(0));
    max_y = max(max_y, pt_w(1));
    max_z = max(max_z, pt_w(2));

    min_x = min(min_x, pt_w(0));
    min_y = min(min_y, pt_w(1));
    min_z = min(min_z, pt_w(2));

    // raycasting between ray origin and point

    if (vox_idx != INVALID_IDX)
    {
      if (md_.flag_rayend_[vox_idx] == md_.raycast_num_)
      {
        continue;
      }
      else
      {
        md_.flag_rayend_[vox_idx] = md_.raycast_num_;
      }
    }

    // Use exactly the same transform as posToIndex and LocalMapPatch. Division
    // and multiplication can straddle an integer boundary (0.30/0.05 < 6,
    // but 0.30*20 == 6), putting ray free evidence in the wrong Z slice.
    raycaster.setInput(md_.ray_pos_ * mp_.resolution_inv_, pt_w * mp_.resolution_inv_);

    while (raycaster.step(ray_pt))
    {
      Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution_;
      length = (tmp - md_.ray_pos_).norm();

      Eigen::Vector3i traversal_id;
      posToIndex(tmp, traversal_id);
      const bool historically_occupied =
          isInMap(traversal_id) &&
          md_.occupancy_buffer_[toAddress(traversal_id)] >
              mp_.min_occupancy_log_;
      // Forward rays share cells near the sensor. Deduplicate their evidence,
      // but keep traversing so each ray can reach its own visible endpoint.
      const int traversal_addr = isInMap(traversal_id)
                                     ? toAddress(traversal_id)
                                     : INVALID_IDX;
      if (traversal_addr != INVALID_IDX &&
          md_.flag_traverse_[traversal_addr] == md_.raycast_num_)
      {
        if (plan_env::rayTraversalStopsAtOccupied(historically_occupied))
          break;
        continue;
      }
      vox_idx = setCacheOccupancy(tmp, 0);

      if (vox_idx != INVALID_IDX)
      {
        if (md_.flag_traverse_[vox_idx] == md_.raycast_num_)
        {
          continue;
        }
        else
        {
          md_.flag_traverse_[vox_idx] = md_.raycast_num_;
        }
      }

      // A ray may challenge the first historical wall cell, but it cannot use
      // that same observation to clear unknown space behind the wall.  Only a
      // later frame, after the wall cell has accumulated enough explicit free
      // evidence to become non-occupied, may continue beyond it.
      if (plan_env::rayTraversalStopsAtOccupied(historically_occupied))
        break;
    }
  }

  // Bounded symmetric evidence. A hazard frame adds one hit, a support frame
  // subtracts one, up to the confirmation bound either way. A projection needs
  // two net hits and is retracted when the support evidence has cancelled
  // every hit with no hazard in between, so a cell whose label alternates
  // -- the flicker a rough floor produces when the per-frame fit moves --
  // tops out below the projection threshold instead of latching forever.
  for(const auto& observation:terrain_hazards) {
    auto inserted=md_.terrain_projections_.try_emplace(observation.first);
    auto& record=inserted.first->second;
    if(inserted.second) record.id=observation.second.first;
    if(record.id!=observation.second.first) {
      setTerrainProjection(record.id,false);
      record.id=observation.second.first;
    }
    record.last_scan=md_.terrain_scan_;
    record.evidence=observation.second.second;
    if(record.hits<static_cast<uint8_t>(mp_.occupied_miss_confirmation_frames_))
      ++record.hits;
    if(record.hits>=2) {
      setTerrainProjection(record.id,true);
      md_.occupancy_sources_[{record.id.x(),record.id.y(),record.id.z()}].terrain=record.evidence;
    }
  }
  for(const auto& cell:terrain_support) {
    if(terrain_hazards.count(cell)) continue;
    auto found=md_.terrain_projections_.find(cell);
    if(found==md_.terrain_projections_.end()) continue;
    auto& record=found->second;
    if(record.hits>0) --record.hits;
    if(record.hits==0) {
      setTerrainProjection(record.id,false);
      md_.terrain_projections_.erase(found);
    }
  }

  RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
      "[RAY_ENDPOINT_CHANNELS] self=%zu low_surface=%zu high_surface=%zu obstacle=%zu terrain_proj=%zu terrain_active=%zu body_z=%.3f band=[%.3f,%.3f]; raw low/high endpoints are separate; typed support clears and hazards occupy",
      self_endpoints,low_endpoints,high_endpoints,obstacle_endpoints,projections_live,projections_active,md_.ray_body_pos_.z(),
      md_.ray_body_pos_.z()-mp_.low_return_z_below_body_,
      md_.ray_body_pos_.z()+mp_.high_return_z_above_body_);
  min_x = min(min_x, md_.ray_pos_(0));
  min_y = min(min_y, md_.ray_pos_(1));
  min_z = min(min_z, md_.ray_pos_(2));

  max_x = max(max_x, md_.ray_pos_(0));
  max_y = max(max_y, md_.ray_pos_(1));
  max_z = max(max_z, md_.ray_pos_(2));
  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);
  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // update occupancy cached in queue
  Eigen::Vector3d local_range_min = md_.ray_pos_ - mp_.local_update_range_;
  Eigen::Vector3d local_range_max = md_.ray_pos_ + mp_.local_update_range_;

  Eigen::Vector3i min_id, max_id;
  posToIndex(local_range_min, min_id);
  posToIndex(local_range_max, max_id);
  boundIndex(min_id);
  boundIndex(max_id);

  // std::cout << "cache all: " << md_.cache_voxel_.size() << std::endl;

  while (!md_.cache_voxel_.empty())
  {

    Eigen::Vector3i idx = md_.cache_voxel_.front();
    int idx_ctns = toAddress(idx);
    md_.cache_voxel_.pop();

    const bool hit_wins =
        md_.count_hit_[idx_ctns] >=
        md_.count_hit_and_miss_[idx_ctns] - md_.count_hit_[idx_ctns];
    double log_odds_update = hit_wins ? mp_.prob_hit_log_ : mp_.prob_miss_log_;

    md_.count_hit_[idx_ctns] = md_.count_hit_and_miss_[idx_ctns] = 0;

    if (hit_wins)
    {
      md_.occupied_miss_confirmation_count_[idx_ctns] = 0;
    }
    else if (md_.occupancy_buffer_[idx_ctns] > mp_.min_occupancy_log_)
    {
      uint8_t &confirmations =
          md_.occupied_miss_confirmation_count_[idx_ctns];
      if (confirmations < 255)
        ++confirmations;
      if (!plan_env::occupiedMissHasEnoughConfirmation(
              confirmations, mp_.occupied_miss_confirmation_frames_))
        continue;
    }
    else
    {
      md_.occupied_miss_confirmation_count_[idx_ctns] = 0;
    }

    if (log_odds_update >= 0 && md_.occupancy_buffer_[idx_ctns] >= mp_.clamp_max_log_)
    {
      auto source=md_.occupancy_sources_.find({idx.x(),idx.y(),idx.z()});
      if(source!=md_.occupancy_sources_.end()) source->second.measured_scan=md_.terrain_scan_;
      continue;
    }
    else if (log_odds_update <= 0 && md_.occupancy_buffer_[idx_ctns] <= mp_.clamp_min_log_)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
      continue;
    }

    bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                    idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
    if (!in_local)
    {
      applyOccupancyUpdate(idx, mp_.clamp_min_log_);
    }

    const double new_log_odds =
        std::min(std::max(md_.occupancy_buffer_[idx_ctns] + log_odds_update, mp_.clamp_min_log_),
                 mp_.clamp_max_log_);
    applyOccupancyUpdate(idx, new_log_odds);
  }
}

Eigen::Vector3d GridMap::closetPointInMap(const Eigen::Vector3d &pt, const Eigen::Vector3d &ray_pos)
{
  Eigen::Vector3d diff = pt - ray_pos;
  Eigen::Vector3d max_tc = mp_.map_max_boundary_ - ray_pos;
  Eigen::Vector3d min_tc = mp_.map_min_boundary_ - ray_pos;

  double min_t = 1000000;

  for (int i = 0; i < 3; ++i)
  {
    if (fabs(diff[i]) > 0)
    {

      double t1 = max_tc[i] / diff[i];
      if (t1 > 0 && t1 < min_t)
        min_t = t1;

      double t2 = min_tc[i] / diff[i];
      if (t2 > 0 && t2 < min_t)
        min_t = t2;
    }
  }

  return ray_pos + (min_t - 1e-3) * diff;
}

void GridMap::visCallback()
{
  // Read the last completely fused frame without contending with 100 Hz poses
  // or the 50 ms writer timer. Their phase used to drop every other RViz tick.
  visualization_refresh_requested_.store(true);
  const auto view = std::atomic_load(&latest_visualization_snapshot_);
  if (view)
  {
    const auto stamp = rclcpp::Time(view->stamp_ns, node_->get_clock()->get_clock_type());
    if (map_pub_->get_subscription_count() > 0 && !view->occupied.empty())
      publishOccupancyCloud(view->occupied, *view->inflated,
          view->maximum_z, stamp, map_pub_);
    if (map_inf_pub_->get_subscription_count() > 0)
      publishOccupancyCloud(view->inflated->buffer, *view->inflated,
          view->maximum_z, stamp, map_inf_pub_);
    if (body_obstacle_pub_->get_subscription_count() > 0)
      publishOccupancyCloud(view->inflated->buffer, *view->inflated,
          view->maximum_z, stamp, body_obstacle_pub_, view->body_minimum_z);
    if (low_surface_pub_->get_subscription_count() > 0) {
      if (mp_.separate_low_returns_)
        publishLowSurfaceReturns(view->low_surface_returns,stamp);
      else if (!view->occupied.empty())
        publishOccupancyCloud(view->occupied, *view->inflated,
            std::min(view->maximum_z, view->body_minimum_z - 1e-6),
            stamp, low_surface_pub_);
    }
    if(terrain_evidence_pub_->get_subscription_count()>0)
      publishTerrainFacts(view->terrain_facts,stamp);
    if (high_surface_pub_->get_subscription_count() > 0)
      publishHighSurfaceReturns(view->high_surface_returns,stamp);
  }
  // Small, optional live-frame displays still use a non-blocking read lock.
  std::shared_lock<std::shared_mutex> lock(map_mutex_, std::try_to_lock);
  if (lock.owns_lock())
  {
    publishSlidingMapFrame();
    publishSlidingMapBBox();
    publishDepthCloud();
  }
}

void GridMap::updateVisualizationSnapshotLocked(InflatedOccupancySnapshotPtr snapshot)
{
  const bool terrain_wanted=terrain_evidence_pub_->get_subscription_count()>0;
  const bool raw_wanted = map_pub_->get_subscription_count() > 0 ||
      low_surface_pub_->get_subscription_count() > 0 ||
      high_surface_pub_->get_subscription_count() > 0;
  const bool inflated_wanted = map_inf_pub_->get_subscription_count() > 0 ||
      body_obstacle_pub_->get_subscription_count() > 0;
  if ((!raw_wanted && !inflated_wanted && !terrain_wanted) ||
      !visualization_refresh_requested_.exchange(false))
    return;
  auto view = std::make_shared<VisualizationSnapshot>();
  view->inflated = std::move(snapshot);
  if (raw_wanted)
  {
    view->occupied.resize(md_.occupancy_buffer_.size());
    for(size_t i=0;i<view->occupied.size();++i)
      view->occupied[i]=md_.occupancy_buffer_[i]>mp_.min_occupancy_log_;
  }
  if (low_surface_pub_->get_subscription_count() > 0)
    view->low_surface_returns=md_.low_surface_returns_;
  if (high_surface_pub_->get_subscription_count() > 0)
    view->high_surface_returns=md_.high_surface_returns_;
  view->maximum_z = md_.has_ray_pose_
      ? md_.ray_pos_(2) + mp_.vis_height_
      : std::numeric_limits<double>::infinity();
  view->body_minimum_z = local_patch_reference_z_initialized_
      ? local_patch_reference_z_ - mp_.body_obstacle_z_below_
      : -std::numeric_limits<double>::infinity();
  if(terrain_wanted) {
    for(const auto& cell:view->inflated->terrain_facts) {
      view->terrain_facts.push_back(cell.second.lowest);
      if(!cell.second.lowest.measured.isApprox(cell.second.highest.measured))
        view->terrain_facts.push_back(cell.second.highest);
    }
  }
  view->stamp_ns = node_->now().nanoseconds();
  std::atomic_store(&latest_visualization_snapshot_,
      std::shared_ptr<const VisualizationSnapshot>(std::move(view)));
}

void GridMap::updateOccupancyCallback()
{
  const auto callback_started = std::chrono::steady_clock::now();
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  const auto lock_acquired = std::chrono::steady_clock::now();
  if (!md_.occ_need_update_)
  {
    return;
  }

  /* update occupancy */
  // ros::Time t1, t2, t3, t4;
  // t1 = ros::Time::now();

  if (!md_.use_cloud_update_)
    projectDepthImage();
  // t2 = ros::Time::now();
  const int projected_points = md_.proj_points_cnt;
  raycastProcess();
  map_revision_.fetch_add(1);
  last_map_update_ns_.store(node_->now().nanoseconds());
  const auto fusion_finished = std::chrono::steady_clock::now();
  auto snapshot = buildInflatedOccupancySnapshotLocked();
  std::atomic_store(&latest_inflated_snapshot_, snapshot);
  updateVisualizationSnapshotLocked(snapshot);
  const auto snapshot_finished = std::chrono::steady_clock::now();
  // t3 = ros::Time::now();

  // t4 = ros::Time::now();

  // cout << setprecision(7);
  // cout << "t2=" << (t2-t1).toSec() << " t3=" << (t3-t2).toSec() << " t4=" << (t4-t3).toSec() << endl;;

  // md_.fuse_time_ += (t2 - t1).toSec();
  // md_.max_fuse_time_ = max(md_.max_fuse_time_, (t2 - t1).toSec());

  // if (mp_.show_occ_time_)
  //   ROS_WARN("Fusion: cur t = %lf, avg t = %lf, max t = %lf", (t2 - t1).toSec(),
  //            md_.fuse_time_ / md_.update_num_, md_.max_fuse_time_);

  md_.occ_need_update_ = false;
  md_.use_cloud_update_ = false;
  lock.unlock();


  const double lock_wait_ms = std::chrono::duration<double, std::milli>(
      lock_acquired - callback_started).count();
  const double fusion_ms = std::chrono::duration<double, std::milli>(
      fusion_finished - lock_acquired).count();
  const double snapshot_ms = std::chrono::duration<double, std::milli>(
      snapshot_finished - fusion_finished).count();
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[MAP_FUSION_TIMING] lock_wait=%.1fms raycast=%.1fms snapshot=%.1fms points=%d revision=%lu",
      lock_wait_ms, fusion_ms, snapshot_ms, projected_points,
      static_cast<unsigned long>(snapshot->revision));
}

namespace {
struct Fnv1aHash {
  uint64_t value{1469598103934665603ULL};
  void add(const void* data, size_t size) {
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < size; ++i) {
      value ^= bytes[i];
      value *= 1099511628211ULL;
    }
  }
  template <typename T>
  void add(const T& v) { add(&v, sizeof(T)); }
};

struct TerrainTileAccumulator {
  std::vector<std::array<double, 3>> ground_points;
  uint32_t support{0}, rise{0}, drop{0}, overhead{0}, columns{0}, last_scan{0};
};
}  // namespace

void GridMap::fillTerrainTilesLocked(
    double origin_x, double origin_y, uint32_t width, uint32_t height,
    std::vector<scan_planner_msgs::msg::TerrainTileSummary>& tiles) const
{
  tiles.clear();
  if (md_.terrain_facts_.empty())
    return;
  const double tile_size = local_patch_terrain_tile_size_;
  const double max_x = origin_x + width * local_patch_resolution_;
  const double max_y = origin_y + height * local_patch_resolution_;
  std::map<std::pair<int, int>, TerrainTileAccumulator> accumulated;
  for (const auto& entry : md_.terrain_facts_)
  {
    const double x = (entry.first[0] + 0.5) * mp_.resolution_;
    const double y = (entry.first[1] + 0.5) * mp_.resolution_;
    if (x < origin_x || x >= max_x || y < origin_y || y >= max_y)
      continue;
    const auto& column = entry.second;
    if (column.scan == 0)
      continue;
    const double ground_z =
        column.lowest.measured.z() - column.lowest.ground_relative_height;
    const auto tx = static_cast<int>(std::floor(x / tile_size));
    const auto ty = static_cast<int>(std::floor(y / tile_size));
    auto& tile = accumulated[{tx, ty}];
    tile.ground_points.push_back({x, y, ground_z});
    ++tile.columns;
    tile.last_scan = std::max(tile.last_scan, column.scan);
    const auto low = terrain_motion_policy_.assess(column.lowest);
    const auto high = terrain_motion_policy_.assess(column.highest);
    if (low == plan_env::TerrainVerdict::SUPPORT &&
        high == plan_env::TerrainVerdict::SUPPORT)
      ++tile.support;
    if (high == plan_env::TerrainVerdict::BODY_HEIGHT_RISE)
      ++tile.rise;
    if (low == plan_env::TerrainVerdict::DROP)
      ++tile.drop;
    if (high == plan_env::TerrainVerdict::OVERHEAD)
      ++tile.overhead;
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  for (const auto& entry : accumulated)
  {
    const auto& key = entry.first;
    const auto& accumulated_tile = entry.second;
    scan_planner_msgs::msg::TerrainTileSummary tile;
    tile.tile_x = key.first;
    tile.tile_y = key.second;
    tile.tile_size = static_cast<float>(tile_size);
    tile.measured_columns = accumulated_tile.columns;
    tile.support_cells = accumulated_tile.support;
    tile.rise_cells = accumulated_tile.rise;
    tile.drop_cells = accumulated_tile.drop;
    tile.overhead_cells = accumulated_tile.overhead;
    tile.last_scan = accumulated_tile.last_scan;
    tile.ground_min = tile.ground_median = tile.ground_max = nan;
    tile.slope = tile.roughness = nan;
    std::vector<double> zs;
    zs.reserve(accumulated_tile.ground_points.size());
    double sx = 0, sy = 0, sz = 0, sxx = 0, sxy = 0, syy = 0, sxz = 0, syz = 0;
    for (const auto& point : accumulated_tile.ground_points)
    {
      zs.push_back(point[2]);
      sx += point[0];
      sy += point[1];
      sz += point[2];
      sxx += point[0] * point[0];
      sxy += point[0] * point[1];
      syy += point[1] * point[1];
      sxz += point[0] * point[2];
      syz += point[1] * point[2];
    }
    const auto [minimum, maximum] = std::minmax_element(zs.begin(), zs.end());
    tile.ground_min = static_cast<float>(*minimum);
    tile.ground_max = static_cast<float>(*maximum);
    auto middle = zs.begin() + zs.size() / 2;
    std::nth_element(zs.begin(), middle, zs.end());
    tile.ground_median = static_cast<float>(*middle);
    const double n = static_cast<double>(zs.size());
    Eigen::Matrix3d normal;
    normal << n, sx, sy, sx, sxx, sxy, sy, sxy, syy;
    if (zs.size() >= 3 && std::abs(normal.determinant()) > 1e-9)
    {
      const Eigen::Vector3d coefficients =
          normal.ldlt().solve(Eigen::Vector3d(sz, sxz, syz));
      double residual = 0.0;
      for (const auto& point : accumulated_tile.ground_points)
      {
        const double error =
            point[2] - (coefficients(0) + coefficients(1) * point[0] +
                        coefficients(2) * point[1]);
        residual += error * error;
      }
      tile.slope = static_cast<float>(std::atan(
          std::hypot(coefficients(1), coefficients(2))));
      tile.roughness = static_cast<float>(std::sqrt(residual / n));
    }
    Fnv1aHash hash;
    hash.add(tile.tile_x);
    hash.add(tile.tile_y);
    hash.add(std::llround(tile.ground_min * 1000.0));
    hash.add(std::llround(tile.ground_median * 1000.0));
    hash.add(std::llround(tile.ground_max * 1000.0));
    hash.add(std::llround(tile.slope * 1000.0));
    hash.add(std::llround(tile.roughness * 1000.0));
    hash.add(tile.support_cells);
    hash.add(tile.rise_cells);
    hash.add(tile.drop_cells);
    hash.add(tile.overhead_cells);
    hash.add(tile.measured_columns);
    tile.content_revision = hash.value;
    tiles.push_back(tile);
  }
}

void GridMap::publishLocalMapPatch()
{
  // A fixed timer publishes the newest fused revision. A throttle inside a
  // 10 Hz sensor callback would quantize 0.25 s to 0.30 s instead.
  const auto started = std::chrono::steady_clock::now();
  scan_planner_msgs::msg::LocalMapPatch patch;
  std::shared_lock<std::shared_mutex> lock(map_mutex_);
  if (!buildLocalMapPatchLocked(patch))
    return;
  patch.generation_ms = static_cast<float>(std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count());
  last_local_patch_revision_ = patch.map_revision;
  lock.unlock();
  local_patch_pub_->publish(patch);
  RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
      "[LOCAL_MAP_PATCH] revision=%lu size=%ux%u generation=%.1fms period=%.3fs",
      static_cast<unsigned long>(patch.map_revision), patch.width, patch.height,
      patch.generation_ms, local_patch_period_);
}

bool GridMap::buildLocalMapPatchLocked(
    scan_planner_msgs::msg::LocalMapPatch& patch) const
{
  if (!local_patch_enabled_ || !local_patch_pub_ ||
      local_patch_pub_->get_subscription_count() == 0)
    return false;

  const uint64_t revision = map_revision_.load();
  if (revision == 0 || revision == last_local_patch_revision_)
    return false;

  const double min_x = mp_.map_bound_min_idx_(0) * mp_.resolution_;
  const double min_y = mp_.map_bound_min_idx_(1) * mp_.resolution_;
  const double max_x = (mp_.map_bound_max_idx_(0) + 1) * mp_.resolution_;
  const double max_y = (mp_.map_bound_max_idx_(1) + 1) * mp_.resolution_;
  const double origin_x =
      std::floor(min_x / local_patch_resolution_) * local_patch_resolution_;
  const double origin_y =
      std::floor(min_y / local_patch_resolution_) * local_patch_resolution_;
  patch.resolution = static_cast<float>(local_patch_resolution_);
  patch.origin.x = origin_x;
  patch.origin.y = origin_y;
  const double reference_z =
      local_patch_follow_body_z_ && local_patch_reference_z_initialized_
          ? local_patch_reference_z_
          : local_patch_z_;
  patch.origin.z = reference_z;
  patch.width = static_cast<uint32_t>(
      std::ceil((max_x - origin_x) / local_patch_resolution_));
  patch.height = static_cast<uint32_t>(
      std::ceil((max_y - origin_y) / local_patch_resolution_));
  patch.occupancy.assign(static_cast<size_t>(patch.width) * patch.height, -1);

  const int z =
      static_cast<int>(std::floor(reference_z * mp_.resolution_inv_));
  if (z >= mp_.map_bound_min_idx_(2) && z <= mp_.map_bound_max_idx_(2))
  {
    auto address = [this, z](int x, int y) {
      int lx = x % mp_.map_voxel_num_(0);
      int ly = y % mp_.map_voxel_num_(1);
      int lz = z % mp_.map_voxel_num_(2);
      if (lx < 0) lx += mp_.map_voxel_num_(0);
      if (ly < 0) ly += mp_.map_voxel_num_(1);
      if (lz < 0) lz += mp_.map_voxel_num_(2);
      return lx * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) +
             ly * mp_.map_voxel_num_(2) + lz;
    };
    for (uint32_t row = 0; row < patch.height; ++row)
    {
      const int y = static_cast<int>(std::floor(
          (origin_y + (row + 0.5) * local_patch_resolution_) *
          mp_.resolution_inv_));
      if (y < mp_.map_bound_min_idx_(1) || y > mp_.map_bound_max_idx_(1))
        continue;
      for (uint32_t col = 0; col < patch.width; ++col)
      {
        const int x = static_cast<int>(std::floor(
            (origin_x + (col + 0.5) * local_patch_resolution_) *
            mp_.resolution_inv_));
        if (x < mp_.map_bound_min_idx_(0) || x > mp_.map_bound_max_idx_(0))
          continue;
        const int addr = address(x, y);
        // Raw measured slice: no inflation, no hazard projection. Global owns
        // clearance; terrain evidence travels separately in terrain_tiles.
        const double value = md_.occupancy_buffer_[addr];
        const bool unknown =
            value <= mp_.clamp_min_log_ - mp_.unknown_flag_ + 1e-9;
        const int8_t state = unknown ? -1 : (value > 0.0 ? 100 : 0);
        patch.occupancy[static_cast<size_t>(row) * patch.width + col] = state;
      }
    }
  }
  patch.header.stamp = node_->now();
  patch.header.frame_id = mp_.frame_id_;
  patch.map_revision = revision;
  patch.pose_revision = 0;
  patch.scan = md_.terrain_scan_;
  fillTerrainTilesLocked(origin_x, origin_y, patch.width, patch.height,
                         patch.terrain_tiles);
  Fnv1aHash content_hash;
  content_hash.add(patch.occupancy.data(), patch.occupancy.size());
  for (const auto& tile : patch.terrain_tiles)
  {
    content_hash.add(tile.tile_x);
    content_hash.add(tile.tile_y);
    content_hash.add(tile.content_revision);
  }
  if (!last_local_patch_content_initialized_ ||
      content_hash.value != last_local_patch_content_hash_)
  {
    ++content_revision_;
    last_local_patch_content_hash_ = content_hash.value;
    last_local_patch_content_initialized_ = true;
  }
  patch.content_revision = content_revision_;
  return true;
}

void GridMap::depthPoseCallback(const sensor_msgs::msg::Image::ConstSharedPtr &img,
                                const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  if (mp_.sensor_type_ != "depth")
    return;

  /* get depth image */
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(img, img->encoding);

  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
  {
    (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, mp_.k_depth_scaling_factor_);
  }
  cv_ptr->image.copyTo(md_.depth_image_);

  // std::cout << "depth: " << md_.depth_image_.cols << ", " << md_.depth_image_.rows << std::endl;

  /* get pose */
  const geometry_msgs::msg::Pose &sensor_pose = pose->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
    return;
  ray_q.normalize();

  Eigen::Vector3d ray_pos(sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.depth_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.depth_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }

  nav_msgs::msg::Odometry extrinsic_pose = *pose;
  extrinsic_pose.pose.pose.position.x = ray_pos.x();
  extrinsic_pose.pose.pose.position.y = ray_pos.y();
  extrinsic_pose.pose.pose.position.z = ray_pos.z();
  extrinsic_pose.pose.pose.orientation.x = ray_q.x();
  extrinsic_pose.pose.pose.orientation.y = ray_q.y();
  extrinsic_pose.pose.pose.orientation.z = ray_q.z();
  extrinsic_pose.pose.pose.orientation.w = ray_q.w();
  extrinsic_pose.child_frame_id =
      pose->child_frame_id.empty() ? "sensor_extrinsic" : pose->child_frame_id + "_extrinsic";
  extrinsic_pose_pub_->publish(extrinsic_pose);

  plan_env::TimedPose body_at_scan;
  if (!bodyPoseAtScanLocked(rclcpp::Time(img->header.stamp).nanoseconds(),body_at_scan))
    return;
  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.ray_body_pos_=body_at_scan.position;
  md_.ray_body_q_=body_at_scan.orientation;
  md_.proj_hit_flags_.clear();
  md_.use_cloud_update_ = false;
  updateSlidingMap(md_.ray_pos_);
  if (isInMap(md_.ray_pos_))
  {
    md_.has_ray_pose_ = true;
    md_.update_num_ += 1;
    md_.occ_need_update_ = true;
  }
  else
  {
    md_.occ_need_update_ = false;
  }
}

void GridMap::sensorPoseCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose_msg)
{
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  if (mp_.sensor_type_ != "lidar")
  {
    return;
  }

  const geometry_msgs::msg::Pose &sensor_pose = pose_msg->pose.pose;
  Eigen::Quaterniond ray_q(sensor_pose.orientation.w, sensor_pose.orientation.x,
                           sensor_pose.orientation.y, sensor_pose.orientation.z);
  if (ray_q.norm() < 1e-6)
  {
    return;
  }
  ray_q.normalize();

  const int64_t stamp_ns = rclcpp::Time(pose_msg->header.stamp).nanoseconds();
  if (stamp_ns <= 0)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[GridMap] dropping sensor_pose with a zero timestamp");
    return;
  }

  const Eigen::Vector3d body_pos(
      sensor_pose.position.x, sensor_pose.position.y, sensor_pose.position.z);
  const plan_env::TimedPose timed_pose{stamp_ns, body_pos, ray_q};
  const auto insertion = std::lower_bound(
      lidar_pose_history_.begin(), lidar_pose_history_.end(), stamp_ns,
      [](const plan_env::TimedPose& pose, int64_t stamp) {
        return pose.stamp_ns < stamp;
      });
  if (insertion != lidar_pose_history_.end() && insertion->stamp_ns == stamp_ns)
    *insertion = timed_pose;
  else
    lidar_pose_history_.insert(insertion, timed_pose);
  const int64_t oldest_allowed = stamp_ns - static_cast<int64_t>(
      mp_.pose_history_seconds_ * 1e9);
  while (!lidar_pose_history_.empty() &&
         lidar_pose_history_.front().stamp_ns < oldest_allowed)
    lidar_pose_history_.pop_front();

  Eigen::Vector3d ray_pos = body_pos;
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d pose_r = ray_q.toRotationMatrix();
    ray_pos += pose_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(pose_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0));
    ray_q.normalize();
  }
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
  {
    return;
  }

  // A newer pose must not overwrite the origin of an unconsumed scan.
  // Its projected endpoints and origin form one timestamped observation.
  if(!md_.occ_need_update_) {
    md_.ray_pos_ = ray_pos;
    md_.ray_q_ = ray_q;
    updateSlidingMap(ray_pos);
  }
  md_.has_ray_pose_ = true;
}

void GridMap::slidingMapFrameCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &pose)
{
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  const geometry_msgs::msg::Point &pos = pose->pose.pose.position;
  md_.sliding_map_frame_pos_ = Eigen::Vector3d(pos.x, pos.y, pos.z);
  const auto& q=pose->pose.pose.orientation;
  Eigen::Quaterniond body_q(q.w,q.x,q.y,q.z);
  const int64_t body_stamp=rclcpp::Time(pose->header.stamp).nanoseconds();
  if (body_stamp>0 && md_.sliding_map_frame_pos_.allFinite() &&
      body_q.coeffs().allFinite() && body_q.norm()>1e-6) {
    const plan_env::TimedPose sample{body_stamp,md_.sliding_map_frame_pos_,body_q.normalized()};
    const auto insertion=std::lower_bound(body_pose_history_.begin(),body_pose_history_.end(),
        body_stamp,[](const auto& item,int64_t stamp){return item.stamp_ns<stamp;});
    if (insertion!=body_pose_history_.end() && insertion->stamp_ns==body_stamp) *insertion=sample;
    else body_pose_history_.insert(insertion,sample);
    const int64_t oldest=body_stamp-static_cast<int64_t>(mp_.pose_history_seconds_*1e9);
    while (!body_pose_history_.empty() && body_pose_history_.front().stamp_ns<oldest)
      body_pose_history_.pop_front();
  }
  if (!local_patch_follow_body_z_ || !std::isfinite(pos.z) ||
      !locomotionReady())
    return;

  int64_t stamp_ns = rclcpp::Time(pose->header.stamp).nanoseconds();
  if (stamp_ns <= 0)
    stamp_ns = node_->now().nanoseconds();
  if (!local_patch_reference_z_initialized_)
  {
    local_patch_reference_z_ = pos.z;
    local_patch_reference_stamp_ns_ = stamp_ns;
    local_patch_reference_z_initialized_ = true;
    return;
  }
  if (stamp_ns <= local_patch_reference_stamp_ns_)
    return;

  const double dt_seconds = std::min(
      0.5, static_cast<double>(stamp_ns - local_patch_reference_stamp_ns_) * 1e-9);
  local_patch_reference_z_ = plan_env::updateHeightReference(
      local_patch_reference_z_, pos.z, dt_seconds,
      local_patch_z_filter_tau_, local_patch_z_max_rate_);
  local_patch_reference_stamp_ns_ = stamp_ns;
}

void GridMap::usePlanningClearanceMarginForCurrentThread(double margin) const
{
  tls_clearance_owner_ = this;
  tls_planning_clearance_margin_ = std::max(0.0, margin);
  tls_clearance_offsets_.clear();
  const int cells = static_cast<int>(std::ceil(
      tls_planning_clearance_margin_ / mp_.resolution_));
  for (int x = -cells; x <= cells; ++x)
    for (int y = -cells; y <= cells; ++y)
    {
      const Eigen::Vector2d offset(
          x * mp_.resolution_, y * mp_.resolution_);
      if (offset.norm() <= tls_planning_clearance_margin_ + 1e-9)
        tls_clearance_offsets_.push_back(offset);
    }
  if (tls_clearance_offsets_.empty())
    tls_clearance_offsets_.push_back(Eigen::Vector2d::Zero());
}

void GridMap::clearPlanningClearanceMarginForCurrentThread() const
{
  if (tls_clearance_owner_ != this)
    return;
  tls_clearance_owner_ = nullptr;
  tls_planning_clearance_margin_ = 0.0;
  tls_clearance_offsets_.clear();
}

bool GridMap::bodyPoseAtScanLocked(int64_t stamp_ns, plan_env::TimedPose& pose) const
{
  const auto result=plan_env::lookupTimedPose(body_pose_history_,stamp_ns,
      static_cast<int64_t>(mp_.max_interpolation_gap_seconds_*1e9),
      static_cast<int64_t>(mp_.max_nearest_pose_age_seconds_*1e9),pose);
  return result==plan_env::PoseLookupResult::EXACT ||
      result==plan_env::PoseLookupResult::INTERPOLATED ||
      result==plan_env::PoseLookupResult::NEAREST;
}

void GridMap::locomotionStateCallback(
    const scan_planner_msgs::msg::LocomotionState::ConstSharedPtr &msg)
{
  perception_pose_valid_.store(msg->perception_pose_valid);
  locomotion_state_stamp_ns_.store(node_->now().nanoseconds());
}

bool GridMap::locomotionReady() const
{
  const int64_t stamp = locomotion_state_stamp_ns_.load();
  return stamp > 0 && perception_pose_valid_.load() &&
         (node_->now().nanoseconds() - stamp) <= 500000000LL;
}

void GridMap::cloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &img)
{
  if (!locomotionReady())
    return;
  const auto callback_started = std::chrono::steady_clock::now();
  if (mp_.sensor_type_ != "lidar")
  {
    return;
  }

  // Identified 3-D rays keep their original organized indices. The UINT8
  // return type travels alongside XYZ; a clearing limit is never an obstacle.
  size_t return_type_offset=0;
  if (mp_.use_ray_cloud_) {
    const auto field=std::find_if(img->fields.begin(),img->fields.end(),
        [](const auto& value){return value.name=="return_type";});
    if (field==img->fields.end() || field->datatype!=sensor_msgs::msg::PointField::UINT8 ||
        field->count!=1 || field->offset>=img->point_step || img->width==0 ||
        img->height==0 || img->row_step<static_cast<uint64_t>(img->width)*img->point_step ||
        img->data.size()<static_cast<uint64_t>(img->row_step)*img->height) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
          "[GridMap] dropping malformed identified 3-D ray cloud");
      return;
    }
    return_type_offset=field->offset;
  }
  pcl::PointCloud<pcl::PointXYZ> latest_cloud;
  pcl::fromROSMsg(*img, latest_cloud);
  const auto decoded = std::chrono::steady_clock::now();
  if (latest_cloud.points.empty())
  {
    return;
  }

  const int64_t cloud_stamp_ns = rclcpp::Time(img->header.stamp).nanoseconds();
  if (cloud_stamp_ns <= 0)
  {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "[GridMap] dropping lidar cloud with a zero timestamp");
    return;
  }

  plan_env::TimedPose pose_at_scan, body_at_scan;
  bool body_pose_valid=false;
  plan_env::PoseLookupResult pose_result;
  size_t pose_history_size = 0;
  {
    std::shared_lock<std::shared_mutex> lock(map_mutex_);
    pose_history_size = lidar_pose_history_.size();
    body_pose_valid=bodyPoseAtScanLocked(cloud_stamp_ns,body_at_scan);
    pose_result = plan_env::lookupTimedPose(
        lidar_pose_history_, cloud_stamp_ns,
        static_cast<int64_t>(mp_.max_interpolation_gap_seconds_ * 1e9),
        static_cast<int64_t>(mp_.max_nearest_pose_age_seconds_ * 1e9),
        pose_at_scan);
  }
  if (pose_result != plan_env::PoseLookupResult::EXACT &&
      pose_result != plan_env::PoseLookupResult::INTERPOLATED &&
      pose_result != plan_env::PoseLookupResult::NEAREST)
  {
    RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 1000,
        "[GridMap] dropping lidar cloud without a time-aligned pose (result=%d)",
        static_cast<int>(pose_result));
    return;
  }

  if (!body_pose_valid) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(),*node_->get_clock(),1000,
        "[GridMap] dropping cloud without capture-time body pose for self/low filtering");
    return;
  }
  Eigen::Vector3d ray_pos = pose_at_scan.position;
  Eigen::Quaterniond ray_q = pose_at_scan.orientation;
  if (mp_.need_extrinsic_)
  {
    const Eigen::Matrix3d body_r = ray_q.toRotationMatrix();
    ray_pos += body_r * mp_.lidar_extrinsic_.block<3, 1>(0, 3);
    ray_q = Eigen::Quaterniond(
        body_r * mp_.lidar_extrinsic_.block<3, 3>(0, 0)).normalized();
  }
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[CLOUD_POSE_SYNC] mode=%d stamp_ns=%ld history=%zu lidar_origin=(%.3f,%.3f,%.3f)",
      static_cast<int>(pose_result), static_cast<long>(cloud_stamp_ns),
      pose_history_size, ray_pos.x(), ray_pos.y(), ray_pos.z());

  const Eigen::Matrix3d sensor_r = ray_q.toRotationMatrix();
  if (!std::isfinite(ray_pos.x()) || !std::isfinite(ray_pos.y()) || !std::isfinite(ray_pos.z()))
  {
    return;
  }

  std::vector<Eigen::Vector3d> projected_points;
  projected_points.reserve(latest_cloud.points.size());
  std::vector<uint8_t> projected_hits;
  if (mp_.use_ray_cloud_) projected_hits.reserve(latest_cloud.points.size());

  for (size_t i = 0; i < latest_cloud.points.size(); ++i)
  {
    uint8_t return_type=1;
    if (mp_.use_ray_cloud_) {
      const size_t offset=(i/img->width)*img->row_step+
          (i%img->width)*img->point_step+return_type_offset;
      return_type=img->data[offset];
      // Wire contract (plan_env/ground_height.hpp): 0 is invalid, 1 a measured
      // return without a height estimate, 2 a measured no-return, 3..252 a
      // measured return carrying its height above the observed local ground
      // (1 cm steps over [-0.50, +1.99] m), 253..255 reserved and treated as
      // unknown. Nothing is dropped just because a byte used to be a label.
      if (return_type<1) continue;
      if (return_type>plan_env::kHeightLast) return_type=plan_env::kGroundUnknown;
    }
    const pcl::PointXYZ &pt = latest_cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      continue;

    Eigen::Vector3d pt_world;
    if (mp_.cloud_is_world_)
    {
      pt_world = Eigen::Vector3d(pt.x, pt.y, pt.z);
    }
    else
    {
      const Eigen::Vector3d pt_sensor(pt.x, pt.y, pt.z);
      pt_world = sensor_r * pt_sensor + ray_pos;
    }
    const Eigen::Vector3d devi = pt_world - ray_pos;
    const double ray_length = devi.norm();
    const bool in_local_range =
        fabs(devi(0)) <= mp_.local_update_range_(0) && fabs(devi(1)) <= mp_.local_update_range_(1) &&
        fabs(devi(2)) <= mp_.local_update_range_(2);
    if (!in_local_range && ray_length <= mp_.max_ray_length_)
      continue;

    projected_points.push_back(pt_world);
    if (mp_.use_ray_cloud_) projected_hits.push_back(return_type==2 ? 0 : return_type);
  }

  const auto filtered = std::chrono::steady_clock::now();
  if (projected_points.empty())
  {
    return;
  }

  const auto lock_requested = std::chrono::steady_clock::now();
  std::unique_lock<std::shared_mutex> lock(map_mutex_);
  const auto lock_acquired = std::chrono::steady_clock::now();
  updateSlidingMap(ray_pos);
  md_.ray_pos_ = ray_pos;
  md_.ray_q_ = ray_q;
  md_.ray_body_pos_=body_at_scan.position;
  md_.ray_body_q_=body_at_scan.orientation;
  md_.has_cloud_ = true;
  md_.proj_points_.assign(projected_points.begin(), projected_points.end());
  md_.proj_hit_flags_=std::move(projected_hits);
  md_.proj_points_cnt = static_cast<int>(projected_points.size());
  md_.use_cloud_update_ = true;
  md_.occ_need_update_ = true;

  const auto committed = std::chrono::steady_clock::now();
  lock.unlock();
  const double decode_ms = std::chrono::duration<double, std::milli>(
      decoded - callback_started).count();
  const double filter_ms = std::chrono::duration<double, std::milli>(
      filtered - decoded).count();
  const double lock_wait_ms = std::chrono::duration<double, std::milli>(
      lock_acquired - lock_requested).count();
  const double commit_ms = std::chrono::duration<double, std::milli>(
      committed - lock_acquired).count();
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[CLOUD_TIMING] decode=%.1fms filter=%.1fms lock_wait=%.1fms commit=%.1fms input=%zu accepted=%zu",
      decode_ms, filter_ms, lock_wait_ms, commit_ms,
      latest_cloud.points.size(), projected_points.size());
}

void GridMap::scanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan) {
  if(!locomotionReady() || !mp_.use_laser_scan_ || scan->ranges.empty() ||
      scan->ranges.size()>16384 || scan->time_increment!=0. ||
      !std::isfinite(scan->angle_min) || !std::isfinite(scan->angle_increment) ||
      scan->angle_increment==0.) return;
  const int64_t stamp=rclcpp::Time(scan->header.stamp).nanoseconds();
  if(stamp<=0) return;
  const int64_t clock_now=node_->now().nanoseconds();
  if(clock_now<last_laser_clock_ns_) last_laser_stamp_ns_=0;
  last_laser_clock_ns_=clock_now;
  if(stamp<=last_laser_stamp_ns_) return;
  plan_env::TimedPose pose,body_at_scan;
  bool body_pose_valid=false;
  plan_env::PoseLookupResult result;
  { std::shared_lock<std::shared_mutex> lock(map_mutex_);
    body_pose_valid=bodyPoseAtScanLocked(stamp,body_at_scan);
    result=plan_env::lookupTimedPose(lidar_pose_history_,stamp,
        static_cast<int64_t>(mp_.max_interpolation_gap_seconds_*1e9),
        static_cast<int64_t>(mp_.max_nearest_pose_age_seconds_*1e9),pose); }
  if(result!=plan_env::PoseLookupResult::EXACT &&
      result!=plan_env::PoseLookupResult::INTERPOLATED &&
      result!=plan_env::PoseLookupResult::NEAREST) return;
  if (!body_pose_valid) return;
  Eigen::Vector3d origin=pose.position;
  Eigen::Quaterniond orientation=pose.orientation;
  if(mp_.need_extrinsic_) {
    origin+=orientation.toRotationMatrix()*mp_.lidar_extrinsic_.block<3,1>(0,3);
    orientation=Eigen::Quaterniond(orientation.toRotationMatrix()*
        mp_.lidar_extrinsic_.block<3,3>(0,0)).normalized();
  }
  if(!origin.allFinite() || !orientation.coeffs().allFinite()) return;
  std::vector<Eigen::Vector3d> points;std::vector<uint8_t> hits;
  points.reserve(scan->ranges.size());hits.reserve(scan->ranges.size());
  for(size_t i=0;i<scan->ranges.size();++i) {
    const auto ray=plan_env::laserRangeEvidence(scan->ranges[i],scan->range_min,
        scan->range_max,mp_.no_return_range_);
    if(!ray.valid) continue;
    const double angle=scan->angle_min+i*scan->angle_increment;
    const Eigen::Vector3d delta=orientation*Eigen::Vector3d(
        ray.length*std::cos(angle),ray.length*std::sin(angle),0.);
    if(!delta.allFinite()) continue;
    const bool local=(delta.cwiseAbs().array()<=mp_.local_update_range_.array()).all();
    if(!local && ray.length<=mp_.max_ray_length_) continue;
    points.push_back(origin+delta);hits.push_back(ray.hit ? 1 : 0);
  }
  if(points.empty()) return;
  { std::unique_lock<std::shared_mutex> lock(map_mutex_);
    updateSlidingMap(origin);md_.ray_pos_=origin;md_.ray_q_=orientation;
    md_.ray_body_pos_=body_at_scan.position;md_.ray_body_q_=body_at_scan.orientation;
    md_.has_cloud_=true;md_.proj_points_.assign(points.begin(),points.end());
    md_.proj_hit_flags_=std::move(hits);md_.proj_points_cnt=static_cast<int>(points.size());
    md_.use_cloud_update_=true;md_.occ_need_update_=true;last_laser_stamp_ns_=stamp; }
  RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(),2000,
      "[LASER_RAY_EVIDENCE] stamp=%ld input=%zu accepted=%zu no_return_limit=%.2fm; one original scan, no XYZ re-fusion",
      static_cast<long>(stamp),scan->ranges.size(),points.size(),mp_.no_return_range_);
}

void GridMap::publishTerrainFacts(
    const std::vector<plan_env::TerrainEvidence>& facts,const rclcpp::Time& stamp)
{
  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id=mp_.frame_id_;message.header.stamp=stamp;
  sensor_msgs::PointCloud2Modifier modifier(message);
  modifier.setPointCloud2Fields(6,
      "x",1,sensor_msgs::msg::PointField::FLOAT32,
      "y",1,sensor_msgs::msg::PointField::FLOAT32,
      "z",1,sensor_msgs::msg::PointField::FLOAT32,
      "ground_height",1,sensor_msgs::msg::PointField::FLOAT32,
      "capture_body_z",1,sensor_msgs::msg::PointField::FLOAT32,
      "observation_scan",1,sensor_msgs::msg::PointField::UINT32);
  modifier.resize(facts.size());
  sensor_msgs::PointCloud2Iterator<float> x(message,"x"),y(message,"y"),z(message,"z");
  sensor_msgs::PointCloud2Iterator<float> h(message,"ground_height"),body(message,"capture_body_z");
  sensor_msgs::PointCloud2Iterator<uint32_t> scan(message,"observation_scan");
  for(const auto& fact:facts) {
    *x=fact.measured.x();*y=fact.measured.y();*z=fact.measured.z();
    *h=fact.ground_relative_height;*body=fact.capture_body_z;*scan=fact.scan;
    ++x;++y;++z;++h;++body;++scan;
  }
  terrain_evidence_pub_->publish(message);
}

void GridMap::publishLowSurfaceReturns(
    const std::vector<Eigen::Vector3d>& points,const rclcpp::Time& stamp)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.reserve(points.size());
  for (const auto& point:points)
    cloud.points.emplace_back(point.x(),point.y(),point.z());
  cloud.width=cloud.points.size();cloud.height=1;cloud.is_dense=true;
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud,message);
  message.header.frame_id=mp_.frame_id_;message.header.stamp=stamp;
  low_surface_pub_->publish(message); // publish empty frames too, clearing stale display
}

void GridMap::publishHighSurfaceReturns(
    const std::vector<Eigen::Vector3d>& points,const rclcpp::Time& stamp)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.reserve(points.size());
  for (const auto& point:points)
    cloud.points.emplace_back(point.x(),point.y(),point.z());
  cloud.width=cloud.points.size();cloud.height=1;cloud.is_dense=true;
  sensor_msgs::msg::PointCloud2 message;
  pcl::toROSMsg(cloud,message);
  message.header.frame_id=mp_.frame_id_;message.header.stamp=stamp;
  high_surface_pub_->publish(message); // publish empty frames too, clearing stale display
}

void GridMap::publishOccupancyCloud(
    const std::vector<char>& buffer, const InflatedOccupancySnapshot& snapshot,
    double maximum_z, const rclcpp::Time& stamp,
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr& publisher,
    double minimum_z)
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  const double resolution = 1.0 / snapshot.resolution_inv;
  Eigen::Vector3i maximum = snapshot.bound_max;
  Eigen::Vector3i minimum = snapshot.bound_min;
  if (std::isfinite(minimum_z))
    minimum(2) = std::max(minimum(2), static_cast<int>(
        std::ceil(minimum_z * snapshot.resolution_inv - 0.5)));
  if (std::isfinite(maximum_z))
    maximum(2) = std::min(maximum(2), static_cast<int>(
        std::floor(maximum_z * snapshot.resolution_inv - 0.5)));
  auto local_index = [](int index, int size) {
    const int local = index % size;
    return local < 0 ? local + size : local;
  };
  for (int x = snapshot.bound_min(0); x <= maximum(0); ++x)
  {
    const int x_address = local_index(x, snapshot.voxel_num(0)) *
        snapshot.voxel_num(1) * snapshot.voxel_num(2);
    for (int y = snapshot.bound_min(1); y <= maximum(1); ++y)
    {
      const int xy_address = x_address +
          local_index(y, snapshot.voxel_num(1)) * snapshot.voxel_num(2);
      for (int z = minimum(2); z <= maximum(2); ++z)
      {
        if (buffer[xy_address + local_index(z, snapshot.voxel_num(2))] == 0)
          continue;
        pt.x = (x + 0.5) * resolution;
        pt.y = (y + 0.5) * resolution;
        pt.z = (z + 0.5) * resolution;
        cloud.push_back(pt);
      }
    }
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = stamp;
  publisher->publish(cloud_msg);
  if (publisher == map_inf_pub_ || publisher == body_obstacle_pub_)
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[MAP_VISUALIZATION] inflated_points=%u bytes=%zu period=%.3fs source_revision=%lu display_min_z=%.3f collision_buffer=FULL",
        cloud_msg.width, cloud_msg.data.size(), local_patch_period_,
        static_cast<unsigned long>(snapshot.revision), minimum_z);
}

void GridMap::publishSlidingMapFrame()
{
  if (mp_.sliding_map_frame_id_.empty())
    return;

  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = node_->now();
  transform.header.frame_id = mp_.frame_id_;
  transform.child_frame_id = mp_.sliding_map_frame_id_;
  transform.transform.translation.x = md_.sliding_map_frame_pos_.x();
  transform.transform.translation.y = md_.sliding_map_frame_pos_.y();
  transform.transform.translation.z = md_.sliding_map_frame_pos_.z();
  transform.transform.rotation.w = 1.0;
  tf_broadcaster_->sendTransform(transform);
}

void GridMap::publishSlidingMapBBox()
{
  if (sliding_map_bbox_pub_->get_subscription_count() == 0)
    return;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = mp_.frame_id_;
  marker.header.stamp = node_->now();
  marker.ns = "sliding_map";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.04;
  marker.color.r = 0.0;
  marker.color.g = 0.8;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  const Eigen::Vector3d& min_pt = mp_.map_min_boundary_;
  const Eigen::Vector3d& max_pt = mp_.map_max_boundary_;
  Eigen::Vector3d corners[8] = {
      {min_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), min_pt.y(), min_pt.z()},
      {max_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), max_pt.y(), min_pt.z()},
      {min_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), min_pt.y(), max_pt.z()},
      {max_pt.x(), max_pt.y(), max_pt.z()},
      {min_pt.x(), max_pt.y(), max_pt.z()},
  };

  auto pushPoint = [&](const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x();
    point.y = p.y();
    point.z = p.z();
    marker.points.push_back(point);
  };

  const int edges[12][2] = {
      {0, 1}, {1, 2}, {2, 3}, {3, 0},
      {4, 5}, {5, 6}, {6, 7}, {7, 4},
      {0, 4}, {1, 5}, {2, 6}, {3, 7},
  };

  for (const auto& edge : edges)
  {
    pushPoint(corners[edge[0]]);
    pushPoint(corners[edge[1]]);
  }

  sliding_map_bbox_pub_->publish(marker);
}

void GridMap::publishUnknown()
{
  pcl::PointXYZ pt;
  pcl::PointCloud<pcl::PointXYZ> cloud;

  Eigen::Vector3i min_cut = md_.local_bound_min_;
  Eigen::Vector3i max_cut = md_.local_bound_max_;

  boundIndex(max_cut);
  boundIndex(min_cut);

  for (int x = min_cut(0); x <= max_cut(0); ++x)
    for (int y = min_cut(1); y <= max_cut(1); ++y)
      for (int z = min_cut(2); z <= max_cut(2); ++z)
      {

        if (md_.occupancy_buffer_[toAddress(x, y, z)] < mp_.clamp_min_log_ - 1e-3)
        {
          Eigen::Vector3d pos;
          indexToPos(Eigen::Vector3i(x, y, z), pos);
          if (md_.has_ray_pose_ && pos(2) > md_.ray_pos_(2) + mp_.vis_height_)
            continue;

          pt.x = pos(0);
          pt.y = pos(1);
          pt.z = pos(2);
          cloud.push_back(pt);
        }
      }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  unknown_pub_->publish(cloud_msg);
}

bool GridMap::odomValid() { return md_.has_ray_pose_; }

bool GridMap::hasDepthObservation() { return md_.has_first_depth_; }

Eigen::Vector3d GridMap::getOrigin() { return mp_.map_origin_; }

// int GridMap::getVoxelNum() {
//   return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1] * mp_.map_voxel_num_[2];
// }

void GridMap::getRegion(Eigen::Vector3d &ori, Eigen::Vector3d &size)
{
  ori = mp_.map_origin_, size = mp_.map_size_;
}

// GridMap

void GridMap::publishDepthCloud()
{
  if (depth_cloud_pub_->get_subscription_count() == 0)
    return;

  if (md_.proj_points_cnt == 0)
    return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ pt;

  for (int i = 0; i < md_.proj_points_cnt; ++i)
  {
    pt.x = md_.proj_points_[i](0);
    pt.y = md_.proj_points_[i](1);
    pt.z = md_.proj_points_[i](2);
    cloud.push_back(pt);
  }

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id_;

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = node_->now();
  depth_cloud_pub_->publish(cloud_msg);
}
