#pragma once
#include <cstdint>
#include <array>
#include <algorithm>
#include <cmath>

namespace scan_planner {
// Small installed facts only: no control points, map copy, or second owner.
struct RollingExecutionContract {
  uint8_t mode{0}; bool measured{false},fixed_yaw{false},known{false};
  double vx{0},vy{0},wz{0},correction{0},command_floor{0},effective_speed{0};
  double terminal_speed{0},deceleration{0},latency{0};
  template<class Message> static RollingExecutionContract from(const Message& m) {
    RollingExecutionContract c;c.known=true;c.mode=m.execution_mode;
    c.measured=m.use_measured_progress;c.fixed_yaw=!m.yaw_pts.empty();
    c.vx=m.max_forward_speed;c.vy=m.max_lateral_speed;c.wz=m.max_yaw_rate;
    c.correction=m.max_tracking_correction;c.command_floor=m.min_forward_command;
    c.effective_speed=m.min_forward_effective_speed;c.terminal_speed=m.terminal_speed_limit;
    c.deceleration=m.terminal_braking_deceleration;c.latency=m.terminal_command_latency;return c;
  }
  template<class Message> void apply(Message& m) const {
    m.execution_mode=mode;m.use_measured_progress=measured;
    m.max_forward_speed=vx;m.max_lateral_speed=vy;m.max_yaw_rate=wz;
    m.max_tracking_correction=correction;m.min_forward_command=command_floor;
    m.min_forward_effective_speed=effective_speed;
    // New endpoint restrictions remain authoritative. Carry an older limit
    // only if it is more conservative; live command continuity is checked
    // separately because the new endpoint may have moved.
    if(terminal_speed>0 && (m.terminal_speed_limit<=0 || terminal_speed<m.terminal_speed_limit))
      m.terminal_speed_limit=terminal_speed;
    if(m.terminal_speed_limit>0) {
      if(deceleration>0 && (m.terminal_braking_deceleration<=0 || deceleration<m.terminal_braking_deceleration))
        m.terminal_braking_deceleration=deceleration;
      m.terminal_command_latency=std::max(m.terminal_command_latency,latency);
    }
  }
  RollingExecutionContract resolved(double cap_vx,double cap_vy,double cap_wz) const {
    auto c=*this;c.vx=vx>0?std::min(vx,cap_vx):cap_vx;
    c.vy=vy>0?std::min(vy,cap_vy):cap_vy;c.wz=wz>0?std::min(wz,cap_wz):cap_wz;
    c.correction=correction>0?correction:std::max(c.vx,c.vy);return c;
  }
  bool ordinary() const {
    if(!known || fixed_yaw || !measured || (mode!=0 && mode!=1))return false;
    for(double n:std::array<double,9>{vx,vy,wz,correction,command_floor,effective_speed,
        terminal_speed,deceleration,latency})if(!std::isfinite(n)||n<0)return false;
    if(mode==1 && (command_floor<=0 || effective_speed<=0))return false;
    return terminal_speed<=0 || deceleration>0;
  }
};
inline const char* rollingContractMismatch(const RollingExecutionContract& a,
    const RollingExecutionContract& b) {
  if(!a.ordinary() || !b.ordinary())return "NOT_ORDINARY";
  if(a.mode!=b.mode)return "MODE";
  const std::array<double,6> x{a.vx,a.vy,a.wz,a.correction,a.command_floor,a.effective_speed};
  const std::array<double,6> y{b.vx,b.vy,b.wz,b.correction,b.command_floor,b.effective_speed};
  for(size_t i=0;i<x.size();++i)if(std::abs(x[i]-y[i])>1e-9)return "LIMITS_OR_GAIT";
  // Endpoint braking also depends on endpoint geometry. Its actual command
  // must be compared at the live entry, not certified by equal scalars alone.
  return nullptr;
}
} // namespace scan_planner
