/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#include "modules/planning/lattice/trajectory_generation/piecewise_braking_trajectory_generator.h"

#include <cmath>

#include "cyber/common/log.h"

namespace apollo {
namespace planning {

std::shared_ptr<Curve1d> PiecewiseBrakingTrajectoryGenerator::Generate(
    const double s_target, const double s_curr, const double v_target,
    const double v_curr, const double a_comfort, const double d_comfort,
    const double max_time) {
  std::shared_ptr<PiecewiseAccelerationTrajectory1d> ptr_trajectory =
      std::make_shared<PiecewiseAccelerationTrajectory1d>(s_curr, v_curr);//使用当前速度v,s生成一个制动轨迹

  double s_dist = s_target - s_curr;

  double comfort_stop_dist = ComputeStopDistance(v_curr, d_comfort);//以舒适减速度进行制动需要的距离

  // if cannot stop using comfort deceleration, then brake in the beginning.
  if (comfort_stop_dist > s_dist) { //制动距离不够吧 
    double stop_d = ComputeStopDeceleration(s_dist, v_curr);//如果想要速度减到0需要的加速度的大小 
    double stop_t = v_curr / stop_d;//需要的时间
    ptr_trajectory->AppendSegment(-stop_d, stop_t);

    if (ptr_trajectory->ParamLength() < max_time) {
      ptr_trajectory->AppendSegment(0.0,
                                    max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;
  }

  // otherwise, the vehicle can stop from current speed with comfort brake.
  if (v_curr > v_target) {
    double t_cruise = (s_dist - comfort_stop_dist) / v_target;//还可以保持巡航速度的时间
    double t_rampdown = (v_curr - v_target) / d_comfort;//需要减速到目标速度的时间
    double t_dec = v_target / d_comfort;//从目标速度减速到0的时间

    ptr_trajectory->AppendSegment(-d_comfort, t_rampdown);//先减速再巡航再减速
    ptr_trajectory->AppendSegment(0.0, t_cruise);
    ptr_trajectory->AppendSegment(-d_comfort, t_dec);//减速到停止

    if (ptr_trajectory->ParamLength() < max_time) {//添加一段速度为0的轨迹
      ptr_trajectory->AppendSegment(0.0,
                                    max_time - ptr_trajectory->ParamLength());
    }
    return ptr_trajectory;

  } else {//当前速度小于目标速度
    double t_rampup = (v_target - v_curr) / a_comfort;//加速到目标速度
    double t_rampdown = (v_target - v_curr) / d_comfort;//
    double s_ramp = (v_curr + v_target) * (t_rampup + t_rampdown) * 0.5;

    double s_rest = s_dist - s_ramp - comfort_stop_dist;
    if (s_rest > 0) {//还有剩余距离
      double t_cruise = s_rest / v_target;//还可以巡航的时间
      double t_dec = v_target / d_comfort;//刹车需要的时间

      // construct the trajectory
      ptr_trajectory->AppendSegment(a_comfort, t_rampup);//加速的轨迹
      ptr_trajectory->AppendSegment(0.0, t_cruise);//巡航的轨迹
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);//减速到0的轨迹

      if (ptr_trajectory->ParamLength() < max_time) {
        ptr_trajectory->AppendSegment(0.0,
                                      max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    } else {//没有剩余距离
      double s_rampup_rampdown = s_dist - comfort_stop_dist;
      double v_max = std::sqrt(v_curr * v_curr + 2.0 * a_comfort * d_comfort *
                                                     s_rampup_rampdown /
                                                     (a_comfort + d_comfort));

      double t_acc = (v_max - v_curr) / a_comfort;
      double t_dec = v_max / d_comfort;

      // construct the trajectory
      ptr_trajectory->AppendSegment(a_comfort, t_acc);
      ptr_trajectory->AppendSegment(-d_comfort, t_dec);

      if (ptr_trajectory->ParamLength() < max_time) {
        ptr_trajectory->AppendSegment(0.0,
                                      max_time - ptr_trajectory->ParamLength());
      }
      return ptr_trajectory;
    }
  }
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDistance(
    const double v, const double dec) {
  ACHECK(dec > 0.0);
  return v * v / dec * 0.5;
}

double PiecewiseBrakingTrajectoryGenerator::ComputeStopDeceleration(
    const double dist, const double v) {
  return v * v / dist * 0.5;
}

}  // namespace planning
}  // namespace apollo
