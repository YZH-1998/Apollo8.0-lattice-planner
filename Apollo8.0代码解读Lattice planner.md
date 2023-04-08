#### Apollo8.0代码解读Lattice planner

**代码位置：apollo/modules/planning/planner/lattice/lattic_planner.cc**

##### 主要函数：Plan（）

```c++
Status LatticePlanner::Plan(const TrajectoryPoint& planning_start_point,
                            Frame* frame,
                            ADCTrajectory* ptr_computed_trajectory) {
```

输入：车辆当前位置的轨迹点、当前帧规划所需要的信息

输出：计算出ADC（自动驾驶控制器）可以执行的轨迹 

**主要函数：PlanOnReferenceLine** 函数名的意思基于参考路径进行规划

```c++
Status LatticePlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_init_point, Frame* frame,
    ReferenceLineInfo* reference_line_info)
```

输入：规划开始时汽车的起点、当前帧所需要的信息、参考路径信息

主要分为以下7个步骤：

### **1.官方注释：obtain a reference line and transform it to the PathPoint format 要把参考路径点的数据格式ReferencePoint-->PathPoint转化形式**

```c++
auto ptr_reference_line =
      std::make_shared<std::vector<PathPoint>>(ToDiscretizedReferenceLine(
          reference_line_info->reference_line().reference_points()));
```

```c++
std::vector<PathPoint> ToDiscretizedReferenceLine(
    const std::vector<ReferencePoint>& ref_points) {
  double s = 0.0;
  std::vector<PathPoint> path_points;
  for (const auto& ref_point : ref_points) {
    PathPoint path_point;
    path_point.set_x(ref_point.x());
    path_point.set_y(ref_point.y());
    path_point.set_theta(ref_point.heading());
    path_point.set_kappa(ref_point.kappa());
    path_point.set_dkappa(ref_point.dkappa());

    if (!path_points.empty()) {
      double dx = path_point.x() - path_points.back().x();
      double dy = path_point.y() - path_points.back().y();
      s += std::sqrt(dx * dx + dy * dy);//求解s坐标
    }
    path_point.set_s(s);
    path_points.push_back(std::move(path_point));//move以后 path_point的状态未知，可以避免复制；
  }
  return path_points;
}
```

原本路径数据的x,y,theta,kappa,dkappa等数据不变化直接赋值就好；函数下半部分经典的求取s坐标，当然采用的也是近似的形式，汽车比较大，只要路径点之间距离小一些完全可以接受。

##### **总结：1.参考路径点数据格式转换ReferencePoint-->PathPoint  2.增加了s坐标**

### **2.官方注释：compute the matched point of the init planning point on the reference  根据规划的起点计算相应的匹配点**

这里不只是匹配点的计算，计算完匹配点后还将前一个点后一个点送入寻找投影点的函数中，并利用线性插值的方法来近似投影点的各种信息。

```c++
  PathPoint matched_point = PathMatcher::MatchToPath(//寻找匹配点
      *ptr_reference_line, planning_init_point.path_point().x(),//输入：参考路径、规划起点x坐标、y坐标
      planning_init_point.path_point().y()); 
```

```c++
PathPoint PathMatcher::MatchToPath(const std::vector<PathPoint>& reference_line,
                                   const double x, const double y) {
  CHECK_GT(reference_line.size(), 0U);
  auto func_distance_square = [](const PathPoint& point, const double x,//匿名函数求到路径点距离的平方
                                 const double y) {
    double dx = point.x() - x;
    double dy = point.y() - y;
    return dx * dx + dy * dy;
  };

  double distance_min = func_distance_square(reference_line.front(), x, y);
  std::size_t index_min = 0; 

  for (std::size_t i = 1; i < reference_line.size(); ++i) {//求出路径上最近的索引点
    double distance_temp = func_distance_square(reference_line[i], x, y);
    if (distance_temp < distance_min) {
      distance_min = distance_temp;
      index_min = i;
    }
  }
  //选取匹配点前后的各1个点 
  std::size_t index_start = (index_min == 0) ? index_min : index_min - 1;//不是第一个点 减1
  std::size_t index_end =
      (index_min + 1 == reference_line.size()) ? index_min : index_min + 1;//不是最后一个点 加1

  if (index_start == index_end) {
    return reference_line[index_start];
  }

  return FindProjectionPoint(reference_line[index_start],//找到了匹配点后 再寻找投影点 
                             reference_line[index_end], x, y);
}
```

这里显然认为把匹配点当作投影点的精度是不够的，不然后面就省了大量的计算。接下来是计算投影点：

```c++
PathPoint PathMatcher::FindProjectionPoint(const PathPoint& p0,
                                           const PathPoint& p1, const double x,
                                           const double y) {
  double v0x = x - p0.x();
  double v0y = y - p0.y();

  double v1x = p1.x() - p0.x();
  double v1y = p1.y() - p0.y();

  double v1_norm = std::sqrt(v1x * v1x + v1y * v1y);//投影直线的模
  double dot = v0x * v1x + v0y * v1y;//（v0x,v0y)*(v1x, v1y)向量之间的点乘 除以模做投影

  double delta_s = dot / v1_norm;
  return InterpolateUsingLinearApproximation(p0, p1, p0.s() + delta_s); //求出投影点的s坐标
}
```

这里投影点s坐标计算显然也是近似的，把路径当作一条直线，因为s坐标的计算就是近似的，所以这里也是如此。这里的计算比较好理解，规划起点的位置A，匹配点后一个点B，前一个点是C，把向量BA投影到BC上即可，大小就是delta_s。接下来就是利用线性插值来估算投影点的其他信息。

```c++
PathPoint InterpolateUsingLinearApproximation(const PathPoint &p0,
                                              const PathPoint &p1,
                                              const double s) {
  double s0 = p0.s();
  double s1 = p1.s();
  
  //x = x0 + weight*(x1- x0) = (1 - weight)*x0 + weight*x1;
  PathPoint path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x() + weight * p1.x();
  double y = (1 - weight) * p0.y() + weight * p1.y();
  double theta = slerp(p0.theta(), p0.s(), p1.theta(), p1.s(), s);
  double kappa = (1 - weight) * p0.kappa() + weight * p1.kappa();
  double dkappa = (1 - weight) * p0.dkappa() + weight * p1.dkappa();
  double ddkappa = (1 - weight) * p0.ddkappa() + weight * p1.ddkappa();
  path_point.set_x(x);
  path_point.set_y(y);
  path_point.set_theta(theta);
  path_point.set_kappa(kappa);
  path_point.set_dkappa(dkappa);
  path_point.set_ddkappa(ddkappa);
  path_point.set_s(s);
  return path_point;
}
```

计算出投影点的其他信息：x, y, theta, kappa, dkappa, ddkappa, s；

##### **总结：根据离散好的路径和规划的起点，利用向量的投影以及线性插值等方法计算出投影点的 s坐标以及各种信息；**

### **3.官方注释： according to the matched point, compute the init state in Frenet frame 根据匹配点计算出在frenet坐标系下的规划起点初始状态**

##### 计算得到l l' l'' s s. s..用  std::array<double, 3> init_s, init_d来存储。根据投影点和规划起点的信息来计算。计算好这个以后，后面就要开始进行规划了。具体的推到过程可以上b站看老王讲的，或者：https://blog.csdn.net/u013468614/article/details/108748016有推导过程

```c++
ComputeInitFrenetState(matched_point, planning_init_point, &init_s, &init_d);//就调用下面这一个函数
  CartesianFrenetConverter::cartesian_to_frenet(
      matched_point.s(), matched_point.x(), matched_point.y(),
      matched_point.theta(), matched_point.kappa(), matched_point.dkappa(),
      cartesian_state.path_point().x(), cartesian_state.path_point().y(),//规划起点 x,y,v,a,theta, kappa
      cartesian_state.v(), cartesian_state.a(),
      cartesian_state.path_point().theta(),
      cartesian_state.path_point().kappa(), ptr_s, ptr_d);//输出的量s, d
}
```

```c++
void CartesianFrenetConverter::cartesian_to_frenet(//根据车辆当前点和投影点的信息 求取车辆的frenet坐标
    const double rs, const double rx, const double ry, const double rtheta,
    const double rkappa, const double rdkappa, const double x, const double y,
    const double v, const double a, const double theta, const double kappa,
    std::array<double, 3>* const ptr_s_condition,
    std::array<double, 3>* const ptr_d_condition) {
  const double dx = x - rx;
  const double dy = y - ry;

  const double cos_theta_r = std::cos(rtheta);
  const double sin_theta_r = std::sin(rtheta);

  const double cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx;// (-sin, cos) * (dx, dy)
  ptr_d_condition->at(0) =
      std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);//1 l
      //copysign(x,y) 返回大小为x，符号为y的值
  const double delta_theta = theta - rtheta;//偏航角误差
  const double tan_delta_theta = std::tan(delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);

  const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);//1-kl
  ptr_d_condition->at(1) = one_minus_kappa_r_d * tan_delta_theta;//2 l'

  const double kappa_r_d_prime =
      rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

  ptr_d_condition->at(2) =
      -kappa_r_d_prime * tan_delta_theta +
      one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
          (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);//3 l''

  ptr_s_condition->at(0) = rs;//1 s 

  ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;//2 s. = v*cos(delta)/(1-kl)

  const double delta_theta_prime =
      one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
  ptr_s_condition->at(2) =//3 s..
      (a * cos_delta_theta -
       ptr_s_condition->at(1) * ptr_s_condition->at(1) *
           (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
      one_minus_kappa_r_d;
}
```

后面还有一个函数：

```c++
  auto ptr_prediction_querier = std::make_shared<PredictionQuerier>(//保存了当前侦所看到的所有障碍物
      frame->obstacles(), ptr_reference_line);
```

```c++
PredictionQuerier::PredictionQuerier(
    const std::vector<const Obstacle*>& obstacles,
    const std::shared_ptr<std::vector<common::PathPoint>>& ptr_reference_line)
    : ptr_reference_line_(ptr_reference_line) {
  for (const auto ptr_obstacle : obstacles) {
    if (common::util::InsertIfNotPresent(&id_obstacle_map_, ptr_obstacle->Id(),
                                         ptr_obstacle)) {//发现新的障碍物即插入成功到id_obstacle_map_ 
      obstacles_.push_back(ptr_obstacle);//添加到obstacles
    } else {
      AWARN << "Duplicated obstacle found [" << ptr_obstacle->Id() << "]";
    }
  }
}
```

主要作用就是将当前这一帧的障碍物更新，是看否有新的障碍物，如果有重复的也会显示出其ID号；如果有新的，则添加进来：PredictionQuerier这个类里面有有几个私有变量用来存储障碍物的信息和上面的函数写入对应；

```c++
 private:
  std::unordered_map<std::string, const Obstacle*> id_obstacle_map_;

  std::vector<const Obstacle*> obstacles_;

  std::shared_ptr<std::vector<common::PathPoint>> ptr_reference_line_;
```

#### 总结：1.计算出规划起点状态的frenet坐标 2.障碍物的更新

### 4.官方注释：parse the decision and get the planning target 解析决策并且得到规划的目标

### 4.1 创建ST图PathTimeGraph:

```c++
  auto ptr_path_time_graph = std::make_shared<PathTimeGraph>(
      ptr_prediction_querier->GetObstacles(), *ptr_reference_line,
      reference_line_info, init_s[0],
      init_s[0] + FLAGS_speed_lon_decision_horizon, 0.0,
      FLAGS_trajectory_time_length, init_d);
```

输入：预测队列中的障碍物、参考线、参考线信息、第三步得到的规划起点s坐标、规划终点s坐标、规划起点时间0、规划终点时间FLAGS_trajectory_time_length、规划起点的横向坐标l l' l''

```c++
PathTimeGraph::PathTimeGraph(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points,
    const ReferenceLineInfo* ptr_reference_line_info, const double s_start,
    const double s_end, const double t_start, const double t_end,
    const std::array<double, 3>& init_d) {
  CHECK_LT(s_start, s_end);
  CHECK_LT(t_start, t_end);
  path_range_.first = s_start;
  path_range_.second = s_end;
  time_range_.first = t_start;
  time_range_.second = t_end;
  ptr_reference_line_info_ = ptr_reference_line_info;
  init_d_ = init_d;

  SetupObstacles(obstacles, discretized_ref_points);
}
```

除了检查变量CHECK_LT、变量赋值、主要的函数就是设置障碍物的信息：SetupObstacles(obstacles, discretized_ref_points);

```c++
void PathTimeGraph::SetupObstacles(
    const std::vector<const Obstacle*>& obstacles,
    const std::vector<PathPoint>& discretized_ref_points) {
  for (const Obstacle* obstacle : obstacles) {//遍历每一个障碍物
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->HasTrajectory()) {
      SetStaticObstacle(obstacle, discretized_ref_points);//处理静态障碍物
    } else {
      SetDynamicObstacle(obstacle, discretized_ref_points);//处理动态障碍物
    }
  }

  std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
            [](const SLBoundary& sl0, const SLBoundary& sl1) {//对于静态障碍物排序 排序的规则是引用了匿名函数
              return sl0.start_s() < sl1.start_s();  //按照s坐标从小到大排序
            });

  for (auto& path_time_obstacle : path_time_obstacle_map_) {
    path_time_obstacles_.push_back(path_time_obstacle.second);
  }
}
```

首先是设置障碍物函数信息：忽略掉虚拟障碍物，如果障碍物有轨迹->设置动态障碍物，否测设置为静态障碍物；

### 4.1.1先看如何处理静态障碍物：

```c++
void PathTimeGraph::SetStaticObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  const Polygon2d& polygon = obstacle->PerceptionPolygon();

  std::string obstacle_id = obstacle->Id();
  SLBoundary sl_boundary =
      ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points);

  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;
  ptr_reference_line_info_->reference_line().GetLaneWidth(
      sl_boundary.start_s(), &left_width, &right_width);
  if (sl_boundary.start_s() > path_range_.second || //判断障碍物是否超过了范围
      sl_boundary.end_s() < path_range_.first ||
      sl_boundary.start_l() > left_width ||
      sl_boundary.end_l() < -right_width) {
    ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.";
    return;
  }

  path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);//画ST图（s,t)坐标 四个点 还有障碍物的编号
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.start_s(), FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(
      SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(
      obstacle_id, sl_boundary.end_s(), FLAGS_trajectory_time_length));
  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
  ADEBUG << "ST-Graph mapping static obstacle: " << obstacle_id
         << ", start_s : " << sl_boundary.start_s()
         << ", end_s : " << sl_boundary.end_s()
         << ", start_l : " << sl_boundary.start_l()
         << ", end_l : " << sl_boundary.end_l();
}
```

第一个函数obstacle->PerceptionPolygon()障碍物感知多边形具体没怎么看，怎么处理的并不关心，只需要知道处理完成以后成多边形，并且可以通过GetAllVertices(),函数获得其顶点；

第二个函数ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points)：

输入：障碍物处理成为多边形的各个顶点、离散化后的参考线

```c++
SLBoundary PathTimeGraph::ComputeObstacleBoundary(
    const std::vector<common::math::Vec2d>& vertices,
    const std::vector<PathPoint>& discretized_ref_points) const {
  double start_s(std::numeric_limits<double>::max());
  double end_s(std::numeric_limits<double>::lowest());
  double start_l(std::numeric_limits<double>::max());
  double end_l(std::numeric_limits<double>::lowest());

  for (const auto& point : vertices) {//遍历所有的顶点
    auto sl_point = PathMatcher::GetPathFrenetCoordinate(discretized_ref_points,
                                                         point.x(), point.y());
    start_s = std::fmin(start_s, sl_point.first);
    end_s = std::fmax(end_s, sl_point.first);
    start_l = std::fmin(start_l, sl_point.second);
    end_l = std::fmax(end_l, sl_point.second);
  }

  SLBoundary sl_boundary;
  sl_boundary.set_start_s(start_s);
  sl_boundary.set_end_s(end_s);
  sl_boundary.set_start_l(start_l);
  sl_boundary.set_end_l(end_l);

  return sl_boundary;
}
```

这个函数PathMatcher::GetPathFrenetCoordinate(discretized_ref_points, point.x(), point.y());就是遍历静态障碍物的所有顶点，将其坐标point.x(), point.y()转化为frenet坐标系下，并设置SL图下障碍物的四个顶点，效果图如下，借用知乎大佬。但好像程序并没有画...画的是ST图，画到的是path_time_obstacle_map_:

<img src="/home/yzh/Pictures/Screenshot from 2023-03-09 10-34-26.png" alt="Screenshot from 2023-03-09 10-34-26" style="zoom:67%;" /><img src="/home/yzh/Pictures/Screenshot from 2023-03-09 11-00-50.png" alt="Screenshot from 2023-03-09 11-00-50" style="zoom: 50%;" />

### 4.1.2接下来是动态障碍物的处理

和处理静态障碍物最主要的区别是在while()这个循环里面，要将规划时间区域的每一个离散时刻FLAGS_trajectory_time_resolution动态障碍物给表示出来，那么问题来了，预测模块给出的障碍物的轨迹点，上面的时间戳可能并不能和规划模块的时间戳是正好重合的，那么怎么办呢？GetPointAtTime(relative_time);给出答案，还是使用线性插值的方法,使其与规划模块的时间对应上。

处理动态障碍物的函数：

```c++
void PathTimeGraph::SetDynamicObstacle(
    const Obstacle* obstacle,
    const std::vector<PathPoint>& discretized_ref_points) {
  double relative_time = time_range_.first;
  while (relative_time < time_range_.second) {
    TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);//时间对应，进行插点
    Box2d box = obstacle->GetBoundingBox(point);
    SLBoundary sl_boundary =
        ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);

    double left_width = FLAGS_default_reference_line_width * 0.5;
    double right_width = FLAGS_default_reference_line_width * 0.5;
    ptr_reference_line_info_->reference_line().GetLaneWidth(
        sl_boundary.start_s(), &left_width, &right_width);

    // The obstacle is not shown on the region to be considered.
    if (sl_boundary.start_s() > path_range_.second ||
        sl_boundary.end_s() < path_range_.first ||
        sl_boundary.start_l() > left_width ||
        sl_boundary.end_l() < -right_width) {
      if (path_time_obstacle_map_.find(obstacle->Id()) !=
          path_time_obstacle_map_.end()) {
        break;
      }
      relative_time += FLAGS_trajectory_time_resolution;
      continue;
    }
```

GetPointAtTime(relative_time)函数：

```c++
common::TrajectoryPoint Obstacle::GetPointAtTime(
    const double relative_time) const {
  const auto& points = trajectory_.trajectory_point();
  if (points.size() < 2) {
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(perception_obstacle_.position().x());
    point.mutable_path_point()->set_y(perception_obstacle_.position().y());
    point.mutable_path_point()->set_z(perception_obstacle_.position().z());
    point.mutable_path_point()->set_theta(perception_obstacle_.theta());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.mutable_path_point()->set_dkappa(0.0);
    point.mutable_path_point()->set_ddkappa(0.0);
    point.set_v(0.0);
    point.set_a(0.0);
    point.set_relative_time(0.0);
    return point;
  } else {
    auto comp = [](const common::TrajectoryPoint p, const double time) {
      return p.relative_time() < time;
    };
    
    auto it_lower =
        std::lower_bound(points.begin(), points.end(), relative_time, comp);
    
    if (it_lower == points.begin()) {
      return *points.begin();
    } else if (it_lower == points.end()) {
      return *points.rbegin();
    }
    return common::math::InterpolateUsingLinearApproximation(
        *(it_lower - 1), *it_lower, relative_time);
  }
}
```

这里我理解的是lower_bound函数找到时间距离规划模块给出的时间最近的轨迹点，这个轨迹点的时间是大于等于规划模块的时间的，然后线性插值的时候，再找到前一个点lower - 1,然后两个点就可以调用common::math::InterpolateUsingLinearApproximation( *(it_lower - 1), *it_lower, relative_time)进行插值得到规划模块想要的时间的轨迹点的各个信息如x,y,theta等信息。接下来就可以根据测量的动态障碍物的长度、宽度求出box箱子的各个点。

#### 下一个函数比较简单：返回这个点的x,y,theta,还有障碍物的长度和宽度等信息；

```c++
obstacle->GetBoundingBox(point);
common::math::Box2d Obstacle::GetBoundingBox(
    const common::TrajectoryPoint& point) const {
  return common::math::Box2d({point.path_point().x(), point.path_point().y()},
                             point.path_point().theta(),
                             perception_obstacle_.length(),
                             perception_obstacle_.width());
}
```

接下来的操作肯定就是求出这个动态障碍物长方体的四个顶点box.GetAllCorners()，然后投影到frenet坐标系下 ComputeObstacleBoundary，求出SL图上障碍物的四个点；

```c++
    SLBoundary sl_boundary =
        ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points);
```

GetAllCorners()：第一个计算是根据路径点、长度、宽度、角度 求出长方体的四个顶点，画图看一下比较好计算；

ComputeObstacleBoundary(box.GetAllCorners(), discretized_ref_points)：里面主要投影的计算，前面介绍过了。

然后画的ST图,也是将障碍物的四个点画到path_time_obstacle_map_上面。

### 4.1.3 其余操作

```C++
  std::sort(static_obs_sl_boundaries_.begin(), static_obs_sl_boundaries_.end(),
            [](const SLBoundary& sl0, const SLBoundary& sl1) {
              return sl0.start_s() < sl1.start_s();
            });

  for (auto& path_time_obstacle : path_time_obstacle_map_) {
    path_time_obstacles_.push_back(path_time_obstacle.second);
  }
```

1.首先在static_obs_sl_boundaries_按照静态障碍物的起始位置远近按照从近到远进行排序；

2.把之前存储在path_time_obstacle_map_ 里的动态障碍物和静态障碍物的障碍物存储到path_time_obstacles_里面；

### 5.官方注释：generate 1d trajectory bundle for longitudinal and lateral respectively 分别产生纵向和横向轨迹簇

```C++
  Trajectory1dGenerator trajectory1d_generator(
      init_s, init_d, ptr_path_time_graph, ptr_prediction_querier);
  std::vector<std::shared_ptr<Curve1d>> lon_trajectory1d_bundle;
  std::vector<std::shared_ptr<Curve1d>> lat_trajectory1d_bundle;
  trajectory1d_generator.GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle);
```



#### 第一个操作是创建这个类Trajectory1dGenerator，里面是赋值传入的参数；主要的函数是：GenerateTrajectoryBundles(
      planning_target, &lon_trajectory1d_bundle, &lat_trajectory1d_bundle)

#### 输入变量：规划的目标

#### 输出：纵向一维轨迹簇、横向一维轨迹簇

具体这个函数是分为两个部分，纵向和横向分开计算：

```C++
void Trajectory1dGenerator::GenerateTrajectoryBundles(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle,
    Trajectory1DBundle* ptr_lat_trajectory_bundle) {
  GenerateLongitudinalTrajectoryBundle(planning_target,
                                       ptr_lon_trajectory_bundle);

  GenerateLateralTrajectoryBundle(ptr_lat_trajectory_bundle);
}
```

### 5.1 产生纵向轨迹簇 GenerateLongitudinalTrajectoryBundle

```C++
void Trajectory1dGenerator::GenerateLongitudinalTrajectoryBundle(
    const PlanningTarget& planning_target,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  // cruising trajectories are planned regardlessly.
  GenerateSpeedProfilesForCruising(planning_target.cruise_speed(),//巡航
                                   ptr_lon_trajectory_bundle);

  GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);//超车或跟车

  if (planning_target.has_stop_point()) { //停车
    GenerateSpeedProfilesForStopping(planning_target.stop_point().s(),
                                     ptr_lon_trajectory_bundle);
  }
}
```

里面也分成了三个部分，分别是巡航速度规划、在有障碍物的情况下跟车或超车的速度规划、停车的速度规划，其中速度巡航规划采用的多项式是4次，因为终点的S坐标并不确定，只有终点的速度信息；而后面两种情况因为跟车超车或停车都是有目标的终点的具有s,v两个信息，采用的是5次多项式来产生纵向的轨迹。

### 5.1.1巡航速度规划GenerateSpeedProfilesForCruising

```c++
void Trajectory1dGenerator::GenerateSpeedProfilesForCruising(
    const double target_speed,
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  ADEBUG << "cruise speed is  " << target_speed;
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForCruising(target_speed);
  if (end_conditions.empty()) {
    return;
  }

  // For the cruising case, We use the "QuarticPolynomialCurve1d" class (not the
  // "QuinticPolynomialCurve1d" class) to generate curves. Therefore, we can't
  // invoke the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<4>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}
```

重点就是这个采样终点情况的函数：

```C++
std::vector<Condition> EndConditionSampler::SampleLonEndConditionsForCruising(
    const double ref_cruise_speed) const {
  CHECK_GT(FLAGS_num_velocity_sample, 1U);

  // time interval is one second plus the last one 0.01
  static constexpr size_t num_of_time_samples = 9;
  std::array<double, num_of_time_samples> time_samples;
  for (size_t i = 1; i < num_of_time_samples; ++i) {
    auto ratio =
        static_cast<double>(i) / static_cast<double>(num_of_time_samples - 1);
    time_samples[i] = FLAGS_trajectory_time_length * ratio;
  }
  time_samples[0] = FLAGS_polynomial_minimal_param;

  std::vector<Condition> end_s_conditions;
  for (const auto& time : time_samples) {//每个采样时刻点
    double v_upper = std::min(feasible_region_.VUpper(time), ref_cruise_speed);//以最大加速度加速所能达到的最大速度
    double v_lower = feasible_region_.VLower(time);//以最大的减速度刹车所能达到的速度
    State lower_end_s = {0.0, v_lower, 0.0};
    end_s_conditions.emplace_back(lower_end_s, time);//最小速度采样

    State upper_end_s = {0.0, v_upper, 0.0};
    end_s_conditions.emplace_back(upper_end_s, time);//最大速度采样

    double v_range = v_upper - v_lower;
    // Number of sample velocities
    size_t num_of_mid_points =
        std::min(static_cast<size_t>(FLAGS_num_velocity_sample - 2),
                 static_cast<size_t>(v_range / FLAGS_min_velocity_sample_gap));

    if (num_of_mid_points > 0) {
      double velocity_seg =
          v_range / static_cast<double>(num_of_mid_points + 1);
      for (size_t i = 1; i <= num_of_mid_points; ++i) {
        State end_s = {0.0, v_lower + velocity_seg * static_cast<double>(i),
                       0.0};//中间速度采样
        end_s_conditions.emplace_back(end_s, time);
      }
    }
  }
  return end_s_conditions;
}
```

比较重要的两个函数是VUpper和VLower这两个：

```C++
double FeasibleRegion::VUpper(const double t) const {
  return init_s_[1] + FLAGS_longitudinal_acceleration_upper_bound * t;
}

double FeasibleRegion::VLower(const double t) const {
  return t < t_at_zero_speed_//刹车时间
             ? init_s_[1] + FLAGS_longitudinal_acceleration_lower_bound * t//v - at
             : 0.0;
}
```

最大的速度就是V = V0 + a*t 这个加速度就是一个常数，纵向设置的最大的加速度；

最小的速度就是进行刹车操作，如果大于刹车时间，那么速度是0，不然就实际计算；

这样就有了最大速度的状态和最小速度的状态，然后进行速度的采样，介于最小最大的速度之间；结束位置的状态是一个三维变量{s,v,a}，在巡航速度规划中，终端的位置s坐标不知道，可以知道的是速度、以及加速度为0;采样完成之后，计算的是4次多项式的各个参数，并没有真实的生成轨迹；

```C++
inline void Trajectory1dGenerator::GenerateTrajectory1DBundle<4>(
    const std::array<double, 3>& init_state,
    const std::vector<std::pair<std::array<double, 3>, double>>& end_conditions,//vector<conditions>
    //conditions<array<double, 3>, double> (state,t)
    std::vector<std::shared_ptr<Curve1d>>* ptr_trajectory_bundle) const {
  CHECK_NOTNULL(ptr_trajectory_bundle);
  ACHECK(!end_conditions.empty());
  
  ptr_trajectory_bundle->reserve(ptr_trajectory_bundle->size() +
                                 end_conditions.size());
  for (const auto& end_condition : end_conditions) {
    auto ptr_trajectory1d = std::make_shared<LatticeTrajectory1d>(
        std::shared_ptr<Curve1d>(new QuarticPolynomialCurve1d(
            init_state, {end_condition.first[1], end_condition.first[2]},//s s. s..  v 0
            end_condition.second)));//t
    //纵向采样了好多条 每一条记录下目标终点的速度、时间
    ptr_trajectory1d->set_target_velocity(end_condition.first[1]);
    ptr_trajectory1d->set_target_time(end_condition.second);
    ptr_trajectory_bundle->push_back(ptr_trajectory1d);
  }
}
```

4次多项式，5个参数，传入的5个已知量是起点状态s s. s.. ，终点状态s. s..问题得到解决；接下来是障碍物的跟车和超车：

### 5.1.2  GenerateSpeedProfilesForPathTimeObstacles(ptr_lon_trajectory_bundle);

#### 超车或跟车

```c++
void Trajectory1dGenerator::GenerateSpeedProfilesForPathTimeObstacles(
    Trajectory1DBundle* ptr_lon_trajectory_bundle) const {
  auto end_conditions =
      end_condition_sampler_.SampleLonEndConditionsForPathTimePoints();
  if (end_conditions.empty()) {
    return;
  }

  // Use the common function to generate trajectory bundles.
  GenerateTrajectory1DBundle<5>(init_lon_state_, end_conditions,
                                ptr_lon_trajectory_bundle);
}
```

这个采样的函数就比较复杂了,进去以后是获取采样点这个函数，里面又可以分为跟车采样轨迹和超车采样轨迹，首先介绍跟车采样轨迹：

#### 5.1.2.1跟车采样轨迹

```c++
void EndConditionSampler::QueryFollowPathTimePoints(
    const common::VehicleConfig& vehicle_config, const std::string& obstacle_id,
    std::vector<SamplePoint>* const sample_points) const {
  std::vector<STPoint> follow_path_time_points =
      ptr_path_time_graph_->GetObstacleSurroundingPoints(
          obstacle_id, -FLAGS_numerical_epsilon, FLAGS_time_min_density);

  for (const auto& path_time_point : follow_path_time_points) {
    double v = ptr_prediction_querier_->ProjectVelocityAlongReferenceLine(//障碍物在frenet坐标下的速度
        obstacle_id, path_time_point.s(), path_time_point.t());
    // Generate candidate s
    double s_upper = path_time_point.s() -
                     vehicle_config.vehicle_param().front_edge_to_center();
    double s_lower = s_upper - FLAGS_default_lon_buffer;
    CHECK_GE(FLAGS_num_sample_follow_per_timestamp, 2U);
    double s_gap =
        FLAGS_default_lon_buffer /
        static_cast<double>(FLAGS_num_sample_follow_per_timestamp - 1);
    for (size_t i = 0; i < FLAGS_num_sample_follow_per_timestamp; ++i) {
      double s = s_lower + s_gap * static_cast<double>(i);
      SamplePoint sample_point;
      sample_point.path_time_point = path_time_point;
      sample_point.path_time_point.set_s(s);
      sample_point.ref_v = v;
      sample_points->push_back(std::move(sample_point));
    }
  }
}
```

这个函数又分为两个部分：1首先是获取ST图上障碍物的周围点 2把障碍物的速度投影到参考路径即frenet坐标下 这个速度就是我们最后要设置结束状态的速度；

```C++
GetObstacleSurroundingPoints(
    const std::string& obstacle_id, const double s_dist, //s_dist t_min_density都是常数
    const double t_min_density)
```

这个函数根据传入参数s_dist的不同划分为跟车或者超车，如果是跟车的话获取的就是ST上障碍物的下面那条边的左边点和右边点，不能超过这个障碍物，然后按照时间间隔进行采样，对s进行插值；

```C++
double PredictionQuerier::ProjectVelocityAlongReferenceLine(
    const std::string& obstacle_id, const double s, const double t) const {
  ACHECK(id_obstacle_map_.find(obstacle_id) != id_obstacle_map_.end());

  const auto& trajectory = id_obstacle_map_.at(obstacle_id)->Trajectory();
  int num_traj_point = trajectory.trajectory_point_size();
  if (num_traj_point < 2) {
    return 0.0;//轨迹点只有一个 认为是静态障碍物 v是0
  }

  if (t < trajectory.trajectory_point(0).relative_time() ||
      t > trajectory.trajectory_point(num_traj_point - 1).relative_time()) {
    return 0.0;
  }

  auto matched_it =//轨迹点
      std::lower_bound(trajectory.trajectory_point().begin(),
                       trajectory.trajectory_point().end(), t,
                       [](const common::TrajectoryPoint& p, const double t) {
                         return p.relative_time() < t;
                       });//找到距离t时间最近的轨迹点 大于等于t时刻的点

  double v = matched_it->v();
  double theta = matched_it->path_point().theta();
  double v_x = v * std::cos(theta);
  double v_y = v * std::sin(theta);

  common::PathPoint obstacle_point_on_ref_line = 
      common::math::PathMatcher::MatchToPath(*ptr_reference_line_, s);//把跟随点s投入frenet坐标系
  auto ref_theta = obstacle_point_on_ref_line.theta();

  return std::cos(ref_theta) * v_x + std::sin(ref_theta) * v_y;// 
}
```

思路是找到预测模块给出障碍物的轨迹上当前时刻最近的点，也就是t时间时障碍物应该处在的位置，得到速度以及方向角，再将障碍物的s坐标将其线性插值路径上，得到对应的方向角，最后返回的结果就是障碍物的速度向量投影到frenet坐标系下的方向向量上。也就是说只关注障碍物在s方向上的速度。然后再按照s坐标按照一定的间隔进行采样，这样跟车采样就完成了。

### 5.1.2.2 超车采样

和跟车采样区别不大，只不过障碍物的周围点选取变成了ST图中障碍物的上边界的两个点，然后再进行采样，因为你要超车嘛，然后再加上往前的一段距离就可以。然后将采样终点的状态返回。将起点状态s s. s.. 终点s s.  s..6个参数带入5次多项式，求出相应的系数。

### 5.1.3停车采样

```C++
  if (planning_target.has_stop_point()) { //停车
    GenerateSpeedProfilesForStopping(planning_target.stop_point().s(),
                                     ptr_lon_trajectory_bundle);
  }
```

如果有停车点，按照纵向规划时间间隔进行采样，终端状态是（s_target, 0, 0),然后也是带入5次多项式进行求解系数。

## 5.2 横向轨迹采样

```C++
void Trajectory1dGenerator::GenerateLateralTrajectoryBundle(
    Trajectory1DBundle* ptr_lat_trajectory_bundle) const {
  if (!FLAGS_lateral_optimization) {//不使用优化的方法
    auto end_conditions = end_condition_sampler_.SampleLatEndConditions();
    // Use the common function to generate trajectory bundles.
    GenerateTrajectory1DBundle<5>(init_lat_state_, end_conditions,
                                  ptr_lat_trajectory_bundle);
  } else {//使用优化的方法
    double s_min = init_lon_state_[0]; //s
    double s_max = s_min + FLAGS_max_s_lateral_optimization;//优化方法有个设置好的纵向距离

    double delta_s = FLAGS_default_delta_s_lateral_optimization;

    auto lateral_bounds =
        ptr_path_time_graph_->GetLateralBounds(s_min, s_max, delta_s);//求横向坐标的约束

    // LateralTrajectoryOptimizer lateral_optimizer;
    std::unique_ptr<LateralQPOptimizer> lateral_optimizer(
        new LateralOSQPOptimizer);
    lateral_optimizer->optimize(init_lat_state_, delta_s, lateral_bounds);
    auto lateral_trajectory = lateral_optimizer->GetOptimalTrajectory();
    ptr_lat_trajectory_bundle->push_back(
        std::make_shared<PiecewiseJerkTrajectory1d>(lateral_trajectory));
  }
}
```

#### 主要就是分为两种，第一种是不使用优化的方法；第二种是使用OSQP进行二次规划的求解；

#### 第一种：不使用优化的方法：

```C++
std::vector<Condition> EndConditionSampler::SampleLatEndConditions() const {
  std::vector<Condition> end_d_conditions;
  std::array<double, 3> end_d_candidates = {0.0, -0.5, 0.5};
  std::array<double, 4> end_s_candidates = {10.0, 20.0, 40.0, 80.0};

  for (const auto& s : end_s_candidates) {
    for (const auto& d : end_d_candidates) {
      State end_d_state = {d, 0.0, 0.0};//d d. d.. 横向没有速度和加速度
      end_d_conditions.emplace_back(end_d_state, s);//横向终端状态的采样结果是横向坐标各信息 s坐标
    }
  }
  return end_d_conditions;
}
```

横向轨迹就是d(s)的表达式，自变量是s，沿着s轴坐标10，20，40，80分别采样-0.5 0 0.5的d坐标，也就是一共12个目标终点，12条横向轨迹；横向状态的最后只有d坐标偏移，横向速度、加速度都是0；

然后再用横向的初始状态、终止状态用5次多项式来拟合，计算出系数并保留。

第二种方法：使用优化的方法

重要的函数就是求得横向坐标约束 然后放入求解器进行求解

```C++
    auto lateral_bounds =
        ptr_path_time_graph_->GetLateralBounds(s_min, s_max, delta_s);
```

 具体来看：

```C++
std::vector<std::pair<double, double>> PathTimeGraph::GetLateralBounds(
    const double s_start, const double s_end, const double s_resolution) {
  CHECK_LT(s_start, s_end);
  CHECK_GT(s_resolution, FLAGS_numerical_epsilon);
  std::vector<std::pair<double, double>> bounds;
  std::vector<double> discretized_path;
  double s_range = s_end - s_start;
  double s_curr = s_start;
  size_t num_bound = static_cast<size_t>(s_range / s_resolution);

  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_width = vehicle_config.vehicle_param().width();

  // Initialize bounds by reference line width 沿着道路中心线建立初步的边界 
  for (size_t i = 0; i < num_bound; ++i) {
    double left_width = FLAGS_default_reference_line_width / 2.0;
    double right_width = FLAGS_default_reference_line_width / 2.0;
    ptr_reference_line_info_->reference_line().GetLaneWidth(s_curr, &left_width,
                                                            &right_width);
    double ego_d_lower = init_d_[0] - ego_width / 2.0;
    double ego_d_upper = init_d_[0] + ego_width / 2.0;
    bounds.emplace_back(//pair
        std::min(-right_width, ego_d_lower - FLAGS_bound_buffer),
        std::max(left_width, ego_d_upper + FLAGS_bound_buffer));
    discretized_path.push_back(s_curr);
    s_curr += s_resolution;
  }
  //根据静态障碍物 更新边界
  for (const SLBoundary& static_sl_boundary : static_obs_sl_boundaries_) {
    UpdateLateralBoundsByObstacle(static_sl_boundary, discretized_path, s_start,
                                  s_end, &bounds);
  }

  for (size_t i = 0; i < bounds.size(); ++i) {//左右各压缩半个车宽 给汽车的宽度留出来距离 
    bounds[i].first += ego_width / 2.0;
    bounds[i].second -= ego_width / 2.0;
    if (bounds[i].first >= bounds[i].second) {
      bounds[i].first = 0.0;
      bounds[i].second = 0.0;
    }
  }
  return bounds;
}
```

其中最重要的是根据障碍物的位置和边界来更新横向坐标的边界 原本的边界是车在中心道路线上，默认马路的宽度的1/2；

```C++
void PathTimeGraph::UpdateLateralBoundsByObstacle(
    const SLBoundary& sl_boundary, const std::vector<double>& discretized_path,
    const double s_start, const double s_end,
    std::vector<std::pair<double, double>>* const bounds) {
  if (sl_boundary.start_s() > s_end || sl_boundary.end_s() < s_start) {
    return;//超过规划范围 不考虑
  }
  auto start_iter = std::lower_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  auto end_iter = std::upper_bound(
      discretized_path.begin(), discretized_path.end(), sl_boundary.start_s());
  size_t start_index = start_iter - discretized_path.begin();
  size_t end_index = end_iter - discretized_path.begin();
  if (sl_boundary.end_l() > -FLAGS_numerical_epsilon &&
      sl_boundary.start_l() < FLAGS_numerical_epsilon) {//障碍物横跨道路中心线
    for (size_t i = start_index; i < end_index; ++i) {//障碍物在路径上的索引范围
      bounds->operator[](i).first = -FLAGS_numerical_epsilon;//右 lower
      bounds->operator[](i).second = FLAGS_numerical_epsilon;//左 upper
    }//将路径索引范围内的边界设置为一个微小的负数和正数，表示路径的左右边界均可以被占据。
    return;
  }//FLAGS_numerical_epsilon是最小的正数 <it 那就是<0
  if (sl_boundary.end_l() < FLAGS_numerical_epsilon) {//end_l就是最大的坐标 说明障碍物在道路的右侧
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->operator[](i).first = //更改的是右边界
          std::max(bounds->operator[](i).first,
                   sl_boundary.end_l() + FLAGS_nudge_buffer);//FLAGS_nudge_buffer缓冲值 防止规划的时候撞上
    }
    return;
  }
  if (sl_boundary.start_l() > -FLAGS_numerical_epsilon) {//障碍物在道路的左侧 更改的是左边界
    for (size_t i = start_index; i < std::min(end_index + 1, bounds->size());
         ++i) {
      bounds->operator[](i).second =
          std::min(bounds->operator[](i).second,
                   sl_boundary.start_l() - FLAGS_nudge_buffer);
    }
    return;
  }
}
```







主要分为了三种情况：1.障碍物横跨道路中心线的两侧，那其实车就不能向前通过了，所以横向采样的边界为（-0， 0）；

2.障碍物最大的d坐标在道路中心线的右侧，那边就更新横向采样的右边界；

2.障碍物最大小的d坐标在道路中心线的坐侧，那边就更新横向采样的坐边界；

## 6轨迹评估 

#### 官方注释 

first, evaluate the feasibility of the 1d trajectories according to dynamic constraints.

second, evaluate the feasible longitudinal and lateral trajectory pairs and sort them according to the cost.

第一，根据动态的约束评估一维轨迹的可行性；第二，评估可行的纵向横向轨迹对，并且将它们按照代价值排序；

```C++
TrajectoryEvaluator::TrajectoryEvaluator(//输入：初始状态s 规划目标 纵向轨迹 横向轨迹 ST图 离散的参考路径
    const std::array<double, 3>& init_s, const PlanningTarget& planning_target,
    const std::vector<PtrTrajectory1d>& lon_trajectories,
    const std::vector<PtrTrajectory1d>& lat_trajectories,
    std::shared_ptr<PathTimeGraph> path_time_graph,
    std::shared_ptr<std::vector<PathPoint>> reference_line)
    : path_time_graph_(path_time_graph),
      reference_line_(reference_line),
      init_s_(init_s) {
  const double start_time = 0.0;
  const double end_time = FLAGS_trajectory_time_length;
  path_time_intervals_ = path_time_graph_->GetPathBlockingIntervals(//s_upper, s_lower 获取有障碍物的时候
   start_time, end_time, FLAGS_trajectory_time_resolution);

  reference_s_dot_ = ComputeLongitudinalGuideVelocity(planning_target);

  // if we have a stop point along the reference line,
  // filter out the lon. trajectories that pass the stop point.
  double stop_point = std::numeric_limits<double>::max();
  if (planning_target.has_stop_point()) {
    stop_point = planning_target.stop_point().s();
  }
  for (const auto& lon_trajectory : lon_trajectories) {//先检查纵向轨迹
    double lon_end_s = lon_trajectory->Evaluate(0, end_time);
    if (init_s[0] < stop_point &&
        lon_end_s + FLAGS_lattice_stop_buffer > stop_point) {
      continue;
    }//如果轨迹停止点超过了目标点的终点位置 就略过

    if (!ConstraintChecker1d::IsValidLongitudinalTrajectory(*lon_trajectory)) {
      continue;
    }//还检查轨迹是否有效
    for (const auto& lat_trajectory : lat_trajectories) {//再检查横向轨迹
      /**
       * The validity of the code needs to be verified.
      if (!ConstraintChecker1d::IsValidLateralTrajectory(*lat_trajectory,
                                                         *lon_trajectory)) {
        continue;
      }
      */
      double cost = Evaluate(planning_target, lon_trajectory, lat_trajectory);
      cost_queue_.emplace(Trajectory1dPair(lon_trajectory, lat_trajectory),
                          cost);
    }
  }
  ADEBUG << "Number of valid 1d trajectory pairs: " << cost_queue_.size();
}
```

### 6.1 获取每个时刻对应所有障碍物的(s_lower,  s_upper)坐标；

第 一个函数是path_time_graph_->GetPathBlockingIntervals(start_time, end_time, FLAGS_trajectory_time_resolution)，是根据ST图的时间分辨率，求出每个时刻障碍物的s最小最大的坐标；具体函数如下：

```C++
std::vector<std::vector<std::pair<double, double>>>
PathTimeGraph::GetPathBlockingIntervals(const double t_start,
                                        const double t_end,
                                        const double t_resolution) {
  std::vector<std::vector<std::pair<double, double>>> intervals;
  for (double t = t_start; t <= t_end; t += t_resolution) {//每个时刻
    intervals.push_back(GetPathBlockingIntervals(t));//运行这个函数
  }
  return intervals;
}
```

```C++
std::vector<std::pair<double, double>> PathTimeGraph::GetPathBlockingIntervals(
    const double t) const {
  ACHECK(time_range_.first <= t && t <= time_range_.second);
  std::vector<std::pair<double, double>> intervals;
  for (const auto& pt_obstacle : path_time_obstacles_) {
    if (t > pt_obstacle.max_t() || t < pt_obstacle.min_t()) {
      continue;
    }
    double s_upper = lerp(pt_obstacle.upper_left_point().s(),//找ST图上面的点进行插值
                          pt_obstacle.upper_left_point().t(),
                          pt_obstacle.upper_right_point().s(),
                          pt_obstacle.upper_right_point().t(), t);

    double s_lower = lerp(pt_obstacle.bottom_left_point().s(),
                          pt_obstacle.bottom_left_point().t(),
                          pt_obstacle.bottom_right_point().s(),
                          pt_obstacle.bottom_right_point().t(), t);

    intervals.emplace_back(s_lower, s_upper);//t时刻对应的s坐标的上下界 可以对应多个障碍物的上下界
  }
  return intervals;
}
```

### 6.2  计算参考轨迹的速度---理论上每一个时刻应该的速度是多少

```C++
reference_s_dot_ = ComputeLongitudinalGuideVelocity(planning_target);
std::vector<double> TrajectoryEvaluator::ComputeLongitudinalGuideVelocity(
    const PlanningTarget& planning_target) const {
  std::vector<double> reference_s_dot;//返回值
  double cruise_v = planning_target.cruise_speed();//返回设定的目标速度
  if (!planning_target.has_stop_point()) {//没有停止点
    PiecewiseAccelerationTrajectory1d lon_traj(init_s_[0], cruise_v);//(s, v)
    lon_traj.AppendSegment(//添加FLAGS_trajectory_time_length最后的状态s v t a 
        0.0, FLAGS_trajectory_time_length + FLAGS_numerical_epsilon);//规划最后时刻的速度 参数是加速度和时间间隔  (a,t) 在轨迹规划的最后时刻 添加（加速度a，持续时间t） 
    //添加的分段就是巡航速度开FLAGS_trajectory_time_length时间
    for (double t = 0.0; t < FLAGS_trajectory_time_length;
         t += FLAGS_trajectory_time_resolution) {
      reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));//得到每一个时刻点的速度 其实就是规划速度
    } //规划这是一个匀速的运动
  } else {//有停止点
    double dist_s = planning_target.stop_point().s() - init_s_[0];
    if (dist_s < FLAGS_numerical_epsilon) {//停止点太近
      PiecewiseAccelerationTrajectory1d lon_traj(init_s_[0], 0.0);//s, v
      lon_traj.AppendSegment(
          0.0, FLAGS_trajectory_time_length + FLAGS_numerical_epsilon);

      for (double t = 0.0; t < FLAGS_trajectory_time_length;
           t += FLAGS_trajectory_time_resolution) {
        reference_s_dot.emplace_back(lon_traj.Evaluate(1, t));//s. = v
      }
      return reference_s_dot;
    }
    //停止点在一定距离外

    double a_comfort = FLAGS_longitudinal_acceleration_upper_bound * 
                       FLAGS_comfort_acceleration_factor;
    //舒适加速度，即车辆加速时允许的最大加速度
    double d_comfort = -FLAGS_longitudinal_acceleration_lower_bound * 
                       FLAGS_comfort_acceleration_factor;
    //舒适减速度，即车辆减速时允许的最大减速度
    std::shared_ptr<Trajectory1d> lon_ref_trajectory = //产生分段刹车轨迹
        PiecewiseBrakingTrajectoryGenerator::Generate(
            planning_target.stop_point().s(), init_s_[0],
            planning_target.cruise_speed(), init_s_[1], a_comfort, d_comfort,
            FLAGS_trajectory_time_length + FLAGS_numerical_epsilon);

    for (double t = 0.0; t < FLAGS_trajectory_time_length;
         t += FLAGS_trajectory_time_resolution) {
      reference_s_dot.emplace_back(lon_ref_trajectory->Evaluate(1, t));
    }
  }
  return reference_s_dot;
}
```

分析：如果没有停止点的话那就是巡航，这段规划的轨迹每个时刻的速度是固定的。如果有停止点，再去判断：停止点已经到达的话，那么速度就都设置为0；如果离停止点还有一段距离，计算出舒适的加速度和减速度的大小，计算分段刹车的轨迹，然后把轨迹的速度计算出来存储。看一下产生分段轨迹的函数：

```C++
std::shared_ptr<Curve1d> PiecewiseBrakingTrajectoryGenerator::Generate(
    const double s_target, const double s_curr, const double v_target,
    const double v_curr, const double a_comfort, const double d_comfort,
    const double max_time) {
  std::shared_ptr<PiecewiseAccelerationTrajectory1d> ptr_trajectory =
      std::make_shared<PiecewiseAccelerationTrajectory1d>(s_curr, v_curr);//使用当前速度v,s生成一个制动轨迹

  double s_dist = s_target - s_curr;//距离目标终点的s距离

  double comfort_stop_dist = ComputeStopDistance(v_curr, d_comfort);//以舒适减速度进行制动需要的距离

  // if cannot stop using comfort deceleration, then brake in the beginning.
  if (comfort_stop_dist > s_dist) { //制动距离不够吧 
    double stop_d = ComputeStopDeceleration(s_dist, v_curr);//如果想要速度减到0需要的加速度的大小 已经是最小的a
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
```

首先第一种情况是，1以舒适加速度进行刹车，但是刹车距离不够，这种情况下就采取以最小的加速度进行刹车，尽可能保证舒适，在终点位置进行停车；

2.如果当前速度是大于巡航的目标速度，由于现在肯定是舒适加速度刹车完全可以，那么肯定还可以再巡航一段时间，现在路径存储的三段就是：先减速到目标速度，然后保持一段时间目标速度，再从目标速度减速到0；

3.如果当前速度小于目标巡航速度，那么汽车要以舒适的加速度加速到目标速度，肯定还有一段从目标速度减速到0的过程，那么现在的问题就是看是否有剩余距离可以提供巡航，这个的计算就是看s_rest ，如果有剩余时间，那么就再巡航一会；如果剩余距离不够，那么说明没有办法采取舒适的加速到巡航速度，但是采取的措施并不是加大加速度，而是并不加速到巡航速度，小于目标速度。然后就进行舒适减速度减速。

#### 总结:根据目标的设置，就得到了每个时刻的参考的速度。

### 6.3 计算横向、纵向轨迹对的cost值

接下来遍历横向和纵向轨迹，纵向轨迹会剔除掉一些：

1.如果轨迹停止点超过了目标点的终点位置 就略过这条轨迹；

2.如果规划的每个时刻纵向轨迹的速度、加速度、jerk不在设定的范围内，就忽略；

接下来就是来评估纵向轨迹和横向轨迹的结合：

横向和纵向分别考虑不同的代价:

```C++
double TrajectoryEvaluator::Evaluate(
    const PlanningTarget& planning_target,
    const PtrTrajectory1d& lon_trajectory,
    const PtrTrajectory1d& lat_trajectory,
    std::vector<double>* cost_components) const {
  // Costs:
  // 1. Cost of missing the objective, e.g., cruise, stop, etc.
  // 2. Cost of longitudinal jerk
  // 3. Cost of longitudinal collision
  // 4. Cost of lateral offsets
  // 5. Cost of lateral comfort

  // Longitudinal costs
  double lon_objective_cost =
      LonObjectiveCost(lon_trajectory, planning_target, reference_s_dot_);//纵向目标代价

  double lon_jerk_cost = LonComfortCost(lon_trajectory);//纵向舒适度代价

  double lon_collision_cost = LonCollisionCost(lon_trajectory);//碰撞代价

  double centripetal_acc_cost = CentripetalAccelerationCost(lon_trajectory);//向心加速度代价

  // decides the longitudinal evaluation horizon for lateral trajectories.
  double evaluation_horizon =
      std::min(FLAGS_speed_lon_decision_horizon,
               lon_trajectory->Evaluate(0, lon_trajectory->ParamLength()));
  std::vector<double> s_values;
  for (double s = 0.0; s < evaluation_horizon;
       s += FLAGS_trajectory_space_resolution) {
    s_values.emplace_back(s);
  }

  // Lateral costs
  double lat_offset_cost = LatOffsetCost(lat_trajectory, s_values);//横向距离的大小

  double lat_comfort_cost = LatComfortCost(lon_trajectory, lat_trajectory);//横向舒适度

  if (cost_components != nullptr) {
    cost_components->emplace_back(lon_objective_cost);
    cost_components->emplace_back(lon_jerk_cost);
    cost_components->emplace_back(lon_collision_cost);
    cost_components->emplace_back(lat_offset_cost);
  }

  return lon_objective_cost * FLAGS_weight_lon_objective +
         lon_jerk_cost * FLAGS_weight_lon_jerk +
         lon_collision_cost * FLAGS_weight_lon_collision +
         centripetal_acc_cost * FLAGS_weight_centripetal_acceleration +
         lat_offset_cost * FLAGS_weight_lat_offset +
         lat_comfort_cost * FLAGS_weight_lat_comfort;
}
```

然后将纵向轨迹，横向轨迹，计算的代价值放入到cost_queue_当中；

### 6.4 碰撞检测计算

```C++
CollisionChecker collision_checker(frame->obstacles(), init_s[0], init_d[0] ，                 *ptr_reference_line, reference_line_info,ptr_path_time_graph);
BuildPredictedEnvironment(obstacles, ego_vehicle_s, ego_vehicle_d,//里面是构造预测环境函数
                            discretized_reference_line);
void CollisionChecker::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles, const double ego_vehicle_s,
    const double ego_vehicle_d,
    const std::vector<PathPoint>& discretized_reference_line) {
  ACHECK(predicted_bounding_rectangles_.empty());

  // If the ego vehicle is in lane,
  // then, ignore all obstacles from the same lane.
  bool ego_vehicle_in_lane = IsEgoVehicleInLane(ego_vehicle_s, ego_vehicle_d);//车是否在Lane上
  std::vector<const Obstacle*> obstacles_considered;//输出 要考虑的障碍物
  for (const Obstacle* obstacle : obstacles) {//遍历所有的障碍物
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (ego_vehicle_in_lane && 
        (IsObstacleBehindEgoVehicle(obstacle, ego_vehicle_s, discretized_reference_line) ||
         !ptr_path_time_graph_->IsObstacleInGraph(obstacle->Id()))) {//忽略不在建立的图上的障碍物
      continue;//IsObstacleBehindEgoVehicle忽略跟他在一条道路上且s坐标小于它的障碍物 
    }

    obstacles_considered.push_back(obstacle);//放入需要考虑的障碍物的列表中
  }

  double relative_time = 0.0;
  while (relative_time < FLAGS_trajectory_time_length) {//把障碍物每一个时刻的轨迹点 画成一个箱子
    std::vector<Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles_considered) {
      // If an obstacle has no trajectory, it is considered as static.
      // Obstacle::GetPointAtTime has handled this case.
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      box.LongitudinalExtend(2.0 * FLAGS_lon_collision_buffer);//纵向膨胀
      box.LateralExtend(2.0 * FLAGS_lat_collision_buffer);//横向膨胀
      predicted_env.push_back(std::move(box));//添加到预测的环境中
    }
    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}


```

就是把每个时刻所有的障碍物的预测轨迹点换成箱子并且进行膨胀，添加到预测矩形边界上面；

## 7 组合轨迹并选取路径

### 官方注释：always get the best pair of trajectories to combine; return the first collision-free trajectory 返回代价最小的无碰撞路径；

```C++
  size_t constraint_failure_count = 0;
  size_t collision_failure_count = 0;
  size_t combined_constraint_failure_count = 0;

  size_t lon_vel_failure_count = 0;
  size_t lon_acc_failure_count = 0;
  size_t lon_jerk_failure_count = 0;
  size_t curvature_failure_count = 0;
  size_t lat_acc_failure_count = 0;
  size_t lat_jerk_failure_count = 0;

  size_t num_lattice_traj = 0;

  while (trajectory_evaluator.has_more_trajectory_pairs()) {
    double trajectory_pair_cost =
        trajectory_evaluator.top_trajectory_pair_cost();
    auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();//把第一个搞走取下一个

    // combine two 1d trajectories to one 2d trajectory
    auto combined_trajectory = TrajectoryCombiner::Combine(
        *ptr_reference_line, *trajectory_pair.first, *trajectory_pair.second,
        planning_init_point.relative_time());

    // check longitudinal and lateral acceleration
    // considering trajectory curvatures
    auto result = ConstraintChecker::ValidTrajectory(combined_trajectory);
    if (result != ConstraintChecker::Result::VALID) {
      ++combined_constraint_failure_count;

      switch (result) {
        case ConstraintChecker::Result::LON_VELOCITY_OUT_OF_BOUND:
          lon_vel_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_ACCELERATION_OUT_OF_BOUND:
          lon_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LON_JERK_OUT_OF_BOUND:
          lon_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::CURVATURE_OUT_OF_BOUND:
          curvature_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_ACCELERATION_OUT_OF_BOUND://向心加速度
          lat_acc_failure_count += 1;
          break;
        case ConstraintChecker::Result::LAT_JERK_OUT_OF_BOUND://程序个注释了
          lat_jerk_failure_count += 1;
          break;
        case ConstraintChecker::Result::VALID:
        default:
          // Intentional empty
          break;
      }
      continue;
    }

    // check collision with other obstacles
    if (collision_checker.InCollision(combined_trajectory)) {
      ++collision_failure_count;
      continue;
    }
```

首先用变量来计数出现各种失败的轨迹的数量，最重要的函数是组合：如何把两个轨迹合并成为一条轨迹:

```C++
DiscretizedTrajectory TrajectoryCombiner::Combine(
    const std::vector<PathPoint>& reference_line, const Curve1d& lon_trajectory,
    const Curve1d& lat_trajectory, const double init_relative_time) {
  DiscretizedTrajectory combined_trajectory;

  double s0 = lon_trajectory.Evaluate(0, 0.0);//纵向轨迹的起始点
  double s_ref_max = reference_line.back().s();//参考路径的最大s
  double accumulated_trajectory_s = 0.0;
  PathPoint prev_trajectory_point;

  double last_s = -FLAGS_numerical_epsilon;// 0
  double t_param = 0.0;
  while (t_param < FLAGS_trajectory_time_length) {
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about t_param > lon_trajectory.ParamLength() situation
    double s = lon_trajectory.Evaluate(0, t_param);
    if (last_s > 0.0) {
      s = std::max(last_s, s);
    }
    last_s = s;

    double s_dot =
        std::max(FLAGS_numerical_epsilon, lon_trajectory.Evaluate(1, t_param));
    double s_ddot = lon_trajectory.Evaluate(2, t_param);
    if (s > s_ref_max) {
      break;//纵向轨迹的坐标s超过参考系的s 结束
    }

    double relative_s = s - s0;//相对位置
    // linear extrapolation is handled internally in LatticeTrajectory1d;
    // no worry about s_param > lat_trajectory.ParamLength() situation
    double d = lat_trajectory.Evaluate(0, relative_s);//横向轨迹的各个参数
    double d_prime = lat_trajectory.Evaluate(1, relative_s);
    double d_pprime = lat_trajectory.Evaluate(2, relative_s);

    PathPoint matched_ref_point = PathMatcher::MatchToPath(reference_line, s);//将s坐标进行投影

    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double kappa = 0.0;
    double v = 0.0;
    double a = 0.0;

    const double rs = matched_ref_point.s();
    const double rx = matched_ref_point.x();
    const double ry = matched_ref_point.y();
    const double rtheta = matched_ref_point.theta();
    const double rkappa = matched_ref_point.kappa();
    const double rdkappa = matched_ref_point.dkappa();

    std::array<double, 3> s_conditions = {rs, s_dot, s_ddot};
    std::array<double, 3> d_conditions = {d, d_prime, d_pprime};
    CartesianFrenetConverter::frenet_to_cartesian(//转换到全局坐标系下
        rs, rx, ry, rtheta, rkappa, rdkappa, s_conditions, d_conditions, &x, &y,
        &theta, &kappa, &v, &a);

    if (prev_trajectory_point.has_x() && prev_trajectory_point.has_y()) {
      double delta_x = x - prev_trajectory_point.x();
      double delta_y = y - prev_trajectory_point.y();
      double delta_s = std::hypot(delta_x, delta_y);
      accumulated_trajectory_s += delta_s;
    }

    TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->set_x(x);
    trajectory_point.mutable_path_point()->set_y(y);
    trajectory_point.mutable_path_point()->set_s(accumulated_trajectory_s);
    trajectory_point.mutable_path_point()->set_theta(theta);
    trajectory_point.mutable_path_point()->set_kappa(kappa);
    trajectory_point.set_v(v);
    trajectory_point.set_a(a);
    trajectory_point.set_relative_time(t_param + init_relative_time);

    combined_trajectory.AppendTrajectoryPoint(trajectory_point);

    t_param = t_param + FLAGS_trajectory_time_resolution;

    prev_trajectory_point = trajectory_point.path_point();
  }
  return combined_trajectory;
}
```

那就是根据规划的每个时刻t，根据纵向轨迹获取{s, s. , s..},

1.根据s值和横向轨迹我们可以获取{l, l , l..}

2.根据参考线和s值我们还可以获取这个轨迹点投影到frenet坐标系的投影点，获取投影点的x,y,s,theta,kappa,dkappa.

3.根据投影点的各信息，{s,s., s..},{l, l‘, l’‘}获取这个轨迹点的全局坐标，从frenet转化为全局坐标，然后计算出其累积的s坐标；

把这些信息转化为一个轨迹点，并把它添加到轨迹上；

然后对于轨迹进行合法性的检查，然后再检查是否会碰撞障碍物。

如果轨迹是可执行的，则设置为

```C++
 reference_line_info->SetDrivable(true);
```

























































































