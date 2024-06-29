/*******************************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "hybrid_a_star/hybrid_a_star.h"
#include "hybrid_a_star/display_tools.h"
#include "hybrid_a_star/timer.h"
#include "hybrid_a_star/trajectory_optimizer.h"

#include <iostream>

HybridAStar::HybridAStar(double steering_angle, int steering_angle_discrete_num, double segment_length,
                         int segment_length_discrete_num, double wheel_base, double steering_penalty,
                         double reversing_penalty, double steering_change_penalty, double shot_distance,
                         int grid_size_phi) {
    // 存储车辆轴距
    wheel_base_ = wheel_base;
    // 存储每段长度
    segment_length_ = segment_length;
    // 将角度转换为弧度
    steering_radian_ = steering_angle * M_PI / 180.0; // angle to radian
    // 存储转向角离散值
    steering_discrete_num_ = steering_angle_discrete_num;
    // 计算转向角离散步长
    steering_radian_step_size_ = steering_radian_ / steering_discrete_num_;
    // 计算移动步长
    move_step_size_ = segment_length / segment_length_discrete_num;
    // 存储离散段数
    segment_length_discrete_num_ = static_cast<int>(segment_length_discrete_num);
    // 存储转向惩罚
    steering_penalty_ = steering_penalty;
    // 存储转向变化惩罚
    steering_change_penalty_ = steering_change_penalty;
    // 存储倒车惩罚
    reversing_penalty_ = reversing_penalty;
    // 存储射击距离
    shot_distance_ = shot_distance;

    // 检查每段长度是否可以被步长整除
    CHECK_EQ(static_cast<float>(segment_length_discrete_num_ * move_step_size_), static_cast<float>(segment_length_))
        << "The segment length must be divisible by the step size. segment_length: "
        << segment_length_ << " | step_size: " << move_step_size_;

    // 创建路径
    rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));
    // 设置ties_breaker
    tie_breaker_ = 1.0 + 1e-3;

    // 设置状态网格大小
    STATE_GRID_SIZE_PHI_ = grid_size_phi;
    // 设置角度分辨率
    ANGULAR_RESOLUTION_ = 360.0 / STATE_GRID_SIZE_PHI_ * M_PI / 180.0;
}

HybridAStar::~HybridAStar() {
    ReleaseMemory();
}

// 初始化混合A星算法
void HybridAStar::Init(double x_lower, double x_upper, double y_lower, double y_upper,
                       double state_grid_resolution, double map_grid_resolution) {
    // 设置车辆形状
    SetVehicleShape(4.7, 2.0, 1.3);

    // 地图空间的x下界
    map_x_lower_ = x_lower;
    // 地图空间的x上界
    map_x_upper_ = x_upper;
    // 地图空间的y下界
    map_y_lower_ = y_lower;
    // 地图空间的y上界
    map_y_upper_ = y_upper;
    // 状态网格的分辨率
    STATE_GRID_RESOLUTION_ = state_grid_resolution;
    // 地图网格的分辨率
    MAP_GRID_RESOLUTION_ = map_grid_resolution;

    // 计算状态网格的x轴大小
    STATE_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / STATE_GRID_RESOLUTION_);
    // 计算状态网格的y轴大小
    STATE_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / STATE_GRID_RESOLUTION_);

    // 计算地图网格的x轴大小
    MAP_GRID_SIZE_X_ = std::floor((map_x_upper_ - map_x_lower_) / MAP_GRID_RESOLUTION_);
    // 计算地图网格的y轴大小
    MAP_GRID_SIZE_Y_ = std::floor((map_y_upper_ - map_y_lower_) / MAP_GRID_RESOLUTION_);

    // 如果map_data_不为空，则删除数组，并将map_data_置为空
    if (map_data_) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    // 分配一个新的uint8_t数组，大小为MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_
    map_data_ = new uint8_t[MAP_GRID_SIZE_X_ * MAP_GRID_SIZE_Y_];

    // 销毁state_node_map_数组
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {

            // 如果state_node_map_[i]为空，则跳过
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                // 如果state_node_map_[i][j]为空，则跳过
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    // 如果state_node_map_[i][j][k]不为空，则删除
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
                // 删除state_node_map_[i][j]
                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }
            // 删除state_node_map_[i]
            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        // 删除state_node_map_
        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    // 重新创建state_node_map_数组
    state_node_map_ = new StateNode::Ptr **[STATE_GRID_SIZE_X_];
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        state_node_map_[i] = new StateNode::Ptr *[STATE_GRID_SIZE_Y_];
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            state_node_map_[i][j] = new StateNode::Ptr[STATE_GRID_SIZE_PHI_];
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                // 初始化state_node_map_[i][j][k]
                state_node_map_[i][j][k] = nullptr;
            }
        }
    }
}

// 检查两点之间的路径是否可以通过
inline bool HybridAStar::LineCheck(double x0, double y0, double x1, double y1) {
    // 如果x1-x2的值大于y1-y2的值，则交换x0,y0,x1,y1的值
    bool steep = (std::abs(y1 - y0) > std::abs(x1 - x0));

    if (steep) {
        std::swap(x0, y0);
        std::swap(y1, x1);
    }

    // 如果x0>x1，则交换x0,x1的值
    if (x0 > x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    // 计算x1-x0的值
    auto delta_x = x1 - x0;
    // 计算y1-y0的绝对值
    auto delta_y = std::abs(y1 - y0);
    // 计算y1-y0的值除以x1-x0的值的值
    auto delta_error = delta_y / delta_x;
    // 声明类型为delta_x的变量error
    decltype(delta_x) error = 0;
    // 声明类型为delta_x的变量y_step
    decltype(delta_x) y_step;
    // 声明类型为delta_x的变量yk
    auto yk = y0;

    // 如果y0<y1，则y_step为1，否则y_step为-1
    if (y0 < y1) {
        y_step = 1;
    } else {
        y_step = -1;
    }

    // 计算x1-x0的值加1的值
    auto N = static_cast<unsigned int>(x1 - x0);
    for (unsigned int i = 0; i < N; ++i) {
        // 如果steep为true，则判断HasObstacle(yk, x0+i*1.0)是否为true，或者判断BeyondBoundary(yk*MAP_GRID_RESOLUTION_, (x0+i)*MAP_GRID_RESOLUTION_)是否为true
        if (steep) {
            if (HasObstacle(Vec2i(yk, x0 + i * 1.0))
                || BeyondBoundary(Vec2d(yk, (x0 + i) * MAP_GRID_RESOLUTION_))
                    ) {
                return false;
            }
        // 否则判断HasObstacle(x0+i*1.0, yk)是否为true，或者判断BeyondBoundary((x0+i)*MAP_GRID_RESOLUTION_, yk*MAP_GRID_RESOLUTION_)是否为true
        } else {
            if (HasObstacle(Vec2i(x0 + i * 1.0, yk))
                || BeyondBoundary((x0 + i) * MAP_GRID_RESOLUTION_, yk * MAP_GRID_RESOLUTION_)) {
                return false;
            }
        }

        // 计算error的值加delta_error的值
        error += delta_error;
        // 如果error的值大于等于0.5，则yk的值加y_step的值，计算error的值减1.0的值
        if (error >= 0.5) {
            yk += y_step;
            error = error - 1.0;
        }
    }

    return true; 
}

// 检测是否会发生碰撞
bool HybridAStar::CheckCollision(const double &x, const double &y, const double &theta) {
    // 定义一个计时器
    Timer timer;
    // 定义一个2x2的矩阵，用于转换车辆形状
    Mat2d R;
    R << std::cos(theta), -std::sin(theta),
            std::sin(theta), std::cos(theta);

    MatXd transformed_vehicle_shape;
    // 初始化矩阵大小
    transformed_vehicle_shape.resize(8, 1);
    // 车辆形状转换为矩阵
    for (unsigned int i = 0; i < 4u; ++i) {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0)
                = R * vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(x, y);
    }

    // 将坐标转换为网格索引
    Vec2i transformed_pt_index_0 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(0, 0)
    );
    Vec2i transformed_pt_index_1 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(2, 0)
    );

    Vec2i transformed_pt_index_2 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(4, 0)
    );

    Vec2i transformed_pt_index_3 = Coordinate2MapGridIndex(
            transformed_vehicle_shape.block<2, 1>(6, 0)
    );

    double y1, y0, x1, x0;
    // pt1 -> pt0
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_1.x());
    y1 = static_cast<double>(transformed_pt_index_1.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt2 -> pt1
    x0 = static_cast<double>(transformed_pt_index_1.x());
    y0 = static_cast<double>(transformed_pt_index_1.y());
    x1 = static_cast<double>(transformed_pt_index_2.x());
    y1 = static_cast<double>(transformed_pt_index_2.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt3 -> pt2
    x0 = static_cast<double>(transformed_pt_index_2.x());
    y0 = static_cast<double>(transformed_pt_index_2.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x1, y1, x0, y0)) {
        return false;
    }

    // pt0 -> pt3
    x0 = static_cast<double>(transformed_pt_index_0.x());
    y0 = static_cast<double>(transformed_pt_index_0.y());
    x1 = static_cast<double>(transformed_pt_index_3.x());
    y1 = static_cast<double>(transformed_pt_index_3.y());

    if (!LineCheck(x0, y0, x1, y1)) {
        return false;
    }

    check_collision_use_time += timer.End();
    num_check_collision++;
    return true;
}

// 检查给定的网格索引是否有障碍物
bool HybridAStar::HasObstacle(const int grid_index_x, const int grid_index_y) const {
    // 检查网格索引是否在有效范围内
    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            // 检查网格索引处的地图数据是否为障碍物
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

// 检查给定的Vec2i是否有障碍物
bool HybridAStar::HasObstacle(const Vec2i &grid_index) const {
    int grid_index_x = grid_index[0];
    int grid_index_y = grid_index[1];

    return (grid_index_x >= 0 && grid_index_x < MAP_GRID_SIZE_X_
            && grid_index_y >= 0 && grid_index_y < MAP_GRID_SIZE_Y_
            && (map_data_[grid_index_y * MAP_GRID_SIZE_X_ + grid_index_x] == 1));
}

// 设置障碍物
void HybridAStar::SetObstacle(unsigned int x, unsigned int y) {
    // 如果x，y超出了地图大小，则返回
    if (x > static_cast<unsigned int>(MAP_GRID_SIZE_X_)
        || y > static_cast<unsigned int>(MAP_GRID_SIZE_Y_)) {
        return;
    }

    // 将x，y对应的地图数据设置为障碍物
    map_data_[x + y * MAP_GRID_SIZE_X_] = 1;
}

// 设置障碍物
void HybridAStar::SetObstacle(const double pt_x, const double pt_y) {
    // 如果pt_x，pt_y超出了地图范围，则返回
    if (pt_x < map_x_lower_ || pt_x > map_x_upper_ ||
        pt_y < map_y_lower_ || pt_y > map_y_upper_) {
        return;
    }

    // 将pt_x，pt_y对应的地图数据设置为障碍物
    int grid_index_x = static_cast<int>((pt_x - map_x_lower_) / MAP_GRID_RESOLUTION_);
    int grid_index_y = static_cast<int>((pt_y - map_y_lower_) / MAP_GRID_RESOLUTION_);

    map_data_[grid_index_x + grid_index_y * MAP_GRID_SIZE_X_] = 1;
}

// 设置车辆形状，包括长度、宽度、后轴距离
void HybridAStar::SetVehicleShape(double length, double width, double rear_axle_dist) {
    // 初始化车辆形状向量
    vehicle_shape_.resize(8);
    // 设置车辆形状向量的前两个元素，表示车辆的前轴到车辆左前角距离
    vehicle_shape_.block<2, 1>(0, 0) = Vec2d(-rear_axle_dist, width / 2);
    vehicle_shape_.block<2, 1>(2, 0) = Vec2d(length - rear_axle_dist, width / 2);
    // 设置车辆形状向量的后两个元素，表示车辆的后轴到车辆右后角距离
    vehicle_shape_.block<2, 1>(4, 0) = Vec2d(length - rear_axle_dist, -width / 2);
    vehicle_shape_.block<2, 1>(6, 0) = Vec2d(-rear_axle_dist, -width / 2);

    // 设置车辆形状离散化时使用的步长
    const double step_size = move_step_size_;
    // 计算车辆形状离散化时需要的元素数量
    const auto N_length = static_cast<unsigned int>(length / step_size);
    const auto N_width = static_cast<unsigned int> (width / step_size);
    // 初始化车辆形状离散化向量
    vehicle_shape_discrete_.resize(2, (N_length + N_width) * 2u);

    // 计算车辆形状离散化时车辆左前角到右前角的向量
    const Vec2d edge_0_normalized = (vehicle_shape_.block<2, 1>(2, 0)
                                     - vehicle_shape_.block<2, 1>(0, 0)).normalized();
    // 计算车辆形状离散化时车辆右后角到左后角的向量
    const Vec2d edge_1_normalized = (vehicle_shape_.block<2, 1>(4, 0)
                                     - vehicle_shape_.block<2, 1>(2, 0)).normalized();
    // 计算车辆形状离散化时车辆左前角到右前角的距离
    for (unsigned int i = 0; i < N_length; ++i) {
        // 计算车辆形状离散化时车辆左前角到右前角的距离
        vehicle_shape_discrete_.block<2, 1>(0, i + N_length)
                = vehicle_shape_.block<2, 1>(4, 0) - edge_0_normalized * i * step_size;
        // 计算车辆形状离散化时车辆左前角到右前角的距离
        vehicle_shape_discrete_.block<2, 1>(0, i)
                = vehicle_shape_.block<2, 1>(0, 0) + edge_0_normalized * i * step_size;
    }

    // 计算车辆形状离散化时车辆右后角到左后角的距离
    for (unsigned int i = 0; i < N_width; ++i) {
        // 计算车辆形状离散化时车辆右后角到左后角的距离
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i)
                = vehicle_shape_.block<2, 1>(2, 0) + edge_1_normalized * i * step_size;
        // 计算车辆形状离散化时车辆右后角到左后角的距离
        vehicle_shape_discrete_.block<2, 1>(0, (2 * N_length) + i + N_width)
                = vehicle_shape_.block<2, 1>(6, 0) - edge_1_normalized * i * step_size;
    }
}

// 属性为未使用，返回一个Vec2d类型的值，传入参数为pt
__attribute__((unused)) Vec2d HybridAStar::CoordinateRounding(const Vec2d &pt) const {
    // 将pt坐标映射到网格坐标系
    return MapGridIndex2Coordinate(Coordinate2MapGridIndex(pt));
}

// 将网格坐标系映射到坐标系
Vec2d HybridAStar::MapGridIndex2Coordinate(const Vec2i &grid_index) const {
    Vec2d pt;
    // 将网格坐标转换为坐标系坐标
    pt.x() = ((double) grid_index[0] + 0.5) * MAP_GRID_RESOLUTION_ + map_x_lower_;
    pt.y() = ((double) grid_index[1] + 0.5) * MAP_GRID_RESOLUTION_ + map_y_lower_;

    return pt;
}

// 将状态转换为索引
Vec3i HybridAStar::State2Index(const Vec3d &state) const {
    Vec3i index;

    // 将状态转换为索引
    index[0] = std::min(std::max(int((state[0] - map_x_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_X_ - 1);
    index[1] = std::min(std::max(int((state[1] - map_y_lower_) / STATE_GRID_RESOLUTION_), 0), STATE_GRID_SIZE_Y_ - 1);
    index[2] = std::min(std::max(int((state[2] - (-M_PI)) / ANGULAR_RESOLUTION_), 0), STATE_GRID_SIZE_PHI_ - 1);

    return index;
}

// 将坐标系坐标转换为网格坐标系
Vec2i HybridAStar::Coordinate2MapGridIndex(const Vec2d &pt) const {
    Vec2i grid_index;

    // 将坐标系坐标转换为网格坐标系
    grid_index[0] = int((pt[0] - map_x_lower_) / MAP_GRID_RESOLUTION_);
    grid_index[1] = int((pt[1] - map_y_lower_) / MAP_GRID_RESOLUTION_);
    return grid_index;
}

// 获取相邻节点
void HybridAStar::GetNeighborNodes(const StateNode::Ptr &curr_node_ptr,
                                   std::vector<StateNode::Ptr> &neighbor_nodes) {
    // 清除相邻节点
    neighbor_nodes.clear();

    // 遍历转向角
    for (int i = -steering_discrete_num_; i <= steering_discrete_num_; ++i) {
        VectorVec3d intermediate_state;
        bool has_obstacle = false;

        // 获取当前节点状态
        double x = curr_node_ptr->state_.x();
        double y = curr_node_ptr->state_.y();
        double theta = curr_node_ptr->state_.z();

        // 计算转向角
        const double phi = i * steering_radian_step_size_;

        // 向前
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            // 计算 intermediate state
            DynamicModel(move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            // 检查碰撞
            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        // 转换为 grid index
        Vec3i grid_index = State2Index(intermediate_state.back());
        // 检查是否超出边界
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            // 创建相邻向前节点
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_forward_node_ptr->state_ = intermediate_state.back();
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        // 清除 intermediate state
        intermediate_state.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();

        // 向后
        has_obstacle = false;
        for (int j = 1; j <= segment_length_discrete_num_; j++) {
            // 计算 intermediate state
            DynamicModel(-move_step_size_, phi, x, y, theta);
            intermediate_state.emplace_back(Vec3d(x, y, theta));

            // 检查碰撞
            if (!CheckCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }

        // 转换为 grid index
        if (!BeyondBoundary(intermediate_state.back().head(2)) && !has_obstacle) {
            // 创建相邻向后节点
            grid_index = State2Index(intermediate_state.back());
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_state;
            neighbor_backward_node_ptr->state_ = intermediate_state.back();
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

// 计算step_size步长后车子的状态
void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) const {
    // 计算新的x坐标
    x = x + step_size * std::cos(theta);
    // 计算新的y坐标
    y = y + step_size * std::sin(theta);
    // 计算新的theta角度
    theta = Mod2Pi(theta + step_size / wheel_base_ * std::tan(phi));
}

double HybridAStar::Mod2Pi(const double &x) {
    double v = fmod(x, 2 * M_PI);

    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

bool HybridAStar::BeyondBoundary(const Vec2d &pt) const {
    return pt.x() < map_x_lower_ || pt.x() > map_x_upper_ || pt.y() < map_y_lower_ || pt.y() > map_y_upper_;
}

// 用于计算两个状态节点之间的启发式函数
// 启发式函数用于评估从当前节点到目标节点的路径质量，从而帮助算法选择最佳路径。
double HybridAStar::ComputeH(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &terminal_node_ptr) {
    double h;
    // L2
    // h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).norm();

    // L1
    h = (current_node_ptr->state_.head(2) - terminal_node_ptr->state_.head(2)).lpNorm<1>();

    if (h < 3.0 * shot_distance_) {
        // 如果h小于3倍shot_distance_距离，则使用欧氏距离
        h = rs_path_ptr_->Distance(current_node_ptr->state_.x(), current_node_ptr->state_.y(),
                                   current_node_ptr->state_.z(),
                                   terminal_node_ptr->state_.x(), terminal_node_ptr->state_.y(),
                                   terminal_node_ptr->state_.z());
    }

    return h;
}

double HybridAStar::ComputeG(const StateNode::Ptr &current_node_ptr,
                             const StateNode::Ptr &neighbor_node_ptr) const {
    double g;
    // 如果邻居节点是前向方向
    if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
        // 如果邻居节点转向级别与当前节点转向级别不同
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            // 如果邻居节点转向级别为0
            if (neighbor_node_ptr->steering_grade_ == 0) {
                // 计算转向级别改变惩罚的g值
                g = segment_length_ * steering_change_penalty_;
            } else {
                // 计算转向级别改变惩罚和转向惩罚的g值
                g = segment_length_ * steering_change_penalty_ * steering_penalty_;
            }
        } else {
            // 如果邻居节点转向级别与当前节点转向级别相同
            if (neighbor_node_ptr->steering_grade_ == 0) {
                // 计算前向行驶的g值
                g = segment_length_;
            } else {
                // 计算转向级别和前向行驶惩罚的g值
                g = segment_length_ * steering_penalty_;
            }
        }
    } else {
        // 如果邻居节点是后向方向
        // 如果邻居节点转向级别与当前节点转向级别不同
        if (neighbor_node_ptr->steering_grade_ != current_node_ptr->steering_grade_) {
            // 如果邻居节点转向级别为0
            if (neighbor_node_ptr->steering_grade_ == 0) {
                // 计算转向级别改变惩罚的g值
                g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
            } else {
                // 计算转向级别改变惩罚和转向惩罚和后向惩罚的g值
                g = segment_length_ * steering_change_penalty_ * steering_penalty_ * reversing_penalty_;
            }
        } else {
            // 如果邻居节点转向级别与当前节点转向级别相同
            if (neighbor_node_ptr->steering_grade_ == 0) {
                // 计算后向行驶的g值
                g = segment_length_ * reversing_penalty_;
            } else {
                // 计算转向级别和后向行驶惩罚的g值
                g = segment_length_ * steering_penalty_ * reversing_penalty_;
            }
        }
    }

    return g;
}

bool HybridAStar::Search(const Vec3d &start_state, const Vec3d &goal_state) {
    Timer search_used_time;

    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_grid_index = State2Index(start_state);
    const Vec3i goal_grid_index = State2Index(goal_state);

    auto goal_node_ptr = new StateNode(goal_grid_index);
    goal_node_ptr->state_ = goal_state;
    goal_node_ptr->direction_ = StateNode::NO;
    goal_node_ptr->steering_grade_ = 0;

    auto start_node_ptr = new StateNode(start_grid_index);
    start_node_ptr->state_ = start_state;
    start_node_ptr->steering_grade_ = 0;
    start_node_ptr->direction_ = StateNode::NO;
    start_node_ptr->node_status_ = StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(start_state);
    start_node_ptr->g_cost_ = 0.0;
    start_node_ptr->f_cost_ = ComputeH(start_node_ptr, goal_node_ptr);

    state_node_map_[start_grid_index.x()][start_grid_index.y()][start_grid_index.z()] = start_node_ptr;
    state_node_map_[goal_grid_index.x()][goal_grid_index.y()][goal_grid_index.z()] = goal_node_ptr;

    openset_.clear();
    openset_.insert(std::make_pair(0, start_node_ptr));

    std::vector<StateNode::Ptr> neighbor_nodes_ptr;
    StateNode::Ptr current_node_ptr;
    StateNode::Ptr neighbor_node_ptr;

    int count = 0;
    while (!openset_.empty()) {
        current_node_ptr = openset_.begin()->second;
        current_node_ptr->node_status_ = StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());

        if ((current_node_ptr->state_.head(2) - goal_node_ptr->state_.head(2)).norm() <= shot_distance_) {
            double rs_length = 0.0;
            if (AnalyticExpansions(current_node_ptr, goal_node_ptr, rs_length)) {
                terminal_node_ptr_ = goal_node_ptr;

                StateNode::Ptr grid_node_ptr = terminal_node_ptr_->parent_node_;
                while (grid_node_ptr != nullptr) {
                    grid_node_ptr = grid_node_ptr->parent_node_;
                    path_length_ = path_length_ + segment_length_;
                }
                path_length_ = path_length_ - segment_length_ + rs_length;

                std::cout << "ComputeH use time(ms): " << compute_h_time << std::endl;
                std::cout << "check collision use time(ms): " << check_collision_use_time << std::endl;
                std::cout << "GetNeighborNodes use time(ms): " << neighbor_time << std::endl;
                std::cout << "average time of check collision(ms): "
                          << check_collision_use_time / num_check_collision
                          << std::endl;
                ROS_INFO("\033[1;32m --> Time in Hybrid A star is %f ms, path length: %f  \033[0m\n",
                         search_used_time.End(), path_length_);

                check_collision_use_time = 0.0;
                num_check_collision = 0.0;
                return true;
            }
        }

        Timer timer_get_neighbor;
        GetNeighborNodes(current_node_ptr, neighbor_nodes_ptr);
        neighbor_time = neighbor_time + timer_get_neighbor.End();

        for (unsigned int i = 0; i < neighbor_nodes_ptr.size(); ++i) {
            neighbor_node_ptr = neighbor_nodes_ptr[i];

            Timer timer_compute_g;
            const double neighbor_edge_cost = ComputeG(current_node_ptr, neighbor_node_ptr);
            compute_g_time = compute_g_time + timer_get_neighbor.End();

            Timer timer_compute_h;
            const double current_h = ComputeH(current_node_ptr, goal_node_ptr) * tie_breaker_;
            compute_h_time = compute_h_time + timer_compute_h.End();

            const Vec3i &index = neighbor_node_ptr->grid_index_;
            if (state_node_map_[index.x()][index.y()][index.z()] == nullptr) {
                neighbor_node_ptr->g_cost_ = current_node_ptr->g_cost_ + neighbor_edge_cost;
                neighbor_node_ptr->parent_node_ = current_node_ptr;
                neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;
                neighbor_node_ptr->f_cost_ = neighbor_node_ptr->g_cost_ + current_h;
                openset_.insert(std::make_pair(neighbor_node_ptr->f_cost_, neighbor_node_ptr));
                state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_OPENSET) {
                double g_cost_temp = current_node_ptr->g_cost_ + neighbor_edge_cost;

                if (state_node_map_[index.x()][index.y()][index.z()]->g_cost_ > g_cost_temp) {
                    neighbor_node_ptr->g_cost_ = g_cost_temp;
                    neighbor_node_ptr->f_cost_ = g_cost_temp + current_h;
                    neighbor_node_ptr->parent_node_ = current_node_ptr;
                    neighbor_node_ptr->node_status_ = StateNode::IN_OPENSET;

                    /// TODO: This will cause a memory leak
                    //delete state_node_map_[index.x()][index.y()][index.z()];
                    state_node_map_[index.x()][index.y()][index.z()] = neighbor_node_ptr;
                } else {
                    delete neighbor_node_ptr;
                }
                continue;
            } else if (state_node_map_[index.x()][index.y()][index.z()]->node_status_ == StateNode::IN_CLOSESET) {
                delete neighbor_node_ptr;
                continue;
            }
        }

        count++;
        if (count > 50000) {
            ROS_WARN("Exceeded the number of iterations, the search failed");
            return false;
        }
    }

    return false;
}

VectorVec4d HybridAStar::GetSearchedTree() {
    VectorVec4d tree;
    Vec4d point_pair;

    visited_node_number_ = 0;
    for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
        for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
            for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                if (state_node_map_[i][j][k] == nullptr || state_node_map_[i][j][k]->parent_node_ == nullptr) {
                    continue;
                }

                const unsigned int number_states = state_node_map_[i][j][k]->intermediate_states_.size() - 1;
                for (unsigned int l = 0; l < number_states; ++l) {
                    point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[l].head(2);
                    point_pair.tail(2) = state_node_map_[i][j][k]->intermediate_states_[l + 1].head(2);

                    tree.emplace_back(point_pair);
                }

                point_pair.head(2) = state_node_map_[i][j][k]->intermediate_states_[0].head(2);
                point_pair.tail(2) = state_node_map_[i][j][k]->parent_node_->state_.head(2);
                tree.emplace_back(point_pair);
                visited_node_number_++;
            }
        }
    }

    return tree;
}

void HybridAStar::ReleaseMemory() {
    if (map_data_ != nullptr) {
        delete[] map_data_;
        map_data_ = nullptr;
    }

    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }

                delete[] state_node_map_[i][j];
                state_node_map_[i][j] = nullptr;
            }

            delete[] state_node_map_[i];
            state_node_map_[i] = nullptr;
        }

        delete[] state_node_map_;
        state_node_map_ = nullptr;
    }

    terminal_node_ptr_ = nullptr;
}

__attribute__((unused)) double HybridAStar::GetPathLength() const {
    return path_length_;
}

VectorVec3d HybridAStar::GetPath() const {
    VectorVec3d path;

    std::vector<StateNode::Ptr> temp_nodes;

    StateNode::Ptr state_grid_node_ptr = terminal_node_ptr_;
    while (state_grid_node_ptr != nullptr) {
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr = state_grid_node_ptr->parent_node_;
    }

    std::reverse(temp_nodes.begin(), temp_nodes.end());
    for (const auto &node: temp_nodes) {
        path.insert(path.end(), node->intermediate_states_.begin(),
                    node->intermediate_states_.end());
    }

    return path;
}

void HybridAStar::Reset() {
    if (state_node_map_) {
        for (int i = 0; i < STATE_GRID_SIZE_X_; ++i) {
            if (state_node_map_[i] == nullptr)
                continue;

            for (int j = 0; j < STATE_GRID_SIZE_Y_; ++j) {
                if (state_node_map_[i][j] == nullptr)
                    continue;

                for (int k = 0; k < STATE_GRID_SIZE_PHI_; ++k) {
                    if (state_node_map_[i][j][k] != nullptr) {
                        delete state_node_map_[i][j][k];
                        state_node_map_[i][j][k] = nullptr;
                    }
                }
            }
        }
    }

    path_length_ = 0.0;
    terminal_node_ptr_ = nullptr;
}

bool HybridAStar::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr, double &length) {
    VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(current_node_ptr->state_,
                                                        goal_node_ptr->state_,
                                                        move_step_size_, length);

    for (const auto &pose: rs_path_poses)
        if (BeyondBoundary(pose.head(2)) || !CheckCollision(pose.x(), pose.y(), pose.z())) {
            return false;
        };

    goal_node_ptr->intermediate_states_ = rs_path_poses;
    goal_node_ptr->parent_node_ = current_node_ptr;

    auto begin = goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}
