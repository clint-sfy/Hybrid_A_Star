/*
 * @Descripttion: sfy_code
 * @version: 
 * @Author: Fengyuan Shen
 * @Date: 2024-06-29 12:35:17
 * @LastEditors: Fengyuan Shen
 * @LastEditTime: 2024-06-29 13:03:49
 */
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

#include "hybrid_a_star/init_pose_subscriber.h"

// 构造函数，用于初始化节点句柄、订阅的主题名称和缓冲区大小
InitPoseSubscriber2D::InitPoseSubscriber2D(ros::NodeHandle &nh,
                                           const std::string &topic_name,
                                           size_t buff_size) {
    // 使用节点句柄订阅指定主题，缓冲区大小，回调函数为MessageCallBack，当前对象指针this
    subscriber_ = nh.subscribe(
            topic_name, buff_size, &InitPoseSubscriber2D::MessageCallBack, this
    );
}

// 定义一个函数，用于接收初始姿态信息
void InitPoseSubscriber2D::MessageCallBack(
        const geometry_msgs::PoseWithCovarianceStampedPtr &init_pose_ptr
) {
    // 锁定缓冲区
    buff_mutex_.lock();
    // 将初始姿态信息添加到缓冲区
    init_poses_.emplace_back(init_pose_ptr);
    // 解锁缓冲区
    buff_mutex_.unlock();
}

// 定义一个名为InitPoseSubscriber2D的类
void InitPoseSubscriber2D::ParseData(
        std::deque<geometry_msgs::PoseWithCovarianceStampedPtr> &pose_data_buff
) {
    // 锁定缓冲区
    buff_mutex_.lock();
    // 如果初始姿态列表不为空
    if (!init_poses_.empty()) {
        // 将初始姿态列表中的姿态插入到姿态数据缓冲区中
        pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(), init_poses_.end());
        // 清空初始姿态列表
        init_poses_.clear();
    }
    // 解锁缓冲区
    buff_mutex_.unlock();
}