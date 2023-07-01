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

#include "hybrid_a_star/hybrid_a_star_flow.h"

#include "3rd/backward.hpp"

#include <ros/ros.h>

namespace backward {
backward::SignalHandling sh;
}

HybridAStarFlow * kinodynamic_astar_flow_ptr;
// ros::NodeHandle * node_handle_ptr;

int main(int argc, char **argv) {
    ros::init(argc, argv, "run_hybrid_astar");
    ros::NodeHandle node_handle("~");

    // node_handle_ptr = &node_handle;

    manual = node_handle.param("manual", false);
    
    // 初始化vehicel参数
    // 创建HybridAStar对象
    // 创建订阅者("/map","/initialpose","/move_base/goal")、发布者("searched_path","searched_tree","vehicle_path")
    HybridAStarFlow kinodynamic_astar_flow(node_handle);

    kinodynamic_astar_flow_ptr = &kinodynamic_astar_flow;

    ros::Rate rate(10);

    while (ros::ok()) {
        kinodynamic_astar_flow.Run();

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}