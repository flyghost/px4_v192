/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * @file position_estimator_inav_params.c
 *
 * @author Anton Babushkin <rk3dov@gmail.com>
 *
 * Parameters definition for position_estimator_inav
 */

#pragma once

#include <parameters/param.h>

struct position_estimator_inav_params {
	float w_z_baro;			//权重  z轴  气压计位置   0.5
	float w_z_gps_p;		//权重  z轴  GPS位置     0.005f
	float w_z_gps_v;		//权重  z轴  GPS速度     0.0f
	float w_z_vision_p;		//权重  z轴  视觉位置     5.0f
	float w_z_lidar;		//权重  z轴  激光雷达位置  3.0f

	float w_xy_gps_p;		//权重  xy轴  GPS位置  1.0f
	float w_xy_gps_v;		//权重  xy轴  GPS速度  2.0f
	float w_xy_vision_p;		//权重  xy轴  视觉位置  7.0f
	float w_xy_vision_v;		//权重  xy轴  视觉速度  0.0f

	float w_mocap_p;		//权重        MOCAP系统  10.0f
	float w_xy_flow;		//权重  xy轴  光流位置      0.8f
	float w_xy_res_v;		//权重  xy轴  用于重置速度   0.5f
	float w_gps_flow;		//权重  xy轴  当光流可用时，将GPS权重乘以该权重  0.1f
	float w_acc_bias;		//权重        加速度计偏差估计  0.05f

	float flow_k;			//光流比例因子    1.35f   0~10
	float flow_q_min;		//最小可接受光流质量 0.3f  0~1
	float lidar_err;		//新表面声纳最大误差  0.2f
	float land_t;			//地面探测器时间  3.0f   如果在这段时间内没有在低油门下发生高度变化，车辆假定着陆
	float land_disp;		//地面探测器高度扩散阈值 0.7f
	float land_thr;			//地面探测器油门阈值 0.2f
	int32_t no_vision;		//禁用视觉输入  0
	float delay_gps;		//GPS延时补偿 0.2f

	float flow_module_offset_x;	//X方向的光流模块偏移（旋转中心）  0.0f
	float flow_module_offset_y;	//Y方向的光流模块偏移（旋转中心）  0.0f
	int32_t disable_mocap;		//如果使用GPS，则设置为0
	int32_t enable_lidar_alt_est;	//用于高度估计的激光雷达 0
	float lidar_calibration_offset;	//激光雷达校准偏差 0.0f
	int32_t att_ext_hdg_m;		//外部航向使用模式（从运动捕捉/视觉）0
};

struct position_estimator_inav_param_handles {
	param_t w_z_baro;	//权重  z轴  气压计位置   0.5
	param_t w_z_gps_p;	//权重  z轴  GPS位置     0.005f
	param_t w_z_gps_v;	//权重  z轴  GPS速度     0.0f
	param_t w_z_vision_p;	//权重  z轴  视觉位置     5.0f
	param_t w_z_lidar;	//权重  z轴  激光雷达位置  3.0f

	param_t w_xy_gps_p;	//权重  xy轴  GPS位置  1.0f
	param_t w_xy_gps_v;	//权重  xy轴  GPS速度  2.0f
	param_t w_xy_vision_p;	//权重  xy轴  视觉位置  7.0f
	param_t w_xy_vision_v;	//权重  xy轴  视觉速度  0.0f

	param_t w_mocap_p;	//权重        MOCAP系统  10.0f
	param_t w_xy_flow;	//权重  xy轴  光流位置      0.8f
	param_t w_xy_res_v;	//权重  xy轴  用于重置速度   0.5f
	param_t w_gps_flow;	//权重  xy轴  当光流可用时，将GPS权重乘以该权重  0.1f
	param_t w_acc_bias;	//权重        加速度计偏差估计  0.05f

	param_t flow_k;		//光流比例因子    1.35f   0~10
	param_t flow_q_min;	//最小可接受光流质量 0.3f  0~1
	param_t lidar_err;	//新表面声纳最大误差  0.2f
	param_t land_t;		//地面探测器时间  3.0f   如果在这段时间内没有在低油门下发生高度变化，车辆假定着陆
	param_t land_disp;	//地面探测器高度扩散阈值 0.7f
	param_t land_thr;	//地面探测器油门阈值 0.2f
	param_t no_vision;	//禁用视觉输入  0
	param_t delay_gps;	//GPS延时补偿 0.2f

	param_t flow_module_offset_x;		//X方向的光流模块偏移（旋转中心）  0.0f
	param_t flow_module_offset_y;		//Y方向的光流模块偏移（旋转中心）  0.0f
	param_t disable_mocap;			//如果使用GPS，则设置为0
	param_t enable_lidar_alt_est;		//用于高度估计的激光雷达 0
	param_t lidar_calibration_offset;	//激光雷达校准偏差 0.0f
	param_t att_ext_hdg_m;			//外部航向使用模式（从运动捕捉/视觉）0
};

#define CBRK_NO_VISION_KEY	328754

/**
 * Initialize all parameter handles and values
 *
 */
int inav_parameters_init(struct position_estimator_inav_param_handles *h);

/**
 * Update all parameters
 *
 */
int inav_parameters_update(const struct position_estimator_inav_param_handles *h,
			   struct position_estimator_inav_params *p);
