/****************************************************************************
 *
 *   Copyright (c) 2013-2018 PX4 Development Team. All rights reserved.
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

/**
 * @file position_estimator_inav_main.c
 * Model-identification based position estimator for multirotors
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Nuno Marques <n.marques21@hotmail.com>
 * @author Christoph Tobler <toblech@student.ethz.ch>
 */
#include <px4_posix.h>
#include <px4_tasks.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <fcntl.h>
#include <string.h>
#include <px4_config.h>
#include <math.h>
#include <float.h>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/vehicle_air_data.h>
#include <poll.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <lib/ecl/geo/geo.h>
#include <drivers/drv_hrt.h>
#include <platforms/px4_defines.h>

#include <terrain_estimation/terrain_estimator.h>
#include "position_estimator_inav_params.h"
#include "inertial_filter.h"

#define MIN_VALID_W 0.00001f			// 最小的权重值
#define PUB_INTERVAL 10000			// limit publish rate to 100 Hz  将发布速率限制为100 Hz
#define EST_BUF_SIZE 250000 / PUB_INTERVAL	// buffer size is 0.5s           缓冲区大小为0.5s
#define MAX_WAIT_FOR_BARO_SAMPLE 3000000 	// wait 3 secs for the baro to respond  等待3秒钟，等待BARO响应。

static bool thread_should_exit = false;		/**< Deamon exit flag */    // 守护线程退出标志
static bool thread_running = false;		/**< Deamon status flag */  // 守护线程状态标志
static int position_estimator_inav_task;	/**< Handle of deamon task / thread */  // 守护任务/线程的处理
static bool inav_verbose_mode = false;

static const hrt_abstime vision_topic_timeout = 500000;	// Vision topic timeout = 0.5s
static const hrt_abstime mocap_topic_timeout = 500000;		// Mocap topic timeout = 0.5s
static const hrt_abstime gps_topic_timeout = 500000;		// GPS topic timeout = 0.5s
static const hrt_abstime flow_topic_timeout = 1000000;	// optical flow topic timeout = 1s
static const hrt_abstime lidar_timeout = 150000;	// lidar timeout = 150ms
static const hrt_abstime lidar_valid_timeout = 1000000;	// estimate lidar distance during this time after lidar loss
static const unsigned updates_counter_len = 1000000;
static const float max_flow = 1.0f;	// max flow value that can be used, rad/s

extern "C" __EXPORT int position_estimator_inav_main(int argc, char *argv[]);

int position_estimator_inav_thread_main(int argc, char *argv[]);

static void usage(const char *reason);

static inline int min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

static inline int max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

/**
 * Print the correct usage.
 */
static void usage(const char *reason)
{
	if (reason && *reason) {
		PX4_INFO("%s", reason);
	}

	PX4_INFO("usage: position_estimator_inav {start|stop|status} [-v]\n");
}

/**
 * The position_estimator_inav_thread only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int position_estimator_inav_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return -1;
	}

	if (!strcmp(argv[1], "start")) {
		if (thread_running) {
			warnx("already running");
			/* this is not an error */
			return 0;
		}

		inav_verbose_mode = false;

		if ((argc > 2) && (!strcmp(argv[2], "-v"))) {
			inav_verbose_mode = true;
		}

		thread_should_exit = false;
		position_estimator_inav_task = px4_task_spawn_cmd("position_estimator_inav",
					       SCHED_DEFAULT, SCHED_PRIORITY_MAX - 5, 4600,
					       position_estimator_inav_thread_main,
					       (argv && argc > 2) ? (char *const *) &argv[2] : (char *const *) nullptr);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (thread_running) {
			warnx("stop");
			thread_should_exit = true;

		} else {
			warnx("not started");
		}

		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("is running");

		} else {
			warnx("not started");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

#ifdef INAV_DEBUG
static void write_debug_log(const char *msg, float dt, float x_est[2], float y_est[2], float z_est[2],
			    float x_est_prev[2], float y_est_prev[2], float z_est_prev[2],
			    float acc[3], float corr_gps[3][2], float w_xy_gps_p, float w_xy_gps_v, float corr_mocap[3][1], float w_mocap_p,
			    float corr_vision[3][2], float w_xy_vision_p, float w_z_vision_p, float w_xy_vision_v)
{
	FILE *f = fopen(PX4_STORAGEDIR"/inav.log", "a");

	if (f) {
		char *s = malloc(256);
		unsigned n = snprintf(s, 256,
				      "%llu %s\n\tdt=%.5f x_est=[%.5f %.5f] y_est=[%.5f %.5f] z_est=[%.5f %.5f] x_est_prev=[%.5f %.5f] y_est_prev=[%.5f %.5f] z_est_prev=[%.5f %.5f]\n",
				      (unsigned long long)hrt_absolute_time(), msg, (double)dt,
				      (double)x_est[0], (double)x_est[1], (double)y_est[0], (double)y_est[1], (double)z_est[0], (double)z_est[1],
				      (double)x_est_prev[0], (double)x_est_prev[1], (double)y_est_prev[0], (double)y_est_prev[1], (double)z_est_prev[0],
				      (double)z_est_prev[1]);
		fwrite(s, 1, n, f);
		n = snprintf(s, 256,
			     "\tacc=[%.5f %.5f %.5f] gps_pos_corr=[%.5f %.5f %.5f] gps_vel_corr=[%.5f %.5f %.5f] w_xy_gps_p=%.5f w_xy_gps_v=%.5f mocap_pos_corr=[%.5f %.5f %.5f] w_mocap_p=%.5f\n",
			     (double)acc[0], (double)acc[1], (double)acc[2],
			     (double)corr_gps[0][0], (double)corr_gps[1][0], (double)corr_gps[2][0], (double)corr_gps[0][1], (double)corr_gps[1][1],
			     (double)corr_gps[2][1],
			     (double)w_xy_gps_p, (double)w_xy_gps_v, (double)corr_mocap[0][0], (double)corr_mocap[1][0], (double)corr_mocap[2][0],
			     (double)w_mocap_p);
		fwrite(s, 1, n, f);
		n = snprintf(s, 256,
			     "\tvision_pos_corr=[%.5f %.5f %.5f] vision_vel_corr=[%.5f %.5f %.5f] w_xy_vision_p=%.5f w_z_vision_p=%.5f w_xy_vision_v=%.5f\n",
			     (double)corr_vision[0][0], (double)corr_vision[1][0], (double)corr_vision[2][0], (double)corr_vision[0][1],
			     (double)corr_vision[1][1], (double)corr_vision[2][1],
			     (double)w_xy_vision_p, (double)w_z_vision_p, (double)w_xy_vision_v);
		fwrite(s, 1, n, f);
		free(s);
	}

	fsync(fileno(f));
	fclose(f);
}
#else
#define write_debug_log(...)
#endif

/****************************************************************************
 * main
 ****************************************************************************/
int position_estimator_inav_thread_main(int argc, char *argv[])
{
	orb_advert_t mavlink_log_pub = nullptr;

	float x_est[2] = { 0.0f, 0.0f };	// pos, vel   //N系 x轴  位置, 速度
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel   //N系 y轴  位置, 速度
	float z_est[2] = { 0.0f, 0.0f };	// pos, vel   //N系 z轴  高度, 速度  最终程序所估计的高度

	float est_buf[EST_BUF_SIZE][3][2];	// 估算位置缓冲器
	float R_buf[EST_BUF_SIZE][3][3];	// 旋转矩阵缓冲器
	float R_gps[3][3];			// GPS校正力矩的旋转矩阵 // rotation matrix for GPS correction moment
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	int buf_ptr = 0;

	static const float min_eph_epv = 2.0f;	// min EPH/EPV, used for weight calculation  // 最小EPH/EPV，用于权重计算
	static const float max_eph_epv = 20.0f;	// max EPH/EPV acceptable for estimation

	float eph = max_eph_epv;   // 水平因子，表示水平的准确度
	float epv = 1.0f;

	float eph_flow = 1.0f;    // 因子越小，代表当前获得的水平信息准确度越高

	float eph_vision = 0.2f;
	float epv_vision = 0.2f;

	float eph_mocap = 0.05f;
	float epv_mocap = 0.05f;

	float x_est_prev[2], y_est_prev[2], z_est_prev[2];
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));

	int baro_init_cnt = 0;			//气压计校准时，循环采样的自加值
	int baro_init_num = 200;		//气压计校准时，计算零点偏移，需要采样的值的个数=200
	float baro_offset = 0.0f;		// baro offset for reference altitude, initialized on start, then adjusted

	hrt_abstime accel_timestamp = 0;	//两个时间戳相加
	hrt_abstime baro_timestamp = 0;		//气压计时间戳

	bool ref_inited = false;
	hrt_abstime ref_init_start = 0;
	const hrt_abstime ref_init_delay = 1000000;	// wait for 1s after 3D fix
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));

	uint16_t accel_updates = 0;
	uint16_t baro_updates = 0;
	uint16_t gps_updates = 0;
	uint16_t attitude_updates = 0;
	uint16_t flow_updates = 0;
	uint16_t vision_updates = 0;
	uint16_t mocap_updates = 0;

	hrt_abstime updates_counter_start = hrt_absolute_time();
	hrt_abstime pub_last = hrt_absolute_time();

	hrt_abstime t_prev = 0;

	/* store error when sensor updates, but correct on each time step to avoid jumps in estimated value */
	// 传感器更新时存储错误，但在每个时间步骤中更正，以避免估计值跳变。
	float acc[] = { 0.0f, 0.0f, 0.0f };		// N系   xyz轴加速度    准确加速度数据
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// B系   xyz轴加速度偏移量
	float corr_baro = 0.0f;				// 气压计动态误差
	float corr_gps[3][2] = {	// N系   GPS动态测量中产生的误差
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
	float w_gps_xy = 1.0f;
	float w_gps_z = 1.0f;

	float corr_vision[3][2] = {     // N系   视觉动态误差
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};

	float corr_mocap[3][1] = {      // N系   Mocap动态误差
		{ 0.0f },		// N (pos)
		{ 0.0f },		// E (pos)
		{ 0.0f },		// D (pos)
	};
	const int mocap_heading = 2;

	float dist_ground = 0.0f;	// 激光雷达高度估计的变量   //variables for lidar altitude estimation
	float corr_lidar = 0.0f;	// 激光雷达矫正系数
	float lidar_offset = 0.0f;	// 激光雷达偏移
	int lidar_offset_count = 0;
	bool lidar_first = true;	// 第一次激光雷达估计高度，，用于激光校准
	bool use_lidar = false;		// 如果激光雷达有效并且启用了激光雷达高度估计，则使用激光雷达。
	bool use_lidar_prev = false;	// 前一次是否使用激光

	float corr_flow[] = { 0.0f, 0.0f };	// N E
	float w_flow = 0.0f;

	hrt_abstime lidar_time = 0;		// 上次激光雷达测量时间（未过滤）// time of last lidar measurement (not filtered)
	hrt_abstime lidar_valid_time = 0;	// 用于校正（滤波）的最后一次激光雷达测量时间// time of last lidar measurement used for correction (filtered)

	int n_flow = 0;
	float gyro_offset_filtered[] = { 0.0f, 0.0f, 0.0f };	// 陀螺仪偏移 = 光流陀螺仪 - 姿态陀螺仪
	float flow_gyrospeed[] = { 0.0f, 0.0f, 0.0f };		// 光流陀螺仪原始值
	float flow_gyrospeed_filtered[] = { 0.0f, 0.0f, 0.0f };	// 光流陀螺仪经过滤波后的值
	float att_gyrospeed_filtered[] = { 0.0f, 0.0f, 0.0f };	// 姿态陀螺仪经过滤波后的值
	float yaw_comp[] = { 0.0f, 0.0f };
	hrt_abstime flow_time = 0;
	float flow_min_dist = 0.2f;				// 光流距离地面最小距离 = 20cm

	bool gps_valid = false;			// GPS is valid
	bool lidar_valid = false;		// lidar is valid
	bool flow_valid = false;		// flow is valid
	bool flow_accurate = false;		// flow should be accurate (this flag not updated if flow_valid == false)
	bool vision_xy_valid = false;		// vision XY is valid
	bool vision_z_valid = false;		// vision Z is valid
	bool vision_vxy_valid = false;		// vision VXY is valid
	bool vision_vz_valid = false;		// vision VZ is valid
	bool mocap_xy_valid = false;		// mocap XY is valid
	bool mocap_z_valid = false;		// mocap Z is valid

	/* set pose/velocity as invalid if standard deviation is bigger than max_std_dev */
	/* TODO: the user should be allowed to set these values by a parameter */
	static constexpr float ep_max_std_dev = 100.0f;	// position estimation max std deviation
	static constexpr float ev_max_std_dev = 100.0f;	// velocity estimation max std deviation

	/* declare and safely initialize all structs */
	struct actuator_controls_s actuator;
	memset(&actuator, 0, sizeof(actuator));
	struct actuator_armed_s armed;
	memset(&armed, 0, sizeof(armed));
	struct sensor_combined_s sensor;
	memset(&sensor, 0, sizeof(sensor));
	struct vehicle_gps_position_s gps;
	memset(&gps, 0, sizeof(gps));
	struct vehicle_attitude_s att;
	memset(&att, 0, sizeof(att));
	struct vehicle_local_position_s pos;
	memset(&pos, 0, sizeof(pos));
	struct optical_flow_s flow;
	memset(&flow, 0, sizeof(flow));
	struct vehicle_odometry_s visual_odom;
	memset(&visual_odom, 0, sizeof(visual_odom));
	struct vehicle_odometry_s mocap;
	memset(&mocap, 0, sizeof(mocap));
	struct vehicle_global_position_s global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	struct distance_sensor_s lidar;
	memset(&lidar, 0, sizeof(lidar));
	struct vehicle_rates_setpoint_s rates_setpoint;
	memset(&rates_setpoint, 0, sizeof(rates_setpoint));
	struct vehicle_air_data_s airdata;
	memset(&airdata, 0, sizeof(vehicle_air_data_s));

	/* subscribe */
	int parameter_update_sub = orb_subscribe(ORB_ID(parameter_update));
	int actuator_sub = orb_subscribe(ORB_ID_VEHICLE_ATTITUDE_CONTROLS);
	int armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	int sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	int vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	int optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
	int vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	int visual_odom_sub = orb_subscribe(ORB_ID(vehicle_visual_odometry));
	int mocap_position_sub = orb_subscribe(ORB_ID(vehicle_mocap_odometry));
	int vehicle_rate_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	int vehicle_air_data_sub = orb_subscribe(ORB_ID(vehicle_air_data));
	// because we can have several distance sensor instances with different orientations
	int distance_sensor_subs[ORB_MULTI_MAX_INSTANCES];

	for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {
		distance_sensor_subs[i] = orb_subscribe_multi(ORB_ID(distance_sensor), i);
	}

	/* advertise */
	orb_advert_t vehicle_local_position_pub = orb_advertise(ORB_ID(vehicle_local_position), &pos);// 本地位置
	orb_advert_t vehicle_global_position_pub = nullptr;    // 全球位置

	struct position_estimator_inav_params params;
	memset(&params, 0, sizeof(params));
	struct position_estimator_inav_param_handles pos_inav_param_handles;
	/* initialize parameter handles */
	inav_parameters_init(&pos_inav_param_handles);

	/* first parameters read at start up */
	struct parameter_update_s param_update;
	orb_copy(ORB_ID(parameter_update), parameter_update_sub,
		 &param_update); /* read from param topic to clear updated flag */
	/* first parameters update */
	inav_parameters_update(&pos_inav_param_handles, &params);

	px4_pollfd_struct_t fds_init[1] = {};
	fds_init[0].fd = sensor_combined_sub;
	fds_init[0].events = POLLIN;

	/* wait for initial baro value */
	bool wait_baro = true;
	TerrainEstimator terrain_estimator;

	thread_running = true;
	hrt_abstime baro_wait_for_sample_time = hrt_absolute_time(); // 获取当前绝对时间，用于气压计校准轮循的超时判断

	while (wait_baro && !thread_should_exit) {
		int ret = px4_poll(&fds_init[0], 1, 1000);

		// 轮循错误
		if (ret < 0) {
			/* poll error */
			mavlink_log_info(&mavlink_log_pub, "[inav] poll error on init");

		}
		// 超时
		else if (hrt_absolute_time() - baro_wait_for_sample_time > MAX_WAIT_FOR_BARO_SAMPLE) {
			wait_baro = false;
			mavlink_log_info(&mavlink_log_pub, "[inav] timed out waiting for a baro sample");

		} else if (ret > 0) {
			if (fds_init[0].revents & POLLIN) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				bool baro_updated = false;  // 检查气压计是否更新
				orb_check(vehicle_air_data_sub, &baro_updated);

				// 更新则拷贝
				if (baro_updated) {
					orb_copy(ORB_ID(vehicle_air_data), vehicle_air_data_sub, &airdata);

					// 气压计校准值未计算完成、当前气压计时间戳和上一次时间戳不一致（说明气压计值有效）
					if (wait_baro && airdata.timestamp != baro_timestamp) {

						baro_timestamp = airdata.timestamp;
						baro_wait_for_sample_time = hrt_absolute_time();

						/* mean calculation over several measurements */
						// 多次测量的平均值计算
						if (baro_init_cnt < baro_init_num) {
							if (PX4_ISFINITE(airdata.baro_alt_meter)) {
								baro_offset += airdata.baro_alt_meter;
								baro_init_cnt++;
							}

						} else {
							wait_baro = false;
							baro_offset /= (float) baro_init_cnt;
							pos.z_valid = true;
							pos.v_z_valid = true;
						}
					}
				}


			}

		} else {
			PX4_WARN("INAV poll timeout");
		}
	}

	/* main loop */
	px4_pollfd_struct_t fds[1];
	fds[0].fd = vehicle_attitude_sub;
	fds[0].events = POLLIN;

	while (!thread_should_exit) {
		int ret = px4_poll(fds, 1, 20);  // 最大等待20 ms=50 Hz最小速率 // wait maximal 20 ms = 50 Hz minimum rate
		hrt_abstime t = hrt_absolute_time();

		if (ret < 0) {
			/* poll error */
			mavlink_log_info(&mavlink_log_pub, "[inav] poll error on init");
			continue;

		} else if (ret > 0) {
			/* act on attitude updates */

			/* vehicle attitude */
			// 车辆姿态
			orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub, &att);
			attitude_updates++;

			bool updated;

			/* parameter update */
			// 更新定义的变量值 （没有必要）
			orb_check(parameter_update_sub, &updated);

			if (updated) {
				struct parameter_update_s update;
				orb_copy(ORB_ID(parameter_update), parameter_update_sub, &update);
				inav_parameters_update(&pos_inav_param_handles, &params); // 如果更新了则更新
			}

			/* actuator */
			//执行机构  （没有用到）
			orb_check(actuator_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID_VEHICLE_ATTITUDE_CONTROLS, actuator_sub, &actuator);
			}

			/* armed */ //没有用到
			orb_check(armed_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(actuator_armed), armed_sub, &armed);
			}



			/*******************************START************************************/
			//     通过在最下面得到的acc_bias[],
			//     计算修正后的导航坐标系下的加速度acc[]，
			//     之后用此加速度进行一次、二次积分得到预计速度和位置
			/************************************************************************/
			/* sensor combined */
			orb_check(sensor_combined_sub, &updated);

			matrix::Dcmf R = matrix::Quatf(att.q);

			if (updated) {
				orb_copy(ORB_ID(sensor_combined), sensor_combined_sub, &sensor);

				if (sensor.timestamp + sensor.accelerometer_timestamp_relative != accel_timestamp) {
					/* correct accel bias */
					sensor.accelerometer_m_s2[0] -= acc_bias[0];  // B系 加速度矫正
					sensor.accelerometer_m_s2[1] -= acc_bias[1];
					sensor.accelerometer_m_s2[2] -= acc_bias[2];

					/* transform acceleration vector from body frame to NED frame */
					//加速度向量从机体坐标系转到NED坐标系
					for (int i = 0; i < 3; i++) {
						acc[i] = 0.0f;

						for (int j = 0; j < 3; j++) {
							acc[i] += R(i, j) * sensor.accelerometer_m_s2[j];  // B系 -> N系
						}
					}

					//修正后的导航坐标系下的加速度
					acc[2] += CONSTANTS_ONE_G; // 补偿重力加速度，z轴的方向是向下，所以是相加

					// 记录时间戳
					accel_timestamp = sensor.timestamp + sensor.accelerometer_timestamp_relative;
					accel_updates++;
				}

				// 由于airdata没有使用copy更新，所以仅能运行一次
				if (airdata.timestamp != baro_timestamp) {
					//高度差 = 起飞点高度 - 气压计当前高度 - z轴高度（负）
					//即 z轴实际高度-气压计测得高度+气压计的偏移量
					//这个高度差就是气压计应该校正的值
					corr_baro = baro_offset - airdata.baro_alt_meter - z_est[0]; // 计算气压计校正系数
					baro_timestamp = airdata.timestamp;
					baro_updates++;
				}
			}
			/*******************************END**************************************/



			/*******************************START************************************/
			//     激光雷达高度估计
			//     用于后续的光流使用
			//     地形估计器需要单独更新激光雷达
			/************************************************************************/
			/* lidar alt estimation
			 * update lidar separately, needed by terrain estimator */
			for (unsigned i = 0; i < ORB_MULTI_MAX_INSTANCES; i++) {

				orb_check(distance_sensor_subs[i], &updated);

				if (updated) {

					orb_copy(ORB_ID(distance_sensor), distance_sensor_subs[i], &lidar);

					//寻找传感器方向是面临向下旋转的
					if (lidar.orientation != distance_sensor_s::ROTATION_DOWNWARD_FACING) {
						updated = false;

					} else {
						lidar.current_distance += params.lidar_calibration_offset;
						break; // only the first valid distance sensor instance is used
					}
				}
			}

			// 找到向下的传感器
			if (updated) { //check if altitude estimation for lidar is enabled and new sensor data

				// 是否启用激光雷达高度估计 = 0， 激光距离要大于最小距离， 小于最大距离， 且cosx*cosy > 0.7
				if (params.enable_lidar_alt_est && lidar.current_distance > lidar.min_distance
				    && lidar.current_distance < lidar.max_distance
				    && (R(2, 2) > 0.7f)) {

					// 判断是否第一次运行这段代码。
					if (!use_lidar_prev && use_lidar) { // 前一刻未使用雷达 ，，且使用激光估计高度
						lidar_first = true;
					}

					use_lidar_prev = use_lidar;

					lidar_time = t;
					dist_ground = lidar.current_distance * R(2, 2); //vertical distance

					// 激光雷达零篇校准，只运行一次
					if (lidar_first) {
						lidar_first = false;
						lidar_offset = dist_ground + z_est[0];
						mavlink_log_info(&mavlink_log_pub, "[inav] LIDAR: new ground offset");
						warnx("[inav] LIDAR: new ground offset");
					}

					// 计算激光雷达校正系数
					corr_lidar = lidar_offset - dist_ground - z_est[0];

					// 判断校正系数是否过大，过大则需要重新计算偏移lidar_offset
					// 即便有误差，也不会过大，如果过大，肯定就是校准有问题，就对雷达进行校准
					if (fabsf(corr_lidar) > params.lidar_err) { //check for spike
						corr_lidar = 0;
						lidar_valid = false;
						lidar_offset_count++;

						// 如果连续的较大/较小测量值—>新地面偏移量—>钢筋
						// 重新计算偏移
						if (lidar_offset_count > 3) { //if consecutive bigger/smaller measurements -> new ground offset -> reinit
							lidar_first = true;
							lidar_offset_count = 0;
						}

					} else {
						corr_lidar = lidar_offset - dist_ground - z_est[0];
						lidar_valid = true;
						lidar_offset_count = 0;
						lidar_valid_time = t;
					}

				} else {
					lidar_valid = false;
				}
			}
			/*******************************END**************************************/



			/*******************************START************************************/
			//     光流
			//     需要配合激光雷达使用
			/************************************************************************/
			/* optical flow */
			orb_check(optical_flow_sub, &updated);

			if (updated && lidar_valid) {  // 光流数据更新并且激光有效
				orb_copy(ORB_ID(optical_flow), optical_flow_sub, &flow);

				flow_time = t;   //光流开始时间
				float flow_q = flow.quality / 255.0f;  //光流质量单位化
				float dist_bottom = lidar.current_distance;  //光流距离地面距离，激光雷达的距离

				// 激光距离要大于最小距离， 小于最大距离， 且cosx*cosy > 0.7
				// 光流所有的运算都要基于这个条件
				if (dist_bottom > flow_min_dist && flow_q > params.flow_q_min && R(2, 2) > 0.7f) {
					/* distance to surface */
					//float flow_dist = dist_bottom / PX4_R(att.R, 2, 2); //use this if using sonar
					// 距地面距离
					float flow_dist = dist_bottom; //use this if using lidar

					/* check if flow if too large for accurate measurements */
					/* calculate estimated velocity in body frame */
					// 检查流量是否过大，以便于准确测量
					// 计算机体坐标系中的速度
					float body_v_est[2] = { 0.0f, 0.0f };

					for (int i = 0; i < 2; i++) {
						// R的转置 × _est[x,y,z]
						// N系 -->B系
						body_v_est[i] = R(0, i) * x_est[1] + R(1, i) * y_est[1] + R(2, i) * z_est[1];
					}

					/* set this flag if flow should be accurate according to current velocity and attitude rate estimate */
					// 如果根据当前速度和姿态角速度估计光流应准确，则设置此标志
					// flow_accurate判断光流精度能否使用
					// xy轴机体速度/距离 = 机体角速度
					// 机体角速度-飞控测得的角速度 < 某个阈值，表明精度可用
					flow_accurate = fabsf(body_v_est[1] / flow_dist - att.rollspeed) < max_flow &&
							fabsf(body_v_est[0] / flow_dist + att.pitchspeed) < max_flow;

					/*calculate offset of flow-gyro using already calibrated gyro from autopilot*/
					// 用自动驾驶仪上已校准的陀螺仪计算光流陀螺仪的偏移量
					flow_gyrospeed[0] = flow.gyro_x_rate_integral / (float)flow.integration_timespan * 1000000.0f;
					flow_gyrospeed[1] = flow.gyro_y_rate_integral / (float)flow.integration_timespan * 1000000.0f;
					flow_gyrospeed[2] = flow.gyro_z_rate_integral / (float)flow.integration_timespan * 1000000.0f;

					//moving average
					//滑动均值滤波
					//这一段代码的最终目的为了求解陀螺仪偏移gyro_offset_filtered[]
					if (n_flow >= 100) {
						gyro_offset_filtered[0] = flow_gyrospeed_filtered[0] - att_gyrospeed_filtered[0];
						gyro_offset_filtered[1] = flow_gyrospeed_filtered[1] - att_gyrospeed_filtered[1];
						gyro_offset_filtered[2] = flow_gyrospeed_filtered[2] - att_gyrospeed_filtered[2];
						n_flow = 0;
						flow_gyrospeed_filtered[0] = 0.0f;
						flow_gyrospeed_filtered[1] = 0.0f;
						flow_gyrospeed_filtered[2] = 0.0f;
						att_gyrospeed_filtered[0] = 0.0f;
						att_gyrospeed_filtered[1] = 0.0f;
						att_gyrospeed_filtered[2] = 0.0f;

					} else {
						flow_gyrospeed_filtered[0] = (flow_gyrospeed[0] + n_flow * flow_gyrospeed_filtered[0]) / (n_flow + 1);
						flow_gyrospeed_filtered[1] = (flow_gyrospeed[1] + n_flow * flow_gyrospeed_filtered[1]) / (n_flow + 1);
						flow_gyrospeed_filtered[2] = (flow_gyrospeed[2] + n_flow * flow_gyrospeed_filtered[2]) / (n_flow + 1);
						att_gyrospeed_filtered[0] = (att.pitchspeed + n_flow * att_gyrospeed_filtered[0]) / (n_flow + 1);
						att_gyrospeed_filtered[1] = (att.rollspeed + n_flow * att_gyrospeed_filtered[1]) / (n_flow + 1);
						att_gyrospeed_filtered[2] = (att.yawspeed + n_flow * att_gyrospeed_filtered[2]) / (n_flow + 1);
						n_flow++;
					}


					/*yaw compensation (flow sensor is not in center of rotation) -> params in QGC*/
					// 偏航yaw补偿（光流传感器不在旋转中心）-->QGC中的参数
					yaw_comp[0] = - params.flow_module_offset_y * (flow_gyrospeed[2] - gyro_offset_filtered[2]);
					yaw_comp[1] = params.flow_module_offset_x * (flow_gyrospeed[2] - gyro_offset_filtered[2]);

					/*
					 * Convert raw flow from the optical_flow uORB topic (rad) to angular flow (rad/s)
					 * Note that the optical_flow uORB topic defines positive delta angles as produced by RH rotations
					 * around the correspdonding body axes.
					 */

					float flow_ang[2];

					/* check for vehicle rates setpoint - it below threshold -> dont subtract -> better hover */
					orb_check(vehicle_rate_sp_sub, &updated);

					if (updated) {
						orb_copy(ORB_ID(vehicle_rates_setpoint), vehicle_rate_sp_sub, &rates_setpoint);
					}

					float rate_threshold = 0.15f;

					/* calculate the angular flow rate produced by a negative velocity along the X body axis */
					// 计算沿X轴的负速度产生的角流量
					if (fabsf(rates_setpoint.pitch) < rate_threshold) {
						//warnx("[inav] test ohne comp");
						flow_ang[0] = (-flow.pixel_flow_y_integral / (float)flow.integration_timespan * 1000000.0f) *
							      params.flow_k;//for now the flow has to be scaled (to small)

					} else {
						//warnx("[inav] test mit comp");
						//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)
						flow_ang[0] = (-(flow.pixel_flow_y_integral - flow.gyro_y_rate_integral) / (float)flow.integration_timespan * 1000000.0f
							       + gyro_offset_filtered[0]) * params.flow_k;//for now the flow has to be scaled (to small)
					}

					/* calculate the angular flow rate produced by a negative velocity along the Y body axis */
					if (fabsf(rates_setpoint.roll) < rate_threshold) {
						flow_ang[1] = (flow.pixel_flow_x_integral / (float)flow.integration_timespan * 1000000.0f) *
							      params.flow_k;//for now the flow has to be scaled (to small)

					} else {
						//calculate flow [rad/s] and compensate for rotations (and offset of flow-gyro)
						flow_ang[1] = ((flow.pixel_flow_x_integral - flow.gyro_x_rate_integral) / (float)flow.integration_timespan * 1000000.0f
							       + gyro_offset_filtered[1]) * params.flow_k;//for now the flow has to be scaled (to small)
					}

					/* flow measurements vector */
					float flow_m[3];

					if (fabsf(rates_setpoint.yaw) < rate_threshold) {
						flow_m[0] = -flow_ang[0] * flow_dist;
						flow_m[1] = -flow_ang[1] * flow_dist;

					} else {
						flow_m[0] = -flow_ang[0] * flow_dist - yaw_comp[0] * params.flow_k;
						flow_m[1] = -flow_ang[1] * flow_dist - yaw_comp[1] * params.flow_k;
					}

					flow_m[2] = z_est[1];

					/* velocity in NED */
					float flow_v[2] = { 0.0f, 0.0f };

					/* project measurements vector to NED basis, skip Z component */
					for (int i = 0; i < 2; i++) {
						for (int j = 0; j < 3; j++) {
							flow_v[i] += R(i, j) * flow_m[j];
						}
					}

					/* velocity correction */
					corr_flow[0] = flow_v[0] - x_est[1];
					corr_flow[1] = flow_v[1] - y_est[1];
					/* adjust correction weight */
					float flow_q_weight = (flow_q - params.flow_q_min) / (1.0f - params.flow_q_min);
					w_flow = R(2, 2) * flow_q_weight / fmaxf(1.0f, flow_dist);


					/* if flow is not accurate, reduce weight for it */
					// TODO make this more fuzzy
					if (!flow_accurate) {
						w_flow *= 0.05f;
					}

					/* under ideal conditions, on 1m distance assume EPH = 10cm */
					eph_flow = 0.1f / w_flow;

					flow_valid = true;

				} else {
					w_flow = 0.0f;
					flow_valid = false;
				}

				flow_updates++;
			}
			/*******************************END**************************************/


			/* check no vision circuit breaker is set */
			if (params.no_vision != CBRK_NO_VISION_KEY) {
				/* vehicle visual odometry */
				orb_check(visual_odom_sub, &updated);

				if (updated) {
					orb_copy(ORB_ID(vehicle_visual_odometry), visual_odom_sub, &visual_odom);

					static float last_vision_x = 0.0f;
					static float last_vision_y = 0.0f;
					static float last_vision_z = 0.0f;

					vision_xy_valid = PX4_ISFINITE(visual_odom.x)
							  && (PX4_ISFINITE(visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_X_VARIANCE]) ? sqrtf(fmaxf(
									  visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_X_VARIANCE],
									  visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_Y_VARIANCE])) <= ep_max_std_dev : true);
					vision_z_valid = PX4_ISFINITE(visual_odom.z)
							 && (PX4_ISFINITE(visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_X_VARIANCE]) ?
							     visual_odom.pose_covariance[visual_odom.COVARIANCE_MATRIX_Z_VARIANCE] <= ep_max_std_dev : true);
					vision_vxy_valid = PX4_ISFINITE(visual_odom.vx)
							   && PX4_ISFINITE(visual_odom.velocity_covariance[visual_odom.COVARIANCE_MATRIX_VX_VARIANCE]) ? sqrtf(
								   fmaxf(visual_odom.velocity_covariance[visual_odom.COVARIANCE_MATRIX_VX_VARIANCE],
									 visual_odom.velocity_covariance[visual_odom.COVARIANCE_MATRIX_VY_VARIANCE])) <= ev_max_std_dev : true;
					vision_vz_valid = PX4_ISFINITE(visual_odom.vz)
							  && (PX4_ISFINITE(visual_odom.velocity_covariance[visual_odom.COVARIANCE_MATRIX_VX_VARIANCE]) ?
							      visual_odom.velocity_covariance[visual_odom.COVARIANCE_MATRIX_VZ_VARIANCE] <= ep_max_std_dev : true);

					/* reset position estimate on first vision update */
					if (vision_xy_valid) {
						x_est[0] = visual_odom.x;
						y_est[0] = visual_odom.y;

						last_vision_x = visual_odom.x;
						last_vision_y = visual_odom.y;

					} else {
						warnx("VISION XY estimate not valid");
						mavlink_log_info(&mavlink_log_pub, "[inav] VISION XY estimate not valid");

					}

					if (vision_vxy_valid) {
						x_est[1] = visual_odom.vx;
						y_est[1] = visual_odom.vy;

					} else {
						warnx("VISION VXY estimate not valid");
						mavlink_log_info(&mavlink_log_pub, "[inav] VISION VXY estimate not valid");

					}

					/* only reset the z estimate if the z weight parameter is not zero */
					if (params.w_z_vision_p > MIN_VALID_W) {
						if (vision_z_valid) {
							z_est[0] = visual_odom.z;

							last_vision_z = visual_odom.z;

						} else {
							warnx("VISION Z estimate not valid");
							mavlink_log_info(&mavlink_log_pub, "[inav] VISION Z estimate not valid");

						}

						if (vision_vz_valid) {
							z_est[1] = visual_odom.vz;

						} else {
							warnx("VISION VZ estimate not valid");
							mavlink_log_info(&mavlink_log_pub, "[inav] VISION VZ estimate not valid");

						}
					}

					/* calculate correction for position */
					if (vision_xy_valid) {
						corr_vision[0][0] = visual_odom.x - x_est[0];
						corr_vision[1][0] = visual_odom.y - y_est[0];
					}

					if (vision_z_valid) {
						corr_vision[2][0] = visual_odom.z - z_est[0];
					}

					static hrt_abstime last_vision_time = 0;

					float vision_dt = (visual_odom.timestamp - last_vision_time) / 1e6f;
					last_vision_time = visual_odom.timestamp;

					if (vision_vxy_valid) {
						/* calculate correction for XY velocity from external estimation */
						corr_vision[0][1] = visual_odom.vx - x_est[1];
						corr_vision[1][1] = visual_odom.vy - y_est[1];

					} else if (vision_dt > 0.000001f && vision_dt < 0.2f && vision_xy_valid) {
						visual_odom.vx = (visual_odom.x - last_vision_x) / vision_dt;
						visual_odom.vy = (visual_odom.y - last_vision_y) / vision_dt;

						last_vision_x = visual_odom.x;
						last_vision_y = visual_odom.y;

						/* calculate correction for XY velocity */
						corr_vision[0][1] = visual_odom.vx - x_est[1];
						corr_vision[1][1] = visual_odom.vy - y_est[1];

					} else {
						/* assume zero motion in XY plane */
						corr_vision[0][1] = 0.0f - x_est[1];
						corr_vision[1][1] = 0.0f - y_est[1];

					}

					if (vision_vz_valid) {
						/* calculate correction for Z velocity from external estimation */
						corr_vision[2][1] = visual_odom.vz - z_est[1];

					} else if (vision_dt > 0.000001f && vision_dt < 0.2f && vision_z_valid) {
						visual_odom.vz = (visual_odom.z - last_vision_z) / vision_dt;

						last_vision_z = visual_odom.z;

						/* calculate correction for Z velocity */
						corr_vision[2][1] = visual_odom.vz - z_est[1];

					} else {
						/* assume zero motion in Z plane */
						corr_vision[2][1] = 0.0f - z_est[1];
					}

					vision_updates++;
				}
			}

			/* vehicle mocap position */
			orb_check(mocap_position_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_mocap_odometry), mocap_position_sub, &mocap);

				mocap_xy_valid = PX4_ISFINITE(mocap.x)
						 && (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE]) ? sqrtf(fmaxf(
								 mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE],
								 mocap.pose_covariance[mocap.COVARIANCE_MATRIX_Y_VARIANCE])) <= ep_max_std_dev : true);
				mocap_z_valid = PX4_ISFINITE(mocap.z)
						&& (PX4_ISFINITE(mocap.pose_covariance[mocap.COVARIANCE_MATRIX_X_VARIANCE]) ?
						    mocap.pose_covariance[mocap.COVARIANCE_MATRIX_Z_VARIANCE] <= ep_max_std_dev : true);

				if (!params.disable_mocap) {
					/* reset position estimate on first mocap update */
					if (mocap_xy_valid) {
						x_est[0] = mocap.x;
						y_est[0] = mocap.y;
					}

					if (mocap_z_valid) {
						z_est[0] = mocap.z;
					}

					if (!mocap_xy_valid || !mocap_z_valid) {
						warnx("MOCAP data not valid");
						mavlink_log_info(&mavlink_log_pub, "[inav] MOCAP data not valid");;

					}

					/* calculate correction for position */
					if (mocap_xy_valid) {
						corr_mocap[0][0] = mocap.x - x_est[0];
						corr_mocap[1][0] = mocap.y - y_est[0];
					}

					if (mocap_z_valid) {
						corr_mocap[2][0] = mocap.z - z_est[0];
					}

					mocap_updates++;
				}
			}

			/* vehicle GPS position */
			orb_check(vehicle_gps_position_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(vehicle_gps_position), vehicle_gps_position_sub, &gps);

				bool reset_est = false;

				/* hysteresis for GPS quality */
				// 判断是否有效，上次无效，这次判断是否有效，上次有效，判断本次是否无效
				if (gps_valid) {
					if (gps.eph > max_eph_epv || gps.epv > max_eph_epv || gps.fix_type < 3) {
						gps_valid = false;
						mavlink_log_info(&mavlink_log_pub, "[inav] GPS signal lost");
						warnx("[inav] GPS signal lost");
					}

				} else {
					if (gps.eph < max_eph_epv * 0.7f && gps.epv < max_eph_epv * 0.7f && gps.fix_type >= 3) {
						gps_valid = true;
						reset_est = true;
						mavlink_log_info(&mavlink_log_pub, "[inav] GPS signal found");
						warnx("[inav] GPS signal found");
					}
				}

				if (gps_valid) {
					double lat = gps.lat * 1e-7;
					double lon = gps.lon * 1e-7;
					float alt = gps.alt * 1e-3;

					/* initialize reference position if needed */
					// gps未初始化，则初始化GPS
					if (!ref_inited) {
						if (ref_init_start == 0) {
							ref_init_start = t;

						} else if (t > ref_init_start + ref_init_delay) {
							ref_inited = true;

							/* set position estimate to (0, 0, 0), use GPS velocity for XY */
							x_est[0] = 0.0f;
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = 0.0f;
							y_est[1] = gps.vel_e_m_s;

							pos.ref_lat = lat;
							pos.ref_lon = lon;
							pos.ref_alt = alt + z_est[0];
							pos.ref_timestamp = t;

							/* initialize projection */
							map_projection_init(&ref, lat, lon);
							// XXX replace this print
							warnx("init ref: lat=%.7f, lon=%.7f, alt=%8.4f", (double)lat, (double)lon, (double)alt);
							mavlink_log_info(&mavlink_log_pub, "[inav] init ref: %.7f, %.7f, %8.4f", (double)lat, (double)lon, (double)alt);
						}
					}

					if (ref_inited) {
						/* project GPS lat lon to plane */
						float gps_proj[2];
						map_projection_project(&ref, lat, lon, &(gps_proj[0]), &(gps_proj[1]));

						/* reset position estimate when GPS becomes good */
						if (reset_est) {
							x_est[0] = gps_proj[0];
							x_est[1] = gps.vel_n_m_s;
							y_est[0] = gps_proj[1];
							y_est[1] = gps.vel_e_m_s;
						}

						/* calculate index of estimated values in buffer */
						// GPS计算是有延时的，最大才5HZ，所以计算的动态误差不是弥补当前，而是弥补200ms以前
						int est_i = buf_ptr - 1 - min(EST_BUF_SIZE - 1, max(0, (int)(params.delay_gps * 1000000.0f / PUB_INTERVAL)));

						if (est_i < 0) {
							est_i += EST_BUF_SIZE;
						}

						/* calculate correction for position */
						corr_gps[0][0] = gps_proj[0] - est_buf[est_i][0][0];
						corr_gps[1][0] = gps_proj[1] - est_buf[est_i][1][0];
						corr_gps[2][0] = pos.ref_alt - alt - est_buf[est_i][2][0];

						/* calculate correction for velocity */
						if (gps.vel_ned_valid) {
							corr_gps[0][1] = gps.vel_n_m_s - est_buf[est_i][0][1];
							corr_gps[1][1] = gps.vel_e_m_s - est_buf[est_i][1][1];
							corr_gps[2][1] = gps.vel_d_m_s - est_buf[est_i][2][1];

						} else {
							corr_gps[0][1] = 0.0f;
							corr_gps[1][1] = 0.0f;
							corr_gps[2][1] = 0.0f;
						}

						/* save rotation matrix at this moment */
						memcpy(R_gps, R_buf[est_i], sizeof(R_gps));
						//定位因子越小越好，假设gps.eph足够好，w_gps_xy为1，反之，w_gps_xy则会衰减
						w_gps_xy = min_eph_epv / fmaxf(min_eph_epv, gps.eph);
						w_gps_z = min_eph_epv / fmaxf(min_eph_epv, gps.epv);
					}

				} else {
					/* no GPS lock */
					memset(corr_gps, 0, sizeof(corr_gps));
					ref_init_start = 0;
				}

				gps_updates++;
			}
		}

		/*
		* 当代码运行的这里的时候，说明我们已经获取了所有传感器的动态误差corr
		*/

		matrix::Dcm<float> R = matrix::Quatf(att.q);

		/*
		* 融合前保证传感器数据是有效的，gps和mocap传感器是很容易超时的，实际中我们可以适当把参数调大一点
		*/
		/* check for timeout on FLOW topic */
		if ((flow_valid || lidar_valid) && t > (flow_time + flow_topic_timeout)) {  // 1s
			flow_valid = false;
			warnx("FLOW timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] FLOW timeout");
		}

		/* check for timeout on GPS topic */
		if (gps_valid && (t > (gps.timestamp + gps_topic_timeout))) {
			gps_valid = false;
			warnx("GPS timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] GPS timeout");
		}

		/* check for timeout on vision topic */
		if ((vision_xy_valid || vision_z_valid) && (t > (visual_odom.timestamp + vision_topic_timeout))) {
			vision_xy_valid = false;
			vision_z_valid = false;
			warnx("VISION timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] VISION timeout");
		}

		/* check for timeout on mocap topic */
		if ((mocap_xy_valid || mocap_z_valid) && (t > (mocap.timestamp + mocap_topic_timeout))) {
			mocap_xy_valid = false;
			mocap_z_valid = false;
			warnx("MOCAP timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] MOCAP timeout");
		}

		/* check for lidar measurement timeout */
		if (lidar_valid && (t > (lidar_time + lidar_timeout))) {
			lidar_valid = false;
			warnx("LIDAR timeout");
			mavlink_log_info(&mavlink_log_pub, "[inav] LIDAR timeout");
		}

		float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
		t_prev = t;

		/* increase EPH/EPV on each step */
		if (eph < 0.000001f) { //get case where eph is 0 -> would stay 0
			eph = 0.001;
		}

		if (eph < max_eph_epv) {
			eph *= 1.0f + dt;
		}

		if (epv < 0.000001f) { //get case where epv is 0 -> would stay 0
			epv = 0.001;
		}

		if (epv < max_eph_epv) {
			epv += 0.005f * dt;	// add 1m to EPV each 200s (baro drift)
		}


		/************************************************************************/
		//      根据传感器数据是否有效，判断使用哪一个传感器进行加速度计偏差融合
		/************************************************************************/
		/* use GPS if it's valid and reference position initialized */
		bool use_gps_xy = ref_inited && gps_valid && params.w_xy_gps_p > MIN_VALID_W; // 如果有效且参考位置已初始化，则使用GPS
		bool use_gps_z = ref_inited && gps_valid && params.w_z_gps_p > MIN_VALID_W;
		/* use VISION if it's valid and has a valid weight parameter */
		bool use_vision_xy = vision_xy_valid && params.w_xy_vision_p > MIN_VALID_W; // 如果视觉有效且具有有效的权重参数，则使用视觉
		bool use_vision_z = vision_z_valid && params.w_z_vision_p > MIN_VALID_W;
		/* use MOCAP if it's valid and has a valid weight parameter */
		bool use_mocap = mocap_xy_valid && mocap_z_valid && params.w_mocap_p > MIN_VALID_W // 如果mocap有效并且具有有效的权重参数，则使用mocap
				 && params.att_ext_hdg_m == mocap_heading; //check if external heading is mocap//检查外部航向是否为MOCAP

		if (params.disable_mocap) { //disable mocap if fake gps is used // 如果使用假GPS，禁用MOCAP
			use_mocap = false;
		}

		/* use flow if it's valid and (accurate or no GPS available) */
		bool use_flow = flow_valid && (flow_accurate || !use_gps_xy); // 如果光流有效且（精确或无GPS可用），则使用光流。

		/* use LIDAR if it's valid and lidar altitude estimation is enabled */
		use_lidar = lidar_valid && params.enable_lidar_alt_est;  // 如果激光雷达有效并且启用了激光雷达高度估计，则使用激光雷达。
		/************************************************************************/



		bool can_estimate_xy = (eph < max_eph_epv) || use_gps_xy || use_flow || use_vision_xy || use_mocap;

		bool dist_bottom_valid = (t < lidar_valid_time + lidar_valid_timeout);

		/************************************************************************/
		//      计算权重
		/************************************************************************/
		float w_xy_gps_p = params.w_xy_gps_p * w_gps_xy;  // w_gps_xy是否会衰减（0~1）
		float w_xy_gps_v = params.w_xy_gps_v * w_gps_xy;
		float w_z_gps_p = params.w_z_gps_p * w_gps_z;
		float w_z_gps_v = params.w_z_gps_v * w_gps_z;

		float w_xy_vision_p = params.w_xy_vision_p;
		float w_xy_vision_v = params.w_xy_vision_v;
		float w_z_vision_p = params.w_z_vision_p;

		float w_mocap_p = params.w_mocap_p;

		/* reduce GPS weight if optical flow is good */
		if (use_flow && flow_accurate) {
			w_xy_gps_p *= params.w_gps_flow;
			w_xy_gps_v *= params.w_gps_flow;
		}

		/* baro offset correction */
		// 气压偏移校正
		if (use_gps_z) {
			float offs_corr = corr_gps[2][0] * w_z_gps_p * dt;
			baro_offset += offs_corr;
			corr_baro += offs_corr;
		}
		/************************************************************************/

		// 融合过程

		/************************************************************************/
		//      使用GPS融合       计算加速度偏差acc_bias
		/************************************************************************/
		/* accelerometer bias correction for GPS (use buffered rotation matrix) */
		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };//加速度计偏差校正（使用缓冲旋转矩阵）（GPS、视觉）

		if (use_gps_xy) {
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;// 位置要比加速度高两阶，所以乘以两个权重
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;  // 速度要比加速度高一阶，所以乘以一个权重
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
		}

		if (use_gps_z) {
			accel_bias_corr[2] -= corr_gps[2][0] * w_z_gps_p * w_z_gps_p;
			accel_bias_corr[2] -= corr_gps[2][1] * w_z_gps_v;
		}

		/* transform error vector from NED frame to body frame */
		// 旋转到机体上
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R_gps[j][i] * accel_bias_corr[j];
			}

			if (PX4_ISFINITE(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}
		/************************************************************************/



		/*******************************START************************************/
		//      使用视觉       计算加速度偏差acc_bias
		/************************************************************************/
		/* accelerometer bias correction for VISION (use buffered rotation matrix) */
		accel_bias_corr[0] = 0.0f;// 加速度计视觉偏差校正（使用缓冲旋转矩阵）
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_vision_xy) {
			accel_bias_corr[0] -= corr_vision[0][0] * w_xy_vision_p * w_xy_vision_p;
			accel_bias_corr[0] -= corr_vision[0][1] * w_xy_vision_v;
			accel_bias_corr[1] -= corr_vision[1][0] * w_xy_vision_p * w_xy_vision_p;
			accel_bias_corr[1] -= corr_vision[1][1] * w_xy_vision_v;
		}

		if (use_vision_z) {
			accel_bias_corr[2] -= corr_vision[2][0] * w_z_vision_p * w_z_vision_p;
		}

		/* accelerometer bias correction for MOCAP (use buffered rotation matrix) */
		accel_bias_corr[0] = 0.0f;
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_mocap) {
			accel_bias_corr[0] -= corr_mocap[0][0] * w_mocap_p * w_mocap_p;
			accel_bias_corr[1] -= corr_mocap[1][0] * w_mocap_p * w_mocap_p;
			accel_bias_corr[2] -= corr_mocap[2][0] * w_mocap_p * w_mocap_p;
		}

		/* transform error vector from NED frame to body frame */
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R(j, i) * accel_bias_corr[j];
			}

			if (PX4_ISFINITE(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}
		/*********************************END************************************/



		/*******************************START************************************/
		//  使用光流       计算加速度偏差acc_bias  高度方向偏差计算需要使用激光雷达或气压计
		/************************************************************************/
		/* accelerometer bias correction for flow and baro (assume that there is no delay) */
		accel_bias_corr[0] = 0.0f;
		accel_bias_corr[1] = 0.0f;
		accel_bias_corr[2] = 0.0f;

		if (use_flow) {
			accel_bias_corr[0] -= corr_flow[0] * params.w_xy_flow;
			accel_bias_corr[1] -= corr_flow[1] * params.w_xy_flow;
		}

		if (use_lidar) {
			accel_bias_corr[2] -= corr_lidar * params.w_z_lidar * params.w_z_lidar;

		} else {
			// 使用气压计：加速度偏移向量矫正
			accel_bias_corr[2] -= corr_baro * params.w_z_baro * params.w_z_baro;
		}

		/* transform error vector from NED frame to body frame */
		// 误差向量从导航坐标系转到机体坐标系
		for (int i = 0; i < 3; i++) {
			float c = 0.0f;

			for (int j = 0; j < 3; j++) {
				c += R(j, i) * accel_bias_corr[j];
			}

			if (PX4_ISFINITE(c)) {
				acc_bias[i] += c * params.w_acc_bias * dt;
			}
		}
		/*********************************END************************************/



		/*******************************START************************************/
		//  处理z轴
		//  1、预计导航坐标系下z轴位置和速度
		//  2、利用该传感器得到的速度和位置修正，得到修正后的z_est[]
		/************************************************************************/
		// 高度惯性滤波预测
		/* inertial filter prediction for altitude */
		inertial_filter_predict(dt, z_est, acc[2]);

		if (!(PX4_ISFINITE(z_est[0]) && PX4_ISFINITE(z_est[1]))) {
			write_debug_log("BAD ESTIMATE AFTER Z PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
					acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
					corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
			memcpy(z_est, z_est_prev, sizeof(z_est));
		}

		/* inertial filter correction for altitude */
		// 使用其他传感器进行高度惯性滤波校正
		if (use_lidar) {
			inertial_filter_correct(corr_lidar, dt, z_est, 0, params.w_z_lidar);

		} else {
			// 气压计高度位置矫正
			inertial_filter_correct(corr_baro, dt, z_est, 0, params.w_z_baro);
		}

		if (use_gps_z) {
			epv = fminf(epv, gps.epv);

			inertial_filter_correct(corr_gps[2][0], dt, z_est, 0, w_z_gps_p);
			inertial_filter_correct(corr_gps[2][1], dt, z_est, 1, w_z_gps_v);
		}

		if (use_vision_z) {
			epv = fminf(epv, epv_vision);
			inertial_filter_correct(corr_vision[2][0], dt, z_est, 0, w_z_vision_p);
		}

		if (use_mocap) {
			epv = fminf(epv, epv_mocap);
			inertial_filter_correct(corr_mocap[2][0], dt, z_est, 0, w_mocap_p);
		}

		if (!(PX4_ISFINITE(z_est[0]) && PX4_ISFINITE(z_est[1]))) {
			write_debug_log("BAD ESTIMATE AFTER Z CORRECTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
					acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
					corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
			memcpy(z_est, z_est_prev, sizeof(z_est));
			memset(corr_gps, 0, sizeof(corr_gps));
			memset(corr_vision, 0, sizeof(corr_vision));
			memset(corr_mocap, 0, sizeof(corr_mocap));
			corr_baro = 0;

		} else {
			memcpy(z_est_prev, z_est, sizeof(z_est));
		}
		/*********************************END************************************/



		/*******************************START************************************/
		//  处理z轴
		//  1、预计导航坐标系下xy轴位置和速度
		//  2、利用该传感器得到的速度和位置修正，得到修正后的x_est[] ，y_est[]
		/************************************************************************/
		if (can_estimate_xy) {
			/* inertial filter prediction for position */
			// 预测位置和速度
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);

			if (!(PX4_ISFINITE(x_est[0]) && PX4_ISFINITE(x_est[1]) && PX4_ISFINITE(y_est[0]) && PX4_ISFINITE(y_est[1]))) {
				write_debug_log("BAD ESTIMATE AFTER PREDICTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
						acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
						corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
			}

			/* inertial filter correction for position */
			// 使用光流    则进行位置惯性滤波器校正
			if (use_flow) {
				eph = fminf(eph, eph_flow);

				inertial_filter_correct(corr_flow[0], dt, x_est, 1, params.w_xy_flow * w_flow);
				inertial_filter_correct(corr_flow[1], dt, y_est, 1, params.w_xy_flow * w_flow);
			}

			// 使用GPS
			if (use_gps_xy) {
				eph = fminf(eph, gps.eph);

				inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
				inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

				if (gps.vel_ned_valid && t < gps.timestamp + gps_topic_timeout) {
					inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
					inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
				}
			}

			// 使用视觉
			if (use_vision_xy) {
				eph = fminf(eph, eph_vision);

				inertial_filter_correct(corr_vision[0][0], dt, x_est, 0, w_xy_vision_p);
				inertial_filter_correct(corr_vision[1][0], dt, y_est, 0, w_xy_vision_p);

				if (w_xy_vision_v > MIN_VALID_W) {
					inertial_filter_correct(corr_vision[0][1], dt, x_est, 1, w_xy_vision_v);
					inertial_filter_correct(corr_vision[1][1], dt, y_est, 1, w_xy_vision_v);
				}
			}

			//使用MOCAP
			if (use_mocap) {
				eph = fminf(eph, eph_mocap);

				inertial_filter_correct(corr_mocap[0][0], dt, x_est, 0, w_mocap_p);
				inertial_filter_correct(corr_mocap[1][0], dt, y_est, 0, w_mocap_p);
			}

			if (!(PX4_ISFINITE(x_est[0]) && PX4_ISFINITE(x_est[1]) && PX4_ISFINITE(y_est[0]) && PX4_ISFINITE(y_est[1]))) {
				write_debug_log("BAD ESTIMATE AFTER CORRECTION", dt, x_est, y_est, z_est, x_est_prev, y_est_prev, z_est_prev,
						acc, corr_gps, w_xy_gps_p, w_xy_gps_v, corr_mocap, w_mocap_p,
						corr_vision, w_xy_vision_p, w_z_vision_p, w_xy_vision_v);
				memcpy(x_est, x_est_prev, sizeof(x_est));
				memcpy(y_est, y_est_prev, sizeof(y_est));
				memset(corr_gps, 0, sizeof(corr_gps));
				memset(corr_vision, 0, sizeof(corr_vision));
				memset(corr_mocap, 0, sizeof(corr_mocap));
				memset(corr_flow, 0, sizeof(corr_flow));

			} else {
				memcpy(x_est_prev, x_est, sizeof(x_est));
				memcpy(y_est_prev, y_est, sizeof(y_est));
			}

		} else {
			/* gradually reset xy velocity estimates */
			// 逐渐重置xy速度估计
			inertial_filter_correct(-x_est[1], dt, x_est, 1, params.w_xy_res_v);
			inertial_filter_correct(-y_est[1], dt, y_est, 1, params.w_xy_res_v);
		}
		/*********************************END************************************/

		/* run terrain estimator */
		// 运行地形估计器
		terrain_estimator.predict(dt, &att, &sensor, &lidar);
		terrain_estimator.measurement_update(hrt_absolute_time(), &gps, &lidar, &att);

		if (inav_verbose_mode) {
			/* print updates rate */
			if (t > updates_counter_start + updates_counter_len) {
				float updates_dt = (t - updates_counter_start) * 0.000001f;
				warnx(
					"updates rate: accelerometer = %.1f/s, baro = %.1f/s, gps = %.1f/s, attitude = %.1f/s, flow = %.1f/s, vision = %.1f/s, mocap = %.1f/s",
					(double)(accel_updates / updates_dt),
					(double)(baro_updates / updates_dt),
					(double)(gps_updates / updates_dt),
					(double)(attitude_updates / updates_dt),
					(double)(flow_updates / updates_dt),
					(double)(vision_updates / updates_dt),
					(double)(mocap_updates / updates_dt));
				updates_counter_start = t;
				accel_updates = 0;
				baro_updates = 0;
				gps_updates = 0;
				attitude_updates = 0;
				flow_updates = 0;
				vision_updates = 0;
				mocap_updates = 0;
			}
		}

		if (t > pub_last + PUB_INTERVAL) {
			pub_last = t;

			/* push current estimate to buffer */
			est_buf[buf_ptr][0][0] = x_est[0];
			est_buf[buf_ptr][0][1] = x_est[1];
			est_buf[buf_ptr][1][0] = y_est[0];
			est_buf[buf_ptr][1][1] = y_est[1];
			est_buf[buf_ptr][2][0] = z_est[0];
			est_buf[buf_ptr][2][1] = z_est[1];

			/* push current rotation matrix to buffer */
			memcpy(R_buf[buf_ptr], &R._data[0][0], sizeof(R._data));

			buf_ptr++;

			if (buf_ptr >= EST_BUF_SIZE) {
				buf_ptr = 0;
			}


			/* publish local position */
			pos.xy_valid = can_estimate_xy;
			pos.v_xy_valid = can_estimate_xy;
			pos.xy_global = pos.xy_valid && use_gps_xy;
			pos.z_global = pos.z_valid && use_gps_z;
			pos.x = x_est[0];
			pos.vx = x_est[1];
			pos.y = y_est[0];
			pos.vy = y_est[1];
			pos.z = z_est[0];
			pos.vz = z_est[1];
			pos.ax = NAN;
			pos.ay = NAN;
			pos.az = NAN;
			pos.yaw = matrix::Eulerf(matrix::Quatf(att.q)).psi();
			pos.dist_bottom_valid = dist_bottom_valid;
			pos.eph = eph;
			pos.epv = epv;
			pos.evh = 0.0f;
			pos.evv = 0.0f;
			pos.vxy_max = INFINITY;
			pos.vz_max = INFINITY;
			pos.hagl_min = INFINITY;
			pos.hagl_max = INFINITY;

			// this estimator does not provide a separate vertical position time derivative estimate, so use the vertical velocity
			pos.z_deriv = z_est[1];

			if (pos.dist_bottom_valid) {
				pos.dist_bottom = dist_ground;
				pos.dist_bottom_rate = - z_est[1];
			}

			pos.timestamp = t;

			orb_publish(ORB_ID(vehicle_local_position), vehicle_local_position_pub, &pos);

			if (pos.xy_global && pos.z_global) {
				/* publish global position */
				global_pos.timestamp = t;

				double est_lat, est_lon;
				map_projection_reproject(&ref, pos.x, pos.y, &est_lat, &est_lon);

				global_pos.lat = est_lat;
				global_pos.lon = est_lon;
				global_pos.alt = pos.ref_alt - pos.z;

				global_pos.vel_n = pos.vx;
				global_pos.vel_e = pos.vy;
				global_pos.vel_d = pos.vz;

				global_pos.yaw = matrix::Eulerf(matrix::Quatf(R)).psi();

				global_pos.eph = eph;
				global_pos.epv = epv;

				if (terrain_estimator.is_valid()) {
					global_pos.terrain_alt = global_pos.alt - terrain_estimator.get_distance_to_ground();
					global_pos.terrain_alt_valid = true;

				} else {
					global_pos.terrain_alt_valid = false;
				}

				if (vehicle_global_position_pub == nullptr) {
					vehicle_global_position_pub = orb_advertise(ORB_ID(vehicle_global_position), &global_pos);

				} else {
					orb_publish(ORB_ID(vehicle_global_position), vehicle_global_position_pub, &global_pos);
				}
			}
		}
	}

	warnx("stopped");
	mavlink_log_info(&mavlink_log_pub, "[inav] stopped");
	thread_running = false;
	return 0;
}


int inav_parameters_init(struct position_estimator_inav_param_handles *h)
{
	h->w_z_baro = param_find("INAV_W_Z_BARO");
	h->w_z_gps_p = param_find("INAV_W_Z_GPS_P");
	h->w_z_gps_v = param_find("INAV_W_Z_GPS_V");
	h->w_z_vision_p = param_find("INAV_W_Z_VIS_P");
	h->w_z_lidar = param_find("INAV_W_Z_LIDAR");
	h->w_xy_gps_p = param_find("INAV_W_XY_GPS_P");
	h->w_xy_gps_v = param_find("INAV_W_XY_GPS_V");
	h->w_xy_vision_p = param_find("INAV_W_XY_VIS_P");
	h->w_xy_vision_v = param_find("INAV_W_XY_VIS_V");
	h->w_mocap_p = param_find("INAV_W_MOC_P");
	h->w_xy_flow = param_find("INAV_W_XY_FLOW");
	h->w_xy_res_v = param_find("INAV_W_XY_RES_V");
	h->w_gps_flow = param_find("INAV_W_GPS_FLOW");
	h->w_acc_bias = param_find("INAV_W_ACC_BIAS");
	h->flow_k = param_find("INAV_FLOW_K");
	h->flow_q_min = param_find("INAV_FLOW_Q_MIN");
	h->lidar_err = param_find("INAV_LIDAR_ERR");
	h->land_t = param_find("INAV_LAND_T");
	h->land_disp = param_find("INAV_LAND_DISP");
	h->land_thr = param_find("INAV_LAND_THR");
	h->no_vision = param_find("CBRK_NO_VISION");
	h->delay_gps = param_find("INAV_DELAY_GPS");
	h->flow_module_offset_x = param_find("INAV_FLOW_DIST_X");
	h->flow_module_offset_y = param_find("INAV_FLOW_DIST_Y");
	h->disable_mocap = param_find("INAV_DISAB_MOCAP");
	h->enable_lidar_alt_est = param_find("INAV_LIDAR_EST");
	h->lidar_calibration_offset = param_find("INAV_LIDAR_OFF");
	h->att_ext_hdg_m = param_find("ATT_EXT_HDG_M");

	return 0;
}

int inav_parameters_update(const struct position_estimator_inav_param_handles *h,
			   struct position_estimator_inav_params *p)
{
	param_get(h->w_z_baro, &(p->w_z_baro));
	param_get(h->w_z_gps_p, &(p->w_z_gps_p));
	param_get(h->w_z_gps_v, &(p->w_z_gps_v));
	param_get(h->w_z_vision_p, &(p->w_z_vision_p));
	param_get(h->w_z_lidar, &(p->w_z_lidar));
	param_get(h->w_xy_gps_p, &(p->w_xy_gps_p));
	param_get(h->w_xy_gps_v, &(p->w_xy_gps_v));
	param_get(h->w_xy_vision_p, &(p->w_xy_vision_p));
	param_get(h->w_xy_vision_v, &(p->w_xy_vision_v));
	param_get(h->w_mocap_p, &(p->w_mocap_p));
	param_get(h->w_xy_flow, &(p->w_xy_flow));
	param_get(h->w_xy_res_v, &(p->w_xy_res_v));
	param_get(h->w_gps_flow, &(p->w_gps_flow));
	param_get(h->w_acc_bias, &(p->w_acc_bias));
	param_get(h->flow_k, &(p->flow_k));
	param_get(h->flow_q_min, &(p->flow_q_min));
	param_get(h->lidar_err, &(p->lidar_err));
	param_get(h->land_t, &(p->land_t));
	param_get(h->land_disp, &(p->land_disp));
	param_get(h->land_thr, &(p->land_thr));
	param_get(h->no_vision, &(p->no_vision));
	param_get(h->delay_gps, &(p->delay_gps));
	param_get(h->flow_module_offset_x, &(p->flow_module_offset_x));
	param_get(h->flow_module_offset_y, &(p->flow_module_offset_y));
	param_get(h->disable_mocap, &(p->disable_mocap));
	param_get(h->enable_lidar_alt_est, &(p->enable_lidar_alt_est));
	param_get(h->lidar_calibration_offset, &(p->lidar_calibration_offset));
	param_get(h->att_ext_hdg_m, &(p->att_ext_hdg_m));

	return 0;
}
