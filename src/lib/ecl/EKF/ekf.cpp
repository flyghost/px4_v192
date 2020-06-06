/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <ecl.h>
#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	_state.vel.setZero();
	_state.pos.setZero();
	_state.gyro_bias.setZero();
	_state.accel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setZero();
	_state.quat_nominal(0) = 1.0f;

	_output_new.vel.setZero();
	_output_new.pos.setZero();
	_output_new.quat_nominal.setZero();
	_output_new.quat_nominal(0) = 1.0f;

	_delta_angle_corr.setZero();
	_imu_down_sampled.delta_ang.setZero();
	_imu_down_sampled.delta_vel.setZero();
	_imu_down_sampled.delta_ang_dt = 0.0f;
	_imu_down_sampled.delta_vel_dt = 0.0f;
	_imu_down_sampled.time_us = timestamp;

	_q_down_sampled(0) = 1.0f;
	_q_down_sampled(1) = 0.0f;
	_q_down_sampled(2) = 0.0f;
	_q_down_sampled(3) = 0.0f;

	_imu_updated = false;
	_NED_origin_initialised = false;
	_gps_speed_valid = false;

	_filter_initialised = false;
	_terrain_initialised = false;
	_sin_tilt_rng = sinf(_params.rng_sens_pitch);
	_cos_tilt_rng = cosf(_params.rng_sens_pitch);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_dt_ekf_avg = FILTER_UPDATE_PERIOD_S;

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_accel_mag_filt = 0.0f;
	_ang_rate_mag_filt = 0.0f;
	_prev_dvel_bias_var.zero();

	return ret;
}

bool Ekf::update()
{
	bool updated = false;

	if (!_filter_initialised) {
		_filter_initialised = initialiseFilter();

		if (!_filter_initialised) {
			return false;
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {

		// perform state and covariance prediction for the main filter
		predictState();
		predictCovariance();

		// control fusion of observation data
		controlFusionModes();

		// run a separate filter for terrain estimation
		runTerrainEstimator();

		updated = true;
	}

	// the output observer always runs
	// Use full rate IMU data at the current time horizon
	calculateOutputStates();

	return updated;
}

// 初始化滤波器
bool Ekf::initialiseFilter()
{
	// 继续累积测量值，直到我们至少有10个样本用于所需的传感器
	// IMU delta角度的测量值的和
	const imuSample &imu_init = _imu_buffer.get_newest();
	_delVel_sum += imu_init.delta_vel;

	// 磁力计测量值求和
	if (_mag_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_mag_sample_delayed)) {
		if ((_mag_counter == 0) && (_mag_sample_delayed.time_us != 0)) {
			// 初始化计数器当我们开始从缓冲区获取数据时
			_mag_counter = 1;

		} else if ((_mag_counter != 0) && (_mag_sample_delayed.time_us != 0)) {
			// 增加样本数量并对测量值进行低通滤波
			_mag_counter ++;

			// 在确定所有坏的初始数据都已被清除之前，不要开始使用数据
			if (_mag_counter == (uint8_t)(_obs_buffer_length + 1)) {
				// 初始化滤波器状态
				_mag_filt_state = _mag_sample_delayed.mag;

			} else if (_mag_counter > (uint8_t)(_obs_buffer_length + 1)) {
				// 对数据进行噪声滤波
				_mag_filt_state = _mag_filt_state * 0.9f + _mag_sample_delayed.mag * 0.1f;
			}
		}
	}

	// Count the number of external vision measurements received
	// 统计从外部视觉测量获得的量
	if (_ext_vision_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_ev_sample_delayed)) {
		if ((_ev_counter == 0) && (_ev_sample_delayed.time_us != 0)) {
			// initialise the counter
			_ev_counter = 1;

			// set the height fusion mode to use external vision data when we start getting valid data from the buffer
			if (_primary_hgt_source == VDIST_SENSOR_EV) {
				_control_status.flags.baro_hgt = false;
				_control_status.flags.gps_hgt = false;
				_control_status.flags.rng_hgt = false;
				_control_status.flags.ev_hgt = true;
			}

		} else if ((_ev_counter != 0) && (_ev_sample_delayed.time_us != 0)) {
			// increment the sample count
			_ev_counter ++;
		}
	}

	// set the default height source from the adjustable parameter
	// 从可调参数设置默认高度源
	if (_hgt_counter == 0) {
		_primary_hgt_source = _params.vdist_sensor_type;
	}

	// accumulate enough height measurements to be confident in the quality of the data
	// we use baro height initially and switch to GPS/range/EV finder later when it passes checks.
	if (_baro_buffer.pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed)) {
		if ((_hgt_counter == 0) && (_baro_sample_delayed.time_us != 0)) {
			// initialise the counter and height fusion method when we start getting data from the buffer
			setControlBaroHeight();
			_hgt_counter = 1;

		} else if ((_hgt_counter != 0) && (_baro_sample_delayed.time_us != 0)) {
			// increment the sample count and apply a LPF to the measurement
			_hgt_counter ++;

			// don't start using data until we can be certain all bad initial data has been flushed
			if (_hgt_counter == (uint8_t)(_obs_buffer_length + 1)) {
				// initialise filter states
				_baro_hgt_offset = _baro_sample_delayed.hgt;

			} else if (_hgt_counter > (uint8_t)(_obs_buffer_length + 1)) {
				// noise filter the data
				_baro_hgt_offset = 0.9f * _baro_hgt_offset + 0.1f * _baro_sample_delayed.hgt;
			}
		}
	}

	// 检查是否有足够的测量值，如果没有，返回错误
	bool hgt_count_fail = _hgt_counter <= 2u * _obs_buffer_length;   // 气压计
	bool mag_count_fail = _mag_counter <= 2u * _obs_buffer_length;   // 磁力计

	if (hgt_count_fail || mag_count_fail) {
		return false;

	} else {
		// reset variables that are shared with post alignment GPS checks
		_gps_drift_velD = 0.0f;
		_gps_alt_ref = 0.0f;

		// 清零所有状态
		_state.vel.setZero();
		_state.pos.setZero();
		_state.gyro_bias.setZero();
		_state.accel_bias.setZero();
		_state.mag_I.setZero();
		_state.mag_B.setZero();
		_state.wind_vel.setZero();

		// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
		float pitch = 0.0f;
		float roll = 0.0f;

		// 用于初始化的总样本
		if (_delVel_sum.norm() > 0.001f) {
			_delVel_sum.normalize();
			pitch = asinf(_delVel_sum(0));
			roll = atan2f(-_delVel_sum(1), -_delVel_sum(2));

		} else {
			return false;
		}

		// 计算初始倾斜对准
		Eulerf euler_init(roll, pitch, 0.0f);
		_state.quat_nominal = Quatf(euler_init);

		// 更新旋转矩阵( B -> R )
		_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

		// 计算初始磁场和偏航对准
		_control_status.flags.yaw_align = resetMagHeading(_mag_filt_state, false, false);

		// 初始化状态协方差矩阵
		initialiseCovariance();

		// 使用测量的方差更新偏航角方差
		if (_control_status.flags.ev_yaw) {
			// 使用来自外部视觉数据的误差估计来做:这从来都不是真的
			increaseQuatYawErrVariance(sq(fmaxf(_ev_sample_delayed.angErr, 1.0e-2f)));
		} else if (_params.mag_fusion_type <= MAG_FUSE_TYPE_AUTOFW) {
			// 使用磁航向调整参数
			increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));  // fmaxf(0.3 , 1.0e-2f)
		}

		if (_control_status.flags.rng_hgt) {
			// if we are using the range finder as the primary source, then calculate the baro height at origin so  we can use baro as a backup
			// so it can be used as a backup ad set the initial height using the range finder
			const baroSample &baro_newest = _baro_buffer.get_newest();
			_baro_hgt_offset = baro_newest.hgt;
			_state.pos(2) = -math::max(_rng_filt_state * _R_rng_to_earth_2_2, _params.rng_gnd_clearance);
			ECL_INFO("EKF using range finder height - commencing alignment");

		} else if (_control_status.flags.ev_hgt) {
			// if we are using external vision data for height, then the vertical position state needs to be reset
			// because the initialisation position is not the zero datum
			resetHeight();

		}

		// 尝试初始化地形估计器
		_terrain_initialised = initHagl();

		// reset the essential fusion timeout counters
		// 重置基本的融合超时计数器
		_time_last_hgt_fuse = _time_last_imu;
		_time_last_pos_fuse = _time_last_imu;
		_time_last_delpos_fuse = _time_last_imu;
		_time_last_vel_fuse = _time_last_imu;
		_time_last_hagl_fuse = _time_last_imu;
		_time_last_of_fuse = _time_last_imu;

		// reset the output predictor state history to match the EKF initial values
		// 重置输出预测器状态历史记录以匹配EKF初始值
		alignOutputFilter();

		return true;
	}
}

void Ekf::predictState()
{
	if (!_earth_rate_initialised) {         // 真：当我们知道地球自转率时（需要GPS）
		if (_NED_origin_initialised) {  // NED原点已初始化
			calcEarthRateNED(_earth_rate_NED, (float)_pos_ref.lat_rad);
			_earth_rate_initialised = true;
		}
	}

	// apply imu bias corrections
	Vector3f corrected_delta_ang = _imu_sample_delayed.delta_ang - _state.gyro_bias;
	Vector3f corrected_delta_vel = _imu_sample_delayed.delta_vel - _state.accel_bias;

	// correct delta angles for earth rotation rate
	corrected_delta_ang -= -_R_to_earth.transpose() * _earth_rate_NED * _imu_sample_delayed.delta_ang_dt;

	// convert the delta angle to a delta quaternion
	Quatf dq;
	dq.from_axis_angle(corrected_delta_ang);

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = _state.quat_nominal * dq;

	// quaternions must be normalised whenever they are modified
	_state.quat_nominal.normalize();

	// save the previous value of velocity so we can use trapzoidal integration
	Vector3f vel_last = _state.vel;

	// update transformation matrix from body to world frame
	_R_to_earth = quat_to_invrotmat(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	float alpha = 1.0f - _imu_sample_delayed.delta_vel_dt;
	_accel_lpf_NE(0) = _accel_lpf_NE(0) * alpha + corrected_delta_vel_ef(0);
	_accel_lpf_NE(1) = _accel_lpf_NE(1) * alpha + corrected_delta_vel_ef(1);

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * _imu_sample_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * _imu_sample_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (_imu_sample_delayed.delta_vel_dt + _imu_sample_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	input = math::constrain(input, 0.5f * FILTER_UPDATE_PERIOD_S, 2.0f * FILTER_UPDATE_PERIOD_S);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;
}

bool Ekf::collect_imu(const imuSample &imu)
{
	// accumulate and downsample IMU data across a period FILTER_UPDATE_PERIOD_MS long

	// copy imu data to local variables
	_imu_sample_new = imu;

	// accumulate the time deltas
	_imu_down_sampled.delta_ang_dt += imu.delta_ang_dt;
	_imu_down_sampled.delta_vel_dt += imu.delta_vel_dt;

	// use a quaternion to accumulate delta angle data
	// this quaternion represents the rotation from the start to end of the accumulation period
	Quatf delta_q;
	delta_q.rotate(imu.delta_ang);
	_q_down_sampled = _q_down_sampled * delta_q;
	_q_down_sampled.normalize();

	// rotate the accumulated delta velocity data forward each time so it is always in the updated rotation frame
	Dcmf delta_R(delta_q.inversed());
	_imu_down_sampled.delta_vel = delta_R * _imu_down_sampled.delta_vel;

	// accumulate the most recent delta velocity data at the updated rotation frame
	// assume effective sample time is halfway between the previous and current rotation frame
	_imu_down_sampled.delta_vel += (imu.delta_vel + delta_R * imu.delta_vel) * 0.5f;

	// if the target time delta between filter prediction steps has been exceeded
	// write the accumulated IMU data to the ring buffer
	const float target_dt = FILTER_UPDATE_PERIOD_S;

	if (_imu_down_sampled.delta_ang_dt >= target_dt - _imu_collection_time_adj) {

		// accumulate the amount of time to advance the IMU collection time so that we meet the
		// average EKF update rate requirement
		_imu_collection_time_adj += 0.01f * (_imu_down_sampled.delta_ang_dt - target_dt);
		_imu_collection_time_adj = math::constrain(_imu_collection_time_adj, -0.5f * target_dt, 0.5f * target_dt);

		imuSample imu_sample_new;
		imu_sample_new.delta_ang = _q_down_sampled.to_axis_angle();
		imu_sample_new.delta_vel = _imu_down_sampled.delta_vel;
		imu_sample_new.delta_ang_dt = _imu_down_sampled.delta_ang_dt;
		imu_sample_new.delta_vel_dt = _imu_down_sampled.delta_vel_dt;
		imu_sample_new.time_us = imu.time_us;

		_imu_buffer.push(imu_sample_new);

		// get the oldest data from the buffer
		_imu_sample_delayed = _imu_buffer.get_oldest();

		// calculate the minimum interval between observations required to guarantee no loss of data
		// this will occur if data is overwritten before its time stamp falls behind the fusion time horizon
		// 计算保证数据不丢失所需的观测之间的最小间隔
		// 如果数据在其时间戳落后于融合时间范围之前被覆盖，就会发生这种情况
		_min_obs_interval_us = (imu_sample_new.time_us - _imu_sample_delayed.time_us) / (_obs_buffer_length - 1);

		// reset
		_imu_down_sampled.delta_ang.setZero();
		_imu_down_sampled.delta_vel.setZero();
		_imu_down_sampled.delta_ang_dt = 0.0f;
		_imu_down_sampled.delta_vel_dt = 0.0f;
		_q_down_sampled(0) = 1.0f;
		_q_down_sampled(1) = _q_down_sampled(2) = _q_down_sampled(3) = 0.0f;

		_imu_updated = true;

	} else {
		_imu_updated = false;
	}

	return _imu_updated;
}

/*
 * Implement a strapdown INS algorithm using the latest IMU data at the current time horizon.
 * Buffer the INS states and calculate the difference with the EKF states at the delayed fusion time horizon.
 * Calculate delta angle, delta velocity and velocity corrections from the differences and apply them at the
 * current time horizon so that the INS states track the EKF states at the delayed fusion time horizon.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian:
 * “Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements”
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
void Ekf::calculateOutputStates()
{
	// Use full rate IMU data at the current time horizon
	const imuSample &imu = _imu_sample_new;

	// correct delta angles for bias offsets
	const float dt_scale_correction = _dt_imu_avg / _dt_ekf_avg;

	// Apply corrections to the delta angle required to track the quaternion states at the EKF fusion time horizon
	const Vector3f delta_angle{imu.delta_ang - _state.gyro_bias * dt_scale_correction + _delta_angle_corr};

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = _R_to_earth_now(2, 0) * delta_angle(0) +
				     _R_to_earth_now(2, 1) * delta_angle(1) +
				     _R_to_earth_now(2, 2) * delta_angle(2);
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu.delta_ang_dt;

	// convert the delta angle to an equivalent delta quaternions
	Quatf dq;
	dq.from_axis_angle(delta_angle);

	// rotate the previous INS quaternion by the delta quaternions
	_output_new.time_us = imu.time_us;
	_output_new.quat_nominal = _output_new.quat_nominal * dq;

	// the quaternions must always be normalised after modification
	_output_new.quat_nominal.normalize();

	// calculate the rotation matrix from body to earth frame
	_R_to_earth_now = quat_to_invrotmat(_output_new.quat_nominal);

	// correct delta velocity for bias offsets
	const Vector3f delta_vel{imu.delta_vel - _state.accel_bias * dt_scale_correction};

	// rotate the delta velocity to earth frame
	Vector3f delta_vel_NED{_R_to_earth_now * delta_vel};

	// correct for measured acceleration due to gravity
	delta_vel_NED(2) += CONSTANTS_ONE_G * imu.delta_vel_dt;

	// calculate the earth frame velocity derivatives
	if (imu.delta_vel_dt > 1e-4f) {
		_vel_deriv_ned = delta_vel_NED * (1.0f / imu.delta_vel_dt);
	}

	// save the previous velocity so we can use trapezoidal integration
	const Vector3f vel_last{_output_new.vel};

	// increment the INS velocity states by the measurement plus corrections
	// do the same for vertical state used by alternative correction algorithm
	_output_new.vel += delta_vel_NED;
	_output_vert_new.vel_d += delta_vel_NED(2);

	// use trapezoidal integration to calculate the INS position states
	// do the same for vertical state used by alternative correction algorithm
	const Vector3f delta_pos_NED = (_output_new.vel + vel_last) * (imu.delta_vel_dt * 0.5f);
	_output_new.pos += delta_pos_NED;
	_output_vert_new.vel_d_integ += delta_pos_NED(2);

	// accumulate the time for each update
	_output_vert_new.dt += imu.delta_vel_dt;

	// correct velocity for IMU offset
	if (imu.delta_ang_dt > 1e-4f) {
		// calculate the average angular rate across the last IMU update
		const Vector3f ang_rate = imu.delta_ang * (1.0f / imu.delta_ang_dt);

		// calculate the velocity of the IMU relative to the body origin
		const Vector3f vel_imu_rel_body = cross_product(ang_rate, _params.imu_pos_body);

		// rotate the relative velocity into earth frame
		_vel_imu_rel_body_ned = _R_to_earth_now * vel_imu_rel_body;
	}

	// store the INS states in a ring buffer with the same length and time coordinates as the IMU data buffer
	if (_imu_updated) {
		_output_buffer.push(_output_new);
		_output_vert_buffer.push(_output_vert_new);

		// get the oldest INS state data from the ring buffer
		// this data will be at the EKF fusion time horizon
		_output_sample_delayed = _output_buffer.get_oldest();
		_output_vert_delayed = _output_vert_buffer.get_oldest();

		// calculate the quaternion delta between the INS and EKF quaternions at the EKF fusion time horizon
		Quatf quat_inv = _state.quat_nominal.inversed();
		Quatf q_error =  quat_inv * _output_sample_delayed.quat_nominal;
		q_error.normalize();

		// convert the quaternion delta to a delta angle
		float scalar;

		if (q_error(0) >= 0.0f) {
			scalar = -2.0f;

		} else {
			scalar = 2.0f;
		}

		const Vector3f delta_ang_error{scalar * q_error(1), scalar * q_error(2), scalar * q_error(3)};

		// calculate a gain that provides tight tracking of the estimator attitude states and
		// adjust for changes in time delay to maintain consistent damping ratio of ~0.7
		const float time_delay = fmaxf((imu.time_us - _imu_sample_delayed.time_us) * 1e-6f, _dt_imu_avg);
		const float att_gain = 0.5f * _dt_imu_avg / time_delay;

		// calculate a corrrection to the delta angle
		// that will cause the INS to track the EKF quaternions
		_delta_angle_corr = delta_ang_error * att_gain;

		// calculate velocity and position tracking errors
		const Vector3f vel_err{_state.vel - _output_sample_delayed.vel};
		const Vector3f pos_err{_state.pos - _output_sample_delayed.pos};

		// collect magnitude tracking error for diagnostics
		_output_tracking_error[0] = delta_ang_error.norm();
		_output_tracking_error[1] = vel_err.norm();
		_output_tracking_error[2] = pos_err.norm();

		/*
		 * Loop through the output filter state history and apply the corrections to the velocity and position states.
		 * This method is too expensive to use for the attitude states due to the quaternion operations required
		 * but because it eliminates the time delay in the 'correction loop' it allows higher tracking gains
		 * to be used and reduces tracking error relative to EKF states.
		 */

		// Complementary filter gains
		const float vel_gain = _dt_ekf_avg / math::constrain(_params.vel_Tau, _dt_ekf_avg, 10.0f);
		const float pos_gain = _dt_ekf_avg / math::constrain(_params.pos_Tau, _dt_ekf_avg, 10.0f);
		{
			/*
			 * Calculate a correction to be applied to vel_d that casues vel_d_integ to track the EKF
			 * down position state at the fusion time horizon using an alternative algorithm to what
			 * is used for the vel and pos state tracking. The algorithm applies a correction to the vel_d
			 * state history and propagates vel_d_integ forward in time using the corrected vel_d history.
			 * This provides an alternative vertical velocity output that is closer to the first derivative
			 * of the position but does degrade tracking relative to the EKF state.
			 */

			// calculate down velocity and position tracking errors
			const float vel_d_err = (_state.vel(2) - _output_vert_delayed.vel_d);
			const float pos_d_err = (_state.pos(2) - _output_vert_delayed.vel_d_integ);

			// calculate a velocity correction that will be applied to the output state history
			// using a PD feedback tuned to a 5% overshoot
			const float vel_d_correction = pos_d_err * pos_gain + vel_d_err * pos_gain * 1.1f;

			/*
			 * Calculate corrections to be applied to vel and pos output state history.
			 * The vel and pos state history are corrected individually so they track the EKF states at
			 * the fusion time horizon. This option provides the most accurate tracking of EKF states.
			 */

			// loop through the vertical output filter state history starting at the oldest and apply the corrections to the
			// vel_d states and propagate vel_d_integ forward using the corrected vel_d
			uint8_t index = _output_vert_buffer.get_oldest_index();

			const uint8_t size = _output_vert_buffer.get_length();

			for (uint8_t counter = 0; counter < (size - 1); counter++) {
				const uint8_t index_next = (index + 1) % size;
				outputVert &current_state = _output_vert_buffer[index];
				outputVert &next_state = _output_vert_buffer[index_next];

				// correct the velocity
				if (counter == 0) {
					current_state.vel_d += vel_d_correction;
				}

				next_state.vel_d += vel_d_correction;

				// position is propagated forward using the corrected velocity and a trapezoidal integrator
				next_state.vel_d_integ = current_state.vel_d_integ + (current_state.vel_d + next_state.vel_d) * 0.5f * next_state.dt;

				// advance the index
				index = (index + 1) % size;
			}

			// update output state to corrected values
			_output_vert_new = _output_vert_buffer.get_newest();

			// reset time delta to zero for the next accumulation of full rate IMU data
			_output_vert_new.dt = 0.0f;
		}

		{
			/*
			 * Calculate corrections to be applied to vel and pos output state history.
			 * The vel and pos state history are corrected individually so they track the EKF states at
			 * the fusion time horizon. This option provides the most accurate tracking of EKF states.
			 */

			// calculate a velocity correction that will be applied to the output state history
			_vel_err_integ += vel_err;
			const Vector3f vel_correction = vel_err * vel_gain + _vel_err_integ * sq(vel_gain) * 0.1f;

			// calculate a position correction that will be applied to the output state history
			_pos_err_integ += pos_err;
			const Vector3f pos_correction = pos_err * pos_gain + _pos_err_integ * sq(pos_gain) * 0.1f;

			// loop through the output filter state history and apply the corrections to the velocity and position states
			for (uint8_t index = 0; index < _output_buffer.get_length(); index++) {
				// a constant velocity correction is applied
				_output_buffer[index].vel += vel_correction;

				// a constant position correction is applied
				_output_buffer[index].pos += pos_correction;
			}

			// update output state to corrected values
			_output_new = _output_buffer.get_newest();
		}
	}
}

/*
 * Predict the previous quaternion output state forward using the latest IMU delta angle data.
*/
Quatf Ekf::calculate_quaternion() const
{
	// Correct delta angle data for bias errors using bias state estimates from the EKF and also apply
	// corrections required to track the EKF quaternion states
	const Vector3f delta_angle{_imu_sample_new.delta_ang - _state.gyro_bias * (_dt_imu_avg / _dt_ekf_avg) + _delta_angle_corr};

	// increment the quaternions using the corrected delta angle vector
	// the quaternions must always be normalised after modification
	return Quatf{_output_new.quat_nominal * AxisAnglef{delta_angle}}.unit();
}
