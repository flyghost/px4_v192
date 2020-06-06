/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
 * @file AttitudeControl.cpp
 */

#include <AttitudeControl.hpp>

#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>

using namespace matrix;

void AttitudeControl::setProportionalGain(const matrix::Vector3f &proportional_gain)
{
	_proportional_gain = proportional_gain;

	// prepare yaw weight from the ratio between roll/pitch and yaw gains
	const float roll_pitch_gain = (proportional_gain(0) + proportional_gain(1)) / 2.f;
	_yaw_w = math::constrain(proportional_gain(2) / roll_pitch_gain, 0.f, 1.f);

	_proportional_gain(2) = roll_pitch_gain;
}

// 轴角法，解耦合控制
matrix::Vector3f AttitudeControl::update(matrix::Quatf q, matrix::Quatf qd, float yawspeed_feedforward)
{
	// ensure input quaternions are exactly normalized because acosf(1.00001) == NaN
	q.normalize();     // 归一化
	qd.normalize();

	// calculate reduced desired attitude neglecting vehicle's yaw to prioritize roll and pitch
	const Vector3f e_z = q.dcm_z();      // 四元数转到旋转矩阵，同时选定z轴，当前姿态的Z轴
	const Vector3f e_z_d = qd.dcm_z();   // 期望姿态的Z轴
	Quatf qd_red(e_z, e_z_d);            // 算出e_z到e_z_d的最短旋转  // 从当前姿态Z轴旋转到期望姿态Z轴的四元数：期望姿态z轴 - 当前姿态z轴

	// qd_red已经归一化了，所以就是判断 qe_red(1) = 1 或者 qe_red(2) = 1 的情况
	// 即对应的四元数为(0,1,0,0)和(0,0,1,0)
	// q0=cos(α/2)     q1=xsin(α/2)     q2=ysin(α/2)     q3=zsin(α/2)
	// 当cos(α/2)为0时，可知α为±180°，故此时旋转轴应为（1,0,0）,或者（0,1,0）
	// 可知，当旋转轴为x轴或者y轴时，航向角是不会发生转动的
	// 故此时可以直接把期望姿态作为过渡姿态使用。所以把期望姿态qd直接赋值给过渡姿态qd_red
	if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
		// In the infinitesimal corner case where the vehicle and thrust have the completely opposite direction,
		// full attitude control anyways generates no yaw input and directly takes the combination of
		// roll and pitch leading to the correct desired yaw. Ignoring this case would still be totally safe and stable.
		qd_red = qd;   // 直接把期望姿态当做过渡姿态

	} else {
		// transform rotation from current to desired thrust vector into a world frame reduced desired attitude
		qd_red *= q;   // 将角度差弥补到当前姿态，得到过渡姿态
	}

	// mix full and reduced desired attitude
	Quatf q_mix = qd_red.inversed() * qd;   		// 期望姿态在过渡姿态中的映射，也就是两个姿态差
	q_mix *= math::signNoZero(q_mix(0));    		// math::signNoZero(q_mix(0))得到符号，然后将符号赋值给q_mix
	// catch numerical problems with the domain of acosf and asinf
	q_mix(0) = math::constrain(q_mix(0), -1.f, 1.f);	// 限幅到 -180°到180°
	q_mix(3) = math::constrain(q_mix(3), -1.f, 1.f);
	// Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))))将期望姿态的yaw乘以权重，得到过渡姿态中的期望姿态
	qd = qd_red * Quatf(cosf(_yaw_w * acosf(q_mix(0))), 0, 0, sinf(_yaw_w * asinf(q_mix(3))));  // 过渡姿态 * 过渡姿态中的期望姿态 = 解耦后的期望姿态

	// quaternion attitude control law, qe is rotation from q to qd
	// 算出由“当前姿态”--->>>>“解耦后的期望旋转”的旋转四元数
	const Quatf qe = q.inversed() * qd;            // 姿态差（解耦后的期望姿态-当前姿态）

	// using sin(alpha/2) scaled rotation axis as attitude error (see quaternion definition by axis angle)
	// also taking care of the antipodal unit quaternion ambiguity
	// qe.imag()是qe的虚部，对应（xsin(α/2),ysin(α/2),zsin(α/2)）
	// 即qe.imag()=（xα/2,yα/2,zα/2），然后乘以2就可以得到（xα,yα,zα）
	const Vector3f eq = 2.f * math::signNoZero(qe(0)) * qe.imag();   // 期望姿态相对于当前姿态中的xyz轴旋转角：期望姿态角度 - 当前姿态角度

	// calculate angular rates setpoint
	// 计算b系角速度期望值
	matrix::Vector3f rate_setpoint = eq.emult(_proportional_gain);   // 外环P运算

	// Feed forward the yaw setpoint rate.
	// yaw_sp_move_rate is the feed forward commanded rotation around the world z-axis,
	// but we need to apply it in the body frame (because _rates_sp is expressed in the body frame).
	// Therefore we infer the world z-axis (expressed in the body frame) by taking the last column of R.transposed (== q.inversed)
	// and multiply it by the yaw setpoint rate (yaw_sp_move_rate).
	// This yields a vector representing the commanded rotatation around the world z-axis expressed in the body frame
	// such that it can be added to the rates setpoint.
	rate_setpoint += q.inversed().dcm_z() * yawspeed_feedforward;

	// limit rates
	for (int i = 0; i < 3; i++) {
		rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
	}

	return rate_setpoint;
}
