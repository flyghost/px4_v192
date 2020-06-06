/*
 * inertial_filter.c
 *
 *   Copyright (C) 2013 Anton Babushkin. All rights reserved.
 *   Author: 	Anton Babushkin	<rk3dov@gmail.com>
 */

#include "px4_defines.h"
#include "inertial_filter.h"
#include <cmath>


// 预测函数
/*
 * 对于匀变速运动
 * 速度v = v0 + a*t
 * 位移s = v0*t + 0.5*a*t^2
 *
*/
void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (PX4_ISFINITE(dt)) {
		if (!PX4_ISFINITE(acc)) {
			acc = 0.0f;
		}

		x[0] += x[1] * dt + acc * dt * dt / 2.0f;   // 位移
		x[1] += acc * dt;                           // 速度
	}
}

// 校正函数
/*
 * e修正系数，dt周期时间，x[2]是两个float型成员，x[0]是位置，x[1]是速度
 * i表示修正位置还是速度：0修正位置，1修正速度
 * w权重系数
 *
*/
void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (PX4_ISFINITE(e) && PX4_ISFINITE(w) && PX4_ISFINITE(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}
