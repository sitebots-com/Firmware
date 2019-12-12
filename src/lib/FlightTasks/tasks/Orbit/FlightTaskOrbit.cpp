/****************************************************************************
 *
 *   Copyright (c) 2018-2019 PX4 Development Team. All rights reserved.
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
 * @file FlightTaskOrbit.cpp
 */

#include "FlightTaskOrbit.hpp"
#include <mathlib/mathlib.h>
#include <lib/ecl/geo/geo.h>
#include <uORB/topics/orbit_status.h>

using namespace matrix;

FlightTaskOrbit::FlightTaskOrbit() : _circle_approach_line(_position)
{
}

FlightTaskOrbit::~FlightTaskOrbit()
{
	orb_unadvertise(_orbit_status_pub);
}

bool FlightTaskOrbit::applyCommandParameters(const vehicle_command_s &command)
{
	bool ret = true;
	// save previous velocity and roatation direction
	float v = fabsf(_v);
	bool clockwise = _v > 0;

	// commanded radius
	if (PX4_ISFINITE(command.param1)) {
		clockwise = command.param1 > 0;
		const float r = fabsf(command.param1);
		ret = ret && setRadius(r);
	}

	// commanded velocity, take sign of radius as rotation direction
	if (PX4_ISFINITE(command.param2)) {
		v = command.param2;
	}

	ret = ret && setVelocity(v * (clockwise ? 1.f : -1.f));

	// commanded heading behavior
	if (PX4_ISFINITE(command.param3)) {
		_yaw_behavior = command.param3;
	}

	// TODO: apply x,y / z independently in geo library
	// commanded center coordinates
	// if(PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6)) {
	// 	map_projection_global_project(command.param5, command.param6, &_center(0), &_center(1));
	// }

	if(PX4_ISFINITE(command.param4)) {
		_lidar_target = command.param4;
		if (_lidar_target < 5.0f) {
			PX4_WARN("[Orbit] Flying too close to the ground");
			ret = false;
		}
	}

	if (PX4_ISFINITE(command.param5) && PX4_ISFINITE(command.param6) && PX4_ISFINITE(command.param7)) {
		if (globallocalconverter_tolocal(command.param5, command.param6, command.param7, &_center(0), &_center(1),
						 &_commanded_alt)) {
			// global to local conversion failed
			ret = false;
		}
	}

	// perpendicularly approach the orbit circle again when new parameters get commanded
	_in_circle_approach = true;
	// sendTelemetry();

	return ret;
}

bool FlightTaskOrbit::sendTelemetry()
{
	orbit_status_s orbit_status = {};
	orbit_status.timestamp = hrt_absolute_time();
	orbit_status.radius = math::signNoZero(_v) * _r;
	orbit_status.frame = _in_circle_approach ? 0 : 1; // MAV_FRAME::MAV_FRAME_GLOBAL
	float z;

	if (globallocalconverter_toglobal(_center(0), _center(1), _position_setpoint(2), &orbit_status.x, &orbit_status.y,
					  &z)) {
		return false; // don't send the message if the transformation failed
	}
	orbit_status.z = _dist_to_bottom;

	if (_orbit_status_pub == nullptr) {
		_orbit_status_pub = orb_advertise(ORB_ID(orbit_status), &orbit_status);

	} else {
		orb_publish(ORB_ID(orbit_status), _orbit_status_pub, &orbit_status);
	}

	return true;
}

bool FlightTaskOrbit::setRadius(float r)
{
	// clip the radius to be within range
	r = math::constrain(r, _radius_min, _radius_max);

	// small radius is more important than high velocity for safety
	if (!checkAcceleration(r, _v, _acceleration_max)) {
		_v = math::sign(_v) * sqrtf(_acceleration_max * r);
	}

	_r = r;
	return true;
}

bool FlightTaskOrbit::setVelocity(const float v)
{
	if (fabs(v) < _velocity_max &&
	    checkAcceleration(_r, v, _acceleration_max)) {
		_v = v;
		return true;
	}

	return false;
}

bool FlightTaskOrbit::checkAcceleration(float r, float v, float a)
{
	return v * v < a * r;
}

bool FlightTaskOrbit::activate()
{
	bool ret = FlightTask::activate();
	_r = _radius_min;
	_v =  1.f;
	_center = Vector2f(_position);
	// _center(0) -= _r; TODO: was this necessary?

	_initial_heading = _yaw;

	// need a valid position and velocity
	ret = ret && PX4_ISFINITE(_position(0))
	      && PX4_ISFINITE(_position(1))
	      && PX4_ISFINITE(_position(2))
	      && PX4_ISFINITE(_velocity(0))
	      && PX4_ISFINITE(_velocity(1))
	      && PX4_ISFINITE(_velocity(2));

	return ret;
}

bool FlightTaskOrbit::update()
{
	FlightTask::updateInitialize();
	Vector2f center_to_position = Vector2f(_position) - _center;

	switch (_yaw_behavior) {
	case 0:
		// make vehicle front always point towards the center
		_yaw_setpoint = atan2f(center_to_position(1), center_to_position(0)) + M_PI_F;
		break;

	case 1:
		// make vehicle keep the same heading as in the beginning of the flight task
		_yaw_setpoint = _initial_heading;
		break;

	case 2:
		// no yaw setpoint
		break;

	case 3:
		if (!_in_circle_approach) {
			if (_v > 0) {
				_yaw_setpoint = atan2f(center_to_position(1), center_to_position(0)) + M_PI_F / 2.f;

			} else {
				_yaw_setpoint = atan2f(center_to_position(1), center_to_position(0)) - M_PI_F / 2.f;
			}

		}

		break;

	case 4:
		_dist_to_bottom =  _sub_vehicle_local_position->get().dist_bottom;
		if (_in_circle_approach) {
			generate_altitude_approach_setpoints();
		} else {
			generate_altitude_lidar_setpoints();
		}
		return true;

	default:
		PX4_WARN("[Orbit] Invalid yaw behavior. Defaulting to poiting torwards the center.");
		_yaw_setpoint = atan2f(center_to_position(1), center_to_position(0)) + M_PI_F;
		break;
	}

	if (_in_circle_approach) {
		generate_circle_approach_setpoints();

	} else {
		generate_circle_setpoints(center_to_position);
	}

	// publish information to UI
	sendTelemetry();

	return true;
}

void FlightTaskOrbit::generate_altitude_approach_setpoints()
{
	if (_circle_approach_line.isEndReached()) {
		Vector3f target = Vector3f(_center(0), _center(1), _commanded_alt);
		_circle_approach_line.setLineFromTo(_position, target);
	}
	_circle_approach_line.generateSetpoints(_position_setpoint, _velocity_setpoint);
	setRadius(fabsf(_position(2) - _commanded_alt));
	sendTelemetry();
	_in_circle_approach = !_circle_approach_line.isEndReached();
	if (!_in_circle_approach) {
		_resetSetpoints();
		generate_altitude_lidar_setpoints();
	}
}

void FlightTaskOrbit::generate_altitude_lidar_setpoints()
{
	_position_setpoint(0) = _center(0);
	_position_setpoint(1) = _center(1);
	if(PX4_ISFINITE(_dist_to_bottom)) {
		_position_setpoint(2) = _position(2) - (_lidar_target - _dist_to_bottom);
		setRadius(fabsf(_dist_to_bottom - _lidar_target));
		_lidar_failures = 0;
	} else if (_lidar_failures < 10) {
		_lidar_failures++;
	} else if (_lidar_failures < 20) {
		_lidar_failures++;
		_position_setpoint(2) = _position(2);
	} else {
		setRadius(0);
	}
	sendTelemetry();
}

void FlightTaskOrbit::generate_circle_approach_setpoints()
{
	if (_circle_approach_line.isEndReached()) {
		// calculate target point on circle and plan a line trajectory
		Vector2f start_to_center = _center - Vector2f(_position);
		Vector2f start_to_circle = (start_to_center.norm() - _r) * start_to_center.unit_or_zero();
		Vector2f closest_circle_point = Vector2f(_position) + start_to_circle;
		Vector3f target = Vector3f(closest_circle_point(0), closest_circle_point(1), _position(2));
		_circle_approach_line.setLineFromTo(_position, target);
		_circle_approach_line.setSpeed(_param_mpc_xy_cruise.get());
		_yaw_setpoint = atan2f(start_to_circle(1), start_to_circle(0));
	}

	// follow the planned line and switch to orbiting once the circle is reached
	_circle_approach_line.generateSetpoints(_position_setpoint, _velocity_setpoint);
	_in_circle_approach = !_circle_approach_line.isEndReached();
}

void FlightTaskOrbit::generate_circle_setpoints(Vector2f center_to_position)
{
	// xy velocity to go around in a circle
	Vector2f velocity_xy(-center_to_position(1), center_to_position(0));
	velocity_xy = velocity_xy.unit_or_zero();
	velocity_xy *= _v;

	// xy velocity adjustment to stay on the radius distance
	velocity_xy += (_r - center_to_position.norm()) * center_to_position.unit_or_zero();

	_velocity_setpoint(0) = velocity_xy(0);
	_velocity_setpoint(1) = velocity_xy(1);
	_position_setpoint(0) = _position_setpoint(1) = NAN;

	// yawspeed feed-forward because we know the necessary angular rate
	_yawspeed_setpoint = _v / _r;
}
