/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"
#include "mavlink_simple_analyzer.h"
#include "mavlink_high_latency2.h"

#include "streams/actuator_control_target.h"
#include "streams/adsb_vehicle.h"
#include "streams/altitude.h"
#include "streams/att_pos_mocap.h"
#include "streams/attitude.h"
#include "streams/attitude_quaternion.h"
#include "streams/attitude_target.h"
#include "streams/battery_status.h"
#include "streams/camera_capture.h"
#include "streams/camera_image_captured.h"
#include "streams/camera_trigger.h"
#include "streams/collision.h"
#include "streams/command_long.h"
#include "streams/debug.h"
#include "streams/debug_float_array.h"
#include "streams/debug_vect.h"
#include "streams/distance_sensor.h"
#include "streams/estimator_status.h"
#include "streams/extended_sys_state.h"
#include "streams/global_position_int.h"
#include "streams/gps_raw_int.h"
#include "streams/gps2_raw.h"
#include "streams/ground_truth.h"
#include "streams/heartbeat.h"
#include "streams/highres_imu.h"
#include "streams/hil_actuator_controls.h"
#include "streams/home_position.h"
#include "streams/local_position_ned.h"
#include "streams/manual_control.h"
#include "streams/mount_orientation.h"
#include "streams/named_value_float.h"
#include "streams/nav_controller_output.h"
#include "streams/obstacle_distance.h"
#include "streams/odometry.h"
#include "streams/optical_flow_rad.h"
#include "streams/orbit_execution_status.h"
#include "streams/ping.h"
#include "streams/position_target_global_int.h"
#include "streams/position_target_local_ned.h"
#include "streams/rc_channels.h"
#include "streams/scaled_imu.h"
#include "streams/scaled_pressure.h"
#include "streams/servo_output_raw.h"
#include "streams/statustext.h"
#include "streams/sys_status.h"
#include "streams/system_time.h"
#include "streams/timesync.h"
#include "streams/trajectory_representation_waypoints.h"
#include "streams/utm_global_position.h"
#include "streams/vibration.h"
#include "streams/vfr_hud.h"
#include "streams/wind_cov.h"

#include <commander/px4_custom_mode.h>
#include <uORB/topics/vehicle_status.h>

uint16_t cm_uint16_from_m_float(float m)
{
	if (m < 0.0f) {
		return 0;

	} else if (m > 655.35f) {
		return 65535;
	}

	return (uint16_t)(m * 100.0f);
}

void get_mavlink_navigation_mode(const struct vehicle_status_s *const status, uint8_t *mavlink_base_mode,
				 union px4_custom_mode *custom_mode)
{
	custom_mode->data = 0;
	*mavlink_base_mode = 0;

	/* HIL */
	if (status->hil_state == vehicle_status_s::HIL_STATE_ON) {
		*mavlink_base_mode |= MAV_MODE_FLAG_HIL_ENABLED;
	}

	/* arming state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/* main state */
	*mavlink_base_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	const uint8_t auto_mode_flags	= MAV_MODE_FLAG_AUTO_ENABLED
					  | MAV_MODE_FLAG_STABILIZE_ENABLED
					  | MAV_MODE_FLAG_GUIDED_ENABLED;

	switch (status->nav_state) {
	case vehicle_status_s::NAVIGATION_STATE_MANUAL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | (status->vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING ? MAV_MODE_FLAG_STABILIZE_ENABLED : 0);
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ACRO:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
		break;

	case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
		break;

	case vehicle_status_s::NAVIGATION_STATE_STAB:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_POSCTL:
		*mavlink_base_mode	|= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
					   | MAV_MODE_FLAG_STABILIZE_ENABLED
					   | MAV_MODE_FLAG_GUIDED_ENABLED; // TODO: is POSCTL GUIDED?
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_ORBIT:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED
				      | MAV_MODE_FLAG_STABILIZE_ENABLED
				      | MAV_MODE_FLAG_GUIDED_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_POSCTL_ORBIT;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_FOLLOW_TARGET:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_FOLLOW_TARGET;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_PRECLAND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_PRECLAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_RTL:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_RTL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_AUTO_LAND:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDENGFAIL:
	case vehicle_status_s::NAVIGATION_STATE_AUTO_LANDGPSFAIL:

	/* fallthrough */
	case vehicle_status_s::NAVIGATION_STATE_DESCEND:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
		custom_mode->sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LAND;
		break;

	case vehicle_status_s::NAVIGATION_STATE_TERMINATION:
		*mavlink_base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
		break;

	case vehicle_status_s::NAVIGATION_STATE_OFFBOARD:
		*mavlink_base_mode |= auto_mode_flags;
		custom_mode->main_mode = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
		break;

	case vehicle_status_s::NAVIGATION_STATE_MAX:
		/* this is an unused case, ignore */
		break;

	}
}

void get_mavlink_mode_state(const struct vehicle_status_s *const status, uint8_t *mavlink_state,
			    uint8_t *mavlink_base_mode, uint32_t *mavlink_custom_mode)
{
	*mavlink_state = 0;
	*mavlink_base_mode = 0;
	*mavlink_custom_mode = 0;

	union px4_custom_mode custom_mode;
	get_mavlink_navigation_mode(status, mavlink_base_mode, &custom_mode);
	*mavlink_custom_mode = custom_mode.data;

	/* set system state */
	if (status->arming_state == vehicle_status_s::ARMING_STATE_INIT
	    || status->arming_state == vehicle_status_s::ARMING_STATE_IN_AIR_RESTORE
	    || status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY_ERROR) {	// TODO review
		*mavlink_state = MAV_STATE_UNINIT;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		*mavlink_state = MAV_STATE_ACTIVE;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_STANDBY) {
		*mavlink_state = MAV_STATE_STANDBY;

	} else if (status->arming_state == vehicle_status_s::ARMING_STATE_SHUTDOWN) {
		*mavlink_state = MAV_STATE_POWEROFF;

	} else {
		*mavlink_state = MAV_STATE_CRITICAL;
	}
}

static const StreamListItem streams_list[] = {
	create_stream_list_item<MavlinkStreamHeartbeat>(),
	create_stream_list_item<MavlinkStreamStatustext>(),
	create_stream_list_item<MavlinkStreamCommandLong>(),
	create_stream_list_item<MavlinkStreamSysStatus>(),
	create_stream_list_item<MavlinkStreamBatteryStatus>(),
	create_stream_list_item<MavlinkStreamHighresIMU>(),
	create_stream_list_item<MavlinkStreamScaledIMU<0> >(),
	create_stream_list_item<MavlinkStreamScaledIMU<1> >(),
	create_stream_list_item<MavlinkStreamScaledIMU<2> >(),
	create_stream_list_item<MavlinkStreamScaledPressure<0> >(),
	// create_stream_list_item<MavlinkStreamScaledPressure<1> >(),
	// create_stream_list_item<MavlinkStreamScaledPressure<2> >(),
	create_stream_list_item<MavlinkStreamAttitude>(),
	create_stream_list_item<MavlinkStreamAttitudeQuaternion>(),
	create_stream_list_item<MavlinkStreamVFRHUD>(),
	create_stream_list_item<MavlinkStreamGPSRawInt>(),
	create_stream_list_item<MavlinkStreamGPS2Raw>(),
	create_stream_list_item<MavlinkStreamSystemTime>(),
	create_stream_list_item<MavlinkStreamTimesync>(),
	create_stream_list_item<MavlinkStreamGlobalPositionInt>(),
	create_stream_list_item<MavlinkStreamLocalPositionNED>(),
	create_stream_list_item<MavlinkStreamOdometry>(),
	create_stream_list_item<MavlinkStreamEstimatorStatus>(),
	create_stream_list_item<MavlinkStreamVibration>(),
	create_stream_list_item<MavlinkStreamAttPosMocap>(),
	create_stream_list_item<MavlinkStreamHomePosition>(),
	create_stream_list_item<MavlinkStreamServoOutputRaw<0> >(),
	create_stream_list_item<MavlinkStreamServoOutputRaw<1> >(),
	create_stream_list_item<MavlinkStreamHILActuatorControls>(),
	create_stream_list_item<MavlinkStreamPositionTargetGlobalInt>(),
	create_stream_list_item<MavlinkStreamLocalPositionSetpoint>(),
	create_stream_list_item<MavlinkStreamAttitudeTarget>(),
	create_stream_list_item<MavlinkStreamRCChannels>(),
	create_stream_list_item<MavlinkStreamManualControl>(),
	create_stream_list_item<MavlinkStreamTrajectoryRepresentationWaypoints>(),
	create_stream_list_item<MavlinkStreamOpticalFlowRad>(),
	create_stream_list_item<MavlinkStreamActuatorControlTarget<0> >(),
	create_stream_list_item<MavlinkStreamActuatorControlTarget<1> >(),
	create_stream_list_item<MavlinkStreamNamedValueFloat>(),
	create_stream_list_item<MavlinkStreamDebug>(),
	create_stream_list_item<MavlinkStreamDebugVect>(),
	create_stream_list_item<MavlinkStreamDebugFloatArray>(),
	create_stream_list_item<MavlinkStreamNavControllerOutput>(),
	create_stream_list_item<MavlinkStreamCameraCapture>(),
	create_stream_list_item<MavlinkStreamCameraTrigger>(),
	create_stream_list_item<MavlinkStreamCameraImageCaptured>(),
	create_stream_list_item<MavlinkStreamDistanceSensor>(),
	create_stream_list_item<MavlinkStreamExtendedSysState>(),
	create_stream_list_item<MavlinkStreamAltitude>(),
	create_stream_list_item<MavlinkStreamADSBVehicle>(),
	create_stream_list_item<MavlinkStreamUTMGlobalPosition>(),
	create_stream_list_item<MavlinkStreamCollision>(),
	create_stream_list_item<MavlinkStreamWind>(),
	create_stream_list_item<MavlinkStreamMountOrientation>(),
	create_stream_list_item<MavlinkStreamHighLatency2>(),
	create_stream_list_item<MavlinkStreamGroundTruth>(),
	create_stream_list_item<MavlinkStreamPing>(),
	create_stream_list_item<MavlinkStreamOrbitStatus>(),
	create_stream_list_item<MavlinkStreamObstacleDistance>()
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}
