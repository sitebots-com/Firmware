#ifndef MAVLINK_STREAM_ATTITUDE_QUATERNION_H
#define MAVLINK_STREAM_ATTITUDE_QUATERNION_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>

#include <lib/conversion/rotation.h>
#include <lib/matrix/matrix/math.hpp>

class MavlinkStreamAttitudeQuaternion : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttitudeQuaternion::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATTITUDE_QUATERNION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_QUATERNION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeQuaternion(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_QUATERNION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamAttitudeQuaternion(MavlinkStreamAttitudeQuaternion &) = delete;
	MavlinkStreamAttitudeQuaternion &operator = (const MavlinkStreamAttitudeQuaternion &) = delete;

protected:
	explicit MavlinkStreamAttitudeQuaternion(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			vehicle_status_s status{};
			_status_sub.copy(&status);

			mavlink_attitude_quaternion_t msg{};

			msg.time_boot_ms = att.timestamp / 1000;
			msg.q1 = att.q[0];
			msg.q2 = att.q[1];
			msg.q3 = att.q[2];
			msg.q4 = att.q[3];
			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			if (status.is_vtol && status.is_vtol_tailsitter && (status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
				// This is a tailsitter VTOL flying in fixed wing mode:
				// indicate that reported attitude should be rotated by
				// 90 degrees upward pitch for user display
				get_rot_quaternion(ROTATION_PITCH_90).copyTo(msg.repr_offset_q);

			} else {
				// Normal case
				// zero rotation should be [1 0 0 0]:
				// `get_rot_quaternion(ROTATION_NONE).copyTo(msg.repr_offset_q);`
				// but to save bandwidth, we instead send [0, 0, 0, 0].
				msg.repr_offset_q[0] = 0.0f;
				msg.repr_offset_q[1] = 0.0f;
				msg.repr_offset_q[2] = 0.0f;
				msg.repr_offset_q[3] = 0.0f;
			}

			mavlink_msg_attitude_quaternion_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_ATTITUDE_QUATERNION_H */
