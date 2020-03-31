#ifndef MAVLINK_STREAM_ATTITUDE_H
#define MAVLINK_STREAM_ATTITUDE_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>

#include <lib/matrix/matrix/math.hpp>

class MavlinkStreamAttitude : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttitude::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATTITUDE";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitude(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	/* do not allow top copying this class */
	MavlinkStreamAttitude(MavlinkStreamAttitude &) = delete;
	MavlinkStreamAttitude &operator = (const MavlinkStreamAttitude &) = delete;


protected:
	explicit MavlinkStreamAttitude(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_attitude_s att;

		if (_att_sub.update(&att)) {
			vehicle_angular_velocity_s angular_velocity{};
			_angular_velocity_sub.copy(&angular_velocity);

			mavlink_attitude_t msg{};

			const matrix::Eulerf euler = matrix::Quatf(att.q);
			msg.time_boot_ms = att.timestamp / 1000;
			msg.roll = euler.phi();
			msg.pitch = euler.theta();
			msg.yaw = euler.psi();

			msg.rollspeed = angular_velocity.xyz[0];
			msg.pitchspeed = angular_velocity.xyz[1];
			msg.yawspeed = angular_velocity.xyz[2];

			mavlink_msg_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_ATTITUDE_H */
