#ifndef MAVLINK_STREAM_ATTITUDE_TARGET_H
#define MAVLINK_STREAM_ATTITUDE_TARGET_H

#include "../mavlink_messages.h"

#include <lib/matrix/matrix/math.hpp>

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

class MavlinkStreamAttitudeTarget : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttitudeTarget::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATTITUDE_TARGET";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATTITUDE_TARGET;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttitudeTarget(mavlink);
	}

	unsigned get_size() override
	{
		return _att_sp_sub.advertised() ? MAVLINK_MSG_ID_ATTITUDE_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _att_sp_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _att_rates_sp_sub{ORB_ID(vehicle_rates_setpoint)};

	/* do not allow top copying this class */
	MavlinkStreamAttitudeTarget(MavlinkStreamAttitudeTarget &) = delete;
	MavlinkStreamAttitudeTarget &operator = (const MavlinkStreamAttitudeTarget &) = delete;

protected:
	explicit MavlinkStreamAttitudeTarget(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_attitude_setpoint_s att_sp;

		if (_att_sp_sub.update(&att_sp)) {

			mavlink_attitude_target_t msg{};

			msg.time_boot_ms = att_sp.timestamp / 1000;
			matrix::Quatf(att_sp.q_d).copyTo(msg.q);

			vehicle_rates_setpoint_s att_rates_sp{};
			_att_rates_sp_sub.copy(&att_rates_sp);

			msg.body_roll_rate = att_rates_sp.roll;
			msg.body_pitch_rate = att_rates_sp.pitch;
			msg.body_yaw_rate = att_rates_sp.yaw;

			msg.thrust = att_sp.thrust_body[0];

			mavlink_msg_attitude_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_ATTITUDE_TARGET_H */
