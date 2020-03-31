#ifndef MAVLINK_STREAM_LOCAL_POSITION_NED_H
#define MAVLINK_STREAM_LOCAL_POSITION_NED_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamLocalPositionNED : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamLocalPositionNED::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "LOCAL_POSITION_NED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_LOCAL_POSITION_NED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionNED(mavlink);
	}

	unsigned get_size() override
	{
		return _lpos_sub.advertised() ? MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionNED(MavlinkStreamLocalPositionNED &) = delete;
	MavlinkStreamLocalPositionNED &operator = (const MavlinkStreamLocalPositionNED &) = delete;

protected:
	explicit MavlinkStreamLocalPositionNED(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_local_position_s lpos;

		if (_lpos_sub.update(&lpos)) {
			mavlink_local_position_ned_t msg{};

			msg.time_boot_ms = lpos.timestamp / 1000;
			msg.x = lpos.x;
			msg.y = lpos.y;
			msg.z = lpos.z;
			msg.vx = lpos.vx;
			msg.vy = lpos.vy;
			msg.vz = lpos.vz;

			mavlink_msg_local_position_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_LOCAL_POSITION_NED_H */
