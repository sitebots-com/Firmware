#ifndef MAVLINK_STREAM_HEARTBEAT_H
#define MAVLINK_STREAM_HEARTBEAT_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_status.h>

class MavlinkStreamHeartbeat : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHeartbeat::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HEARTBEAT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HEARTBEAT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHeartbeat(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HEARTBEAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() override
	{
		return true;
	}

private:
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};

	/* do not allow top copying this class */
	MavlinkStreamHeartbeat(MavlinkStreamHeartbeat &) = delete;
	MavlinkStreamHeartbeat &operator = (const MavlinkStreamHeartbeat &) = delete;

protected:
	explicit MavlinkStreamHeartbeat(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		// always send the heartbeat, independent of the update status of the topics
		vehicle_status_s status{};
		_status_sub.copy(&status);

		uint8_t base_mode = 0;
		uint32_t custom_mode = 0;
		uint8_t system_status = 0;
		get_mavlink_mode_state(&status, &system_status, &base_mode, &custom_mode);

		mavlink_msg_heartbeat_send(_mavlink->get_channel(), _mavlink->get_system_type(), MAV_AUTOPILOT_PX4,
					   base_mode, custom_mode, system_status);

		return true;
	}
};

#endif /* MAVLINK_STREAM_HEARTBEAT_H */
