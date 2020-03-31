#ifndef MAVLINK_STREAM_PING_H
#define MAVLINK_STREAM_PING_H

#include "../mavlink_messages.h"

class MavlinkStreamPing : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamPing::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "PING";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_PING;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPing(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_PING_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate() override
	{
		return true;
	}

private:
	uint32_t _sequence;

	/* do not allow top copying this class */
	MavlinkStreamPing(MavlinkStreamPing &) = delete;
	MavlinkStreamPing &operator = (const MavlinkStreamPing &) = delete;

protected:
	explicit MavlinkStreamPing(Mavlink *mavlink) : MavlinkStream(mavlink),
		_sequence(0)
	{}

	bool send(const hrt_abstime t) override
	{
		mavlink_ping_t msg = {};

		msg.time_usec = hrt_absolute_time();
		msg.seq = _sequence++;
		msg.target_system = 0; // All systems
		msg.target_component = 0; // All components

		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif /* MAVLINK_STREAM_PING_H */
