#ifndef MAVLINK_STREAM_DEBUG_H
#define MAVLINK_STREAM_DEBUG_H

#include "../mavlink_messages.h"

#include <uORB/topics/debug_value.h>

class MavlinkStreamDebug : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDebug::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DEBUG";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebug(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_sub.advertised() ? MAVLINK_MSG_ID_DEBUG_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_sub{ORB_ID(debug_value)};

	/* do not allow top copying this class */
	MavlinkStreamDebug(MavlinkStreamDebug &) = delete;
	MavlinkStreamDebug &operator = (const MavlinkStreamDebug &) = delete;

protected:
	explicit MavlinkStreamDebug(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_value_s debug;

		if (_debug_sub.update(&debug)) {
			mavlink_debug_t msg{};
			msg.time_boot_ms = debug.timestamp / 1000ULL;
			msg.ind = debug.ind;
			msg.value = debug.value;

			mavlink_msg_debug_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_DEBUG_H */
