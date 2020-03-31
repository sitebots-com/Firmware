#ifndef MAVLINK_STREAM_NAMED_VALUE_FLOAT_H
#define MAVLINK_STREAM_NAMED_VALUE_FLOAT_H

#include "../mavlink_messages.h"

#include <uORB/topics/debug_key_value.h>

class MavlinkStreamNamedValueFloat : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamNamedValueFloat::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "NAMED_VALUE_FLOAT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAMED_VALUE_FLOAT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNamedValueFloat(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_sub.advertised() ? MAVLINK_MSG_ID_NAMED_VALUE_FLOAT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_sub{ORB_ID(debug_key_value)};

	/* do not allow top copying this class */
	MavlinkStreamNamedValueFloat(MavlinkStreamNamedValueFloat &) = delete;
	MavlinkStreamNamedValueFloat &operator = (const MavlinkStreamNamedValueFloat &) = delete;

protected:
	explicit MavlinkStreamNamedValueFloat(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_key_value_s debug;

		if (_debug_sub.update(&debug)) {
			mavlink_named_value_float_t msg{};

			msg.time_boot_ms = debug.timestamp / 1000ULL;
			memcpy(msg.name, debug.key, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.value = debug.value;

			mavlink_msg_named_value_float_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_NAMED_VALUE_FLOAT_H */
