#ifndef MAVLINK_STREAM_DEBUG_FLOAT_ARRAY_H
#define MAVLINK_STREAM_DEBUG_FLOAT_ARRAY_H

#include "../mavlink_messages.h"

#include <uORB/topics/debug_array.h>

class MavlinkStreamDebugFloatArray : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDebugFloatArray::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DEBUG_FLOAT_ARRAY";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugFloatArray(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_array_sub.advertised() ? MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_array_sub{ORB_ID(debug_array)};

	/* do not allow top copying this class */
	MavlinkStreamDebugFloatArray(MavlinkStreamDebugFloatArray &);
	MavlinkStreamDebugFloatArray &operator = (const MavlinkStreamDebugFloatArray &);

protected:
	explicit MavlinkStreamDebugFloatArray(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_array_s debug;

		if (_debug_array_sub.update(&debug)) {
			mavlink_debug_float_array_t msg{};

			msg.time_usec = debug.timestamp;
			msg.array_id = debug.id;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';

			for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
				msg.data[i] = debug.data[i];
			}

			mavlink_msg_debug_float_array_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_DEBUG_FLOAT_ARRAY_H */
