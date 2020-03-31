#ifndef MAVLINK_STREAM_DEBUG_VECT_H
#define MAVLINK_STREAM_DEBUG_VECT_H

#include "../mavlink_messages.h"

#include <uORB/topics/debug_vect.h>

class MavlinkStreamDebugVect : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDebugVect::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DEBUG_VECT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DEBUG_VECT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDebugVect(mavlink);
	}

	unsigned get_size() override
	{
		return _debug_sub.advertised() ? MAVLINK_MSG_ID_DEBUG_VECT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _debug_sub{ORB_ID(debug_vect)};

	/* do not allow top copying this class */
	MavlinkStreamDebugVect(MavlinkStreamDebugVect &) = delete;
	MavlinkStreamDebugVect &operator = (const MavlinkStreamDebugVect &) = delete;

protected:
	explicit MavlinkStreamDebugVect(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		debug_vect_s debug;

		if (_debug_sub.update(&debug)) {
			mavlink_debug_vect_t msg{};

			msg.time_usec = debug.timestamp;
			memcpy(msg.name, debug.name, sizeof(msg.name));
			/* enforce null termination */
			msg.name[sizeof(msg.name) - 1] = '\0';
			msg.x = debug.x;
			msg.y = debug.y;
			msg.z = debug.z;

			mavlink_msg_debug_vect_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_DEBUG_VECT_H */
