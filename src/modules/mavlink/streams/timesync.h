#ifndef MAVLINK_STREAM_TIMESYNC_H
#define MAVLINK_STREAM_TIMESYNC_H

#include "../mavlink_messages.h"

class MavlinkStreamTimesync : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamTimesync::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "TIMESYNC";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_TIMESYNC;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamTimesync(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_TIMESYNC_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamTimesync(MavlinkStreamTimesync &) = delete;
	MavlinkStreamTimesync &operator = (const MavlinkStreamTimesync &) = delete;

protected:
	explicit MavlinkStreamTimesync(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		mavlink_timesync_t msg{};

		msg.tc1 = 0;
		msg.ts1 = hrt_absolute_time() * 1000; // boot time in nanoseconds

		mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &msg);

		return true;
	}
};

#endif /* MAVLINK_STREAM_TIMESYNC_H */
