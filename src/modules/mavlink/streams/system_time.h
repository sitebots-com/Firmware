#ifndef MAVLINK_STREAM_SYSTEM_TIME_H
#define MAVLINK_STREAM_SYSTEM_TIME_H

#include "../mavlink_messages.h"

#include <px4_platform_common/time.h>

class MavlinkStreamSystemTime : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamSystemTime::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "SYSTEM_TIME";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamSystemTime(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_SYSTEM_TIME_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamSystemTime(MavlinkStreamSystemTime &) = delete;
	MavlinkStreamSystemTime &operator = (const MavlinkStreamSystemTime &) = delete;

protected:
	explicit MavlinkStreamSystemTime(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		timespec tv;
		px4_clock_gettime(CLOCK_REALTIME, &tv);

		mavlink_system_time_t msg{};
		msg.time_boot_ms = hrt_absolute_time() / 1000;
		msg.time_unix_usec = (uint64_t)tv.tv_sec * 1000000 + tv.tv_nsec / 1000;

		// If the time is before 2001-01-01, it's probably the default 2000
		// and we don't need to bother sending it because it's definitely wrong.
		if (msg.time_unix_usec > 978307200000000) {
			mavlink_msg_system_time_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_SYSTEM_TIME_H */
