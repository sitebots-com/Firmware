#ifndef MAVLINK_STREAM_COLLISION_H
#define MAVLINK_STREAM_COLLISION_H

#include "../mavlink_messages.h"

#include <uORB/topics/collision_report.h>

class MavlinkStreamCollision : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCollision::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "COLLISION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_COLLISION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCollision(mavlink);
	}

	unsigned get_size() override
	{
		return _collision_sub.advertised() ? MAVLINK_MSG_ID_COLLISION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _collision_sub{ORB_ID(collision_report)};

	/* do not allow top copying this class */
	MavlinkStreamCollision(MavlinkStreamCollision &) = delete;
	MavlinkStreamCollision &operator = (const MavlinkStreamCollision &) = delete;

protected:
	explicit MavlinkStreamCollision(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		collision_report_s report;
		bool sent = false;

		while (_collision_sub.update(&report)) {
			mavlink_collision_t msg = {};

			msg.src = report.src;
			msg.id = report.id;
			msg.action = report.action;
			msg.threat_level = report.threat_level;
			msg.time_to_minimum_delta = report.time_to_minimum_delta;
			msg.altitude_minimum_delta = report.altitude_minimum_delta;
			msg.horizontal_minimum_delta = report.horizontal_minimum_delta;

			mavlink_msg_collision_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

		return sent;
	}
};

#endif /* MAVLINK_STREAM_COLLISION_H */
