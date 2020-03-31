#ifndef MAVLINK_STREAM_HOME_POSITION_H
#define MAVLINK_STREAM_HOME_POSITION_H

#include "../mavlink_messages.h"

#include <uORB/topics/home_position.h>

class MavlinkStreamHomePosition : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHomePosition::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HOME_POSITION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HOME_POSITION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHomePosition(mavlink);
	}

	unsigned get_size() override
	{
		return _home_sub.advertised() ? (MAVLINK_MSG_ID_HOME_POSITION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _home_sub{ORB_ID(home_position)};

	/* do not allow top copying this class */
	MavlinkStreamHomePosition(MavlinkStreamHomePosition &) = delete;
	MavlinkStreamHomePosition &operator = (const MavlinkStreamHomePosition &) = delete;

protected:
	explicit MavlinkStreamHomePosition(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		/* we're sending the GPS home periodically to ensure the
		 * the GCS does pick it up at one point */
		home_position_s home;

		if (_home_sub.advertised() && _home_sub.copy(&home)) {
			if (home.valid_hpos) {
				mavlink_home_position_t msg{};

				msg.latitude = home.lat * 1e7;
				msg.longitude = home.lon * 1e7;
				msg.altitude = home.alt * 1e3f;

				msg.x = home.x;
				msg.y = home.y;
				msg.z = home.z;

				matrix::Quatf q(matrix::Eulerf(0.0f, 0.0f, home.yaw));
				msg.q[0] = q(0);
				msg.q[1] = q(1);
				msg.q[2] = q(2);
				msg.q[3] = q(3);

				msg.approach_x = 0.0f;
				msg.approach_y = 0.0f;
				msg.approach_z = 0.0f;

				msg.time_usec = home.timestamp;

				mavlink_msg_home_position_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_HOME_POSITION_H */
