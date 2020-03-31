#ifndef MAVLINK_STREAM_ATT_POS_MOCAP_H
#define MAVLINK_STREAM_ATT_POS_MOCAP_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_odometry.h>

class MavlinkStreamAttPosMocap : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamAttPosMocap::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ATT_POS_MOCAP";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ATT_POS_MOCAP;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamAttPosMocap(mavlink);
	}

	unsigned get_size() override
	{
		return _mocap_sub.advertised() ? MAVLINK_MSG_ID_ATT_POS_MOCAP_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _mocap_sub{ORB_ID(vehicle_mocap_odometry)};

	/* do not allow top copying this class */
	MavlinkStreamAttPosMocap(MavlinkStreamAttPosMocap &) = delete;
	MavlinkStreamAttPosMocap &operator = (const MavlinkStreamAttPosMocap &) = delete;

protected:
	explicit MavlinkStreamAttPosMocap(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_odometry_s mocap;

		if (_mocap_sub.update(&mocap)) {
			mavlink_att_pos_mocap_t msg{};

			msg.time_usec = mocap.timestamp;
			msg.q[0] = mocap.q[0];
			msg.q[1] = mocap.q[1];
			msg.q[2] = mocap.q[2];
			msg.q[3] = mocap.q[3];
			msg.x = mocap.x;
			msg.y = mocap.y;
			msg.z = mocap.z;

			mavlink_msg_att_pos_mocap_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_ATT_POS_MOCAP_H */
