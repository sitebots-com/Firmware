#ifndef MAVLINK_STREAM_MOUNT_ORIENTATION_H
#define MAVLINK_STREAM_MOUNT_ORIENTATION_H

#include "../mavlink_messages.h"

#include <uORB/topics/mount_orientation.h>
#include <uORB/topics/vehicle_local_position.h>

class MavlinkStreamMountOrientation : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamMountOrientation::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "MOUNT_ORIENTATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MOUNT_ORIENTATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamMountOrientation(mavlink);
	}

	unsigned get_size() override
	{
		return _mount_orientation_sub.advertised() ? MAVLINK_MSG_ID_MOUNT_ORIENTATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _mount_orientation_sub{ORB_ID(mount_orientation)};
	uORB::Subscription _lpos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamMountOrientation(MavlinkStreamMountOrientation &) = delete;
	MavlinkStreamMountOrientation &operator = (const MavlinkStreamMountOrientation &) = delete;

protected:
	explicit MavlinkStreamMountOrientation(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		mount_orientation_s mount_orientation;

		if (_mount_orientation_sub.update(&mount_orientation)) {
			mavlink_mount_orientation_t msg{};

			msg.roll = math::degrees(mount_orientation.attitude_euler_angle[0]);
			msg.pitch = math::degrees(mount_orientation.attitude_euler_angle[1]);
			msg.yaw = math::degrees(mount_orientation.attitude_euler_angle[2]);

			vehicle_local_position_s lpos{};
			_lpos_sub.copy(&lpos);
			msg.yaw_absolute = math::degrees(matrix::wrap_2pi(lpos.yaw + mount_orientation.attitude_euler_angle[2]));

			mavlink_msg_mount_orientation_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_MOUNT_ORIENTATION_H */
