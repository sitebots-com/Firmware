#ifndef MAVLINK_STREAM_POSITION_TARGET_LOCAL_NED_H
#define MAVLINK_STREAM_POSITION_TARGET_LOCAL_NED_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_local_position_setpoint.h>

class MavlinkStreamLocalPositionSetpoint : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamLocalPositionSetpoint::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "POSITION_TARGET_LOCAL_NED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamLocalPositionSetpoint(mavlink);
	}

	unsigned get_size() override
	{
		return _pos_sp_sub.advertised() ? MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};

	/* do not allow top copying this class */
	MavlinkStreamLocalPositionSetpoint(MavlinkStreamLocalPositionSetpoint &) = delete;
	MavlinkStreamLocalPositionSetpoint &operator = (const MavlinkStreamLocalPositionSetpoint &) = delete;

protected:
	explicit MavlinkStreamLocalPositionSetpoint(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_local_position_setpoint_s pos_sp;

		if (_pos_sp_sub.update(&pos_sp)) {
			mavlink_position_target_local_ned_t msg{};

			msg.time_boot_ms = pos_sp.timestamp / 1000;
			msg.coordinate_frame = MAV_FRAME_LOCAL_NED;
			msg.x = pos_sp.x;
			msg.y = pos_sp.y;
			msg.z = pos_sp.z;
			msg.yaw = pos_sp.yaw;
			msg.yaw_rate = pos_sp.yawspeed;
			msg.vx = pos_sp.vx;
			msg.vy = pos_sp.vy;
			msg.vz = pos_sp.vz;
			msg.afx = pos_sp.acceleration[0];
			msg.afy = pos_sp.acceleration[1];
			msg.afz = pos_sp.acceleration[2];

			mavlink_msg_position_target_local_ned_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_POSITION_TARGET_LOCAL_NED_H */
