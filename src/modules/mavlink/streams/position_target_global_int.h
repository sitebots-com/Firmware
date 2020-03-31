#ifndef MAVLINK_STREAM_POSITION_TARGET_GLOBAL_INT_H
#define MAVLINK_STREAM_POSITION_TARGET_GLOBAL_INT_H

#include "../mavlink_messages.h"

#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

class MavlinkStreamPositionTargetGlobalInt : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamPositionTargetGlobalInt::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "POSITION_TARGET_GLOBAL_INT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamPositionTargetGlobalInt(mavlink);
	}

	unsigned get_size() override
	{
		return _pos_sp_triplet_sub.advertised() ? MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT_LEN +
		       MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _lpos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
	uORB::Subscription _pos_sp_triplet_sub{ORB_ID(position_setpoint_triplet)};

	/* do not allow top copying this class */
	MavlinkStreamPositionTargetGlobalInt(MavlinkStreamPositionTargetGlobalInt &) = delete;
	MavlinkStreamPositionTargetGlobalInt &operator = (const MavlinkStreamPositionTargetGlobalInt &) = delete;

protected:
	explicit MavlinkStreamPositionTargetGlobalInt(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		vehicle_control_mode_s control_mode{};
		_control_mode_sub.copy(&control_mode);

		if (control_mode.flag_control_position_enabled) {

			position_setpoint_triplet_s pos_sp_triplet{};
			_pos_sp_triplet_sub.copy(&pos_sp_triplet);

			if (pos_sp_triplet.timestamp > 0 && pos_sp_triplet.current.valid && PX4_ISFINITE(pos_sp_triplet.current.lat)
			    && PX4_ISFINITE(pos_sp_triplet.current.lon)) {

				mavlink_position_target_global_int_t msg{};

				msg.time_boot_ms = hrt_absolute_time() / 1000;
				msg.coordinate_frame = MAV_FRAME_GLOBAL_INT;
				msg.lat_int = pos_sp_triplet.current.lat * 1e7;
				msg.lon_int = pos_sp_triplet.current.lon * 1e7;
				msg.alt = pos_sp_triplet.current.alt;

				vehicle_local_position_setpoint_s lpos_sp;

				if (_lpos_sp_sub.copy(&lpos_sp)) {
					// velocity
					msg.vx = lpos_sp.vx;
					msg.vy = lpos_sp.vy;
					msg.vz = lpos_sp.vz;

					// acceleration
					msg.afx = lpos_sp.acceleration[0];
					msg.afy = lpos_sp.acceleration[1];
					msg.afz = lpos_sp.acceleration[2];

					// yaw
					msg.yaw = lpos_sp.yaw;
					msg.yaw_rate = lpos_sp.yawspeed;
				}

				mavlink_msg_position_target_global_int_send_struct(_mavlink->get_channel(), &msg);

				return true;
			}
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_POSITION_TARGET_GLOBAL_INT_H */
