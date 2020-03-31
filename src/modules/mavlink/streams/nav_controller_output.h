#ifndef MAVLINK_STREAM_NAV_CONTROLLER_OUTPUT_H
#define MAVLINK_STREAM_NAV_CONTROLLER_OUTPUT_H

#include "../mavlink_messages.h"

#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>

class MavlinkStreamNavControllerOutput : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamNavControllerOutput::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "NAV_CONTROLLER_OUTPUT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamNavControllerOutput(mavlink);
	}

	unsigned get_size() override
	{
		return (_pos_ctrl_status_sub.advertised()) ?
		       MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _pos_ctrl_status_sub{ORB_ID(position_controller_status)};
	uORB::Subscription _tecs_status_sub{ORB_ID(tecs_status)};

	/* do not allow top copying this class */
	MavlinkStreamNavControllerOutput(MavlinkStreamNavControllerOutput &) = delete;
	MavlinkStreamNavControllerOutput &operator = (const MavlinkStreamNavControllerOutput &) = delete;

protected:
	explicit MavlinkStreamNavControllerOutput(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_pos_ctrl_status_sub.updated()) {

			position_controller_status_s pos_ctrl_status{};
			_pos_ctrl_status_sub.copy(&pos_ctrl_status);

			tecs_status_s tecs_status{};
			_tecs_status_sub.copy(&tecs_status);

			mavlink_nav_controller_output_t msg{};

			msg.nav_roll = math::degrees(pos_ctrl_status.nav_roll);
			msg.nav_pitch = math::degrees(pos_ctrl_status.nav_pitch);
			msg.nav_bearing = (int16_t)math::degrees(pos_ctrl_status.nav_bearing);
			msg.target_bearing = (int16_t)math::degrees(pos_ctrl_status.target_bearing);
			msg.wp_dist = (uint16_t)pos_ctrl_status.wp_dist;
			msg.xtrack_error = pos_ctrl_status.xtrack_error;
			msg.alt_error = tecs_status.altitude_filtered - tecs_status.altitude_sp;
			msg.aspd_error = tecs_status.airspeed_filtered - tecs_status.airspeed_sp;

			mavlink_msg_nav_controller_output_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_NAV_CONTROLLER_OUTPUT_H */
