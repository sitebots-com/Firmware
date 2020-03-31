#ifndef MAVLINK_STREAM_MANUAL_CONTROL_H
#define MAVLINK_STREAM_MANUAL_CONTROL_H

#include "../mavlink_messages.h"

#include <uORB/topics/manual_control_setpoint.h>

class MavlinkStreamManualControl : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamManualControl::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "MANUAL_CONTROL";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_MANUAL_CONTROL;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamManualControl(mavlink);
	}

	unsigned get_size() override
	{
		return _manual_sub.advertised() ? (MAVLINK_MSG_ID_MANUAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _manual_sub{ORB_ID(manual_control_setpoint)};

	/* do not allow top copying this class */
	MavlinkStreamManualControl(MavlinkStreamManualControl &) = delete;
	MavlinkStreamManualControl &operator = (const MavlinkStreamManualControl &) = delete;

protected:
	explicit MavlinkStreamManualControl(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		manual_control_setpoint_s manual;

		if (_manual_sub.update(&manual)) {
			mavlink_manual_control_t msg{};

			msg.target = mavlink_system.sysid;
			msg.x = manual.x * 1000;
			msg.y = manual.y * 1000;
			msg.z = manual.z * 1000;
			msg.r = manual.r * 1000;
			unsigned shift = 2;
			msg.buttons = 0;
			msg.buttons |= (manual.mode_switch << (shift * 0));
			msg.buttons |= (manual.return_switch << (shift * 1));
			msg.buttons |= (manual.posctl_switch << (shift * 2));
			msg.buttons |= (manual.loiter_switch << (shift * 3));
			msg.buttons |= (manual.acro_switch << (shift * 4));
			msg.buttons |= (manual.offboard_switch << (shift * 5));

			mavlink_msg_manual_control_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_MANUAL_CONTROL_H */
