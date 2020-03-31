#ifndef MAVLINK_STREAM_CAMERA_TRIGGER_H
#define MAVLINK_STREAM_CAMERA_TRIGGER_H

#include "../mavlink_messages.h"

#include <uORB/topics/camera_trigger.h>

class MavlinkStreamCameraTrigger : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraTrigger::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_TRIGGER";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_TRIGGER;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraTrigger(mavlink);
	}

	bool const_rate() override
	{
		return true;
	}

	unsigned get_size() override
	{
		return _trigger_sub.advertised() ? MAVLINK_MSG_ID_CAMERA_TRIGGER_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _trigger_sub{ORB_ID(camera_trigger)};

	/* do not allow top copying this class */
	MavlinkStreamCameraTrigger(MavlinkStreamCameraTrigger &) = delete;
	MavlinkStreamCameraTrigger &operator = (const MavlinkStreamCameraTrigger &) = delete;

protected:
	explicit MavlinkStreamCameraTrigger(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		camera_trigger_s trigger;

		if (_trigger_sub.update(&trigger)) {
			mavlink_camera_trigger_t msg{};

			msg.time_usec = trigger.timestamp;
			msg.seq = trigger.seq;

			/* ensure that only active trigger events are sent */
			if (trigger.timestamp > 0) {

				mavlink_msg_camera_trigger_send_struct(_mavlink->get_channel(), &msg);

				vehicle_command_s vcmd{};
				vcmd.timestamp = hrt_absolute_time();
				vcmd.param1 = 0.0f; // all cameras
				vcmd.param2 = 0.0f; // duration 0 because only taking one picture
				vcmd.param3 = 1.0f; // only take one
				vcmd.param4 = NAN;
				vcmd.param5 = (double)NAN;
				vcmd.param6 = (double)NAN;
				vcmd.param7 = NAN;
				vcmd.command = MAV_CMD_IMAGE_START_CAPTURE;
				vcmd.target_system = mavlink_system.sysid;
				vcmd.target_component = MAV_COMP_ID_CAMERA;

				MavlinkCommandSender::instance().handle_vehicle_command(vcmd, _mavlink->get_channel());

				// TODO: move this camera_trigger and publish as a vehicle_command
				/* send MAV_CMD_DO_DIGICAM_CONTROL*/
				mavlink_command_long_t digicam_ctrl_cmd{};

				digicam_ctrl_cmd.target_system = 0; // 0 for broadcast
				digicam_ctrl_cmd.target_component = MAV_COMP_ID_CAMERA;
				digicam_ctrl_cmd.command = MAV_CMD_DO_DIGICAM_CONTROL;
				digicam_ctrl_cmd.confirmation = 0;
				digicam_ctrl_cmd.param1 = NAN;
				digicam_ctrl_cmd.param2 = NAN;
				digicam_ctrl_cmd.param3 = NAN;
				digicam_ctrl_cmd.param4 = NAN;
				digicam_ctrl_cmd.param5 = 1;   // take 1 picture
				digicam_ctrl_cmd.param6 = NAN;
				digicam_ctrl_cmd.param7 = NAN;

				mavlink_msg_command_long_send_struct(_mavlink->get_channel(), &digicam_ctrl_cmd);

				return true;
			}
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_CAMERA_TRIGGER_H */
