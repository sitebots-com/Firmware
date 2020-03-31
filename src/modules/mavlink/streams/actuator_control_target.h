#ifndef MAVLINK_STREAM_ACTUATOR_CONTROL_TARGET_H
#define MAVLINK_STREAM_ACTUATOR_CONTROL_TARGET_H

#include "../mavlink_messages.h"

#include <uORB/topics/actuator_controls.h>

template <int N>
class MavlinkStreamActuatorControlTarget : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamActuatorControlTarget<N>::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "ACTUATOR_CONTROL_TARGET0";

		case 1:
			return "ACTUATOR_CONTROL_TARGET1";

		case 2:
			return "ACTUATOR_CONTROL_TARGET2";

		case 3:
			return "ACTUATOR_CONTROL_TARGET3";
		}
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamActuatorControlTarget<N>(mavlink);
	}

	unsigned get_size() override
	{
		return (_act_ctrl_sub
			&& _act_ctrl_sub->advertised()) ? (MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription *_act_ctrl_sub{nullptr};

	/* do not allow top copying this class */
	MavlinkStreamActuatorControlTarget(MavlinkStreamActuatorControlTarget &) = delete;
	MavlinkStreamActuatorControlTarget &operator = (const MavlinkStreamActuatorControlTarget &) = delete;

protected:
	explicit MavlinkStreamActuatorControlTarget(Mavlink *mavlink) : MavlinkStream(mavlink)
	{
		// XXX this can be removed once the multiplatform system remaps topics
		switch (N) {
		case 0:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_0)};
			break;

		case 1:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_1)};
			break;

		case 2:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_2)};
			break;

		case 3:
			_act_ctrl_sub = new uORB::Subscription{ORB_ID(actuator_controls_3)};
			break;
		}
	}

	~MavlinkStreamActuatorControlTarget() override
	{
		delete _act_ctrl_sub;
	}

	bool send(const hrt_abstime t) override
	{
		actuator_controls_s act_ctrl;

		if (_act_ctrl_sub && _act_ctrl_sub->update(&act_ctrl)) {
			mavlink_actuator_control_target_t msg{};

			msg.time_usec = act_ctrl.timestamp;
			msg.group_mlx = N;

			for (unsigned i = 0; i < sizeof(msg.controls) / sizeof(msg.controls[0]); i++) {
				msg.controls[i] = act_ctrl.control[i];
			}

			mavlink_msg_actuator_control_target_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_ACTUATOR_CONTROL_TARGET_H */
