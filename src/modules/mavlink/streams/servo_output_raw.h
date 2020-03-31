#ifndef MAVLINK_STREAM_SERVO_OUTPUT_RAW_H
#define MAVLINK_STREAM_SERVO_OUTPUT_RAW_H

#include "../mavlink_messages.h"

#include <uORB/topics/actuator_outputs.h>

template <int N>
class MavlinkStreamServoOutputRaw : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamServoOutputRaw<N>::get_name_static();
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static constexpr const char *get_name_static()
	{
		switch (N) {
		case 0:
			return "SERVO_OUTPUT_RAW_0";

		case 1:
			return "SERVO_OUTPUT_RAW_1";
		}
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamServoOutputRaw<N>(mavlink);
	}

	unsigned get_size() override
	{
		return _act_sub.advertised() ? MAVLINK_MSG_ID_SERVO_OUTPUT_RAW_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _act_sub{ORB_ID(actuator_outputs), N};

	/* do not allow top copying this class */
	MavlinkStreamServoOutputRaw(MavlinkStreamServoOutputRaw &) = delete;
	MavlinkStreamServoOutputRaw &operator = (const MavlinkStreamServoOutputRaw &) = delete;

protected:
	explicit MavlinkStreamServoOutputRaw(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		actuator_outputs_s act;

		if (_act_sub.update(&act)) {
			mavlink_servo_output_raw_t msg{};

			static_assert(sizeof(act.output) / sizeof(act.output[0]) >= 16, "mavlink message requires at least 16 outputs");

			msg.time_usec = act.timestamp;
			msg.port = N;
			msg.servo1_raw = act.output[0];
			msg.servo2_raw = act.output[1];
			msg.servo3_raw = act.output[2];
			msg.servo4_raw = act.output[3];
			msg.servo5_raw = act.output[4];
			msg.servo6_raw = act.output[5];
			msg.servo7_raw = act.output[6];
			msg.servo8_raw = act.output[7];
			msg.servo9_raw = act.output[8];
			msg.servo10_raw = act.output[9];
			msg.servo11_raw = act.output[10];
			msg.servo12_raw = act.output[11];
			msg.servo13_raw = act.output[12];
			msg.servo14_raw = act.output[13];
			msg.servo15_raw = act.output[14];
			msg.servo16_raw = act.output[15];

			mavlink_msg_servo_output_raw_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_SERVO_OUTPUT_RAW_H */
