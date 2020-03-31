#ifndef MAVLINK_STREAM_OPTICAL_FLOW_RAD_H
#define MAVLINK_STREAM_OPTICAL_FLOW_RAD_H

#include "../mavlink_messages.h"

#include <uORB/topics/optical_flow.h>

class MavlinkStreamOpticalFlowRad : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamOpticalFlowRad::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "OPTICAL_FLOW_RAD";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_OPTICAL_FLOW_RAD;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamOpticalFlowRad(mavlink);
	}

	unsigned get_size() override
	{
		return _flow_sub.advertised() ? (MAVLINK_MSG_ID_OPTICAL_FLOW_RAD_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _flow_sub{ORB_ID(optical_flow)};

	/* do not allow top copying this class */
	MavlinkStreamOpticalFlowRad(MavlinkStreamOpticalFlowRad &) = delete;
	MavlinkStreamOpticalFlowRad &operator = (const MavlinkStreamOpticalFlowRad &) = delete;

protected:
	explicit MavlinkStreamOpticalFlowRad(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		optical_flow_s flow;

		if (_flow_sub.update(&flow)) {
			mavlink_optical_flow_rad_t msg{};

			msg.time_usec = flow.timestamp;
			msg.sensor_id = flow.sensor_id;
			msg.integrated_x = flow.pixel_flow_x_integral;
			msg.integrated_y = flow.pixel_flow_y_integral;
			msg.integrated_xgyro = flow.gyro_x_rate_integral;
			msg.integrated_ygyro = flow.gyro_y_rate_integral;
			msg.integrated_zgyro = flow.gyro_z_rate_integral;
			msg.distance = flow.ground_distance_m;
			msg.quality = flow.quality;
			msg.integration_time_us = flow.integration_timespan;
			msg.sensor_id = flow.sensor_id;
			msg.time_delta_distance_us = flow.time_since_last_sonar_update;
			msg.temperature = flow.gyro_temperature;

			mavlink_msg_optical_flow_rad_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_OPTICAL_FLOW_RAD_H */
