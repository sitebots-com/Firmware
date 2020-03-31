#ifndef MAVLINK_STREAM_DISTANCE_SENSOR_H
#define MAVLINK_STREAM_DISTANCE_SENSOR_H

#include "../mavlink_messages.h"

#include <uORB/topics/distance_sensor.h>

class MavlinkStreamDistanceSensor : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamDistanceSensor::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "DISTANCE_SENSOR";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_DISTANCE_SENSOR;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamDistanceSensor(mavlink);
	}

	unsigned get_size() override
	{
		return _distance_sensor_sub.advertised() ? (MAVLINK_MSG_ID_DISTANCE_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};

	/* do not allow top copying this class */
	MavlinkStreamDistanceSensor(MavlinkStreamDistanceSensor &) = delete;
	MavlinkStreamDistanceSensor &operator = (const MavlinkStreamDistanceSensor &) = delete;

protected:
	explicit MavlinkStreamDistanceSensor(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		distance_sensor_s dist_sensor;

		if (_distance_sensor_sub.update(&dist_sensor)) {
			mavlink_distance_sensor_t msg{};

			msg.time_boot_ms = dist_sensor.timestamp / 1000; /* us to ms */

			switch (dist_sensor.type) {
			case MAV_DISTANCE_SENSOR_ULTRASOUND:
				msg.type = MAV_DISTANCE_SENSOR_ULTRASOUND;
				break;

			case MAV_DISTANCE_SENSOR_LASER:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;

			case MAV_DISTANCE_SENSOR_INFRARED:
				msg.type = MAV_DISTANCE_SENSOR_INFRARED;
				break;

			default:
				msg.type = MAV_DISTANCE_SENSOR_LASER;
				break;
			}

			msg.current_distance = dist_sensor.current_distance * 1e2f; // m to cm
			msg.id               = dist_sensor.id;
			msg.max_distance     = dist_sensor.max_distance * 1e2f;     // m to cm
			msg.min_distance     = dist_sensor.min_distance * 1e2f;     // m to cm
			msg.orientation      = dist_sensor.orientation;
			msg.covariance       = dist_sensor.variance * 1e4f;         // m^2 to cm^2

			mavlink_msg_distance_sensor_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_DISTANCE_SENSOR_H */
