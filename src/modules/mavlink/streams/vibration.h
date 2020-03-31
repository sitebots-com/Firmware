#ifndef MAVLINK_STREAM_VIBRATION_H
#define MAVLINK_STREAM_VIBRATION_H

#include "../mavlink_messages.h"

#include <uORB/topics/sensor_accel_status.h>
#include <uORB/topics/sensor_gyro_status.h>
#include <uORB/topics/sensor_selection.h>

class MavlinkStreamVibration : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamVibration::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "VIBRATION";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_VIBRATION;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamVibration(mavlink);
	}

	unsigned get_size() override
	{
		const unsigned size = MAVLINK_MSG_ID_VIBRATION_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

		if (_sensor_selection_sub.advertised()) {
			return size;
		}

		for (auto &x : _sensor_accel_status_sub) {
			if (x.advertised()) {
				return size;
			}
		}

		for (auto &x : _sensor_gyro_status_sub) {
			if (x.advertised()) {
				return size;
			}
		}

		return 0;
	}

private:
	uORB::Subscription _sensor_selection_sub{ORB_ID(sensor_selection)};

	uORB::Subscription _sensor_accel_status_sub[3] {
		{ORB_ID(sensor_accel_status), 0},
		{ORB_ID(sensor_accel_status), 1},
		{ORB_ID(sensor_accel_status), 2},
	};

	uORB::Subscription _sensor_gyro_status_sub[3] {
		{ORB_ID(sensor_gyro_status), 0},
		{ORB_ID(sensor_gyro_status), 1},
		{ORB_ID(sensor_gyro_status), 2},
	};

	/* do not allow top copying this class */
	MavlinkStreamVibration(MavlinkStreamVibration &) = delete;
	MavlinkStreamVibration &operator = (const MavlinkStreamVibration &) = delete;

protected:
	explicit MavlinkStreamVibration(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		bool updated = _sensor_selection_sub.updated();

		// check for sensor_accel_status update
		if (!updated) {
			for (int i = 0; i < 3; i++) {
				if (_sensor_accel_status_sub[i].updated() || _sensor_gyro_status_sub[i].updated()) {
					updated = true;
					break;
				}
			}
		}

		if (updated) {

			mavlink_vibration_t msg{};
			msg.time_usec = hrt_absolute_time();

			// VIBRATION usage not to mavlink spec, this is our current usage.
			//  vibration_x : Primary gyro delta angle coning metric = filtered length of (delta_angle x prev_delta_angle)
			//  vibration_y : Primary gyro high frequency vibe = filtered length of (delta_angle - prev_delta_angle)
			//  vibration_z : Primary accel high frequency vibe = filtered length of (delta_velocity - prev_delta_velocity)

			sensor_selection_s sensor_selection{};
			_sensor_selection_sub.copy(&sensor_selection);

			// primary gyro coning and high frequency vibration metrics
			if (sensor_selection.gyro_device_id != 0) {
				for (auto &x : _sensor_gyro_status_sub) {
					sensor_gyro_status_s status;

					if (x.copy(&status)) {
						if (status.device_id == sensor_selection.gyro_device_id) {
							msg.vibration_x = status.coning_vibration;
							msg.vibration_y = status.vibration_metric;
							break;
						}
					}
				}
			}

			// primary accel high frequency vibration metric
			if (sensor_selection.accel_device_id != 0) {
				for (auto &x : _sensor_accel_status_sub) {
					sensor_accel_status_s status;

					if (x.copy(&status)) {
						if (status.device_id == sensor_selection.accel_device_id) {
							msg.vibration_z = status.vibration_metric;
							break;
						}
					}
				}
			}

			// accel 0, 1, 2 cumulative clipping
			for (int i = 0; i < 3; i++) {
				sensor_accel_status_s acc_status;

				if (_sensor_accel_status_sub[i].copy(&acc_status)) {

					const uint32_t clipping = acc_status.clipping[0] + acc_status.clipping[1] + acc_status.clipping[2];

					switch (i) {
					case 0:
						msg.clipping_0 = clipping;
						break;

					case 1:
						msg.clipping_1 = clipping;
						break;

					case 2:
						msg.clipping_2 = clipping;
						break;
					}
				}
			}

			mavlink_msg_vibration_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_VIBRATION_H */
