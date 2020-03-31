#ifndef MAVLINK_STREAM_HIGHRES_IMU_H
#define MAVLINK_STREAM_HIGHRES_IMU_H

#include "../mavlink_messages.h"

#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>

class MavlinkStreamHighresIMU : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamHighresIMU::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "HIGHRES_IMU";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamHighresIMU(mavlink);
	}

	unsigned get_size() override
	{
		return MAVLINK_MSG_ID_HIGHRES_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

private:
	uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};
	uORB::Subscription _magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _air_data_sub{ORB_ID(vehicle_air_data)};

	/* do not allow top copying this class */
	MavlinkStreamHighresIMU(MavlinkStreamHighresIMU &) = delete;
	MavlinkStreamHighresIMU &operator = (const MavlinkStreamHighresIMU &) = delete;

protected:
	explicit MavlinkStreamHighresIMU(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		sensor_combined_s sensor;

		if (_sensor_sub.update(&sensor)) {
			uint16_t fields_updated = 0;

			fields_updated |= (1 << 0) | (1 << 1) | (1 << 2); // accel
			fields_updated |= (1 << 3) | (1 << 4) | (1 << 5); // gyro

			vehicle_magnetometer_s magnetometer{};

			if (_magnetometer_sub.update(&magnetometer)) {
				/* mark third group dimensions as changed */
				fields_updated |= (1 << 6) | (1 << 7) | (1 << 8);

			} else {
				_magnetometer_sub.copy(&magnetometer);
			}

			vehicle_air_data_s air_data{};

			if (_air_data_sub.update(&air_data)) {
				/* mark fourth group (baro fields) dimensions as changed */
				fields_updated |= (1 << 9) | (1 << 11) | (1 << 12);

			} else {
				_air_data_sub.copy(&air_data);
			}

			differential_pressure_s differential_pressure{};

			if (_differential_pressure_sub.update(&differential_pressure)) {
				/* mark fourth group (dpres field) dimensions as changed */
				fields_updated |= (1 << 10);

			} else {
				_differential_pressure_sub.copy(&differential_pressure);
			}

			estimator_sensor_bias_s bias{};
			_bias_sub.copy(&bias);

			mavlink_highres_imu_t msg{};

			msg.time_usec = sensor.timestamp;
			msg.xacc = sensor.accelerometer_m_s2[0] - bias.accel_bias[0];
			msg.yacc = sensor.accelerometer_m_s2[1] - bias.accel_bias[1];
			msg.zacc = sensor.accelerometer_m_s2[2] - bias.accel_bias[2];
			msg.xgyro = sensor.gyro_rad[0] - bias.gyro_bias[0];
			msg.ygyro = sensor.gyro_rad[1] - bias.gyro_bias[1];
			msg.zgyro = sensor.gyro_rad[2] - bias.gyro_bias[2];
			msg.xmag = magnetometer.magnetometer_ga[0] - bias.mag_bias[0];
			msg.ymag = magnetometer.magnetometer_ga[1] - bias.mag_bias[1];
			msg.zmag = magnetometer.magnetometer_ga[2] - bias.mag_bias[2];
			msg.abs_pressure = air_data.baro_pressure_pa;
			msg.diff_pressure = differential_pressure.differential_pressure_raw_pa;
			msg.pressure_alt = air_data.baro_alt_meter;
			msg.temperature = air_data.baro_temp_celcius;
			msg.fields_updated = fields_updated;

			mavlink_msg_highres_imu_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_HIGHRES_IMU_H */
