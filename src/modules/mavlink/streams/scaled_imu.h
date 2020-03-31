#ifndef MAVLINK_STREAM_SCALED_IMU_H
#define MAVLINK_STREAM_SCALED_IMU_H

#include "../mavlink_messages.h"

#include <lib/ecl/geo/geo.h>
#include <lib/matrix/matrix/math.hpp>

#include <uORB/topics/sensor_accel_integrated.h>
#include <uORB/topics/sensor_gyro_integrated.h>
#include <uORB/topics/sensor_mag.h>

using matrix::Vector3f;

const char *MSG_NAMES_SCALED_IMU[] = {
	"SCALED_IMU",
	"SCALED_IMU2",
	"SCALED_IMU3"
};

class MavlinkStreamScaledIMUBase : public MavlinkStream
{
protected:
	explicit MavlinkStreamScaledIMUBase(Mavlink *mavlink, int N) : MavlinkStream(mavlink),
		_raw_accel_sub(ORB_ID(sensor_accel_integrated), N),
		_raw_gyro_sub(ORB_ID(sensor_gyro_integrated), N),
		_raw_mag_sub(ORB_ID(sensor_mag), N)
	{}

	bool send(const hrt_abstime t) override
	{
		if (_raw_accel_sub.updated() || _raw_gyro_sub.updated() || _raw_mag_sub.updated()) {

			sensor_accel_integrated_s sensor_accel{};
			_raw_accel_sub.copy(&sensor_accel);

			sensor_gyro_integrated_s sensor_gyro{};
			_raw_gyro_sub.copy(&sensor_gyro);

			sensor_mag_s sensor_mag{};
			_raw_mag_sub.copy(&sensor_mag);

			// Accelerometer in mG
			const float accel_dt_inv = 1.e6f / (float)sensor_accel.dt;
			const Vector3f accel = Vector3f{sensor_accel.delta_velocity} * accel_dt_inv * 1000.0f / CONSTANTS_ONE_G;

			const float gyro_dt_inv = 1.e6f / (float)sensor_gyro.dt;
			const Vector3f gyro = Vector3f{sensor_gyro.delta_angle} * gyro_dt_inv * 1000.0f;

			send_mavlink_message(sensor_accel.timestamp / 1000, accel, gyro, sensor_mag);
			return true;
		}

		return false;
	}

	virtual void send_mavlink_message(uint32_t time_boot_ms, const Vector3f &sensor_accel, const Vector3f &gyro,
					  sensor_mag_s &sensor_mag) = 0;

private:
	uORB::Subscription _raw_accel_sub;
	uORB::Subscription _raw_gyro_sub;
	uORB::Subscription _raw_mag_sub;

	// do not allow to copy this class
	MavlinkStreamScaledIMUBase(MavlinkStreamScaledIMUBase &) = delete;
	MavlinkStreamScaledIMUBase &operator = (const MavlinkStreamScaledIMUBase &) = delete;
};

template <int N, typename Derived, uint16_t MSG_ID_SCALED_IMU, unsigned MSG_ID_SCALED_IMU_LEN, typename MavlinkMsg>
class MavlinkStreamScaledIMUTemplate : public MavlinkStreamScaledIMUBase
{
public:
	const char *get_name() const override
	{
		return get_name_static();
	}

	static const char *get_name_static()
	{
		return MSG_NAMES_SCALED_IMU[N];
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static constexpr uint16_t get_id_static()
	{
		return MSG_ID_SCALED_IMU;
	}

	unsigned get_size() override
	{
		return MSG_ID_SCALED_IMU_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new Derived(mavlink);
	}

private:
	typedef void (*SendMavlinkHandler)(mavlink_channel_t,  const MavlinkMsg *);
	SendMavlinkHandler _send_mavlink;

protected:
	explicit MavlinkStreamScaledIMUTemplate(Mavlink *mavlink,
						SendMavlinkHandler handler) : MavlinkStreamScaledIMUBase(mavlink, N),
		_send_mavlink(handler)
	{}

	void send_mavlink_message(uint32_t time_boot_ms, const Vector3f &accel, const Vector3f &gyro,
				  sensor_mag_s &sensor_mag) override
	{
		MavlinkMsg msg{};
		msg.time_boot_ms = time_boot_ms;
		msg.xacc = (int16_t)accel(0);
		msg.yacc = (int16_t)accel(1);
		msg.zacc = (int16_t)accel(2);
		msg.xgyro = gyro(0);
		msg.ygyro = gyro(1);
		msg.zgyro = gyro(2);
		msg.xmag = sensor_mag.x * 1000.0f; // Gauss -> MilliGauss
		msg.ymag = sensor_mag.y * 1000.0f; // Gauss -> MilliGauss
		msg.zmag = sensor_mag.z * 1000.0f; // Gauss -> MilliGauss
		msg.temperature = sensor_mag.temperature;

		_send_mavlink(_mavlink->get_channel(), &msg);
	}
};

template <int N> struct MavlinkStreamScaledIMU {};

template <>
class MavlinkStreamScaledIMU<0> : public
	MavlinkStreamScaledIMUTemplate<0, MavlinkStreamScaledIMU<0>, MAVLINK_MSG_ID_SCALED_IMU, MAVLINK_MSG_ID_SCALED_IMU_LEN, mavlink_scaled_imu_t>
{
public:
	typedef MavlinkStreamScaledIMUTemplate<0, MavlinkStreamScaledIMU<0>, MAVLINK_MSG_ID_SCALED_IMU, MAVLINK_MSG_ID_SCALED_IMU_LEN, mavlink_scaled_imu_t>
	Base;

	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : Base(mavlink, mavlink_msg_scaled_imu_send_struct) {}
};

template <>
class MavlinkStreamScaledIMU<1> : public
	MavlinkStreamScaledIMUTemplate<1, MavlinkStreamScaledIMU<1>, MAVLINK_MSG_ID_SCALED_IMU2, MAVLINK_MSG_ID_SCALED_IMU2_LEN, mavlink_scaled_imu2_t>
{
public:
	typedef MavlinkStreamScaledIMUTemplate<1, MavlinkStreamScaledIMU<1>, MAVLINK_MSG_ID_SCALED_IMU2, MAVLINK_MSG_ID_SCALED_IMU2_LEN, mavlink_scaled_imu2_t>
	Base;

	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : Base(mavlink, mavlink_msg_scaled_imu2_send_struct) {}
};

template <>
class MavlinkStreamScaledIMU<2> : public
	MavlinkStreamScaledIMUTemplate<2, MavlinkStreamScaledIMU<2>, MAVLINK_MSG_ID_SCALED_IMU3, MAVLINK_MSG_ID_SCALED_IMU3_LEN, mavlink_scaled_imu3_t>
{
public:
	typedef MavlinkStreamScaledIMUTemplate<2, MavlinkStreamScaledIMU<2>, MAVLINK_MSG_ID_SCALED_IMU3, MAVLINK_MSG_ID_SCALED_IMU3_LEN, mavlink_scaled_imu3_t>
	Base;

	explicit MavlinkStreamScaledIMU(Mavlink *mavlink) : Base(mavlink, mavlink_msg_scaled_imu3_send_struct) {}
};


#endif /* MAVLINK_STREAM_SCALED_IMU_H */
