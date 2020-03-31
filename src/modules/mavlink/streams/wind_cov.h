#ifndef MAVLINK_STREAM_WIND_COV_H
#define MAVLINK_STREAM_WIND_COV_H

#include "../mavlink_messages.h"

#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wind_estimate.h>

class MavlinkStreamWind : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamWind::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "WIND_COV";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_WIND_COV;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamWind(mavlink);
	}

	unsigned get_size() override
	{
		return _wind_estimate_sub.advertised() ? MAVLINK_MSG_ID_WIND_COV_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _wind_estimate_sub{ORB_ID(wind_estimate)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};

	/* do not allow top copying this class */
	MavlinkStreamWind(MavlinkStreamWind &) = delete;
	MavlinkStreamWind &operator = (const MavlinkStreamWind &) = delete;

protected:
	explicit MavlinkStreamWind(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		wind_estimate_s wind_estimate;

		if (_wind_estimate_sub.update(&wind_estimate)) {
			mavlink_wind_cov_t msg{};

			msg.time_usec = wind_estimate.timestamp;

			msg.wind_x = wind_estimate.windspeed_north;
			msg.wind_y = wind_estimate.windspeed_east;
			msg.wind_z = 0.0f;

			msg.var_horiz = wind_estimate.variance_north + wind_estimate.variance_east;
			msg.var_vert = 0.0f;

			vehicle_local_position_s lpos{};
			_local_pos_sub.copy(&lpos);
			msg.wind_alt = (lpos.z_valid && lpos.z_global) ? (-lpos.z + lpos.ref_alt) : NAN;

			msg.horiz_accuracy = 0.0f;
			msg.vert_accuracy = 0.0f;

			mavlink_msg_wind_cov_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_WIND_COV_H */
