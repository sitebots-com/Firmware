#ifndef MAVLINK_STREAM_ESTIMATOR_STATUS_H
#define MAVLINK_STREAM_ESTIMATOR_STATUS_H

#include "../mavlink_messages.h"

#include <uORB/topics/estimator_status.h>

class MavlinkStreamEstimatorStatus : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamEstimatorStatus::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "ESTIMATOR_STATUS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_ESTIMATOR_STATUS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamEstimatorStatus(mavlink);
	}

	unsigned get_size() override
	{
		return _est_sub.advertised() ? MAVLINK_MSG_ID_ESTIMATOR_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _est_sub{ORB_ID(estimator_status)};

	/* do not allow top copying this class */
	MavlinkStreamEstimatorStatus(MavlinkStreamEstimatorStatus &) = delete;
	MavlinkStreamEstimatorStatus &operator = (const MavlinkStreamEstimatorStatus &) = delete;

protected:
	explicit MavlinkStreamEstimatorStatus(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		estimator_status_s est;

		if (_est_sub.update(&est)) {
			mavlink_estimator_status_t est_msg{};
			est_msg.time_usec = est.timestamp;
			est_msg.vel_ratio = est.vel_test_ratio;
			est_msg.pos_horiz_ratio = est.pos_test_ratio;
			est_msg.pos_vert_ratio = est.hgt_test_ratio;
			est_msg.mag_ratio = est.mag_test_ratio;
			est_msg.hagl_ratio = est.hagl_test_ratio;
			est_msg.tas_ratio = est.tas_test_ratio;
			est_msg.pos_horiz_accuracy = est.pos_horiz_accuracy;
			est_msg.pos_vert_accuracy = est.pos_vert_accuracy;
			est_msg.flags = est.solution_status_flags;
			mavlink_msg_estimator_status_send_struct(_mavlink->get_channel(), &est_msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_ESTIMATOR_STATUS_H */
