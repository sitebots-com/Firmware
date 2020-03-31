#ifndef MAVLINK_STREAM_CAMERA_IMAGE_CAPTURED_H
#define MAVLINK_STREAM_CAMERA_IMAGE_CAPTURED_H

#include "../mavlink_messages.h"

#include <uORB/topics/camera_capture.h>

class MavlinkStreamCameraImageCaptured : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamCameraImageCaptured::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "CAMERA_IMAGE_CAPTURED";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	bool const_rate() override
	{
		return true;
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamCameraImageCaptured(mavlink);
	}

	unsigned get_size() override
	{
		return _capture_sub.advertised() ? MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	uORB::Subscription _capture_sub{ORB_ID(camera_capture)};

	/* do not allow top copying this class */
	MavlinkStreamCameraImageCaptured(MavlinkStreamCameraImageCaptured &) = delete;
	MavlinkStreamCameraImageCaptured &operator = (const MavlinkStreamCameraImageCaptured &) = delete;

protected:
	explicit MavlinkStreamCameraImageCaptured(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		camera_capture_s capture;

		if (_capture_sub.update(&capture)) {

			mavlink_camera_image_captured_t msg{};

			msg.time_boot_ms = capture.timestamp / 1000;
			msg.time_utc = capture.timestamp_utc;
			msg.camera_id = 1;	// FIXME : get this from uORB
			msg.lat = capture.lat * 1e7;
			msg.lon = capture.lon * 1e7;
			msg.alt = capture.alt * 1e3f;
			msg.relative_alt = capture.ground_distance * 1e3f;
			msg.q[0] = capture.q[0];
			msg.q[1] = capture.q[1];
			msg.q[2] = capture.q[2];
			msg.q[3] = capture.q[3];
			msg.image_index = capture.seq;
			msg.capture_result = capture.result;
			msg.file_url[0] = '\0';

			mavlink_msg_camera_image_captured_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_CAMERA_IMAGE_CAPTURED_H */
