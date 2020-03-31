#ifndef MAVLINK_STREAM_STATUSTEXT_H
#define MAVLINK_STREAM_STATUSTEXT_H

#include "../mavlink_messages.h"

#include <uORB/topics/mavlink_log.h>

class MavlinkStreamStatustext : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamStatustext::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "STATUSTEXT";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_STATUSTEXT;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamStatustext(mavlink);
	}

	unsigned get_size() override
	{
		return _mavlink->get_logbuffer()->empty() ? 0 : (MAVLINK_MSG_ID_STATUSTEXT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES);
	}

private:
	/* do not allow top copying this class */
	MavlinkStreamStatustext(MavlinkStreamStatustext &) = delete;
	MavlinkStreamStatustext &operator = (const MavlinkStreamStatustext &) = delete;

protected:
	int _id{0};

	explicit MavlinkStreamStatustext(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		if (!_mavlink->get_logbuffer()->empty() && _mavlink->is_connected()) {

			mavlink_log_s mavlink_log{};

			if (_mavlink->get_logbuffer()->get(&mavlink_log)) {

				mavlink_statustext_t msg{};
				const char *text = mavlink_log.text;
				constexpr unsigned max_chunk_size = sizeof(msg.text);
				msg.severity = mavlink_log.severity;
				msg.chunk_seq = 0;
				msg.id = _id++;
				unsigned text_size;

				while ((text_size = strlen(text)) > 0) {
					unsigned chunk_size = math::min(text_size, max_chunk_size);

					if (chunk_size < max_chunk_size) {
						memcpy(&msg.text[0], &text[0], chunk_size);
						// pad with zeros
						memset(&msg.text[0] + chunk_size, 0, max_chunk_size - chunk_size);

					} else {
						memcpy(&msg.text[0], &text[0], chunk_size);
					}

					mavlink_msg_statustext_send_struct(_mavlink->get_channel(), &msg);

					if (text_size <= max_chunk_size) {
						break;

					} else {
						text += max_chunk_size;
					}

					msg.chunk_seq += 1;
				}

				return true;
			}
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_STATUSTEXT_H */
