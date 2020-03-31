#ifndef MAVLINK_STREAM_RC_CHANNELS_H
#define MAVLINK_STREAM_RC_CHANNELS_H

#include "../mavlink_messages.h"

#include <uORB/topics/input_rc.h>

class MavlinkStreamRCChannels : public MavlinkStream
{
public:
	const char *get_name() const override
	{
		return MavlinkStreamRCChannels::get_name_static();
	}

	static constexpr const char *get_name_static()
	{
		return "RC_CHANNELS";
	}

	static constexpr uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_RC_CHANNELS;
	}

	uint16_t get_id() override
	{
		return get_id_static();
	}

	static MavlinkStream *new_instance(Mavlink *mavlink)
	{
		return new MavlinkStreamRCChannels(mavlink);
	}

	unsigned get_size() override
	{
		return _rc_sub.advertised() ? (MAVLINK_MSG_ID_RC_CHANNELS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) : 0;
	}

private:
	uORB::Subscription _rc_sub{ORB_ID(input_rc)};

	/* do not allow top copying this class */
	MavlinkStreamRCChannels(MavlinkStreamRCChannels &) = delete;
	MavlinkStreamRCChannels &operator = (const MavlinkStreamRCChannels &) = delete;

protected:
	explicit MavlinkStreamRCChannels(Mavlink *mavlink) : MavlinkStream(mavlink)
	{}

	bool send(const hrt_abstime t) override
	{
		input_rc_s rc;

		if (_rc_sub.update(&rc)) {

			/* send RC channel data and RSSI */
			mavlink_rc_channels_t msg{};

			msg.time_boot_ms = rc.timestamp / 1000;
			msg.chancount = rc.channel_count;
			msg.chan1_raw = (rc.channel_count > 0) ? rc.values[0] : UINT16_MAX;
			msg.chan2_raw = (rc.channel_count > 1) ? rc.values[1] : UINT16_MAX;
			msg.chan3_raw = (rc.channel_count > 2) ? rc.values[2] : UINT16_MAX;
			msg.chan4_raw = (rc.channel_count > 3) ? rc.values[3] : UINT16_MAX;
			msg.chan5_raw = (rc.channel_count > 4) ? rc.values[4] : UINT16_MAX;
			msg.chan6_raw = (rc.channel_count > 5) ? rc.values[5] : UINT16_MAX;
			msg.chan7_raw = (rc.channel_count > 6) ? rc.values[6] : UINT16_MAX;
			msg.chan8_raw = (rc.channel_count > 7) ? rc.values[7] : UINT16_MAX;
			msg.chan9_raw = (rc.channel_count > 8) ? rc.values[8] : UINT16_MAX;
			msg.chan10_raw = (rc.channel_count > 9) ? rc.values[9] : UINT16_MAX;
			msg.chan11_raw = (rc.channel_count > 10) ? rc.values[10] : UINT16_MAX;
			msg.chan12_raw = (rc.channel_count > 11) ? rc.values[11] : UINT16_MAX;
			msg.chan13_raw = (rc.channel_count > 12) ? rc.values[12] : UINT16_MAX;
			msg.chan14_raw = (rc.channel_count > 13) ? rc.values[13] : UINT16_MAX;
			msg.chan15_raw = (rc.channel_count > 14) ? rc.values[14] : UINT16_MAX;
			msg.chan16_raw = (rc.channel_count > 15) ? rc.values[15] : UINT16_MAX;
			msg.chan17_raw = (rc.channel_count > 16) ? rc.values[16] : UINT16_MAX;
			msg.chan18_raw = (rc.channel_count > 17) ? rc.values[17] : UINT16_MAX;

			msg.rssi = (rc.channel_count > 0) ? rc.rssi : 0;

			mavlink_msg_rc_channels_send_struct(_mavlink->get_channel(), &msg);
			return true;
		}

		return false;
	}
};

#endif /* MAVLINK_STREAM_RC_CHANNELS_H */
