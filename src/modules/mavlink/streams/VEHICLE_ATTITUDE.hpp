#ifndef VEHICLE_ATTITUDE_HPP
#define VEHICLE_ATTITUDE_HPP

#include <uORB/topics/vehicle_attitude.h>

class MavlinkStreamVehicleAttitude : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleAttitude(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_ATTITUDE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_ATTITUDE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_attitude_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_ATTITUDE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleAttitude(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	bool send() override
	{
		if (_vehicle_attitude_sub.updated()) {

			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);

			mavlink_vehicle_attitude_t msg{};
			msg.q_w = vehicle_attitude.q[0];
			msg.q_x = vehicle_attitude.q[1];
			msg.q_y = vehicle_attitude.q[2];
			msg.q_z = vehicle_attitude.q[3];

			msg.delta_q_w = vehicle_attitude.delta_q_reset[0];
			msg.delta_q_x = vehicle_attitude.delta_q_reset[1];
			msg.delta_q_y = vehicle_attitude.delta_q_reset[2];
			msg.delta_q_z = vehicle_attitude.delta_q_reset[3];

			mavlink_msg_vehicle_attitude_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_ATTITUDE_HPP
