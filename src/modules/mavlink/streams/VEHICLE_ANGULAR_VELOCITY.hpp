#ifndef VEHICLE_ANGULAR_VELOCITY_HPP
#define VEHICLE_ANGULAR_VELOCITY_HPP

#include <uORB/topics/vehicle_angular_velocity.h>

class MavlinkStreamVehicleAngularVelocity : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamVehicleAngularVelocity(mavlink); }

	static constexpr const char *get_name_static() { return "VEHICLE_ANGULAR_VELOCITY"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_VEHICLE_ANGULAR_VELOCITY; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_vehicle_angular_velocity_sub.advertised()) {
			return MAVLINK_MSG_ID_VEHICLE_ANGULAR_VELOCITY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamVehicleAngularVelocity(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};

	bool send() override
	{
		if (_vehicle_angular_velocity_sub.updated()) {

			vehicle_angular_velocity_s vehicle_angular_velocity{};
			_vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);

			mavlink_vehicle_angular_velocity_t msg{};
			msg.angular_velocity_x = vehicle_angular_velocity.xyz[0];
			msg.angular_velocity_y = vehicle_angular_velocity.xyz[1];
			msg.angular_velocity_z = vehicle_angular_velocity.xyz[2];

			mavlink_msg_vehicle_angular_velocity_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}
		return false;
	}
};

#endif // VEHICLE_ANGULAR_VELOCITY_HPP
