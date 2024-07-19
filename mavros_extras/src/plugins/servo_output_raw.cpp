/**
 * @brief servo_output_raw plugin
 * @file servo_output_raw.cpp
 * @author meng chaoheng
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2024 meng chaoheng.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/ServoOutputRaw.h>
#include <mavros_msgs/MotorRPM.h>
namespace mavros {
namespace extra_plugins {
/**
 * @brief servo_output_raw plugin.
 */
class ServoOutputRawPlugin : public plugin::PluginBase {
public:
	ServoOutputRawPlugin() : PluginBase(),
		nh("~")
	{ }

	/**
	 * Plugin initializer. Constructor should not do this.
	 */
	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		nh.param<std::string>("frame_id", frame_id, "map");
		servo_output_raw_pub = nh.advertise<mavros_msgs::ServoOutputRaw>("servo_output_raw", 10);
        motorpwm_pub = nh.advertise<mavros_msgs::MotorRPM>("motorpwm", 10);

	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&ServoOutputRawPlugin::handle_servo_output_raw),

		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;

	ros::Publisher servo_output_raw_pub;
    ros::Publisher motorpwm_pub;

	void handle_servo_output_raw(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERVO_OUTPUT_RAW &servo_output_raw)
	{
        // std::cout << "handle_servo_output_raw" <<  std::endl;

		auto ros_msg = boost::make_shared<mavros_msgs::ServoOutputRaw>();
		ros_msg->header = m_uas->synchronized_header(frame_id, servo_output_raw.time_usec);
        ros_msg->port = servo_output_raw.port;
		ros_msg->servo1_raw = servo_output_raw.servo1_raw;
        ros_msg->servo2_raw = servo_output_raw.servo2_raw;
        ros_msg->servo3_raw = servo_output_raw.servo3_raw;
        ros_msg->servo4_raw = servo_output_raw.servo4_raw;
        ros_msg->servo5_raw = servo_output_raw.servo5_raw;
        ros_msg->servo6_raw = servo_output_raw.servo6_raw;
        ros_msg->servo7_raw = servo_output_raw.servo7_raw;
        ros_msg->servo8_raw = servo_output_raw.servo8_raw;
        ros_msg->servo9_raw = servo_output_raw.servo9_raw;
        ros_msg->servo10_raw = servo_output_raw.servo10_raw;
        ros_msg->servo11_raw = servo_output_raw.servo11_raw;
        ros_msg->servo12_raw = servo_output_raw.servo12_raw;
        ros_msg->servo13_raw = servo_output_raw.servo13_raw;
        ros_msg->servo14_raw = servo_output_raw.servo14_raw;
        ros_msg->servo15_raw = servo_output_raw.servo15_raw;
        ros_msg->servo16_raw = servo_output_raw.servo16_raw;

		servo_output_raw_pub.publish(ros_msg);

        auto pwm = boost::make_shared<mavros_msgs::MotorRPM>();
        pwm->header = m_uas->synchronized_header(frame_id, servo_output_raw.time_usec);
        pwm->rpm[0]=((servo_output_raw.servo1_raw-1000)/1000)*1000 * 60/M_PI;
        pwm->rpm[1]=((servo_output_raw.servo2_raw-1000)/1000)*1000 * 60/M_PI;
        pwm->rpm[2]=((servo_output_raw.servo3_raw-1000)/1000)*1000 * 60/M_PI;
        pwm->rpm[3]=((servo_output_raw.servo4_raw-1000)/1000)*1000 * 60/M_PI;

        motorpwm_pub.publish(pwm);

	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ServoOutputRawPlugin, mavros::plugin::PluginBase)
