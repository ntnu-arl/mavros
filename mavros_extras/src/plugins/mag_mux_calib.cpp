/**
 * @brief Camera IMU synchronisation plugin
 * @file mag_mux_calib.cpp
 * @author Mohammed Kabir < mhkabir98@gmail.com >
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 Mohammed Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/MagMuxCalib.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief MagMuxCalib plugin
 *
 * Puglish 4 magnetometer values for x,y,z using a mux.
 */
class MagMuxCalibPlugin : public plugin::PluginBase {
public:
	MagMuxCalibPlugin() : PluginBase(),
		mag_mux_calib_nh("~mag_mux_calib")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		mag_mux_calib_pub = mag_mux_calib_nh.advertise<mavros_msgs::MagMuxCalib>("data", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&MagMuxCalibPlugin::handle_mag_mux_calib)
		};
	}

private:
	ros::NodeHandle mag_mux_calib_nh;
	ros::Publisher mag_mux_calib_pub;

	void handle_mag_mux_calib(const mavlink::mavlink_message_t *msg, mavlink::common::msg::mag_mux_calib &MagMuxCalib)
	{
		auto mag_mux_calib_msg = boost::make_shared<mavros_msgs::MagMuxCalib>();

		mag_mux_calib_msg->header.stamp = m_uas->synchronise_stamp(MagMuxCalib.time_usec);
		// std::cout << MagMuxCalib.mags[0] << std::endl;
		for (int i = 0; i < 4; i++)
		{
			mag_mux_calib_msg->xyz[i].x = MagMuxCalib.xyz[i*3];
			mag_mux_calib_msg->xyz[i].y = MagMuxCalib.xyz[i*3 + 1];
			mag_mux_calib_msg->xyz[i].z = MagMuxCalib.xyz[i*3 + 2];
			mag_mux_calib_msg->center[i].x = MagMuxCalib.center[i*3];
			mag_mux_calib_msg->center[i].y = MagMuxCalib.center[i*3 + 1];
			mag_mux_calib_msg->center[i].z = MagMuxCalib.center[i*3 + 2];
			mag_mux_calib_msg->max[i].x = MagMuxCalib.max[i*3];
			mag_mux_calib_msg->max[i].y = MagMuxCalib.max[i*3 + 1];
			mag_mux_calib_msg->max[i].z = MagMuxCalib.max[i*3 + 2];
		}

		// mag_mux_calib_msg->mag_mux_calib[0].y = MagMuxCalib.mags[1];
		// mag_mux_calib_msg->mag_mux_calib[0].z = MagMuxCalib.mags[2];
		// mag_mux_calib_msg->mag_mux_calib[1].x = MagMuxCalib.mags[3];
		// mag_mux_calib_msg->mag_mux_calib[1].y = MagMuxCalib.mags[4];
		// mag_mux_calib_msg->mag_mux_calib[1].z = MagMuxCalib.mags[5];
		// mag_mux_calib_msg->mag_mux_calib[2].x = MagMuxCalib.mags[6];
		// mag_mux_calib_msg->mag_mux_calib[2].y = MagMuxCalib.mags[7];
		// mag_mux_calib_msg->mag_mux_calib[2].z = MagMuxCalib.mags[8];
		// mag_mux_calib_msg->mag_mux_calib[3].x = MagMuxCalib.mags[9];
		// mag_mux_calib_msg->mag_mux_calib[3].y = MagMuxCalib.mags[10];
		// mag_mux_calib_msg->mag_mux_calib[4].z = MagMuxCalib.mags[11];
		// 	int i=0, j=0;
		// for (i = 0; i < 5; i++) {
		// 	mag_mux_calib_msg->mag_mux_calib[i].x = MagMuxCalib.mags[j++];
		// 	mag_mux_calib_msg->mag_mux_calib[i].y = MagMuxCalib.mags[j++];
		// 	mag_mux_calib_msg->mag_mux_calib[i].z = MagMuxCalib.mags[j++];
		// }
		mag_mux_calib_pub.publish(mag_mux_calib_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MagMuxCalibPlugin, mavros::plugin::PluginBase)
