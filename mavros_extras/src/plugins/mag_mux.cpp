/**
 * @brief Camera IMU synchronisation plugin
 * @file mag_mux.cpp
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

#include <mavros_msgs/MagMux.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief MagMux plugin
 *
 * Puglish 4 magnetometer values for x,y,z using a mux.
 */
class MagMuxPlugin : public plugin::PluginBase {
public:
	MagMuxPlugin() : PluginBase(),
		mag_mux_nh("~mag_mux")
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		mag_mux_pub = mag_mux_nh.advertise<mavros_msgs::MagMux>("raw", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&MagMuxPlugin::handle_mag_mux)
		};
	}

private:
	ros::NodeHandle mag_mux_nh;
	ros::Publisher mag_mux_pub;

	void handle_mag_mux(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MAG_MUX &magmux)
	{
		auto mag_mux_msg = boost::make_shared<mavros_msgs::MagMux>();

		mag_mux_msg->header.stamp = m_uas->synchronise_stamp(magmux.time_usec);
		// std::cout << magmux.mags[0] << std::endl;
		for (int i = 0; i < 4; i++)
		{
			mag_mux_msg->mags[i].x = magmux.mags[i*3];
			mag_mux_msg->mags[i].y = magmux.mags[i*3 + 1];
			mag_mux_msg->mags[i].z = magmux.mags[i*3 + 2];
		}

		// mag_mux_msg->mag_mux[0].y = magmux.mags[1];
		// mag_mux_msg->mag_mux[0].z = magmux.mags[2];
		// mag_mux_msg->mag_mux[1].x = magmux.mags[3];
		// mag_mux_msg->mag_mux[1].y = magmux.mags[4];
		// mag_mux_msg->mag_mux[1].z = magmux.mags[5];
		// mag_mux_msg->mag_mux[2].x = magmux.mags[6];
		// mag_mux_msg->mag_mux[2].y = magmux.mags[7];
		// mag_mux_msg->mag_mux[2].z = magmux.mags[8];
		// mag_mux_msg->mag_mux[3].x = magmux.mags[9];
		// mag_mux_msg->mag_mux[3].y = magmux.mags[10];
		// mag_mux_msg->mag_mux[4].z = magmux.mags[11];
		// 	int i=0, j=0;
		// for (i = 0; i < 5; i++) {
		// 	mag_mux_msg->mag_mux[i].x = magmux.mags[j++];
		// 	mag_mux_msg->mag_mux[i].y = magmux.mags[j++];
		// 	mag_mux_msg->mag_mux[i].z = magmux.mags[j++];
		// }
		mag_mux_pub.publish(mag_mux_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MagMuxPlugin, mavros::plugin::PluginBase)
