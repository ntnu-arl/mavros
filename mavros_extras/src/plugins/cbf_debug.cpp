/**
 * @brief CBF debug plugin
 * @file cbf_debug.cpp
 * @author Morten Nissov <morten.c.nissov@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2025 Morten Nissov <morten.c.nissov@gmail.com>.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/CbfDebug.h>
#include <geometry_msgs/TwistStamped.h>

namespace mavros
{
namespace extra_plugins
{
/**
 * @brief CBF Debug plugin
 */
class CbfDebugPlugin : public plugin::PluginBase
{
public:
  CbfDebugPlugin() : PluginBase(), nh("~")
  {}

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

        cbf_debug_pub = nh.advertise<mavros_msgs::CbfDebug>("cbf/debug", 10);
        cbf_inacc_pub = nh.advertise<geometry_msgs::TwistStamped>("cbf/debug/acc_input", 10);
        cbf_outacc_pub = nh.advertise<geometry_msgs::TwistStamped>("cbf/debug/acc_output", 10);
	}

	Subscriptions get_subscriptions() override
	{
		return {
			make_handler(&CbfDebugPlugin::handle_cbf_debug)
		};
	}


private:
	ros::NodeHandle nh;

    ros::Publisher cbf_debug_pub;
    mavros_msgs::CbfDebug _cbf_debug;
    ros::Publisher cbf_inacc_pub;
    geometry_msgs::TwistStamped _cbf_input_acc;
    ros::Publisher cbf_outacc_pub;
    geometry_msgs::TwistStamped _cbf_output_acc;

  void handle_cbf_debug(const mavlink::mavlink_message_t* msg, mavlink::common::msg::CBF_DEBUG& cbf_debug)
  {
    _cbf_debug.header.stamp = m_uas->synchronise_stamp(cbf_debug.time_usec);
    _cbf_debug.cbf_duration = cbf_debug.cbf_duration;
    _cbf_debug.qp_fail = cbf_debug.qp_fail;
    _cbf_debug.h = cbf_debug.h;
    _cbf_debug.h1 = cbf_debug.h1;
    _cbf_debug.h2 = cbf_debug.h2;
    _cbf_debug.input.x = cbf_debug.input[0];
    _cbf_debug.input.y = cbf_debug.input[1];
    _cbf_debug.input.z = cbf_debug.input[2];
    _cbf_debug.output.x = cbf_debug.output[0];
    _cbf_debug.output.y = cbf_debug.output[1];
    _cbf_debug.output.z = cbf_debug.output[2];
    _cbf_debug.slack.x = cbf_debug.slack[0];
    _cbf_debug.slack.y = cbf_debug.slack[1];

    _cbf_input_acc.header.stamp = m_uas->synchronise_stamp(cbf_debug.time_usec);
    _cbf_input_acc.header.frame_id = "base_link";
    _cbf_input_acc.twist.linear.x = cbf_debug.input[0];
    _cbf_input_acc.twist.linear.y = -cbf_debug.input[1];   // from NED
    _cbf_input_acc.twist.linear.z = -cbf_debug.input[2];   // from NED

    _cbf_output_acc.header.stamp = m_uas->synchronise_stamp(cbf_debug.time_usec);
    _cbf_output_acc.header.frame_id = "base_link";
    _cbf_output_acc.twist.linear.x = cbf_debug.output[0];
    _cbf_output_acc.twist.linear.y = -cbf_debug.output[1];   // from NED
    _cbf_output_acc.twist.linear.z = -cbf_debug.output[2];   // from NED

    cbf_debug_pub.publish(_cbf_debug);
    cbf_inacc_pub.publish(_cbf_input_acc);
    cbf_outacc_pub.publish(_cbf_output_acc);
  }
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CbfDebugPlugin, mavros::plugin::PluginBase)
