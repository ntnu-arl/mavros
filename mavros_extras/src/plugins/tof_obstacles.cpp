#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_msgs/WaypointList.h>
#include <mavros_msgs/Waypoint.h>

#include <iostream>

namespace mavros {
namespace extra_plugins{

class TimeOfFlightObstaclesPlugin : public plugin::PluginBase {
public:
    TimeOfFlightObstaclesPlugin()
    : PluginBase(),
      nh("~tof_obstacles")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        tof_obstacles_sub = nh.subscribe("/mavros/tof_obstacles", 1, &TimeOfFlightObstaclesPlugin::tofObstaclesCallback, this);
    };

    Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber tof_obstacles_sub;

    void tofObstaclesCallback(const mavros_msgs::WaypointList::ConstPtr &req)
    {
        // ROS_INFO_STREAM("Got " << req->waypoints.size() << " obstacles");

        if (req->waypoints.size() == 0) return;

        std::vector<mavlink::common::msg::TOF_OBSTACLES_CHUNK> chunks;

        const size_t POINTS_PER_CHUNK = sizeof(mavlink::common::msg::TOF_OBSTACLES_CHUNK::points_x)/sizeof(float);
        size_t num_points = req->waypoints.size();

        for (size_t chunk_id = 0; chunk_id < num_points/POINTS_PER_CHUNK; chunk_id++) {
            chunks.emplace_back();
            auto& chunk = chunks.back();
            chunk.chunk_id = (uint8_t)chunk_id;
            chunk.num_points_chunk = (uint8_t)POINTS_PER_CHUNK;
            for (size_t i = 0; i < POINTS_PER_CHUNK; i++) {
                const auto& waypoint = req->waypoints[chunk.chunk_id * POINTS_PER_CHUNK + i];
                chunk.points_x[i] = (float)waypoint.x_lat;
                chunk.points_y[i] = (float)waypoint.y_long;
                chunk.points_z[i] = (float)waypoint.z_alt;
            }
        }
        if (num_points % POINTS_PER_CHUNK != 0) {
            chunks.emplace_back();
            auto& extra_chunk = chunks.back();
            extra_chunk.chunk_id = (uint8_t)(num_points/POINTS_PER_CHUNK);
            extra_chunk.num_points_chunk = (uint8_t)(num_points % POINTS_PER_CHUNK);
            for (size_t i = 0; i < POINTS_PER_CHUNK; i++) {
                const auto& waypoint = req->waypoints[extra_chunk.chunk_id * POINTS_PER_CHUNK + i];
                extra_chunk.points_x[i] = (float)waypoint.x_lat;
                extra_chunk.points_y[i] = (float)waypoint.y_long;
                extra_chunk.points_z[i] = (float)waypoint.z_alt;
            }
        }
        for (auto& chunk : chunks) {
            chunk.num_chunks = (uint8_t)chunks.size();
            chunk.num_points_total = (uint32_t)num_points;
        }
        
        // Send each chunk individually
        for (const auto& chunk : chunks) {
            try {
                UAS_FCU(m_uas)->send_message(chunk);
                // ROS_INFO_STREAM("Sent chunk " << (int)chunk.chunk_id + 1 << "/" << (int)chunk.num_chunks
                //                 << " with " << (int)chunk.num_points_chunk << " points"
                //                 << " (total " << (int)chunk.num_points_total << ")");
                ros::Duration(0.01).sleep();
            } catch (std::length_error e) {
                ROS_ERROR(e.what());
            }
        }
    }
};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TimeOfFlightObstaclesPlugin, mavros::plugin::PluginBase)