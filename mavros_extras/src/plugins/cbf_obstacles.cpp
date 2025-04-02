#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <iostream>

namespace mavros {
namespace extra_plugins{

class CbfObstaclesPlugin : public plugin::PluginBase {
public:
    CbfObstaclesPlugin()
    : PluginBase(),
      _nh("~mavros_obstacles")
    { };

    void initialize(UAS &uas_)
    {
        PluginBase::initialize(uas_);
        _obstacles_sub = _nh.subscribe("/mavros/obstacles", 1, &CbfObstaclesPlugin::tofObstaclesCallback, this);
    };

    Subscriptions get_subscriptions()
    {
        return {/* RX disabled */ };
    }

private:
    ros::NodeHandle _nh;
    ros::Subscriber _obstacles_sub;
    const size_t POINTS_PER_CHUNK = sizeof(mavlink::common::msg::OBSTACLES_CHUNK::points_x)/sizeof(float);

    void tofObstaclesCallback(const sensor_msgs::PointCloud2::ConstPtr &req)
    {

        size_t num_points = req->height * req->width;
        if (num_points == 0) return;

        // ROS_INFO_STREAM("Got " << req->waypoints.size() << " obstacles");

        std::vector<mavlink::common::msg::OBSTACLES_CHUNK> chunks;

        // Iterators for pointcloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_msg, "z");

        // Fill full chunks
        for (size_t chunk_id = 0; chunk_id < num_points/POINTS_PER_CHUNK; chunk_id++) {
            chunks.emplace_back();
            auto& chunk = chunks.back();
            chunk.chunk_id = (uint8_t)chunk_id;
            chunk.num_points_chunk = (uint8_t)POINTS_PER_CHUNK;
            for (size_t i = 0; i < POINTS_PER_CHUNK; i++, ++iter_x, ++iter_y, ++iter_z) {
                chunk.points_x[i] = *iter_x;
                chunk.points_y[i] = *iter_y;
                chunk.points_z[i] = *iter_z;
                // const auto& waypoint = req->waypoints[chunk.chunk_id * POINTS_PER_CHUNK + i];
                // chunk.points_x[i] = (float)waypoint.x_lat;
                // chunk.points_y[i] = (float)waypoint.y_long;
                // chunk.points_z[i] = (float)waypoint.z_alt;
            }
        }

        // Fill last incomplete chunk, if any
        if (num_points % POINTS_PER_CHUNK != 0) {
            chunks.emplace_back();
            auto& extra_chunk = chunks.back();
            extra_chunk.chunk_id = (uint8_t)(num_points/POINTS_PER_CHUNK);
            extra_chunk.num_points_chunk = (uint8_t)(num_points % POINTS_PER_CHUNK);
            for (size_t i = 0; i < POINTS_PER_CHUNK; i++, ++iter_x, ++iter_y, ++iter_z) {
                chunk.points_x[i] = *iter_x;
                chunk.points_y[i] = *iter_y;
                chunk.points_z[i] = *iter_z;
                // const auto& waypoint = req->waypoints[extra_chunk.chunk_id * POINTS_PER_CHUNK + i];
                // extra_chunk.points_x[i] = (float)waypoint.x_lat;
                // extra_chunk.points_y[i] = (float)waypoint.y_long;
                // extra_chunk.points_z[i] = (float)waypoint.z_alt;
            }
        }

        // Send each chunk individually
        for (auto& chunk : chunks) {
            chunk.num_chunks = (uint8_t)chunks.size();
            chunk.num_points_total = (uint32_t)num_points;

            try {
                UAS_FCU(m_uas)->send_message(chunk);
                // ROS_INFO_STREAM("Sent chunk " << (int)chunk.chunk_id + 1 << "/" << (int)chunk.num_chunks
                //                 << " with " << (int)chunk.num_points_chunk << " points"
                //                 << " (total " << (int)chunk.num_points_total << ")");
                // ros::Duration(0.01).sleep();
            } catch (std::length_error e) {
                ROS_ERROR("%s", e.what());
            }
        }
    }
};
}   // namespace extra_plugins
}   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::CbfObstaclesPlugin, mavros::plugin::PluginBase)
