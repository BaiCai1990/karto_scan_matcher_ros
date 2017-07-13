#ifndef _KARTO_ROS_H_
#define _KARTO_ROS_H_

#include "ros/ros.h"
#include "ros/console.h"
#include "message_filters/subscriber.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "visualization_msgs/MarkerArray.h"

#include "nav_msgs/MapMetaData.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/GetMap.h"

#include "karto_scan_matcher/Mapper.h"
#include "karto_scan_matcher/OccupancyGrid.h"

#include <boost/thread.hpp>

#include <string>
#include <map>
#include <vector>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class KartoROS
{
public:
    KartoROS();
    ~KartoROS();

    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool mapCallback(nav_msgs::GetMap::Request  &req,
                     nav_msgs::GetMap::Response &res);

private:
    bool getOdomPose(KartoScanMatcher::Pose2& karto_pose, const ros::Time& t);
    KartoScanMatcher::LaserRangeFinder* getLaser(const sensor_msgs::LaserScan::ConstPtr& scan);
    bool addScan(KartoScanMatcher::LaserRangeFinder* laser,
                 const sensor_msgs::LaserScan::ConstPtr& scan,
                 KartoScanMatcher::Pose2& karto_pose);
    bool updateMap();
    void publishTransform();
    void publishLoop(double transform_publish_period);
    void publishGraphVisualization();

private:
    // ROS handles
    ros::NodeHandle node_;
    tf::TransformListener tf_;
    tf::TransformBroadcaster* tfB_;
	
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;
	
    ros::Publisher sst_;
    ros::Publisher sstm_;
    ros::ServiceServer ss_;

    // The map that will be published / send to service callers
    nav_msgs::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
	
    int throttle_scans_;
    ros::Duration map_update_interval_;
	double resolution_;
    boost::mutex map_mutex_;
    boost::mutex map_to_odom_mutex_;

    // Karto bookkeeping
    KartoScanMatcher::Mapper* mapper_;

    std::map<std::string, KartoScanMatcher::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    // Internal state
    bool got_map_;
    int laser_count_;
    boost::thread* transform_thread_;
    tf::Transform map_to_odom_;
    unsigned marker_count_;
    bool inverted_laser_;
	
};  // KartoROS

#endif 