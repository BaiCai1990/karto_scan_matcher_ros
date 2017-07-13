#include "karto_ros/karto_ros.h"

KartoROS::KartoROS() :
        got_map_(false),
        laser_count_(0),
        transform_thread_(NULL),
        marker_count_(0)
{
	map_to_odom_.setIdentity();
	// Retrieve parameters
	ros::NodeHandle private_nh_("~");
	if(!private_nh_.getParam("odom_frame", odom_frame_))
		odom_frame_ = "odom";
	if(!private_nh_.getParam("map_frame", map_frame_))
		map_frame_ = "map";
	if(!private_nh_.getParam("base_frame", base_frame_))
		base_frame_ = "base_link";
	if(!private_nh_.getParam("throttle_scans", throttle_scans_))
		throttle_scans_ = 1;
	
	
	double tmp;
	if(!private_nh_.getParam("map_update_interval", tmp))
		tmp = 5.0;
	map_update_interval_.fromSec(tmp);
	
	if(!private_nh_.getParam("resolution", resolution_))
	{
		// Compatibility with slam_gmapping, which uses "delta" to mean
		// resolution
		if(!private_nh_.getParam("delta", resolution_))
			resolution_ = 0.05;
	}
	
	double transform_publish_period;
	private_nh_.param("transform_publish_period", transform_publish_period, 0.05);

	// Set up advertisements and subscriptions
	tfB_ = new tf::TransformBroadcaster();
	sst_ = node_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	sstm_ = node_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	ss_ = node_.advertiseService("dynamic_map", &KartoROS::mapCallback, this);
	scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(node_, "scan", 5);
	scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame_, 5);
	scan_filter_->registerCallback(boost::bind(&KartoROS::laserCallback, this, _1));

	// Create a thread to periodically publish the latest map->odom
	// transform; it needs to go out regularly, uninterrupted by potentially
	// long periods of computation in our main loop.
	transform_thread_ = new boost::thread(boost::bind(&KartoROS::publishLoop, this, transform_publish_period));

	// Initialize Karto structures
	mapper_ = new KartoScanMatcher::Mapper();

	// Setting General Parameters from the Parameter Server
	bool use_scan_matching;
	if(private_nh_.getParam("use_scan_matching", use_scan_matching))
		mapper_->setParamUseScanMatching(use_scan_matching);
	
	bool use_scan_barycenter;
	if(private_nh_.getParam("use_scan_barycenter", use_scan_barycenter))
		mapper_->setParamUseScanBarycenter(use_scan_barycenter);

	double minimum_travel_distance;
	if(private_nh_.getParam("minimum_travel_distance", minimum_travel_distance))
		mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

	double minimum_travel_heading;
	if(private_nh_.getParam("minimum_travel_heading", minimum_travel_heading))
		mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

	int scan_buffer_size;
	if(private_nh_.getParam("scan_buffer_size", scan_buffer_size))
		mapper_->setParamScanBufferSize(scan_buffer_size);

	double scan_buffer_maximum_scan_distance;
	if(private_nh_.getParam("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
		mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

	// Setting Correlation Parameters from the Parameter Server

	double correlation_search_space_dimension;
	if(private_nh_.getParam("correlation_search_space_dimension", correlation_search_space_dimension))
		mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

	double correlation_search_space_resolution;
	if(private_nh_.getParam("correlation_search_space_resolution", correlation_search_space_resolution))
		mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

	double correlation_search_space_smear_deviation;
	if(private_nh_.getParam("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
		mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);


	// Setting Scan Matcher Parameters from the Parameter Server

	double fine_search_angle_offset;
	if(private_nh_.getParam("fine_search_angle_offset", fine_search_angle_offset))
		mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

	double coarse_search_angle_offset;
	if(private_nh_.getParam("coarse_search_angle_offset", coarse_search_angle_offset))
		mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

	double coarse_angle_resolution;
	if(private_nh_.getParam("coarse_angle_resolution", coarse_angle_resolution))
		mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);
}

KartoROS::~KartoROS()
{
	if(transform_thread_)
	{
		transform_thread_->join();
		delete transform_thread_;
	}
	if (scan_filter_)
		delete scan_filter_;
	if (scan_filter_sub_)
		delete scan_filter_sub_;
	if (mapper_)
		delete mapper_;
	// TODO: delete the pointers in the lasers_ map; not sure whether or not
	// I'm supposed to do that.
}

void KartoROS::publishLoop(double transform_publish_period)
{
	if(transform_publish_period == 0)
		return;

	ros::Rate r(1.0 / transform_publish_period);
	while(ros::ok())
	{
		publishTransform();
		r.sleep();
	}
}

void KartoROS::publishTransform()
{
	boost::mutex::scoped_lock(map_to_odom_mutex_);
	ros::Time tf_expiration = ros::Time::now() + ros::Duration(0.05);
	tfB_->sendTransform(tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
}

KartoScanMatcher::LaserRangeFinder* KartoROS::getLaser(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	// Check whether we know about this laser yet
	if(lasers_.find(scan->header.frame_id) == lasers_.end()) // 说明laser_是空的
	{
		// New laser; need to create a Karto device for it.

		// Get the laser's pose, relative to base.
		tf::Stamped<tf::Pose> ident;
		tf::Stamped<tf::Transform> laser_pose;
		ident.setIdentity();
		ident.frame_id_ = scan->header.frame_id;
		ident.stamp_ = scan->header.stamp;
		try{
			tf_.transformPose(base_frame_, ident, laser_pose);
		}
		catch(tf::TransformException e){
			ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
			return NULL;
		}

		double yaw = tf::getYaw(laser_pose.getRotation());

		ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
				scan->header.frame_id.c_str(),
				laser_pose.getOrigin().x(),
				laser_pose.getOrigin().y(),
				yaw);
		// To account for lasers that are mounted upside-down,
		// we create a point 1m above the laser and transform it into the laser frame
		// if the point's z-value is <=0, it is upside-down

		tf::Vector3 v;
		v.setValue(0, 0, 1 + laser_pose.getOrigin().z());
		tf::Stamped<tf::Vector3> up(v, scan->header.stamp, base_frame_);

		try{
			tf_.transformPoint(scan->header.frame_id, up, up);
			ROS_DEBUG("Z-Axis in sensor frame: %.3f", up.z());
		}
		catch (tf::TransformException& e){
			ROS_WARN("Unable to determine orientation of laser: %s", e.what());
			return NULL;
		}

		bool inverse = lasers_inverted_[scan->header.frame_id] = up.z() <= 0;
		if (inverse)
			ROS_INFO("laser is mounted upside-down");


		// Create a laser range finder device and copy in data from the first
		// scan
		KartoScanMatcher::LaserRangeFinder* object;
		std::string name = scan->header.frame_id;
		KartoScanMatcher::LaserRangeFinder* laser = object->CreateLaserRangeFinder(KartoScanMatcher::LaserRangeFinder_Custom, std::string(name));
		laser->SetOffsetPose(KartoScanMatcher::Pose2(laser_pose.getOrigin().x(), laser_pose.getOrigin().y(), yaw));
		laser->SetMinimumRange(scan->range_min);
		laser->SetMaximumRange(scan->range_max);
		laser->SetMinimumAngle(scan->angle_min);
		laser->SetMaximumAngle(scan->angle_max);
		laser->SetAngularResolution(scan->angle_increment);
		// TODO: expose this, and many other parameters
		laser->SetRangeThreshold(12.0);
		ROS_DEBUG("laser range threshold: %.3f", laser->GetRangeThreshold());

		// Store this laser device for later
		lasers_[scan->header.frame_id] = laser;
	}

	return lasers_[scan->header.frame_id];
}

bool KartoROS::getOdomPose(KartoScanMatcher::Pose2& karto_pose, const ros::Time& t)
{
	// Get the robot's pose
	tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0), tf::Vector3(0,0,0)), t, base_frame_);
	tf::Stamped<tf::Transform> odom_pose;
	try{
		tf_.transformPose(odom_frame_, ident, odom_pose);
	}
	catch(tf::TransformException e){
		ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
		return false;
	}
	double yaw = tf::getYaw(odom_pose.getRotation());

	karto_pose = KartoScanMatcher::Pose2(odom_pose.getOrigin().x(),
							odom_pose.getOrigin().y(),
							yaw);
	return true;
}

void KartoROS::laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
	laser_count_++;
	if ((laser_count_ % throttle_scans_) != 0)
		return;

	static ros::Time last_map_update(0,0);

	// Check whether we know about this laser yet
	KartoScanMatcher::LaserRangeFinder* laser = getLaser(scan);

	if(!laser)
	{
		ROS_WARN("Failed to create laser device for %s; discarding scan", scan->header.frame_id.c_str());
		return;
	}

	KartoScanMatcher::Pose2 odom_pose;
	
	if(addScan(laser, scan, odom_pose))
	{
		ROS_DEBUG("added scan at pose: %.3f %.3f %.3f", 
				odom_pose.GetX(),
				odom_pose.GetY(),
				odom_pose.GetHeading());

		if(!got_map_ || (scan->header.stamp - last_map_update) > map_update_interval_)
		{
			
			if(updateMap())
			{
				last_map_update = scan->header.stamp;
				got_map_ = true;
				ROS_DEBUG("Updated the map");
			}
		}
	}
}

bool KartoROS::updateMap()
{
	boost::mutex::scoped_lock(map_mutex_);

	KartoScanMatcher::OccupancyGrid* occ_grid = KartoScanMatcher::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

	if(!occ_grid)
		return false;

	if(!got_map_) // 初始化
	{
		map_.map.info.resolution = resolution_;
		map_.map.info.origin.position.x = 0.0;
		map_.map.info.origin.position.y = 0.0;
		map_.map.info.origin.position.z = 0.0;
		map_.map.info.origin.orientation.x = 0.0;
		map_.map.info.origin.orientation.y = 0.0;
		map_.map.info.origin.orientation.z = 0.0;
		map_.map.info.origin.orientation.w = 1.0;
	} 

	// Translate to ROS format
	kt_int32s width = occ_grid->GetWidth();
	kt_int32s height = occ_grid->GetHeight();
	KartoScanMatcher::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

	if(map_.map.info.width != (unsigned int) width || 
		map_.map.info.height != (unsigned int) height ||
		map_.map.info.origin.position.x != offset.GetX() ||
		map_.map.info.origin.position.y != offset.GetY())
	{
		map_.map.info.origin.position.x = offset.GetX();
		map_.map.info.origin.position.y = offset.GetY();
		map_.map.info.width = width;
		map_.map.info.height = height;
		map_.map.data.resize(map_.map.info.width * map_.map.info.height);
	}

	for (kt_int32s y=0; y<height; y++)
	{
		for (kt_int32s x=0; x<width; x++) 
		{
			// Getting the value at position x,y
			kt_int8u value = occ_grid->GetValue(KartoScanMatcher::Vector2<kt_int32s>(x, y));

			switch (value)
			{
				case KartoScanMatcher::GridStates_Unknown:
					map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
					break;
				case KartoScanMatcher::GridStates_Occupied:
					map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
					break;
				case KartoScanMatcher::GridStates_Free:
					map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
					break;
				default:
					ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
					break;
			}
		}
	}
	
	// Set the header information on the map
	map_.map.header.stamp = ros::Time::now();
	map_.map.header.frame_id = map_frame_;

	sst_.publish(map_.map);
	sstm_.publish(map_.map.info);

	delete occ_grid;

	return true;
}

bool KartoROS::addScan(KartoScanMatcher::LaserRangeFinder* laser, const sensor_msgs::LaserScan::ConstPtr& scan, KartoScanMatcher::Pose2& karto_pose)
{
	if(!getOdomPose(karto_pose, scan->header.stamp))
		return false;
	
	// Create a vector of doubles for karto
	std::vector<kt_double> readings;

	if (lasers_inverted_[scan->header.frame_id]) 
	{
		for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin(); it != scan->ranges.rend(); ++it)
		{
			readings.push_back(*it);
		}
	}
	else 
	{
		for(std::vector<float>::const_iterator it = scan->ranges.begin(); it != scan->ranges.end(); ++it)
		{
			readings.push_back(*it);
		}
	}
	
	// Create localized range scan
	KartoScanMatcher::LocalizedRangeScan* range_scan = new KartoScanMatcher::LocalizedRangeScan(laser, readings);
	range_scan->SetOdometricPose(karto_pose);
	range_scan->SetCorrectedPose(karto_pose);

	// Add the localized range scan to the mapper
	bool processed;
	if((processed = mapper_->Process(range_scan)))
	{
		//std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " << range_scan->GetCorrectedPose() << std::endl;
		
		KartoScanMatcher::Pose2 corrected_pose = range_scan->GetCorrectedPose(); // 校正后的机器人位姿

		// Compute the map->odom transform
		tf::Stamped<tf::Pose> odom_to_map;
		try{
			tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, corrected_pose.GetHeading()),
																 tf::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(),
																 scan->header.stamp, base_frame_),odom_to_map);
		}
		catch(tf::TransformException e){
			ROS_ERROR("Transform from base_link to odom failed\n");
			odom_to_map.setIdentity();
		}

		map_to_odom_mutex_.lock();
		map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
									 tf::Point(      odom_to_map.getOrigin() ) ).inverse();
		map_to_odom_mutex_.unlock();

	}
	else
		delete range_scan;

	return processed;
}

bool KartoROS::mapCallback(nav_msgs::GetMap::Request& req, nav_msgs::GetMap::Response& res)
{
	boost::mutex::scoped_lock(map_mutex_);
	if(got_map_ && map_.map.info.width && map_.map.info.height)
	{
		res = map_;
		return true;
	}
	else
		return false;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "karto_ros");

	KartoROS karto_ros;

	ros::spin();

	return 0;
}
