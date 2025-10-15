#include "gmapper/mapper.h"
#include <nlohmann/json.hpp>
#include "segmentation_interfaces/msg/segmentation_result.hpp"

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

using std::placeholders::_1;

SlamGmapping::SlamGmapping():
    Node("semap"),
    scan_filter_sub_(nullptr),
    scan_filter_(nullptr),
    laser_count_(0),
    transform_thread_(nullptr)
{
    buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
     auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    buffer_->setCreateTimerInterface(timer_interface);
    tfl_ = std::make_shared<tf2_ros::TransformListener>(*buffer_);
    node_ = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node *) {});
    tfB_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
    map_to_odom_.setIdentity();
    seed_ = static_cast<unsigned long>(time(nullptr));
    init();
    startLiveSlam();
}

void SlamGmapping::init() {
    gsp_ = new GMapping::GridSlamProcessor();

    gsp_laser_ = nullptr;
    gsp_odom_ = nullptr;
    got_first_scan_ = false;
    got_map_ = false;

    throttle_scans_ = 1;
    base_frame_ = "fasem_link";
    map_frame_ = "map";
    odom_frame_ = "fasem_odom";
    transform_publish_period_ = 0.05;

    map_update_interval_ = tf2::durationFromSec(0.5);
    maxUrange_ = 80.0;  maxRange_ = 0.0;
    minimum_score_ = 0;
    sigma_ = 0.05;
    kernelSize_ = 1;
    lstep_ = 0.05;
    astep_ = 0.05;
    iterations_ = 5;
    lsigma_ = 0.075;
    ogain_ = 3.0;
    lskip_ = 0;
    srr_ = 0.1;
    srt_ = 0.2;
    str_ = 0.1;
    stt_ = 0.2;
    linearUpdate_ = 1.0;
    angularUpdate_ = 0.5;
    temporalUpdate_ = 1.0;
    resampleThreshold_ = 0.5;
    particles_ = 30;
    xmin_ = -10.0;
    ymin_ = -10.0;
    xmax_ = 10.0;
    ymax_ = 10.0;
    delta_ = 0.05;
    occ_thresh_ = 0.25;
    llsamplerange_ = 0.01;
    llsamplestep_ = 0.01;
    lasamplerange_ = 0.005;
    lasamplestep_ = 0.005;
    tf_delay_ = transform_publish_period_;

    labelFails_ = std::vector<int>(360, 100);
    labelJSON_ = nlohmann::json();
}

void SlamGmapping::startLiveSlam() {
    // Topics Initiation
    entropy_publisher_ = this->create_publisher<std_msgs::msg::Float64>("entropy", rclcpp::SystemDefaultsQoS());
    sst_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::SystemDefaultsQoS());
    sstm_ = this->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rclcpp::SystemDefaultsQoS());
    scan_filter_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>
            (node_, "fasem_scan", rclcpp::SensorDataQoS().get_rmw_qos_profile());
    scan_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>
            (*scan_filter_sub_, *buffer_, odom_frame_, 10, node_);
    scan_filter_->registerCallback(std::bind(&SlamGmapping::laserCallback, this, std::placeholders::_1));
    transform_thread_ = std::make_shared<std::thread>
            (std::bind(&SlamGmapping::publishLoop, this, transform_publish_period_));
    label_sub_ = this->create_subscription<segmentation_interfaces::msg::SegmentationResult>(
        "/segmentation_result", rclcpp::SystemDefaultsQoS(),
        std::bind(&SlamGmapping::labelCallback, this, std::placeholders::_1)
    );
    
    // Publish initial identity transform immediately
    publishTransform();
}

void SlamGmapping::labelCallback(const segmentation_interfaces::msg::SegmentationResult::SharedPtr msg)
{
    try {
        nlohmann::json j;
        j["count"] = msg->count;
        j["timestamp"] = msg->timestamp;
        j["detected"] = msg->detected;
        
        // Protect labelJSON_ with mutex during write operation
        std::lock_guard<std::mutex> lock(labelJSON_mutex_);
        labelJSON_ = j;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error processing segmentation result: %s", e.what());
    }
}

void SlamGmapping::publishLoop(double transform_publish_period){
    if (transform_publish_period == 0)
        return;
    
    // Use higher frequency for more consistent transform publishing
    double effective_period = std::min(transform_publish_period, 0.02); // Max 50Hz
    rclcpp::Rate r(1.0 / effective_period);
    
    while (rclcpp::ok()) {
        publishTransform();
        r.sleep();
    }
}

SlamGmapping::~SlamGmapping()
{
    if(transform_thread_){
        transform_thread_->join();
    }

    delete gsp_;
    delete gsp_laser_;
    delete gsp_odom_;
}

bool SlamGmapping::getOdomPose(GMapping::OrientedPoint& gmap_pose, const rclcpp::Time& t)
{
    // Get the pose of the centered laser at the right time
    centered_laser_pose_.header.stamp = t;
    // Get the laser's pose that is centered
    geometry_msgs::msg::PoseStamped odom_pose;
    try
    {
        buffer_->transform(centered_laser_pose_, odom_pose, odom_frame_, tf2::durationFromSec(1.0));
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }

    double yaw = tf2::getYaw(odom_pose.pose.orientation);

    gmap_pose = GMapping::OrientedPoint(odom_pose.pose.position.x,
                                        odom_pose.pose.position.y,
                                        yaw);
    return true;
}

bool SlamGmapping::initMapper(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    laser_frame_ = scan->header.frame_id;
    // Get the laser's pose, relative to base.
    geometry_msgs::msg::PoseStamped ident;
    geometry_msgs::msg::PoseStamped laser_pose;

    try{
        ident.header.frame_id = laser_frame_;
        ident.header.stamp = scan->header.stamp;
        tf2::Transform transform;
        transform.setIdentity();
        tf2::toMsg(transform, ident.pose);
        buffer_->transform(ident, laser_pose, base_frame_);
    }
    catch (tf2::TransformException& e){
        RCLCPP_WARN(this->get_logger(), "Failed to compute laser pose, aborting initialization (%s)", e.what());
        return false;
    }

    // create a point 1m above the laser position and transform it into the laser-frame
    geometry_msgs::msg::PointStamped up;
    up.header.stamp = scan->header.stamp;
    up.header.frame_id = base_frame_;
    up.point.x = up.point.y = 0;
    up.point.z = 1 + laser_pose.pose.position.z;
    try
    {
        buffer_->transform(up, up, laser_frame_);
    }
    catch(tf2::TransformException& e)
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine orientation of laser: %s", e.what());
        return false;
    }

    // gmapping doesnt take roll or pitch into account. So check for correct sensor alignment.
    if (fabs(fabs(up.point.z) - 1) > 0.001)
    {
        RCLCPP_INFO(this->get_logger(),
                "Laser has to be mounted planar! Z-coordinate has to be 1 or -1, but gave: %.5f", up.point.z);
        return false;
    }

    gsp_laser_beam_count_ = static_cast<unsigned int>(scan->ranges.size());

    double angle_center = (scan->angle_min + scan->angle_max)/2;

    centered_laser_pose_.header.frame_id = laser_frame_;
    centered_laser_pose_.header.stamp = get_clock()->now();
    tf2::Quaternion q;

    if (up.point.z > 0)
    {
        do_reverse_range_ = scan->angle_min > scan->angle_max;
        q.setEuler(angle_center, 0, 0);
        RCLCPP_INFO(this->get_logger(),"Laser is mounted upwards.");
    }
    else
    {
        do_reverse_range_ = scan->angle_min < scan->angle_max;
        q.setEuler(-angle_center, 0, M_PI);
        RCLCPP_INFO(this->get_logger(), "Laser is mounted upside down.");
    }

    centered_laser_pose_.pose.position.x = 0;
    centered_laser_pose_.pose.position.y = 0;
    centered_laser_pose_.pose.position.z = 0;

    centered_laser_pose_.pose.orientation.w = q.getW();
    centered_laser_pose_.pose.orientation.x = q.getX();
    centered_laser_pose_.pose.orientation.y = q.getY();
    centered_laser_pose_.pose.orientation.z = q.getZ();

    // Compute the angles of the laser from -x to x, basically symmetric and in increasing order
    laser_angles_.resize(scan->ranges.size());
    // Make sure angles are started so that they are centered
    double theta = - std::fabs(scan->angle_min - scan->angle_max)/2;
    for(unsigned int i=0; i<scan->ranges.size(); ++i)
    {
        laser_angles_[i]=theta;
        theta += std::fabs(scan->angle_increment);
    }

    RCLCPP_DEBUG(this->get_logger(), "Laser angles in laser-frame: min: %.3f max: %.3f inc: %.3f",
            scan->angle_min, scan->angle_max, scan->angle_increment);
    RCLCPP_DEBUG(this->get_logger(), "Laser angles in top-down centered laser-frame: min: %.3f max: %.3f inc: %.3f",
            laser_angles_.front(), laser_angles_.back(), std::fabs(scan->angle_increment));

    GMapping::OrientedPoint gmap_pose(0, 0, 0);

    // setting maxRange and maxUrange here so we can set a reasonable default
    maxRange_ = scan->range_max - 0.01;
    maxUrange_ = maxRange_;

    // The laser must be called "FLASER".
    // We pass in the absolute value of the computed angle increment, on the
    // assumption that GMapping requires a positive angle increment.  If the
    // actual increment is negative, we'll swap the order of ranges before
    // feeding each scan to GMapping.
    gsp_laser_ = new GMapping::RangeSensor("FLASER", gsp_laser_beam_count_, fabs(scan->angle_increment), gmap_pose,
                                           0.0, maxRange_);

    GMapping::SensorMap smap;
    smap.insert(make_pair(gsp_laser_->getName(), gsp_laser_));
    gsp_->setSensorMap(smap);

    gsp_odom_ = new GMapping::OdometrySensor(odom_frame_);

    /// @todo Expose setting an initial pose
    GMapping::OrientedPoint initialPose;
    if(!getOdomPose(initialPose, scan->header.stamp))
    {
        RCLCPP_WARN(this->get_logger(), "Unable to determine inital pose of laser! Starting point will be set to zero.");
        initialPose = GMapping::OrientedPoint(0.0, 0.0, 0.0);
    }

    gsp_->setMatchingParameters(maxUrange_, maxRange_, sigma_,
                                kernelSize_, lstep_, astep_, iterations_,
                                lsigma_, ogain_, static_cast<unsigned int>(lskip_));

    gsp_->setMotionModelParameters(srr_, srt_, str_, stt_);
    gsp_->setUpdateDistances(linearUpdate_, angularUpdate_, resampleThreshold_);
    gsp_->setUpdatePeriod(temporalUpdate_);
    gsp_->setgenerateMap(false);
    gsp_->GridSlamProcessor::init(static_cast<unsigned int>(particles_), xmin_, ymin_, xmax_, ymax_,
                                  delta_, initialPose);
    gsp_->setllsamplerange(llsamplerange_);
    gsp_->setllsamplestep(llsamplestep_);
    /// @todo Check these calls; in the gmapping gui, they use
    /// llsamplestep and llsamplerange intead of lasamplestep and
    /// lasamplerange.  It was probably a typo, but who knows.
    gsp_->setlasamplerange(lasamplerange_);
    gsp_->setlasamplestep(lasamplestep_);
    gsp_->setminimumScore(minimum_score_);

    // Call the sampling function once to set the seed.
    GMapping::sampleGaussian(1, static_cast<unsigned int>(seed_));

    RCLCPP_INFO(this->get_logger(), "Initialization complete");

    return true;
}

bool SlamGmapping::addScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan, GMapping::OrientedPoint& gmap_pose) {
    if (!getOdomPose(gmap_pose, scan->header.stamp))
        return false;

    if (scan->ranges.size() != gsp_laser_beam_count_)
        return false;

    // GMapping wants an array of doubles...
    auto *ranges_double = new double[scan->ranges.size()];
    // If the angle increment is negative, we have to invert the order of the readings.
    if (do_reverse_range_) {
        RCLCPP_DEBUG(this->get_logger(), "Inverting scan");
        int num_ranges = static_cast<int>(scan->ranges.size());
        for (int i = 0; i < num_ranges; i++) {
            // Must filter out short readings, because the mapper won't
            if (scan->ranges[num_ranges - i - 1] < scan->range_min)
                ranges_double[i] = (double) scan->range_max;
            else
                ranges_double[i] = (double) scan->ranges[num_ranges - i - 1];
        }
    } else {
        for (unsigned int i = 0; i < scan->ranges.size(); i++) {
            // Must filter out short readings, because the mapper won't
            if (scan->ranges[i] < scan->range_min)
                ranges_double[i] = (double) scan->range_max;
            else
                ranges_double[i] = (double) scan->ranges[i];
        }
    }

    GMapping::RangeReading reading(static_cast<unsigned int>(scan->ranges.size()),
                                   ranges_double,
                                   gsp_laser_,
                                   scan->header.stamp.sec);

    // ...but it deep copies them in RangeReading constructor, so we don't
    // need to keep our array around.
    delete[] ranges_double;

    reading.setPose(gmap_pose);

    RCLCPP_DEBUG(this->get_logger(), "processing scan");

    return gsp_->processScan(reading);
}


void SlamGmapping::laserCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    last_scan_timestamp_ = get_clock()->now();
    laser_count_++;
    if ((laser_count_ % throttle_scans_) != 0)
        return;

    tf2::TimePoint last_map_update = tf2::TimePointZero;

    // We can't initialize the mapper until we've got the first scan
    if(!got_first_scan_)
    {
        if(!initMapper(scan))
            return;
        got_first_scan_ = true;
    }
    auto new_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
    
    // Modify the intensities
    std::vector<float> intensities = new_scan->intensities; // Create a non-const copy
    for (size_t i = 0; i < new_scan->ranges.size(); ++i) {
        intensities.push_back(new_scan->ranges[i]);
    }
    new_scan->intensities = intensities;

    GMapping::OrientedPoint odom_pose;

    if(addScan(new_scan, odom_pose))
    {
        GMapping::OrientedPoint mpose = gsp_->getParticles()[gsp_->getBestParticleIndex()].pose;

        tf2::Quaternion q;
        q.setRPY(0, 0, mpose.theta);
        tf2::Transform laser_to_map = tf2::Transform(q, tf2::Vector3(mpose.x, mpose.y, 0.0)).inverse();
        q.setRPY(0, 0, odom_pose.theta);
        tf2::Transform odom_to_laser = tf2::Transform(q, tf2::Vector3(odom_pose.x, odom_pose.y, 0.0));

        map_to_odom_mutex_.lock();
        map_to_odom_ = (odom_to_laser * laser_to_map).inverse();
        map_to_odom_mutex_.unlock();

        tf2::TimePoint timestamp = tf2_ros::fromMsg(new_scan->header.stamp);
        if(!got_map_ || (timestamp - last_map_update) > map_update_interval_)
        {
            updateMap(new_scan);
            last_map_update = tf2_ros::fromMsg(new_scan->header.stamp);
        }
    }
}

double SlamGmapping::computePoseEntropy()
{
    double weight_total=0.0;
    for (const auto &it : gsp_->getParticles()) {
        weight_total += it.weight;
    }
    double entropy = 0.0;
    for (const auto &it : gsp_->getParticles()) {
        if(it.weight/weight_total > 0.0)
            entropy += it.weight/weight_total * log(it.weight/weight_total);
    }
    return -entropy;
}

void SlamGmapping::updateMap(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan)
{
    RCLCPP_WARN(this->get_logger(), "Trying to update the map");
    map_mutex_.lock();

    GMapping::ScanMatcher matcher;

    matcher.setLaserParameters(static_cast<unsigned int>(scan->ranges.size()), &(laser_angles_[0]),
                               gsp_laser_->getPose());

    GMapping::GridSlamProcessor::Particle best =
            gsp_->getParticles()[gsp_->getBestParticleIndex()];
            
    int rounded_theta = static_cast<int>(std::round(best.pose.theta * 180.0 / M_PI));
    rounded_theta = ((rounded_theta % 360) + 360) % 360;
    RCLCPP_WARN(this->get_logger(), "Robot view angle: %d degrees", rounded_theta);

    bool segmentation_error = false;
    if (launch_pose_rotation.size() < 10) {
        launch_pose_rotation.push_back(rounded_theta);
        RCLCPP_WARN(this->get_logger(), "Odometry pose drift is still being collected, size: %zu", launch_pose_rotation.size());
        map_mutex_.unlock();
        labelReadsCollection_.push_back(labelFails_);
        return;
    }
    else {
        RCLCPP_WARN(this->get_logger(), "Calculating average initial rotation from the last 50 elements");
        // Calculate the average of the last 50 elements
        int sum = 0;
        for (size_t i = 0; i < launch_pose_rotation.size(); ++i) {
            sum += launch_pose_rotation[i];
        }
        average_launch_rotation = sum / static_cast<int>(launch_pose_rotation.size());
    }
    RCLCPP_WARN(this->get_logger(), "Average initial rotation: %d", average_launch_rotation);

    // Create a local copy of labelJSON_ to avoid blocking during processing
    nlohmann::json local_labelJSON;
    {
        std::lock_guard<std::mutex> lock(labelJSON_mutex_);
        local_labelJSON = labelJSON_;
    }

    if (local_labelJSON.is_null() || local_labelJSON.empty()) {
        segmentation_error = true;
        RCLCPP_WARN(this->get_logger(), "No segmentation data, filled with zeros..");
    } else {
        if (!local_labelJSON.contains("detected") || !local_labelJSON["detected"].is_array() || local_labelJSON["detected"].size() != 360) {
            segmentation_error = true;
            RCLCPP_WARN(this->get_logger(), "Segmentation data is not valid, filled with zeros..");
        }
    }

    // Segmentation data shifting based on the robot's quaternion
    // We assume that the label data is in the same order as the laser scan
    // and that it is 360 elements long, corresponding to the 360 degrees of the laser scan.
    if (!segmentation_error) {
        labelReads_ = local_labelJSON["detected"].get<std::vector<int>>();
        int shift = ((rounded_theta - average_launch_rotation) % 360 + 360) % 360;
        std::vector<int> rotated_labelReads(360);
        for (size_t i = 0; i < 360; ++i) {
            rotated_labelReads[(i + shift) % 360] = labelReads_[i];
        }
        labelReadsCollection_.push_back(rotated_labelReads);
    }
    else {
        labelReadsCollection_.push_back(labelFails_);
        RCLCPP_WARN(this->get_logger(), "Filled, resuming..");
    }

    RCLCPP_WARN(this->get_logger(), "Step 1");
    matcher.setlaserMaxRange(maxRange_);
    matcher.setusableRange(maxUrange_);
    matcher.setgenerateMap(true);

    RCLCPP_WARN(this->get_logger(), "Step 2");
    std_msgs::msg::Float64 entropy;
    entropy.data = computePoseEntropy();
    if(entropy.data > 0.0)
        entropy_publisher_->publish(entropy);

    RCLCPP_WARN(this->get_logger(), "Step 3");
    if(!got_map_) {
        map_.info.resolution = static_cast<nav_msgs::msg::MapMetaData::_resolution_type>(delta_);
        map_.info.origin.position.x = 0.0;
        map_.info.origin.position.y = 0.0;
        map_.info.origin.position.z = 0.0;
        map_.info.origin.orientation.x = 0.0;
        map_.info.origin.orientation.y = 0.0;
        map_.info.origin.orientation.z = 0.0;
        map_.info.origin.orientation.w = 1.0;
    }

    RCLCPP_WARN(this->get_logger(), "Step 4");
    GMapping::Point center;
    center.x=(xmin_ + xmax_) / 2.0;
    center.y=(ymin_ + ymax_) / 2.0;

    RCLCPP_WARN(this->get_logger(), "Step 5");
    GMapping::ScanMatcherMap smap(center, xmin_, ymin_, xmax_, ymax_,
                                  delta_);

    RCLCPP_WARN(this->get_logger(), "Step 6");
    int indexer = 0;
    int collection_length = static_cast<int>(labelReadsCollection_.size());
    RCLCPP_DEBUG(this->get_logger(), "Trajectory tree:");
    for(GMapping::GridSlamProcessor::TNode* n = best.node; n; n = n->parent)
    {
        RCLCPP_DEBUG(this->get_logger(), "  %.3f %.3f %.3f",
                  n->pose.x,
                  n->pose.y,
                  n->pose.theta);
        if(!n->reading)
        {
            RCLCPP_DEBUG(this->get_logger(), "Reading is NULL");
            continue;
        }
        indexer++;
        matcher.invalidateActiveArea();
        matcher.computeActiveArea(smap, n->pose, &((*n->reading)[0]));
        matcher.registerScan(smap, n->pose, &((*n->reading)[0]), labelReadsCollection_[collection_length - indexer]);
    }
    
    RCLCPP_WARN(this->get_logger(), "=====================================");
    if (!segmentation_error && !local_labelJSON.is_null() && !local_labelJSON.empty()) {
        RCLCPP_WARN(this->get_logger(), "Label number %d received", local_labelJSON["count"].get<int>());
    } else {
        RCLCPP_WARN(this->get_logger(), "No valid segmentation data available");
    }
    RCLCPP_WARN(this->get_logger(), "=====================================");
    // RCLCPP_WARN(this->get_logger(), "============================");
    // RCLCPP_WARN(this->get_logger(), "===========valid nodes: %d", indexer);
    // RCLCPP_WARN(this->get_logger(), "===========total label: %d", collection_length);
    // RCLCPP_WARN(this->get_logger(), "============================");

    // the map may have expanded, so resize ros message as well
    if(map_.info.width != (unsigned int) smap.getMapSizeX() || map_.info.height != (unsigned int) smap.getMapSizeY()) {

        // NOTE: The results of ScanMatcherMap::getSize() are different from the parameters given to the constructor
        //       so we must obtain the bounding box in a different way
        GMapping::Point wmin = smap.map2world(GMapping::IntPoint(0, 0));
        GMapping::Point wmax = smap.map2world(GMapping::IntPoint(smap.getMapSizeX(), smap.getMapSizeY()));
        xmin_ = wmin.x; ymin_ = wmin.y;
        xmax_ = wmax.x; ymax_ = wmax.y;
        
        map_.info.width = static_cast<nav_msgs::msg::MapMetaData::_width_type>(smap.getMapSizeX());
        map_.info.height = static_cast<nav_msgs::msg::MapMetaData::_height_type>(smap.getMapSizeY());
        map_.info.origin.position.x = xmin_;
        map_.info.origin.position.y = ymin_;
        map_.data.resize(map_.info.width * map_.info.height);
    }

    for(int x=0; x < smap.getMapSizeX(); x++)
    {
        for(int y=0; y < smap.getMapSizeY(); y++)
        {
            /// @todo Sort out the unknown vs. free vs. obstacle thresholding
            GMapping::IntPoint p(x, y);
            double occ=smap.cell(p);
            int label = smap.cell(p).getLabel();
            
            assert(occ <= 1.0);
            if(occ < 0) {
                map_.data[MAP_IDX(map_.info.width, x, y)] = -1;
            }
            else if(occ > occ_thresh_)
            {
                map_.data[MAP_IDX(map_.info.width, x, y)] = label;
            }
            else {
                map_.data[MAP_IDX(map_.info.width, x, y)] = 0;
            }
        }
    }
    got_map_ = true;

    //make sure to set the header information on the map
    map_.header.stamp = get_clock()->now();
    map_.header.frame_id = map_frame_;

    sst_->publish(map_);
    sstm_->publish(map_.info);

    map_mutex_.unlock();
    RCLCPP_WARN(this->get_logger(), "Updated map successfully!");
}

void SlamGmapping::publishTransform()
{
    map_to_odom_mutex_.lock();
    // Use current time instead of future time to avoid timing issues
    rclcpp::Time current_time = get_clock()->now();
    geometry_msgs::msg::TransformStamped transform;
    transform.header.frame_id = map_frame_;
    transform.header.stamp = current_time;
    transform.child_frame_id = odom_frame_;
    try {
        transform.transform = tf2::toMsg(map_to_odom_);
        tfB_->sendTransform(transform);
    }
    catch (tf2::LookupException& te){
        RCLCPP_INFO(this->get_logger(), te.what());
    }
    map_to_odom_mutex_.unlock();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto slam_gmapping_node = std::make_shared<SlamGmapping>();
    rclcpp::spin(slam_gmapping_node);
    rclcpp::shutdown();
    return(0);
}