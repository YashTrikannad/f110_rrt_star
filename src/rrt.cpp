// This file contains the class definition of RRT

#include "f110_rrt/rrt.h"
#include "f110_rrt/csv_reader.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <thread>
#include <chrono>

static const bool debug = true;

/// RRT Object Constructor
/// @param nh - node handle to the ros node
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()), tf2_listener_(tf_buffer_)
{
    // ROS Topic Names
    std::string pose_topic, scan_topic, drive_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("drive_topic", drive_topic);

    // Load Input Map from map_server
    input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));

    if(input_map_.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }
    ROS_INFO("Map Load Successful.");

    // Read Global Path
    f110::CSVReader reader("/home/yash/yasht_ws/src/f110_rrt/sensor_data/gtpose_fast.csv");
    global_path_ = reader.getData();

    // get transform from laser to map
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    // ROS Publishers and Subscribers
    dynamic_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);
    waypoint_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("waypoint_viz_marker", 1000);

    scan_sub_ = nh_.subscribe(scan_topic, 1, &RRT::scan_callback, this);
    sleep(1);
    pose_sub_ = nh_.subscribe(pose_topic, 1, &RRT::pose_callback, this);
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    tree_viz_pub_ = nh_.advertise<visualization_msgs::Marker>("tree_viz_marker", 1);

    // Tree Visualization
    points_.header.frame_id = line_strip_.header.frame_id = "/map";
    points_.ns = "rrt_viz";
    points_.pose.orientation.w = line_strip_.pose.orientation.w = 1.0;
    points_.id = 0;
    line_strip_.id = 1;
    points_.type = visualization_msgs::Marker::POINTS;
    line_strip_.type = visualization_msgs::Marker::LINE_LIST;
    points_.scale.x = 0.1;
    points_.scale.y = 0.1;
    line_strip_.scale.x = 0.05;
    points_.color.g = 1.0f;
    points_.color.a = 1.0;
    line_strip_.color.r = 1.0;
    line_strip_.color.a = 1.0;

    // Waypoint Visualization
    unique_id_ = 0;

    // Map Data
    map_cols_ = input_map_.info.width;
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

    // RRT Parameters
    nh_.getParam("max_rrt_iters", max_rrt_iters_);
    nh_.getParam("max_expansion_distance", max_expansion_distance_);
    nh_.getParam("collision_checking_points", collision_checking_points_);
    nh_.getParam("goal_tolerance", goal_tolerance_);
    nh_.getParam("local_trackpoint_tolerance", local_trackpoint_tolerance_);
    nh_.getParam("lookahead_distance", lookahead_distance_);
    nh_.getParam("local_lookahead_distance", local_lookahead_distance_);

    // Car Parameters
    nh_.getParam("high_speed", high_speed_);
    nh_.getParam("medium_speed", medium_speed_);
    nh_.getParam("low_speed", low_speed_);

    // Local Map Parameters
    nh_.getParam("length_local_map", length_local_map_);
    nh_.getParam("width_local_map", width_local_map_);

    ROS_INFO("Created new RRT Object.");
}

/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    try
    {
        tf_laser_to_map_ = tf_buffer_.lookupTransform("map", "laser", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }
    const auto translation = tf_laser_to_map_.transform.translation;
    const double yaw = tf::getYaw(tf_laser_to_map_.transform.rotation);

    const auto start = static_cast<int>(scan_msg->ranges.size()/3);
    const auto end = static_cast<int>(2*scan_msg->ranges.size()/3);
    const double angle_increment = scan_msg->angle_increment;
    double theta = scan_msg->angle_min + angle_increment*(start-1);

    for(int i=start; i<end; ++i)
    {
        theta+=angle_increment;

        const double hit_range = scan_msg->ranges[i];
        if(std::isinf(hit_range) || std::isnan(hit_range)) continue;

        const double x_base_link = hit_range*cos(theta);
        const double y_base_link = hit_range*sin(theta);

        if(x_base_link > 5 || y_base_link > 2) continue;

        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.x;
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.y;

        const auto index = get_row_major_index(x_map, y_map);

        if(input_map_.data[index] != 100)
        {
            input_map_.data[index] = 100;
            new_obstacles_.emplace_back(index);
        }
    }

    clear_obstacles_count_++;
    if(clear_obstacles_count_ > 5)
    {
        for(const auto index: new_obstacles_)
        {
            input_map_.data[index] = 0;
        }
        new_obstacles_.clear();
        clear_obstacles_count_ = 0;
        ROS_DEBUG("Obstacles Cleared");
    }

//    dynamic_map_pub_.publish(input_map_);
    ROS_DEBUG("Map Updated");
}

/// The pose callback when subscribed to particle filter's inferred pose (RRT Main Loop)
/// @param pose_msg - pointer to the incoming pose message
void RRT::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
    current_x_ = pose_msg->pose.position.x;
    current_y_ = pose_msg->pose.position.y;

    if(unique_id_ == 0)
    {
        visualize_waypoint_data();
    }

    const auto trackpoint = get_best_global_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

    std::vector<Node> tree;

    tree.emplace_back(Node(pose_msg->pose.position.x, pose_msg->pose.position.y, -1));

    int count = 0;
    while(count < max_rrt_iters_)
    {
        count++;

        // Sample a Node
        const auto sample_node = sample();

        // Check if it is not occupied
        if(is_collided(sample_node[0], sample_node[1]))
        {
            ROS_DEBUG("Sample Node Colliding");
            continue;
        }

        // Find the nearest node in the tree to the sample node
        const int nearest_node_id = nearest(tree, sample_node);

        // Steer the tree from the nearest node towards the sample node
        const Node new_node = steer(tree[nearest_node_id], nearest_node_id, sample_node);

        // Check if the segment joining the two nodes is in collision
        if(is_collided(tree[nearest_node_id], new_node))
        {
            ROS_DEBUG("Sample Node Edge Colliding");
            continue;
        }
        ROS_DEBUG("Sample Node Edge Found");
        if(is_goal(new_node, trackpoint[0], trackpoint[1]))
        {
            ROS_DEBUG("Goal reached. Backtracking ...");
            local_path_ = find_path(tree, new_node);
            ROS_INFO("Path Found");

            const auto trackpoint_and_distance =
                    get_best_local_trackpoint({pose_msg->pose.position.x,pose_msg->pose.position.y});

            const auto local_trackpoint = trackpoint_and_distance.first;
            const double distance = trackpoint_and_distance.second;

            geometry_msgs::Pose goal_way_point;
            goal_way_point.position.x = local_trackpoint[0];
            goal_way_point.position.y = local_trackpoint[1];
            goal_way_point.position.z = 0;
            goal_way_point.orientation.x = 0;
            goal_way_point.orientation.y = 0;
            goal_way_point.orientation.z = 0;
            goal_way_point.orientation.w = 1;

            geometry_msgs::Pose goal_way_point_car_frame;
            tf2::doTransform(goal_way_point, goal_way_point_car_frame, tf_map_to_laser_);

            const double steering_angle = 2*(goal_way_point_car_frame.position.y)/pow(distance, 2);

            publish_corrected_speed_and_steering(steering_angle);

            visualize_trackpoints(goal_way_point.position.x, goal_way_point.position.y,
                                  trackpoint[0], trackpoint[1]);

            break;
        }

        tree.emplace_back(new_node);
    }
}

/// This method returns a sampled point from the free space
/// (restrict so that it only samples a small region of interest around the car's current position)
/// @return - the sampled point in free space
std::array<double, 2> RRT::sample()
{
    std::uniform_real_distribution<>::param_type x_param(0.3, 2.0);
    std::uniform_real_distribution<>::param_type y_param(-1.5, 1.5);
    x_dist.param(x_param);
    y_dist.param(y_param);

    geometry_msgs::Pose sample_point;
    sample_point.position.x = x_dist(gen);
    sample_point.position.y = y_dist(gen);
    sample_point.position.z = 0;
    sample_point.orientation.x = 0;
    sample_point.orientation.y = 0;
    sample_point.orientation.z = 0;
    sample_point.orientation.w = 1;
    tf2::doTransform(sample_point, sample_point, tf_laser_to_map_);\

    return {sample_point.position.x, sample_point.position.y};
}

/// This method returns the nearest node id on the tree to the sampled point
/// @param tree - the current RRT tree
/// @param sampled_point - the sampled point in free space
/// @return - id of nearest node on the tree
int RRT::nearest(const std::vector<Node> &tree, const std::array<double, 2> &sampled_point)
{
    int nearest_node = 0;
    double nearest_node_distance = std::numeric_limits<double>::max();
    for(size_t i=0; i< tree.size(); i++)
    {
        const auto distance_sqr = pow(tree[i].x - sampled_point[0], 2) + pow(tree[i].y - sampled_point[1], 2);
        if(distance_sqr < nearest_node_distance)
        {
            nearest_node = i;
            nearest_node_distance = distance_sqr;
        }
    }
    return nearest_node;
}

/// The function steer:(x,y)->z returns a point such that z is “closer”
/// to y than x is. The point z returned by the function steer will be
/// such that z minimizes ||z−y|| while at the same time maintaining
/// ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0
/// basically, expand the tree towards the sample point (within a max dist)
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param sampled_point - the sampled point in free space
/// @return - new node created from steering
Node RRT::steer(const Node &nearest_node, const int nearest_node_index, const std::array<double, 2> &sampled_point)
{
    const double x = sampled_point[0] - nearest_node.x;
    const double y = sampled_point[1] - nearest_node.y;
    const double distance = pow(x, 2) + pow(y, 2);

    Node new_node{};
    if(distance < max_expansion_distance_)
    {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    }
    else
    {
        const double theta = atan2(y, x);
        new_node.x = nearest_node.x + cos(theta)*max_expansion_distance_;
        new_node.y = nearest_node.y + sin(theta)*max_expansion_distance_;
    }
    new_node.parent_index = nearest_node_index;

    return new_node;
}

/// This method returns a boolean indicating if the path between the
/// nearest node and the new node created from steering is collision free
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param new_node - new node created from steering
/// @return - true if in collision, false otherwise
bool RRT::is_collided(const Node &nearest_node, const Node &new_node)
{
    double x_increment = (new_node.x - nearest_node.x)/collision_checking_points_;
    double y_increment = (new_node.y - nearest_node.y)/collision_checking_points_;

    double current_x = nearest_node.x;
    double current_y = nearest_node.y;

    for(int i=0; i<collision_checking_points_; i++)
    {
        current_x += x_increment;
        current_y += y_increment;
        if(is_collided(current_x, current_y))
        {
            return true;
        }
    }

    return false;
}

/// This method checks if the latest node added to the tree is close
/// enough (defined by goal_threshold) to the goal so we can terminate
/// the search and find a path
/// @param latest_added_node - latest addition to the tree
/// @param goal_x - x coordinate of the current goal in map frame
/// @param goal_y - y coordinate of the current goal in map frame
/// @return - true if node close enough to the goal
bool RRT::is_goal(const Node &latest_added_node, double goal_x, double goal_y)
{
    const double distance = sqrt(pow(latest_added_node.x - goal_x,2)+pow(latest_added_node.y - goal_y,2));
    return distance < goal_tolerance_;
}

/// This method traverses the tree from the node that has been determined
/// as goal
/// @param tree - the RRT tree
/// @param latest_added_node - latest addition to the tree that has been
/// determined to be close enough to the goal
/// @return - the vector that represents the order of the nodes traversed as the found path
std::vector<std::array<double, 2>> RRT::find_path(const std::vector<Node> &tree, const Node &latest_added_node)
{
    std::vector<std::array<double, 2>> found_path;
    Node current_node = latest_added_node;

    while(current_node.parent_index != -1)
    {
        std::array<double, 2> local_trackpoint{current_node.x, current_node.y};
        found_path.emplace_back(local_trackpoint);
        current_node = tree[current_node.parent_index];
    }

    return found_path;
}

/// Checks if a sample in the workspace is colliding with an obstacle
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return true if the point is colliding with an obstacle
bool RRT::is_collided(const double x_map, const double y_map)
{
    const auto index = get_row_major_index(x_map, y_map);
    return input_map_.data[index] == 100;
}

/// Returns the row major index for the map
/// @param x_map - x coordinates in map frame
/// @param y_map - y coordinates in map frame
/// @return row major index of the map
int RRT::get_row_major_index(const double x_map, const double y_map)
{
    const auto x_index = static_cast<int>((x_map - input_map_.info.origin.position.x)/input_map_.info.resolution);
    const auto y_index = static_cast<int>((y_map - input_map_.info.origin.position.y)/input_map_.info.resolution);
    return y_index*map_cols_ + x_index;
}

/// Returns the best way point from the global plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame
std::array<double, 2> RRT::get_best_global_trackpoint(const std::array<double, 2>& current_pose)
{
    try
    {
        tf_map_to_laser_ = tf_buffer_.lookupTransform("laser", "map", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    int best_trackpoint_index = -1;
    double best_trackpoint_distance = std::numeric_limits<double>::max();

    for(int i=0; i<global_path_.size(); ++i)
    {
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = global_path_[i][0];
        goal_way_point.position.y = global_path_[i][1];
        goal_way_point.position.z = 0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);

        if(goal_way_point.position.x < 0) continue;

        double distance = std::abs(lookahead_distance_ -
                sqrt(pow(goal_way_point.position.x, 2)+ pow(goal_way_point.position.y, 2)));

        if(distance < best_trackpoint_distance)
        {
            const auto row_major_index = get_row_major_index(global_path_[i][0], global_path_[i][1]);
            if (input_map_.data[row_major_index] == 100) continue;
            best_trackpoint_distance = distance;
            best_trackpoint_index = i;
        }
    }
    return global_path_[best_trackpoint_index];
}

/// Returns the best way point from the local plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame
std::pair<std::array<double, 2>, double> RRT::get_best_local_trackpoint(const std::array<double, 2>& current_pose)
{
    std::array<double, 2> closest_point{};
    double closest_distance_to_current_pose = std::numeric_limits<double>::max();
    double closest_distance = std::numeric_limits<double>::max();
    for(const auto& trackpoint : local_path_)
    {
        double distance = sqrt(pow(trackpoint[0] - current_pose[0], 2)
                               + pow(trackpoint[1] - current_pose[1], 2));

        double diff_distance = std::abs(local_lookahead_distance_ - distance);
        if(diff_distance < closest_distance)
        {
            closest_distance_to_current_pose = distance;
            closest_distance = diff_distance;
            closest_point = trackpoint;
        }
        if(diff_distance < local_trackpoint_tolerance_)
        {
            return {trackpoint, distance};
        }
    }
    ROS_WARN("No Local TrackPoint found within the local_trackpoint_tolerance. Approximated the Closest Point Available");
    return {closest_point, closest_distance_to_current_pose};
}

/// Publish way point with respect to the given input properties
/// @param way_point - (x, y) wrt to the frame id
/// @param frame_id - frame represented in waypoint
/// @param r - red intensity
/// @param g - green intensity
/// @param b - blue intenisty
/// @param transparency (Do not put 0)
/// @param scale_x
/// @param scale_y
/// @param scale_z
void RRT::add_way_point_visualization(const std::array<double, 2>& way_point, const std::string& frame_id, double r,
        double g, double b, double transparency = 0.5, double scale_x=0.2, double scale_y=0.2, double scale_z=0.2)
{
    visualization_msgs::Marker way_point_marker;
    way_point_marker.header.frame_id = frame_id;
    way_point_marker.header.stamp = ros::Time();
    way_point_marker.ns = "pure_pursuit";
    way_point_marker.id = unique_id_;
    way_point_marker.type = visualization_msgs::Marker::SPHERE;
    way_point_marker.action = visualization_msgs::Marker::ADD;
    way_point_marker.pose.position.x = way_point[0];
    way_point_marker.pose.position.y = way_point[1];
    way_point_marker.pose.position.z = 0;
    way_point_marker.pose.orientation.x = 0.0;
    way_point_marker.pose.orientation.y = 0.0;
    way_point_marker.pose.orientation.z = 0.0;
    way_point_marker.pose.orientation.w = 1.0;
    way_point_marker.scale.x = 0.2;
    way_point_marker.scale.y = 0.2;
    way_point_marker.scale.z = 0.2;
    way_point_marker.color.a = transparency;
    way_point_marker.color.r = r;
    way_point_marker.color.g = g;
    way_point_marker.color.b = b;
    waypoint_viz_pub_.publish(way_point_marker);
    unique_id_++;
}

/// visualize all way points in the global path
void RRT::visualize_waypoint_data()
{
    const size_t increment = global_path_.size()/100;
    for(size_t i=0; i<global_path_.size(); i=i+increment)
    {
        add_way_point_visualization(global_path_[i], "map", 1.0, 0.0, 1.0, 0.5);
    }
    ROS_INFO("Published All Global WayPoints.");
}

/// visualize trackpoints handles the creation and deletion of trackpoints
void RRT::visualize_trackpoints(double x_local, double y_local, double x_global, double y_global)
{
    visualization_msgs::Marker line;

    line.header.frame_id    = "/map";
    line.header.stamp       = ros::Time::now();
    line.lifetime           = ros::Duration(0.1);
    line.id                 = 1;
    line.ns                 = "rrt";
    line.type               = visualization_msgs::Marker::LINE_STRIP;
    line.action             = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x            = 0.05;
    line.color.r            = 0.0f;
    line.color.g            = 0.0f;
    line.color.b            = 1.0f;
    line.color.a            = 1.0f;

    geometry_msgs::Point current;
    geometry_msgs::Point local;
    geometry_msgs::Point global;

    current.x = current_x_;
    current.y = current_y_;
    current.z =  0;

    local.x = x_local;
    local.y = y_local;
    local.z = 0;

    global.x = x_global;
    global.y = y_global;
    global.z = 0;

    line.points.push_back(current);
    line.points.push_back(local);
    line.points.push_back(global);

    tree_viz_pub_.publish(line);
    unique_id_++;
}

/// Returns the appropriate speed based on the steering angle
/// @param steering_angle
/// @return
void RRT::publish_corrected_speed_and_steering(double steering_angle)
{
    ackermann_msgs::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = ros::Time::now();
    drive_msg.header.frame_id = "base_link";

    drive_msg.drive.steering_angle = steering_angle;
    if(steering_angle > 0.1)
    {
        if (steering_angle > 0.2)
        {
            drive_msg.drive.speed = low_speed_;
            if (steering_angle > 0.4)
            {
                drive_msg.drive.steering_angle = 0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else if(steering_angle < -0.1)
    {
        if (steering_angle < -0.2)
        {
            drive_msg.drive.speed = low_speed_;
            if (steering_angle < -0.4)
            {
                drive_msg.drive.steering_angle = -0.4;
            }
        }
        else
        {
            drive_msg.drive.speed = medium_speed_;
        }
    }
    else
    {
        drive_msg.drive.speed = high_speed_;
    }

    drive_pub_.publish(drive_msg);
}