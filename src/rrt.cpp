// This file contains the class definition of tree nodes and RRT
#include "f110_rrt/rrt.h"
#include "f110_rrt/csv_reader.h"

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
    f110::CSVReader reader("$(find f110_rrt)/sensor_data/gtpose.csv");
    global_path_ = reader.getData();

    // ROS Publishers and Subscribers
    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    pose_sub_ = nh_.subscribe(pose_topic, 1, &RRT::pose_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    dynamic_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);

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
    nh_.getParam("global_trackpoint_tolerance", global_trackpoint_tolerance_);
    nh_.getParam("local_trackpoint_tolerance", local_trackpoint_tolerance_);
    nh_.getParam("lookahead_distance", lookahead_distance_);
    nh_.getParam("local_lookahead_distance", local_lookahead_distance_);

    ROS_INFO("Created new RRT Object.");
}

/// The scan callback updates the occupancy grid
/// @param scan_msg - pointer to the incoming scan message
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    try
    {
        listener_.lookupTransform("/map", "/laser", ros::Time(0), tf_base_link_to_map_);
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    const auto translation = tf_base_link_to_map_.getOrigin();
    const double yaw = tf::getYaw(tf_base_link_to_map_.getRotation());

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

        const double x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + translation.getX();
        const double y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + translation.getY();

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

    dynamic_map_pub_.publish(input_map_);
    ROS_DEBUG("Map Updated");
}

/// The pose callback when subscribed to particle filter's inferred pose (RRT Main Loop)
/// @param pose_msg - pointer to the incoming pose message
void RRT::pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg)
{
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
        if(is_collided(sample_node[0], sample_node[1])) continue;

        // Find the nearest node in the tree to the sample node
        const int nearest_node_id = nearest(tree, sample_node);

        // Steer the tree from the nearest node towards the sample node
        const Node new_node = steer(tree[nearest_node_id], nearest_node_id, sample_node);

        // Check if the segment joining the two nodes is in collision
        if(is_collided(tree[nearest_node_id], new_node)) continue;

        if(is_goal(new_node, trackpoint[0], trackpoint[1]))
        {
            ROS_INFO("Goal reached. Backtracking ...");
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
            tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);

            const double steering_angle = 2*(goal_way_point.position.y)/pow(distance, 2);

            // Publish drive message
            ackermann_msgs::AckermannDriveStamped drive_msg;
            drive_msg.header.stamp = ros::Time::now();
            drive_msg.header.frame_id = "base_link";

            // Thresholding for limiting the movement of car wheels to avoid servo locking and variable speed
            // adjustment
            drive_msg.drive.steering_angle = steering_angle;
            drive_msg.drive.speed = 1.0;

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
    std::uniform_real_distribution<>::param_type x_param(0.0, 4.0);
    std::uniform_real_distribution<>::param_type y_param(-0.7, 0.7);
    x_dist.param(x_param);
    y_dist.param(y_param);

    return {x_dist(gen), y_dist(gen)};
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

// Method to be added for RRT*
/// This method returns the cost associated with a node
/// @param tree - the current tree
/// @param node - the node the cost is calculated for
/// @return - the cost value associated with the node
double RRT::cost(std::vector<Node> &tree, Node &node)
{

    double cost = 0;

    return cost;
}

// Method to be added for RRT*
/// This method returns the cost of the straight line path between two nodes
/// @param n1 - the Node at one end of the path
/// @param n2 - the Node at the other end of the path
/// @return - the cost value associated with the path
double RRT::line_cost(Node &n1, Node &n2)
{


    double cost = 0;

    return cost;
}

// Method to be added for RRT*
/// This method returns the set of Nodes in the neighborhood of a node.
/// @param tree - the current tree
/// @param node - the node to find the neighborhood for
/// @return - the index of the nodes in the neighborhood
std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{

    std::vector<int> neighborhood;

    return neighborhood;
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
        tf_map_to_laser_ = tf_buffer_.lookupTransform("/laser", "map", ros::Time(0));
    }
    catch (tf::TransformException& ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(0.1).sleep();
    }

    for(const auto& trackpoint : global_path_)
    {
        geometry_msgs::Pose goal_way_point;
        goal_way_point.position.x = trackpoint[0];
        goal_way_point.position.y = trackpoint[1];
        goal_way_point.position.z = 0;
        goal_way_point.orientation.x = 0;
        goal_way_point.orientation.y = 0;
        goal_way_point.orientation.z = 0;
        goal_way_point.orientation.w = 1;
        tf2::doTransform(goal_way_point, goal_way_point, tf_map_to_laser_);

        if(goal_way_point.position.x < 0) continue;

        double distance = sqrt(pow(goal_way_point.position.x, 2)+ pow(goal_way_point.position.y, 2));

        if(std::abs(lookahead_distance_ - distance) < global_trackpoint_tolerance_)
        {
            const auto row_major_index = get_row_major_index(trackpoint[0], trackpoint[1]);
            if (input_map_.data[row_major_index] == 100) continue;
            return trackpoint;
        }
    }
    ROS_ERROR("No Global TrackPoint found within the trackpoint_tolerance");
    return current_pose;
}

/// Returns the best way point from the local plan to track
/// @param current_pose - current pose (x, y) of the car in map frame
/// @param lookahead_distance - ideal distance to find a trackpoint ahead of the current pose
/// @return trackpoint (x, y) in map frame
std::pair<std::array<double, 2>, double> RRT::get_best_local_trackpoint(const std::array<double, 2>& current_pose)
{
    for(const auto& trackpoint : local_path_)
    {
        double distance = sqrt(pow(trackpoint[0] - current_pose[0], 2)
                               + pow(trackpoint[1] - current_pose[1], 2));

        if(std::abs(local_lookahead_distance_ - distance) < local_trackpoint_tolerance_)
        {
            return {trackpoint, distance};
        }
    }
    ROS_ERROR("No Local TrackPoint found within the local_trackpoint_tolerance_");
}