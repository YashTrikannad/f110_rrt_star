// This file contains the class definition of tree nodes and RRT
#include "f110_rrt/rrt.h"

/// RRT Object Constructor
/// @param nh - node handle to the ros node
RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())())
{
    std::string pose_topic, scan_topic, drive_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);
    nh_.getParam("drive_topic", drive_topic);

    input_map_  = *(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map",ros::Duration(2)));

    if(input_map_.data.empty())
    {
        std::__throw_invalid_argument("Input Map Load Unsuccessful\"");
    }
    ROS_INFO("Map Load Successful.");

    drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 1);
    pose_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pose_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);
    dynamic_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("dynamic_map", 1);

    map_cols_ = input_map_.info.width;
    new_obstacles_ = {};
    new_obstacles_.reserve(2000);
    clear_obstacles_count_ = 0;

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

        const double x_map_top_left = x_map - input_map_.info.origin.position.x;
        const double y_map_top_left = y_map - input_map_.info.origin.position.y;

        const auto x_index = static_cast<int>(x_map_top_left/input_map_.info.resolution);
        const auto y_index = static_cast<int>(y_map_top_left/input_map_.info.resolution);

        const size_t index = y_index*map_cols_ + x_index;
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
    // tree as std::vector
    std::vector<Node> tree;

    // TODO: fill in the RRT main loop



    // path found as Path message

}

/// This method returns a sampled point from the free space
/// (restrict so that it only samples a small region of interest around the car's current position)
/// @return - the sampled point in free space
std::vector<double> RRT::sample() {

    std::vector<double> sampled_point;
    // TODO: fill in this method

    
    return sampled_point;
}

/// This method returns the nearest node on the tree to the sampled point
/// @param tree - the current RRT tree
/// @param sampled_point - the sampled point in free space
/// @return - index of nearest node on the tree
int RRT::nearest(std::vector<Node> &tree, std::vector<double> &sampled_point)
{

    int nearest_node = 0;
    // TODO: fill in this method

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
Node RRT::steer(Node &nearest_node, std::vector<double> &sampled_point)
{

    Node new_node;
    // TODO: fill in this method

    return new_node;
}

/// This method returns a boolean indicating if the path between the
/// nearest node and the new node created from steering is collision free
/// @param nearest_node - nearest node on the tree to the sampled point
/// @param new_node - new node created from steering
/// @return - true if in collision, false otherwise
bool RRT::check_collision(Node &nearest_node, Node &new_node) {


    bool collision = false;
    // TODO: fill in this method

    return collision;
}

/// This method checks if the latest node added to the tree is close
/// enough (defined by goal_threshold) to the goal so we can terminate
/// the search and find a path
/// @param latest_added_node - latest addition to the tree
/// @param goal_x - x coordinate of the current goal
/// @param goal_y - y coordinate of the current goal
/// @return - true if node close enough to the goal
bool RRT::is_goal(Node &latest_added_node, double goal_x, double goal_y) {


    bool close_enough = false;
    // TODO: fill in this method

    return close_enough;
}

/// This method traverses the tree from the node that has been determined
/// as goal
/// @param tree - the RRT tree
/// @param latest_added_node - latest addition to the tree that has been
/// determined to be close enough to the goal
/// @return - the vector that represents the order of the nodes traversed as the found path
std::vector<Node> RRT::find_path(std::vector<Node> &tree, Node &latest_added_node)
{
    
    std::vector<Node> found_path;
    // TODO: fill in this method

    return found_path;
}

/// This method returns the cost associated with a node
/// @param tree - the current tree
/// @param node - the node the cost is calculated for
/// @return - the cost value associated with the node
double RRT::cost(std::vector<Node> &tree, Node &node) {

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

/// This method returns the cost of the straight line path between two nodes
/// @param n1 - the Node at one end of the path
/// @param n2 - the Node at the other end of the path
/// @return - the cost value associated with the path
double RRT::line_cost(Node &n1, Node &n2) {


    double cost = 0;
    // TODO: fill in this method

    return cost;
}

/// This method returns the set of Nodes in the neighborhood of a node.
/// @param tree - the current tree
/// @param node - the node to find the neighborhood for
/// @return - the index of the nodes in the neighborhood
std::vector<int> RRT::near(std::vector<Node> &tree, Node &node)
{

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}