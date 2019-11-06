// This file contains the class definition of tree nodes and RRT

#include "rrt/rrt.h"


RRT::RRT(ros::NodeHandle &nh): nh_(nh), gen((std::random_device())()) {

    // TODO: Load parameters from yaml file, you could add your own parameters to the rrt_params.yaml file
    std::string pose_topic, scan_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("scan_topic", scan_topic);

    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    pf_sub_ = nh_.subscribe(pose_topic, 10, &RRT::pf_callback, this);
    scan_sub_ = nh_.subscribe(scan_topic, 10, &RRT::scan_callback, this);

    // TODO: create a occupancy grid

    ROS_INFO("Created new RRT Object.");
}

/// The scan callback, update your occupancy grid here
/// @param scan_msg - pointer to the incoming scan message
void RRT::scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg)
{
    // TODO: update your occupancy grid
}

/// The pose callback when subscribed to particle filter's inferred pose (RRT Main Loop)
/// @param pose_msg - pointer to the incoming pose message
void RRT::pf_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {

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