// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

// ros
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

// standard
#include <cmath>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>
#include <random>

/// Struct defining the Node object in the RRT tree.
struct Node
{
    double x;
    double y;
    double cost;
    int index;
    int parent_index;
    bool is_root = false;
};

struct Point
{
    double x;
    double y;
};

/// RRT Class used for searching across the graph
class RRT {
public:
    RRT(ros::NodeHandle &nh);

private:
    ros::NodeHandle nh_;

    ros::Subscriber map_sub_;
    ros::Subscriber pose_sub_;
    ros::Subscriber scan_sub_;
    ros::Publisher drive_pub_;
    ros::Publisher dynamic_map_pub_;

    tf::TransformListener listener_;
    tf::StampedTransform tf_base_link_to_map_;

    // random generator, use this
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;

    nav_msgs::OccupancyGrid input_map_;
    int map_rows_;
    int map_cols_;

    /// The pose callback when subscribed to particle filter's inferred pose (RRT Main Loop)
    /// @param pose_msg - pointer to the incoming pose message
    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);

    /// The scan callback, update your occupancy grid here
    /// @param scan_msg - pointer to the incoming scan message
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);

    /// RRT methods

    /// This method returns a sampled point from the free space
    /// (restrict so that it only samples a small region of interest around the car's current position)
    /// @return - the sampled point in free space
    std::vector<double> sample();

    /// This method returns the nearest node on the tree to the sampled point
    /// @param tree - the current RRT tree
    /// @param sampled_point - the sampled point in free space
    /// @return - index of nearest node on the tree
    int nearest(std::vector<Node> &tree, std::vector<double> &sampled_point);

    /// The function steer:(x,y)->z returns a point such that z is “closer”
    /// to y than x is. The point z returned by the function steer will be
    /// such that z minimizes ||z−y|| while at the same time maintaining
    /// ||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0
    /// basically, expand the tree towards the sample point (within a max dist)
    /// @param nearest_node - nearest node on the tree to the sampled point
    /// @param sampled_point - the sampled point in free space
    /// @return - new node created from steering
    Node steer(Node &nearest_node, std::vector<double> &sampled_point);

    /// This method returns a boolean indicating if the path between the
    /// nearest node and the new node created from steering is collision free
    /// @param nearest_node - nearest node on the tree to the sampled point
    /// @param new_node - new node created from steering
    /// @return - true if in collision, false otherwise
    bool check_collision(Node &nearest_node, Node &new_node);

    /// This method checks if the latest node added to the tree is close
    /// enough (defined by goal_threshold) to the goal so we can terminate
    /// the search and find a path
    /// @param latest_added_node - latest addition to the tree
    /// @param goal_x - x coordinate of the current goal
    /// @param goal_y - y coordinate of the current goal
    /// @return - true if node close enough to the goal
    bool is_goal(Node &latest_added_node, double goal_x, double goal_y);

    /// This method traverses the tree from the node that has been determined
    /// as goal
    /// @param tree - the RRT tree
    /// @param latest_added_node - latest addition to the tree that has been
    /// determined to be close enough to the goal
    /// @return - the vector that represents the order of the nodes traversed as the found path
    std::vector<Node> find_path(std::vector<Node> &tree, Node &latest_added_node);

    /// RRT* methods

    /// This method returns the cost associated with a node
    /// @param tree - the current tree
    /// @param node - the node the cost is calculated for
    /// @return - the cost value associated with the node
    double cost(std::vector<Node> &tree, Node &node);

    /// This method returns the cost of the straight line path between two nodes
    /// @param n1 - the Node at one end of the path
    /// @param n2 - the Node at the other end of the path
    /// @return - the cost value associated with the path
    double line_cost(Node &n1, Node &n2);

    /// This method returns the set of Nodes in the neighborhood of a node.
    /// @param tree - the current tree
    /// @param node - the node to find the neighborhood for
    /// @return - the index of the nodes in the neighborhood
    std::vector<int> near(std::vector<Node> &tree, Node &node);

};

