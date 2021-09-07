#include "ros/ros.h"
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include "helper_functions.h"
#include "rrt_star.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <nav_msgs/GridCells.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseArray.h>


class IPPNode {
private:
    global_params parameters;
    rrt_star* RRT;
    Point current_goal;
    // Publisher
    ros::Publisher pub_path;
    ros::Publisher pub_gmrf;
    ros::Publisher marker_pub;
    ros::Publisher gmrf_plot_pub;
    ros::Publisher yaw_angle;
    ros::Publisher GMRF_pred_var;
    ros::Publisher pred_var_sum;
    ros::Publisher path_x;
    ros::Publisher path_y;

    // FOR RQT
    ros::Publisher pub_robot_pose;
    
    // Subscriber
    ros::Subscriber sub_pose;
    ros::Subscriber sub_meas;

    visualization_msgs::Marker points, line;
    visualization_msgs::Marker goal_path_plot;
    visualization_msgs::Marker pose_arrow;
    visualization_msgs::Marker goal_point;

    // GMRF
    visualization_msgs::MarkerArray map_marker;
    visualization_msgs::Marker map_cell;

    int subscriber_counter = 0;    
    vector<MyPose> traversed_path;
    vector<double> GMRF_cov;
    vector<float> pred_vars;

    MyPose true_pose;

public: 
    
    // Constructor
    IPPNode(global_params parameters_, MyPose init_pose, ros::NodeHandle node){
        parameters = parameters_;
        cout << "Calling Constructor" << endl;
        list<obstacle*> empty_obstacle_list;
        obstacle* obst_line = new line_obstacle(parameters, (Point){2, 5}, (Point){8, 5});
        empty_obstacle_list.push_back(obst_line);
        obstacle* obst_line1 = new line_obstacle(parameters, (Point){5, 5}, (Point){5, 10});
        empty_obstacle_list.push_back(obst_line1);
        RRT = new rrt_star(parameters, init_pose, empty_obstacle_list);
        cout << init_pose.at(0) << endl;
        current_goal = {2, 2};
        RRT->update_goal(current_goal);

        // build tree
        while (RRT->all_nodes.size() <= 6000) {
            // add new nodes to tree
            RRT->expand();
        }
        RRT->update_goal(current_goal);
        RRT->generate_path();
       
        cout << "Finished Building Tree!" << endl;
        
        // publishers
        pub_path = node.advertise<geometry_msgs::Pose2D>("/IPP/goal_path", 1000);
        pub_gmrf = node.advertise<std_msgs::Float64>("/IPP/gmrf_belief", 1);
        marker_pub = node.advertise<visualization_msgs::Marker>("IPP/visualization_marker", 10);
        gmrf_plot_pub = node.advertise<visualization_msgs::MarkerArray>("IPP/gmrf_marker", 1);
        yaw_angle = node.advertise<std_msgs::Float64>("yaw_angle", 1);
        pub_robot_pose = node.advertise<geometry_msgs::Pose2D>("/IPP/robot_pose", 1000);
        path_x = node.advertise<std_msgs::Float32MultiArray>("IPP/path_x", 1);
        path_y = node.advertise<std_msgs::Float32MultiArray>("IPP/path_y", 1);
        GMRF_pred_var =  node.advertise<std_msgs::Float32MultiArray>("IPP/GMRF_cov", 1);
        pred_var_sum = node.advertise<std_msgs::Float32MultiArray>("IPP/pred_var", 1);
        
        // Test
        if (parameters.visualize_rviz){
            visualize_tree();
        }
        // subscribers
        sub_pose = node.subscribe("ground_truth/state", 1, &IPPNode::pose_callback, this);
        sub_meas = node.subscribe("sensor_measurement", 1, &IPPNode::meas_callback, this);

        // for visualization
        //visualizations  points and lines..
        points.header.frame_id = "map";
        line.header.frame_id = "map";
        goal_path_plot.header.frame_id = "map";
        pose_arrow.header.frame_id = "map";
        goal_point.header.frame_id = "map";
        points.header.stamp = ros::Time(0);
        line.header.stamp = ros::Time(0);
        goal_path_plot.header.stamp = ros::Time(0);
        pose_arrow.header.stamp = ros::Time(0);
        goal_point.header.stamp = ros::Time(0);
             
        points.ns = line.ns = goal_point.ns = goal_path_plot.ns = pose_arrow.ns = "markers";
        points.id = 0;
        line.id = 1;
        goal_path_plot.id = 2;
        pose_arrow.id = 3;
        goal_point.id = 4;


        points.type = points.POINTS;
        line.type = line.LINE_LIST;
        goal_path_plot.type = goal_path_plot.LINE_LIST;
        pose_arrow.type = pose_arrow.ARROW;
        goal_point.type = goal_point.SPHERE;

        //Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        points.action = points.ADD;
        line.action = line.ADD;
        goal_path_plot.action = line.ADD;
        pose_arrow.action = pose_arrow.ADD;
        goal_point.action = goal_point.ADD;
        points.pose.orientation.w = 1.0;
        line.pose.orientation.w = 1.0;
        goal_path_plot.pose.orientation.w = 1.0;
        line.scale.x =  0.03;
        line.scale.y = 0.03;
        goal_path_plot.scale.x =  0.1;
        goal_path_plot.scale.y =  0.1;
        pose_arrow.scale.x = 0.6;
        pose_arrow.scale.y = .1;
        points.scale.x = 0.03; 
        points.scale.y = 0.03; 
        goal_point.scale.x = 0.2;
        goal_point.scale.y = 0.2;
        goal_point.scale.z = 0.1;

        line.color.r = 9.0/255.0;
        line.color.g = 91.0/255.0;
        line.color.b = 236.0/255.0;
        
        goal_path_plot.color.r = 200/255.0;
        goal_path_plot.color.g = 0/255.0;
        goal_path_plot.color.b = 50/255.0;
        
        points.color.r = 255.0/255.0;
        points.color.g = 0.0/255.0;
        points.color.b = 0.0/255.0;

        pose_arrow.color.r = 0/255.0;
        pose_arrow.color.g = 255.0/255.0;
        pose_arrow.color.b = 0.0/255.0;

        goal_point.color.r = 1;
        goal_point.color.g = 0;
        goal_point.color.b = 0;

        points.color.a = 1.0;
        line.color.a = 0.5;
        goal_path_plot.color.a = 1.0;
        pose_arrow.color.a = 1.0;
        goal_point.color.a = 1;

        points.lifetime = ros::Duration(1);
        line.lifetime = ros::Duration(1);
        goal_path_plot.lifetime = ros::Duration(1);
        pose_arrow.lifetime = ros::Duration(1);
        goal_point.lifetime = ros::Duration(1);

        pose_arrow.pose.position.z = 0.0;

        map_cell.header.stamp = ros::Time::now();
        map_cell.header.frame_id = "map";
        map_cell.type = visualization_msgs::Marker::CUBE;
        map_cell.lifetime = ros::Duration(1);

        map_cell.action =  visualization_msgs::Marker::ADD;
        map_cell.pose.orientation.w = 1.0;
        map_cell.mesh_use_embedded_materials = false;

        map_cell.scale.x = 0.9f;
        map_cell.scale.y = 0.9f;
        map_cell.scale.z = 0.1f;
    };

    // Destructor
    ~IPPNode(){
        cout << "Node Deleted!" << endl;
    };

    void maintain_tree(){
        if (RRT->all_nodes.size() <= 5000) {
            // add new nodes to tree
            RRT->expand();
        } else {
            // perform only rewiring
            // cout << ros::Time::now();
            RRT->expand(false);
            // cout << ros::Time::now();
        }
    }

    // Path publisher
    void publish_path_msg(){
        geometry_msgs::Pose2D msg;
        if (RRT->path.size() > 2){
            msg.x = RRT->path.at(1).at(0);
            msg.y = RRT->path.at(1).at(1);
            msg.theta = RRT->path.at(1).at(2);
            pub_path.publish(msg);
        }
    };

    // GMRF publisher
    void publish_gmrf_msg();

    // pose callback
    void pose_callback(const nav_msgs::Odometry &msg){
        geometry_msgs::Quaternion q = msg.pose.pose.orientation;
        tf::Quaternion q_;
        q_.setW(q.w);
        q_.setX(q.x);
        q_.setY(q.y);
        q_.setZ(q.z);
        double roll, pitch, yaw;
        tf::Matrix3x3 m(q_);
        m.getRPY(roll, pitch, yaw);
        MyPose new_pose = {(float)msg.pose.pose.position.x, (float)msg.pose.pose.position.y, (float)yaw};
        // MyPose new_pose = {(float)msg.pose.pose.position.x, (float)msg.pose.pose.position.y, (float)RRT->path.at(1).at(2)};
        if (subscriber_counter == 0){
            RRT->update_pose(new_pose);
            subscriber_counter = 0;
            traversed_path.push_back(new_pose);
        }
        else{
            subscriber_counter++;
        }
        MyPose root_pose = RRT->return_root()->return_pose();
        true_pose = new_pose;
        // update goal
        if (euclidean_dist(RRT->current_pose, (MyPose){current_goal.at(0), current_goal.at(1), 0}) < 0.8 ||
            RRT->closest_to_goal->return_path_info_utility() < RRT->return_max_utility_node()->return_path_info_utility() / 10) {
            auto goal_node = RRT->return_max_utility_node();
            current_goal = {goal_node->return_pose().at(0), goal_node->return_pose().at(1)};
            RRT->update_goal(current_goal);
            // rewire to get improved path
            RRT->rewire_f_r();
            RRT->rewire();
        }

        // if (euclidean_dist(RRT->current_pose, RRT->path.at(2)) < 0.4){
        RRT->generate_path();
        // }

        // publish_yaw_angle();
        // Rviz
        pose_arrow.pose.position.x = new_pose.at(0);
        pose_arrow.pose.position.y = new_pose.at(1);
        pose_arrow.pose.orientation = msg.pose.pose.orientation;
        marker_pub.publish(pose_arrow);
        RRT->condition_GMRF((Point){RRT->current_pose.at(0), RRT->current_pose.at(1)}, 10.0);
        pred_vars.push_back(RRT->gmrf_belief->return_predictive_variance());
        if (parameters.visualize_rviz){
            visualize_tree();
            visualize_goal_path();
            visualize_gmrf();
        }

        geometry_msgs::Pose2D pose_msg;
        pose_msg.x = RRT->current_pose.at(0);
        pose_msg.y = RRT->current_pose.at(1);
        pose_msg.theta = RRT->current_pose.at(2);
        pub_robot_pose.publish(pose_msg);
    };

    // measurement callback
    void meas_callback(const std_msgs::Float64 &msg){
        float measurement = msg.data;
        RRT->condition_GMRF((Point){RRT->current_pose.at(0), RRT->current_pose.at(1)}, measurement);
    };

    void visualize_tree(){
        line.points.clear();
        points.points.clear();

        // plot recursion
        plot_recursion(RRT->return_root());
        marker_pub.publish(line);
        marker_pub.publish(points);
    }

    void plot_recursion(node* current_node){
        geometry_msgs::Point p;
        p.x = current_node->return_pose().at(0);
        p.y = current_node->return_pose().at(1);
        p.z = 0;
    
        points.points.push_back(p);
        if (current_node->has_parent){
            line.points.push_back(p);
            p.x = current_node->return_parent()->return_pose().at(0);
            p.y = current_node->return_parent()->return_pose().at(1);
            p.z = 0;
            line.points.push_back(p);
        }
        for (node* child:current_node->return_children()){
            plot_recursion(child);
        }
    }

    void visualize_goal_path(){
        goal_path_plot.points.clear();
        geometry_msgs::Point p;
        for (int i = 0; i < RRT->path.size() - 1; i++){
            p.x = RRT->path.at(i).at(0);
            p.y = RRT->path.at(i).at(1);
            p.z = 0;
            goal_path_plot.points.push_back(p);
            p.x = RRT->path.at(i+1).at(0);
            p.y = RRT->path.at(i+1).at(1);
            p.z = 0;
            goal_path_plot.points.push_back(p);
        }
        marker_pub.publish(goal_path_plot);
    }

    void visualize_gmrf(){
        map_marker.markers.clear();
        int32_t id = 0;
        int x_res = 50;
        int y_res = 50;

        float scale_x = (parameters.x_range.at(1) - parameters.x_range.at(0))/x_res;
        float scale_y = (parameters.y_range.at(1) - parameters.y_range.at(0))/y_res;

        std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
        for (int i = 1; i <= x_res -1; i++) {
            for (int j = 1; j <= y_res - 1; j++) {
                map_cell.id = id;
                map_cell.pose.position.x =  (float)i * scale_x;
                map_cell.pose.position.y = (float)j * scale_y;
                map_cell.pose.position.z = -0.2;

                auto cov = RRT->gmrf_belief->return_cov_at_location((Point){(float)map_cell.pose.position.x, (float)map_cell.pose.position.y});
                color.r = 100 * cov/255.0f;
                color.g = 10/255.0f;
                color.b = 100/255.0f;
                color.a = 1;
                map_cell.color =color;
                map_marker.markers.push_back(map_cell);
                id++;
            }
        }
        gmrf_plot_pub.publish(map_marker);
    }

    void publish_yaw_angle(){
        float yaw = atan2(RRT->path.at(2).at(1) - true_pose.at(1), RRT->path.at(2).at(0) - true_pose.at(0));
        std_msgs::Float64 yaw_msg;
        yaw_msg.data = yaw;
        yaw_angle.publish(yaw_msg);

        // DEBUG
        // cout << "YAW CMD : " << yaw << endl;
        goal_point.pose.position.x = RRT->path.at(2).at(0);
        goal_point.pose.position.y = RRT->path.at(2).at(1);
        goal_point.pose.position.z = 0;
        marker_pub.publish(goal_point);
        // cout << "Angle Error : " << (yaw - RRT->current_pose.at(2)) << endl;
    }


    void publish_path(){
        std_msgs::Float32MultiArray msg_x;
        std_msgs::Float32MultiArray msg_y;
        for (MyPose pose:traversed_path){
            msg_x.data.push_back(pose.at(0));
            msg_y.data.push_back(pose.at(1));
        }
        path_x.publish(msg_x);
        path_y.publish(msg_y);
    }

    void publish_GMRF_cov(){
        GMRF_cov = RRT->gmrf_belief->return_cov_python_format();
        std_msgs::Float32MultiArray msg;
        for (auto c:GMRF_cov){
            msg.data.push_back(c);
        }
        GMRF_pred_var.publish(msg);
    }

    void publish_pred_var_sum(){
        std_msgs::Float32MultiArray msg;
        for (auto p:pred_vars){
            msg.data.push_back(p);
        }
        pred_var_sum.publish(msg);
    }
};


int main(int argc, char **argv)
{
    cout << "NODE IPP" << endl;
    // PARAMETER BEGIN
    global_params global_p;
    // field range
    global_p.x_range = {0, 10};
    global_p.y_range = {0, 10};

    // motion model parameters
    global_p.vel = 0.1;
    global_p.dt = 1;
    global_p.nominal_step = 1;

    // weights
    global_p.c_mean = 0;
    global_p.weight_distance = 1; // 0.2
    global_p.weight_steering = .1; // 0.04
    global_p.weight_information = 1; // 0.08
    global_p.weight_cov = 1;
    global_p.weight_mean = 0;
    global_p.discount = 0.99;

    // RRT Grid
    global_p.cell_size = .3;
    global_p.n_x = ceil((global_p.x_range.at(1) - global_p.x_range.at(0)) / global_p.cell_size);
    global_p.n_y = ceil((global_p.x_range.at(1) - global_p.y_range.at(0)) / global_p.cell_size);
    global_p.rewire_time = 80;

    // GMRF
    global_p.gmrf_nx = 25;
    global_p.gmrf_ny = 25;
    global_p.meas_noise = 0.01;

    // sensor
    global_p.sensor_angle = 40 * M_PI / 180;
    global_p.sensor_range = 2.5;
    global_p.mapping = false;

    global_p.visualize_rviz = true;
    // PARAMETERS END

    // init pose
    MyPose init_pose = {0, 0, 0};

    ros::init(argc, argv, "IPP_RRT_Node");
    ros::NodeHandle n;


    IPPNode ipp = IPPNode(global_p, init_pose, n);
    //ros::ServiceServer service = n.advertiseService("visualize", &IPPNode::visualize_tree, &ipp);

    ros::Rate rate(30);

    int k = 0;
    while (ros::ok()){
        ipp.maintain_tree();
        ipp.publish_yaw_angle();
        ros::spinOnce();
        rate.sleep();
        cout << "Iteration : " << k << endl;
        if (k % 10 == 0){
            ipp.publish_GMRF_cov();
            ipp.publish_pred_var_sum();
            ipp.publish_path();
        }
        k++;
    }

    return 0;
}
