#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <thread>
#include <random>
#include <algorithm>
#include <ctime>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "trajectory_generator_waypoint.h"//essentials

using namespace std;
using namespace Eigen;

//Planning params
double _Vel, _Acc;
int _der_order;
double _vis_traj_width;

ros::Publisher _trajectory_pub;
//ros::Subscriber _pos_sub;
geometry_msgs::PointStamped current_position;

//rviz visualization
ros::Publisher _wp_traj_vis_pub;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff; //the coeffs of generated traj.
VectorXd _polyTime;  //time allocation
MatrixXd _wayPoints;

//Specify initial & end states here.
Vector3d _startPos  = Vector3d::Zero(); 
Vector3d _startVel  = Vector3d::Zero();
Vector3d _startAcc  = Vector3d::Zero();
Vector3d _endVel    = Vector3d::Zero();
Vector3d _endAcc    = Vector3d::Zero();

void getWaypoints();
VectorXd timeAllocation(const MatrixXd& Path);
Vector3d getPosPoly(const MatrixXd& polyCoeff, int k, double t);
void trajGeneration(const MatrixXd& path);
void visWayPointTraj(const MatrixXd& polyCoeff, const VectorXd& time);
void updateUavPosition(const geometry_msgs::PointStamped& msg);
double getDistanceToTarget(const Vector3d& target);
bool reachTargetPosition(const Vector3d& target, double max_error);
void goToPos(const Vector3d& pos, double yaw);
void delay(float secs);
void simFlight(const MatrixXd& polyCoeff, const VectorXd &time);



int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    nh_private.param("planning/vel", _Vel, 1.0);
    nh_private.param("planning/acc", _Acc, 1.0);
    nh_private.param("planning/der_order", _der_order, 4);  // the order of derivative, _der_order = 3->minimum jerk, _der_order = 4->minimum snap
    nh_private.param("vis/vis_traj_width", _vis_traj_width, 0.05);
    
    switch (_der_order) {
        case 3: ROS_INFO("Minimum Jerk Trajectory Generation"); break;
        case 4: ROS_INFO("Minimum Snap Trajectory Generation"); break;
        default: break;
    }

    // Unpause Gazebo
    std_srvs::Empty srv;
    bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    unsigned int i = 0;

    while (i <= 10 && !unpaused) {
        ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        i++;
    }
    if (!unpaused) {
        ROS_FATAL("Could not wake up Gazebo.");
        return -1;
    } 
    else {
        ROS_INFO("Unpaused the Gazebo simulation.");
    } 
    ros::Duration(5.0).sleep();
    
    _poly_num1D = 2 * _der_order;//_poly_num1D is the maximum order of polynomial. order = 2 * der_order - 1, so 2 * _der_order of coef are needed.
    _trajectory_pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
    //_pos_sub = nh.subscribe(std::string("/firefly/odometry_sensor1/position").c_str(), 10, &updateUavPosition);
    _wp_traj_vis_pub = nh_private.advertise<visualization_msgs::Marker>("vis_trajectory", 1);
    
    ros::Rate rate(100);
    while(ros::ok()) 
    {
        getWaypoints();
        trajGeneration(_wayPoints);
        visWayPointTraj( _polyCoeff, _polyTime);
        simFlight( _polyCoeff, _polyTime);
        ros::spinOnce();
        rate.sleep();
    }
    ros::shutdown();
    return 0;
}

void getWaypoints() {
    //Get waypoints via keyboard input.
    int waypoint_num;
    cout << "Input the number of waypoints: ";
    cin >> waypoint_num;
    ROS_INFO("%d waypoints needed.", waypoint_num);
    goToPos(_startPos, 0.0);//Go to the start Position, origin by default.
    vector<Vector3d> waypoint_list;
    waypoint_list.clear();

    float x, y, z;
    for (int i = 0; i < waypoint_num; i++) {
        cout << " Waypoint " << i+1 << ": ";
        cin >> x >> y >> z;
        ROS_INFO("Waypint %d [%f, %f, %f]", i+1, x, y, z);
        Vector3d pt(x, y, z);
        waypoint_list.push_back(pt);
    }

    MatrixXd waypoints(waypoint_num + 1, 3);
    waypoints.row(0) = _startPos;
    for (int i = 0; i < waypoint_num; i++) {
        waypoints.row(i + 1) = waypoint_list[i];
    }
    _wayPoints = waypoints;
}

VectorXd timeAllocation(const MatrixXd& Path) {
    VectorXd time(Path.rows() - 1);
    for (int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i + 1) - Path.row(i)).norm();
        double x1 = _Vel * _Vel / (2 * _Acc);
        double x2 = distance - 2 * x1;
        double t1 = _Vel / _Acc;
        double t2 = x2 / _Vel;
        time(i) = 2 * t1 + t2;
    }
    return time;
}

//Trajectory Generation 
void trajGeneration(const MatrixXd& path) {
    TrajectoryGeneratorWaypoint trajectoryGeneratorWaypoint;
    MatrixXd vel = MatrixXd::Zero(2, 3);
    MatrixXd acc = MatrixXd::Zero(2, 3);
    vel.row(0) = _startVel;
    vel.row(1) = _endVel;
    acc.row(0) = _startAcc;
    acc.row(1) = _endAcc;
    _polyTime = timeAllocation(path);
    _polyCoeff = trajectoryGeneratorWaypoint.PolyQPGeneration(_der_order, path, vel, acc, _polyTime);
}

Vector3d getPosPoly( const MatrixXd& polyCoeff, int k, double t ) {
    Vector3d ret;
    for ( int dim = 0; dim < 3; dim++ ) {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );//coeff:{px0, px1, ..., py0, py1, ..., pz0, pz1, ...}
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++) {
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);
        }
        ret(dim) = coeff.dot(time);//ret(0) for x, 1 for y, 2 for z.
    }
    return ret;
}

//Visualize the trajectory in Rviz.
void visWayPointTraj( const MatrixXd& polyCoeff, const VectorXd& time) {
    visualization_msgs::Marker _traj_vis;
    //Marker params
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 0;
    _traj_vis.type = visualization_msgs::Marker::SPHERE_LIST;
    _traj_vis.action = visualization_msgs::Marker::ADD;
    _traj_vis.scale.x = _vis_traj_width;
    _traj_vis.scale.y = _vis_traj_width;
    _traj_vis.scale.z = _vis_traj_width;
    _traj_vis.pose.orientation.x = 0.0;
    _traj_vis.pose.orientation.y = 0.0;
    _traj_vis.pose.orientation.z = 0.0;
    _traj_vis.pose.orientation.w = 1.0;

    _traj_vis.color.a = 1.0;
    _traj_vis.color.r = 1.0;
    _traj_vis.color.g = 0.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    for(int i = 0; i < time.size(); i++ ) {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1) {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);
          if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    ROS_WARN("Trajectory length is %f m", traj_len);
    _wp_traj_vis_pub.publish(_traj_vis);
}

//Comfirm that the UAV has reached the desired position.
void updateUavPosition(const geometry_msgs::PointStamped& msg) {
    current_position = msg;
}

double getDistanceToTarget(const Vector3d& target) {
    double temp = 0;
    temp += pow((target[0] - current_position.point.x), 2);
    temp += pow((target[1] - current_position.point.y), 2);
    temp += pow((target[2] - current_position.point.z), 2);
    temp = sqrt(temp);
    return temp;
}

bool reachTargetPosition(const Vector3d& target, double max_error) {
    double temp = getDistanceToTarget(target);
    //cout << temp << endl;
    if (temp < max_error)
        return true;
    return false;
}

//navigate the UAV to a specific position.
void goToPos(const Vector3d& pos, double yaw) {
    trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(pos, yaw, &trajectory_msg);
    _trajectory_pub.publish(trajectory_msg);
}

//Wait for some seconds. It seems that sleep() is useless. 
void delay(float secs) {
        
	clock_t delay = secs * CLOCKS_PER_SEC;       
	clock_t start = clock();
	while (clock() - start < delay);
}

//waypoint publisher
void simFlight(const MatrixXd& polyCoeff, const VectorXd& time) {
    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();
    Vector3d pos;
    double desired_yaw = 0.0;
    double seg = 0.01;
    for(int i = 0; i < time.size(); i++) {   
        for (double t = 0.0; t < time(i); t += seg) {
            pos = getPosPoly(polyCoeff, i, t);
            cur(0) = pos(0);
            cur(1) = pos(1);
            cur(2) = pos(2);
            //cout << pos(0) <<',' << pos(1) << ',' << pos(2) << endl;
            goToPos(pos, desired_yaw);
            double dis;
            if (++count) {
                dis = (pre - cur).norm();//Calculate the length of the traj
                traj_len += dis;
                delay(seg); //delay for set segment time. Without this delay, UAV will go directly to the end point.
            }
            pre = cur;
        }
        cout << pos(0) <<',' << pos(1) << ',' << pos(2) << endl;
    }
    ROS_WARN("Trajectory length is %f m.", traj_len);
}