#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <trajectory_generator/coeff_msgs.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include "trajectory_generator_waypoint.h"//essentials
#include <trajectory_generator/alpha_msgs.h>
#include <ctime>

using namespace std;
using namespace Eigen;


//Planning info
double _vis_traj_width;
double _Vel, _Acc;
double _target = 2.5;
int _der_order, _min_order;

//Sampling Info


//ros::Publisher _wp_traj_vis_pub, _wp_path_vis_pub;
ros::Publisher _coeff_pub;
//ros::Subscriber _alpha_sub;

// for planning
int _poly_num1D;
MatrixXd _polyCoeff; //the coeffs of generated traj.
VectorXd _polyTime;  //time allocation
Vector3d _startPos  = Vector3d::Zero(); //initial & end states.
Vector3d _startVel  = Vector3d::Zero();
Vector3d _startAcc  = Vector3d::Zero();
Vector3d _endVel    = Vector3d::Zero();
Vector3d _endAcc    = Vector3d::Zero();
MatrixXd _xzPoly;

//func
Vector3d getPosPoly(MatrixXd& polyCoeff, int k, double t);
VectorXd timeAllocation(MatrixXd& Path);
void trajGeneration(Eigen::MatrixXd& path);
MatrixXd initWaypoints();
MatrixXd getWaypoints();
MatrixXd getXZCoeff(MatrixXd& polyCoeff);
double alphaCheck(double tu, double x0, double z0, double vxu, double vzu, double alpha);
double evaluate(double vz, double total_t, double min_distance);
void alphaCallback(const trajectory_generator::alpha_msgs::ConstPtr& msg);
void delay(float secs) {
        
	clock_t delay = secs * CLOCKS_PER_SEC;       
	clock_t start = clock();
	while (clock() - start < delay);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_node");
    ros::NodeHandle nh("~");
    
    nh.param("planning/vel", _Vel, 0.1);
    nh.param("planning/acc", _Acc, 1.0);
    nh.param("planning/dev_order", _der_order, 4);  // the order of derivative, _der_order = 3->minimum jerk, _der_order = 4->minimum snap
    nh.param("planning/min_order", _min_order, 3);

    cout << _Vel<<endl;
    //_poly_num1D is the maximum order of polynomial. order = 2 * der_order - 1, so 2*_der_order of coef are needed.
    _poly_num1D = 2 * _der_order;

    _coeff_pub = nh.advertise<trajectory_generator::coeff_msgs>("coeff", 1000);
    
    
    ros::Rate rate(10);
    int count = -1;
    while(ros::ok()) 
    {
        MatrixXd wayPoints;
        if (count == -1){
            delay(2);
            count++;
            //wayPoints = initWaypoints();
            continue;
            
        } else {
            wayPoints = getWaypoints();
        }
        trajGeneration(wayPoints);
        count += 1;
        //delay(2);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

MatrixXd initWaypoints(){
    MatrixXd waypoints(2, 3);
    Vector3d pt(1, 0, 1);
    waypoints.row(0) = _startPos;waypoints.row(1) = pt;
    return waypoints;
}
MatrixXd getWaypoints(){
    //Get waypoints via keyboard input.
    int waypoint_num;
    cout << "Input the number of waypoints: " << endl;
    cin >> waypoint_num;
    cout <<  waypoint_num << " waypoints needed." << endl;

    vector<Vector3d> wp_list;
    wp_list.clear();

    double x, y, z; y =0.;
    for (int i = 0; i < waypoint_num; i++){
        cout << i + 1 << " waypoint:";
        cin >> x >> z;
        cout << "x: " << x << ", y: " << y << ", z: " << z << endl;
        Vector3d pt(x, y, z);
        wp_list.push_back(pt);
    }
    MatrixXd waypoints(waypoint_num + 1, 3);
    waypoints.row(0) = _startPos;
    for (int i = 0; i < waypoint_num; i++){
        waypoints.row(i + 1) = wp_list[i];
    }
    return waypoints;
}

void trajGeneration(Eigen::MatrixXd& path)
{
    TrajectoryGeneratorWaypoint tgw;    
    MatrixXd vel  = MatrixXd::Zero(2, 3); 
    MatrixXd acc  = MatrixXd::Zero(2, 3);
    vel.row(0)  = _startVel;    
    vel.row(1)  = _endVel;
    acc.row(0)  = _startAcc;
    acc.row(1)  = _endAcc;
    _polyTime  = timeAllocation(path);
    _polyCoeff = tgw.PolyQPGeneration(_der_order, path, vel, acc, _polyTime);

    //cout << _polyCoeff << endl;
    //cout<<"\n\n";
    _xzPoly = getXZCoeff(_polyCoeff);
    _xzPoly.resize(1, _poly_num1D*2*_polyCoeff.rows());
    
    //_xzPoly.resize(_polyCoeff.rows(), _poly_num1D*2);
    // cout << _polyTime<<endl;
    VectorXd mat = _xzPoly.row(0);
    vector<double> vec(mat.data(), mat.data() + mat.rows() * mat.cols());
    vector<double> time(_polyTime.data(), _polyTime.data() + _polyTime.rows() * _polyTime.cols());
    
    trajectory_generator::coeff_msgs msg;
    msg.coeff_num = _poly_num1D; msg.segment = _polyCoeff.rows();
    msg.coeff = vec; msg.time = time; msg.target = _target;
    _coeff_pub.publish(msg);
    ROS_INFO_STREAM("UAV trajectory generated.");
    ROS_INFO_STREAM("Target: 2.0");
    // std_msgs::Float32MultiArray msg; msg.data = _xzPoly;
    // _coeff_pub.publish(msg)

    
}

Vector3d getPosPoly(MatrixXd& polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 3; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _poly_num1D, _poly_num1D );//coeff:{px0, px1, ..., py0, py1, ..., pz0, pz1, ...}
        VectorXd time  = VectorXd::Zero( _poly_num1D );
        
        for(int j = 0; j < _poly_num1D; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //ret(0) for x, 1 for y, 2 for z.
    }

    return ret;
}

MatrixXd getXZCoeff(MatrixXd& polyCoeff){
    MatrixXd xzCoeff = MatrixXd::Zero(polyCoeff.rows(), 2*_poly_num1D);
    xzCoeff.block(0,0,polyCoeff.rows(), _poly_num1D) = polyCoeff.block(0,0,polyCoeff.rows(), _poly_num1D);
    xzCoeff.block(0,_poly_num1D,polyCoeff.rows(), _poly_num1D) = polyCoeff.block(0,_poly_num1D*2,polyCoeff.rows(), _poly_num1D);
    return xzCoeff;
}


VectorXd timeAllocation(MatrixXd& Path)
{ 
    VectorXd time(Path.rows() - 1);

    for(int i = 0; i < time.rows(); i++)
    {
        double distance = (Path.row(i+1) - Path.row(i)).norm();
        double x1 = _Vel * _Vel / (2 * _Acc); 
        double x2 = distance - 2 * x1;
        double t1 = _Vel / _Acc;
        double t2 = x2 / _Vel;
        time(i) = 2 * t1 + t2;
    }
    //cout << time;
    return time;
}

//VectorXd get
/*
void visWayPointTraj( MatrixXd polyCoeff, VectorXd time)
{
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


    for(int i = 0; i < time.size(); i++ )
    {   
        for (double t = 0.0; t < time(i); t += 0.01, count += 1)
        {
          pos = getPosPoly(polyCoeff, i, t);
          cur(0) = pt.x = pos(0);
          cur(1) = pt.y = pos(1);
          cur(2) = pt.z = pos(2);
          _traj_vis.points.push_back(pt);

          //if (count) traj_len += (pre - cur).norm();
          pre = cur;
        }
    }
    //ROS_WARN("Trajectory length is %f m", traj_len);
    _wp_traj_vis_pub.publish(_traj_vis);
}

void visWayPointPath(MatrixXd path)
{
    visualization_msgs::Marker points, line_list;
    int id = 0;
    points.header.frame_id    = line_list.header.frame_id    = "map";
    points.header.stamp       = line_list.header.stamp       = ros::Time::now();
    points.ns                 = line_list.ns                 = "wp_point";
    points.action             = line_list.action             = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;
    points.pose.orientation.x = line_list.pose.orientation.x = 0.0;
    points.pose.orientation.y = line_list.pose.orientation.y = 0.0;
    points.pose.orientation.z = line_list.pose.orientation.z = 0.0;

    points.id    = id;
    line_list.id = id;

    points.type    = visualization_msgs::Marker::SPHERE_LIST;
    line_list.type = visualization_msgs::Marker::LINE_STRIP;

    points.scale.x = 0.3;
    points.scale.y = 0.3;
    points.scale.z = 0.3;
    points.color.a = 1.0;
    points.color.r = 0.0;
    points.color.g = 0.0;
    points.color.b = 0.0;

    line_list.scale.x = 0.05;
    line_list.scale.y = 0.05;
    line_list.scale.z = 0.05;
    line_list.color.a = 1.0;

    
    line_list.color.r = 0.0;
    line_list.color.g = 1.0;
    line_list.color.b = 0.0;
    
    line_list.points.clear();

    for(int i = 0; i < path.rows(); i++){
      geometry_msgs::Point p;
      p.x = path(i, 0);
      p.y = path(i, 1); 
      p.z = path(i, 2); 

      points.points.push_back(p);
      line_list.points.push_back(p);
    }

    _wp_path_vis_pub.publish(points);
    _wp_path_vis_pub.publish(line_list);
}
*/