#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <random>
#include <ros/ros.h>
#include <trajectory_generator/coeff_msgs.h>
#include <trajectory_generator/alpha_msgs.h>
#include <ros/console.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>
#include <Eigen/Eigen>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace Eigen;



double _target;


double _vis_traj_width;
ros::Subscriber _alpha_sub, _coeff_sub;

MatrixXd _polyCoeff;
ros::Publisher _wp_traj_vis_pub, _st_traj_vis_pub;

vector<double> _time;
vector<double> _errs, _times, _vzs;
vector<VectorXd> _pos_vel;
vector<VectorXd> _shoot_pos, _cur_shoot_pos;
int _coeff_num, _segment; 

vector<double> _alphas;
int _sample_num;
int _launch_x, _launch_z, launch_angle;
double _v0 = 3.0, _gamma=0.25, _D=0.03, _m=0.1, _g=9.81, _dt=0.05;

void coeffCallback(const trajectory_generator::coeff_msgs::ConstPtr& msg);
void alphaCallback(const trajectory_generator::alpha_msgs::ConstPtr& msg);
void getPosVel(const MatrixXd& polyCoeff, int k, double t );
Vector4d alphaCheck(double tu, double x0, double z0, double vxu, double vzu, double alpha);
void visWayPointTraj();
Vector3d getPosPoly(const MatrixXd& polyCoeff, int k, double t);
void visShootTraj();
double evaluate(const Vector4d& evaluation);

int main(int argc, char** argv){

    ros::init(argc, argv, "check_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    nh_private.param("vis/vis_traj_width", _vis_traj_width, 0.05);

    
    _wp_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("/trajectory_generator_node/vis_uav_trajectory", 10);
    _st_traj_vis_pub = nh.advertise<visualization_msgs::Marker>("/trajectory_generator_node/vis_shoot_trajectory", 10);
    
    
    _coeff_sub = nh.subscribe<trajectory_generator::coeff_msgs>("/trajectory_generator_node/coeff", 1000, coeffCallback);
    _alpha_sub = nh.subscribe<trajectory_generator::alpha_msgs>("/trajectory_generator_node/alpha", 2000, alphaCallback);
    
    ros::spin();
    return 0;
}

void alphaCallback(const trajectory_generator::alpha_msgs::ConstPtr& msg) {
    _sample_num = msg->sample_num;
    int size = msg->alphas.size();
    double min_distance, err;
    _alphas.clear();
    for (int i = 0; i < msg->alphas.size();i++){
         _alphas.push_back(msg->alphas[i]);
    }
    ROS_INFO_STREAM("Alpha Check Starting...");
    //for (int k = 0; k < _time.size(); k++) cout << _time[k]<<endl;
    int mode = 3;
    bool flag = 1;

    double min_err=100.0;double min_time = 100.0; double min_vz = -100.0;
    double fx,fz,fvx,fvz,ft, fa, fvh;
    int count = 0, pc=0; double tu0 = 0.0; bool cont = true;
    
    
    for (int k = 0; k < _time.size(); k++){
        for (double t = 0.0; t < _time[k]; t += _dt){
            getPosVel(_polyCoeff, k, t);
            if (_pos_vel[pc][0] > _target){
                cont = false;
                
            }
            for (int i = 0; i < 2; i++){
                if (!cont){
                    continue;
                }
                double alpha = _alphas[count++];
                if (int(alpha) == int(114514)){
                    continue;
                }

                double u_angle = atan(_pos_vel[pc][1] / _pos_vel[pc][0]);
                if (alpha > u_angle){
                    continue;
                }
                
                //err vz t
                Vector4d evaluation = alphaCheck(tu0 + t, _pos_vel[pc][0], _pos_vel[pc][1], _pos_vel[pc][2], _pos_vel[pc][3], alpha);
                // ofstream outfile("/home/hsy/points.txt", ios::app);
                // outfile << evaluation[0] << "," << evaluation[1] << "," << evaluation[2] <<"," <<_pos_vel[pc][0]<<","<< _pos_vel[pc][1]<< endl;

                // outfile.close();
                _errs.push_back(evaluation[0]); _times.push_back(evaluation[2]);_vzs.push_back(evaluation[1]);
                //MIN ERR
                if (mode == 1 && min_err > evaluation[0]){
                    min_time = evaluation[2];
                    min_err = evaluation[0];
                    min_vz = evaluation[1];
                    _shoot_pos = _cur_shoot_pos;
                    fx = _pos_vel[pc][0]; fz = _pos_vel[pc][1]; 
                    fvh = evaluation[1];
                    fvx = _pos_vel[pc][2]; fvz = _pos_vel[pc][3]; ft = evaluation[2]; fa = alpha;
                }
                //MIN TIME
                if (mode == 2 && min_time > evaluation[2]){
                    min_time = evaluation[2];
                    min_err = evaluation[0];
                    min_vz = evaluation[1];
                    _shoot_pos = _cur_shoot_pos;
                    fx = _pos_vel[pc][0]; fz = _pos_vel[pc][1]; 
                    fvh = evaluation[1];
                    fvx = _pos_vel[pc][2]; fvz = _pos_vel[pc][3]; ft = evaluation[2]; fa = alpha;
                }
                //MIN VZ
                if (mode == 3 && min_vz < evaluation[1]){
                    min_time = evaluation[2];
                    min_err = evaluation[0];
                    min_vz = evaluation[1];
                    _shoot_pos = _cur_shoot_pos;
                    fx = _pos_vel[pc][0]; fz = _pos_vel[pc][1]; 
                    fvh = evaluation[1];
                    fvx = _pos_vel[pc][2]; fvz = _pos_vel[pc][3]; ft = evaluation[2]; fa = alpha;
                }
                // EVAL FUNC
                // if (mode == 4 && flag && evaluation[0] < 0.001){
                //     flag = 0;
                //     min_time = evaluation[2];
                //     min_err = evaluation[0];
                //     min_vz = evaluation[1];
                //     _shoot_pos = _cur_shoot_pos;
                //     fx = _pos_vel[pc][0]; fz = _pos_vel[pc][1]; 
                //     fvh = evaluation[1];
                //     fvx = _pos_vel[pc][2]; fvz = _pos_vel[pc][3]; ft = evaluation[2]; fa = alpha;
                // }
                cout <<  "err: " << evaluation[0] << " vz: " << evaluation[1] << " time: " << evaluation[2] << endl;
                cout << endl;
                cout << "-";
            }
            pc += 1;
        }
        tu0 += _time[k];
    }
    cout <<endl;
    
    cout << count << "Alphas Checked" << endl;
    cout << "Checking x0: " <<fx<<" z0: "<<fz<<" vxu: "<<fvx<<" vzu: "<<fvz<<" a: "<<fa<<endl;
    cout <<  "err: " << min_err << " vz: " << fvh << " time: " << ft << endl;
    cout << _shoot_pos.size()<<endl;
    visWayPointTraj();
    visShootTraj();
    ROS_INFO_STREAM("DONE");
}

void getPosVel(const MatrixXd& polyCoeff, int k, double t ){
    VectorXd ret(4);
    VectorXd time = VectorXd::Zero(_coeff_num);
    VectorXd vel_time = VectorXd::Zero(_coeff_num - 1);
    for(int j = 0; j < _coeff_num; j ++){
        if(j==0)
            {time(j) = 1.0; vel_time(j) = 1.0;}
        else if (j == _coeff_num - 1){
            time(j) = pow(t, j);
        } else {
            time(j) = pow(t, j); vel_time(j) = (j+1)*pow(t, j);  
        }       
    }
    for (int dim = 0; dim < 2; dim++){
        VectorXd coeff = (polyCoeff.row(k)).segment(dim * _coeff_num, _coeff_num);
        ret(dim) = coeff.dot(time);
        coeff =  (polyCoeff.row(k)).segment(dim * _coeff_num + 1, _coeff_num - 1);
        ret(dim + 2) = coeff.dot(vel_time);
    }
    _pos_vel.push_back(ret);
}

void coeffCallback(const trajectory_generator::coeff_msgs::ConstPtr& msg){
    _pos_vel.clear();
    _time.clear();
    _coeff_num = msg->coeff_num; _segment = msg->segment; _target = msg->target;
    int size = msg->coeff.size();

    MatrixXd polyCoeff(1, _coeff_num*2*_segment);
    for (int i = 0; i < msg->coeff.size();i++){
        polyCoeff(0,i) = msg->coeff[i];
    }
    for (int j = 0; j < msg->time.size();j++){
        _time.push_back(msg->time[j]);
    }
    polyCoeff.resize(_segment, _coeff_num*2);
    _polyCoeff = polyCoeff;
    ROS_INFO_STREAM("Check Node coeff updated.");
}

double evaluate(const Vector4d& evaluation){
    //err vz t
    double a,b,c;
    return a*evaluation[0]+b*evaluation[1]+c*evaluation[2];
}

Vector4d alphaCheck(double tu, double x0, double z0, double vxu, double vzu, double alpha) {
    cout << "Checking x0: " <<x0<<" z0: "<<z0<<" vxu: "<<vxu<<" vzu: "<<vzu<<" a: "<<alpha <<" tu: " << tu<<endl;
    _cur_shoot_pos.clear();
    double z = z0, x = x0;
    double x1 = vxu + _v0* cos(alpha),z1 = vzu + _v0*sin(alpha);
    double v, x2, z2;
    double c = _gamma * _D * _D / _m;
    double t = 0, k;
    double _d_t = 0.001;
    VectorXd xz(2);
    while (z>0) {
        xz[0] = x; xz[1] = z;
        _cur_shoot_pos.push_back(xz);
        
        x += x1 * _d_t; 
        z += z1 * _d_t;
        v = sqrt(x1 * x1 + z1 * z1);
        k = _gamma * _D * _D / _m * v;
        x2 = -k * x1;
        z2 = -_g - k * z1;
        x1 += x2 * _d_t;
        z1 += z2 * _d_t;
        t += _d_t;
    }
    double err = abs(x - _target);
    //double score = evaluate(vz, tu + t, 0);
    Vector4d eval(err, z1, tu+t,0);
    return eval;
}

Vector3d getPosPoly(const MatrixXd& polyCoeff, int k, double t )
{
    Vector3d ret;

    for ( int dim = 0; dim < 2; dim++ )
    {
        VectorXd coeff = (polyCoeff.row(k)).segment( dim * _coeff_num, _coeff_num );//coeff:{px0, px1, ..., py0, py1, ..., pz0, pz1, ...}
        VectorXd time  = VectorXd::Zero( _coeff_num );
        
        for(int j = 0; j < _coeff_num; j ++)
          if(j==0)
              time(j) = 1.0;
          else
              time(j) = pow(t, j);

        ret(dim) = coeff.dot(time);
        //ret(0) for x, 1 for y, 2 for z.
    }
    ret(3) = ret(2);
    ret(2) = 0.0;
    return ret;
}

void visWayPointTraj()
{
    ROS_INFO_STREAM("Visualization");
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

    // for(int i = 0; i < time.size(); i++ )
    // {   
    //     for (double t = 0.0; t < time[i]; t += 0.01, count += 1)
    //     {
    //         ROS_INFO_STREAM("HERE");
    //         pos = getPosPoly(polyCoeff, i, t);
    //         cur(0) = pt.x = pos(0);
    //         cur(1) = pt.y = pos(1);
    //         cur(2) = pt.z = pos(2);
    //         _traj_vis.points.push_back(pt);

    //         //if (count) traj_len += (pre - cur).norm();
    //         pre = cur;
    //     }
    // }
    int i;
    for (i = 0; i < _pos_vel.size(); i++){
        pt.x = _pos_vel[i][0];
        pt.y = 0.0;
        pt.z = _pos_vel[i][1];
        _traj_vis.points.push_back(pt);
    }
    cout << i << " waypoints"<<endl;
    ROS_INFO_STREAM("HERE");
    //ROS_WARN("Trajectory length is %f m", traj_len);
    _wp_traj_vis_pub.publish(_traj_vis);
}

void visShootTraj()
{
    ROS_INFO_STREAM("Visualization Shooting");
    visualization_msgs::Marker _traj_vis;
    
    //Marker params
    _traj_vis.header.stamp       = ros::Time::now();
    _traj_vis.header.frame_id    = "map";

    _traj_vis.ns = "traj_node/trajectory_waypoints";
    _traj_vis.id = 1;
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
    _traj_vis.color.r = 0.0;
    _traj_vis.color.g = 1.0;
    _traj_vis.color.b = 0.0;

    double traj_len = 0.0;
    int count = 0;
    Vector3d cur, pre;
    cur.setZero();
    pre.setZero();

    _traj_vis.points.clear();
    Vector3d pos;
    geometry_msgs::Point pt;

    int i;
    for (i = 0; i < _shoot_pos.size(); i++){
        pt.x = _shoot_pos[i][0];
        pt.y = 0.0;
        pt.z = _shoot_pos[i][1];
        _traj_vis.points.push_back(pt);
    }
    cout << i << " shooting waypoints"<<endl;
    ROS_INFO_STREAM("SHOOT");
    //ROS_WARN("Trajectory length is %f m", traj_len);
    _st_traj_vis_pub.publish(_traj_vis);
}