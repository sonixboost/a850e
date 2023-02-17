#include <ros/ros.h>
#include <stdio.h>
#include <cmath>
#include <errno.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include "common.hpp"

bool target_changed = false;
Position target;
void cbTarget(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    target.x = msg->point.x;
    target.y = msg->point.y;
}

Position pos_rbt(0, 0);
double ang_rbt = 10; // set to 10, because ang_rbt is between -pi and pi, and integer for correct comparison while waiting for motion to load
void cbPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    auto &p = msg->pose.position;
    pos_rbt.x = p.x;
    pos_rbt.y = p.y;

    // euler yaw (ang_rbt) from quaternion <-- stolen from wikipedia
    auto &q = msg->pose.orientation; // reference is always faster than copying. but changing it means changing the referenced object.
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ang_rbt = atan2(siny_cosp, cosy_cosp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "turtle_move");
    ros::NodeHandle nh;

    // Get ROS Parameters
    bool enable_move;
    if (!nh.param("enable_move", enable_move, true))
        ROS_WARN(" TMOVE : Param enable_move not found, set to true");
    bool verbose;
    if (!nh.param("verbose_move", verbose, false))
        ROS_WARN(" TMOVE : Param verbose_move not found, set to false");
    double Kp_lin;
    if (!nh.param("Kp_lin", Kp_lin, 1.0))
        ROS_WARN(" TMOVE : Param Kp_lin not found, set to 1.0");
    double Ki_lin;
    if (!nh.param("Ki_lin", Ki_lin, 0.0))
        ROS_WARN(" TMOVE : Param Ki_lin not found, set to 0");
    double Kd_lin;
    if (!nh.param("Kd_lin", Kd_lin, 0.0))
        ROS_WARN(" TMOVE : Param Kd_lin not found, set to 0");
    double max_lin_vel;
    if (!nh.param("max_lin_vel", max_lin_vel, 0.22))
        ROS_WARN(" TMOVE : Param max_lin_vel not found, set to 0.22");
    double max_lin_acc;
    if (!nh.param("max_lin_acc", max_lin_acc, 1.0))
        ROS_WARN(" TMOVE : Param max_lin_acc not found, set to 1");
    double Kp_ang;
    if (!nh.param("Kp_ang", Kp_ang, 1.0))
        ROS_WARN(" TMOVE : Param Kp_ang not found, set to 1.0");
    double Ki_ang;
    if (!nh.param("Ki_ang", Ki_ang, 0.0))
        ROS_WARN(" TMOVE : Param Ki_ang not found, set to 0");
    double Kd_ang;
    if (!nh.param("Kd_ang", Kd_ang, 0.0))
        ROS_WARN(" TMOVE : Param Kd_ang not found, set to 0");
    double max_ang_vel;
    if (!nh.param("max_ang_vel", max_ang_vel, 2.84))
        ROS_WARN(" TMOVE : Param max_ang_vel not found, set to 2.84");
    double max_ang_acc;
    if (!nh.param("max_ang_acc", max_ang_acc, 4.0))
        ROS_WARN(" TMOVE : Param max_ang_acc not found, set to 4");
    double move_iter_rate;
    if (!nh.param("move_iter_rate", move_iter_rate, 25.0))
        ROS_WARN(" TMOVE : Param move_iter_rate not found, set to 25");

    // Subscribers
    ros::Subscriber sub_target = nh.subscribe("target", 1, &cbTarget);
    ros::Subscriber sub_pose = nh.subscribe("pose", 1, &cbPose);

    // Publishers
    ros::Publisher pub_cmd = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1, true);

    // prepare published messages
    geometry_msgs::Twist msg_cmd; // all properties are initialised to zero.

    // Setup rate
    ros::Rate rate(move_iter_rate); // same as publishing rate of pose topic

    // wait for other nodes to load
    ROS_INFO(" TMOVE : Waiting for topics");
    while (ros::ok() && nh.param("run", true) && ang_rbt == 10) // not dependent on main.cpp, but on motion.cpp
    {
        rate.sleep();
        ros::spinOnce(); //update the topics
    }

    // Setup variables
    double cmd_lin_vel = 0, cmd_ang_vel = 0;
    double dt;
    double prev_time = ros::Time::now().toSec();

    ////////////////// DECLARE VARIABLES HERE //////////////////
    //double error_lin = dist_euc(target.y, pos_rbt.y, target.x, pos_rbt.x); //initial positional error
    double error_lin = target.x - pos_rbt.x; //linear experiment
    double sum_error_lin = 0;
    double prev_error_lin = error_lin;
    double pkr = 0; //proportion component, positional
    double ikr = 0; //integral component, positional
    double dkr = 0; //differential component, positional

    double error_ang = limit_angle(atan2(target.y - pos_rbt.y,target.x - pos_rbt.x) - ang_rbt); //initial angular error
    double sum_error_ang = 0;
    double prev_error_ang = error_ang;
    double pko = 0; //proportion component, angular
    double iko = 0; //integral component, angular
    double dko = 0; //differential component, angular

    double prev_cmd_lin_vel = 0;
    double prev_cmd_ang_vel = 0;
    double akr; //Estimated acceleration in control signal at k.
    double akr_sat; //Saturated acceleration at k.
    double ako;
    double ako_sat;

    //experiments
    bool rise_time_ang1 = false;
    bool rise_time_ang2 = false;
    double time_start_ang;
    double time_end_ang;
    double time_diff_ang;
    double max_os_ang = 0;

    double initial_error_lin = error_lin;
    bool rise_time_lin1 = false;
    bool rise_time_lin2 = false;
    double time_start_lin;
    double time_end_lin;
    double time_diff_lin;
    double max_os_lin = 0;

    ROS_INFO(" TMOVE : ===== BEGIN =====");

    // main loop
    if (enable_move)
    {
        while (ros::ok() && nh.param("run", true))
        {
            // update all topics
            ros::spinOnce();

            dt = ros::Time::now().toSec() - prev_time;
            if (dt == 0) // ros doesn't tick the time fast enough
                continue;
            prev_time += dt;

            ////////////////// MOTION CONTROLLER HERE //////////////////
            error_ang = limit_angle(atan2(target.y - pos_rbt.y,target.x - pos_rbt.x) - ang_rbt);
            sum_error_ang += error_ang;
            pko = Kp_ang * error_ang;
            iko = Ki_ang * sum_error_ang * dt;
            dko = Kd_ang * (error_ang - prev_error_ang)/dt;
            cmd_ang_vel = pko + iko + dko; 
            prev_error_ang = error_ang;

            error_lin = sqrt((target.y - pos_rbt.y)*(target.y - pos_rbt.y) + (target.x - pos_rbt.x)*(target.x - pos_rbt.x));
            //error_lin = target.x - pos_rbt.x; //linear experiment
            sum_error_lin += error_lin;
            pkr = Kp_lin * error_lin;
            ikr = Ki_lin * sum_error_lin * dt;
            dkr = Kd_lin * (error_lin - prev_error_lin)/dt;
            cmd_lin_vel  = cos(error_ang/2)*cos(error_ang/2)*(pkr + ikr + dkr); //equation 9 with coupling
            //cmd_lin_vel  = pkr + ikr + dkr; 
            prev_error_lin = error_lin;

            akr = (cmd_lin_vel - prev_cmd_lin_vel)/dt;
            akr_sat = sat(akr, max_lin_acc);
            cmd_lin_vel = sat(prev_cmd_lin_vel + akr*dt, max_lin_vel);
            prev_cmd_lin_vel = cmd_lin_vel;

            ako = (cmd_ang_vel - prev_cmd_ang_vel)/dt;
            ako_sat = sat(ako, max_ang_acc);
            cmd_ang_vel = sat(prev_cmd_ang_vel + ako*dt, max_ang_vel);
            prev_cmd_ang_vel = cmd_ang_vel;


            //experiments
            /*if (abs(limit_angle(ang_rbt)) >= 0.1*M_PI && !rise_time_ang1) //robot rotated 10%
            {
                time_start_ang = prev_time;
                rise_time_ang1 = true;
            }
            if (abs(limit_angle(ang_rbt)) >= 0.9*M_PI && !rise_time_ang2) //robot rotated 90%
            {
                time_end_ang = prev_time;
                rise_time_ang2 = true;
            }
            if (error_ang <= 0) //robot has reached/overshoot goal
            {
                if (max_os_ang <= abs(error_ang)) //obtaining max overshoot angle value
                {
                    max_os_ang = abs(error_ang);
                }
            }
            time_diff_ang = time_end_ang - time_start_ang; 
            
            if ((target.x - pos_rbt.x) <= 0.9*0.2 && !rise_time_lin1) //robot moved 10%
            {
                time_start_lin = prev_time;
                rise_time_lin1 = true;
            }
            if ((target.x - pos_rbt.x) <= 0.1*0.2 && !rise_time_lin2) //robot moved 90%
            {
                time_end_lin = prev_time;
                rise_time_lin2 = true;
            }
            if (error_lin <= 0) //robot has reached/overshoot goal
            {
                if (max_os_lin <= (pos_rbt.x - target.x)) //obtaining max overshoot distance value
                {
                    max_os_lin = pos_rbt.x - target.x;
                }
            }
            time_diff_lin = time_end_lin - time_start_lin;*/
            
            // publish speeds
            msg_cmd.linear.x = cmd_lin_vel;
            msg_cmd.angular.z = cmd_ang_vel; 
            pub_cmd.publish(msg_cmd);

            // verbose
            if (verbose)
            {
                ROS_INFO(" TMOVE :  FV(%6.3f) AV(%6.3f)", cmd_lin_vel, cmd_ang_vel);
                /*ROS_INFO(" experiment_ang :  time taken(%6.3f) error_ang(%6.3f, %6.3f) max_os(%6.3f)",
                time_diff_ang, limit_angle(ang_rbt), error_ang, max_os_ang);
                ROS_INFO(" experiment_lin :  time taken(%6.3f) lin(%6.3f) err_lin %6.3f) max_os(%6.3f)",
                time_diff_lin, initial_error_lin, pos_rbt.x, max_os_lin);*/
            }

            // wait for rate
            rate.sleep();
        }
    }

    // attempt to stop the motors (does not work if ros wants to shutdown)
    msg_cmd.linear.x = 0;
    msg_cmd.angular.z = 0;
    pub_cmd.publish(msg_cmd);

    ROS_INFO(" TMOVE : ===== END =====");
    return 0;
}