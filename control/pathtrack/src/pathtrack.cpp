#include <ros/ros.h>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32MultiArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <common/public.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <common/mydisplay.h>
#include <geometry_msgs/Twist.h>
#include <data_comm/car_ctr.h>
#include <data_comm/car_state.h>
#include <data_comm/paw_state.h>
#include <mqtt_comm/task.h>
#include <stdlib.h>

using namespace std;

class TPathTrack
{
private:
    ros::NodeHandle *nh_local, *nh;
    ros::Publisher aimpoint_pub, carctr_pub, sp_distance; //,following_err_pub;
    ros::Subscriber localpath_sub, carstate_sub, remainpath_sub, passedpath_sub, task_sub, wheel_err_sub, paw_state_sub ,shotpose_sub;
    nav_msgs::Path localpath;
    int self_turn_ctr = 0, turnmode_ctr = 0;
    data_comm::car_state cur_carstate;
    mqtt_comm::task cur_task;
    TNodeCheck *nodecheck;
    unsigned int stop_code = 0;
    float angle_offset = 0;
    bool test_flag = false;
    float test_speed = 0;

    geometry_msgs::PointStamped aimpoint;
    float aim_range = 2, ref_speed = 0.6; //, stop_erro = 0.5;
    float steering_property = 1;
    float track_angle_err = 0, orien_angle_err = 0;
    bool run_enable = false;
    int lidar_location_ctr = 0;
    string paw_state;
    float remain_path_length = 0;
    float passed_path_length = 0;
    float turn_speed_max = 0;

    bool mqtt_stop_enable, obs_stop_enable, traffic_stop_enable, laser_work_enable = true;
    bool white_obs_flag = false;
    float speedlimit = 999, obstacle_speedlimit = 999, traffic_speedlimit = 999, mqtt_speedlimit = 999;
    float wheel_err_front_internal = 999, wheel_err_rear_internal = 999;
    // float wheel_err_front_external = 999, wheel_err_rear_external = 999;

    bool lidar_work_enable = false;

    data_comm::paw_state paw_state_msg;
    geometry_msgs::PoseStamped shotpose;
    float shotpose_yaw;

public:
    TPathTrack()
    {
        nh_local = new ros::NodeHandle("~");
        nh = new ros::NodeHandle();
        aimpoint_pub = nh_local->advertise<geometry_msgs::PointStamped>("aimpoint", 10); // 发布预瞄点
        // following_err_pub = nh_local->advertise<geometry_msgs::PointStamped>("following_err", 10); // 发布预瞄点

        nodecheck = new TNodeCheck(nh_local, "node_rate");
        nodecheck->Find("node_rate")->SetLimit(10);

        carctr_pub = nh_local->advertise<data_comm::car_ctr>("ctr_cmd", 10); // 发布控制信息

        localpath_sub = nh->subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, &TPathTrack::LocalPathCallback, this);
        carstate_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &TPathTrack::CarStateCallback, this);
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &TPathTrack::TaskCallback, this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &TPathTrack::RemainPathCallback, this);
        passedpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/passedpath", 10, &TPathTrack::PassedPathCallback, this);
        wheel_err_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check_angle/wheel_err", 10, &TPathTrack::WheelErrCallback, this);
        paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, &TPathTrack::PawStateCallback, this);
        shotpose_sub = nh->subscribe<geometry_msgs::PoseStamped>("/localpathplan/target_pose", 10, &TPathTrack::ShotPoseCallback, this);

        aimpoint.header.frame_id = "";

        nh_local->getParam("angle_offset", angle_offset);
        nh_local->getParam("test_speed", test_speed);

        nh_local->getParam("/cloud_location_pro/lidar_work_enable", lidar_work_enable);
        test_flag = (fabs(test_speed) > 0.01);
    };

    void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void PassedPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void ShotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void Run();
    int SelfTurnCtr();
    int MoveModeCtr();
    void UpdateCtrParam(float vel);
    void GetAimAngleErr(bool is_hengyi_);
    int StopCtr();
    int StopCheck();
    int CheckMoveMode();
    void CheckSpeedLimit();
    void PubCarCtr(data_comm::car_ctr ctr);
    void PawStateCallback(const data_comm::paw_state::ConstPtr &msg);
};

void TPathTrack::ShotPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    shotpose = *msg;
    shotpose_yaw = GetYawFromPose(shotpose);
    return;
}

void TPathTrack::PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    paw_state_msg = *msg;
}

void TPathTrack::PubCarCtr(data_comm::car_ctr ctr)
{
    if (ctr.turnmode == 0 || ctr.turnmode == 3)
    {
        static data_comm::car_ctr last_ctr;
        float dvel = ctr.speed - last_ctr.speed;
        float max_dvel = 0.05;
        if (dvel > max_dvel)
            ctr.speed = last_ctr.speed + max_dvel;
        else if (dvel < -max_dvel)
            ctr.speed = last_ctr.speed - max_dvel;
        last_ctr = ctr;

        if (ctr.turnmode == 0)
            ctr.angle += angle_offset;
    }
    // printf("%.2f\n", ctr.angle);

    carctr_pub.publish(ctr);
}

void TPathTrack::WheelErrCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    wheel_err_rear_internal = msg->data[0];
    wheel_err_front_internal = msg->data[1];

    // wheel_err_rear_external = msg->data[2];
    // wheel_err_front_external = msg->data[3];

    // printf("wheel_err_front:%f\n", wheel_err_front_external);
    return;
}

//  接收任务指令
void TPathTrack::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("task") != string::npos)
        cur_task = *msg;
    else if (msg->cmd == "speedLimit ctrl")
    {
        mqtt_speedlimit = msg->path.front().vehSpeed;
    }
    else if (msg->cmd == "obstacleDetectionDisable ctrl")
    {
        nh_local->setParam("obs_stop_enable", (msg->subcmd == "0"));
    }
}

void TPathTrack::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length = msg->data;
}

void TPathTrack::PassedPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    passed_path_length = msg->data;
}

void TPathTrack::LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) // 接收局部路径 获取预瞄点
{
    localpath = *msg;
    // ROS_INFO("%d\n", localpath.poses.size());

    CheckSpeedLimit();

    aimpoint.point.x = aimpoint.point.y = 0;
    aimpoint.header = msg->header;
    aimpoint.header.stamp = ros::Time();
    if (localpath.poses.size() > 1)
    {
        aimpoint.point = (localpath.poses.end() - 1)->pose.position;
    }
    else
    {
        aimpoint.header.frame_id = "";
        localpath.poses.clear();
        self_turn_ctr = turnmode_ctr = 0;
        return;
    }

    float dd = 0;
    for (auto it = localpath.poses.begin(); it != localpath.poses.end() - 2; ++it) // 寻找预瞄点
    {
        float ds = GetDistanceXY(it->pose.position, (it + 1)->pose.position);
        dd += ds;
        if (dd >= aim_range)
        {
            aimpoint.point = it->pose.position;
            break;
        }
    }

    if (dd < aim_range)
    {
        // geometry_msgs::PoseStamped target_pose = *(localpath.poses.end() - 2);
        float L1 = aim_range - dd + 0.05; //  最后引导距离
        geometry_msgs::PoseStamped p1 = *(localpath.poses.end() - 1);
        geometry_msgs::PoseStamped p2 = *(localpath.poses.end() - 2);
        p1.pose.orientation = GetQuaternionMsgByPoints(p2.pose.position, p1.pose.position);
        geometry_msgs::PoseStamped p = GetExtendPoseByPose(p1, L1);
        aimpoint.point = p.pose.position;
    }

    ref_speed = localpath.poses[0].pose.position.z; //  速度为首点Z数据
    if (ref_speed > speedlimit)
        ref_speed = speedlimit;

    // printf("vel0=%.2f\n", localpath.poses[0].pose.position.z);
    // printf("vel1=%.2f\n", localpath.poses[1].pose.position.z);
}

void TPathTrack::CarStateCallback(const data_comm::car_state::ConstPtr &msg) //  接收车体状态信息
{
    cur_carstate = *msg;
}

int TPathTrack::SelfTurnCtr() //  自身转动控制
{
    int set_movemode = CheckMoveMode();
    bool is_hengyi_ = (set_movemode == 3);
    static float last_angle_err = 0;
    last_angle_err = track_angle_err;
    GetAimAngleErr(is_hengyi_);

    static TTimer turn_tmr;
    float max_angle_err = 10;
    if (self_turn_ctr == 0 && fabs(track_angle_err) > max_angle_err && speedlimit > 0.1)
        self_turn_ctr = 1, turn_tmr.Clear();

    // if(self_turn_ctr) ROS_INFO("%.1f  %d\n", track_angle_err, self_turn_ctr);

    if (self_turn_ctr == 0)
        return 0;
    turnmode_ctr = 0;

    data_comm::car_ctr car_ctr;
    car_ctr.workmode = 3;
    car_ctr.speed = car_ctr.angle = 0;
    if (self_turn_ctr == 1) // 停止运动
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        if (turn_tmr.GetValue() > 0.2 && fabs(cur_carstate.speed[0]) < 0.01)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 2) // 设置自转动
    {
        car_ctr.turnmode = 2;
        if (turn_tmr.GetValue() > 0.2 && cur_carstate.turnmode == 2)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 3) //  转动至角度误差小于阈值
    {
        ///////////////////////////
        /*   分段调节
        float max_err=0.1;
        car_ctr.turnmode=2;

        // 简单分段调整速度  待调试
        float turn_speed=0.03;
        if(fabs(track_angle_err)>30)  turn_speed=0.3;
        else if(fabs(track_angle_err)>15)  turn_speed=0.2;
        else if(fabs(track_angle_err) > 6) turn_speed=0.1;

        // ROS_INFO("err_angle=%.2f", track_angle_err);
        if (track_angle_err > max_err)  car_ctr.speed = turn_speed;
        else if (track_angle_err < -max_err)  car_ctr.speed = -turn_speed;
        */
        ///////////////////////////

        float max_err = 0.1;      // 最小误差角（由于线性状态机的继续前进的判断有误差变号条件，不需要依靠较大的最小误差保证状态机前进）
        float slow_down_err = 30; // 开始减速的误差角

        // 最大旋转速度             //////////////
        float turn_speed_min = 0.03; // 最小旋转速度
        car_ctr.turnmode = 2;        // 设置自传模式

        float err_abs = fabs(track_angle_err); // 误差的绝对值
        // ROS_INFO("err_angle=%.2f", track_angle_err);
        if (max_err < err_abs && err_abs < slow_down_err) // 在开始减速的误差角度和最小误差角之间，速度线性地由最大转向速度递减为最小转向速度
            car_ctr.speed = turn_speed_min + (err_abs - max_err) / (slow_down_err - max_err) * (turn_speed_max - turn_speed_min);
        else if (err_abs > slow_down_err) // 在开始减速误差角之外采用最大旋转速度
            car_ctr.speed = turn_speed_max;

        if (car_ctr.speed > speedlimit)
            car_ctr.speed = speedlimit;
        if (track_angle_err < -max_err) // 如果是反转加负号
            car_ctr.speed *= -1;

        ///////////////////

        if (ref_speed < 0)
            car_ctr.speed *= -1;

        if (fabs(track_angle_err) < max_err || last_angle_err * track_angle_err < 0)
        {
            car_ctr.speed = 0;
            self_turn_ctr++, turn_tmr.Clear();
        }
    }
    else if (self_turn_ctr == 4) // stop turn
    {
        car_ctr.turnmode = 2;
        if (turn_tmr.GetValue() > 0.5)
            self_turn_ctr++, turn_tmr.Clear();
    }
    else if (self_turn_ctr == 5) // 设置为阿克曼转向
    {
        car_ctr.turnmode = set_movemode;
        if (turn_tmr.GetValue() > 2 && car_ctr.turnmode == cur_carstate.turnmode)
            self_turn_ctr = 0;
    }

    // printf("err=%.2f ctr=%d\n", track_angle_err, self_turn_ctr);
    // carctr_pub.publish(car_ctr);

    // printf("%d\n", car_ctr.turnmode);
    PubCarCtr(car_ctr);

    return self_turn_ctr;
}

void TPathTrack::UpdateCtrParam(float vel) //  根据速度规划预瞄点和转向控制系数
{
    vel = fabs(vel);
    if (vel > 5 || speedlimit < 1)
        aim_range = 25, steering_property = 0.6;
    else if (vel > 4.7)
        aim_range = 20, steering_property = 0.7;
    else if (vel > 3.6)
        aim_range = 20, steering_property = 0.8;
    else if (vel > 2.8)
        aim_range = 20, steering_property = 1.0;
    else if (vel > 1)
        aim_range = 15, steering_property = 1.2;
    else if (vel > 0.3)
        aim_range = 4, steering_property = 3;
    else
        aim_range = 2, steering_property = 4;

    if (cur_carstate.turnmode == 2)
        aim_range = 10;
}

void TPathTrack::GetAimAngleErr(bool is_hengyi_) //  根据横移状态和运动速度判断跟踪角度误差
{
    track_angle_err = 0;
    if (aimpoint.header.frame_id == "" || localpath.poses.size() < 1)
    {
        return;
    }

    geometry_msgs::PointStamped dst_p;
    transformPoint("base_link", aimpoint, dst_p, "");

    if (ref_speed >= 0 && !is_hengyi_)
    {
        track_angle_err = atan2(dst_p.point.y, dst_p.point.x);
    }
    else if (ref_speed < 0 && !is_hengyi_)
        track_angle_err = atan2(dst_p.point.y, -dst_p.point.x);
    else if (ref_speed >= 0 && is_hengyi_) // left
        track_angle_err = -atan2(dst_p.point.x, dst_p.point.y);
    else if (ref_speed < 0 && is_hengyi_) //  right
        track_angle_err = -atan2(dst_p.point.x, -dst_p.point.y);

    track_angle_err *= 180 / M_PI;

    // printf("hengyi=%d vel=%.1f angle=%.2f\n", is_hengyi_, ref_speed, track_angle_err);

    orien_angle_err = 0;
    geometry_msgs::PoseStamped dst_orien;
    transformPose("base_link", localpath.poses[0], dst_orien, "");
    orien_angle_err = GetYawFromPose(dst_orien) * 180 / M_PI;

    // printf("----------    %f---------     %f--------------\n", track_angle_err ,orien_angle_err);
}

int TPathTrack::CheckMoveMode()
{
    int res = 0; // return res;

    if (localpath.poses.size()) //  根据预瞄点判断是否横移
    {
        geometry_msgs::PoseStamped p1 = localpath.poses.front();
        geometry_msgs::PoseStamped p2 = localpath.poses.back();
        float pose_angle = GetYawFromPose(p1);
        float path_angle = GetAngleByPoints(p1.pose.position, p2.pose.position);
        float anglex = (path_angle - pose_angle) * 180 / M_PI;

        // printf("%.2f %.2f %.2f\n", pose_angle * 180 / M_PI, path_angle * 180 / M_PI,  anglex);

        if (anglex > 180)
            anglex -= 360;
        else if (anglex < -180)
            anglex += 360;

        // printf("%.1f\n", anglex);

        //  return check
        if (fabs(anglex) > 160)
            ref_speed = -fabs(ref_speed);
        else if (abs(anglex) < 20)
            ref_speed = fabs(ref_speed);

        //  hengyi check
        if (fabs(anglex + 90) < 20)
            res = -3, ref_speed = -fabs(ref_speed); //  right move
        else if (fabs(anglex - 90) < 20)
            res = 3, ref_speed = fabs(ref_speed); // left move
    }

    return abs(res);
}

int TPathTrack::MoveModeCtr() // 切换turnmode
{
    int set_turnmode = CheckMoveMode();
    static TTimer ctr_tmr;
    // printf("set_turnmode=%d self_turn=%d turn_ctr=%d\n", set_turnmode, self_turn_ctr, turnmode_ctr);

    if (turnmode_ctr == 0 && self_turn_ctr == 0 && set_turnmode != cur_carstate.turnmode)
        turnmode_ctr = 1, ctr_tmr.Clear();

    // ROS_INFO("%d %d %d %d\n", set_turnmode, cur_carstate.turnmode, turnmode_ctr, self_turn_ctr);
    if (turnmode_ctr == 0)
        return 0;
    // ROS_INFO("%d\n", turnmode_ctr);

    data_comm::car_ctr car_ctr;
    car_ctr.workmode = 3;
    car_ctr.speed = car_ctr.angle = 0;
    if (turnmode_ctr == 1) // 停止运动
    {
        car_ctr.turnmode = cur_carstate.turnmode;
        if (ctr_tmr.GetValue() > 0.2 && fabs(cur_carstate.speed[0] < 0.01))
            turnmode_ctr++, ctr_tmr.Clear(); // 等待停止
    }
    if (turnmode_ctr == 2) // 设置turnmode
    {
        car_ctr.turnmode = set_turnmode;
        // printf("2=%d %d %d\n", cur_carstate.turnmode, set_turnmode, car_ctr.turnmode);
        if (ctr_tmr.GetValue() > 4 && set_turnmode == cur_carstate.turnmode)
            turnmode_ctr = 0;
    }
    PubCarCtr(car_ctr);
    // carctr_pub.publish(car_ctr);
    return turnmode_ctr;
}

int TPathTrack::StopCtr() //  停车控制
{
    int stop_flag = StopCheck();
    if (stop_flag)
    {
        data_comm::car_ctr car_ctr;
        car_ctr.workmode = 2;
        car_ctr.turnmode = cur_carstate.turnmode;
        car_ctr.speed = car_ctr.angle = 0;
        PubCarCtr(car_ctr);
        // printf("stop_flag=%d ----car_ctr: %d\n", stop_flag, car_ctr.workmode);
        // carctr_pub.publish(car_ctr);
    }

    return stop_flag;
}

int TPathTrack::StopCheck() //  综合判断是否停车  1 障碍物 2 局部路径 3 运动使能 4 抱夹作业 5 雷达定位
{
    float mqtt_rate = 0;
    nh->getParam("/mqtt_comm/check/node_rate", mqtt_rate);
    float gps_rate = 0;
    nh->getParam("/gps_pro/check/node_rate", gps_rate);
    float can_rate = 0;
    nh->getParam("/can_comm/check/node_rate", can_rate);

    if (cur_task.cmd.find("pick") != string::npos && remain_path_length < 12)
        white_obs_flag = true;
    else if (cur_task.cmd.find("release") != string::npos && remain_path_length < 12)
        white_obs_flag = true;
    else
        white_obs_flag = false;

    // ROS_INFO("white obs=%d", white_obs_flag);

    static TTimer none_obs_tmr, none_gps_tmr, none_mqtt_tmr, none_can_tmr;
    if (obstacle_speedlimit < 0.01 && !white_obs_flag)
        none_obs_tmr.Clear();
    if (gps_rate < 5)
        none_gps_tmr.Clear();
    if (mqtt_rate < 0.2)
        none_mqtt_tmr.Clear();
    if (can_rate < 10)
        none_can_tmr.Clear();

    // printf("%.2f\n",can_rate);
    string stop_str = "";
    for (int i = 0; i < 16; i++)
        SetBit(stop_code, i, 0);

    // ROS_INFO("%d", localpath.poses.size());
    if (localpath.poses.size() == 0) //  无有效路径
    {
        stop_str = "no path", SetBit(stop_code, 0, 1);
        float task_dt = (ros::Time::now() - cur_task.stamp).toSec();
        if (task_dt > 4)
            cur_task.cmd = "";
    }

    if (!run_enable) //  自动导航断使能
        stop_str = "run disable", SetBit(stop_code, 1, 1);

    // 夹爪作业
    if (paw_state == "car_stop" || paw_state == "length_change_start" || paw_state == "length_change_done" || paw_state == "baojia_start" || (paw_state == "paw_dropping" && remain_path_length < 6))
        stop_str = "paw working", SetBit(stop_code, 2, 1);

    if (lidar_location_ctr > 1) //  转台扫描作业
        stop_str = "turntable working", SetBit(stop_code, 3, 1);

    if (none_can_tmr.GetValue() < 3) //  CAN通讯断开
        stop_str = "can lost", SetBit(stop_code, 4, 1);

    if (none_mqtt_tmr.GetValue() < 10 && mqtt_stop_enable) //  mqtt通讯断开
        stop_str = "mqtt lost", SetBit(stop_code, 5, 1);

    if (none_gps_tmr.GetValue() < 3) //  gps无固定解
        stop_str = "gps lost", SetBit(stop_code, 6, 1);

    if (none_obs_tmr.GetValue() < 3 && obs_stop_enable && !white_obs_flag) //  障碍物
        stop_str = "obstacle", SetBit(stop_code, 7, 1);

    if (traffic_speedlimit < 0.01 && traffic_stop_enable) //  交通管制
        stop_str = "traffic", SetBit(stop_code, 8, 1);

    if (cur_carstate.workmode == 1) // 急停按钮
        stop_str = "emc stop", SetBit(stop_code, 9, 1);

    if (cur_carstate.ctrmode == 0) //  手动按钮
        stop_str = "manual", SetBit(stop_code, 10, 1);

    // printf("%d\n", cur_carstate.ctrmode);

    if (stop_str != "")
    {
        nh_local->setParam("stop_reason", stop_str);
        return 1;
    }
    else
        return 0;
}

float GetVelByDistnce(float max_d, float max_vel, float min_d, float min_vel, float cur_d)
{
    if(cur_d>=max_d)  return max_vel;
    else if(cur_d<=min_d)  return min_vel;
    else
    {
        float v=min_vel+(cur_d-min_d)/(max_d-min_d)*(max_vel-min_vel);
        return v;
    }
}

void TPathTrack::CheckSpeedLimit()
{
    // nh_local->getParam("/lidar_radar_obs/speedlimit", obstacle_speedlimit);
    float obs_dis_laser = 999, obs_dis_lidar = 999, obs_dis;
    nh_local->getParam("/laser_radar_obs/obstacle_dis", obs_dis_laser);
    nh_local->getParam("/lidar_radar_obs/obstacle_dis", obs_dis_lidar);
    obs_dis = min(obs_dis_laser, obs_dis_lidar);
    obstacle_speedlimit = GetVelByDistnce(15, 3, 2.5, 0, obs_dis);

    nh_local->getParam("/traffic_ctr/speedlimit", traffic_speedlimit);

    speedlimit = 999;
    if (speedlimit >= obstacle_speedlimit && !white_obs_flag && obs_stop_enable)
        speedlimit = obstacle_speedlimit;
    if (speedlimit >= traffic_speedlimit && traffic_stop_enable)
        speedlimit = traffic_speedlimit;
    if (speedlimit >= mqtt_speedlimit)
        speedlimit = mqtt_speedlimit;
    if (speedlimit < 0.001)
        speedlimit = 0.001;

    // printf("speedlimit=%.2f obs_speedlimit=%.2f  mqtt_speedlimit=%.2f traffic_speedlimit=%.2f\n", speedlimit, obstacle_speedlimit, mqtt_speedlimit, traffic_speedlimit);
}

void TPathTrack::Run() //  主运行函数
{
    nodecheck->Find("node_rate")->Beat();

    CheckMoveMode();
    bool in_flag = false;
    float wheel_err = 0;
    if (remain_path_length < 7 && cur_task.cmd == "pick task") // 取车
    {
        // float wheel_err_front = wheel_err_front_external ,wheel_err_rear = wheel_err_rear_external;
        float wheel_err_front = wheel_err_front_internal, wheel_err_rear = wheel_err_rear_internal;
        // ROS_INFO("external:%f internal:%f\n", wheel_err_rear_external, wheel_err_rear_internal);
        if (ref_speed > 0 && fabs(wheel_err_front) < 1) // 正向
            in_flag = true, wheel_err = wheel_err_front;
        else if (ref_speed < 0 && fabs(wheel_err_rear) < 1) // 逆向
            in_flag = true, wheel_err = wheel_err_rear;
    }

    bool out_flag = false;
    if (passed_path_length < 7 && paw_state_msg.carhold_state != 2) // 退车
    {
        if (ref_speed > 0 && fabs(wheel_err_rear_internal) < 1) // 前进退车
            out_flag = true, wheel_err = -wheel_err_rear_internal;
        else if (ref_speed < 0 && fabs(wheel_err_front_internal) < 1) // 后退退车
            out_flag = true, wheel_err = -wheel_err_front_internal;
    }

    bool laser_enable = in_flag || out_flag;

    if (!test_flag)
    {
        nh_local->getParam("run_enable", run_enable);
        nh_local->getParam("mqtt_stop_enable", mqtt_stop_enable);
        nh_local->getParam("obs_stop_enable", obs_stop_enable);
        nh_local->getParam("traffic_stop_enable", traffic_stop_enable);
        nh_local->getParam("turn_speed_max", turn_speed_max);

        nh_local->getParam("/cloud_location_pro/location_ctr", lidar_location_ctr);
        nh_local->getParam("/pawcontrol/paw_state", paw_state);

        nh_local->setParam("stop_code", (int)stop_code);
        // printf("%x\n", stop_code);

        UpdateCtrParam(ref_speed);
        if (localpath.poses.size())
            aimpoint_pub.publish(aimpoint);

            // 控制云台是否扫描
        // if(localpath.poses.size())
        //     printf("%d   %d\n", localpath.poses.size(), ((GetYawFromPose(localpath.poses[0]) - shotpose_yaw) < 15));
        if (remain_path_length < 12 && cur_task.cmd == "pick task" && localpath.poses.size() &&(GetYawFromPose(localpath.poses[0]) - shotpose_yaw) < 15)
            if (ref_speed < 0)
            {
                lidar_work_enable = false;
                nh_local->setParam("/cloud_location_pro/lidar_work_enable", lidar_work_enable);
            }
            else if (ref_speed > 0)
            {
                lidar_work_enable = true;
                nh_local->setParam("/cloud_location_pro/lidar_work_enable", lidar_work_enable);
            }

        SetBit(stop_code, 16, 0);
        if (cur_task.cmd.find("task") != string::npos && !laser_enable && SelfTurnCtr())
        {
            nh_local->setParam("stop_reason", "self turning");
            SetBit(stop_code, 16, 1);
            return;
        }
        SetBit(stop_code, 17, 0);
        if (cur_task.cmd.find("task") != string::npos && MoveModeCtr())
        {
            nh_local->setParam("stop_reason", "movemode changing");
            SetBit(stop_code, 17, 1);
            return;
        }
        if (StopCtr())
            return;
    }

    // 纯跟踪控制
    data_comm::car_ctr car_ctr;
    car_ctr.turnmode = cur_carstate.turnmode;
    car_ctr.workmode = 3;
    car_ctr.speed = ref_speed;
    if (cur_task.cmd == "pick task" && remain_path_length < 7)
        car_ctr.angle = steering_property * (track_angle_err + orien_angle_err);
    else
        car_ctr.angle = steering_property * track_angle_err;

    if (laser_enable && laser_work_enable)
    {
        car_ctr.angle = -80 * wheel_err;
        float max_value = 3;
        if (car_ctr.angle < -max_value)
            car_ctr.angle = -max_value;
        else if (car_ctr.angle > max_value)
            car_ctr.angle = max_value;
        ROS_INFO("%.3f\n", wheel_err);
    }
    if (test_flag)
    {
        car_ctr.turnmode = 0;
        car_ctr.workmode = 3;
        car_ctr.angle = 0;
        // car_ctr.angle=-100*wheel_err;
        car_ctr.speed = test_speed;
    }
    PubCarCtr(car_ctr);

    if (fabs(ref_speed) > 0.01)
        nh_local->setParam("stop_reason", ""), SetBit(stop_code, 18, 0);
    else
        nh_local->setParam("stop_reason", "vel zero"), SetBit(stop_code, 18, 1);
    // printf("%.2f %.2f\n", car_ctr.angle, track_angle_err);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pathtrack");
    TPathTrack pathtrack;

    ros::Rate rate(50);
    while (ros::ok())
    {
        pathtrack.Run();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}