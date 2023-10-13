#include <data_comm/paw_ctr.h>
#include <data_comm/paw_state.h>
#include <data_comm/car_state.h>
#include <data_comm/car_ctr.h>
#include <common/public.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <mqtt_comm/task.h>
#include <common/mydisplay.h>

using namespace std;

/** 流程
01 wait_for_paw_drop
02 paw_drop_start
03 paw_dropping
04 paw_drop_done
05 back_wheel_reach
06 back_wheel_leave
07 front_wheel_reach
08 car_stop
09 length_change_start
10 length_change_done
11 baojia_start
12 baojia_done
13 car_drop_start
14 car_dropping
15 car_drop_done
**/

class PAW_CTR
{
private:
    string paw_ctr;
    double front_wheel_stop_distance;
    double back_wheel_stop_distance;

    data_comm::paw_ctr paw_ctr_msg;
    data_comm::paw_state paw_state_msg;
    data_comm::car_state car_state_msg;

    TDataFilter center1_y, center3_y, center2_y, center4_y, center1_x, center3_x, center2_x, center4_x, len1, len2, len3, len4;
    mqtt_comm::task cur_task;

    ros::NodeHandle *nh;

    ros::Subscriber scanlidar_pos_sub, paw_state_sub, car_state_sub, task_sub, remainpath_sub, carctr_sub;
    float remain_path_length=0;

    ros::Publisher paw_ctr_pub;
    bool back_flag=false;

    TNodeCheck *nodecheck;

public:
    void ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void PawStateCallback(const data_comm::paw_state::ConstPtr &msg);
    void CarStateCallback(const data_comm::car_state::ConstPtr &msg);
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void RemainPathCallback(const std_msgs::Float64::ConstPtr &msg);
    void CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg);

    void Run();

    PAW_CTR(): center1_x(2), center3_x(2), center2_x(2), center4_x(2), center1_y(2), center3_y(2), center2_y(2), center4_y(2), len1(2), len2(2), len3(2), len4(2)
    {
        nh = new ros::NodeHandle("~");

        nodecheck=new TNodeCheck(nh, "node_rate");
	    nodecheck->Find("node_rate")->SetLimit(15);

        // paw_state_str = "baojia_done";
        // paw_state_str = "wait_for_paw_drop";
        paw_ctr = "";
        nh->getParam("front_wheel_stop_distance", front_wheel_stop_distance);
        nh->getParam("back_wheel_stop_distance", back_wheel_stop_distance);

        scanlidar_pos_sub = nh->subscribe<std_msgs::Float32MultiArray>("/laserscan_check/wheel_distance", 10, &PAW_CTR::ScanPosCallback, this);
        paw_state_sub = nh->subscribe<data_comm::paw_state>("/can_comm/paw_state", 10, &PAW_CTR::PawStateCallback, this);
        car_state_sub = nh->subscribe<data_comm::car_state>("/can_comm/car_state", 10, &PAW_CTR::CarStateCallback, this);
        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &PAW_CTR::TaskCallback, this);
        remainpath_sub = nh->subscribe<std_msgs::Float64>("/local_path_plan/remainpath", 10, &PAW_CTR::RemainPathCallback, this);
        carctr_sub = nh->subscribe<data_comm::car_ctr>("/pathtrack/ctr_cmd", 10, &PAW_CTR::CarCtrCallback, this);

        paw_ctr_pub = nh->advertise<data_comm::paw_ctr>("/pawcontrol/paw_ctr", 10);

        paw_ctr_msg.left_paw_distance_control = 0;
        paw_ctr_msg.left_paw_speed = 0;
        paw_ctr_msg.right_paw_distance_control = 0;
        paw_ctr_msg.right_paw_speed = 0;
        paw_ctr_msg.paw_lift_control = 0;
        paw_ctr_msg.vehicle_put_control = 0;

        paw_state_msg.updown_state = 0;
        paw_state_msg.carhold_state = 0;
        paw_state_msg.left_axischange_state = 0;
        paw_state_msg.right_axischange_state = 0;
    }
};

void PAW_CTR::ScanPosCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    center1_x.GetValue(msg->data[0]),  center1_y.GetValue(msg->data[1]),  len1.GetValue(msg->data[2]);
    center2_x.GetValue(msg->data[3]),  center2_y.GetValue(msg->data[4]),  len2.GetValue(msg->data[5]);
    center3_x.GetValue(msg->data[6]),  center3_y.GetValue(msg->data[7]),  len3.GetValue(msg->data[8]);
    center4_x.GetValue(msg->data[9]),  center4_y.GetValue(msg->data[10]), len4.GetValue(msg->data[11]);

    // printf("%.2f %.2f %.2f %.2f\n", len1.value, len2.value, len3.value, len4.value);
}

void PAW_CTR::PawStateCallback(const data_comm::paw_state::ConstPtr &msg)
{
    paw_state_msg = *msg;
}

void PAW_CTR::CarStateCallback(const data_comm::car_state::ConstPtr &msg)
{
    car_state_msg = *msg;
}

//  接收任务指令
void PAW_CTR::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("task")!=string::npos)
    {
        cur_task = *msg;
        if (msg->cmd == "pick task")  paw_ctr = "wait_for_paw_drop"; //"wait_for_paw_drop";  //"paw_drop_done";
        else if (msg->cmd == "release task")  paw_ctr = "baojia_done";    
    }
    else if (cur_task.cmd == "taskStatus ctrl" && cur_task.subcmd=="0")
    {
        cur_task.cmd="";
    }
}

void PAW_CTR::RemainPathCallback(const std_msgs::Float64::ConstPtr &msg)
{
    remain_path_length=msg->data;
}

void PAW_CTR::CarCtrCallback(const data_comm::car_ctr::ConstPtr &msg) // 接收控制信号
{
    back_flag=(msg->speed<0);
}

/*
传感器配置
3------2
|      |--->车头朝向
1------4
*/
void PAW_CTR::Run()
{
    float time_spend = ros::Time::now().toSec() - cur_task.stamp.toSec();
    float drop_length = 10;

    // printf("%.2f\n",remain_path_length);

    // 取车夹爪控制状态机
    if (cur_task.cmd == "pick task" && car_state_msg.ctrmode)
    {
        if (paw_ctr == "wait_for_paw_drop" && time_spend > 4 && remain_path_length < drop_length)
        {
            paw_ctr_msg.paw_lift_control = 2;
            paw_ctr = "paw_dropping";
            paw_ctr_msg.vehicle_put_control = 0; // 0x00无动作
        }
        else if (paw_ctr == "paw_dropping" && paw_state_msg.updown_state == 2)
        {
            paw_ctr_msg.paw_lift_control = 0;
            paw_ctr = "paw_drop_done";
        }
        else if (paw_ctr == "paw_drop_done" && remain_path_length<1 && center2_x.value+center4_x.value>0)
        {
            float len=0.5*(len2.value+len4.value);
            if(len>0.1)
            {
                float center_y=0.5*(center2_y.value+center4_y.value);
                if (center_y>-front_wheel_stop_distance && !back_flag)  paw_ctr = "car_stop";
                else if(center_y<front_wheel_stop_distance+0.02 && back_flag)  paw_ctr = "car_stop";
            }
        }
        else if (paw_ctr == "car_stop" && fabs(car_state_msg.speed[0]) < 0.01)
        {
            paw_ctr = "length_change_start";
            // ROS_INFO("STOP:%.3f %.3f", center2_y.value, center4_y.value);
        }
        else if (paw_ctr == "length_change_start") // TODO 分段
        {
            paw_ctr_msg.left_paw_speed = 0;
            float center_y = 0.5 * (center1_y.value + center3_y.value);
            if(fabs(center_y)>5)  paw_ctr_msg.left_paw_speed = 3; // 高速
            else paw_ctr_msg.left_paw_speed = 1; // 低速

            // 方向设置
            if (center_y > back_wheel_stop_distance)  paw_ctr_msg.left_paw_distance_control = 1; // 增加
            else if (center_y < -back_wheel_stop_distance)  paw_ctr_msg.left_paw_distance_control = 2; // 缩短
            else paw_ctr_msg.left_paw_distance_control = 0;

            paw_ctr_msg.right_paw_distance_control = 0; // TODO
            paw_ctr_msg.right_paw_speed = 0;

            float len=0.5*(len1.value+len3.value);
            if (paw_ctr_msg.left_paw_distance_control==0 && len>0.2) paw_ctr="length_change_done";
        }
        else if (paw_ctr == "length_change_done")
        {
            if (paw_state_msg.left_axischange_state==0 || paw_state_msg.left_axischange_state==2 || paw_state_msg.left_axischange_state==4)
            {
                paw_ctr_msg.vehicle_put_control = 0xAA; // 0xAA自动取车
                paw_ctr = "baojia_start";
            }
        }
        else if (paw_ctr == "baojia_start" && paw_state_msg.carhold_state == 2)
        {
            paw_ctr = "baojia_done";
            paw_ctr_msg.vehicle_put_control = 0; // 0x00无动作
        }
    }
    else if (cur_task.cmd == "release task" && car_state_msg.ctrmode) // 放车夹爪控制状态机
    {
        if (paw_ctr == "baojia_done" && time_spend > 5 && remain_path_length < 0.05 && fabs(car_state_msg.speed[0]) < 0.01)
        {
            paw_ctr_msg.vehicle_put_control = 0xBB; // 0xBB自动放车
            paw_ctr = "car_dropping";
        }
        else if (paw_ctr == "car_dropping" && paw_state_msg.carhold_state == 4)
        {
            paw_ctr = "wait_for_paw_drop";         //"car_drop_done";
            paw_ctr_msg.vehicle_put_control = 0;   // 0x00无动作
        }
    }
    else if(cur_task.cmd=="") 
    {
        paw_ctr_msg.vehicle_put_control = 0; // 0x00无动作
        paw_ctr_msg.paw_lift_control = 0; 
        paw_ctr = "paw_idle";
    }

    // printf("paw=%s\n", paw_state_str.c_str());
    nh->setParam("paw_state", paw_ctr);
    paw_ctr_pub.publish(paw_ctr_msg);

    nodecheck->Find("node_rate")->Beat();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pawcontrol");

    PAW_CTR pawctr;
    ros::Rate looprate(20);
    while (ros::ok())
    {
        pawctr.Run();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};