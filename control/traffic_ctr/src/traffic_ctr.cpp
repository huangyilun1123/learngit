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
#include <visualization_msgs/MarkerArray.h>

#include <mqtt_comm/task.h>
#include <common/mydisplay.h>

using namespace std;

double utm_x_zero, utm_y_zero;
nav_msgs::Path localpath;

class Traffic_Area
{
    public:
        string caption;
        visualization_msgs::MarkerArray markers;
        float length, width;

        ros::NodeHandle *nh;
        ros::Publisher markers_pub;

        int ID=0;
        bool Inside_Flag=false;
        float Distance_Stop=0;
        int lightstatus=0;
        bool according=false;

        Traffic_Area(int id, string str)
        {
            ID=id;
            caption=str;
            nh = new ros::NodeHandle("~");
            char topic_name[100];
            sprintf(topic_name, "markers%d",ID);
            markers_pub = nh->advertise<visualization_msgs::MarkerArray>(topic_name, 10);
        };

        bool CheckRunHeading()
        {
            float run_angle;
            if(localpath.poses.size()<1)  
            {
                geometry_msgs::PoseStamped p_base, p_map;
                p_base.header.stamp=ros::Time::now();
                p_base.header.frame_id="base_link";
                p_base.pose.orientation.w=1;
                transformPose("map", p_base, p_map, "traffic");
                run_angle=GetYawFromPose(p_map)*180/M_PI;
            }
            else
            {
                run_angle=GetAngleByPoints(localpath.poses.front().pose.position, localpath.poses.back().pose.position);
                run_angle*=180.0/M_PI;
            }

            geometry_msgs::PoseStamped area_pose;
            area_pose.pose=markers.markers[0].pose;
            area_pose.header=markers.markers[0].header;
            float area_angle=GetYawFromPose(area_pose)*180/M_PI;

            // printf("%.1f %.1f\n", run_angle, area_angle);
            return fabs(run_angle-area_angle)<15;

        };

        void CheckInside()
        {
            geometry_msgs::PoseStamped center_p, stop_p;
            center_p.header=markers.markers[0].header;
            center_p.pose=markers.markers[0].pose;
            stop_p=GetExtendPoseByPose(center_p, 0.5*length);

            geometry_msgs::PoseStamped p_base;
            transformPose("base_link", center_p, p_base,"traffic");
            float angle_deg=GetYawFromPose(p_base)*180/M_PI;
            Inside_Flag=fabs(p_base.pose.position.y)<0.5*width && fabs(p_base.pose.position.x)<0.5*length && CheckRunHeading(); // && fabs(angle_deg)<15;
            if(Inside_Flag)
            {
                transformPose("base_link", stop_p, p_base, "traffic");
                Distance_Stop=fabs(p_base.pose.position.x);
                
                for(auto &m:markers.markers) 
                    if(lightstatus)  m.color.r=m.color.b=0.0f,  m.color.g=1.0f;
                    else  m.color.r=1.0f, m.color.b=m.color.g=0.0f; 
            }
            else  
            {
                Distance_Stop=999;
                lightstatus=0;
                for(auto &m:markers.markers)  m.color.r=m.color.g=m.color.b=1.0f;
            } 
        }       

        void GetParamFromPoints(vector<geometry_msgs::Point> points, float angle_deg)
        {
            float sum_x=0, sum_y=0;
            for(auto &p:points) 
            {
                p.x-=utm_x_zero, p.y-=utm_y_zero;
                sum_x+=p.x,  sum_y+=p.y;
            }
            float d1=GetDistanceXY(points[0], points[1]);
            float d2=GetDistanceXY(points[0], points[3]);
            if(d1>d2)  width=d2, length=d1;
            else width=d1, length=d2;
            // printf("%.0f %.0f\n",utm_x_zero, utm_y_zero);

            markers.markers.clear();
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "traffic_marker";
            marker.lifetime = ros::Duration(0.2);
            marker.frame_locked = true;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.2f;
            marker.pose.position.x=sum_x*0.25;
            marker.pose.position.y=sum_y*0.25;
            marker.pose.orientation=tf::createQuaternionMsgFromYaw((angle_deg+90)/180.0*M_PI);
            marker.scale.x = length;
            marker.scale.y = width;
            marker.scale.z = 0.2;
            marker.id = 0;
            markers.markers.push_back(marker);

            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation = tf::createQuaternionMsgFromYaw((angle_deg + 90) / 180.0 * M_PI);
            marker.color.a = 0.4f;
            marker.scale.x = 0.5*length;
            marker.scale.y = 1;
            marker.id = 1;
            markers.markers.push_back(marker);

            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.color.a = 1.0f;
            marker.scale.z = 2;
            marker.pose.position.z = 2;
            marker.text = caption;
            marker.id = 2;
            markers.markers.push_back(marker);
        }

        void Publish()
        {
            // for(auto m:markers.markers)  m.header.stamp=ros::Time::now();
            markers_pub.publish(markers);
         }
};


class Traffic_Ctr
{
private:
    mqtt_comm::task cur_task;
    ros::NodeHandle *nh;

    ros::Subscriber task_sub;
    vector<Traffic_Area> traffic_areas;

    TNodeCheck *nodecheck;

public:
    string traffic_state="none";
    void TaskCallback(const mqtt_comm::task::ConstPtr &msg);
    void Run();
    void LoadTrafficArea();

    Traffic_Ctr() 
    {
        nh = new ros::NodeHandle("~");

        nodecheck=new TNodeCheck(nh, "node_rate");
	    nodecheck->Find("node_rate")->SetLimit(15);

        task_sub = nh->subscribe<mqtt_comm::task>("/task_cmd", 10, &Traffic_Ctr::TaskCallback, this);
        LoadTrafficArea();
    }
};


void Traffic_Ctr::LoadTrafficArea()
{
    for (int i = 0; i < 100; i++)
    {
        char paramname[100];
        sprintf(paramname,"traffic_area%d",i);
        if(!nh->hasParam(paramname))  break;

        string caption;
        sprintf(paramname, "traffic_area%d/caption", i);
        nh->getParam(paramname, caption);
        Traffic_Area ta(i, caption);
        float value;
        vector<geometry_msgs::Point> points;
        geometry_msgs::Point p;
        for(int j=0;j<4;j++)
        {
            sprintf(paramname, "traffic_area%d/p%d_x", i, j);
            nh->getParam(paramname, value),  p.x=value;
            sprintf(paramname, "traffic_area%d/p%d_y", i, j);
            nh->getParam(paramname, value), p.y=value;
            points.push_back(p); 
        }
        sprintf(paramname, "traffic_area%d/angle", i);
        nh->getParam(paramname, value);
        ta.GetParamFromPoints(points, value);
        traffic_areas.push_back(ta);
    }
    // printf("nmu=%d\n", traffic_areas.size());
}

//  接收任务指令
void Traffic_Ctr::TaskCallback(const mqtt_comm::task::ConstPtr &msg)
{
    if (msg->cmd.find("ctrl")==string::npos)  return;

    cur_task = *msg;

    if(cur_task.cmd=="trafficLight ctrl")
    {
        if(cur_task.subcmd=="red")  traffic_state="red_signal";
        else if(cur_task.subcmd=="green")  traffic_state="green_signal";  
    }
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

void Traffic_Ctr::Run()
{
    nodecheck->Find("node_rate")->Beat();

    float dis_stop=999;
    bool inside_flag=false;
    for(auto &area:traffic_areas)
    {
        area.CheckInside();
        area.Publish();

        if(area.Inside_Flag)  
        {
            dis_stop=area.Distance_Stop;
            inside_flag=true;
            if(traffic_state=="green_signal")  area.lightstatus=1;
            else area.lightstatus=0;
            // ROS_INFO("to stop:%.2f", area.Distance_Stop);
        }
    }

    if(!inside_flag)  traffic_state="none";
    else if(traffic_state=="none" && inside_flag)  traffic_state="wait_signal";

    if(traffic_state=="none" || traffic_state=="green_signal")  dis_stop=999;

    //  设置速度限制
    float speedlimit=999;
    if(dis_stop<10)  speedlimit=GetVelByDistnce(10, 1, 4, 0, dis_stop);

    nh->setParam("speedlimit", speedlimit);
    nh->setParam("traffic_state", traffic_state);
    // printf("%s  vel=%.2f stop_dis=%.1f\n", traffic_state.c_str(), speedlimit, dis_stop);
}


void LocalPathCallback(const nav_msgs::Path::ConstPtr &msg) // 接收局部路径
{
    localpath = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "traffic_ctr");
    usleep(100000);
    ros::NodeHandle nh("~");
    nh.getParam("/gps_base/utmx_zero", utm_x_zero);
    nh.getParam("/gps_base/utmy_zero", utm_y_zero);
    ros::Subscriber localpath_sub = nh.subscribe<nav_msgs::Path>("/local_path_plan/localpath", 10, LocalPathCallback);

    Traffic_Ctr traffic_ctr;
    ros::Rate looprate(20);
    while (ros::ok())
    {
        traffic_ctr.Run();

        ros::spinOnce();
        looprate.sleep();
    }
    return 0;
};