
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iterator>
#include <ctime>
#include <vector>
#include <set>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/io/file_io.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

vector<vector<double>> mypoints(0, vector<double>(3)); //集
bool loop_closure_being_processed = false;

ros::Publisher pcl_pub;

void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts);
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts);
/*
int mainpoint2pclFile(int argc, char **argv);
int mainPub(int argc, char **argv);*/

/*
int mainxx(int argc, char **argv)
{
    if (argc > 1)
        mainpoint2pclFile(argc, argv);
    else
    {
        mainPub(argc, argv);
    }

    return 0;
}
int mainPub(int argc, char **argv)
{
    ros::init(argc, argv, "orbslam");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/orbslam2_points/output", 10);

    string name = "pts.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;
    //pcl::io::loadPCDFile("pts.pcd", cloud);
    pcl::PCDReader p;
    p.read (name, cloud,0);
    pcl::toROSMsg(cloud, output); // 转换成ROS下的数据类型 最终通过topic发布

    output.header.stamp = ros::Time::now();
    output.header.frame_id = "camera_rgb_frame";

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}*/
/*
--------------------- 
版权声明：本文为CSDN博主「熊猫飞天」的原创文章，遵循CC 4.0 by-sa版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/crp997576280/article/details/74605766*/

#define min3v(v1, v2, v3) ((v1) > (v2) ? ((v2) > (v3) ? (v3) : (v2)) : ((v1) > (v3) ? (v3) : (v2)))
#define max3v(v1, v2, v3) ((v1) < (v2) ? ((v2) < (v3) ? (v3) : (v2)) : ((v1) < (v3) ? (v3) : (v1)))

typedef struct
{
    int red;   // [0,255]
    int green; // [0,255]
    int blue;  // [0,255]
} COLOR_RGB;

typedef struct
{
    float hue;        // [0,360]
    float saturation; // [0,100]
    float luminance;  // [0,100]
} COLOR_HSL;

uint32_t HSLtoRGB(float h)
{
   // float h = hsl->hue;                // h must be [0, 360]
    float s = 0.8; // s must be [0, 1]
    float l = 0.8;  // l must be [0, 1]
    float R, G, B;
    
        float q = (l < 0.5f) ? (l * (1.0f + s)) : (l + s - (l * s));
        float p = (2.0f * l) - q;
        float Hk = h / 360.0f;
        float T[3];
        T[0] = Hk + 0.3333333f; // Tr   0.3333333f=1.0/3.0
        T[1] = Hk;              // Tb
        T[2] = Hk - 0.3333333f; // Tg
        for (int i = 0; i < 3; i++)
        {
            if (T[i] < 0)
                T[i] += 1.0f;
            if (T[i] > 1)
                T[i] -= 1.0f;
            if ((T[i] * 6) < 1)
            {
                T[i] = p + ((q - p) * 6.0f * T[i]);
            }
            else if ((T[i] * 2.0f) < 1) //(1.0/6.0)<=T[i] && T[i]<0.5
            {
                T[i] = q;
            }
            else if ((T[i] * 3.0f) < 2) // 0.5<=T[i] && T[i]<(2.0/3.0)
            {
                T[i] = p + (q - p) * ((2.0f / 3.0f) - T[i]) * 6.0f;
            }
            else
                T[i] = p;
        }
        R = T[0] * 255.0f;
        G = T[1] * 255.0f;
        B = T[2] * 255.0f;
    
    COLOR_RGB rgb;
    rgb.red = (int)((R > 255) ? 255 : ((R < 0) ? 0 : R));
    rgb.green = (int)((G > 255) ? 255 : ((G < 0) ? 0 : G));
    rgb.blue = (int)((B > 255) ? 255 : ((B < 0) ? 0 : B));
    return (rgb.red<<16)+(rgb.green<<8)+rgb.blue;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "savePcl");
    ros::start();

    ros::NodeHandle nodeHandler;
    ros::Subscriber sub_all_kf_and_pts = nodeHandler.subscribe("all_kf_and_pts", 1000, loopClosingCallback);
    pcl_pub = nodeHandler.advertise<sensor_msgs::PointCloud2>("/orbslam2_points/output", 10);
    ros::spin();
    ros::shutdown();

    return 0;
}

void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts)
{
    cout << "loopClosingCallback" << time(NULL) << endl;
    loop_closure_being_processed = true;
    resetGridMap(all_kf_and_pts);
    loop_closure_being_processed = false;
}
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts)
{

    unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
    if ((unsigned int)(all_kf_and_pts->poses[0].position.y) != n_kf ||
        (unsigned int)(all_kf_and_pts->poses[0].position.z) != n_kf)
    {
        printf("resetGridMap :: Unexpected formatting in the keyframe count element\n");
        return;
    }
    printf("Resetting grid map with %d key frames\n", n_kf);

    mypoints.clear();
    unsigned int id = 0;
    for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id)
    {
        const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
        unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;

        for (int cnt = id + 1; cnt < (id + n_pts - 1); cnt++)
        {
            mypoints.push_back({all_kf_and_pts->poses[cnt].position.x, all_kf_and_pts->poses[cnt].position.y, all_kf_and_pts->poses[cnt].position.z});
        }
        cout << "push_back" << n_pts << "size:" << mypoints.size() << endl;
        id += n_pts;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud; // 创建点云（不是指针）

    //填充点云数据
    cloud.width = mypoints.size();        //设置点云宽度
    cloud.height = 1;                     //设置点云高度
    cloud.is_dense = false;               //非密集型
    cloud.points.resize(mypoints.size()); //变形，无序
        //设置这些点的坐标

    ofstream fout;
    fout.open("pts" + to_string(time(NULL)) + ".pcd");

    auto maxz = mypoints[0][1], minz = mypoints[0][1];
    for (size_t i = 0; i < mypoints.size(); ++i)
    {
        if (mypoints[i][1] > maxz)
            maxz = mypoints[i][1];
        if (mypoints[i][1] < minz)
            minz = mypoints[i][1];
    }
    for (size_t i = 0; i < mypoints.size(); ++i)
    {
        fout << mypoints[i][0] << ' ' << mypoints[i][1] << ' ' << mypoints[i][2] << ' ' << HSLtoRGB(360*(mypoints[i][1]-minz)/(maxz-minz)) << endl;
        cloud.points[i].x = mypoints[i][0];
        cloud.points[i].y = mypoints[i][1];
        cloud.points[i].z = mypoints[i][2];
    }
    fout.close();
    //string savepath = "~/test_pcd.pcd";
    //保存到PCD文件
    //pcl::io::savePCDFileASCII(savepath, cloud); //将点云保存到PCD文件中
    //pcl::io::savePCDFileBinary(savepath, cloud);
    std::cerr << "Saved " << mypoints.size() << " data points to test_pcd.pcd." << std::endl;
    //显示点云数据
    //for (size_t i = 0; i < cloud.points.size (); ++i)
    //  std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud, output); // 转换成ROS下的数据类型 最终通过topic发布

    output.header.stamp = ros::Time::now();
    output.header.frame_id = "camera_rgb_frame";

    pcl_pub.publish(output);

    //exit(0);
}
/*
--------------------- 
版权声明：本文为CSDN博主「xuezhisdc」的原创文章，遵循CC 4.0 by-sa版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/xuezhisdc/article/details/51012463
*/
