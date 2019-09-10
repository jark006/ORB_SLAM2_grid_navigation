//#include <Eigen/Dense>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iterator>
#include <ctime>
#include <vector>
#include <set>
#include <math.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/core.hpp>

#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <Converter.h>

using namespace std;


const double PI = 3.14159265358979323846;  /* pi */

// parameters
float scale_factor = 3;
float resize_factor = 5;
float cloud_max_x = 10;
float cloud_min_x = -10.0;
float cloud_max_z = 16;
float cloud_min_z = -5;
float free_thresh = 0.55;
float occupied_thresh = 0.50;
float thresh_diff = 0.01;
int visit_thresh = 0;
float upper_left_x = -1.5;
float upper_left_y = -2.5;
const int resolution = 10;
unsigned int use_local_counters = 0;

float grid_max_x, grid_min_x, grid_max_z, grid_min_z;
cv::Mat global_occupied_counter, global_visit_counter;
cv::Mat local_occupied_counter, local_visit_counter;
cv::Mat local_map_pt_mask;
cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized, grid_map_proba;
;
float norm_factor_x, norm_factor_z;
int h, w;
unsigned int n_kf_received;
bool loop_closure_being_processed = false;
ros::Publisher pub_grid_map;
nav_msgs::OccupancyGrid grid_map_msg;

float kf_pos_x, kf_pos_y, kf_pos_z;
int kf_pos_grid_x = 0, kf_pos_grid_z = 0;

int g_camera_pos_grid_x = 0, g_camera_pos_grid_z = 0; //相机位置
int g_camera_ori_grid_x = 0, g_camera_ori_grid_z = 0; //相机方向位置
int g_target_x, g_target_z;
bool g_target_is_set = false;

std::string param_str;

vector<vector<double>> kf_plantVec(0, vector<double>(3));//关键帧平面的世界坐标集

vector<double> bestplane(3, 0);             //平面拟合 模型方程z=ax+by+c 法向量为[a,b,-1], [0]a [1]b [2]c
vector<double> idealNormalVec = {0, -1, 0};  //理想的平面法向量
vector<vector<double>> rotationMatrix = {
	{1, 0, 0},
	{0, 1, 0},
	{0, 0, 1},
};      //旋转矩阵


void updateGridMap(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose);
void resetGridMap(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose);
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pt_cloud);
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr &camera_pose);
void saveMap(unsigned int id = 0);
void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr &cur_camera_pose);
void ptCallback(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose);
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts);
void parseParams(int argc, char **argv);
void printParams();
void showGridMap(unsigned int id = 0);
void getMixMax(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose,
			   geometry_msgs::Point &min_pt, geometry_msgs::Point &max_pt);
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
				  cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z);
void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
				   unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z);
void getGridMap();
void onMouseHandle(int event, int x, int y, int flags, void *param);

vector<double> CrossProduct(vector<double>& a, vector<double>& b);
double DotProduct(vector<double>& a, vector<double>& b);
double Normalize(vector<double>& v);
void PlaneFittingRansac(vector<vector<double>>& Vectors, vector<double>& bestplane);
vector<vector<double>> CalRotationMatrix(vector<double>& vectorBefore, vector<double>& vectorAfter);
vector<double> RotationAjust(vector<vector<double>>& r, vector<double>& v);



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Monosub");
	ros::start();

	printf("Input %d params\n", argc - 1);
	parseParams(argc, argv);
	printParams();

	{
		const std::vector<float> params = {
			scale_factor, resize_factor, cloud_max_x, cloud_min_x,
			cloud_max_z, cloud_min_z, free_thresh, occupied_thresh,
			(float)use_local_counters, (float)visit_thresh};
		std::ostringstream oss;
		std::copy(params.cbegin(), params.cend(), ostream_iterator<float>(oss, "_"));
		param_str = oss.str();
	}


	grid_max_x = cloud_max_x * scale_factor;
	grid_min_x = cloud_min_x * scale_factor;
	grid_max_z = cloud_max_z * scale_factor;
	grid_min_z = cloud_min_z * scale_factor;
	printf("grid_max: %f, %f\t grid_min: %f, %f\n", grid_max_x, grid_max_z, grid_min_x, grid_min_z);

	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;

	h = grid_res_z;
	w = grid_res_x;
	printf("grid_size: (%d, %d)\n", h, w);
	n_kf_received = 0;

	global_occupied_counter.create(h, w, CV_32SC1);
	global_visit_counter.create(h, w, CV_32SC1);
	global_occupied_counter.setTo(cv::Scalar(0));
	global_visit_counter.setTo(cv::Scalar(0));

	grid_map_msg.data.resize(h * w);
	grid_map_msg.info.width = w;
	grid_map_msg.info.height = h;
	grid_map_msg.info.resolution = 1.0 / scale_factor;

	grid_map_int = cv::Mat(h, w, CV_8SC1, (char *)(grid_map_msg.data.data()));


	grid_map_proba.create(h, w, CV_8UC1);

	grid_map.create(h, w, CV_32FC1);
	grid_map_thresh.create(h, w, CV_8UC1);
	grid_map_thresh_resized.create(h * resize_factor, w * resize_factor, CV_8UC1);
	printf("output_size: (%d, %d)\n", grid_map_thresh_resized.rows, grid_map_thresh_resized.cols);

	local_occupied_counter.create(h, w, CV_32SC1);
	local_visit_counter.create(h, w, CV_32SC1);
	local_map_pt_mask.create(h, w, CV_8UC1);

	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
	norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
	printf("norm_factor_x: %f\n", norm_factor_x);
	printf("norm_factor_z: %f\n", norm_factor_z);

	showGridMap();


	ros::NodeHandle nodeHandler;
	ros::Subscriber sub_pts_and_pose = nodeHandler.subscribe("pts_and_pose", 1000, ptCallback);
	ros::Subscriber sub_all_kf_and_pts = nodeHandler.subscribe("all_kf_and_pts", 1000, loopClosingCallback);
	ros::Subscriber sub_cur_camera_pose = nodeHandler.subscribe("/cur_camera_pose", 1000, cameraPoseCallback);
	pub_grid_map = nodeHandler.advertise<nav_msgs::OccupancyGrid>("grid_map", 1000);

	cv::namedWindow("grid_map_thresh_resized", cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback("grid_map_thresh_resized", onMouseHandle);

	//ros::Subscriber sub_cloud = nodeHandler.subscribe("cloud_in", 1000, cloudCallback);
	//ros::Subscriber sub_kf = nodeHandler.subscribe("camera_pose", 1000, kfCallback);
	//ros::Subscriber sub = nodeHandler.subscribe("/camera/image_raw", 1, &ImageGrabber::GrabImage, &igb);

	ros::spin();
	ros::shutdown();
	cv::destroyAllWindows();
	saveMap(time(NULL));

	return 0;
}

void onMouseHandle(int event, int x, int y, int flags, void *param)
{
	switch (event)
	{
	case cv::EVENT_LBUTTONDOWN:
		g_target_x = static_cast<int>(x / resize_factor);
		g_target_z = static_cast<int>(h - y / resize_factor);
		if (g_target_x < 0 || g_target_x >= w ||
			g_target_z < 0 || g_target_z >= h)
		{
			g_target_is_set = false;
		}
		else
		{
			g_target_is_set = true;
		}
		//printf("onMouseHandle: Set target: %d, %d (Current: %d, %d)\n",
		//		int(g_target_x*resize_factor), int(g_target_z*resize_factor),
		//		int(g_camera_pos_grid_x*resize_factor), int(g_camera_pos_grid_z*resize_factor));
		break;
	}
}

void cameraPoseCallback(const geometry_msgs::Pose::ConstPtr &cur_camera_pose)
{
	const geometry_msgs::Point &location = cur_camera_pose->position;
	const geometry_msgs::Quaternion &orientation = cur_camera_pose->orientation;

	//const float camera_pos_x = location.x * scale_factor;
	//const float camera_pos_z = location.z * scale_factor;

	vector<double> curr_pt_before = {location.x, location.y, location.z};
	auto curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before); //旋转调整
	const float camera_pos_x = curr_pt_after[0] * scale_factor;
	const float camera_pos_z = curr_pt_after[2] * scale_factor;


	Eigen::Quaterniond q = Eigen::Quaterniond (orientation.w, orientation.x, orientation.y, orientation.z).normalized();
	Eigen::Vector3d p1 = Eigen::Vector3d(0, 0, 1);//世界坐标系正前方向，z正前，x左，y下

	Eigen::Vector3d p2 = q * p1;;//相机在世界坐标系的方向

	vector<double> p3 = {p2[0], p2[1], p2[2]};
	p3 = RotationAjust(rotationMatrix, p3);  //旋转调整

	

	const int camera_pos_grid_x = int(floor((camera_pos_x - grid_min_x) * norm_factor_x));
	const int camera_pos_grid_z = int(floor((camera_pos_z - grid_min_z) * norm_factor_z));

	if (camera_pos_grid_x < 0 || camera_pos_grid_x >= w ||
		camera_pos_grid_z < 0 || camera_pos_grid_z >= h)
		return;

	g_camera_pos_grid_x = camera_pos_grid_x;
	g_camera_pos_grid_z = camera_pos_grid_z;


	g_camera_ori_grid_x = g_camera_pos_grid_x + p3[0]*10;
	g_camera_ori_grid_z = g_camera_pos_grid_z + p3[2]*10;

	showGridMap(0);
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr &pt_cloud)
{
	ROS_INFO("I heard: [%s]{%d}", pt_cloud->header.frame_id.c_str(),
			 pt_cloud->header.seq);
}
void kfCallback(const geometry_msgs::PoseStamped::ConstPtr &camera_pose)
{
	ROS_INFO("I heard: [%s]{%d}", camera_pose->header.frame_id.c_str(),
			 camera_pose->header.seq);
}
void saveMap(unsigned int id)
{
	cv::Mat grid_map_int_fliped, grid_map_thresh_fliped, grid_map_thresh_resized_fliped;
	cv::flip(grid_map_int, grid_map_int_fliped, 0);
	cv::flip(grid_map_thresh, grid_map_thresh_fliped, 0);
	cv::flip(grid_map_thresh_resized, grid_map_thresh_resized_fliped, 0);

	printf("saving maps with id: %u\n", id);
	mkdir("results", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	if (id > 0)
	{
		cv::imwrite("results/grid_map_" + to_string(id) + "_" + param_str + ".png", grid_map_int_fliped);
		cv::imwrite("results/grid_map_thresh_" + to_string(id) + "_" + param_str + ".png", grid_map_thresh_fliped);
		cv::imwrite("results/grid_map_thresh_resized_" + to_string(id) + "_" + param_str + ".png", grid_map_thresh_resized_fliped);
	}
	else
	{
		cv::imwrite("results/grid_map_" + param_str + ".png", grid_map_int_fliped);
		cv::imwrite("results/grid_map_thresh_" + param_str + ".png", grid_map_thresh_fliped);
		cv::imwrite("results/grid_map_thresh_resized_" + param_str + ".png", grid_map_thresh_resized_fliped);
	}
}
void ptCallback(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose)
{
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	//if (pts_and_pose->header.seq==0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	printf("Received exit message\n");
	//	ros::shutdown();
	//	exit(0);
	//}
	//	if (!got_start_time) {
	//#ifdef COMPILEDWITHC11
	//		start_time = std::chrono::steady_clock::now();
	//#else
	//		start_time = std::chrono::monotonic_clock::now();
	//#endif
	//		got_start_time = true;
	//	}
	if (loop_closure_being_processed)
	{
		return;
	}

	updateGridMap(pts_and_pose);

	grid_map_msg.info.map_load_time = ros::Time::now();
	pub_grid_map.publish(grid_map_msg);
}
void loopClosingCallback(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts)
{
	//ROS_INFO("Received points and pose: [%s]{%d}", pts_and_pose->header.frame_id.c_str(),
	//	pts_and_pose->header.seq);
	//if (all_kf_and_pts->header.seq == 0) {
	//	cv::destroyAllWindows();
	//	saveMap();
	//	ros::shutdown();
	//	exit(0);
	//}
	
	loop_closure_being_processed = true;
	resetGridMap(all_kf_and_pts);
	loop_closure_being_processed = false;
}

void getMixMax(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose,
			   geometry_msgs::Point &min_pt, geometry_msgs::Point &max_pt)
{

	min_pt.x = min_pt.y = min_pt.z = std::numeric_limits<double>::infinity();
	max_pt.x = max_pt.y = max_pt.z = -std::numeric_limits<double>::infinity();
	for (unsigned int i = 0; i < pts_and_pose->poses.size(); ++i)
	{
		const geometry_msgs::Point &curr_pt = pts_and_pose->poses[i].position;
		if (curr_pt.x < min_pt.x)
		{
			min_pt.x = curr_pt.x;
		}
		if (curr_pt.y < min_pt.y)
		{
			min_pt.y = curr_pt.y;
		}
		if (curr_pt.z < min_pt.z)
		{
			min_pt.z = curr_pt.z;
		}

		if (curr_pt.x > max_pt.x)
		{
			max_pt.x = curr_pt.x;
		}
		if (curr_pt.y > max_pt.y)
		{
			max_pt.y = curr_pt.y;
		}
		if (curr_pt.z > max_pt.z)
		{
			max_pt.z = curr_pt.z;
		}
	}
}
void processMapPt(const geometry_msgs::Point &curr_pt, cv::Mat &occupied,
				  cv::Mat &visited, cv::Mat &pt_mask, int kf_pos_grid_x, int kf_pos_grid_z)
{
	vector<double> curr_pt_before = {curr_pt.x, curr_pt.y, curr_pt.z};
	auto curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before);

	/*cout << "processMapPt Before:";
	for(auto i:curr_pt_before)
		cout << i << ' ';
	cout << endl;
	
	cout << "After:";
	for(auto i:curr_pt_after)
		cout << i << ' ';
	cout << endl;*/
	
	
	//float pt_pos_x = curr_pt.x * scale_factor;
	//float pt_pos_y = curr_pt.y * scale_factor;
	//float pt_pos_z = curr_pt.z * scale_factor;

	float pt_pos_x = curr_pt_after[0] * scale_factor;
	float pt_pos_z = curr_pt_after[2] * scale_factor;


	int pt_pos_grid_x = int(floor((pt_pos_x - grid_min_x) * norm_factor_x));
	int pt_pos_grid_z = int(floor((pt_pos_z - grid_min_z) * norm_factor_z));

	//TODO jark
	//if(curr_pt.y > (kf_pos_y + 0.22))
	//	return;
	if(curr_pt_after[1] > (kf_pos_y + 0.18)) //高度过滤
		return;

	if (pt_pos_grid_x < 0 || pt_pos_grid_x >= w)
		return;

	if (pt_pos_grid_z < 0 || pt_pos_grid_z >= h)
		return;

	// Increment the occupency account of the grid cell where map point is located
	++occupied.at<int>(pt_pos_grid_z, pt_pos_grid_x);
	pt_mask.at<uchar>(pt_pos_grid_z, pt_pos_grid_x) = 255;

	//cout << "----------------------" << endl;
	//cout << okf_pos_grid_x << " " << okf_pos_grid_y << endl;

	// Get all grid cell that the line between keyframe and map point pass through
	/*int x0 = kf_pos_grid_x;
	int y0 = kf_pos_grid_z;
	int x1 = pt_pos_grid_x;
	int y1 = pt_pos_grid_z;
	bool steep = (abs(y1 - y0) > abs(x1 - x0));
	if (steep)
	{
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1)
	{
		swap(x0, x1);
		swap(y0, y1);
	}
	int dx = x1 - x0;
	int dy = abs(y1 - y0);
	double error = 0;
	double deltaerr = ((double)dy) / ((double)dx);
	int y = y0;
	int ystep = (y0 < y1) ? 1 : -1;
	for (int x = x0; x <= x1; ++x)
	{
		if (steep)
		{
			++visited.at<int>(x, y);
		}
		else
		{
			++visited.at<int>(y, x);
		}
		error = error + deltaerr;
		if (error >= 0.5)
		{
			y = y + ystep;
			error = error - 1.0;
		}
	}*/

	int x1 = kf_pos_grid_x;
	int y1 = kf_pos_grid_z;
	int x0 = pt_pos_grid_x;
	int y0 = pt_pos_grid_z;
	
	int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
  	int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
  	int err = (dx>dy ? dx : -dy)/2, e2;

	for(;;){
		++visited.at<int>(y0,x0);
		if (x0==x1 || y0==y1) break;
		e2 = err;
		if (e2 >-dx) { err -= dy; x0 += sx; }
		if (e2 < dy) { err += dx; y0 += sy; }
	}
	/* 
	来源：https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C
	详解：https://blog.csdn.net/cjw_soledad/article/details/78886117 
	*/

}

void processMapPts(const std::vector<geometry_msgs::Pose> &pts, unsigned int n_pts,
				   unsigned int start_id, int kf_pos_grid_x, int kf_pos_grid_z)
{
	unsigned int end_id = start_id + n_pts;
	if (use_local_counters)
	{
		local_map_pt_mask.setTo(0);
		local_occupied_counter.setTo(0);
		local_visit_counter.setTo(0);
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
		{
			processMapPt(pts[pt_id].position, local_occupied_counter, local_visit_counter,
						 local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
		for (int row = 0; row < h; ++row)
		{
			for (int col = 0; col < w; ++col)
			{
				if (local_map_pt_mask.at<uchar>(row, col) == 0)
				{
					local_occupied_counter.at<int>(row, col) = 0;
				}
				else
				{
					local_occupied_counter.at<int>(row, col) = local_visit_counter.at<int>(row, col);
				}
			}
		}
		global_occupied_counter += local_occupied_counter;
		global_visit_counter += local_visit_counter;
	}
	else
	{
		for (unsigned int pt_id = start_id; pt_id < end_id; ++pt_id)
		{
			processMapPt(pts[pt_id].position, global_occupied_counter, global_visit_counter,
						 local_map_pt_mask, kf_pos_grid_x, kf_pos_grid_z);
		}
	}
}

void updateGridMap(const geometry_msgs::PoseArray::ConstPtr &pts_and_pose)
{

	//geometry_msgs::Point min_pt, max_pt;
	//getMixMax(pts_and_pose, min_pt, max_pt);
	//printf("max_pt: %f, %f\t min_pt: %f, %f\n", max_pt.x*scale_factor, max_pt.z*scale_factor,
	//	min_pt.x*scale_factor, min_pt.z*scale_factor);

	//double grid_res_x = max_pt.x - min_pt.x, grid_res_z = max_pt.z - min_pt.z;

	//printf("Received frame %u \n", pts_and_pose->header.seq);

	const geometry_msgs::Point &kf_location = pts_and_pose->poses[0].position;
	//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;

	//TODO
	//std::cout << "XXX " << kf_location.x << ' ' << kf_location.y << ' ' << kf_location.z << endl;


	vector<double> curr_pt_before = {kf_location.x, kf_location.y, kf_location.z};
	vector<double> curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before);

	/*
	cout << "updateGridMap Before:";
	for(auto i:curr_pt_before)
		cout << i << ' ';
	cout << endl;
	
	cout << "After:";
	for(auto i:curr_pt_after)
		cout << i << ' ';
	cout << endl;
	*/

	kf_pos_x = curr_pt_after[0] * scale_factor;
	kf_pos_z = curr_pt_after[2] * scale_factor;

	//kf_pos_x = kf_location.x * scale_factor;
	//kf_pos_z = kf_location.z * scale_factor;

	kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
	kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

	if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
		return;

	if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
		return;
	++n_kf_received;
	unsigned int n_pts = pts_and_pose->poses.size() - 1;
	//printf("Processing key frame %u and %u points\n",n_kf_received, n_pts);
	processMapPts(pts_and_pose->poses, n_pts, 1, kf_pos_grid_x, kf_pos_grid_z);

	getGridMap();
	showGridMap(pts_and_pose->header.seq);
	//cout << endl << "Grid map saved!" << endl;
}

void resetGridMap(const geometry_msgs::PoseArray::ConstPtr &all_kf_and_pts)
{
	global_visit_counter.setTo(0);
	global_occupied_counter.setTo(0);

	unsigned int n_kf = all_kf_and_pts->poses[0].position.x;
	if ((unsigned int)(all_kf_and_pts->poses[0].position.y) != n_kf ||
		(unsigned int)(all_kf_and_pts->poses[0].position.z) != n_kf)
	{
		printf("resetGridMap :: Unexpected formatting in the keyframe count element\n");
		return;
	}
	printf("Resetting grid map with %d key frames\n", n_kf);
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

	kf_plantVec.clear();
	unsigned int id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id)
	{
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		id += n_pts;
		kf_plantVec.push_back({kf_location.x, kf_location.y, kf_location.z}); //获取所有关键帧点集
	}
	PlaneFittingRansac(kf_plantVec, bestplane);//拟合平面
	vector<double> bestNormalVec = {bestplane[0], bestplane[1], -1};
	auto angle = acos(DotProduct(bestNormalVec, idealNormalVec) / (Normalize(bestNormalVec)*Normalize(idealNormalVec))) * 180/PI; //Arc TO degree
	if(angle < 20)
	{
		//cout << "Angle between bese&ideal: " << angle << endl;
		rotationMatrix = CalRotationMatrix(bestNormalVec, idealNormalVec);	//计算旋转矩阵
	}
	else
	{
		bestNormalVec[0] = -bestNormalVec[0]; //反转法向量
		bestNormalVec[1] = -bestNormalVec[1];
		bestNormalVec[2] = -bestNormalVec[2];

		auto angle = acos(DotProduct(bestNormalVec, idealNormalVec) / (Normalize(bestNormalVec)*Normalize(idealNormalVec))) * 180/PI; //Arc TO degree
		
		if(angle < 20)
		{
			//cout << "Angle between bese&ideal: " << angle << endl;
			rotationMatrix = CalRotationMatrix(bestNormalVec, idealNormalVec);	//计算旋转矩阵
		}
		else
			cerr << "Angle ERROR!!!!: " << angle << endl;
	}
	


	//unsigned int id = 0;
	id = 0;
	for (unsigned int kf_id = 0; kf_id < n_kf; ++kf_id)
	{
		const geometry_msgs::Point &kf_location = all_kf_and_pts->poses[++id].position;
		//const geometry_msgs::Quaternion &kf_orientation = pts_and_pose->poses[0].orientation;
		unsigned int n_pts = all_kf_and_pts->poses[++id].position.x;
		if ((unsigned int)(all_kf_and_pts->poses[id].position.y) != n_pts ||
			(unsigned int)(all_kf_and_pts->poses[id].position.z) != n_pts)
		{
			printf("resetGridMap :: Unexpected formatting in the point count element for keyframe %d\n", kf_id);
			return;
		}


		vector<double> curr_pt_before = {kf_location.x, kf_location.y, kf_location.z};
		auto curr_pt_after = RotationAjust(rotationMatrix, curr_pt_before);

		float kf_pos_x = curr_pt_after[0] * scale_factor;
		float kf_pos_z = curr_pt_after[2] * scale_factor;

		//float kf_pos_x = kf_location.x * scale_factor;
		//float kf_pos_z = kf_location.z * scale_factor;

		int kf_pos_grid_x = int(floor((kf_pos_x - grid_min_x) * norm_factor_x));
		int kf_pos_grid_z = int(floor((kf_pos_z - grid_min_z) * norm_factor_z));

		if (kf_pos_grid_x < 0 || kf_pos_grid_x >= w)
			continue;

		if (kf_pos_grid_z < 0 || kf_pos_grid_z >= h)
			continue;

		if (id + n_pts >= all_kf_and_pts->poses.size())
		{
			printf("resetGridMap :: Unexpected end of the input array while processing keyframe %u with %u points: only %u out of %u elements found\n",
				   kf_id, n_pts, all_kf_and_pts->poses.size(), id + n_pts);
			return;
		}
		processMapPts(all_kf_and_pts->poses, n_pts, id + 1, kf_pos_grid_x, kf_pos_grid_z);
		id += n_pts;

		//TODO
		//std::cout << "XXX" << kf_location.x << ' ' << kf_location.y << ' ' << kf_location.z << endl;
	}
	getGridMap();
#ifdef COMPILEDWITHC11
	std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
	std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
	double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
	printf("Done. Time taken: %f secs\n", ttrack);
	pub_grid_map.publish(grid_map_msg);
	showGridMap(all_kf_and_pts->header.seq);
}

void getGridMap()
{
	for (int row = 0; row < h; ++row)
	{
		for (int col = 0; col < w; ++col)
		{
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

			if (visits <= visit_thresh)
			{
				grid_map.at<float>(row, col) = 0.5;
				//grid_map_proba.at<uchar>(row, col) = 128;
			}
			else
			{
				//grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
				grid_map.at<float>(row, col) = 1.0 - (1.0 * occupieds / visits);

				/*
				if(occupieds > 12)
					grid_map_proba.at<uchar>(row, col) = 0;
				else
					grid_map_proba.at<uchar>(row, col) = (uchar)(255 * (1.0 - 1.0 * occupieds / visits));
				*/
			}
			if (grid_map.at<float>(row, col) >= free_thresh)
			{
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh)
			{
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else
			{
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
			//grid_map_proba.at<uchar>(row, col) = (1 - grid_map.at<float>(row, col)) * 255;
		}
	}
	
	cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
}
void showGridMap(unsigned int id)
{
	//cv::imshow("grid_map_msg", cv::Mat(h, w, CV_8SC1, (char*)(grid_map_msg.data.data())));
	//cv::imshow("grid_map_thresh_resized", grid_map_thresh_resized);

	// Flip y axis to align coordinate bewteen image and map
	/*{
		cv::Mat dst;
		cv::flip(grid_map_int, dst, 0);
		cv::circle(dst, cv::Point(g_camera_pos_grid_x, h-g_camera_pos_grid_z), 2, cv::Scalar(128));
		cv::imshow("grid_map_msg", dst);
	}*/

	/*
	{
		cv::Mat dst;
		cv::flip(grid_map_proba, dst, 0); //上下翻转
		cv::imshow("grid_map_probability", dst);
	}
	*/

	{
		cv::Mat dst;
		cv::flip(grid_map_thresh_resized, dst, 0);
		cv::circle(dst, cv::Point(g_camera_pos_grid_x * resize_factor, (h - g_camera_pos_grid_z) * resize_factor), 2 * resize_factor, cv::Scalar(50));
		cv::line(dst, cv::Point(g_camera_pos_grid_x * resize_factor, (h - g_camera_pos_grid_z) * resize_factor), \
				cv::Point(g_camera_ori_grid_x * resize_factor, (h - g_camera_ori_grid_z) * resize_factor), cv::Scalar(60), 1, 8, 0);
		
		cv::circle(dst, cv::Point(g_target_x * resize_factor, (h - g_target_z) * resize_factor), 2 * resize_factor, cv::Scalar(0), cv::FILLED);

		cv::imshow("grid_map_thresh_resized", dst);
	}

	//cv::imshow("grid_map", grid_map);
	int key = cv::waitKey(1) % 256;
	if (key == 27)
	{
		cv::destroyAllWindows();
		ros::shutdown();
		exit(0);
	}
	else if (key == 'f')
	{
		free_thresh -= thresh_diff;
		if (free_thresh <= occupied_thresh)
		{
			free_thresh = occupied_thresh + thresh_diff;
		}

		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'F')
	{
		free_thresh += thresh_diff;
		if (free_thresh > 1)
		{
			free_thresh = 1;
		}
		printf("Setting free_thresh to: %f\n", free_thresh);
	}
	else if (key == 'o')
	{
		occupied_thresh -= thresh_diff;
		if (free_thresh < 0)
		{
			free_thresh = 0;
		}
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 'O')
	{
		occupied_thresh += thresh_diff;
		if (occupied_thresh >= free_thresh)
		{
			occupied_thresh = free_thresh - thresh_diff;
		}
		printf("Setting occupied_thresh to: %f\n", occupied_thresh);
	}
	else if (key == 's')
	{
		saveMap(id);
	}
}

void parseParams(int argc, char **argv)
{
	int arg_id = 1;
	if (argc > arg_id)
	{
		scale_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		resize_factor = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_max_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_min_x = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_max_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		cloud_min_z = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		free_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		occupied_thresh = atof(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		use_local_counters = atoi(argv[arg_id++]);
	}
	if (argc > arg_id)
	{
		visit_thresh = atoi(argv[arg_id++]);
	}
}

void printParams()
{
	printf("Using params:\n");
	printf("scale_factor: %f\n", scale_factor);
	printf("resize_factor: %f\n", resize_factor);
	printf("cloud_max: %f, %f\t cloud_min: %f, %f\n", cloud_max_x, cloud_max_z, cloud_min_x, cloud_min_z);
	//printf("cloud_min: %f, %f\n", cloud_min_x, cloud_min_z);
	printf("free_thresh: %f\n", free_thresh);
	printf("occupied_thresh: %f\n", occupied_thresh);
	printf("use_local_counters: %d\n", use_local_counters);
	printf("visit_thresh: %d\n", visit_thresh);
}





/*
平面拟合 方程z=ax+by+c 法向量为[a,b,-1]
vector<double> bestplane(3, 0);// [0]a [1]b [2]c
PlaneFittingRansac(Vectors, bestplane);
cout << "法向量为： x:" << bestplane[0] << " y:" << bestplane[1] << " z:" << -1 << endl;
 */

void PlaneFittingRansac(vector<vector<double>>& Vectors, vector<double>& bestplane)
{
	if (Vectors[0].size() != 3)
		return;

	cout << "data number:" << Vectors.size() << endl;

	int maxinliers = 0;
	srand((unsigned)time(0));
	//迭代30次
	for (int i = 0; i < 30; i++)
	{
		//步骤1：从数据中随机选择3组数据
		//
		set<int> s;
		while (1)
		{
			int r = rand() % (Vectors.size());
			s.insert(r);
			if (s.size() >= 3)
				break;
		}
		int j = 0;
		int select_num[3] = { 0 };
		for (auto iElement = s.cbegin(); iElement != s.end(); iElement++)
		{
			//cout<<*iElement<<" ";
			select_num[j++] = *iElement;
		}
		//cout << select_num[0] << "," << select_num[1] << "," << select_num[2] << endl;
		//cout << "data1:" << Vectors[select_num[0]][0] << "," << Vectors[select_num[0]][1] << "," << Vectors[select_num[0]][2] << endl;
		//cout << "data2:" << Vectors[select_num[1]][0] << "," << Vectors[select_num[1]][1] << "," << Vectors[select_num[1]][2] << endl;
		//cout << "data3:" << Vectors[select_num[2]][0] << "," << Vectors[select_num[2]][1] << "," << Vectors[select_num[2]][2] << endl;
		//步骤2：通过获得的数据获得模型参数
		double a = ((Vectors[select_num[0]][2] - Vectors[select_num[2]][2])*(Vectors[select_num[1]][1] - Vectors[select_num[2]][1]) - (Vectors[select_num[1]][2] - Vectors[select_num[2]][2])*(Vectors[select_num[0]][1] - Vectors[select_num[2]][1])) / \
			((Vectors[select_num[0]][0] - Vectors[select_num[2]][0])*(Vectors[select_num[1]][1] - Vectors[select_num[2]][1]) - (Vectors[select_num[1]][0] - Vectors[select_num[2]][0])*(Vectors[select_num[0]][1] - Vectors[select_num[2]][1]));
		double b = ((Vectors[select_num[1]][2] - Vectors[select_num[2]][2]) - a * (Vectors[select_num[1]][0] - Vectors[select_num[2]][0])) / \
			(Vectors[select_num[1]][1] - Vectors[select_num[2]][1]);
		double c = Vectors[select_num[0]][2] - a * Vectors[select_num[0]][0] - b * Vectors[select_num[0]][1];
		//cout << "a:" << a << ",b:" << b << ",c:" << c << endl;
		//步骤3：统计内点个数
		int inliers = 0;
		int sigma = 1.0;//阈值,可以自己调整
		for (auto k = 0; k < Vectors.size(); k++)
		{

			if (fabs(a*Vectors[k][0] + b * Vectors[k][1] - Vectors[k][2] + c) < sigma)
				inliers++;
		}
		//cout << "inliers" << inliers << endl;
		//步骤4：选出内点个数最大的参数
		if (inliers > maxinliers)
		{
			maxinliers = inliers;
			bestplane.clear();
			bestplane.push_back(a);
			bestplane.push_back(b);
			bestplane.push_back(c);
		}
	}
	//cout << "法向量为： x:" << bestplane[0]/bestplane[1] << " y:" << 1 << " z:" << -1/bestplane[1] << endl; 
}



//叉乘
vector<double> CrossProduct(vector<double>& a, vector<double>& b)
{
	if (a.size() != 3 || b.size() != 3)
		exit(-1);

	vector<double> c(3);

	c[0] = a[1] * b[2] - a[2] * b[1];
	c[1] = a[2] * b[0] - a[0] * b[2];
	c[2] = a[0] * b[1] - a[1] * b[0];

	return c;
}

//点乘
double DotProduct(vector<double>& a, vector<double>& b)
{
	if (a.size() != 3 || b.size() != 3)
		exit(-1);
	return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}

//模
double Normalize(vector<double>& v)
{
	if (v.size() != 3 )
		exit(-1);

	return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

//罗德里格旋转公式(Rodrigues' rotation formula)
vector<vector<double>> RotationMatrix(double angle, vector<double>& u)
{
	double norm = Normalize(u);
	vector<vector<double>> rotatinMatrix(3, vector<double>(3, 0)); //3X3矩阵

	u[0] = u[0] / norm;
	u[1] = u[1] / norm;
	u[2] = u[2] / norm;

	rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
	rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle)) - u[2] * sin(angle);
	rotatinMatrix[0][2] = u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

	rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
	rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
	rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));

	rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
	rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
	rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));

	return rotatinMatrix;
}

//计算旋转矩阵
vector<vector<double>> CalRotationMatrix(vector<double>& vectorBefore, vector<double>& vectorAfter)
{
	vector<double> rotationAxis; //旋转轴
	double rotationAngle;        //旋转角

	rotationAxis = CrossProduct(vectorBefore, vectorAfter);
	rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
	return RotationMatrix(rotationAngle, rotationAxis);
}

//输入旋转矩阵和向量，输入旋转后向量
inline vector<double> RotationAjust(vector<vector<double>>& r, vector<double>& v)
{
	if (r.size() != 3 || r[0].size() != 3)
		exit(-1);

	vector<double> res(3);

	res[0] = r[0][0] * v[0] + r[0][1] * v[1] + r[0][2] * v[2];
	res[1] = r[1][0] * v[0] + r[1][1] * v[1] + r[1][2] * v[2];
	res[2] = r[2][0] * v[0] + r[2][1] * v[1] + r[2][2] * v[2];

	return res;
}



