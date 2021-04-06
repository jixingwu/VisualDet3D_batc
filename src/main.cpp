#include <iostream>
#include <fstream>
#include <sstream>
#include <stdlib.h>
#include <queue>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <ros/package.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include "darknet_ros_msgs/BoundingBox.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include <visualization_msgs/MarkerArray.h>

#define NUM_FILE 2860
#define LENGTH_LABLE 15
#define M_ROWS  2860 // 更换数据集时需修改为图片数量
#define N_COLS  7 // position.txt中的cols

using namespace std;
using namespace cv;
using namespace Eigen;


int main(int argc, char** argv){
    ros::init(argc, argv, "cube_convert");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Publisher pub_odometry = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("image", 1);
    ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image>("depth", 1);

    ros::Publisher pub_cubeArray = nh.advertise<visualization_msgs::MarkerArray>("visualDet3D/cube_array", 1);
    ros::Publisher pub_bboxes = nh.advertise<darknet_ros_msgs::BoundingBoxes>("visualDet3D/bboxes", 1);

    string base_folder = "/media/jixingwu/datasetj/LargeParkingLot_rich/20210108T122234/data";
// kitti_train: Pedestrian 0.00 0 -0.20 712.40 143.00 810.73 307.92 1.89 0.48 1.20 1.84 1.47 8.41 0.01
// test: Car -1 -1 -1.665443 658.805481 192.106995 700.316650 224.475784 1.527742 1.750741 4.291881 3.651057 2.603817 38.601776 -1.569545 0.7964985370635986

    // -------- read test results in the folder named data, such as 'data/000000.txt' --------
    ifstream infile;
    for(int i = 0; i < NUM_FILE; ++i)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        string dataPath = ss.str() + ".txt";
        infile.open(base_folder + dataPath);
        assert(infile.is_open());
        Eigen::VectorXd label_vector(LENGTH_LABLE); // 1 + 15
        int index = 0; string str;
        while (getline(infile, str))
        {
            istringstream is(str);
            double d; char class_name; char ch;
            for(int j = 0; j < LENGTH_LABLE; ++j)
            {
                is >> class_name; is >> ch;
                is >> d; label_vector(j) = d; is >> ch;
            }
        }
    }
    // -------------- read position.txt into Eigen Matrix
    ifstream infile;
    infile.open(base_folder + "position.txt");
    assert(infile.is_open());
    Eigen::MatrixXd pos_matrix(M_ROWS, N_COLS);
    vector<double> row_vector;
    int index = 0;
    string str;
    while(getline(infile, str)){
        istringstream is(str);
        double t; char ch;
        VectorXd row_matrix(N_COLS); // position.txt有七列
        for (int i = 0; i < N_COLS; ++i) {
            is >> t;
            pos_matrix(index, i) = t;
            is >> ch;
        }
        index++;
    }
    cout<<"pos_matrix rows: "<<pos_matrix.rows()<<endl;
    // ---------------------------------------------------

    for (int row = 0; row < M_ROWS; ++row) {
        Eigen::VectorXd pos_vector(N_COLS);
        pos_vector = pos_matrix.row(row);

        std_msgs::Header header;
        header.frame_id = "world";
        header.stamp = ros::Time(pos_vector(0));
        // ----------- pub odometry
        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.child_frame_id = "world";
        double yaw_angle = pos_vector(6) * M_PI / 180;
        Quaterniond Q = Quaterniond(cos(yaw_angle/2), 0, 0, sin(yaw_angle/2));
        Vector3d    t = Vector3d(pos_vector(1), pos_vector(2), pos_vector(3));
        odometry.pose.pose.position.x = t.x();
        odometry.pose.pose.position.y = t.y();
        odometry.pose.pose.position.z = t.z();
        odometry.pose.pose.orientation.x = Q.x();
        odometry.pose.pose.orientation.y = Q.y();
        odometry.pose.pose.orientation.z = Q.z();
        odometry.pose.pose.orientation.w = Q.w();

        // -------------- pub image

        stringstream ss;
        ss << setfill('0') << setw(6) << row;
        string imagePath = "image/" + ss.str() + ".png";
        cv::Mat im_rgb = cv::imread(base_folder + imagePath, CV_LOAD_IMAGE_UNCHANGED);
        assert(!im_rgb.empty() && "rgb图片加载失败");
        sensor_msgs::ImagePtr imgRGBMsg = cv_bridge::CvImage(header, "bgr8", im_rgb).toImageMsg();



        string depthPath = "depthUint16/" + ss.str() + ".png";
        cv::Mat im_depth = cv::imread(base_folder + depthPath, CV_LOAD_IMAGE_ANYDEPTH);
//        im_depth.convertTo(im_depth, CV_32F);
//        cout<<"depth 值：\n"<<im_depth.col(0)<<endl;
        assert(!im_depth.empty() && "depth图片加载失败");
        sensor_msgs::ImagePtr imgDepthMsg = cv_bridge::CvImage(header, "mono8", im_depth).toImageMsg();

        odometry.header = header;
        imgRGBMsg->header = header;
        imgDepthMsg->header = header;

        pub_image.publish(imgRGBMsg);
        pub_odometry.publish(odometry);
        pub_depth.publish(imgDepthMsg);
        printf("process image %d\n", row);
//        cv::imshow("window", im_depth);
//        waitKey(0);
//        return 1;
    }

    return 0;
}
