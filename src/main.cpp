#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
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
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <visualization_msgs/MarkerArray.h>

#define NUM_FILE 2860
#define LENGTH_LABLE 16
#define POSE_COLS 7

using namespace std;
using namespace cv;
using namespace Eigen;


int main(int argc, char** argv){
    ros::init(argc, argv, "visualDet3D");
    ros::NodeHandle nh("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    ros::Publisher pub_odom = nh.advertise<nav_msgs::Odometry>("odometry", 1);
    ros::Publisher pub_depth = nh.advertise<sensor_msgs::Image>("depth", 1);
    ros::Publisher pub_image = nh.advertise<sensor_msgs::Image>("image", 1);
    ros::Publisher pub_bboxes = nh.advertise<darknet_ros_msgs::BoundingBoxes>("bboxes", 1);
    ros::Publisher pub_cubeArray = nh.advertise<visualization_msgs::MarkerArray>("cube_array", 1);
    ros::Publisher pub_cube = nh.advertise<visualization_msgs::Marker>("cube", 1);

    string base_folder = "/media/jixingwu/datasetj/LargeParkingLot_rich/20210108T122234/";
// kitti_train: Pedestrian 0.00 0 -0.20 712.40 143.00 810.73 307.92 1.89 0.48 1.20 1.84 1.47 8.41 0.01
// test: Car -1 -1 -1.665443 658.805481 192.106995 700.316650 224.475784 1.527742 1.750741 4.291881 3.651057 2.603817 38.601776 -1.569545 0.7964985370635986

    // -------- read test results in the folder named data, such as 'data/000000.txt' --------

    for(int i = 0; i < NUM_FILE; ++i)
    {
        vector<Eigen::VectorXd> label_v;
        vector<string>          class_v;
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        {
            string dataPath = base_folder + "data/" + ss.str() + ".txt";
            ifstream infile(dataPath, ios::in);
            //infile.open(base_folder + dataPath);
            assert(infile.is_open());
            cout << dataPath << endl;
            while (!infile.eof()) {
                string str;
                getline(infile, str);
                istringstream is(str);
                //cout << "is.str(): " << is.str() << endl;
                if (is.str().empty()) {
                    break;
                }

                Eigen::VectorXd label; // 1 + 15
                label.resize(LENGTH_LABLE - 1);
                double d;
                string class_name;
                is >> class_name;

                for (int j = 0; j < LENGTH_LABLE - 1; ++j) {
                    is >> d;
                    label(j) = d;
                }
                label_v.push_back(label);
                class_v.push_back(class_name);
                cout << label.transpose() << endl;
                label.setZero();
            }
            infile.close();
        }
        // TODO: plot 2d box and show the image.
        string imgPath =  "image_2/" + ss.str() + ".png";
        string depthPath = "depthUint16/" + ss.str() + ".png";
        cv::Mat img_depth = cv::imread(base_folder + depthPath, CV_LOAD_IMAGE_ANYDEPTH);
        cv::Mat img = cv::imread(base_folder + imgPath);
        cv::Mat img_show = img.clone();
        for(auto &label : label_v)
        {
            int left = label(3), top = label(4), right = label(5), bottom = label(6);
            cv::rectangle(img_show, Point(left, top), Point(right, bottom), Scalar (0, 255, 0), 2);
        }
        //imshow("winname", img_show);
        //waitKey(1);
        // TODO: pub odometry and reload timestamps
        string posPath = base_folder + "position/" + ss.str() + ".txt";
        Eigen::VectorXd pose;
        {
            ifstream infile(posPath, ios::in);
            assert(infile.is_open());
            while (!infile.eof())
            {
                string str;
                getline(infile, str);
                istringstream is(str);
                if (is.str().empty())
                    break;

                pose.resize(POSE_COLS);
                double d;
                for (int j = 0; j < POSE_COLS - 1; ++j) {
                    is >> d;
                    pose(j) = d;
                }
            }
            cout << pose.transpose() << endl;
        }
        // build odometry msgs
        std_msgs::Header header;
        header.frame_id = "world";
        header.seq = i;
        header.stamp = ros::Time(pose(0));

        nav_msgs::Odometry odometry;
        odometry.header = header;
        odometry.child_frame_id = "world";
        double yaw_angle = pose(6) * M_PI / 180;
        Quaterniond Q = Quaterniond(cos(yaw_angle/2), 0, 0, sin(yaw_angle/2));
        Vector3d    t = Vector3d(pose(1), pose(2), pose(3));
        odometry.pose.pose.position.x = t.x();
        odometry.pose.pose.position.y = t.y();
        odometry.pose.pose.position.z = t.z();
        odometry.pose.pose.orientation.x = Q.x();
        odometry.pose.pose.orientation.y = Q.y();
        odometry.pose.pose.orientation.z = Q.z();
        odometry.pose.pose.orientation.w = Q.w();

        sensor_msgs::ImagePtr imgRGBMsg = cv_bridge::CvImage(header, "bgr8", img).toImageMsg();
        sensor_msgs::ImagePtr imgDepMsg = cv_bridge::CvImage(header, "mono8", img_depth).toImageMsg();

        // TODO: pub bounding boxes as ros topic
        darknet_ros_msgs::BoundingBoxes boundingBoxes;
        for(int j = 0; j < label_v.size(); ++j)
        {
            Eigen::VectorXd list = label_v[j];
            string class_name = class_v[j];
            darknet_ros_msgs::BoundingBox boundingBox;
            boundingBox.Class = class_name;
            boundingBox.id = j;
            boundingBox.probability = list(14);
            boundingBox.xmin = list(3);
            boundingBox.ymin = list(4);
            boundingBox.xmax = list(5);
            boundingBox.ymax = list(6);
            boundingBoxes.bounding_boxes.push_back(boundingBox);
        }
        boundingBoxes.header = header;
        boundingBoxes.header.stamp = ros::Time::now();
        boundingBoxes.image_header = header;

        pub_image.publish(imgRGBMsg);
        pub_depth.publish(imgDepMsg);
        pub_bboxes.publish(boundingBoxes);
        pub_odom.publish(odometry);

        // TODO: pub cube topic message
        for(int j = 0; j < label_v.size(); ++j)
        {
            Eigen::VectorXd list = label_v[j];
            visualization_msgs::Marker marker;
            marker.header = header;
            marker.id = j;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = list(10); marker.pose.position.y = list(11); marker.pose.position.z = list(12);
            double pitch = list(13) * M_PI / 180;
            Quaterniond q = Quaterniond(cos(pitch/2), 0, 0, sin(pitch/2));
            marker.pose.orientation.w = q.w(); marker.pose.orientation.x = q.x(); marker.pose.orientation.y = q.y(); marker.pose.orientation.z = q.z();
            marker.scale.x = list(7); marker.scale.y = list(8); marker.scale.z = list(9);
            marker.color.a = 1.0; marker.color.r = 0; marker.color.g = 1; marker.color.b = 0;
            pub_cube.publish(marker);
        }
    }

    return 0;
}
