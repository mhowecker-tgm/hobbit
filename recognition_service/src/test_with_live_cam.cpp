/*
 * main.cpp
 *
 *  Created on: Sep 1, 2013
 *      Author: aitor
 */

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"
#include "recognizer_msg_and_services/recognize.h"
#include <pcl/common/common.h>
#include <queue>
#include <pcl/console/parse.h>

class TestRecognizer
{
private:
    typedef pcl::PointXYZRGB PointT;
    bool KINECT_OK_;
    int kinect_trials_;
    std::queue<sensor_msgs::PointCloud2::ConstPtr> clouds_;
    int MAX_CLOUDS_;
    ros::NodeHandle n_;
    std::string camera_topic_;

public:

    TestRecognizer ()
    {
      MAX_CLOUDS_ = 1;
      KINECT_OK_ = false;
    }

    void
    checkCloudArrive (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        KINECT_OK_ = true;
    }

    void
    checkKinect ()
    {
        ros::Subscriber sub_pc = n_.subscribe (camera_topic_, 1, &TestRecognizer::checkCloudArrive, this);
        ros::Rate loop_rate (1);
        kinect_trials_ = 0;
        while (!KINECT_OK_ && ros::ok ())
        {
            std::cout << "Checking kinect status..." << std::endl;
            ros::spinOnce ();
            loop_rate.sleep ();
            kinect_trials_++;
            if(kinect_trials_ >= 5)
            {
                std::cout << "Kinect is not working..." << std::endl;
                return;
            }
        }

        KINECT_OK_ = true;
        std::cout << "Kinect is up and running" << std::endl;
    }

    void
    updatePointCloud (const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        std::cout << "Received cloud" << std::endl;
        clouds_.push (msg);
        if (clouds_.size () > (size_t)MAX_CLOUDS_)
        {
            clouds_.pop (); //dismiss oldest element
        }
    }

    void Execute()
    {

        ros::Subscriber sub_pc_ = n_.subscribe (camera_topic_, 1, &TestRecognizer::updatePointCloud, this);
        ros::Rate loop_rate (30);
        while ((int)clouds_.size () < MAX_CLOUDS_)
        {
          ros::spinOnce ();
          loop_rate.sleep ();
        }

        //prepare service call...
        ros::ServiceClient client = n_.serviceClient<recognizer_msg_and_services::recognize>("mp_recognition");
        recognizer_msg_and_services::recognize srv;
        sensor_msgs::PointCloud2::ConstPtr msg = clouds_.front ();
        srv.request.cloud = *msg;

        //parse service call
        std::vector<Eigen::Matrix4f> transforms;
        std::vector<std::string> model_ids;

        if (client.call(srv))
        {
            std::cout << "Found categories:" << static_cast<int>(srv.response.ids.size()) << std::endl;
            std::cout << "Found transformations:" << static_cast<int>(srv.response.transforms.size()) << std::endl;
            for(size_t i=0; i < srv.response.ids.size(); i++)
            {
                std::cout << "   => " << srv.response.ids[i] << std::endl;
                model_ids.push_back(srv.response.ids[i].data);

                Eigen::Matrix4f tt;
                tt.setIdentity(4,4);

                tt(0,3) = srv.response.transforms[i].translation.x;
                tt(1,3) = srv.response.transforms[i].translation.y;
                tt(2,3) = srv.response.transforms[i].translation.z;
                Eigen::Quaternionf q(srv.response.transforms[i].rotation.w,
                                     srv.response.transforms[i].rotation.x,
                                     srv.response.transforms[i].rotation.y,
                                     srv.response.transforms[i].rotation.z);

                Eigen::Matrix3f rot = q.toRotationMatrix();
                tt.block<3,3>(0,0) = rot;

                transforms.push_back(tt);
            }
        }
    }

    void
    readParameters (int argc, char ** argv)
    {
        //recognition parameters
        camera_topic_ = "/camera/depth_registered/points";
        pcl::console::parse_argument (argc, argv, "-cam_topic", camera_topic_);
    }

    void
    initialize (int argc, char ** argv)
    {

        readParameters (argc, argv);
        KINECT_OK_ = false;
        checkKinect ();
    }

};

int
main (int argc, char ** argv)
{
    ros::init (argc, argv, "test_recognition");
    TestRecognizer t;
    t.initialize(argc, argv);
    t.Execute();
    return 0;
}
