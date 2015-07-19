#include "ros/ros.h"
#include <opencv2/opencv.hpp>
//#include <opencv2/highgui/highgui.hpp>
#include "px4flow_hover/tracker_cmd.h"
#include "std_msgs/Bool.h"
#include <iostream>
using namespace std;
using namespace cv;

const char SPACE_KEY = 32;

int main(int argc,char **argv)
{
        ros::init(argc,argv,"test_tracker_cmd_node");
        ros::NodeHandle n;
        ros::Publisher test_cmd_publisher = n.advertise<px4flow_hover::tracker_cmd>("/irobot_tracker_cmd",10);
        px4flow_hover::tracker_cmd tracker_msg;
        int dx_ = 0,dy_ = 0,stop_flag = 0;
        namedWindow("test_tracker_cmd",WINDOW_NORMAL);
        createTrackbar( "stop_flag:", "test_tracker_cmd", &stop_flag, 1, NULL );
        createTrackbar( "dx*10:", "test_tracker_cmd", &dx_, 40, NULL );
        createTrackbar( "dy*10:", "test_tracker_cmd", &dy_, 40, NULL );
        //ros::Rate loop_rate(30);
        while(ros::ok())
        {
                if((char)waitKey(30) == SPACE_KEY)//按空格发布cmd
                {
                        tracker_msg.stop_flag = stop_flag;
                        tracker_msg.dx = (dx_-20)/10.0;
                        tracker_msg.dy = (dy_-20)/10.0;
                        test_cmd_publisher.publish(tracker_msg);
                        ROS_INFO("publish tracker msg..., stop_flag = %d, dx = %f, dy = %f",stop_flag,tracker_msg.dx,tracker_msg.dy);
                }
                //waitKey(0);
                ros::spinOnce();
        }
        return 0;
}
