#include <ros/ros.h>
#include "ar_sys_prep/img_filter_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "img_filter");
    ImgFilterNode img_filter;
    ros::spin();
	return 0;
}
