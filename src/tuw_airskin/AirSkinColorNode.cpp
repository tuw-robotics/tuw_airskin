#include <ros/ros.h>
#include <nodelet/loader.h>

int main ( int argc, char **argv ) {
    ros::init ( argc, argv, "airskin_control" ); /// initializes the ros node with default name
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    std::string nodelet_name = ros::this_node::getName();
    nodelet.load(nodelet_name, "tuw/AirSkinColorNodelet", remap, nargv);
    ros::spin();

    return 0;
}
