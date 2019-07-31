// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <dynamic_reconfigure/server.h>


class optoforceFeatureExtractor{
public:
    optoforceFeatureExtractor(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void initialize(){

        // Get node name
        _name = ros::this_node::getName();

        // Subscribers
        _subs_optoforce[0] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/bl", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "bl"));
        _subs_optoforce[1] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/br", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "br"));
        _subs_optoforce[2] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fl", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "fl"));
        _subs_optoforce[3] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fr", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "fr"));

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }

    void optoforceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, const std::string &topic) {
        printf("Callback %s\n", topic.c_str());
    }

    void spin(){
        ros::spin();
    }

private:

    // Node
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    ros::Publisher _pub_inliers;// Display inliers for each plane

    // Subscribers
    std::array<ros::Subscriber, 4> _subs_optoforce;

};

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc,argv,"optoforceFeatureExtractor");
    ros::NodeHandle nh("~");

    optoforceFeatureExtractor ofe(nh);
    ofe.spin();

    return 0;
}