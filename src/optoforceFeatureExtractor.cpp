// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>


class optoforceFeatureExtractor{
public:
    optoforceFeatureExtractor(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void initialize(){

        // Get node name
        _name = ros::this_node::getName();

        // Publishers
        _pub_feature = _nh.advertise<std_msgs::Float64MultiArray>("optoforceFeature",1);

        // Subscribers
        _sub_optoforce[0] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/bl", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "bl", 3));
        _sub_optoforce[1] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/br", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "br", 2));
        _sub_optoforce[2] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fl", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "fl", 0));
        _sub_optoforce[3] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fr", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "fr", 1));

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }

    void publishFeature(){


        std_msgs::Float64MultiArray msg;

        // set up dimensions
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = 4;
        msg.layout.dim[0].stride = 1;
        msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

        // copy in the data
        msg.data.clear();

        for (int i = 0; i < 4; i++) msg.data.push_back(_forceMeasurements[i]);

        _pub_feature.publish(msg);
    }

    void optoforceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, const std::string &topic, int legIndex) {
        auto forceSum = (float) (fabs(msg->wrench.force.x) + fabs(msg->wrench.force.y) + fabs(msg->wrench.force.z));

        _forceMeasurements[legIndex] = forceSum;

        if (legIndex == 3) publishFeature();
    }

    void spin(){
        ros::spin();
    }

private:

    // Node
    ros::NodeHandle _nh;
    std::string _name;

    // Publishers
    ros::Publisher _pub_feature;

    // Subscribers
    std::array<ros::Subscriber, 4> _sub_optoforce;

    // Other
    std::array<float, 4> _forceMeasurements;

};

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc,argv,"optoforceFeatureExtractor");
    ros::NodeHandle nh("~");

    optoforceFeatureExtractor ofe(nh);
    ofe.spin();

    return 0;
}