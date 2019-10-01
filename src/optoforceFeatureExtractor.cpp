#include <iostream>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <dynamic_reconfigure/server.h>

#include "terrain_characterizer/featureLoggingService.h"

class optoforceFeatureExtractor{
public:
    optoforceFeatureExtractor(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    bool featureLoggingCallback(terrain_characterizer::featureLoggingService::Request  &req,
                                terrain_characterizer::featureLoggingService::Response &res) {
        if (req.logPath.empty()){
            if (_logFile.is_open()){
                ROS_INFO("Closing file and stopping logging");
                for (int i = 0; i < 4; i++) _forceMeasurements[i].clear();

                _logFile.close();
            } else {
                ROS_ERROR("Tried closing file that is not open");
            }
        } else {
            if (!_logFile.is_open()) {
                _logFile.open(req.logPath.c_str());
                _firstPrint = true;
                ROS_INFO("Starting to log to %s", req.logPath.c_str());
            } else {
                ROS_ERROR("Tried opening file that is already open");
            }
        }

        return true;
    }

    void initialize(){

        // Get node name
        _name = ros::this_node::getName();

        // Publishers
        _pub_feature = _nh.advertise<std_msgs::Float64MultiArray>("/dyret/environment/optoforceFeature",1);

        // Services
        _featureLoggingService = _nh.advertiseService("/dyret/optoforceFeatureExtractor/featureLogging", &optoforceFeatureExtractor::featureLoggingCallback, this);

        // Subscribers
        _sub_optoforce[0] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/bl", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "bl", 3));
        _sub_optoforce[1] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/br", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "br", 2));
        _sub_optoforce[2] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fl", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "fl", 0));
        _sub_optoforce[3] = _nh.subscribe<geometry_msgs::WrenchStamped>("/dyret/sensor/contact/fr", 100, boost::bind(&optoforceFeatureExtractor::optoforceCallback, this, _1, "fr", 1));

        receivedMeasurement = {false, false, false, false};

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }

    void publishFeature(){
        std_msgs::Float64MultiArray msg;

        if (!_forceMeasurements[0].empty() && !_forceMeasurements[1].empty() && !_forceMeasurements[2].empty() && !_forceMeasurements[3].empty()) {

            // set up dimensions
            msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
            msg.layout.dim[0].size = 6;
            msg.layout.dim[0].stride = 1;
            msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

            // copy in the data
            msg.data.clear();

            msg.data.resize(6);
            for (int i = 0; i < 4; i++) msg.data[i] = *max_element(std::begin(_forceMeasurements[i]), std::end(_forceMeasurements[i]));

            msg.data[4] = msg.data[0] + msg.data[1];
            msg.data[5] = msg.data[0] + msg.data[1] + msg.data[2] + msg.data[3];

            _pub_feature.publish(msg);

            if (_logFile.is_open()) {

                if (_firstPrint) {
                    _logFile << "leg0, leg1, leg2, leg3, front, all\n";
                    _firstPrint = false;
                } else {
                    _logFile << "\n";
                }

                _logFile << std::to_string(msg.data[0]) << ", "
                         << std::to_string(msg.data[1]) << ", "
                         << std::to_string(msg.data[2]) << ", "
                         << std::to_string(msg.data[3]) << ", "
                         << std::to_string(msg.data[0] + msg.data[1]) << ", "
                         << std::to_string(msg.data[0]+msg.data[1]+msg.data[2]+msg.data[3]);
            }

        }
    }

    void optoforceCallback(const geometry_msgs::WrenchStamped::ConstPtr &msg, const std::string &topic, int legIndex) {
        auto forceSum = (float) (fabs(msg->wrench.force.x) + fabs(msg->wrench.force.y) + fabs(msg->wrench.force.z));

        _forceMeasurements[legIndex].insert(_forceMeasurements[legIndex].begin(), forceSum);

        if (_forceMeasurements[legIndex].size() > numberOfValuesToKeep){
            _forceMeasurements[legIndex].pop_back();
        }

        receivedMeasurement[legIndex] = true;

        if (receivedMeasurement[0] &&
            receivedMeasurement[1] &&
            receivedMeasurement[2] &&
            receivedMeasurement[3]){

            receivedMeasurement = {false, false, false, false};

            if (_forceMeasurements[legIndex].size() == numberOfValuesToKeep) publishFeature();
        }
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

    // Service
    ros::ServiceServer _featureLoggingService;

    // Logging
    std::ofstream _logFile;
    bool _firstPrint;

    // Other
    std::array<std::vector<double>, 4> _forceMeasurements;
    int numberOfValuesToKeep = 600;
    std::array<bool, 4> receivedMeasurement;

};

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc,argv,"optoforceFeatureExtractor");
    ros::NodeHandle nh("~");

    optoforceFeatureExtractor ofe(nh);
    ofe.spin();

    return 0;
}