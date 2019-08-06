// Adapted from https://github.com/apalomer/plane_fitter

// ROS
#include <ros/ros.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>

#include "terrain_characterizer/algorithmParametersConfig.h"

double point2planedistance(pcl::PointXYZ pt, pcl::ModelCoefficients::Ptr coefficients){
    double f1 = fabs(coefficients->values[0]*pt.x+coefficients->values[1]*pt.y+coefficients->values[2]*pt.z+coefficients->values[3]);
    double f2 = sqrt(pow(coefficients->values[0],2)+pow(coefficients->values[1],2)+pow(coefficients->values[2],2));
    return f1/f2;
}

class ColorMap{
public:
    ColorMap(double mn, double mx): mn(mn), mx(mx){}
    void setMinMax(double min, double max){ mn = min; mx = max;}
    void setMin(double min){mn = min;}
    void setMax(double max){mx = max;}
    void getColor(double c,uint8_t& R, uint8_t& G, uint8_t& B){
        double normalized = (c - mn)/(mx-mn) * 2 - 1;
        R = (int) (base(normalized - 0.5) * 255);
        G = (int) (base(normalized) * 255);
        B = (int) (base(normalized + 0.5) * 255);
    }
    void getColor(double c, double &rd, double &gd, double &bd){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(double c){
        uint8_t r;
        uint8_t g;
        uint8_t b;
        getColor(c,r,g,b);
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }


private:
    double interpolate(double val, double y0, double x0, double y1, double x1){
        return (val - x0)*(y1-y0)/(x1-x0) + y0;
    }
    double base(double val){
        if (val <= -0.75) return 0;
        else if (val <= -0.25) return interpolate(val,0,-0.75,1,-0.25);
        else if (val <= 0.25) return 1;
        else if (val <= 0.75) return interpolate(val,1.0,0.25,0.0,0.75);
        else return 0;
    }
private:
    double mn,mx;
};

class Color{
private:
    uint8_t r;
    uint8_t g;
    uint8_t b;

public:
    Color(uint8_t R,uint8_t G,uint8_t B):r(R),g(G),b(B){

    }

    void getColor(uint8_t &R,uint8_t &G,uint8_t &B){
        R = r;
        G = g;
        B = b;
    }
    void getColor(double &rd, double &gd, double &bd){
        rd = (double)r/255;
        gd = (double)g/255;
        bd = (double)b/255;
    }
    uint32_t getColor(){
        return ((uint32_t)r<<16|(uint32_t)g<<8|(uint32_t)b);
    }
};

class pointCloudPlaneFitter{
public:
    pointCloudPlaneFitter(ros::NodeHandle nh): _nh(nh){
        initialize();
    }

    void initialize(){

        // Get node name
        _name = ros::this_node::getName();

        // Get Publisher read
        _pub_inliers = _nh.advertise< sensor_msgs::PointCloud2 >("inliers",2);

        // Subscriber
        _subs = _nh.subscribe("/dyret/sensor/camera/pointcloud",1,&pointCloudPlaneFitter::pointCloudCb,this);

        _max_distance = 0.03; // 30mm

        // Get parameters
        ros::param::param<bool>("~color_pc_with_error",_color_pc_with_error,false);
        ros::param::param<bool>("~enable_sending",_enable_sending,false);

        // Create dynamic reconfigure
        drCallback = boost::bind( &pointCloudPlaneFitter::updateParameters, this, _1, _2);
        drServer.setCallback(drCallback);

        // Create colors palette
        createColors();

        // Inform initialized
        ROS_INFO("%s: node initialized.",_name.c_str());
    }

    void updateParameters(terrain_characterizer::algorithmParametersConfig& config, uint32_t level){
        _color_pc_with_error = config.paint_with_error;
        _enable_sending = config.enable_sending;
    }

    void pointCloudCb(const sensor_msgs::PointCloud2::ConstPtr &msg){

        // Convert to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg,*cloud_msg);
        ROS_DEBUG("%s: new pointcloud (%i,%i)(%zu)",_name.c_str(),cloud_msg->width,cloud_msg->height,cloud_msg->size());

        // Filter cloud
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud_msg);
        pass.setFilterFieldName ("z");
        pass.setFilterLimits(0.001,10000);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter (*cloud);

        // Get segmentation ready
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold(_max_distance);

        // Create pointcloud to publish inliers
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub(new pcl::PointCloud<pcl::PointXYZRGB>);
        int original_size(cloud->height*cloud->width);

        // Fit the plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        // Iterate inliers to get error
        double mean_error(0);
        double max_error(0);
        double min_error(100000);
        std::vector<double> err;
        for (int i=0;i<inliers->indices.size();i++){

            // Get Point
            pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

            // Compute distance
            double d = point2planedistance(pt,coefficients)*1000;// mm
            err.push_back(d);

            // Update statistics
            mean_error += d;
            if (d>max_error) max_error = d;
            if (d<min_error) min_error = d;

        }
        mean_error/=inliers->indices.size();

        // Compute Standard deviation
        ColorMap cm(min_error,max_error);
        double sigma(0);
        for (int i=0;i<inliers->indices.size();i++){

            sigma += pow(err[i] - mean_error,2);


            if (_enable_sending) {
                // Get Point
                pcl::PointXYZ pt = cloud->points[inliers->indices[i]];

                // Copy point to new cloud
                pcl::PointXYZRGB pt_color;
                pt_color.x = pt.x;
                pt_color.y = pt.y;
                pt_color.z = pt.z;
                uint32_t rgb;
                if (_color_pc_with_error)
                    rgb = cm.getColor(err[i]);
                else
                    rgb = colors[0].getColor();
                pt_color.rgb = *reinterpret_cast<float *>(&rgb);
                cloud_pub->points.push_back(pt_color);
            }

        }
        sigma = sqrt(sigma/inliers->indices.size());

        if (_enable_sending) {
            // Extract inliers
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.setNegative(true);
            pcl::PointCloud <pcl::PointXYZ> cloudF;
            extract.filter(cloudF);
            cloud->swap(cloudF);
        }

        // Display information
        /*ROS_INFO("%s: fitted plane: %fx%s%fy%s%fz%s%f=0 (inliers: %zu/%i)",
                 _name.c_str(),
                 coefficients->values[0],(coefficients->values[1]>=0?"+":""),
                 coefficients->values[1],(coefficients->values[2]>=0?"+":""),
                 coefficients->values[2],(coefficients->values[3]>=0?"+":""),
                 coefficients->values[3],
                 inliers->indices.size(),original_size);*/
        ROS_INFO("%s: mean error: %f(mm), standard deviation: %f (mm), max error: %f(mm)",_name.c_str(),mean_error,sigma,max_error);
        ROS_INFO("%s: Using %i of %i points",_name.c_str(), inliers->indices.size(), original_size);

        if (_enable_sending) {
            // Publish points
            sensor_msgs::PointCloud2 cloud_publish;
            pcl::toROSMsg(*cloud_pub, cloud_publish);
            cloud_publish.header = msg->header;
            _pub_inliers.publish(cloud_publish);
        }
    }

    void createColors(){
        uint8_t r = 0;
        uint8_t g = 0;
        uint8_t b = 0;
        for (int i=0;i<20;i++){
            while (r<70 && g < 70 && b < 70){
                r = rand()%(255);
                g = rand()%(255);
                b = rand()%(255);
            }
            Color c(r,g,b);
            r = 0;
            g = 0;
            b = 0;
            colors.push_back(c);
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
    ros::Publisher _pub_inliers;// Display inliers for each plane

    // Subscriber
    ros::Subscriber _subs;

    // Algorithm parameters
    double _max_distance;
    bool _color_pc_with_error;
    bool _enable_sending;

    // Colors
    std::vector<Color> colors;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<terrain_characterizer::algorithmParametersConfig> drServer;
    dynamic_reconfigure::Server<terrain_characterizer::algorithmParametersConfig>::CallbackType drCallback;
};

int main(int argc,char** argv){

    // Initialize ROS
    ros::init(argc,argv,"pointCloudPlaneFitter");
    ros::NodeHandle nh("~");

    pointCloudPlaneFitter pf(nh);
    pf.spin();

    return 0;
}