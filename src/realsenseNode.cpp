#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <numeric>

static const std::string OPENCV_WINDOW1 = "Image window1";

std::vector<float> meanVariances;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/dyret/sensor/camera/depth", 1,&ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW1);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
  }

  void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
    //Process images
    if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
      mono8_img = cv::Mat(float_img.size(), CV_16UC1);}
    cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
  }

  std::string type2str(int type) {
  std::string r;

      uchar depth = type & CV_MAT_DEPTH_MASK;
      uchar chans = 1 + (type >> CV_CN_SHIFT);

      switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
      }

      r += "C";
      r += (chans+'0');

      return r;
    }


  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      //cv_ptr = cv_bridge::toCvCopy(msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat depth_float_img;
    cv::convertScaleAbs(cv_ptr->image, depth_float_img, 0.1);


    cv::applyColorMap(depth_float_img, depth_float_img, cv::COLORMAP_JET);

    // Extract interesting area:

    float rows = cv_ptr->image.rows;
    float cols = cv_ptr->image.cols;

    const int width = 300;
    const int height = 400;

    cv::Mat Y;
    cv_ptr->image(cv::Rect(cols/2.0 - width/2.0,rows/2.0 - height/2.0,width,height)).copyTo(Y);

    //std::string ty =  type2str( Y.type() );
    //printf("Matrix: %s %dx%d \n", ty.c_str(), Y.cols, Y.rows );

    std::vector<float> sums;
    std::vector<float> means;
    std::vector<float> variances;
    std::vector<float> SD;

    for(int i=0; i<Y.rows; i++) { // For each line:
    //for(int i=0; i<50; i++) { // For each line:
        int colCounter = 0;

        // Get sum:
        double sum = 0.0;

        for (int j = 0; j < Y.cols; j++) {
            uint16_t value = Y.at<uint16_t>(i, j);

            if (value > 0 && value < 1000) {
              sum += value;
              colCounter++;
            }

        }

        if (colCounter > 0) { // Only calculate if we have valid points on the line
            sums.push_back(sum);

            // Calculate mean:
            double lineMean = sum / (double) colCounter;
            means.push_back(lineMean);

            double variance = 0.0;

            for (int j = 0; j < Y.cols; j++) {
                uint16_t value = Y.at<uint16_t>(i, j);

                if (value > 0 && value < 1000) { // Valid pixel

                    variance += ((float(value) - lineMean) * (float(value) - lineMean));
                }

                if (variance != variance) { // Check for nan
                    printf("Variance nan: %.4f\n", variance);
                }

                variance /= colCounter;
                variances.push_back(variance);
                SD.push_back(sqrt(variance));

            }
        }

    }

    printf("\n");

    double variance_sum = 0.0;
    double SD_sum = 0.0;
    double rms_sum = 0.0;
    double mean_sum = 0.0;
    int counter = 0;
    for (int i = 0; i < variances.size(); i++){
        if (variances[i] == variances[i]){ // Check for nan
            counter++;
            variance_sum += variances[i];
            SD_sum += SD[i];
            mean_sum += means[i];
        }
    }

    // Get median variance
    std::vector<float> varianceCopy = variances;

    std::sort(std::begin(varianceCopy), std::end(varianceCopy));

    double varianceMedian = varianceCopy[(int) varianceCopy.size()/2];

    double meanVariance = variance_sum / counter;
    double meanSD = SD_sum / counter;

    meanVariances.push_back(meanVariance);

    double meanVarianceSum = 0.0;
    for (int i = 0; i < meanVariances.size(); i++){
        meanVarianceSum += meanVariances[i];
    }
    double meanMeanVariance = meanVarianceSum / meanVariances.size();

    double meanVarianceSD = 0.0;
    for (int i = 0; i < meanVariances.size(); i++){
      meanVarianceSD += sqrt((meanMeanVariance - meanVariances[i]) * (meanMeanVariance - meanVariances[i]));
    }

    meanVarianceSD /= meanVariances.size();

    printf("Mean variance: %.4f, Whole run: %.5f +- %.5f\n",
            meanVariance,
            meanMeanVariance,
            meanVarianceSD);

           /* varianceMedian,
            meanSD,
            mean_sum / counter ,
            ((meanVariance * 163.2) + 840.0) / (mean_sum / counter),
            (((meanVariance * 163.2) + 840.0) / (mean_sum / counter)) * (((meanVariance * 163.2) + 840.0) / (mean_sum / counter)));*/

    cv::rectangle(depth_float_img, cv::Point(cols/2.0 - width/2.0, rows/2.0 - height/2.0), cv::Point(cols/2.0 + width/2.0, rows/2.0 + height/2.0), CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW1, depth_float_img);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
