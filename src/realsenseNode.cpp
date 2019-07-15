#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <numeric>

static const std::string OPENCV_WINDOW1 = "Image window1";
static const std::string OPENCV_WINDOW2 = "Image window2";

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
    image_sub_ = it_.subscribe("/camera/depth/image_rect_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW1);
    cv::namedWindow(OPENCV_WINDOW2);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW1);
    cv::destroyWindow(OPENCV_WINDOW2);
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
    cv::convertScaleAbs(cv_ptr->image, depth_float_img, 0.03);
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

    std::array<float, height> sums;
    std::array<float, height> means;
    std::array<float, height> variance;

    for(int i=0; i<Y.rows; i++) { // For each line:
    //for(int i=0; i < 10; i++) { // DEBUG
        int counter = 0;

        // Get sum:
        sums[i] = 0.0;
        for (int j = 0; j < Y.cols; j++) {
            uint16_t value = Y.at<uint16_t>(i, j);

            if (value > 0) {
              sums[i] += value;
              counter++;
            }
        }

        // Calculate mean:
        means[i] = sums[i] / counter;

        variance[i] = 0.0;
        counter = 0;

        for (int j = 0; j < Y.cols; j++) {
          uint16_t value = Y.at<uint16_t>(i, j);

          if (value > 0) {
              variance[i] += (value - means[i]) * (value - means[i]);
              counter++;
          }

          variance[i] /= counter;

        }

        //printf("Sum: %.2f, Mean: %.2f, Variance: %.2f, Counter: %d\n", sums[i], means[i], variance[i], counter);
    }

    printf("\n");

    double sum = 0.0;
    int counter = 0;
    for (int i = 0; i < variance.size(); i++){
        if (variance[i] == variance[i]){ // Check for nan
            counter++;
            sum += variance[i];
        }
    }

    double mean = sum / counter;
    printf("Mean variance: %.4f\n", mean);

    /*
    for(int i = 0; i < Y.rows; i++){
        const double* Mi = Y.ptr<double>(i);

        for(int j = 0; j < Y.cols; j++) {
            sum += std::max(Mi[j], 0.);
            printf("%.2f ", Mi[j]);
        }
        break;
    }*/

    //printf("Custom sum: %.2f, proper sum: %.2f, mean: %.2f\n", sum, cv::sum(Y).val[0], cv::mean(Y));
    //printf("Proper sum: %.2f, mean: %.2f\n", cv::sum(depth_float_img).val[0], cv::mean(depth_float_img));
    //printf("\n");

    //printf("rows: %.2f, cols: %.2f\n", rows, cols);
    //printf("p1: (%.2f, %.2f)\n", cols/2.0 - width/2.0, rows/2.0 - height/2.0);
    //printf("p2: (%.2f, %.2f)\n", cols/2.0 + width/2.0, rows/2.0 + height/2.0);

    cv::rectangle(depth_float_img, cv::Point(cols/2.0 - width/2.0, rows/2.0 - height/2.0), cv::Point(cols/2.0 + width/2.0, rows/2.0 + height/2.0), CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW1, depth_float_img);
    cv::imshow(OPENCV_WINDOW2, Y);
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
