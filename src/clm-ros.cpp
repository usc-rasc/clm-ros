// SimpleCLM.cpp : Defines the entry point for the console application.
#include "CLM_core.h"

#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <opencv2/videoio/videoio.hpp>  // Video write
//#include <opencv2/videoio/videoio_c.h>  // Video write


#define INFO_STREAM( stream ) \
std::cout << stream << std::endl

#define WARN_STREAM( stream ) \
std::cout << "Warning: " << stream << std::endl

#define ERROR_STREAM( stream ) \
std::cout << "Error: " << stream << std::endl

static void printErrorAndAbort( const std::string & error )
{
  std::cout << error << std::endl;
  abort();
}

#define FATAL_STREAM( stream ) \
printErrorAndAbort( std::string( "Fatal error: " ) + stream )

using namespace std;
using namespace cv;

// Some globals for tracking timing information for visualisation
double fps_tracker = -1.0;
int64 t0 = 0;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  string model_location;
	nh.param("~model_location", model_location, "/");
  // The modules that are being used for tracking
  CLMTracker::CLM clm_model(model_location);	
  CVImagePtr cv_image_ptr = cv_bridge::toCvCopy(msg);
  Mat captured_image = cv_image_ptr->image;

	// If optical centers are not defined just use center of image
	if (cx_undefined)
	{
		cx = captured_image.cols / 2.0f;
		cy = captured_image.rows / 2.0f;
	}
	// Use a rough guess-timate of focal length
	if (fx_undefined)
	{
		fx = 500 * (captured_image.cols / 640.0);
		fy = 500 * (captured_image.rows / 480.0);

		fx = (fx + fy) / 2.0;
		fy = fx;
	}
  Mat_<float> depth_image;
  Mat_<uchar> grayscale_image; 

  if(captured_image.channels() == 3)
  {
    cvtColor(captured_image, grayscale_image, CV_BGR2GRAY);       
  }
  else
  {
    grayscale_image = captured_image.clone();       
  }
    
  // Get depth image
  if(use_depth)
  {
    char* dst = new char[100];
    std::stringstream sstream;

    sstream << depth_directories[f_n] << "\\depth%05d.png";
    sprintf(dst, sstream.str().c_str(), frame_count + 1);
    // Reading in 16-bit png image representing depth
    Mat_<short> depth_image_16_bit = imread(string(dst), -1);

    // Convert to a floating point depth image
    if(!depth_image_16_bit.empty())
    {
      depth_image_16_bit.convertTo(depth_image, CV_32F);
    }
    else
    {
      WARN_STREAM( "Can't find depth image" );
    }
  }
	// The actual facial landmark detection / tracking
	bool detection_success = CLMTracker::DetectLandmarksInVideo(grayscale_image, depth_image, clm_model, clm_parameters);
	// Work out the pose of the head from the tracked model
	Vec6d pose_estimate_CLM;
	if(use_camera_plane_pose)
	{
		pose_estimate_CLM = CLMTracker::GetCorrectedPoseCameraPlane(clm_model, fx, fy, cx, cy);
	}
	else
	{
		pose_estimate_CLM = CLMTracker::GetCorrectedPoseCamera(clm_model, fx, fy, cx, cy);
	}
  while(!captured_image.empty())
  {   


  }

  // Reset the model
  clm_model.Reset();

}

int main (int argc, char **argv)
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, image_callback);
  ros::spin();
  return 0;
}
