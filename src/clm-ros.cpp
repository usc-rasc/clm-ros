// SimpleCLM.cpp : Defines the entry point for the console application.
#include "CLM_core.h"

#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
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
using namespace cv_bridge;
/*
vector<string> get_arguments(int argc, char **argv)
{

	vector<string> arguments;

	for(int i = 0; i < argc; ++i)
	{
		arguments.push_back(string(argv[i]));
	}
	return arguments;
}*/

// Some globals for tracking timing information for visualisation
double fps_tracker = -1.0;
int64 t0 = 0;
/*
// Visualising the results
void visualise_tracking(Mat& captured_image, Mat_<float>& depth_image, const CLMTracker::CLM& clm_model, const CLMTracker::CLMParameters& clm_parameters, int frame_count, double fx, double fy, double cx, double cy)
{

	// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
	double detection_certainty = clm_model.detection_certainty;
	bool detection_success = clm_model.detection_success;

	double visualisation_boundary = 0.2;

	// Only draw if the reliability is reasonable, the value is slightly ad-hoc
	if (detection_certainty < visualisation_boundary)
	{
		CLMTracker::Draw(captured_image, clm_model);

		double vis_certainty = detection_certainty;
		if (vis_certainty > 1)
			vis_certainty = 1;
		if (vis_certainty < -1)
			vis_certainty = -1;

		vis_certainty = (vis_certainty + 1) / (visualisation_boundary + 1);

		// A rough heuristic for box around the face width
		int thickness = (int)std::ceil(2.0* ((double)captured_image.cols) / 640.0);

		Vec6d pose_estimate_to_draw = CLMTracker::GetCorrectedPoseCameraPlane(clm_model, fx, fy, cx, cy);

		// Draw it in reddish if uncertain, blueish if certain
		CLMTracker::DrawBox(captured_image, pose_estimate_to_draw, Scalar((1 - vis_certainty)*255.0, 0, vis_certainty * 255), thickness, fx, fy, cx, cy);

	}

	// Work out the framerate
	if (frame_count % 10 == 0)
	{
		double t1 = cv::getTickCount();
		fps_tracker = 10.0 / (double(t1 - t0) / cv::getTickFrequency());
		t0 = t1;
	}

	// Write out the framerate on the image before displaying it
	char fpsC[255];
	std::sprintf(fpsC, "%d", (int)fps_tracker);
	string fpsSt("FPS:");
	fpsSt += fpsC;
	cv::putText(captured_image, fpsSt, cv::Point(10, 20), CV_FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255, 0, 0));

	if (!clm_parameters.quiet_mode)
	{
		namedWindow("tracking_result", 1);
		imshow("tracking_result", captured_image);

		if (!depth_image.empty())
		{
			// Division needed for visualisation purposes
			imshow("depth", depth_image / 2000.0);
		}

	}
}
*/
int main (int argc, char **argv)
{
	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, imageCallback);
	image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);

	//vector<string> arguments = get_arguments(argc, argv);

	// Some initial parameters that can be overriden from command line	
	//vector<string> files, depth_directories, pose_output_files, tracked_videos_output, landmark_output_files, landmark_3D_output_files;
	
	// By default try webcam 0
	//int device = 0;



	//CLMTracker::CLMParameters clm_parameters(arguments);

	// Get the input output file parameters
	
	// Indicates that rotation should be with respect to camera plane or with respect to camera
	//bool use_camera_plane_pose;
	//CLMTracker::get_video_input_output_params(files, depth_directories, pose_output_files, tracked_videos_output, landmark_output_files, landmark_3D_output_files, use_camera_plane_pose, arguments);
	
	// The modules that are being used for tracking
	CLMTracker::CLM clm_model(clm_parameters.model_location);	

	// Grab camera parameters, if they are not defined (approximate values will be used)
	float fx = 0, fy = 0, cx = 0, cy = 0;
	// Get camera parameters
	CLMTracker::get_camera_params(device, fx, fy, cx, cy, arguments);

	// If cx (optical axis centre) is undefined will use the image size/2 as an estimate
	bool cx_undefined = false;
	bool fx_undefined = false;
	if (cx == 0 || cy == 0)
	{
		cx_undefined = true;
	}
	if (fx == 0 || fy == 0)
	{
		fx_undefined = true;
	}

	// If multiple video files are tracked, use this to indicate if we are done
	bool done = false;	
	int f_n = -1;

	while(!done) // this is not a for loop as we might also be reading from a webcam
	{
		/*
		string current_file;

		// We might specify multiple video files as arguments
		if(files.size() > 0)
		{
			f_n++;			
		    current_file = files[f_n];
		}
		else
		{
			// If we want to write out from webcam
			f_n = 0;
		}

		double fps_vid_in = -1.0;

		bool use_depth = !depth_directories.empty();	

		// Do some grabbing
		VideoCapture video_capture;
		if( current_file.size() > 0 )
		{
			INFO_STREAM( "Attempting to read from file: " << current_file );
			video_capture = VideoCapture( current_file );
			fps_vid_in = video_capture.get(CV_CAP_PROP_FPS);
			
			// Check if fps is nan or less than 0
			if (fps_vid_in != fps_vid_in || fps_vid_in <= 0)
			{
				INFO_STREAM("FPS of the video file cannot be determined, assuming 30");
				fps_vid_in = 30;
			}
		}
		else
		{
			INFO_STREAM( "Attempting to capture from device: " << device );
			video_capture = VideoCapture( device );

			// Read a first frame often empty in camera
			Mat captured_image;
			video_capture >> captured_image;
		}

		if( !video_capture.isOpened() ) FATAL_STREAM( "Failed to open video source" );
		else INFO_STREAM( "Device or file opened");
		*/
		Mat captured_image;
		//video_capture >> captured_image;		

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

		// Creating output files
		std::ofstream pose_output_file;
		if(!pose_output_files.empty())
		{
			pose_output_file.open (pose_output_files[f_n], ios_base::out);
			pose_output_file << "frame, timestamp, confidence, success, Tx, Ty, Tz, Rx, Ry, Rz";
			pose_output_file << endl;
		}
	
		std::ofstream landmarks_output_file;		
		if(!landmark_output_files.empty())
		{
			landmarks_output_file.open(landmark_output_files[f_n], ios_base::out);
			landmarks_output_file << "frame, timestamp, confidence, success";
			for (int i = 0; i < clm_model.pdm.NumberOfPoints(); ++i)
				landmarks_output_file << ", x" << i;

			for (int i = 0; i < clm_model.pdm.NumberOfPoints(); ++i)
				landmarks_output_file << ", y" << i;

			landmarks_output_file << endl;
		}

		std::ofstream landmarks_3D_output_file;
		if(!landmark_3D_output_files.empty())
		{
			landmarks_3D_output_file.open(landmark_3D_output_files[f_n], ios_base::out);

			landmarks_3D_output_file << "frame, timestamp, confidence, success";
			for (int i = 0; i < clm_model.pdm.NumberOfPoints(); ++i)
				landmarks_3D_output_file << ", X" << i;

			for (int i = 0; i < clm_model.pdm.NumberOfPoints(); ++i)
				landmarks_3D_output_file << ", Y" << i;

			for (int i = 0; i < clm_model.pdm.NumberOfPoints(); ++i)
				landmarks_3D_output_file << ", Z" << i;

			landmarks_3D_output_file << endl;
		}
	
		int frame_count = 0;
		
		// saving the videos
		VideoWriter writerFace;
		if(!tracked_videos_output.empty())
		{
			double fps = fps_vid_in == -1 ? 30 : fps_vid_in;
			writerFace = VideoWriter(tracked_videos_output[f_n], CV_FOURCC('D', 'I', 'V', 'X'), fps, captured_image.size(), true);
		}

		// Use for timestamping if using a webcam
		int64 t_initial = cv::getTickCount();

		// Timestamp in seconds of current processing
		double time_stamp = 0;
		
		INFO_STREAM( "Starting tracking");
		while(!captured_image.empty())
		{		

			// Grab the timestamp first
			if (fps_vid_in == -1)
			{
				int64 curr_time = cv::getTickCount();
				time_stamp = (double(curr_time - t_initial) / cv::getTickFrequency());
			}
			else 
			{
				time_stamp = (double)frame_count * (1.0 / fps_vid_in);
			}

			// Reading the images
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
			/*
			// Visualising the results
			// Drawing the facial landmarks on the face and the bounding box around it if tracking is successful and initialised
			double detection_certainty = clm_model.detection_certainty;

			visualise_tracking(captured_image, depth_image, clm_model, clm_parameters, frame_count, fx, fy, cx, cy);
			*/
			// Output the detected facial landmarks
			if(!landmark_output_files.empty())
			{
				double confidence = 0.5 * (1 - clm_model.detection_certainty);
				landmarks_output_file << frame_count + 1 << ", " << time_stamp << ", " << confidence << ", " << detection_success;
				for (int i = 0; i < clm_model.pdm.NumberOfPoints() * 2; ++ i)
				{
					landmarks_output_file << ", " << clm_model.detected_landmarks.at<double>(i);
				}
				landmarks_output_file << endl;
			}

			// Output the detected facial landmarks
			if(!landmark_3D_output_files.empty())
			{
				double confidence = 0.5 * (1 - clm_model.detection_certainty);
				landmarks_3D_output_file << frame_count + 1 << ", " << time_stamp << ", " << confidence << ", " << detection_success;
				Mat_<double> shape_3D = clm_model.GetShape(fx, fy, cx, cy);
				for (int i = 0; i < clm_model.pdm.NumberOfPoints() * 3; ++i)
				{
					landmarks_3D_output_file << ", " << shape_3D.at<double>(i);
				}
				landmarks_3D_output_file << endl;
			}

			// Output the estimated head pose
			if(!pose_output_files.empty())
			{
				double confidence = 0.5 * (1 - clm_model.detection_certainty);
				pose_output_file << frame_count + 1 << ", " << time_stamp << ", " << confidence << ", " << detection_success
					<< ", " << pose_estimate_CLM[0] << ", " << pose_estimate_CLM[1] << ", " << pose_estimate_CLM[2]
					<< ", " << pose_estimate_CLM[3] << ", " << pose_estimate_CLM[4] << ", " << pose_estimate_CLM[5] << endl;
			}
			/*
			// output the tracked video
			if(!tracked_videos_output.empty())
			{		
				writerFace << captured_image;
			}
			*/
			video_capture >> captured_image;
		
			// detect key presses
			char character_press = cv::waitKey(1);
			
			// restart the tracker
			if(character_press == 'r')
			{
				clm_model.Reset();
			}
			// quit the application
			else if(character_press=='q')
			{
				return(0);
			}

			// Update the frame count
			frame_count++;

		}
		
		frame_count = 0;

		// Reset the model, for the next video
		clm_model.Reset();

		pose_output_file.close();
		landmarks_output_file.close();

		// break out of the loop if done with all the files (or using a webcam)
		if(f_n == files.size() -1 || files.empty())
		{
			done = true;
		}
	}

	return 0;
}