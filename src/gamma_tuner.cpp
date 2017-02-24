/*******************************************************************************
//author: Arul Selvam Periyasamy
// mail id: arulselvam@uni-bonn.de
// This node computes the best gamma value
*******************************************************************************/
#include <ros/ros.h>
#include <ros/package.h>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>
#include <chrono>

#include <algorithm>
#include <vector>
#include <string>
#include <cmath>

bool g_can_record = false;
bool g_is_done = false;
std::string g_package_path =  ros::package::getPath("frameshot");
std::string g_left_image_topic = "/vertical_stereo/left/image_raw";
std::string g_right_image_topic = "/vertical_stereo/right/image_raw";
std::string g_param_node_name = "/stereo/multisync_camera_nodelet";  // name of 
// the node that has gamma param 
std::vector<cv::Mat> cv_left_images, cv_right_images;
std::vector<cv::Mat> cv_left_histograms, cv_right_histograms;

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

void stereo_callback(const sensor_msgs::ImageConstPtr& l_msg, const sensor_msgs::ImageConstPtr& r_msg){
	if (g_can_record == true) {
		try{
			cv_bridge::CvImagePtr l_ptr, r_ptr;
			l_ptr = cv_bridge::toCvCopy(l_msg, sensor_msgs::image_encodings::MONO8);
			r_ptr = cv_bridge::toCvCopy(r_msg, sensor_msgs::image_encodings::MONO8);

			cv_left_images.push_back(l_ptr->image);
			cv_right_images.push_back(r_ptr->image);
//  			std::cout<<"Image pushed back"<<std::endl;
			g_can_record = false;
		} catch (cv_bridge::Exception& e){
			ROS_ERROR("cv_bridge exception: %s", e.what());
		}
	}
}

void gammaWatcherCallback() {
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;

	for (int i = 0; i < 14; ++i){
		conf.doubles.clear();
		g_can_record = false;
		double_param.name = "gamma";
		double_param.value = (i * 0.25) + 0.25;
		conf.doubles.push_back(double_param);
		std::cout<<"Gamma value to be set "<<(i * 0.25) + 0.25<<std::endl;
		srv_req.config = conf;
		bool result = ros::service::call(
			g_param_node_name + "/set_parameters",
			srv_req,
			srv_resp);
// 		std::cout<<"Param set "<<result
// 			<<"    vector size "<<cv_images.size()<<std::endl;
		if (result == 0)
			ROS_ERROR("Parameter not set ");
		boost::this_thread::sleep( boost::posix_time::milliseconds(500) );
		g_can_record = true;
		boost::this_thread::sleep( boost::posix_time::milliseconds(500) );
	}
	
	// signal the node spin to stop
	g_is_done = true;
}

void save_images() {
	u_int count = 0;
	char *filename;
	for (auto &i : cv_left_images) {
		asprintf(&filename, "%s/resource/cvimage_left_%hhu.png", g_package_path.c_str(), count);
		cv::imwrite(filename,i);
		count++;
	}
	count = 0;
	for (auto &i : cv_right_images) {
		asprintf(&filename, "%s/resource/cvimage_right_%hhu.png", g_package_path.c_str(), count);
		cv::imwrite(filename,i);
		count++;
	}
	free(filename);
// 	std::cout<<"Images saved to disk "<<std::endl;
}

void compute_histograms(){
	// cv::calcHist parameters
	int histSize = 256;
	float range[] = { 0, 256 } ;
	const float* histRange = { range };
	bool uniform = true;
	bool accumulate = false;
	
	cv_left_histograms.resize(cv_left_images.size());
	for (u_int i = 0; i < cv_left_images.size(); ++i) {
		cv::calcHist( &cv_left_images[i], 1, 0, cv::Mat(), cv_left_histograms[i], 1, &histSize, &histRange, uniform, accumulate );
		double minVal, maxVal;
		cv::minMaxLoc( cv_left_histograms[i], &minVal, &maxVal);
		
//  		std::cout << "imgtype " << type2str(cv_images[i].type())<<" Hist "<< type2str(cv_histograms[i].type()) << " " 
//  			<< minVal <<" " <<maxVal<<std::endl;
	}
	cv_right_histograms.resize(cv_right_images.size());
	for (u_int i = 0; i < cv_right_images.size(); ++i) {
		cv::calcHist( &cv_right_images[i], 1, 0, cv::Mat(), cv_right_histograms[i], 1, &histSize, &histRange, uniform, accumulate );
		double minVal, maxVal;
		cv::minMaxLoc( cv_right_histograms[i], &minVal, &maxVal);
		
//  		std::cout << "imgtype " << type2str(cv_images[i].type())<<" Hist "<< type2str(cv_histograms[i].type()) << " " 
//  			<< minVal <<" " <<maxVal<<std::endl;
	}
}


/**
 * \brief Visualizes the given histogram
 * \param[in] hist The histogram
 * \param[in] name The name of the histogram
 */
void visualizeHistogram(const cv::Mat_<float>& hist,
                        const std::string& name)
{
    int histSize = hist.rows;

    int hist_w = histSize * 2;
    int hist_h = 400;
    double bin_w = (double)hist_w/histSize;

    cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar(0,0,0) );

    cv::Mat_<float> hist_normalized;

    // Normalize the result to [ 0, histImage.rows ]
    cv::normalize(hist, hist_normalized, 0, histImage.rows, cv::NORM_MINMAX);

    // Draw
    for(int i = 0; i < histSize; ++i)
    {
        cv::Point p1(cvRound(bin_w * i),       hist_h - cvRound(hist_normalized(i)));
        cv::Point p2(cvRound(bin_w * (i+1))-1, hist_h - 1);

        cv::rectangle(histImage,
            p1, p2,
            cv::Scalar(255, 255, 255), CV_FILLED
        );
    }

    // Display
    cv::imshow(name.c_str(), histImage);
	cv::waitKey(0);
}
// compare the histogram to find the one that is closest to an uniform distribution
// create an uniformly distributed histogram and do dot product with all other histogram
// The one that has maximum value is what we need
void compare_histograms(){
	cv::Mat uniform_histogram (cv_left_histograms[0].rows,
							   cv_left_histograms[0].cols,
							   cv_left_histograms[0].type(),
							   cv::Scalar( cv_left_images[0].rows * cv_left_images[0].cols /(cv_left_images[0].rows * cv_left_images[0].cols )));
	std::vector<float> closeness(cv_left_images.size(), 0);
	u_int index = 0;
	
// 	std::cout<<"uniform_histogram "<<cv_images[0].rows<<" "<< cv_images[0].cols<<std::endl;
// 	for (int i = 0; i < uniform_histogram.rows; i++ ) {
// 		for (int j = 0; j < uniform_histogram.cols; j++ ) {
// 			std::cout<<uniform_histogram.at<float>(i,j)<<" ";
// 		}
// 		std::cout<<std::endl;
// 	}
	for (auto &hist: cv_left_histograms) {
// 		visualizeHistogram(hist, "histogram");
		closeness[index] = cv::compareHist(uniform_histogram, hist, CV_COMP_BHATTACHARYYA);
		index ++;
	}
	for (auto &hist: cv_right_histograms) {
// 		visualizeHistogram(hist, "histogram");
		closeness[index] = cv::compareHist(uniform_histogram, hist, CV_COMP_BHATTACHARYYA);
		index ++;
	}
// 	for (u_int i = 0; i < closeness.size(); ++i ){
// 		std::cout<<closeness[i]<<std::endl;
// 	}
	
	std::vector<float>::iterator result;
	result = std::min_element(closeness.begin(), closeness.end());
    std::cout << "Best gamma value " << std::distance(closeness.begin(), result) 
		<<" ( " << std::distance(closeness.begin(), result) * 0.25 + 0.25<<" )"<<std::endl;
		
	// set the Best gamma
	dynamic_reconfigure::ReconfigureRequest srv_req;
	dynamic_reconfigure::ReconfigureResponse srv_resp;
	dynamic_reconfigure::DoubleParameter double_param;
	dynamic_reconfigure::Config conf;
	double_param.name = "gamma";
	double_param.value = std::distance(closeness.begin(), result) * 0.25 + 0.25;
	conf.doubles.push_back(double_param);
	
	srv_req.config = conf;
	bool is_gamma_set = ros::service::call(
		g_param_node_name + "/set_parameters",
		srv_req,
		srv_resp);

	if (is_gamma_set == 0)
		ROS_ERROR("Best gamma not set ");
	
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "gamma_tuner");
	ros::NodeHandle nh;
	
	image_transport::ImageTransport it(nh);
	nh.param<std::string>("image_topic", g_left_image_topic, g_left_image_topic);
	nh.param<std::string>("node_name", g_param_node_name, g_param_node_name);
	std::cout<<" image topic " << g_left_image_topic <<std::endl;
	std::cout<<" node_name " << g_param_node_name <<std::endl;
//     image_transport::Subscriber sub = it.subscribe(g_left_image_topic, 1, imageCallback);
	
	message_filters::Subscriber<sensor_msgs::Image> image_Lsub(nh, g_left_image_topic , 1);
	message_filters::Subscriber<sensor_msgs::Image> image_Rsub(nh, g_right_image_topic , 1);
	
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> stereoSyncPolicy;
	message_filters::Synchronizer<stereoSyncPolicy> sync(stereoSyncPolicy(10), image_Lsub, image_Rsub);
	sync.registerCallback(boost::bind(&stereo_callback, _1, _2));
	
	std::cout<<"Started"<<std::endl;

	boost::thread gammaWatcherThread(gammaWatcherCallback);
	while (!g_is_done)
		ros::spinOnce();
	gammaWatcherThread.join();
	save_images();
	compute_histograms();
	compare_histograms();
	std::cout<<"Done "<<std::endl;
	return 0;
}
