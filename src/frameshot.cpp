#define PCL_NO_PRECOMPILE

#include <ros/package.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

#include <string>

#include <termios.h>
#include <boost/thread.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_calibration_parsers/parse.h>
/* pcl::PCLPointCloud2 provides more generic abstract mechnaism to handle the point clouds. Thus avoiding the template based pcl::PointCloud.
struct PointXYZNIG {
    float x,y,z;
    float noise;
    uint16_t intensity;
    uint8_t gray;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZNIG, (float, x, x) (float, y, y) (float, z, z) (float, noise, noise) (uint16_t, intensity, intensity) (uint8_t, gray, gray))
*/
int getch()
{
    static struct termios oldt, newt;
    tcgetattr( STDIN_FILENO, &oldt);           // save old settings
    newt = oldt;
    newt.c_lflag &= ~(ICANON);                 // disable buffering
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

    int c = getchar();  // read character (non-blocking)

    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
    return c;
}

bool g_canWrite = false;
bool g_written = true;
u_int8_t count = 0;
std::string g_folderName;
std::string g_fileName;
std::string g_pcd_topic, g_caminfotopic;
std::string g_rgb_topic, g_depth_topic;
std::string g_mode;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    if (g_canWrite == true) {
        ROS_INFO("writing pcd");
        pcl::PCLPointCloud2 pclCloud;
        pcl_conversions::toPCL(*cloud_msg, pclCloud);
        char *filename;
	
        if (g_fileName.empty())
            asprintf(&filename, "%s/count_%hhu.pcd", g_folderName.c_str(), count);
        else
            asprintf(&filename, "%s/%s_%hhu.pcd", g_folderName.c_str(),g_fileName.c_str(), count);

        pcl::io::savePCDFile(filename, pclCloud,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),true);
        ROS_INFO("Wrote PCD to %s", filename);
        free(filename);
        ros::Duration(0.2).sleep();
        g_written = true;
        g_canWrite = false;
        count++;
    }
}

void cloud_with_caminfo_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg, const sensor_msgs::CameraInfoConstPtr& cam_info){
    if (g_canWrite == true) {
        ROS_INFO("writing pcd");
        pcl::PCLPointCloud2 pclCloud;
        pcl_conversions::toPCL(*cloud_msg, pclCloud);
        char *filenamePCD, *filenameYAML;

        if (g_fileName.empty()) {
            asprintf(&filenamePCD, "%s/count_%hhu.pcd", g_folderName.c_str(), count);
            asprintf(&filenameYAML, "%s/count_%hhu.yaml", g_folderName.c_str(), count);
        } else {
            asprintf(&filenamePCD, "%s/%s_%hhu.pcd", g_folderName.c_str(), g_fileName.c_str(), count);
            asprintf(&filenameYAML, "%s/%s_%hhu.yaml", g_folderName.c_str(),g_fileName.c_str(), count);
        }
        pcl::io::savePCDFile(filenamePCD, pclCloud,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),true);
        camera_calibration_parsers::writeCalibration(filenameYAML,"camera",*cam_info);

        ROS_INFO("Wrote PCD to %s", filenamePCD);
        ROS_INFO("Wrote yaml to %s", filenameYAML);
        free(filenamePCD);
        free(filenameYAML);
        ros::Duration(0.2).sleep();
        g_written = true;
        g_canWrite = false;
        count++;
    }
}

void stereo_callback (const sensor_msgs::ImageConstPtr& imageL,const sensor_msgs::ImageConstPtr& imageR) {
    if (g_canWrite == true) {
        cv_bridge::CvImagePtr L_ptr,R_ptr;
        try {
            L_ptr = cv_bridge::toCvCopy(imageL, sensor_msgs::image_encodings::BGR8);
            R_ptr = cv_bridge::toCvCopy(imageR, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        char *filenameL,*filenameR;

        if (g_fileName.empty()) {
            asprintf(&filenameL, "%s/L_count_%hhu.png", g_folderName.c_str(), count);
            asprintf(&filenameR, "%s/R_count_%hhu.png", g_folderName.c_str(), count);
        } else {
            asprintf(&filenameL, "%s/L_%s_%hhu.png", g_folderName.c_str(),g_fileName.c_str(), count);
            asprintf(&filenameR, "%s/R_%s_%hhu.png", g_folderName.c_str(),g_fileName.c_str(), count);
        }
        cv::imwrite(filenameL,L_ptr->image);
        cv::imwrite(filenameR,R_ptr->image);
        ROS_INFO("Wrote Image to %s ",filenameL);
        ROS_INFO("Wrote Image to %s ",filenameR);

        free(filenameL);
        free(filenameR);
		
        ros::Duration(0.2).sleep();
        g_written = true;
        g_canWrite = false;
        count++;
    }
}

void kinect2_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg,
					  const sensor_msgs::CameraInfoConstPtr& cam_info,
					  const sensor_msgs::ImageConstPtr& rgb,
					  const sensor_msgs::ImageConstPtr& depth) {
	if (g_canWrite == true) {
		cv_bridge::CvImagePtr rgb_ptr, depth_ptr;
        try {
            rgb_ptr = cv_bridge::toCvCopy(rgb, sensor_msgs::image_encodings::BGR8);
            depth_ptr = cv_bridge::toCvCopy(depth, sensor_msgs::image_encodings::BGR8);
		} catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        pcl::PCLPointCloud2 pclCloud;
        pcl_conversions::toPCL(*cloud_msg, pclCloud);
        char *filenamePCD, *filenameYAML;
		char *filenameRGB, *filenameDepth;

        if (g_fileName.empty()) {
            asprintf(&filenameRGB, "%s/L_count_%hhu.png", g_folderName.c_str(), count);
            asprintf(&filenameDepth, "%s/R_count_%hhu.png", g_folderName.c_str(), count);
			asprintf(&filenamePCD, "%s/count_%hhu.pcd", g_folderName.c_str(), count);
            asprintf(&filenameYAML, "%s/count_%hhu.yaml", g_folderName.c_str(), count);
        } else {
            asprintf(&filenameRGB, "%s/L_%s_%hhu.png", g_folderName.c_str(),g_fileName.c_str(), count);
            asprintf(&filenameDepth, "%s/R_%s_%hhu.png", g_folderName.c_str(),g_fileName.c_str(), count);
			asprintf(&filenamePCD, "%s/%s_%hhu.pcd", g_folderName.c_str(), g_fileName.c_str(), count);
            asprintf(&filenameYAML, "%s/%s_%hhu.yaml", g_folderName.c_str(),g_fileName.c_str(), count);
        }
        ROS_INFO("Writing Image to %s ",filenameRGB);
        ROS_INFO("Writing Image to %s ",filenameDepth);
        cv::imwrite(filenameRGB,rgb_ptr->image);
        cv::imwrite(filenameDepth,depth_ptr->image);
        ROS_INFO("Wrote Image to %s ",filenameRGB);
        ROS_INFO("Wrote Image to %s ",filenameDepth);
		
		pcl::io::savePCDFile(filenamePCD, pclCloud,Eigen::Vector4f::Zero(),Eigen::Quaternionf::Identity(),true);
        camera_calibration_parsers::writeCalibration(filenameYAML,"camera",*cam_info);

        ROS_INFO("Wrote PCD to %s", filenamePCD);
        ROS_INFO("Wrote yaml to %s", filenameYAML);
		
        free(filenameRGB);
        free(filenameDepth);
		free(filenamePCD);
		free(filenameYAML);
		
        ros::Duration(0.2).sleep();
        g_written = true;
        g_canWrite = false;
        count++;
	}
}
void keyboardCallback() {
    while (ros::ok()){
        int c = getch();   // call your non-blocking input function
        if (c == ' ') {
            while (g_written != true) {
                ros::Duration(0.2).sleep();
            }
            g_canWrite = true;
        }
    }
}
int main(int argc, char **argv) {
    ros::init(argc, argv, "frameshot");
    ros::NodeHandle nh("~");
    ros::Rate loop_rate(10);
    ros::Subscriber sub;

    nh.getParam("mode",g_mode);
    nh.getParam("folder_name",g_folderName);
    nh.getParam("topic",g_pcd_topic);
    nh.getParam("caminfo_topic",g_caminfotopic);
    nh.getParam("file_name",g_fileName);

    if (g_folderName.empty() || g_pcd_topic.empty() || g_mode.empty())
    {
        ROS_INFO(" ============> param  _mode or _folder_name or _topic is not provided ");
        ROS_INFO(" ============> Aborting");
        return 0;
    }

    ROS_INFO("foldername %s",g_folderName.c_str());
    ROS_INFO("topic %s",g_pcd_topic.c_str());

    if (g_fileName.empty()){
        ROS_INFO(" No filename provided. Using default name!!!");
        ROS_INFO("fileName %s",g_fileName.c_str());
    }
    boost::thread keyboardThread(keyboardCallback);

    if (g_mode=="pcd") {
        sub = nh.subscribe(g_pcd_topic, 1, cloud_callback);
        ros::spin();
    }
    if (g_mode=="stereo"){
        ROS_INFO("stereo mode");
        ROS_INFO( ("/camera1" + g_pcd_topic).c_str());
        ROS_INFO( ("/camera2" + g_pcd_topic).c_str());
        message_filters::Subscriber<sensor_msgs::Image> image_Lsub(nh, ("/camera1" + g_pcd_topic).c_str() , 1);
        message_filters::Subscriber<sensor_msgs::Image> image_Rsub(nh, ("/camera2" + g_pcd_topic).c_str() , 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> stereoSyncPolicy;
        message_filters::Synchronizer<stereoSyncPolicy> sync(stereoSyncPolicy(10), image_Lsub, image_Rsub);
        sync.registerCallback(boost::bind(&stereo_callback, _1, _2));
        boost::thread keyboardThread(keyboardCallback);
        ros::spin();
    }
    if (g_mode=="pcdcaminfo"){
        ROS_INFO("pcdcaminfo mode");
        if (g_caminfotopic.empty()){
            ROS_INFO(" ============> param  _caminfo_topic is not provided ");
            ROS_INFO(" ============> Aborting");
            return 0;
        }
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcdtopic(nh, g_pcd_topic, 1);
        message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo(nh, g_caminfotopic , 1);

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::CameraInfo> camInfoSyncPolicy;
        message_filters::Synchronizer<camInfoSyncPolicy> sync(camInfoSyncPolicy(10), pcdtopic, caminfo);
        sync.registerCallback(boost::bind(&cloud_with_caminfo_callback, _1, _2));
        boost::thread keyboardThread(keyboardCallback);
        ros::spin();
    }
    if (g_mode=="kinect2"){
		ROS_INFO("kinect2 mode (cloud, rgb, depth, camInfo)");
		nh.getParam("rgb_topic",g_rgb_topic);
		nh.getParam("depth_topic",g_depth_topic);
		if (g_pcd_topic.empty() || g_caminfotopic.empty() || g_rgb_topic.empty() || g_depth_topic.empty()){
            ROS_INFO(" ============> Not all params are provided ");
            ROS_INFO(" ============> Aborting");
            return 0;
        }
        message_filters::Subscriber<sensor_msgs::PointCloud2> pcdtopic(nh, g_pcd_topic, 100);
        message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo(nh, g_caminfotopic , 100);
		message_filters::Subscriber<sensor_msgs::Image> rgbtopic(nh, g_rgb_topic , 100);
		message_filters::Subscriber<sensor_msgs::Image> depthtopic(nh, g_depth_topic , 100);
		typedef message_filters::sync_policies::ApproximateTime
									<sensor_msgs::PointCloud2,
									sensor_msgs::CameraInfo,
									sensor_msgs::Image,
									sensor_msgs::Image> kinect2SyncPolicy;
        message_filters::Synchronizer<kinect2SyncPolicy> sync(kinect2SyncPolicy(100), pcdtopic, caminfo, rgbtopic, depthtopic);
        sync.registerCallback(boost::bind(&kinect2_callback, _1, _2, _3, _4));
        boost::thread keyboardThread(keyboardCallback);
        ros::spin();
		
	}
    keyboardThread.join();

    return 0;

}
