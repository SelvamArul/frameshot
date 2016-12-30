#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

int main(int argc, char **argv){
	std::string fileName;
	if(argc < 2){
		std::cout<<" No argument provided"<<std::endl;
		std::cout<<"Usage: pcd2png filename.pcd"<<std::endl;
		return 0;
	} else {
		fileName = std::string(argv[1]);
		std::cout<<fileName<<std::endl;
	}
	PointCloud::Ptr cloud (new PointCloud);
	if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (fileName, *cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}
	std::cout << "Loaded "
            << cloud->width <<" "<< cloud->height << " dimension from "
            << fileName << std::endl;
 	cv::Mat_<float>img (cloud->height,cloud->width);
 	for (int y = 0; y < img.rows; ++y)
 	{
 		for (int x =0; x < img.cols; ++x)
 		{
 			img(y,x) = (*cloud)(x,y).z;
 		}
 	}
 	cv::imshow("img",img);
	cv::waitKey(0);
	std::string pngFileName = fileName.substr(0,fileName.length()-3) + "yml";
	std::cout<<pngFileName<<std::endl;
	cv::FileStorage file(pngFileName, cv::FileStorage::WRITE);
	file << fileName.substr(0,fileName.length()-4) << img;
	return 0;
}
