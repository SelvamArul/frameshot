#include <pcl/features/integral_image_normal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/pca.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

struct picoPointXYZNIG {
    float x,y,z;
    float noise;
    uint16_t intensity;
    uint8_t gray;
};

POINT_CLOUD_REGISTER_POINT_STRUCT (picoPointXYZNIG, (float, x, x) (float, y, y) (float, z, z) (float, noise, noise) (uint16_t, intensity, intensity) (uint8_t, gray, gray))

typedef picoPointXYZNIG picpPoint;
typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<picpPoint> picoPointCloud;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointXYZRGB ColorPoint ;
typedef pcl::PointCloud<ColorPoint> ColorPointCloud;


//! Simple euclidean clustering comparator
class ClusterComparator : public pcl::Comparator<Point>
{
public:
	explicit ClusterComparator(float distanceThreshold)
	 : m_threshold(distanceThreshold*distanceThreshold)
	{}

	virtual bool compare(int idx1, int idx2) const
	{
		const PointCloud& cloud = *getInputCloud();
		const Point& p1 = cloud[idx1];
		const Point& p2 = cloud[idx2];

		if(!pcl::isFinite(p1) || !pcl::isFinite(p2))
			return false;

		return std::abs(p1.getVector3fMap().y() - p2.getVector3fMap().y()) < m_threshold;
	}
private:
	float m_threshold;
};

struct Cluster
	{
		pcl::PointIndices::Ptr indices;
	};

static const bool VISUALIZE = true;

int main(int argc, char **argv){
	std::string fileName;
	if(argc < 2){
		std::cout<<" No argument provided"<<std::endl;
		std::cout<<"Usage: tts filename.pcd"<<std::endl;
		return 0;
	} else {
		fileName = std::string(argv[1]);
		std::cout<<fileName<<std::endl;
	}
	picoPointCloud::Ptr pico_cloud (new picoPointCloud);
	if (pcl::io::loadPCDFile<picpPoint> (fileName, *pico_cloud) == -1)
	{
		PCL_ERROR ("Couldn't read file\n");
		return (-1);
	}
	std::cout << "Loaded "
            << pico_cloud->width <<" "<< pico_cloud->height << " dimension from "
            << fileName << std::endl;
	
	
	//convert picoPointCloud to PointCloud
	PointCloud::Ptr cloud_raw (new PointCloud);
	PointCloud::Ptr cloud (new PointCloud);
	pcl::copyPointCloud(*pico_cloud,  *cloud_raw);
	std::cout<<"conversion done: from picpPointCloud to PontCloud"<<std::endl;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	
	
	Eigen::Matrix4f transform;
	
	
	/*
	Eigen::Affine3f transformSensorInWorldFrame;
	transform << 0,0,0,0,
	             0,0,0,0
	             0,0,0,-0.1
	             0,0,0,1 ;
	transformSensorInWorldFrame.matrix() = transform;
	*/
	// remove outliers in 
	Point min,max;
	
	pcl::getMinMax3D(*cloud_raw,min,max);
	std::cout<<"min / max points :"<<min<<" "<<max<<std::endl;
	
	pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZ> ());
	range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ> >("x", pcl::ComparisonOps::GT, -0.5 ));
	range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ> >("x", pcl::ComparisonOps::LT,  0.5 ));
	range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ> >("y", pcl::ComparisonOps::GT, -0.5));
	range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ> >("y", pcl::ComparisonOps::LT,  0.5));
 	range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ> >("z", pcl::ComparisonOps::GT,  0.13));
	range_cond->addComparison(boost::make_shared<pcl::FieldComparison<pcl::PointXYZ> >("z", pcl::ComparisonOps::LT,  0.2));
	pcl::ConditionalRemoval<pcl::PointXYZ> removal (range_cond);
	removal.setInputCloud (cloud_raw);
	removal.setKeepOrganized(true);
	removal.filter (*cloud);
	
	
	
	
	// Visualize point cloud
	if (false){
		pcl::visualization::PCLVisualizer viewer ("ICP visualization");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 0, 0);
		viewer.addPointCloud (cloud, source_cloud_color_handler, "original_cloud");
		
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
		viewer.addCoordinateSystem();
		
		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}
	
		viewer.close();
		viewer.spinOnce();
	}

	
	// Estimate surface normals
	{
		pcl::IntegralImageNormalEstimation<Point, pcl::Normal> ne;
		ne.setInputCloud(cloud);
		ne.setNormalEstimationMethod(ne.AVERAGE_DEPTH_CHANGE);
		ne.compute(*normals);
	}
	std::cout<<"Estimate surface normals done: "<<std::endl;
	
	// Extract horizontal points
	pcl::PointIndices::Ptr horizontalIndices(new pcl::PointIndices);
	{

		horizontalIndices->indices.reserve(cloud->size());

		const float angleThreshold = cos(M_PI/4.0);

		for(std::size_t i = 0; i < cloud->size(); ++i)
		{
			if(!pcl::isFinite((*cloud)[i]))
				continue;

			if(fabs((*normals)[i].normal_z) > angleThreshold)
				horizontalIndices->indices.push_back(i);
		}
	}
	std::cout<<"Extract horizontal points done: "<<std::endl;
	std::cout<<"Number of points in cloud: "<<cloud->size()<<std::endl;
    std::cout<<"Number of points in horizontalIndices: "<< horizontalIndices->indices.size() <<std::endl;
	
	
	PointCloud::Ptr horizontalOutliersCloud (new PointCloud);
	if (true){
		PointCloud::Ptr newCloud (new PointCloud);
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (cloud);
		extract.setIndices (horizontalIndices);
		extract.setNegative (false);
		extract.filter (*newCloud);
		
		/*
		pcl::visualization::PCLVisualizer viewer ("ICP visualization");
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 255, 0, 0);
		viewer.addPointCloud (newCloud, source_cloud_color_handler, "original_cloud");
		
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
		viewer.addCoordinateSystem();
		
		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}
		viewer.close();
		viewer.spinOnce();
		*/
		
		extract.setInputCloud (cloud);
		extract.setIndices (horizontalIndices);
		extract.setNegative (true);
		extract.filter (*horizontalOutliersCloud);
		
		}
		
	// Fit plane
	Eigen::Vector4f plane;
	pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
	{
		pcl::SACSegmentation<Point> seg;
		seg.setOptimizeCoefficients(false);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);
		seg.setMaxIterations(30);
		seg.setInputCloud(cloud);
		seg.setIndices(horizontalIndices);
		seg.segment(*planeInliers, *coeffs);

		if(planeInliers->indices.size() == 0)
		{
			std::cout<<"0 planeInliers"<<std::endl;
			return 0;
		}

		plane << coeffs->values[0], coeffs->values[1], coeffs->values[2], coeffs->values[3];

		// Flip normal such that it points upwards
		if(plane.z() < 0)
			plane = -plane;
	}
	std::cout<<"Fit plane done: inliners: "<<planeInliers->indices.size()<<std::endl;
	
	
	// Project inliers to plane & calculate convex hull
	PointCloud::Ptr projectedPlaneInliers(new PointCloud);
	PointCloud::Ptr planeHull(new PointCloud);
	Eigen::Vector3f planeCenter = Eigen::Vector3f::Zero();
	{
		pcl::ProjectInliers<Point> proj;
		proj.setInputCloud(cloud);
		proj.setIndices(planeInliers);
		proj.setModelType(pcl::SACMODEL_PLANE);
		proj.setModelCoefficients(coeffs);
		proj.filter(*projectedPlaneInliers);

		pcl::ConvexHull<Point> hull;
		hull.setInputCloud(projectedPlaneInliers);
		hull.setDimension(2);
		hull.reconstruct(*planeHull);

		Eigen::Vector4f min, max;
		pcl::getMinMax3D(*planeHull, min, max);

		planeCenter = ((max + min)/2.0).head<3>();

		Eigen::Vector4f centroid;
		pcl::compute3DCentroid(*planeHull, centroid);

		// Shrink the hull towards the center
		for(std::size_t i = 0; i < planeHull->size(); ++i)
		{
			pcl::PointXYZ& p = (*planeHull)[i];
			p.getVector3fMap() = 0.9 * (p.getVector3fMap() - planeCenter) + planeCenter;
		}
	}
	
	// Extarct the  plane outliers
	PointCloud::Ptr planeOutliersCloud (new PointCloud);
	PointCloud::Ptr planeInliersCloud (new PointCloud);
	pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
	{	
		pcl::ExtractIndices<Point> extract;

		extract.setInputCloud (cloud);
		extract.setIndices (planeInliers);

		extract.setNegative (true);
		extract.filter (objectIndices->indices);
		
		extract.setNegative (false);
		extract.filter (*planeInliersCloud);
	}
	
	std::cout<<"Project inliers to plane & calculate convex hull done: "<<std::endl;
	
	std::cout<<"number of outliers "<<objectIndices->indices.size()<<std::endl;
	
		
	PointCloud::Ptr limitedObjectPoints(new PointCloud);
	// Limit by convex hull
	{
		pcl::PointIndices::Ptr limitedObjectIndices(new pcl::PointIndices);
		
		pcl::ExtractPolygonalPrismData<Point> limiter;
		limiter.setInputPlanarHull(planeHull);
		limiter.setInputCloud(cloud);
		limiter.setIndices(objectIndices);
		limiter.segment(*limitedObjectIndices);
		pcl::ExtractIndices<Point> extract;
		extract.setInputCloud(cloud);
		extract.setIndices(limitedObjectIndices);
		extract.filter(*limitedObjectPoints);
		objectIndices.swap(limitedObjectIndices);
	}
	
	std::cout<<"number of limitedObjectPoints "<<limitedObjectPoints->size()<<std::endl;
	// Visualize 
	{
		pcl::visualization::PCLVisualizer viewer ("ICP visualization");

		viewer.addPolygon<pcl::PointXYZ>(planeHull, 255.0, 0.0, 0.0, "plane");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> planeInliersCloud_handler (planeInliersCloud, 0, 0, 255);
		viewer.addPointCloud (planeInliersCloud, planeInliersCloud_handler, "planeInliersCloud");
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> limitedObjectPoints_handler (limitedObjectPoints, 0, 255, 0);
		viewer.addPointCloud (limitedObjectPoints, limitedObjectPoints_handler, "limitedObjectPoints");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "limitedObjectPoints");

		//pcl::visualization::Camera camera;
		
		std::vector<pcl::visualization::Camera> camera;
		viewer.getCameras(camera);
		
		camera[0].focal[0] = 0.0; camera[0].focal[1] = 0.0; camera[0].focal[2] = 0.15; // Look at this point
		camera[0].pos[0] = 0.0; camera[0].pos[1] = 0.0; camera[0].pos[2] = 0.0;        // camera is at this point
		camera[0].view[0] = 0.0; camera[0].view[1] = -1.0; camera[0].view[2] = 0.0;    // This is the "up" direction
		camera[0].fovy = 45.0 * M_PI / 180.0;

		viewer.setCameraParameters(camera[0]);

		viewer.setBackgroundColor(255.0, 255.0, 255.0);

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}
		
		viewer.close();
		viewer.spinOnce();
	}
	
	//Euclidean Clustering
	std::vector<Cluster> clusters;
	//ColorPointCloud::Ptr segmentationViz (new ColorPoint);
	std::vector<pcl::PointIndices> clusterIndices;
	{
		std::vector<pcl::PointIndices> clusterIndices;

		PointCloud::Ptr organizedMaskedCloud(new PointCloud);
		Point invalid;
		invalid.x = NAN;

		organizedMaskedCloud->points.resize(cloud->size(), invalid);
		organizedMaskedCloud->width = cloud->width;
		organizedMaskedCloud->height = cloud->height;

		std::cout<<"object points: "<< objectIndices->indices.size()<<std::endl;
		for(std::size_t i = 0; i < objectIndices->indices.size(); ++i)
		{
			int idx = objectIndices->indices[i];
			(*organizedMaskedCloud)[idx] = (*cloud)[idx];
		}

		pcl::PointCloud<pcl::Label>::Ptr labels(new pcl::PointCloud<pcl::Label>);

		boost::shared_ptr<ClusterComparator> comp(new ClusterComparator(0.03));
		comp->setInputCloud(organizedMaskedCloud);

		pcl::OrganizedConnectedComponentSegmentation<Point, pcl::Label> segm(comp);
		segm.setInputCloud(organizedMaskedCloud);
		segm.segment(*labels, clusterIndices);

		for(std::size_t i = 0; i < clusterIndices.size(); ++i)
		{
			pcl::PointIndices& indices = clusterIndices[i];

			if(indices.indices.size() < 50)
				continue;

			Cluster cluster;
			cluster.indices.reset(new pcl::PointIndices);
			cluster.indices->indices.swap(indices.indices);
			
			clusters.push_back(cluster);
			std::cout<<"segmentation found "<<clusterIndices.size()
					 <<" clusters, we took "<<  clusters.size() <<std::endl;
		}
	}
	
	
	/*
	pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
	tree->setInputCloud (limitedObjectPoints);
	
	std::vector<pcl::PointIndices> cluster_indices;
	 pcl::EuclideanClusterExtraction<Point> ec;
	ec.setClusterTolerance (0.01);
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (3500);
	ec.setSearchMethod (tree);
	ec.setInputCloud (limitedObjectPoints);
	ec.extract (cluster_indices);
	
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
		PointCloud::Ptr cloud_cluster (new PointCloud);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (limitedObjectPoints->points[*pit]);
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		j++;
	}
	*/

	return 0;
}
