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
		
		// Axis-aligned bounding box
		Eigen::Vector4f aa_center;
		Eigen::Vector4f aa_extent;
		Eigen::Vector4f aa_min;
		Eigen::Vector4f aa_max;
	};

static const bool VISUALIZE = true;
static const float CLUSTER_CLOSENESS = 0.005f;
static const float COMPARATOR_THRESHOLD = 0.1f;
static const int CLUSTER_MIN_POINTS = 5;

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
	
	if (VISUALIZE)
	{
		pcl::visualization::PCLVisualizer viewer ("ICP visualization");
		//pcl::visualization::Camera camera;
		std::vector<pcl::visualization::Camera>  camera;
		viewer.getCameras(camera);
		
		viewer.addPolygon<pcl::PointXYZ>(planeHull, 255.0, 0.0, 0.0, "plane");
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> planeInliersCloud_handler (planeInliersCloud, 0, 0, 127);
		viewer.addPointCloud (planeInliersCloud, planeInliersCloud_handler, "planeInliersCloud");
	
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> limitedObjectPoints_handler (limitedObjectPoints, 0, 255, 0);
		viewer.addPointCloud (limitedObjectPoints, limitedObjectPoints_handler, "limitedObjectPoints");
 
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "limitedObjectPoints");
		
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

		boost::shared_ptr<ClusterComparator> comp(new ClusterComparator(COMPARATOR_THRESHOLD));
		comp->setInputCloud(organizedMaskedCloud);
		pcl::OrganizedConnectedComponentSegmentation<Point, pcl::Label> segm(comp);
		segm.setInputCloud(organizedMaskedCloud);
		segm.segment(*labels, clusterIndices);

		for(std::size_t i = 0; i < clusterIndices.size(); ++i)
		{
			pcl::PointIndices& indices = clusterIndices[i];

			if(indices.indices.size() < CLUSTER_MIN_POINTS)
				continue;

			Cluster cluster;
			cluster.indices.reset(new pcl::PointIndices);
			cluster.indices->indices.swap(indices.indices);
			
			// calculate axis aligned bounding box
			Eigen::Vector4f min, max;
			pcl::getMinMax3D(*cloud, *cluster.indices, cluster.aa_min, cluster.aa_max);
			cluster.aa_center = (cluster.aa_max + cluster.aa_min)/2;
			cluster.aa_extent = (cluster.aa_max - cluster.aa_min)/2;
			
			clusters.push_back(cluster);
			
			uint32_t color = 0;
			switch(clusters.size() % 3)
			{
				case 0: color = 0xFF0000; break;
				case 1: color = 0x00FF00; break;
				case 2: color = 0x0000FF; break;
			}

			for(std::size_t j = 0; j < cluster.indices->indices.size(); ++j)
			{
				int idx = cluster.indices->indices[j];
				const Point& p = (*cloud)[idx];

				pcl::PointXYZRGB out;
				out.getVector4fMap() = p.getVector4fMap();
				out.rgba = color;
			}
		}
		std::cout<<"segmentation found "<<clusterIndices.size()
					 <<" clusters, we took "<<  clusters.size() <<std::endl;
		for (std::size_t i =0; i < clusters.size(); i++){
			std::cout<<i<<" cluster with size "<<clusters[i].indices->indices.size()<<std::endl;
			
		}
	}
	

	float clusterCloseness = 0.005f;
// 	std::cout<<"Enter cluster closeness "<<std::endl;
// 	std::cin>>clusterCloseness;
	
	// Merge close clusters
	{
		for(std::size_t i = 0; i < clusters.size(); ++i)
		{
			for(std::size_t j = i+1; j < clusters.size();)
			{
				Cluster* current = &clusters[i];
				Cluster* compare = &clusters[j];

				bool touch[3];
// 				for(int axis = 0; axis < 3; ++axis)
// 				{
// 					float dist = fabsf(compare->aa_center[axis] - current->aa_center[axis]);
// 					touch[axis] = dist - current->aa_extent[axis] - compare->aa_extent[axis] < clusterCloseness;
// 				}
				float dist = fabsf(compare->aa_center[1] - current->aa_center[1]);
				touch[1] = dist - current->aa_extent[1] - compare->aa_extent[1] < CLUSTER_CLOSENESS;
				if(touch[1])  // touch[0] && touch[1] && touch[2]
				{
					std::cout<<"Merging "<<std::endl;
					std::copy(
						compare->indices->indices.begin(), compare->indices->indices.end(),
						std::back_inserter(current->indices->indices)
					);

					for(int axis = 0; axis < 3; ++axis)
					{
						current->aa_max[axis] = std::max(current->aa_max[axis], compare->aa_max[axis]);
						current->aa_min[axis] = std::min(current->aa_min[axis], compare->aa_min[axis]);
					}
					current->aa_center = (current->aa_max + current->aa_min)/2;
					current->aa_extent = (current->aa_max - current->aa_min)/2;

					clusters.erase(clusters.begin() + j);
				}
				else
					++j;
			}
		}
	}
	
	{
		//after merging
		std::cout<<"# of clusters after merging "<<clusters.size()<<std::endl;
		for (std::size_t i =0; i <clusters.size(); ++i){
			std::cout<<"Cluster "<<i <<" with points "<<clusters[i].indices->indices.size()<<std::endl;
		}
	}
	

	//visualize the clusters
	{
		std::vector<PointCloud::Ptr> ClustersCloud (3);
		
		pcl::ExtractIndices<Point> extract;
		extract.setInputCloud (cloud);
		
		for (std::size_t i =0; i <clusters.size(); ++i){
			ClustersCloud[i].reset(new PointCloud());
			extract.setIndices (clusters[i].indices);
			extract.setNegative (false);
			extract.filter (*ClustersCloud[i]);
		}
		
		pcl::visualization::PCLVisualizer viewer ("ICP visualization");

		viewer.addPolygon<pcl::PointXYZ>(planeHull, 255.0, 0.0, 0.0, "plane");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> planeInliersCloud_handler (planeInliersCloud, 0, 0, 127);
		viewer.addPointCloud (planeInliersCloud, planeInliersCloud_handler, "planeInliersCloud");
		
// 		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> limitedObjectPoints_handler (limitedObjectPoints, 0, 127, 0);
// 		viewer.addPointCloud (limitedObjectPoints, limitedObjectPoints_handler, "limitedObjectPoints");
// 		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "limitedObjectPoints");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c1_handler (limitedObjectPoints, 0, 255, 0);
		viewer.addPointCloud (ClustersCloud[0], c1_handler, "c1");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "c1");
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c2_handler (limitedObjectPoints, 255, 0, 0);
		viewer.addPointCloud (ClustersCloud[1], c2_handler, "c2");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "c2");
		
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> c3_handler (limitedObjectPoints, 0, 0, 255);
		viewer.addPointCloud (ClustersCloud[2], c3_handler, "c3");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "c3");
		
		//pcl::visualization::Camera camera;
		std::vector<pcl::visualization::Camera>  camera;
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
		pcl::PCDWriter writer;
		
		writer.writeBinary("cluster0.pcd", *ClustersCloud[0]);
		writer.writeBinary("cluster1.pcd", *ClustersCloud[1]);
		writer.writeBinary("cluster2.pcd", *ClustersCloud[2]);
	}
	
	return 0;
}
