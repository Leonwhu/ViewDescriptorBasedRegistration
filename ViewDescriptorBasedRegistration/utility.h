#ifndef UTILITY
#define UTILITY

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <ppl.h>
#include <concurrent_vector.h>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image_spherical.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/common/float_image_utils.h>
#include <pcl/io/png_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/contrib/contrib.hpp>

#define GLOG_NO_ABBREVIATED_SEVERITIES
#include <glog/logging.h>

#include "pcl/point_cloud.h"


typedef  pcl::PointCloud<pcl::PointXYZI>::Ptr      pcXYZIPtr;
typedef  pcl::PointCloud<pcl::PointXYZI>            pcXYZI;

typedef  pcl::PointCloud<pcl::PointXYZ>::Ptr      pcXYZPtr;
typedef  pcl::PointCloud<pcl::PointXYZ>            pcXYZ;

typedef  pcl::PointCloud<pcl::PointXY>::Ptr       pcXYPtr;
typedef  pcl::PointCloud<pcl::PointXY>            pcXY;

typedef  pcl::PointCloud<pcl::PointXYZRGB>::Ptr      pcXYZRGBPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGB>           pcXYZRGB;

typedef  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr       pcXYZRGBAPtr;
typedef  pcl::PointCloud<pcl::PointXYZRGBA>            pcXYZRGBA;

typedef pcl::PointCloud<pcl::Normal>::Ptr NormalsPtr;
typedef pcl::PointCloud<pcl::Normal> Normals;


typedef concurrency::concurrent_vector<pcXYZI>  conVectorPCXYZI;
typedef std::vector<pcXYZI>  vectorPCXYZI;

namespace utility
{
	struct pointCloudBound
	{
		double minx;
		double maxx;
		double miny;
		double maxy;
		double minz;
		double maxz;
		pointCloudBound()
		{
			minx = maxx = miny = maxy = minz = maxz = 0.0;
		}
	};

	struct pointCloudBound2d
	{
		double minx;
		double maxx;
		double miny;
		double maxy;
		pointCloudBound2d()
		{
			minx = maxx = miny = maxy = 0.0;
		}
	};

	struct CenterPoint
	{
		double x;
		double y;
		float z;
	};

	struct Bounds
	{
		double min_x;
		double min_y;
		double min_z;
		double max_x;
		double max_y;
		double max_z;
	};


	template<typename PointCloudType>
	void getCloudBound(const PointCloudType & cloud, pointCloudBound & bound)
	{
		double min_x = DBL_MAX;
		double min_y = DBL_MAX;
		double min_z = DBL_MAX;
		double max_x = -DBL_MAX;
		double max_y = -DBL_MAX;
		double max_z = -DBL_MAX;

		for (size_t i = 0; i<cloud.size(); ++i)
		{
			//获取边界
			if (min_x>cloud.points[i].x)
				min_x = cloud.points[i].x;
			if (min_y > cloud.points[i].y)
				min_y = cloud.points[i].y;
			if (min_z > cloud.points[i].z)
				min_z = cloud.points[i].z;
			if (max_x < cloud.points[i].x)
				max_x = cloud.points[i].x;
			if (max_y < cloud.points[i].y)
				max_y = cloud.points[i].y;
			if (max_z < cloud.points[i].z)
				max_z = cloud.points[i].z;
		}
		bound.minx = min_x;
		bound.maxx = max_x;
		bound.miny = min_y;
		bound.maxy = max_y;
		bound.minz = min_z;
		bound.maxz = max_z;
	}

	struct IdSimilarity
	{
		size_t pcIndex;
		float similarity;
		IdSimilarity()
		{
			pcIndex = 1;
			similarity = 0.0f;
		}
	};
	struct ContourPoint
	{
		int highAngle;
		float depth;
		float getDistanceGaussianWeight(ContourPoint &p2)
		{
			float rDepth = 5.0;
			//如果该方向为空，不计算相似度
			if (this->depth < 0.0 || p2.depth < 0.0)
				return 0.0;

			//高斯函数赋权
			float gWeight[4] = {1.0,0.895,0.641,0.368};
			int dAngle = abs(this->highAngle - p2.highAngle);
			float wAngle, wDepth;
			if (dAngle > 3)
				return 0.0;
			else
				wAngle = gWeight[dAngle];
			/*else if (dAngle == 0)
				wAngle = 1.0;
			else if (dAngle == 1)
				wAngle = 0.895;
			else if (dAngle == 2)
				wAngle = 0.641;
			else if (dAngle ==3)
				wAngle = 0.368;*/
			float dDepth = this->depth - p2.depth;
			wDepth = exp(0.0 - dDepth*dDepth / rDepth / rDepth);			
			return wAngle*wDepth;
		}
	};
	
}



#endif