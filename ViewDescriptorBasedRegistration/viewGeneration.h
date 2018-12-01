#pragma once
#include "utility.h"

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

using namespace std;
using namespace utility;

class ViewGeneration
{
public:
	
	void getCloudBound(const pcl::PointCloud<pcl::PointXYZ> &cloud, Bounds & bound);

	//����ALS���ƣ������ӽǵ㣬���ø�������
	bool getViewsFromALS_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALS, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudViews, float resolution);
protected:
private:
};