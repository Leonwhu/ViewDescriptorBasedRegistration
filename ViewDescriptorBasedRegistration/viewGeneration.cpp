#include "viewGeneration.h"
#include <pcl/filters/voxel_grid.h>

#include <vector>

void ViewGeneration::getCloudBound(const pcl::PointCloud<pcl::PointXYZ> &cloud, Bounds & bound){
	double min_x = cloud.points[0].x;
	double min_y = cloud.points[0].y;
	double min_z = cloud.points[0].z;
	double max_x = cloud.points[0].x;
	double max_y = cloud.points[0].y;
	double max_z = cloud.points[0].z;

	for (int i = 0; i<cloud.size(); i++){
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
	bound.min_x = min_x;
	bound.max_x = max_x;
	bound.min_y = min_y;
	bound.max_y = max_y;
	bound.min_z = min_z;
	bound.max_z = max_z;
}

bool ViewGeneration::getViewsFromALS_centroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALS, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudViews, float resolution)
{
	//选取每列体素中最低的点
	Bounds bound;
	getCloudBound(*cloudALS, bound);
	int nx = ceil((bound.max_x - bound.min_x) / resolution);
	int ny = ceil((bound.max_y - bound.min_y) / resolution);
	vector<int> gridIndex;
	gridIndex.resize(nx*ny, -1);
	/*vector<vector<int>> big_grid;
	vector<pair<int, int>> has_grid_index;*/
	//对其分类;
	for (int i = 0; i < cloudALS->points.size(); i++)
	{
		int tx = floor((cloudALS->points[i].x - bound.min_x) / resolution);
		int ty = floor((cloudALS->points[i].y - bound.min_y) / resolution);
		int count = tx + ty*nx;

		if (gridIndex[count]==-1)
			gridIndex[count] = i;
		else
		{
			if ((cloudALS->points[count].z - cloudALS->points[gridIndex[count]].z)<0.0)
				gridIndex[count] = i;
		}		
	}

	for (int i = 0; i < nx; ++i)
	{
		for (int j = 0; j < ny; ++j)
		{
			int count = i + j*nx;
			if (gridIndex[count] != -1)
			{
				//cloudViews->push_back(cloudALS->points[gridIndex[i]]);
				pcl::PointXYZ pt;
				pt.x = bound.min_x + (i - 0.5)*resolution;
				pt.y = bound.min_y + (j - 0.5)*resolution;
				pt.z = cloudALS->points[gridIndex[count]].z;				
				cloudViews->push_back(pt);
			}
		}
	}
	return 1;
}