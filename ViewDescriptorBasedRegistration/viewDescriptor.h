#pragma  once

#include "utility.h"


#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include <iostream> 
#include <opencv2/core/core.hpp> 
#include<opencv2/highgui/highgui.hpp> 

using namespace std;
using namespace utility;

class ViewDescriptor
{
public:

	//给定观测点，点云，角度分辨率，生成对应的深度图
	void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){ cloud = input_cloud; };
	void setViewPoint(pcl::PointXYZ viewpt){ viewPoint = viewpt; };
	void setMinDist(float minDistance){ minDist = minDistance; };
	void setMaxDist(float maxDistance){ maxDist = maxDistance; };
	//void setDescriptor(vector<float> *descriptor){ descriptor = descriptor; };
	void setResolutions(float angularResolutionV, float angularResolutionH){
		angResV = angularResolutionV; angResH = angularResolutionH; Nv = int(180.0 / angResV); Nh = int(360.0 / angResH); 
	};
	void setNvNh(int Nvertical, int Nhorizontal){ Nv = Nvertical; Nh = Nhorizontal; };
	
	bool generateDescriptor();
	bool generateDescriptorByKDTree(std::vector<int> &pointIdxRadiusSearch);
	void calculateSVFValue();
	void calculateSVFValue2DImage();

	float getSVFValue(){ return svf; };
	void convert2DImage();
	void convert2DImage(float maxDist2Grey);
	void outputViewDescriptor3DImage(const string &filename);
	void outputViewDescriptor2DImage(const string &filename);
	
	void filterNoiseBy2DDensity(int radius, int minNum);


	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointXYZ viewPoint;
	cv::Mat depthimage;
	vector<float> viewDepth;
	float angResV;
	float angResH;
	float minDist;
	float maxDist;
	int Nv;
	int Nh;
	float svf=0.0;

protected:
private:
};
