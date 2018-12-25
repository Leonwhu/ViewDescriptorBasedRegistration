#include "viewDescriptor.h"

#include <iostream> 
#include <fstream>
#include <opencv2/core/core.hpp> 
#include<opencv2/highgui/highgui.hpp> 
using namespace cv;
using namespace utility;

#define RAD_TO_DEG (180/(4*atan(1)))//pi=4*atan(1)  
#define DEG_TO_RAD ((4*atan(1))/180)

bool ViewDescriptor::generateDescriptor()
{	
	viewDepth.resize(Nv*Nh, -1.0);
	
	float dx, dy, dz, distance;
	float lati, longi;
	int nx, ny, count;
	for (int i = 0; i < cloud->points.size(); ++i)
	{		
		dx = cloud->points[i].x - viewPoint.x;
		dy = cloud->points[i].y - viewPoint.y;
		dz = cloud->points[i].z - viewPoint.z;
		distance = sqrt(dx*dx + dy*dy + dz*dz);
		float horDist = sqrt(dx*dx + dy*dy);
		//设定小于最小距离，或水平方向大于最大距离的点不参与计算
		if (distance <minDist || horDist >maxDist)
			continue;
		lati = 90.0 - RAD_TO_DEG * asin(dz / distance);
		longi = 90.0 - RAD_TO_DEG * atan2(dy, dx);
		if (longi < 0.0) longi += 360.0;
		nx = int(lati / angResV);
		ny = int(longi / angResH);
		count = ny + nx*Nh;
		if (viewDepth[count] < 0.0)
			viewDepth[count] = distance;
		else if (viewDepth[count] > distance)
			viewDepth[count] = distance;
	}
	calculateSVFValue();
	convert2DImage(200.0f);
	return 1;
}

bool ViewDescriptor::generateDescriptorByKDTree(std::vector<int> &pointIdxRadiusSearch)
{
	viewDepth.resize(Nv*Nh, -1.0);

	float dx, dy, dz, distance;
	float lati, longi;
	int nx, ny, count;
	for (int i = 0; i < pointIdxRadiusSearch.size(); ++i)
	{
		dx = cloud->points[pointIdxRadiusSearch[i]].x - viewPoint.x;
		dy = cloud->points[pointIdxRadiusSearch[i]].y - viewPoint.y;
		dz = cloud->points[pointIdxRadiusSearch[i]].z - viewPoint.z;
		distance = sqrt(dx*dx + dy*dy + dz*dz);
		//设定小于最小距离，或水平方向大于最大距离的点不参与计算
		if (distance <minDist)
			continue;
		lati = 90.0 - RAD_TO_DEG * asin(dz / distance);
		longi = 90.0 - RAD_TO_DEG * atan2(dy, dx);
		if (longi < 0.0) longi += 360.0;
		nx = int(lati / angResV);
		ny = int(longi / angResH);
		count = ny + nx*Nh;
		if (viewDepth[count] < 0.0)
			viewDepth[count] = distance;
		else if (viewDepth[count] > distance)
			viewDepth[count] = distance;
	}
	calculateSVFValue();
	convert2DImage(200.0f);
	return 1;
}

void ViewDescriptor::calculateSVFValue()
{
	//float svf = 0.0;
	int num_empty = 0;
	//天空视场因子只考虑上半球的遮挡
	for (int i = 0; i < Nv / 2; ++i)
	{
		for (int j = 0; j < Nh; ++j)
		{
			int count = j + i*Nh;
			if (viewDepth[count] < 0.0)
				num_empty++;
		}
	}
	svf = float(2 * num_empty) / Nv / Nh;
	return;
}

void ViewDescriptor::calculateSVFValue2DImage()
{
	if (this->depthimage.empty())
	{
		cout << "计算天空视场因子失败，特征描述子无2D深度图！" << endl;
		return;
	}
	//float svf = 0.0;
	int num_empty = 0;
	//天空视场因子只考虑上半球的遮挡
	for (int j = 0; j < this->depthimage.cols; ++j)
	{
		for (int i = 0; i < this->depthimage.rows/2; ++i)
		{
			if (this->depthimage.at<uchar>(i,j)!=255)
			{
				num_empty += (depthimage.rows / 2 - i);
			}
		}
	}
	svf = float(2 * num_empty) / Nv / Nh;
	return;
}

void ViewDescriptor::outputViewDescriptor3DImage(const string &filename)
{	
	ofstream ofs;
	ofs.open(filename);
	if (ofs.is_open())
	{
		for (int i = 0; i < Nv; ++i)
		{
			for (int j = 0; j < Nh; ++j)
			{
				int count = j + i*Nh;
				if (viewDepth[count] < 0.0) continue;
				//viewDepth存储从Nv=90度开始，三维显示从（0,0）开始，y轴向上
				ofs << j << " " << Nv-i << " " << viewDepth[count] << endl;
			}
		}
	}
	ofs.close();
}

void ViewDescriptor::outputViewDescriptor3DSkyline(const string &filename)
{
	ofstream ofs;
	ofs.open(filename);
	if (ofs.is_open())
	{
		for (int j = 0; j < Nh; ++j)
		{
			ofs << j << " " << Nv - skyline.pContours[j].zenithAngle << " " << skyline.pContours[j].depth << endl;
		}
	}
	ofs.close();
}

void ViewDescriptor::outputViewDescriptor2DImage(const string &filename)
{
	//Mat M(Nv, Nh, CV_8UC1);//创建一个灰度图的Mat对象
	//for (int i = 0; i < M.rows; i++)        //遍历每一行每一列并设置其像素值
	//{
	//	for (int j = 0; j < M.cols; j++)
	//	{
	//		M.at<uchar>(i, j) = min(viewDepth[j+i*Nh]-minDist, 100.0f)*255/100;
	//	}
	//}
	imwrite(filename, this->depthimage);
}

void ViewDescriptor::convert2DImage()
{
	//Mat M(Nv, Nh, CV_32FC1);//创建一个灰度图的Mat对象
	Mat M(Nv, Nh, CV_8UC1);//创建一个灰度图的Mat对象
	for (int i = 0; i < M.rows; i++)        //遍历每一行每一列并设置其像素值
	{
		for (int j = 0; j < M.cols; j++)
		{
			//深度距离归化到指定值
			float distance = 100.0;
			M.at<uchar>(i, j) = min(viewDepth[j + i*Nh] - minDist, distance) * 200.0 / distance;
		}
	}
	this->depthimage = M;
}

void ViewDescriptor::convert2DImage(float maxDist2Grey)
{
	//Mat M(Nv, Nh, CV_32FC1);//创建一个灰度图的Mat对象
	Mat M(Nv, Nh, CV_8UC1);//创建一个灰度图的Mat对象
	for (int i = 0; i < M.rows; i++)        //遍历每一行每一列并设置其像素值
	{
		for (int j = 0; j < M.cols; j++)
		{
			//深度距离归化到指定值
			float distance = 100.0;
			M.at<uchar>(i, j) = min(viewDepth[j + i*Nh] - minDist, distance) * maxDist2Grey / distance;
		}
	}
	this->depthimage = M;
}

void ViewDescriptor::filterNoiseBy2DDensity(int radius, int minNum)
{
	if (this->Nv ==0 || this->Nh ==0)
	{
		cout << "未指定特征分辨率！" << endl;
		return;
	}
	vector<int> pDensity;
	for (int i = 0;i < this->Nv;++i)
	{
		for (int j = 0; j < this->Nh; ++j)
		{
			//遍历radius范围内的所有邻域
			int numNeigh = 0; //邻域个数
			int numHasPoint = 0; //邻域中的点个数
			for (int m = i - radius; m < i + radius; ++m)
			{				
				for (int n = j - radius; n < j + radius; ++n)
				{
					//垂直方向有上下界,水平方向边界循环
					if (m >= 0 && m < this->Nv )
					{
						int t;
						if (n >= this->Nh)
							t = n - this->Nh;
						else if (n < 0)
							t = n + this->Nh;
						else
							t = n;
						numNeigh++;
						if (this->viewDepth[t + m*this->Nh] > 0.0)
							numHasPoint++;

					}
				}
			}
			if (numHasPoint <= minNum)
				this->viewDepth[j + i*this->Nh] = -1.0;
		}
	}
}

void ViewDescriptor::generateSkyline()
{
	skyline.Nh = Nh;
	skyline.pContours.resize(skyline.Nh);
	for (int j = 0; j < Nh; ++j)
	{
		skyline.pContours[j].zenithAngle = Nv;
		skyline.pContours[j].depth = -1.0;
		for (int i = 0; i < Nv; ++i)
		{
			if (viewDepth[j + i*Nh] > 0.0)
			{
				skyline.pContours[j].zenithAngle = i;
				skyline.pContours[j].depth = viewDepth[j + i*Nh];
				break;
			}
		}
	}
}

void ViewDescriptor::generateSkylineWithScanAngle(int NvMin, int NvMax)
{
	skyline.Nh = Nh;
	skyline.pContours.resize(skyline.Nh);
	for (int j = 0; j < Nh; ++j)
	{
		skyline.pContours[j].zenithAngle = NvMax;
		skyline.pContours[j].depth = -1.0;
		for (int i = NvMin; i <= NvMax; ++i)
		{
			if (viewDepth[j + i*Nh] > 0.0)
			{
				skyline.pContours[j].zenithAngle = i;
				skyline.pContours[j].depth = viewDepth[j + i*Nh];
				break;
			}
		}
	}
}

Skyline3DContour ViewDescriptor::cutSkylineWithScanAngle()
{
	Skyline3DContour cutSkyline;
	cutSkyline.Nh = Nh;
	cutSkyline.pContours.resize(skyline.Nh);
	for (int j = 0; j < Nh; ++j)
	{
		if (skyline.pContours[j].zenithAngle < NvMin)
		{
			cutSkyline.pContours[j].zenithAngle = NvMin;
			cutSkyline.pContours[j].depth = skyline.pContours[j].depth;//被修改过的天际线边界，深度值信息已不可靠，上边界保留原深度
		}
		else if (skyline.pContours[j].zenithAngle > NvMax)
		{
			cutSkyline.pContours[j].zenithAngle = NvMax;
			cutSkyline.pContours[j].depth = -1.0;//被修改过的天际线边界，深度值信息已不可靠，下边界赋为负值表示无意义
		}
		else
		{
			cutSkyline.pContours[j].zenithAngle = skyline.pContours[j].zenithAngle;
			cutSkyline.pContours[j].depth = skyline.pContours[j].depth;
		}
	}
	return cutSkyline;
}