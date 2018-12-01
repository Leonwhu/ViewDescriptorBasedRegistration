#include "viewDescriptor.h"

#include <iostream> 
#include <fstream>
#include <opencv2/core/core.hpp> 
#include<opencv2/highgui/highgui.hpp> 
using namespace cv;

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
		//�趨С����С����ĵ㲻�������
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
		else if (viewDepth[count] < distance)
			viewDepth[count] = distance;
	}
	calculateSVFValue();
	convert2DImage();
	return 1;
}

void ViewDescriptor::calculateSVFValue()
{
	//float svf = 0.0;
	int num_empty = 0;
	//����ӳ�����ֻ�����ϰ�����ڵ�
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
		cout << "��������ӳ�����ʧ�ܣ�������������2D���ͼ��" << endl;
		return;
	}
	//float svf = 0.0;
	int num_empty = 0;
	//����ӳ�����ֻ�����ϰ�����ڵ�
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
				//viewDepth�洢��Nv=90�ȿ�ʼ����ά��ʾ�ӣ�0,0����ʼ��y������
				ofs << j << " " << Nv-i << " " << viewDepth[count] << endl;
			}
		}
	}
	ofs.close();
}

void ViewDescriptor::outputViewDescriptor2DImage(const string &filename)
{
	//Mat M(Nv, Nh, CV_8UC1);//����һ���Ҷ�ͼ��Mat����
	//for (int i = 0; i < M.rows; i++)        //����ÿһ��ÿһ�в�����������ֵ
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
	//Mat M(Nv, Nh, CV_32FC1);//����һ���Ҷ�ͼ��Mat����
	Mat M(Nv, Nh, CV_8UC1);//����һ���Ҷ�ͼ��Mat����
	for (int i = 0; i < M.rows; i++)        //����ÿһ��ÿһ�в�����������ֵ
	{
		for (int j = 0; j < M.cols; j++)
		{
			//��Ⱦ���黯��ָ��ֵ
			float distance = 100.0;
			M.at<uchar>(i, j) = min(viewDepth[j + i*Nh] - minDist, distance) * 255.0 / distance;
		}
	}
	this->depthimage = M;
}

void ViewDescriptor::convert2DImage(float &maxDist2Grey)
{
	//Mat M(Nv, Nh, CV_32FC1);//����һ���Ҷ�ͼ��Mat����
	Mat M(Nv, Nh, CV_8UC1);//����һ���Ҷ�ͼ��Mat����
	for (int i = 0; i < M.rows; i++)        //����ÿһ��ÿһ�в�����������ֵ
	{
		for (int j = 0; j < M.cols; j++)
		{
			//��Ⱦ���黯��ָ��ֵ
			float distance = 100.0;
			M.at<uchar>(i, j) = min(viewDepth[j + i*Nh] - minDist, distance) * maxDist2Grey / distance;
		}
	}
	this->depthimage = M;
}