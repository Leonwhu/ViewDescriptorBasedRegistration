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
		//�趨С����С���룬��ˮƽ�������������ĵ㲻�������
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
		//�趨С����С���룬��ˮƽ�������������ĵ㲻�������
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
			M.at<uchar>(i, j) = min(viewDepth[j + i*Nh] - minDist, distance) * 200.0 / distance;
		}
	}
	this->depthimage = M;
}

void ViewDescriptor::convert2DImage(float maxDist2Grey)
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

void ViewDescriptor::filterNoiseBy2DDensity(int radius, int minNum)
{
	if (this->Nv ==0 || this->Nh ==0)
	{
		cout << "δָ�������ֱ��ʣ�" << endl;
		return;
	}
	vector<int> pDensity;
	for (int i = 0;i < this->Nv;++i)
	{
		for (int j = 0; j < this->Nh; ++j)
		{
			//����radius��Χ�ڵ���������
			int numNeigh = 0; //�������
			int numHasPoint = 0; //�����еĵ����
			for (int m = i - radius; m < i + radius; ++m)
			{				
				for (int n = j - radius; n < j + radius; ++n)
				{
					//��ֱ���������½�,ˮƽ����߽�ѭ��
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
			cutSkyline.pContours[j].depth = skyline.pContours[j].depth;//���޸Ĺ�������߽߱磬���ֵ��Ϣ�Ѳ��ɿ����ϱ߽籣��ԭ���
		}
		else if (skyline.pContours[j].zenithAngle > NvMax)
		{
			cutSkyline.pContours[j].zenithAngle = NvMax;
			cutSkyline.pContours[j].depth = -1.0;//���޸Ĺ�������߽߱磬���ֵ��Ϣ�Ѳ��ɿ����±߽縳Ϊ��ֵ��ʾ������
		}
		else
		{
			cutSkyline.pContours[j].zenithAngle = skyline.pContours[j].zenithAngle;
			cutSkyline.pContours[j].depth = skyline.pContours[j].depth;
		}
	}
	return cutSkyline;
}