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
		float horDist = sqrt(dx*dx + dy*dy);
		//Éè¶¨Ğ¡ÓÚ×îĞ¡¾àÀë£¬»òË®Æ½·½Ïò´óÓÚ×î´ó¾àÀëµÄµã²»²ÎÓë¼ÆËã
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
		//Éè¶¨Ğ¡ÓÚ×îĞ¡¾àÀë£¬»òË®Æ½·½Ïò´óÓÚ×î´ó¾àÀëµÄµã²»²ÎÓë¼ÆËã
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
	//Ìì¿ÕÊÓ³¡Òò×ÓÖ»¿¼ÂÇÉÏ°ëÇòµÄÕÚµ²
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
		cout << "¼ÆËãÌì¿ÕÊÓ³¡Òò×ÓÊ§°Ü£¬ÌØÕ÷ÃèÊö×ÓÎŞ2DÉî¶ÈÍ¼£¡" << endl;
		return;
	}
	//float svf = 0.0;
	int num_empty = 0;
	//Ìì¿ÕÊÓ³¡Òò×ÓÖ»¿¼ÂÇÉÏ°ëÇòµÄÕÚµ²
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
				//viewDepth´æ´¢´ÓNv=90¶È¿ªÊ¼£¬ÈıÎ¬ÏÔÊ¾´Ó£¨0,0£©¿ªÊ¼£¬yÖáÏòÉÏ
				ofs << j << " " << Nv-i << " " << viewDepth[count] << endl;
			}
		}
	}
	ofs.close();
}

<<<<<<< HEAD
<<<<<<< HEAD
=======
void ViewDescriptor::outputViewDescriptorSkyline(const string &filename)
{

}

>>>>>>> parent of 34153d8... DPSkylineMinimal
=======
>>>>>>> parent of 2d88375... å¢åŠ DPç®—æ³•è®¡ç®—ç›¸ä¼¼åº¦
void ViewDescriptor::outputViewDescriptor2DImage(const string &filename)
{
	//Mat M(Nv, Nh, CV_8UC1);//´´½¨Ò»¸ö»Ò¶ÈÍ¼µÄMat¶ÔÏó
	//for (int i = 0; i < M.rows; i++)        //±éÀúÃ¿Ò»ĞĞÃ¿Ò»ÁĞ²¢ÉèÖÃÆäÏñËØÖµ
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
	//Mat M(Nv, Nh, CV_32FC1);//´´½¨Ò»¸ö»Ò¶ÈÍ¼µÄMat¶ÔÏó
	Mat M(Nv, Nh, CV_8UC1);//´´½¨Ò»¸ö»Ò¶ÈÍ¼µÄMat¶ÔÏó
	for (int i = 0; i < M.rows; i++)        //±éÀúÃ¿Ò»ĞĞÃ¿Ò»ÁĞ²¢ÉèÖÃÆäÏñËØÖµ
	{
		for (int j = 0; j < M.cols; j++)
		{
			//Éî¶È¾àÀë¹é»¯µ½Ö¸¶¨Öµ
			float distance = 100.0;
			M.at<uchar>(i, j) = min(viewDepth[j + i*Nh] - minDist, distance) * 200.0 / distance;
		}
	}
	this->depthimage = M;
}

void ViewDescriptor::convert2DImage(float maxDist2Grey)
{
	//Mat M(Nv, Nh, CV_32FC1);//´´½¨Ò»¸ö»Ò¶ÈÍ¼µÄMat¶ÔÏó
	Mat M(Nv, Nh, CV_8UC1);//´´½¨Ò»¸ö»Ò¶ÈÍ¼µÄMat¶ÔÏó
	for (int i = 0; i < M.rows; i++)        //±éÀúÃ¿Ò»ĞĞÃ¿Ò»ÁĞ²¢ÉèÖÃÆäÏñËØÖµ
	{
		for (int j = 0; j < M.cols; j++)
		{
			//Éî¶È¾àÀë¹é»¯µ½Ö¸¶¨Öµ
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
		cout << "Î´Ö¸¶¨ÌØÕ÷·Ö±æÂÊ£¡" << endl;
		return;
	}
	vector<int> pDensity;
	for (int i = 0;i < this->Nv;++i)
	{
		for (int j = 0; j < this->Nh; ++j)
		{
			//±éÀúradius·¶Î§ÄÚµÄËùÓĞÁÚÓò
			int numNeigh = 0; //ÁÚÓò¸öÊı
			int numHasPoint = 0; //ÁÚÓòÖĞµÄµã¸öÊı
			for (int m = i - radius; m < i + radius; ++m)
			{				
				for (int n = j - radius; n < j + radius; ++n)
				{
					//´¹Ö±·½ÏòÓĞÉÏÏÂ½ç,Ë®Æ½·½Ïò±ß½çÑ­»·
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