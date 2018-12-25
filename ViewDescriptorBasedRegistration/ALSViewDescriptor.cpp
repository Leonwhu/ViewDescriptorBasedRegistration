#include "ALSViewDescriptor.h"
#include <sstream>

using namespace cv;


void ALSViewDescriptor::outputALSViewDescriptors3DImage()
{
	string pathSave;
	pathSave = saveFolderPrefix + "_ALS_3DImages";
	string cmd_creatFile = "mkdir " + pathSave;
	system(cmd_creatFile.c_str());
	for (int i = 0; i < ALSDescriptors->size(); ++i)
	{
		string imageName;
		imageName = pathSave + "\\" + to_string(i) + "_" + to_string(ALSDescriptors->at(i).getSVFValue()) + ".txt";
		ALSDescriptors->at(i).outputViewDescriptor3DImage(imageName);
	}
}


void ALSViewDescriptor::outputALSViewDescriptors2DImage()
{
	string pathSave;
	pathSave = saveFolderPrefix + "_ALS_2DImages";
	string cmd_creatFile = "mkdir " + pathSave;
	system(cmd_creatFile.c_str());
	for (int i = 0; i < ALSDescriptors->size(); ++i)
	{
		string imageName;
		imageName = pathSave + "\\" + to_string(i) + "_" + to_string(ALSDescriptors->at(i).getSVFValue()) + ".png";
		ALSDescriptors->at(i).outputViewDescriptor2DImage(imageName);
	}
}



void ALSViewDescriptor::getViewDescriptorsByDefault()
{
	ALSDescriptors = new vector<ViewDescriptor>();
	for (int i = 0; i < viewPoints->size()/*10*/; ++i)
	{
		ViewDescriptor vd;
		pcl::PointXYZ pt;
		pt.x = viewPoints->points[i].x;
		pt.y = viewPoints->points[i].y;
		pt.z = viewPoints->points[i].z + heightScannerCenter;
		vd.setViewPoint(pt);
		vd.setInputCloud(this->cloudALS);
		vd.setResolutions(this->angResV, this->angResH);
		vd.setMinDist(this->minDist);
		vd.setMaxDist(this->maxDist);
		vd.generateDescriptor();
		ALSDescriptors->push_back(vd);
		if (i%10000==0)
		{
			cout << i << endl;
		}
	}
}

void ALSViewDescriptor::getViewDescriptorsByKDTree()
{
	ALSDescriptors = new vector<ViewDescriptor>();
	pcl::KdTreeFLANN<pcl::PointXY> kdtree;

	// 将非地面点的xy作为 KdTree 输入
	pcl::PointCloud<pcl::PointXY>::Ptr cloudXY(new pcl::PointCloud<pcl::PointXY>());
	pcl::PointXY pt;
	for (int i = 0; i < this->cloudALS->points.size(); ++i)
	{
		pt.x = this->cloudALS->points[i].x;
		pt.y = this->cloudALS->points[i].y;
		cloudXY->push_back(pt);
	}
	kdtree.setInputCloud(cloudXY);

	// 创建一个点作为查找中心
	pcl::PointXY searchPoint;
	for (int i = 0; i < viewPoints->size()/*10*/; ++i)
	{
		//建立观测点
		pcl::PointXYZ pt;
		pt.x = viewPoints->points[i].x;
		pt.y = viewPoints->points[i].y;
		pt.z = viewPoints->points[i].z + heightScannerCenter;

		//查找指定领域范围内的点
		searchPoint.x = pt.x;
		searchPoint.y = pt.y;
		std::vector<int> pointIdxRadiusSearch;
		std::vector<float> pointRadiusSquaredDistance;
		kdtree.radiusSearch(searchPoint, maxDist, pointIdxRadiusSearch, pointRadiusSquaredDistance);
		
		//生成视角描述子
		ViewDescriptor vd;	
		vd.setViewPoint(pt);
		vd.setInputCloud(this->cloudALS);
		vd.setResolutions(this->angResV, this->angResH);
		vd.setMinDist(this->minDist);
		vd.setMaxDist(this->maxDist);
		vd.generateDescriptorByKDTree(pointIdxRadiusSearch);
		ALSDescriptors->push_back(vd);
		if (i % 10000 == 0)
		{
			cout << i << endl;
		}
	}
}

void ALSViewDescriptor::outputDictionaryBinary(const char* p_path)
{
	std::ofstream bout(p_path, std::ios::binary | std::ios::out);
	int dicSize = this->ALSDescriptors->size();
	bout.write((char*)&dicSize, sizeof(int));
	bout.write((char*)&this->Nv, sizeof(int));
	bout.write((char*)&this->Nh, sizeof(int));
	for (int i = 0; i < this->ALSDescriptors->size(); ++i)
	{
		for (int idxNh = 0; idxNh < Nh; ++idxNh)
		{
			bout.write((char*)&ALSDescriptors->at(i).skyline.pContours[idxNh].zenithAngle, sizeof(int));
			bout.write((char*)&ALSDescriptors->at(i).skyline.pContours[idxNh].depth, sizeof(float));
		}
	}
	bout.close();
}

void ALSViewDescriptor::readDictionaryBinary(const char* p_path)
{
	this->ALSDescriptors->swap(vector<ViewDescriptor>());
	std::ifstream bin(p_path, std::ios::binary | std::ios::in);
	int dicSize;
	bin.read((char*)&dicSize, sizeof(int));
	bin.read((char*)&Nv, sizeof(int));
	bin.read((char*)&Nh, sizeof(int));
	for (int i = 0; i < dicSize; ++i)
	{
		ViewDescriptor tempVD;
		for (int idxNh = 0; idxNh < Nh; ++idxNh)
		{
			ContourPoint tempCP;
			bin.read((char*)&tempCP.zenithAngle, sizeof(int));
			bin.read((char*)&tempCP.depth, sizeof(float));
			tempVD.skyline.Nh = Nh;
			tempVD.skyline.pContours.push_back(tempCP);
		}
		ALSDescriptors->push_back(tempVD);
	}
	bin.close();
}

void ALSViewDescriptor::read2DImagesAsDescriptors(vector<string> &fileNames)
{
	this->ALSDescriptors->swap(vector<ViewDescriptor>());
	for (int i = 0; i < fileNames.size()/*1000*/; ++i)
	{
		ViewDescriptor tempvd;
		tempvd.depthimage = imread(fileNames[i], 0);
		this->ALSDescriptors->push_back(tempvd);
	}	
}

void ALSViewDescriptor::read3DImagesAsDescriptors(vector<string> &fileNames)
{
	this->ALSDescriptors->swap(vector<ViewDescriptor>());
	for (int i = 0; i < fileNames.size()/*1000*/; ++i)
	{
		ViewDescriptor tempvd;
		tempvd.Nh = 90;
		tempvd.Nv = 45;
		tempvd.minDist = 3.0;
		tempvd.viewDepth.resize(tempvd.Nh*tempvd.Nv, -1.0);
		ifstream ifs(fileNames[i]);
		string tempLine;
		while (!ifs.eof())
		{
			getline(ifs, tempLine);
			if (tempLine != "")
			{
				int tempv, temph;
				float tempDepth;
				stringstream ss;
				ss << tempLine;
				ss >> temph >> tempv >> tempDepth;
				tempvd.viewDepth[(tempvd.Nv - tempv) * tempvd.Nh + temph] = tempDepth;
			}
		}
		this->ALSDescriptors->push_back(tempvd);
	}
	LOG(INFO) << "读取ALS的3D描述子点个数：" << this->ALSDescriptors->size() << endl;
}

void ALSViewDescriptor::transfer3DImagesTo2DImagesAsDescriptors(vector<string> &fileNames)
{
	this->ALSDescriptors->swap(vector<ViewDescriptor>());
	Nv = 45; Nh = 90; 
	minDist = 3.0;
	float maxDist2Grey = 200, distance = 100.0;
	for (int i = 0; i < fileNames.size()/*1000*/; ++i)
	{
		ViewDescriptor tempvd;
		Mat tempIm(Nv, Nh, CV_8UC1);
		ifstream ifs(fileNames[i]);
		if (ifs.is_open())
		{
			string curLine;
			while (!ifs.eof())
			{
				getline(ifs,curLine);
				if (curLine =="")
					continue;
				int i, j;
				float depth;
				stringstream ss;
				ss << curLine;
				ss >> i >> j >> depth;
				tempIm.at<uchar>(j, i) = min(depth - minDist, distance) * maxDist2Grey / distance;
			}		
		}
		ifs.close();
		tempvd.depthimage = tempIm;
		this->ALSDescriptors->push_back(tempvd);
	}
}