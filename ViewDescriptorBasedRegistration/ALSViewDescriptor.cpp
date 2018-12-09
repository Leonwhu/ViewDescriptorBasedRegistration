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
		vd.generateDescriptor();
		ALSDescriptors->push_back(vd);
		if (i%1000==0)
		{
			cout << i << endl;
		}
	}
}

void ALSViewDescriptor::outputDictionaryBinary(const char* p_path)
{
	/*std::ofstream bout(p_path, std::ios::binary | std::ios::out);
	bout.write((char*)(this->ALSDescriptors->size()), sizeof(int));
	bout.write((char*)&this->Nv, sizeof(int));
	bout.write((char*)&this->Nh, sizeof(int));
	for (int i = 0; i < this->ALSDescriptors->size(); ++i)
	{
		float pt_temp[6];
		pt_temp[0] = cloud_xyz->points[i].x;
		pt_temp[1] = cloud_xyz->points[i].y;
		pt_temp[2] = cloud_xyz->points[i].z;
		pt_temp[3] = cloud_xyz->points[i].normal_x;
		pt_temp[4] = cloud_xyz->points[i].normal_y;
		pt_temp[5] = cloud_xyz->points[i].normal_z;
		ofs.write((char*)&pt_temp, 6 * sizeof(float));
	}
	ofs.close();*/
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