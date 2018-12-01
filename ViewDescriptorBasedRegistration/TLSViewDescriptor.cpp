#include "TLSViewDescriptor.h"

using namespace cv;

void TLSViewDescriptor::getViewDescriptorsByDefault()
{
	TLSDescriptors = new vector<ViewDescriptor>();
	for (int i = 0; i < this->cloudTLS->size(); ++i)
	{
		ViewDescriptor vd;
		pcl::PointXYZ pt(0.0,0.0,0.0);
		vd.setViewPoint(pt);
		vd.setInputCloud(this->cloudTLS->at(i));
		vd.setResolutions(this->angResV, this->angResH);
		vd.setMinDist(this->minDist);
		vd.generateDescriptor();
		TLSDescriptors->push_back(vd);
		if (i % 1000 == 0)
		{
			cout << i << endl;
		}
	}
}

void TLSViewDescriptor::outputTLSViewDescriptors3DImage()
{
	string pathSave;
	pathSave = saveFolderPrefix + "_TLS_3DImages";
	string cmd_creatFile = "mkdir " + pathSave;
	system(cmd_creatFile.c_str());
	for (int i = 0; i < TLSDescriptors->size(); ++i)
	{
		string imageName;
		imageName = pathSave + "\\" + to_string(i) + "_" + to_string(TLSDescriptors->at(i).getSVFValue()) + ".txt";
		TLSDescriptors->at(i).outputViewDescriptor3DImage(imageName);
	}
}

void TLSViewDescriptor::outputTLSViewDescriptors2DImage()
{
	string pathSave;
	pathSave = saveFolderPrefix + "_TLS_2DImages";
	string cmd_creatFile = "mkdir " + pathSave;
	system(cmd_creatFile.c_str());
	for (int i = 0; i < TLSDescriptors->size(); ++i)
	{
		string imageName;
		imageName = pathSave + "\\" + to_string(i) + "_" + to_string(TLSDescriptors->at(i).getSVFValue()) + ".png";
		TLSDescriptors->at(i).outputViewDescriptor2DImage(imageName);
	}
}

void TLSViewDescriptor::read2DImagesAsDescriptors(vector<string> &fileNames)
{
	this->TLSDescriptors->swap(vector<ViewDescriptor>());
	for (int i = 0; i < fileNames.size(); ++i)
	{
		ViewDescriptor tempvd;
		tempvd.depthimage = imread(fileNames[i], -1);
		this->TLSDescriptors->push_back(tempvd);
	}
}