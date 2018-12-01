#pragma once
#include "viewDescriptor.h"
#include "dataIo.h"

class TLSViewDescriptor
{
public:
	void setInputTLS(vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *input_TLS){ cloudTLS = input_TLS; };
	void setSaveFolder(const string &filename){ saveFolderPrefix = filename; };
	void setMinDistance(float minDistance){ minDist = minDistance; };
	void setResolutions(float angularResolutionV, float angularResolutionH){
		angResV = angularResolutionV; angResH = angularResolutionH; Nv = int(180.0 / angResV); Nh = int(360.0 / angResH);
	};
	void getViewDescriptorsByDefault();

	void outputTLSViewDescriptors3DImage();
	void outputTLSViewDescriptors2DImage(); 
	void read2DImagesAsDescriptors(vector<string> &fileNames);

	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *cloudTLS;
	vector<ViewDescriptor> *TLSDescriptors;
private:
	//vector<string> TLSfileNames;
	string saveFolderPrefix;
	float angResV;
	float angResH;
	float minDist;
	int Nv;
	int Nh;
}; 