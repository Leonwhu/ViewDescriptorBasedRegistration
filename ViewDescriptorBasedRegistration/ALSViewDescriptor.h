#pragma  once
#include "viewDescriptor.h"
#include "dataIo.h"

class ALSViewDescriptor
{
public:
	void setInputALS(pcl::PointCloud<pcl::PointXYZ>::Ptr input_ALS){ cloudALS = input_ALS; };
	void setViewPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr view_Points){ viewPoints = view_Points; };
	
	void setSaveFolder(const string &filename){ saveFolderPrefix = filename; };
	void setHeightScannerCenter(float height){ heightScannerCenter = height; };
	void setMinDistance(float minDistance){ minDist = minDistance; };
	void setMaxDistance(float maxDistance){ maxDist = maxDistance; };
	void setResolutions(float angularResolutionV, float angularResolutionH){
		angResV = angularResolutionV; angResH = angularResolutionH; Nv = int(180.0 / angResV); Nh = int(360.0 / angResH);
	};
	void setParamsByParalist(Paralist &paralist)
	{
		setSaveFolder(paralist.saveALSFolderPre);
		setHeightScannerCenter(paralist.heightScanner);
		setMinDistance(paralist.minDist);
		setMaxDistance(paralist.maxDist);
		setResolutions(paralist.resolutionSkyDivision, paralist.resolutionSkyDivision);
	};

	void getViewDescriptorsByDefault();
	void getViewDescriptorsByKDTree();

	void outputALSViewDescriptors3DImage();
	void outputALSViewDescriptors2DImage();
	void outputDictionaryBinary(const char* p_path);

	void read2DImagesAsDescriptors(vector<string> &fileNames);
	void read3DImagesAsDescriptors(vector<string> &fileNames);
	void readDictionaryBinary(const char* p_path);

	void transfer3DImagesTo2DImagesAsDescriptors(vector<string> &fileNames);

	vector<ViewDescriptor> *ALSDescriptors;

private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr viewPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALS;
	string saveFolderPrefix;
	float heightScannerCenter;
	float angResV;
	float angResH;
	float minDist;
	float maxDist;
	int Nv;
	int Nh;
};