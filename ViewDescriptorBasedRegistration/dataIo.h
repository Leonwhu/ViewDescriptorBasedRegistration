#ifndef DATAIO 
#define DATAIO
#include "utility.h"
#include <string>

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>

#include <vector>
#include <boost/filesystem.hpp>
#include <boost/function.hpp>

#include <liblas/liblas.hpp>
#include <liblas/version.hpp>
#include <liblas/point.hpp>

using namespace std;
using namespace utility;

struct Paralist{

	float gridsizeALS;
	float resolutionSkyDivision;
	float minDist;
	float heightScanner;
	string pathALSViews;
	string pathALS2D;
	string pathALS3D;
	string pathTLS2D;
	string pathTLS3D;
	//pcl::PointXYZ pview;

	/*float num_point_bb;

	float feature_r;
	float keypoint_max_ratio;
	int keypoint_min_num;

	float scale;
	float p_pre;
	float p_ED;
	float p_FD;
	float m;

	float converge_t;
	float converge_r;

	bool output;*/
};


class DataIo
{
public:
	//文件夹遍历;
	bool readFileNamesInFolder(const string &folderName, const string & extension, vector<string> &fileNames);

	// las 文件读写;
	//读取点云并重心化
	void readLasData(const string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Bounds & bounds, CenterPoint& center_point, int &pt_num);
	void readLasData(const string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Bounds & bounds, CenterPoint& center_point, int &pt_num, vector<short> *propsClassification);


	bool readLasFileHeader(const std::string &fileName, liblas::Header& header);
	//读入点云不做重心化处理
	bool readLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
	bool readTLSNonground(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
	bool readLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);

	//读入点云xyz及所属类别
	bool readLasWithClassification(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, vector<short> *propsClassification);

	bool writeLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
	bool writeLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZ> &pointCloud);
	void OutputLas(string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CenterPoint center_pt, Bounds bound);

	bool mergeLasFileHeaders(const std::vector<liblas::Header>& header, liblas::Header& mergeFileHeader);
	bool mergeLasFilesColoredByFile(const string &folderName);
	bool mergeLasFilesIntensity(const string &folderName);

	// pcd 文件读写;
	bool readPcdFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
	void mergePcdFilesColoredByFile(const std::string &folderName);
	void mergePcdFilesIntensity(const std::string &folderName);

	//参数文件读取
	void readParalist(string paralistfile);

	//文件名按照点号排序
	void sortFileNames(vector<string> &fileNames);

	Paralist paralist;
protected:

private:



};

#endif