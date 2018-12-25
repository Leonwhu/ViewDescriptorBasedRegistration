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

	float gridsizeALS;                //resolution of ALS grid
	float resolutionSkyDivision;      //resolution of sky division
	float minDist;                    //min valid distance of TLS
	float maxDist;                    //max valid distance of TLS
	float minAngle;                   //min scanning angle of TLS (zenith)
	float maxAngle;                   //max scanning angle of TLS (zenith)
	float heightScanner;              //height of scanner
	float widthDP;                    //width for DP similarity calculation

	//���ɴʵ�
	string pathALSPointCloud;         //path of ALS point cloud
	string saveALSFolderPre;          //path to save ALS view descriptors

	//����ʵ�
	string pathALSViews;              //path of ALS viewpoints
	string pathALS2D;                 //path of ALS 2D image
	string pathALS3D;                 //path of ALS 3D image
	string pathTLS2D;                 //path of TLS 2D image
	string pathTLS3D;                 //path of TLS 3D image
	string pathALSDic;                //path of binary ALS Dictionary 
	int Nv; 
	int Nh;
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
	//�ļ��б���;
	bool readFileNamesInFolder(const string &folderName, const string & extension, vector<string> &fileNames);

	// las �ļ���д;
	//��ȡ���Ʋ����Ļ�
	void readLasData(const string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Bounds & bounds, CenterPoint& center_point, int &pt_num);
	void readLasData(const string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Bounds & bounds, CenterPoint& center_point, int &pt_num, vector<short> *propsClassification);


	bool readLasFileHeader(const std::string &fileName, liblas::Header& header);
	//������Ʋ������Ļ�����
	bool readLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
	bool readTLSNonground(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud);
	bool readLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);

	//�������xyz���������
	bool readLasWithClassification(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, vector<short> *propsClassification);

	bool writeLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
	bool writeLasFile(const string &fileName, pcl::PointCloud<pcl::PointXYZ> &pointCloud);
	void OutputLas(string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CenterPoint center_pt, Bounds bound);

	bool mergeLasFileHeaders(const std::vector<liblas::Header>& header, liblas::Header& mergeFileHeader);
	bool mergeLasFilesColoredByFile(const string &folderName);
	bool mergeLasFilesIntensity(const string &folderName);

	// pcd �ļ���д;
	bool readPcdFile(const string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud);
	void mergePcdFilesColoredByFile(const std::string &folderName);
	void mergePcdFilesIntensity(const std::string &folderName);

	//�����ļ���ȡ
	void readParalist(string paralistfile);

	//�ļ������յ������
	void sortFileNames(vector<string> &fileNames);

	Paralist paralist;
protected:

private:



};

#endif