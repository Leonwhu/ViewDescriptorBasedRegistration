#include "dataIo.h"
#include "utility.h"

#include <string>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>



using namespace  std;
using namespace  boost::filesystem;
using namespace  utility;

bool DataIo::readFileNamesInFolder(const string &folderName, const string & extension, vector<string> &fileNames)
{
	//利用boost遍历文件夹内的文件;
	if (!exists(folderName))
	{
		return 0;
	}
	else
	{
		//遍历该文件夹下的所有ply文件,并保存到fileNames中;
		directory_iterator end_iter;
		for (directory_iterator iter(folderName); iter != end_iter; ++iter)
		{
			if (is_regular_file(iter->status()))
			{
				string fileName;
				fileName = iter->path().string();

				path dir(fileName);

				if (!dir.extension().string().empty())
				{
					if (!fileName.substr(fileName.rfind('.')).compare(extension))
					{
						fileNames.push_back(fileName);
					}
				}
			}
		}
	}
	return 1;
}

bool DataIo::writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
{
	pointCloudBound bound;
	getCloudBound(pointCloud, bound);

	ofstream ofs;
	ofs.open(fileName, std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.minx, bound.miny, bound.minz);
		header.SetMax(bound.maxx, bound.maxy, bound.maxz);
		header.SetOffset((bound.minx + bound.maxx) / 2.0, (bound.miny + bound.maxy) / 2.0, (bound.minz + bound.maxz) / 2.0);

		header.SetScale(0.0000001, 0.0000001, 0.0000001);
		header.SetPointRecordsCount(pointCloud.points.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (int i = 0; i < pointCloud.points.size(); i++)
		{
			pt.SetCoordinates(pointCloud.points[i].x, pointCloud.points[i].y, pointCloud.points[i].z);
			pt.SetIntensity(pointCloud.points[i].intensity);
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}

	return 1;
}

bool DataIo::writeLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ> &pointCloud)
{
	pointCloudBound bound;
	getCloudBound(pointCloud, bound);

	ofstream ofs;
	ofs.open(fileName, std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.minx, bound.miny, bound.minz);
		header.SetMax(bound.maxx, bound.maxy, bound.maxz);
		header.SetOffset((bound.minx + bound.maxx) / 2.0, (bound.miny + bound.maxy) / 2.0, (bound.minz + bound.maxz) / 2.0);

		header.SetScale(0.0000001, 0.0000001, 0.0000001);
		header.SetPointRecordsCount(pointCloud.points.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (int i = 0; i < pointCloud.points.size(); i++)
		{
			pt.SetCoordinates(pointCloud.points[i].x, pointCloud.points[i].y, pointCloud.points[i].z);
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}

	return 1;
}

bool DataIo::mergeLasFileHeaders(const std::vector<liblas::Header>& headers, liblas::Header& mergeFileHeader)
{
	if (headers.empty())
	{
		return 0;
	}

	unsigned int ptNum = 0;

	pointCloudBound bounds;
	bounds.minx = headers[0].GetMinX(); bounds.maxx = headers[0].GetMaxX();
	bounds.miny = headers[0].GetMinY(); bounds.maxy = headers[0].GetMaxY();
	bounds.minz = headers[0].GetMinZ(); bounds.maxz = headers[0].GetMaxZ();

	for (size_t i = 0; i < headers.size(); ++i)
	{
		ptNum += headers[i].GetPointRecordsCount();
		if (headers[i].GetMinX() < bounds.minx)
		{
			bounds.minx = headers[i].GetMinX();
		}
		if (headers[i].GetMinY() < bounds.miny)
		{
			bounds.miny = headers[i].GetMinY();
		}
		if (headers[i].GetMinZ() < bounds.minz)
		{
			bounds.minz = headers[i].GetMinZ();
		}

		if (headers[i].GetMaxX() > bounds.maxx)
		{
			bounds.maxx = headers[i].GetMaxX();
		}
		if (headers[i].GetMaxY() > bounds.maxy)
		{
			bounds.maxy = headers[i].GetMaxY();
		}
		if (headers[i].GetMaxZ() > bounds.maxz)
		{
			bounds.maxz = headers[i].GetMaxZ();
		}
	}
	mergeFileHeader.SetDataFormatId(liblas::ePointFormat2);
	mergeFileHeader.SetVersionMajor(1);
	mergeFileHeader.SetVersionMinor(2);
	mergeFileHeader.SetMin(bounds.minx, bounds.miny, bounds.minz);
	mergeFileHeader.SetMax(bounds.maxx, bounds.maxy, bounds.maxz);
	mergeFileHeader.SetOffset((bounds.minx + bounds.maxx) / 2.0, (bounds.miny + bounds.maxy) / 2.0, (bounds.minz + bounds.maxz) / 2.0);
	mergeFileHeader.SetScale(0.001, 0.001, 0.0001);
	mergeFileHeader.SetPointRecordsCount(ptNum);
	return 1;
}

bool DataIo::readLasFileHeader(const std::string &fileName, liblas::Header& header)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}
	else
	{
		std::ifstream ifs;
		ifs.open(fileName, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			return 0;
		}

		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);

		header = reader.GetHeader();
	}

	return 1;
}

/*读取las文件，不重心化坐标*/
bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		pcl::PointXYZ  pt;

		pt.x = p.GetX();
		pt.y = p.GetY();
		pt.z = p.GetZ();
		//pt.intensity = p.GetIntensity();
		pointCloud->points.push_back(pt);
	}
	
	return 1;
}

bool DataIo::readTLSNonground(const std::string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		if (p.GetColor().GetGreen() != 255)
		{
			continue;
		}
		pcl::PointXYZ  pt;

		pt.x = p.GetX();
		pt.y = p.GetY();
		pt.z = p.GetZ();
		//pt.intensity = p.GetIntensity();
		pointCloud->points.push_back(pt);
	}

	return 1;
}

bool DataIo::readLasFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		pcl::PointXYZI  pt;

		pt.x = p.GetX();
		pt.y = p.GetY();
		pt.z = p.GetZ();
		pt.intensity = p.GetIntensity();
		pointCloud.points.push_back(pt);
	}

	return 1;
}

/*读取las文件并重心化坐标*/
void  DataIo::readLasData(const string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Bounds& bounds, CenterPoint& center_point, int &pt_num)
{
	if (!filename.substr(filename.rfind('.')).compare(".las"))
	{
		std::ifstream ifs;
		ifs.open(filename, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			cout << "未发现匹配项" << endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);

		const liblas::Header& header = reader.GetHeader();
		pt_num = header.GetPointRecordsCount();//给点个数赋值;
		/*给最大，最小坐标值赋值;*/
		bounds.min_x = header.GetMinX();  bounds.max_x = header.GetMaxX();
		bounds.min_y = header.GetMinY();  bounds.max_y = header.GetMaxY();
		bounds.min_z = header.GetMinZ();  bounds.max_z = header.GetMaxZ();
		/*给重心坐标赋值;*/
		center_point.x = (header.GetMinX() + header.GetMaxX()) / 2.0;
		center_point.y = (header.GetMinY() + header.GetMaxY()) / 2.0;
		center_point.z = (header.GetMinZ() + header.GetMaxZ()) / 2.0;

		liblas::PointFormatName format = header.GetDataFormatId();//获得点的格式类型,数据类型0和1没有颜色值，其他数据类型有颜色值;
		bool has_color = false;
		if (liblas::ePointFormat0 == format || liblas::ePointFormat1 == format)
			has_color = false;
		else
			has_color = true;
		int i = 0;
		/*while循环中遍历所有的点;*/
		int num = 0;
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			pcl::PointXYZ  las_point;

			/*将重心化后的坐标和强度值赋值给PCL中的点;*/
			las_point.x = p.GetX() - center_point.x;
			las_point.y = p.GetY() - center_point.y;
			las_point.z = p.GetZ() - center_point.z;
			cloud_xyz->push_back(las_point);//将点压入pointcloud中;
			num++;
		}
	}
	/*cout << "    点个数：" << cloud_xyz->size() << endl;
	cout << "----Finish reading! " << endl;*/
}

void  DataIo::readLasData(const string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz, Bounds& bounds, CenterPoint& center_point, int &pt_num, vector<short> *propsClassification)
{
	if (!filename.substr(filename.rfind('.')).compare(".las"))
	{
		std::ifstream ifs;
		ifs.open(filename, std::ios::in | std::ios::binary);
		if (ifs.bad())
		{
			cout << "未发现匹配项" << endl;
		}
		liblas::ReaderFactory f;
		liblas::Reader reader = f.CreateWithStream(ifs);

		const liblas::Header& header = reader.GetHeader();
		pt_num = header.GetPointRecordsCount();//给点个数赋值;
		/*给最大，最小坐标值赋值;*/
		bounds.min_x = header.GetMinX();  bounds.max_x = header.GetMaxX();
		bounds.min_y = header.GetMinY();  bounds.max_y = header.GetMaxY();
		bounds.min_z = header.GetMinZ();  bounds.max_z = header.GetMaxZ();
		/*给重心坐标赋值;*/
		center_point.x = (header.GetMinX() + header.GetMaxX()) / 2.0;
		center_point.y = (header.GetMinY() + header.GetMaxY()) / 2.0;
		center_point.z = (header.GetMinZ() + header.GetMaxZ()) / 2.0;

		liblas::PointFormatName format = header.GetDataFormatId();//获得点的格式类型,数据类型0和1没有颜色值，其他数据类型有颜色值;
		bool has_color = false;
		if (liblas::ePointFormat0 == format || liblas::ePointFormat1 == format)
			has_color = false;
		else
			has_color = true;
		int i = 0;
		/*while循环中遍历所有的点;*/
		int num = 0;
		while (reader.ReadNextPoint())
		{
			const liblas::Point& p = reader.GetPoint();
			pcl::PointXYZ  las_point;

			/*将重心化后的坐标和强度值赋值给PCL中的点;*/
			las_point.x = p.GetX() - center_point.x;
			las_point.y = p.GetY() - center_point.y;
			las_point.z = p.GetZ() - center_point.z;
			
			cloud_xyz->push_back(las_point);//将点压入pointcloud中;

			short t = p.GetClassification().GetClass();
			propsClassification->push_back(t);
			num++;
		}
	}
	/*cout << "    点个数：" << cloud_xyz->size() << endl;
	cout << "----Finish reading! " << endl;*/
}

bool DataIo::readPcdFile(const std::string &fileName, pcl::PointCloud<pcl::PointXYZI> &pointCloud)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZI>(fileName, pointCloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test_pcd.pcd \n");
		return false;
	}

	return true;
}

bool DataIo::mergeLasFilesIntensity(const std::string &folderName)
{
	//获取文件夹下的las文件名;
	vector<string> fileNames;
	string extension;
	extension = ".las";
	readFileNamesInFolder(folderName, extension, fileNames);

	//读取所有las文件的文件头;
	vector<liblas::Header> headers;
	for (size_t i = 0; i < fileNames.size(); ++i)
	{
		liblas::Header header;
		readLasFileHeader(fileNames[i], header);
		headers.push_back(header);
	}

	//确定合并后的las文件的文件头;
	liblas::Header mergeFileHeader;
	mergeLasFileHeaders(headers, mergeFileHeader);

	string mergeFileName;
	mergeFileName = folderName + "//" + "mergedFileIntensity.las";

	std::ofstream ofs;
	ofs.open(mergeFileName, std::ios::out | std::ios::binary);
	if (!ofs.is_open())
	{
		return 0;
	}
	else
	{
		liblas::Writer writer(ofs, mergeFileHeader);
		liblas::Point pt(&mergeFileHeader);
		//依次读取文件夹中的las文件,并随机赋色;
		srand((unsigned)time(NULL));
		for (size_t i = 0; i < fileNames.size(); ++i)
		{
			pcl::PointCloud<pcl::PointXYZI> pointCloud;
			readLasFile(fileNames[i], pointCloud);
			for (int i = 0; i < pointCloud.points.size(); ++i)
			{
				pt.SetCoordinates((double)pointCloud.points[i].x, (double)pointCloud.points[i].y, (double)pointCloud.points[i].z);
				pt.SetIntensity(pointCloud.points[i].intensity);
				writer.WritePoint(pt);
			}
		}
		ofs.flush();
		ofs.close();
	}

	return 1;
}

bool DataIo::mergeLasFilesColoredByFile(const std::string &folderName)
{
	//获取文件夹下的las文件名;
	vector<string> fileNames;
	string extension;
	extension = ".las";
	readFileNamesInFolder(folderName, extension, fileNames);

	//读取所有las文件的文件头;
	vector<liblas::Header> headers;
	for (size_t i = 0; i < fileNames.size(); ++i)
	{
		liblas::Header header;
		readLasFileHeader(fileNames[i], header);
		headers.push_back(header);
	}

	//确定合并后的las文件的文件头;
	liblas::Header mergeFileHeader;
	mergeLasFileHeaders(headers, mergeFileHeader);

	string mergeFileName;
	mergeFileName = folderName + "//" + "mergedFileColor.las";

	std::ofstream ofs;
	ofs.open(mergeFileName, std::ios::out | std::ios::binary);
	if (!ofs.is_open())
	{
		return 0;
	}
	else
	{
		liblas::Writer writer(ofs, mergeFileHeader);
		liblas::Point pt(&mergeFileHeader);
		//依次读取文件夹中的las文件,并随机赋色;
		srand((unsigned)time(NULL));
		unsigned int R, G, B;
		for (size_t i = 0; i < fileNames.size(); ++i)
		{
			R = rand() % 255;
			G = rand() % 255;
			B = rand() % 255;
			pcl::PointCloud<pcl::PointXYZI> pointCloud;
			readLasFile(fileNames[i], pointCloud);
			for (int i = 0; i < pointCloud.points.size(); ++i)
			{
				pt.SetCoordinates((double)pointCloud.points[i].x, (double)pointCloud.points[i].y, (double)pointCloud.points[i].z);
				pt.SetIntensity(pointCloud.points[i].intensity);
				pt.SetColor(liblas::Color(R, G, B));
				writer.WritePoint(pt);
			}
		}
		ofs.flush();
		ofs.close();
	}

	return 1;
}

void DataIo::readParalist(string paralistfile)
{
	ifstream infile;   //输入流
	infile.open(paralistfile, ios::in);
	if (!infile.is_open()) cout << "Open file failure" << endl;
	infile >> paralist.gridsizeALS;
	infile >> paralist.resolutionSkyDivision;
	infile >> paralist.minDist;
	infile >> paralist.heightScanner;
	infile >> paralist.pathALSViews;
	infile >> paralist.pathALS2D;
	infile >> paralist.pathALS3D;
	infile >> paralist.pathTLS2D;
	infile >> paralist.pathTLS3D;
	/*infile >> paralist.pview.x;
	infile >> paralist.pview.y;
	infile >> paralist.pview.z;*/
	/*infile >> paralist.num_point_bb;
	infile >> paralist.feature_r;
	infile >> paralist.keypoint_max_ratio;
	infile >> paralist.keypoint_min_num;
	infile >> paralist.scale;
	infile >> paralist.p_pre;
	infile >> paralist.p_ED;
	infile >> paralist.p_FD;
	infile >> paralist.m;
	infile >> paralist.converge_t;
	infile >> paralist.converge_r;
	infile >> paralist.output;*/
	infile.close();
}

void DataIo::OutputLas(string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, CenterPoint center_pt, Bounds bound)
{
	std::ofstream ofs;
	ofs.open(file_name, std::ios::out | std::ios::binary);
	if (ofs.is_open())
	{
		liblas::Header header;
		header.SetDataFormatId(liblas::ePointFormat2);
		header.SetVersionMajor(1);
		header.SetVersionMinor(2);
		header.SetMin(bound.min_x, bound.min_y, bound.min_z);
		header.SetMax(bound.max_x, bound.max_y, bound.max_z);
		header.SetOffset(center_pt.x, center_pt.y, center_pt.z);
		header.SetScale(0.000001, 0.000001, 0.0001);
		header.SetPointRecordsCount(cloud->points.size());

		liblas::Writer writer(ofs, header);
		liblas::Point pt(&header);

		for (int i = 0; i < cloud->points.size(); i++)
		{
			pt.SetCoordinates(cloud->points[i].x + center_pt.x, cloud->points[i].y + center_pt.y, cloud->points[i].z + center_pt.z);
			writer.WritePoint(pt);
		}
		ofs.flush();
		ofs.close();
	}
}

bool DataIo::readLasWithClassification(const string &fileName, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, vector<short> *propsClassification)
{
	if (fileName.substr(fileName.rfind('.')).compare(".las"))
	{
		return 0;
	}

	std::ifstream ifs;
	ifs.open(fileName, std::ios::in | std::ios::binary);
	if (ifs.bad())
	{
		cout << "未发现匹配项" << endl;
	}
	liblas::ReaderFactory f;
	liblas::Reader reader = f.CreateWithStream(ifs);

	/*while循环中遍历所有的点;*/
	while (reader.ReadNextPoint())
	{
		const liblas::Point& p = reader.GetPoint();
		pcl::PointXYZ  pt;

		pt.x = p.GetX();
		pt.y = p.GetY();
		pt.z = p.GetZ();
		//pt.intensity = p.GetIntensity();		
		pointCloud->points.push_back(pt);

		short t = p.GetClassification().GetClass();
		propsClassification->push_back(t);

	}

	return 1;
}
bool cmp1(pair<int, string>a, pair<int, string>b)
{
	return a.first < b.first;
}
void DataIo::sortFileNames(vector<string> &fileNames)
{
	vector<pair<int, string>> vec;
	for (int i = 0; i < fileNames.size(); ++i)
	{
		string strIndex;
		strIndex = fileNames[i].substr(fileNames[i].find_last_of("\\")+1);
		strIndex = strIndex.substr(0,strIndex.find("_"));
		vec.push_back({stoi(strIndex),fileNames[i]});
	}
	sort(vec.begin(), vec.end(), cmp1);
	for (int i = 0; i < fileNames.size(); ++i)
	{
		fileNames[i] = vec[i].second;
	}
}