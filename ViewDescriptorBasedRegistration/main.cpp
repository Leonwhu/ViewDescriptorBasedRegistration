#include "utility.h"
#include "dataIo.h"
#include "viewGeneration.h"
#include "viewDescriptor.h"
#include "ALSViewDescriptor.h"
#include "TLSViewDescriptor.h"
#include "similarityEstimation.h"

using namespace std;
using namespace cv;
using namespace utility;



void main()
{
	//timing
	DWORD t1, t2;
	/*-------1. ���ݶ�ȡ------*/
	//�����������
	DataIo io;
	io.readParalist("paraList.txt");

	/*Bounds boundsALS;
	CenterPoint centerALS;
	int ptNumALS;
	string pathALS;
	cout << "Please input the ALS data:" << endl;
	cin >> pathALS;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALS(new pcl::PointCloud<pcl::PointXYZ>);
	io.readLasData(pathALS, cloudALS, boundsALS, centerALS, ptNumALS);*/

	////�������վ����
	//string pathTLSFolder;
	//cout << "Please input the TLS data folder:" << endl;
	//cin >> pathTLSFolder;
	//vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *vec_cloudTLS = new vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
	//vector<string> fileNames;
	//string extension = ".las";
	//io.readFileNamesInFolder(pathTLSFolder, extension, fileNames);
	/*for (int i = 0; i < fileNames.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr curTLS(new pcl::PointCloud<pcl::PointXYZ>);
		io.readLasFile(fileNames[i], curTLS);
		vec_cloudTLS->push_back(curTLS);
	}*/
	
	/*-------2. ���������ʵ�����/��ȡ------*/
	//������->������ɵ���->���
	//������-�ӻ��ص����г�ȡ���ɴʵ���ӽǵ�	
	/*pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSViews(new pcl::PointCloud<pcl::PointXYZ>);
	ViewGeneration vg;
	vg.getViewsFromALS_centroid(cloudALS, cloudALSViews, io.paralist.gridsizeALS);
	io.OutputLas("viewpoints.las", cloudALSViews, centerALS, boundsALS);*/

	//��������ӽ�������
	//ALSViewDescriptor avd;
	//avd.setInputALS(cloudALS);
	//avd.setViewPoints(cloudALSViews);
	//avd.setHeightScannerCenter(1.5);
	//avd.setMinDistance(io.paralist.minDist);
	//avd.setResolutions(io.paralist.resolutionSkyDivision,io.paralist.resolutionSkyDivision);
	//avd.setSaveFolder("C:\\data\\ALS-TLS\\ALSDescriptors");

	//avd.getViewDescriptorsByDefault();
	////avd.outputALSViewDescriptors3DImage();
	//avd.outputALSViewDescriptors2DImage();

	//�����ӽǵ�
	Bounds boundsALSViews;
	CenterPoint centerALSViews;
	int ptNumALSViews;
	string pathALSViews = io.paralist.pathALSViews; 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSViews(new pcl::PointCloud<pcl::PointXYZ>);
	io.readLasFile(pathALSViews, cloudALSViews);/**/

	//����2D�ʵ�
	/*ALSViewDescriptor avd;
	avd.ALSDescriptors = new vector<ViewDescriptor>();
	string pathALSDescriptors = io.paralist.pathALS2D; 
	vector<string> als2DImages;
	string alsExtension = ".png";
	io.readFileNamesInFolder(pathALSDescriptors, alsExtension, als2DImages);
	avd.read2DImagesAsDescriptors(als2DImages);*/

	//����3D�ʵ䲢תΪ2D--����bug
	ALSViewDescriptor avd;
	avd.ALSDescriptors = new vector<ViewDescriptor>();
	string pathALSDescriptors = io.paralist.pathALS2D;
	vector<string> als3DImages;
	string alsExtension = ".txt";
	io.readFileNamesInFolder(pathALSDescriptors, alsExtension, als3DImages);
	avd.transfer3DImagesTo2DImagesAsDescriptors(als3DImages);
	imwrite("test-transfer.png", avd.ALSDescriptors->at(0).depthimage);

	/*-------3. ����վ�ʵ�����/��ȡ------*/
	//��վ����->���ɵ���->���
	/*TLSViewDescriptor tvd;
	tvd.setInputTLS(vec_cloudTLS);
	tvd.setMinDistance(io.paralist.minDist);
	tvd.setResolutions(io.paralist.resolutionSkyDivision, io.paralist.resolutionSkyDivision);
	tvd.setSaveFolder("C:\\data\\ALS-TLS\\TLSDescriptors");

	tvd.getViewDescriptorsByDefault();
	tvd.outputTLSViewDescriptors3DImage();
	tvd.outputTLSViewDescriptors2DImage();*/

	//�����ƥ������
	TLSViewDescriptor tvd;
	tvd.TLSDescriptors = new vector<ViewDescriptor>();
	string pathTLSDescriptors = io.paralist.pathTLS2D; 
	vector<string> tls2DImages;
	string tlsExtension = ".png";
	io.readFileNamesInFolder(pathTLSDescriptors, tlsExtension, tls2DImages);
	tvd.read2DImagesAsDescriptors(tls2DImages);

	/*-------4. ����վ����ƥ��------*/
	//����ƥ��->���ɸѡ
	//��ɨ���ǲ���
	//��ÿһ����վ���Ӵʵ��н�������ƥ��
	vector<vector<SimilarityResult>> *similarity = new vector<vector<SimilarityResult>>();
	SimilarityEstimation se;
	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	Mat imageTLS; 
	//	vector<SimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		SimilarityResult tempSimilarity;
	//		tempSimilarity.alsIndex = j;
	//		Mat imageALS; 
	//		se.phaseCorrelateOpenCV(imageTLS, imageALS, tempSimilarity.phase_shift, tempSimilarity.response);
	//		curTLSSimilarity.push_back(tempSimilarity);		
	//	}
	//	sort(curTLSSimilarity.begin(),curTLSSimilarity.end(), cmpSimilarityResult);	
	//	//���ǰ50
	//	string saveName;
	//	saveName = "SimilarityResult" + to_string(i) +".txt";
	//	ofstream ofs(saveName.c_str());
	//	//ofs << "*******************TLS index��" << i << "*************************" << endl;
	//	for (int k = 0; k < 50; ++k)
	//	{			
	//		ofs << cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].alsIndex << " " << curTLSSimilarity[k].response << " "
	//		    << curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}
	//

	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	Mat imageTLS; 
	//	se.PreProcessByOpenCV(tvd.TLSDescriptors->at(i).depthimage,imageTLS)	;
	//	imwrite("CutMidFilt"+to_string(i)+".png",imageTLS);
	//	vector<SimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		SimilarityResult tempSimilarity;
	//		tempSimilarity.alsIndex = j;
	//		Mat imageALS; 
	//		se.PreProcessByOpenCV(avd.ALSDescriptors->at(j).depthimage, imageALS);
	//		se.phaseCorrelateOpenCV(imageTLS, imageALS, tempSimilarity.phase_shift, tempSimilarity.response);
	//		curTLSSimilarity.push_back(tempSimilarity);		
	//	}
	//	sort(curTLSSimilarity.begin(), curTLSSimilarity.end(), SimilarityEstimation::cmpSimilarityResult);
	//	//���ǰ50
	//	string saveName;
	//	saveName = "SimilarityResult_CutMidFilt" + to_string(i) +".txt";
	//	ofstream ofs(saveName.c_str());
	//	//ofs << "*******************TLS index��" << i << "*************************" << endl;
	//	for (int k = 0; k < 50; ++k)
	//	{			
	//		ofs << cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].alsIndex << " " << curTLSSimilarity[k].response << " "
	//		    << curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}

	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	Mat imageTLS;
	//	se.PreProcessByOpenCV(tvd.TLSDescriptors->at(i).depthimage, imageTLS);
	//	vector<SimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		SimilarityResult tempSimilarity;
	//		Mat imageALS;
	//		se.PreProcessByOpenCV(avd.ALSDescriptors->at(j).depthimage, imageALS);
	//		imwrite("CutMidFilt_ALS_" + to_string(j) + ".png", imageALS);
	//		se.similarityByOccupation(imageTLS, imageALS, tempSimilarity);
	//		tempSimilarity.alsIndex = j;
	//		curTLSSimilarity.push_back(tempSimilarity);
	//	}
	//	std::sort(curTLSSimilarity.begin(), curTLSSimilarity.end()/*, se.cmpSimilarityResult*/);
	//	//���ǰ50
	//	string saveName;
	//	saveName = "SimilarityResult_byOccupation" + to_string(i) + ".txt";
	//	ofstream ofs(saveName.c_str());
	//	//ofs << "*******************TLS index��" << i << "*************************" << endl;
	//	for (int k = 0; k < 50; ++k)
	//	{
	//		ofs << cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].alsIndex << " " << curTLSSimilarity[k].response << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}

	for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	{
		/*Mat imageTLS;
		se.PreProcessByOpenCV(tvd.TLSDescriptors->at(i).depthimage, imageTLS);*/
		vector<SimilarityResult> curTLSSimilarity;
		for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
		{
			SimilarityResult tempSimilarity;
			/*Mat imageALS;
			se.PreProcessByOpenCV(avd.ALSDescriptors->at(j).depthimage, imageALS);
			imwrite("CutMidFilt_ALS_" + to_string(j) + ".png", imageALS);*/
			se.similarityBySkyLine(tvd.TLSDescriptors->at(i).depthimage, avd.ALSDescriptors->at(j).depthimage, tempSimilarity);
			tempSimilarity.alsIndex = j;
			curTLSSimilarity.push_back(tempSimilarity);
		}
		std::sort(curTLSSimilarity.begin(), curTLSSimilarity.end()/*, se.cmpSimilarityResult*/);
		//���ǰ50
		string saveName;
		saveName = "SimilarityResult_bySkyline" + to_string(i) + ".txt";
		ofstream ofs(saveName.c_str());
		//ofs << "*******************TLS index��" << i << "*************************" << endl;
		for (int k = 0; k < 50; ++k)
		{
			ofs << cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
			ofs << curTLSSimilarity[k].alsIndex << " " << curTLSSimilarity[k].response << " "
				<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
		}
		similarity->push_back(curTLSSimilarity);
		ofs.close();
	}
	

	//��ǰN���ƾ��࣬ȥ����Ⱥ�㣬ȷ��ͬ������
	//��������
	//pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);
	//pcl::PointCloud <pcl::Normal>::Ptr normals(new pcl::PointCloud <pcl::Normal>);
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	//normal_estimator.setSearchMethod(tree);
	//normal_estimator.setInputCloud(cloud);
	//normal_estimator.setKSearch(20);
	//normal_estimator.compute(*normals);
	//
	//pcl::IndicesPtr indices(new std::vector <int>);
	//pcl::PassThrough<pcl::PointXYZ> pass;
	//pass.setInputCloud(cloud);
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.0, 1.0);
	//pass.filter(*indices);
	//
	//pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	//reg.setMinClusterSize(50);
	//reg.setMaxClusterSize(1000000);
	//reg.setSearchMethod(tree);
	//reg.setNumberOfNeighbours(30);
	//reg.setInputCloud(cloud);
	////reg.setIndices (indices);
	//reg.setInputNormals(normals);
	//reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
	//reg.setCurvatureThreshold(1.0);
	//
	//std::vector <pcl::PointIndices> clusters;
	//reg.extract(clusters);
	//
	//std::cout << "Number of clusters is equal to " << clusters.size() << std::endl;
	//std::cout << "First cluster has " << clusters[0].indices.size() << " points." << endl;
	//std::cout << "These are the indices of the points of the initial" <<
	//	std::endl << "cloud that belong to the first cluster:" << std::endl;
	//int counter = 0;
	//while (counter < clusters[0].indices.size())
	//{
	//	std::cout << clusters[0].indices[counter] << ", ";
	//	counter++;
	//	if (counter % 10 == 0)
	//		std::cout << std::endl;
	//}
	//std::cout << std::endl;

	//������תƽ�Ʋ���R T
}
