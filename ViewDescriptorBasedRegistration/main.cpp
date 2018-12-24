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
	FLAGS_log_dir = "C:\\logs\\";
	FLAGS_logbufsecs = 0;	//实时输出
	google::InitGoogleLogging("ViewRegistration");

	DWORD t1, t2, t3, t4, t5;
	
	DataIo io;
	io.readParalist("paraList.txt");
	/*-------1. 数据读取------*/
	//读入机载数据
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALS(new pcl::PointCloud<pcl::PointXYZ>);
	Bounds boundsALS;
	CenterPoint centerALS;
	int ptNumALS;
	vector<short> *alsClass = new vector<short>();
	string pathALS;
	/*cout << "Please input the ALS data:" << endl;
	cin >> pathALS;	*/
	pathALS = io.paralist.pathALSPointCloud;
	io.readLasData(pathALS, cloudALS, boundsALS, centerALS, ptNumALS, alsClass);
	LOG(INFO) << "Number of points: " << ptNumALS << endl;

	//读入地面站数据
	/*string pathTLSFolder;
	cout << "Please input the TLS data folder:" << endl;
	cin >> pathTLSFolder;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> *vec_cloudTLS = new vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();
	vector<string> fileNames;
	string extension = ".las";
	io.readFileNamesInFolder(pathTLSFolder, extension, fileNames);
	for (int i = 0; i < fileNames.size(); ++i)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr curTLS(new pcl::PointCloud<pcl::PointXYZ>);
		io.readTLSNonground(fileNames[i], curTLS);
		vec_cloudTLS->push_back(curTLS);
	}*/
	
	/*-------2. 机载特征词典生成/读取------*/
	//格网化->逐点生成单词->输出
	//格网化-从机载点云中抽取生成词典的视角点	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSGround(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSNonground(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSViews(new pcl::PointCloud<pcl::PointXYZ>);
	ViewGeneration vg;
	vg.getALSGround(cloudALS, alsClass, cloudALSGround, cloudALSNonground);
	vg.getViewsFromALS_GroundSample(cloudALSGround, cloudALSViews, io.paralist.gridsizeALS);
	//io.OutputLas("viewpoints.las", cloudALSViews, centerALS, boundsALS);

	////逐点生成视角描述子
	t1 = GetTickCount();
	ALSViewDescriptor avd;
	avd.setInputALS(cloudALSNonground);
	avd.setViewPoints(cloudALSViews);
	avd.setHeightScannerCenter(io.paralist.heightScanner);
	avd.setMinDistance(io.paralist.minDist);
	avd.setMaxDistance(io.paralist.maxDist);
	avd.setResolutions(io.paralist.resolutionSkyDivision,io.paralist.resolutionSkyDivision);
	avd.setSaveFolder(io.paralist.saveALSFolderPre);

	//avd.getViewDescriptorsByDefault();
	avd.getViewDescriptorsByKDTree();
	t2 = GetTickCount();
	LOG(INFO) << "Time for generating view dictionary: " << (t2 - t1)*1.0 / 1000 << " s" << endl;
	
	avd.outputALSViewDescriptors3DImage();
	avd.outputALSViewDescriptors2DImage();
	t3 = GetTickCount();
	LOG(INFO) << "Time for saving view dictionary: " << (t3 - t2)*1.0 / 1000 << " s" << endl;

	//读入视角点，选取离格网中心最近的点为视角点
	//Bounds boundsALSViews;
	//CenterPoint centerALSViews;
	//int ptNumALSViews;
	//string pathALSViews = io.paralist.pathALSViews; 
	////pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSViews(new pcl::PointCloud<pcl::PointXYZ>);
	//io.readLasFile(pathALSViews, cloudALSViews);
	//LOG(INFO) << "读取ALS视角点个数：" << cloudALSViews->points.size() << endl;

	//读入2D词典
	
	//avd.ALSDescriptors = new vector<ViewDescriptor>();
	//string pathALSDescriptors = io.paralist.pathALS2D; 
	//vector<string> als2DImages;
	//string alsExtension = ".png";
	//io.readFileNamesInFolder(pathALSDescriptors, alsExtension, als2DImages);
	//io.sortFileNames(als2DImages);
	//avd.read2DImagesAsDescriptors(als2DImages);/**/

	//读入3D词典
<<<<<<< HEAD
	//ALSViewDescriptor avd;
	/*avd.ALSDescriptors = new vector<ViewDescriptor>();
=======
	ALSViewDescriptor avd;
	avd.ALSDescriptors = new vector<ViewDescriptor>();
>>>>>>> parent of 34153d8... DPSkylineMinimal
	string pathALSDescriptors = io.paralist.pathALS3D;
	vector<string> als3DImages;
	string alsExtension = ".txt";
	io.readFileNamesInFolder(pathALSDescriptors, alsExtension, als3DImages);
	io.sortFileNames(als3DImages);
	avd.read3DImagesAsDescriptors(als3DImages);
	for (int i = 0; i < avd.ALSDescriptors->size(); ++i)
	{
		avd.ALSDescriptors->at(i).convert2DImage(200.0);
<<<<<<< HEAD
	}*/
=======
	}
>>>>>>> parent of 34153d8... DPSkylineMinimal


	/*-------3. 地面站词典生成/读取------*/	
	TLSViewDescriptor tvd;
	//逐站读入->生成单词->输出
	/*tvd.setInputTLS(vec_cloudTLS);
	tvd.setMinDistance(io.paralist.minDist);
	tvd.setMaxDistance(io.paralist.maxDist);
	tvd.setResolutions(io.paralist.resolutionSkyDivision, io.paralist.resolutionSkyDivision);
	tvd.setSaveFolder("C:\\data\\ALS-TLS\\TLSDescriptors");*/

	/*tvd.getViewDescriptorsByDefault();
	tvd.outputTLSViewDescriptors3DImage();
	tvd.outputTLSViewDescriptors2DImage();*/

	//读入3D待匹配特征	
	tvd.TLSDescriptors = new vector<ViewDescriptor>();
	string pathTLSDescriptors = io.paralist.pathTLS3D; 
	vector<string> tls3DImages;
	string tlsExtension = ".txt";	
	io.readFileNamesInFolder(pathTLSDescriptors, tlsExtension, tls3DImages);
	io.sortFileNames(tls3DImages);
	tvd.read3DImagesAsDescriptors(tls3DImages);/**/
<<<<<<< HEAD
	for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	{
		tvd.TLSDescriptors->at(i).filterNoiseBy2DDensity(2, 3);
		tvd.TLSDescriptors->at(i).convert2DImage(200.0);
	}	
=======
	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	tvd.TLSDescriptors->at(i).filterNoiseBy2DDensity(2, 3);
	//	//tvd.TLSDescriptors->at(i).convert2DImage(200.0);
	//	//tvd.TLSDescriptors->at(i).generateSkyline();
	//}	
>>>>>>> parent of 34153d8... DPSkylineMinimal

	/*-------4. 地面站特征匹配------*/
	//特征匹配->结果筛选
	//按扫描仪裁切
	//对每一地面站，从词典中进行特征匹配  
	//vector<vector<PhaseSimilarityResult>> *similarity = new vector<vector<PhaseSimilarityResult>>();
	SimilarityEstimation se;
	se.setMinAngle(io.paralist.minAngle);
	se.setMaxAngle(io.paralist.maxAngle);
	se.setMinDist(io.paralist.minDist);
	se.setMaxDist(io.paralist.maxDist);
	se.setResolution(io.paralist.resolutionSkyDivision);
<<<<<<< HEAD
=======


	int NvMin = int(io.paralist.minAngle / io.paralist.resolutionSkyDivision);
	int NvMax = int(io.paralist.maxAngle / io.paralist.resolutionSkyDivision);
	for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	{
		//滤波并输出skyline特征
		tvd.TLSDescriptors->at(i).filterNoiseBy2DDensity(2, 3);
		tvd.TLSDescriptors->at(i).generateSkylineWithScanAngle(NvMin, NvMax);
	}
	for (int i = 0; i < avd.ALSDescriptors->size(); ++i)
	{
		//输出skyline特征
		avd.ALSDescriptors->at(i).generateSkylineWithScanAngle(NvMin, NvMax);
	}
	se.searchDictionaryBruteForce(tvd, avd, ByDPSkyline, cloudALSViews);
>>>>>>> parent of 34153d8... DPSkylineMinimal
	//相位相关法，2D深度图不做处理
	//ofstream ofs_PhaseCorre("All_PhaseCorre.txt");
	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	Mat imageTLS = tvd.TLSDescriptors->at(i).depthimage;
	//	vector<PhaseSimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		PhaseSimilarityResult tempSimilarity;
	//		Mat imageALS = avd.ALSDescriptors->at(j).depthimage;
	//		se.phaseCorrelateOpenCV(imageTLS, imageALS, tempSimilarity.phase_shift, tempSimilarity.response);
	//      tempSimilarity.alsIndex = j;
	//		curTLSSimilarity.push_back(tempSimilarity);		
	//	}
	//	sort(curTLSSimilarity.begin(),curTLSSimilarity.end());	
	//	for (int k = 0; k < 1; ++k)
	//	{
	//		ofs_PhaseCorre << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs_PhaseCorre << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//输出前50
	//	string saveName;
	//	saveName = "SimilarityResult_PhaseCorre" + to_string(i) +".txt";
	//	ofstream ofs(saveName.c_str());
	//	//ofs << "*******************TLS index：" << i << "*************************" << endl;
	//	for (int k = 0; k < avd.ALSDescriptors->size(); ++k)
	//	{			
	//		ofs << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " " 
	//		    << curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}
	//ofs_PhaseCorre.close();

	////相位相关法，2D图像裁剪去除孤立点
	//ofstream ofs_CutMidFilt("All_CutMidFilt.txt");
	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	Mat imageTLS; 
	//	se.PreProcessByOpenCV(tvd.TLSDescriptors->at(i).depthimage,imageTLS)	;
	//	//imwrite("CutMidFilt"+to_string(i)+".png",imageTLS);
	//	vector<PhaseSimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		PhaseSimilarityResult tempSimilarity;		
	//		Mat imageALS; 
	//		se.PreProcessByOpenCV(avd.ALSDescriptors->at(j).depthimage, imageALS);
	//		se.phaseCorrelateOpenCV(imageTLS, imageALS, tempSimilarity.phase_shift, tempSimilarity.response);
	//      tempSimilarity.alsIndex = j;
	//		curTLSSimilarity.push_back(tempSimilarity);		
	//	}
	//	sort(curTLSSimilarity.begin(), curTLSSimilarity.end());
	//	for (int k = 0; k < 1; ++k)
	//	{
	//		ofs_CutMidFilt << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs_CutMidFilt << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//输出前50
	//	string saveName;
	//	saveName = "SimilarityResult_CutMidFilt" + to_string(i) +".txt";
	//	ofstream ofs(saveName.c_str());
	//	//ofs << "*******************TLS index：" << i << "*************************" << endl;
	//	for (int k = 0; k < avd.ALSDescriptors->size(); ++k)
	//	{			
	//		ofs << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " " 
	//		    << curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}
	//ofs_CutMidFilt.close();

	////按占据状态估计相似性
	//ofstream ofs_OccupationResult("All_Occupation.txt");
	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	vector<PhaseSimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		PhaseSimilarityResult tempSimilarity;
	//		se.similarityByOccupation(tvd.TLSDescriptors->at(i), avd.ALSDescriptors->at(j), tempSimilarity);
	//		tempSimilarity.alsIndex = j;
	//		curTLSSimilarity.push_back(tempSimilarity);
	//	}
	//	sort(curTLSSimilarity.begin(), curTLSSimilarity.end());
	//	LOG(INFO) << "计算第" << i << "站相似性估计，最大相似度为" << curTLSSimilarity[0].response << endl;
	//	//输出前50
	//	string saveName;
	//	saveName = "SimilarityResult_Occupation" + to_string(i) + ".txt";
	//	
	//	for (int k = 0; k < 1; ++k)
	//	{
	//		ofs_OccupationResult << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs_OccupationResult << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	ofstream ofs(saveName.c_str());
	//	
	//	//ofs << "*******************TLS index：" << i << "*************************" << endl;
	//	for (int k = 0; k < avd.ALSDescriptors->size(); ++k)
	//	{
	//		ofs << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}
	//ofs_OccupationResult.close();
	//
	////按天际线估计相似性
	//ofstream ofs_SkylineResult("All_Skyline.txt");
	//for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	//{
	//	vector<PhaseSimilarityResult> curTLSSimilarity;
	//	for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
	//	{
	//		PhaseSimilarityResult tempSimilarity;
	//		se.similarityBySkyLine(tvd.TLSDescriptors->at(i), avd.ALSDescriptors->at(j), tempSimilarity);
	//		tempSimilarity.alsIndex = j;
	//		curTLSSimilarity.push_back(tempSimilarity);
	//	}
	//	sort(curTLSSimilarity.begin(), curTLSSimilarity.end());
	//	for (int k = 0; k < 1; ++k)
	//	{
	//		ofs_SkylineResult << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs_SkylineResult << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//输出前50
	//	string saveName;
	//	saveName = "SimilarityResult_Skyline" + to_string(i) + ".txt";
	//	ofstream ofs(saveName.c_str());
	//	//ofs << "*******************TLS index：" << i << "*************************" << endl;
	//	for (int k = 0; k < avd.ALSDescriptors->size(); ++k)
	//	{
	//		ofs << fixed << setprecision(3)
	//			<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
	//		ofs << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
	//			<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
	//	}
	//	//similarity->push_back(curTLSSimilarity);
	//	ofs.close();
	//}
	//ofs_SkylineResult.close();

	//按天际线+深度估计相似性
	ofstream ofs_SkylineAndDepth("All_SkylineAndDepthLargerALS.txt");
	for (int i = 0; i < tvd.TLSDescriptors->size(); ++i)
	{
		t4 = GetTickCount();
		vector<PhaseSimilarityResult> curTLSSimilarity;
		for (int j = 0; j < avd.ALSDescriptors->size(); ++j)
		{
			PhaseSimilarityResult tempSimilarity;
			se.similarityBySkyLineAndDepth(tvd.TLSDescriptors->at(i), avd.ALSDescriptors->at(j), tempSimilarity);
			tempSimilarity.alsIndex = j;
			curTLSSimilarity.push_back(tempSimilarity);
		}
		sort(curTLSSimilarity.begin(), curTLSSimilarity.end());
		//输出匹配度最高的点
		for (int k = 0; k < 1; ++k)
		{
			ofs_SkylineAndDepth << fixed << setprecision(3)
				<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
			ofs_SkylineAndDepth << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
				<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
		}
		//输出匹配结果
		t5 = GetTickCount();
		LOG(INFO) << "Time for TLS " << to_string(i) << ": " << (t5 - t4)*1.0 / 1000 << " s" << endl;
		string saveName;
		saveName = "SimilarityResult_SkylineAndDepthLargerALS_" + to_string(i) + ".txt";
		ofstream ofs(saveName.c_str());
		//ofs << "*******************TLS index：" << i << "*************************" << endl;
		for (int k = 0; k < avd.ALSDescriptors->size(); ++k)
		{
			ofs << fixed << setprecision(3)
				<< cloudALSViews->points[curTLSSimilarity[k].alsIndex].x << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].y << " " << cloudALSViews->points[curTLSSimilarity[k].alsIndex].z << " ";
			ofs << curTLSSimilarity[k].response << " " << curTLSSimilarity[k].alsIndex << " "
				<< curTLSSimilarity[k].phase_shift.x << " " << curTLSSimilarity[k].phase_shift.y << endl;
		}
		//similarity->push_back(curTLSSimilarity);
		ofs.close();
	}
	ofs_SkylineAndDepth.close();
	

	//对前N相似聚类，去除离群点，确定同名特征
	//区域增长
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

	//计算旋转平移参数R T
}
