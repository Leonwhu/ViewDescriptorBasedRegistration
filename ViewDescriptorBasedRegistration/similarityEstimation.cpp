#include "similarityEstimation.h"

#include <iostream>

//bool SimilarityEstimation::phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double *response)
//{
//	Mat dst1, dst2;
//	//cvtColor(src1, src1, CV_BGR2GRAY);     //转换为灰度图像
//	src1.convertTo(dst1, CV_32FC1);       //转换为32位浮点型
//	//cvtColor(src2, src2, CV_BGR2GRAY);
//	src2.convertTo(dst2, CV_32FC1);
//	/*imshow("2", dst2);
//	waitKey(0);
//	getchar();*/
//	Mat hann;
//	createHanningWindow(hann, dst1.size(), CV_32FC1);
//	double res;
//	res = 0.0;
//	response = &res;
//	phase_shift = phaseCorrelateRes(dst1, dst2, hann, response);
//	return true;
//}

bool SimilarityEstimation::PreProcessByOpenCV(Mat& src, Mat& dst/*, int x, int y, int w, int h*/)
{
	//截取高度角在20-120度之间
	float resolution = 4.0;
	float rangeMin = 20.0; float rangeMax = 120.0;
	int cutRow = int(rangeMin / resolution);
	int recRow = int((rangeMax-rangeMin)/resolution);
	dst = src(Rect(0, cutRow, src.cols, recRow));
	//去除孤立点，3*3格网内仅该点有值
	double val[25] = { 0 }, mid = 0;
	int num = 0;
	for (int i = 0; i < dst.rows; i++){
		for (int j = 0; j < dst.cols; j++){
			if(dst.at<uchar>(i, j)==255)
				continue;
			//CvScalar s = cvGet2D(dst, i, j);
			double val[9] = { 0 }, mid = 0;//分别为存放窗口中所有灰度值的数组和中值
			int num = 0;//记录实际窗口中有几个像素点
			//遍历当前像素点为中心的3X3窗口
			for (int k = -1; k <= 1; k++)
				for (int m = -1; m <= 1; m++)
					if (i + k >= 0 && i + k < dst.rows && j + m >= 0 && j + m < dst.cols){//在图像上，没有超出边界
						if(dst.at<uchar>(i+k, j+m)==255)
							num++;
					}
			////冒泡排序，从大到小
			//for (int k = 0; k < 9; k++){
			//	for (int m = 0; m < 8 - k; m++){
			//		if (val[m] < val[m + 1]){
			//			double temp = val[m];
			//			val[m] = val[m + 1];
			//			val[m + 1] = temp;
			//		}
			//	}
			//}
			//mid = val[num / 2];//求中值
			if (num == 8)
			{
				dst.at<uchar>(i, j) = 255;
			}
		}
	}
	return 0;
}

bool SimilarityEstimation::phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double &response)
{
	Mat dst1, dst2;
	//cvtColor(src1, src1, CV_BGR2GRAY);     //转换为灰度图像
	src1.convertTo(dst1, CV_32FC1);       //转换为32位浮点型
	//cvtColor(src2, src2, CV_BGR2GRAY);
	src2.convertTo(dst2, CV_32FC1);
	/*imshow("2", dst2);
	waitKey(0);
	getchar();*/
	Mat hann;
	createHanningWindow(hann, dst1.size(), CV_32FC1);
	double res;
	res = 0.0;
	double *pres = &res;
	phase_shift = phaseCorrelateRes(dst1, dst2, hann, pres);
	response = *pres;
	return true;
}

//计算每个偏移下的相似性，都为空(-1.0)或都不为空(>0.0)则相似性加一
bool SimilarityEstimation::similarityByOccupation(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr)
{
	if (src1.Nh != src2.Nh || src1.Nv != src2.Nv || src1.Nh == 0 || src1.Nv == 0)
	{
		cout << "计算相似性的两特征维度不一致！" << endl;
		return false;
	}
	if (src1.viewDepth.empty() || src2.viewDepth.empty())
	{
		cout << "计算相似性的两特征深度值未初始化！" << endl;
		return false;
	}
	vector<PhaseSimilarityResult> res;
	for (int deltaCol = 0; deltaCol < src1.Nh; ++deltaCol)
	{
		PhaseSimilarityResult tempSR;
		int numSame = 0;
		for (int j = 0; j < src1.Nh; ++j)		
		{
			for (int i = 0; i <= src1.Nv / 2; ++i)
			{
				int deltaJ = j + deltaCol < src1.Nh ? j + deltaCol : j + deltaCol - src1.Nh;
				int index1 = j + i * src1.Nh;
				int index2 = deltaJ + i * src1.Nh;
				if (src1.viewDepth[index1] * src2.viewDepth[index2] > 0.0)
					numSame++;
			}
		}
		tempSR.response = double(numSame) / (src1.Nh*src1.Nv/2);
		tempSR.phase_shift.x = deltaCol;
		tempSR.phase_shift.y = 0;
		res.push_back(tempSR);
	}
	std::sort(res.begin(), res.end()/*, this->cmpSimilarityResult*/);
	sr = res[0];
	return 1;
}

//计算每个偏移下的相似性，以最高遮挡角判断相似个数，输入应该是未切割过的图像（45*90），输入的TLS特征应预先去噪
bool SimilarityEstimation::similarityBySkyLine(Mat &src1, Mat &src2, PhaseSimilarityResult &sr)
{
	vector<PhaseSimilarityResult> res;
	for (int deltaCol = 0; deltaCol < src1.cols; ++deltaCol)
	{
		PhaseSimilarityResult tempSR;
		int numSame = 0;
		int maxAngle1 = 0, maxAngle2 = 0;
		cvtColor(src1, src1, CV_8UC1);
		for (int j = 0; j < src1.cols; ++j)
		{
			
			for (int i = 0; i < src1.rows; ++i)
			{
				std::cout << src1.at<uchar>(i, j) << std::endl;
				if (src1.at<uchar>(i, j) != 255)
					maxAngle1 = i;
				    break;
			}
			for (int i = 0; i < src1.rows; ++i)
			{
				int deltaJ = j + deltaCol < src1.cols ? j + deltaCol : j + deltaCol - src1.cols;
				if (src2.at<uchar>(i, deltaJ) != 255)
					maxAngle2 = i;
				    break;
			}
			numSame += src1.rows / 2 - abs(maxAngle1 - maxAngle2);
		}
		
		tempSR.response = double(2*numSame) / (src1.rows*src1.cols);
		tempSR.phase_shift.x = deltaCol;
		tempSR.phase_shift.y = 0;
		res.push_back(tempSR);
	}
	std::sort(res.begin(), res.end()/*, cmpSimilarityResult*/);
	sr = res[0];
	return 1;
}

bool SimilarityEstimation::similarityBySkyLine(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr)
{
	if (src1.Nh != src2.Nh || src1.Nv != src2.Nv)
	{
		cout << "计算相似性的两特征维度不一致！" << endl;
		return false;
	}
	if (src1.viewDepth.empty() || src2.viewDepth.empty())
	{
		cout << "计算相似性的两特征深度值未初始化！" << endl;
		return false;
	}
	vector<PhaseSimilarityResult> res;
	for (int deltaCol = 0; deltaCol < src1.Nh; ++deltaCol)
	{
		PhaseSimilarityResult tempSR;
		int numSame = 0;
		int maxAngle1 = 0, maxAngle2 = 0;
		for (int j = 0; j < src1.Nh; ++j)
		{
			for (int i = 0; i < src1.Nv; ++i)
			{
				if (src1.viewDepth[j + i*src1.Nh] > 0.0)
				{
					maxAngle1 = i;
					break;
				}			
			}
			for (int i = 0; i < src1.Nv; ++i)
			{
				int deltaJ = j + deltaCol < src1.Nh ? j + deltaCol : j + deltaCol - src1.Nh;
				if (src2.viewDepth[deltaJ + i*src2.Nh] > 0.0)
				{
					maxAngle2 = i;
					break;
				}				
			}
			if (maxAngle1 > src1.Nv / 2) maxAngle1 = src1.Nv / 2;
			if (maxAngle2 > src1.Nv / 2) maxAngle2 = src1.Nv / 2;

			numSame += src1.Nv / 2 - abs(maxAngle1 - maxAngle2);
		}

		tempSR.response = double(2 * numSame) / (src1.Nh*src1.Nv);
		tempSR.phase_shift.x = deltaCol;
		tempSR.phase_shift.y = 0;
		res.push_back(tempSR);
	}
	std::sort(res.begin(), res.end()/*, cmpSimilarityResult*/);
	sr = res[0];
	return 1;
}

bool SimilarityEstimation::similarityBySkyLineAndDepth(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr)
{
	if (src1.Nh != src2.Nh || src1.Nv != src2.Nv)
	{
		cout << "计算相似性的两特征维度不一致！" << endl;
		return false;
	}
	if (src1.viewDepth.empty() || src2.viewDepth.empty())
	{
		cout << "计算相似性的两特征深度值未初始化！" << endl;
		return false;
	}

	//考虑的天空高度角的最小差
	int minSkyDiff = 3;
	float rDepth = 5.0;

	vector<PhaseSimilarityResult> res;
	for (int deltaCol = 0; deltaCol < src1.Nh; ++deltaCol)
	{
		PhaseSimilarityResult tempSR;
		float numSame = 0.0, gridSimilarity = 0.0, depthSimilarity = 0.0;
		int maxAngle1 = -1, maxAngle2 = -1;
		for (int j = 0; j < src1.Nh; ++j)
		{
			int deltaJ = j + deltaCol < src1.Nh ? j + deltaCol : j + deltaCol - src1.Nh;
			for (int i = NvMin; i <= NvMax; ++i)
			{
				if (src1.viewDepth[j + i*src1.Nh] > 0.0)
				{
					maxAngle1 = i;
					break;
				}
			}
			for (int i = NvMin; i <= NvMax; ++i)
			{				
				if (src2.viewDepth[deltaJ + i*src2.Nh] > 0.0)
				{
					maxAngle2 = i;
					break;
				}
			}
			//如果其中一列高度角为空，则相似度记为0
			if (maxAngle1 < 0 || maxAngle2 < 0 )
				continue;

			/*if (maxAngle1 > NvMax) maxAngle1 = NvMax;
			if (maxAngle2 > NvMax) maxAngle2 = NvMax;*/

			//判断最高遮挡角之差是否在考虑范围内
			int gridD = abs(maxAngle1 - maxAngle2);
			if (gridD <= minSkyDiff)
			{
				float depthD = (src1.viewDepth[j + maxAngle1*src1.Nh] - src2.viewDepth[deltaJ + maxAngle2*src2.Nh]);
				gridSimilarity = exp(0.0 - float(gridD*gridD) / minSkyDiff/minSkyDiff);
				depthSimilarity = exp(0.0 - depthD*depthD / rDepth / rDepth);;
				numSame += gridSimilarity*depthSimilarity;
			}			
		}
		tempSR.response = numSame / src1.Nh;
		tempSR.phase_shift.x = deltaCol;
		tempSR.phase_shift.y = 0;
		res.push_back(tempSR);
	}
	std::sort(res.begin(), res.end()/*, cmpSimilarityResult*/);
	sr = res[0];
	return 1;
}