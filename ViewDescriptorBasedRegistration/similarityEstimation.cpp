#include "similarityEstimation.h"

#include <iostream>

//bool SimilarityEstimation::phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double *response)
//{
//	Mat dst1, dst2;
//	//cvtColor(src1, src1, CV_BGR2GRAY);     //ת��Ϊ�Ҷ�ͼ��
//	src1.convertTo(dst1, CV_32FC1);       //ת��Ϊ32λ������
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

int SimilarityEstimation::PreProcessByOpenCV(Mat& src, Mat& dst/*, int x, int y, int w, int h*/)
{
	//��ȡ�߶Ƚ���30-130��֮��
	float resolution = 4.0;
	float rangeMin = 20.0; float rangeMax = 140.0;
	int cutRow = int(rangeMin / resolution);
	int recRow = int((rangeMax-rangeMin)/resolution);
	dst = src(Rect(0, cutRow, src.cols, recRow));
	//ȥ�������㣬3*3�����ڽ��õ���ֵ
	double val[25] = { 0 }, mid = 0;
	int num = 0;
	for (int i = 0; i < dst.rows; i++){
		for (int j = 0; j < dst.cols; j++){
			if(dst.at<uchar>(i, j)==255)
				continue;
			//CvScalar s = cvGet2D(dst, i, j);
			double val[9] = { 0 }, mid = 0;//�ֱ�Ϊ��Ŵ��������лҶ�ֵ���������ֵ
			int num = 0;//��¼ʵ�ʴ������м������ص�
			//������ǰ���ص�Ϊ���ĵ�3X3����
			for (int k = -1; k <= 1; k++)
				for (int m = -1; m <= 1; m++)
					if (i + k >= 0 && i + k < dst.rows && j + m >= 0 && j + m < dst.cols){//��ͼ���ϣ�û�г����߽�
						if(dst.at<uchar>(i+k, j+m)==255)
							num++;
					}
			////ð�����򣬴Ӵ�С
			//for (int k = 0; k < 9; k++){
			//	for (int m = 0; m < 8 - k; m++){
			//		if (val[m] < val[m + 1]){
			//			double temp = val[m];
			//			val[m] = val[m + 1];
			//			val[m + 1] = temp;
			//		}
			//	}
			//}
			//mid = val[num / 2];//����ֵ
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
	//cvtColor(src1, src1, CV_BGR2GRAY);     //ת��Ϊ�Ҷ�ͼ��
	src1.convertTo(dst1, CV_32FC1);       //ת��Ϊ32λ������
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

//����ÿ��ƫ���µ������ԣ���Ϊ�ջ򶼲�Ϊ���������Լ�һ
bool SimilarityEstimation::similarityByOccupation(Mat &src1, Mat &src2, SimilarityResult &sr)
{
	vector<SimilarityResult> res;
	for (int deltaCol = 0; deltaCol < src1.cols; ++deltaCol)
	{
		SimilarityResult tempSR;
		int numSame = 0;
		for (int i = 0; i < src1.rows; ++i)
		{
			for (int j = 0; j < src1.cols; ++j)
			{
				int deltaJ = j + deltaCol < src1.cols ? j + deltaCol : j + deltaCol - src1.cols;
				if (src1.at<uchar>(i, j) == 255 && src2.at<uchar>(i, deltaJ) == 255)
					numSame++;
				if (src1.at<uchar>(i, j) != 255 && src2.at<uchar>(i, deltaJ) != 255)
					numSame++;
			}
		}
		tempSR.response = double(numSame) / (src1.rows*src1.cols);
		tempSR.phase_shift.x = deltaCol;
		tempSR.phase_shift.y = 0;
		res.push_back(tempSR);
	}
	std::sort(res.begin(), res.end()/*, this->cmpSimilarityResult*/);
	sr = res[0];
	return 1;
}

//����ÿ��ƫ���µ������ԣ�������ڵ����ж����Ƹ���������Ӧ����δ�и����ͼ��45*90��
bool SimilarityEstimation::similarityBySkyLine(Mat &src1, Mat &src2, SimilarityResult &sr)
{
	vector<SimilarityResult> res;
	for (int deltaCol = 0; deltaCol < src1.cols; ++deltaCol)
	{
		SimilarityResult tempSR;
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