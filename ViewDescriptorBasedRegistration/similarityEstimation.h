#pragma  once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 

using namespace cv;


struct SimilarityResult
{
	int alsIndex;
	//int tlsIndex;
	cv::Point2d phase_shift;
	double response;

	bool operator<(SimilarityResult b) const
	{
		if (response> b.response) {
			return true;
		}
		return false;
	}
};
//bool cmpSimilarityResult(SimilarityResult &sr1, SimilarityResult &sr2)
//{
//	return sr1.response > sr2.response;
//}

class SimilarityEstimation
{
public:
	//bool phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double *response);
	int PreProcessByOpenCV(Mat& src, Mat& dst/*, int x, int y, int w, int h*/);

	//Ωµ–Ú≈≈–Ú
	
	bool phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double &response);
	bool similarityByOccupation(Mat &src1, Mat &src2, SimilarityResult &sr);
	bool similarityBySkyLine(Mat &src1, Mat &src2, SimilarityResult &sr);
private:

};


