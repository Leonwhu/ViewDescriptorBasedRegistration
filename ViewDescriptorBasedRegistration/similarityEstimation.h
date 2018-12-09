#pragma  once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 

#include "viewDescriptor.h"

using namespace cv;


struct PhaseSimilarityResult
{
	int alsIndex;
	//int tlsIndex;
	cv::Point2d phase_shift;
	double response;

	bool operator<(PhaseSimilarityResult b) const
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

	bool similarityBySkyLine(Mat &src1, Mat &src2, PhaseSimilarityResult &sr);
	bool similarityBySkyLine(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);
	bool similarityByOccupation(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);
	bool similarityBySkyLineAndDepth(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);

private:

};


