#pragma  once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 

#include "viewDescriptor.h"
#include "ALSViewDescriptor.h"
#include "TLSViewDescriptor.h"

using namespace cv;

enum SimilarityMeasurement
{
	ByPhaseCorre, ByOccupation, BySkyline, BySkylineAndDepth, ByDPSkyline
};

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

	void setMinDist(float minDistance){ minDist = minDistance; };
	void setMaxDist(float maxDistance){ maxDist = maxDistance; };
	void setMinAngle(float minTLSAngle){ minAngle = minTLSAngle; };
	void setMaxAngle(float maxTLSAngle){ maxAngle = maxTLSAngle; };
	void setResolution(float skyResolution){ 
		resolution = skyResolution; 
		NvMin = int(minAngle / resolution); 
		NvMax = int(maxAngle / resolution); };

	//bool phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double *response);
	bool PreProcessByOpenCV(Mat& src, Mat& dst/*, int x, int y, int w, int h*/);

	//Ωµ–Ú≈≈–Ú
	
	bool phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double &response);

	bool similarityBySkyLine(Mat &src1, Mat &src2, PhaseSimilarityResult &sr);
	bool similarityBySkyLine(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);
	bool similarityByOccupation(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);
	bool similarityBySkyLineAndDepth(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);

	bool similarityDP(Skyline3DContour &sky1, Skyline3DContour &sky2, PhaseSimilarityResult &sr);
	PhaseSimilarityResult similaritySkylineOneDP(Skyline3DContour &sky1, Skyline3DContour &sky2, int deltaJ, int width);

	bool searchDictionaryBruteForce(TLSViewDescriptor &tvd, ALSViewDescriptor &avd, SimilarityMeasurement sm, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSViews);

private:
	float minDist;
	float maxDist;
	float minAngle;
	float maxAngle;
	float resolution;
	int NvMin;
	int NvMax;
};


