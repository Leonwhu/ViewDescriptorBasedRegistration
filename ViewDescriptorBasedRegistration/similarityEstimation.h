#pragma  once
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp> 

#include "dataIo.h"
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
		NvMax = int(maxAngle / resolution); 
	};
	void setWidthDP(int width){ widthDP = width; };
	void setParamsByParalist(Paralist &paralist)
	{
		setMinDist(paralist.minDist);
		setMaxDist(paralist.maxDist);
		setMinAngle(paralist.minAngle);
		setMaxAngle(paralist.maxAngle);
		setResolution(paralist.resolutionSkyDivision);
		setWidthDP(paralist.widthDP);
	};

	//bool phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double *response);
	bool PreProcessByOpenCV(Mat& src, Mat& dst/*, int x, int y, int w, int h*/);

	//降序排序
	
	bool phaseCorrelateOpenCV(Mat &src1, Mat &src2, Point2d &phase_shift, double &response);

	bool similarityBySkyLine(Mat &src1, Mat &src2, PhaseSimilarityResult &sr);
	bool similarityBySkyLine(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);
	bool similarityByOccupation(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);
	bool similarityBySkyLineAndDepth(ViewDescriptor &src1, ViewDescriptor &src2, PhaseSimilarityResult &sr);

	//TLS与ALS天际线匹配，相似度计算时需考虑扫描角问题
	bool similarityDPT2A(Skyline3DContour &tlsSkyline, Skyline3DContour &alsSkyline, PhaseSimilarityResult &sr);
	PhaseSimilarityResult similaritySkylineOneDPT2A(Skyline3DContour &tlsSkyline, Skyline3DContour &alsSkyline, int deltaJ, int width);

	bool searchDictionaryBruteForce(TLSViewDescriptor &tvd, ALSViewDescriptor &avd, SimilarityMeasurement sm, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudALSViews);

private:
	int widthDP;
	float minDist;
	float maxDist;
	float minAngle;
	float maxAngle;
	float resolution;
	int NvMin;
	int NvMax;
};


