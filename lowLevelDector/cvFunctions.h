#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <std_msgs/String.h>
#include "opencv2/imgcodecs.hpp"
#include <math.h>
#include <iostream>
#include "stdio.h"
#include <std_msgs/Bool.h>

using namespace cv;

static int edgeThresh = 15; // edge detection threshhold
static int minHessian = 400;
static int pixelThresh = 300; // threshhold witch
static int bins = 3;
const float ratioThresh =  0.7f;

class cvFunctions
{
public:
 cvFunctions();
 void test();
 cv::Mat importImg(std::string pathToImage);
 void edgeDetection(cv::Mat img);
 void detectKey( cv::Mat img1, cv::Mat img2 );
 void matchKey();
 void showMatches();  
 cv::Mat backproj(cv::Mat img);
 int numberOfBlackPixels(cv::Mat img);
 bool notWater(cv::Mat img); 
 std_msgs::Bool boolMsg; 

 ~cvFunctions();
private:
 cv::Mat edgeImg;
 cv::Mat greyScaleImg;
 cv::Mat cedge;
 cv::Mat descriptors1;
 cv::Mat descriptors2; 
 cv::Mat imgMatches;
 cv::Mat temp1;
 cv::Mat temp2;
 cv::Mat hue;
 std::vector<KeyPoint> keypoints1;
 std::vector<KeyPoint> keypoints2; 
 std::vector<std::vector<DMatch> > knnMatchesVec;
 std::vector<DMatch> goodMatches;
 int totalNumberOfPixels;
 int blackPixels;








};
