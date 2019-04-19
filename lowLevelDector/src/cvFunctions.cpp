#include "cvFunctions.h"


cvFunctions::cvFunctions():it(nH)
{

}

cv::Mat  cvFunctions::importImg(std::string pathToImage)
{


    std::string temp =  pathToImage;
    cv::Mat img = cv::imread(temp);

    if (img.empty()) {
        std::cout << "Input image not found at '" << temp << "'\n";

    }

    return img;
}


void cvFunctions::edgeDetection(cv::Mat img)
{
    cv::Mat temp = img;
    cv::Mat tempgray;

    cv::cvtColor(temp, tempgray, COLOR_BGR2GRAY);
    cv::blur(tempgray, edgeImg, cv::Size(3,3 ) );
    cv::Canny(edgeImg, edgeImg, edgeThresh, edgeThresh*3, 3 );
    cedge = cv::Scalar::all(0);
    temp.copyTo(cedge, edgeImg);
    cv::imshow("edges" , cedge);




};



void cvFunctions::detectKey( cv::Mat img1, cv::Mat img2 )
{
    temp1 = img1;
    cv::Mat temp1Gray1;
    cv::Mat temp2Gray2;
    temp2 = img2;

    cv::cvtColor(temp1, temp1Gray1, COLOR_BGR2GRAY);
    cv::cvtColor(temp2, temp2Gray2, COLOR_BGR2GRAY);


    cv::waitKey();


    cv::Ptr<cv::xfeatures2d::SURF> detector = cv::xfeatures2d::SURF::create(minHessian);
    detector->detectAndCompute(temp1Gray1, noArray(), keypoints1, descriptors1);
    detector->detectAndCompute(temp2Gray2, noArray(), keypoints2, descriptors2);

    matchKey();
    showMatches();

};

void cvFunctions::matchKey()
{

    cv::Ptr<cv::DescriptorMatcher> matcher =  DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_SL2);
    matcher->knnMatch(descriptors1, descriptors2, knnMatchesVec, 2);

    // filter matches

    for (size_t i = 0; i < knnMatchesVec.size(); i++)
    {
        if(knnMatchesVec[i][0], ratioThresh * knnMatchesVec[i][1].distance)
        {
            goodMatches.push_back(knnMatchesVec[i][0]);
        }
    }

    std::cout << " match number " << goodMatches.size() << std::endl;
    
};



void cvFunctions::showMatches()
{
    
    cv::drawMatches( temp1, keypoints1, temp2, keypoints2, goodMatches, imgMatches, Scalar::all(-1),
    Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    cv::namedWindow("good matches", WINDOW_NORMAL);
    cv::resizeWindow("good matches", 600, 600);
    cv::imshow("good matches" , imgMatches);

    cv::waitKey();
};



cv::Mat cvFunctions::backproj(cv::Mat img)
{

    cv::Mat temp = img;


    Mat hsv;

    cv::cvtColor( temp, hsv, COLOR_BGR2HSV );
    hue.create(hsv.size(), hsv.depth());

    int ch[] = { 0, 0 };
    mixChannels( &hsv, 1, &hue, 1, ch, 1 );
    const char* window_image = "Source image";
    int histSize = MAX( bins, 2 );
    float hue_range[] = { 0, 180 };
    const float* ranges = { hue_range };
    Mat hist;
    cv::calcHist( &hue, 1, 0, Mat(), hist, 1, &histSize, &ranges, true, false );
    cv::normalize( hist, hist, 0, 255, NORM_MINMAX, -1, Mat() );
    Mat backproj;
    cv::calcBackProject( &hue, 1, 0, hist, backproj, &ranges, 1, true );
    // cv::namedWindow("BackProj", WINDOW_NORMAL);
    // imshow( "BackProj", backproj );
    // resizeWindow( "BackProj", 300,300);
    imagPublisher(backproj);


    int w = 400, h = 400;
    int bin_w = cvRound( (double) w / histSize );
    Mat histImg = Mat::zeros( h, w, CV_8UC3 );
    for (int i = 0; i < bins; i++)
    {
        rectangle( histImg, Point( i*bin_w, h ), Point( (i+1)*bin_w, h - cvRound( hist.at<float>(i)*h/255.0 ) ),
                   Scalar( 0, 0, 255 ), FILLED );
    }

    //debugging functions
    /*
    namedWindow( window_image );
    createTrackbar("* Hue  bins: ", window_image, &bins, 180 , HistBackproj );
    cv::namedWindow("window_image", WINDOW_NORMAL);
    cv::resizeWindow("window_image", 600, 600);
    imshow( "window_image", img );
    cv::namedWindow("hsv", WINDOW_NORMAL);
    cv::resizeWindow("hsv", 600, 600);
     imshow( "hsv", hsv );
    imshow( "Histogram", histImg );
    resizeWindow( "Histogram", 300,300);
    */
    return  backproj;




};

int cvFunctions::numberOfBlackPixels(cv::Mat img) //function to count the number of pixles
{

  totalNumberOfPixels = 0;
  cv::Mat temp =  img;

  totalNumberOfPixels = temp.rows * temp.cols;
  blackPixels = totalNumberOfPixels - cv::countNonZero(temp);


  return blackPixels;



};



bool cvFunctions::notWater(cv::Mat img) //function which return wether or not water has been detected function.
{
       cv::Mat temp = img;
       if( run == true)
            {
                numberOfBlackPixels(backproj(temp));
                std::cout << "num " <<  blackPixels << std::endl;

                 if(blackPixels > pixelThresh)
                    {
                          std::cout << "not water " << std::endl;
                          boolMsg.data =true; 
                          pub.publish(boolMsg); 
                          return 1;

                    }
                    else
                    {
                        std::cout << "water" << std::endl;
                        boolMsg.data = false; 
                        pub.publish(boolMsg); 
                        return 0;

                    }
            }
        else 
            {
              std::cout << " low detector not active \n";  
            }



};

void cvFunctions::imagPublisher(cv::Mat img)
{
    cv::Mat temp = img; 
    sensor_msgs::ImagePtr imgMsg;
    imgMsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", temp).toImageMsg(); 
    imagePub.publish(imgMsg);


}



void cvFunctions::imageCallback(const sensor_msgs::ImageConstPtr& imgMsg)
{

   cv_bridge::CvImagePtr img_ptr;  
   cv::Mat imgRGB; 
 
  try
  {
     
   

    img_ptr = cv_bridge::toCvCopy(imgMsg, sensor_msgs::image_encodings::BGR8); 
    imgRGB = img_ptr->image;
    notWater(imgRGB);	

    
    //cv::imshow("view", imgRGB);
    //cv::waitKey(10);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgMsg->encoding.c_str());
  }
}




void cvFunctions::msgCallback(const std_msgs::Bool::ConstPtr& msg)
{
   run =  msg->data; 
  
}


cvFunctions::~cvFunctions()
{

};


