//
//  Copyright (C) 2016 Danny Havenith
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//

/**
 * Some experiments with using OpenCV to find LEDs in an Image.
 */

#include <iostream>
#include <utility>
#include <tuple>
#include <opencv2/opencv.hpp>
#include <random>

#include <cmath>
#include <stdexcept>
#include <string>


using namespace cv;
namespace
{
    const std::string windowName = "LED Finder";
    const int fixedThreshold = 250;
}




/*************************************************************
Mat base;
Mat lit;

void ShowTweaked( int, void* )
{
    Mat hsv;
    Mat diff = lit - base;
    cvtColor( diff, hsv, CV_BGR2HSV);
    GaussianBlur( hsv, hsv, Size{ 1 + 2 * settings.blurValue, 1 + 2 * settings.blurValue}, 0);
    Mat bitmask;
    inRange( hsv,
            Scalar( settings.lowerHue, 0, settings.lowerThreshold),
            Scalar( settings.upperHue, 255, settings.upperThreshold),
            bitmask);

    SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = settings.minDist;
    params.filterByInertia = false;

    params.filterByConvexity = true;
    params.minConvexity = 0.5;
    params.maxConvexity = 1.1;

    params.filterByColor = true;
    params.blobColor = 255;

    params.filterByArea = true;
    params.minArea = settings.minArea;
    params.maxArea = settings.maxArea;

    params.minThreshold = 150;
    params.maxThreshold = 254;

    params.filterByCircularity = true;
    params.minCircularity = .5;
    params.maxCircularity = 1.1;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    std::vector<KeyPoint> features;
    detector->detect(bitmask, features);

    std::cout << "Found " << features.size() << " features\n";

    Mat dst;
    drawKeypoints(lit, features, dst, Scalar::all(-1),
            DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow(windowName, dst);

    Mat onBitmask;
    drawKeypoints( bitmask, features, onBitmask, Scalar::all(-1),
            DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow( "base image", onBitmask);
}


void TweakParameters( )
{
    namedWindow(windowName, WINDOW_AUTOSIZE);
    imshow( "base image", lit);

    createTrackbar( "min distance",
                    windowName, &settings.minDist,
                    500, ShowTweaked );

    createTrackbar( "min Area",
                    windowName, &settings.minArea,
                    2000, ShowTweaked );
    createTrackbar( "max Area",
                    windowName, &settings.maxArea,
                    2000, ShowTweaked );

    createTrackbar( "lower Treshold",
                    windowName, &settings.lowerThreshold,
                    300, ShowTweaked );
    createTrackbar( "upper Threshold",
                    windowName, &settings.upperThreshold,
                    300, ShowTweaked );

    createTrackbar( "lower Hue",
                    windowName, &settings.lowerHue,
                    300, ShowTweaked );
    createTrackbar( "upper Hue",
                    windowName, &settings.upperHue,
                    300, ShowTweaked );
    createTrackbar( "blur",
                    windowName, &settings.blurValue,
                    10, ShowTweaked );

    ShowTweaked(0,nullptr);
}
***************************************************/

void ShowTweaked( int, void*);

class LedDetector
{
public:
    LedDetector( const std::string &fileName)
    :m_fileName{ fileName}
    {
        Setup();
        ScanSequence();
    }

    void ScanSequence( )
    {
        VideoCapture video;
        video.open( m_fileName);
        if (!video.isOpened())
        {
            throw std::runtime_error(std::string{"Can't open file "} + m_fileName);
        }

        video >> m_previous;
        ShowDetected();
        while( video.read( m_current))
        {
            if (Update())
            {
                // skip next frame if LED detected
                video.read( m_previous);
            }
            else
            {
                m_previous = std::move( m_current);
            }
        }

        std::cout << "Detected " << m_foundLeds.size() << "LEDs.\n";
        ShowDetected();
    }

    void Setup()
    {
        namedWindow(windowName, WINDOW_AUTOSIZE);

        createTrackbar( "min distance",
                        windowName, &settings.minDist,
                        500, ShowTweaked, this);

        createTrackbar( "min Area",
                        windowName, &settings.minArea,
                        2000, ShowTweaked, this );
        createTrackbar( "max Area",
                        windowName, &settings.maxArea,
                        2000, ShowTweaked, this);

        createTrackbar( "lower Treshold",
                        windowName, &settings.lowerThreshold,
                        300, ShowTweaked, this);
        createTrackbar( "upper Threshold",
                        windowName, &settings.upperThreshold,
                        300, ShowTweaked, this);

        createTrackbar( "lower Hue",
                        windowName, &settings.lowerHue,
                        300, ShowTweaked, this);
        createTrackbar( "upper Hue",
                        windowName, &settings.upperHue,
                        300, ShowTweaked, this);
        createTrackbar( "blur",
                        windowName, &settings.blurValue,
                        10, ShowTweaked, this);

    }

    void Feed( const Mat &current, const Mat &previous)
    {
        m_current = current;
        m_previous = previous;
        Update();
        auto key = waitKey(20);
        std::cout << "received key: " << key << std::endl;
    }

    std::vector<KeyPoint> GetResults() const
    {
        return m_foundLeds;
    }

    bool Update( )
    {

        Mat bgr[3];
        split( m_current - m_previous, bgr);

        Mat analysis;
        GaussianBlur( bgr[2], analysis, Size{ 1 + 2 * settings.blurValue, 1 + 2 * settings.blurValue}, 0);

        inRange( analysis,
                Scalar( settings.lowerThreshold),
                Scalar( settings.upperThreshold),
                analysis);

        SimpleBlobDetector::Params params;
        params.minDistBetweenBlobs = settings.minDist;
        params.filterByInertia = false;

        params.filterByConvexity = true;
        params.minConvexity = 0.5;
        params.maxConvexity = 1.1;

        params.filterByColor = true;
        params.blobColor = 255;

        params.filterByArea = true;
        params.minArea = settings.minArea;
        params.maxArea = settings.maxArea;

        params.minThreshold = 150;
        params.maxThreshold = 254;

        params.filterByCircularity = true;
        params.minCircularity = .5;
        params.maxCircularity = 1.1;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        std::vector<KeyPoint> features;
        detector->detect(analysis, features);

        if (features.size() == 1)
        {
            m_foundLeds.push_back( features[0]);
        }
        else if (features.size() > 8)
        {
            m_foundLeds.clear();
        }

        return features.size() == 1;
    }

    void ShowDetected()
    {
        Mat allFeatures;
        drawKeypoints( m_previous, m_foundLeds, allFeatures, Scalar::all(-1),
                DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        imshow( windowName, allFeatures);
    }

private:
    typedef std::vector<KeyPoint> KeyPoints;

    struct Settings {
        int minDist = 3;
        int minArea = 70;
        int maxArea = 3000;
        int lowerThreshold = 65;
        int upperThreshold = 255;
        int lowerHue = 99;
        int upperHue = 105;
        int blurValue = 9;
    };

    Settings settings;
    std::vector<KeyPoint> m_foundLeds;
    Mat m_current;
    Mat m_previous;
    std::string m_fileName;
};

void ShowTweaked( int, void *detector )
{
    reinterpret_cast<LedDetector*>( detector)->ScanSequence();
}

void PrintResult( const std::vector<KeyPoint> &results)
{
    std::cout << "Found " << results.size() << " LEDs\n";
    Point2f lowerLeft = results[0].pt;
    Point2f upperRight = results[0].pt;
    for ( const auto &point: results)
    {
        const auto pt = point.pt;
        if (pt.x < lowerLeft.x) lowerLeft.x = pt.x;
        if (pt.y < lowerLeft.y) lowerLeft.y = pt.y;
        if (pt.x > upperRight.x) upperRight.x = pt.x;
        if (pt.y > upperRight.y) upperRight.y = pt.y;
    }

    auto xRange = upperRight.x - lowerLeft.x;
    auto yRange = upperRight.y - lowerLeft.y;

    for ( const auto &point: results)
    {
        auto x = static_cast<int>( 255 * ((point.pt.x - lowerLeft.x) / xRange));
        auto y = static_cast<int>( 255 * ((point.pt.y - lowerLeft.y)/ yRange));

        std::cout << "{ " << x << ", " << y << "},\n";
    }
}

int main(int argc, char** argv)
{

    if (argc != 2)
    {
        printf("usage: LedMapping <video>\n");
        return -1;
    }

    try
    {
        LedDetector detector{argv[1]};
        waitKey(0);

        PrintResult( detector.GetResults());
    }
    catch( cv::Exception& e )
    {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
    }
    return 0;
}

