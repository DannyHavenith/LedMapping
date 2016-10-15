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
#include <opencv2/opencv.hpp>
#include <random>

#include "nm_simplex_solver.hpp"
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/assignment.hpp>

#include <cmath>


using namespace cv;
namespace
{
    const std::string windowName = "LED Finder";
    const int fixedThreshold = 250;
}

typedef std::vector<KeyPoint> KeyPoints;



struct Settings {
    int minDist = 3;
    int minArea = 250;
    int maxArea = 3000;
    int lowerThreshold = 128;
    int upperThreshold = 255;
    int lowerHue = 99;
    int upperHue = 105;
    int blurValue = 9;
};

Settings settings;
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


int main(int argc, char** argv)
{

    if (argc != 3)
    {
        printf("usage: LedMapping <before image> <lit image>\n");
        return -1;
    }

    base = imread(argv[1]);
    lit = imread( argv[2]);
    if (!base.data || !lit.data)
    {
        std::cerr << "No image data!\n";
        return -1;
    }

    TweakParameters();


    waitKey(0);

    return 0;
}

