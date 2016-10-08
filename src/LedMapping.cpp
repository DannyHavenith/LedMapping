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

class LedFinder
{
public:
    LedFinder(const Mat &baseImage, unsigned int expectedCount) :
            m_expectedCount{ expectedCount },
            m_baseImage{baseImage}
    {
    }

    typedef std::vector<KeyPoint> KeyPoints;
    typedef boost::numeric::ublas::c_vector<double, 3> Parameters;

    static double clip( double min, double max, double value)
    {
        return min + std::fmod( value - min, max - min);
    }

    static void CookBeforeDetection( const Mat &source, Mat &cooked)
    {
        cooked = source.clone();
        //GaussianBlur( source, cooked, Size{ 3,3}, 0);
        //blur( source, cooked, Size{5, 5});
        threshold(cooked, cooked, 254, 255, THRESH_BINARY);
    }

    KeyPoints FindLeds(  double minDistance, double minArea, double maxArea) const
    {
        using std::vector;

        Mat cooked;
        CookBeforeDetection( m_baseImage, cooked);


        SimpleBlobDetector::Params params;
        params.minDistBetweenBlobs = 1;
        params.filterByInertia = false;
        params.filterByConvexity = false;

        params.filterByColor = true;
        params.blobColor = 255;

        params.filterByArea = true;
        params.minArea = 1;
        params.maxArea = maxArea;

        params.minThreshold = 254;
        params.maxThreshold = 256;

        params.filterByCircularity = true;
        params.minCircularity = .5;
        params.maxCircularity = 1.1;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        vector<KeyPoint> features;
        detector->detect(cooked, features);
        return features;
    }

    KeyPoints FindLeds( Parameters parameters) const
    {
        // square the parameters to keep them in positive space.
        parameters = element_prod( parameters, parameters);
        return FindLeds( parameters[0], parameters[1], parameters[2]);
    }

    /**
     * Calculate the cost of a container of features.
     * If the container is empty, we'll return a random, very high cost result
     *
     */
    double CostOfFeatures( const KeyPoints &points) const
    {
        static const double minimalSize = 5;
        if (points.empty())
        {
            // there's a huge 'desert' in the parameter landscape where
            // we find no features at all. To keep the simplex moving,
            // randomize the landscape in this zone
            static std::default_random_engine generator;
            static std::uniform_real_distribution<double> distribution(7, 100);
            double noise = distribution(generator);
            return m_expectedCount * 100 + noise;
        }

        // calculate the squared deviation (std deviation without the sqrroot)
        double average = 0.0;
        for (const auto &point: points)
        {
            average += point.size;
        }
        average /= points.size();

        double deviation = 0.0;
        double smallness = 0.0;
        for (const auto &point: points)
        {
            deviation += (point.size - average) * (point.size - average);
            if (point.size < minimalSize)
            {
                smallness += minimalSize - point.size;
            }

        }

        // this would have been std deviation if I had taken the square root
        // before division, but that is costly and unnecessary here.
        deviation /= points.size();

        return (points.size() - m_expectedCount) * (points.size() - m_expectedCount)
                + deviation * 10;
    }

    /**
     * Function call operator that allows objects of this class to be used
     * as a cost function for minimalization algorithms.
     */
    double operator()(
            const Parameters &parameters) const
    {
        return CostOfFeatures( FindLeds( parameters));
    }

private:
    unsigned int    m_expectedCount;
    Mat             m_baseImage;

};


void ShowResults(const Mat &source, const LedFinder::KeyPoints &features)
{
    std::cout << "Found " << features.size() << " features\n";

    Mat dst;
    drawKeypoints(source, features, dst, Scalar::all(-1),
            DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    imshow(windowName, dst);
}

void RunNelderMead( const Mat &base, const Mat &lit)
{
    Mat hsv;
    cvtColor( lit, hsv, CV_BGR2HSV);

    std::vector<Mat> colors;
    split(hsv, colors);

    Mat &baseForSolving = colors[2];

    imshow( "reds", colors[2]);
    imshow( "original", lit);
    namedWindow(windowName, WINDOW_AUTOSIZE);

    // find the best settings for OpenCV to detect the LEDs.
    const int blue = 2;
    LedFinder ledFinder{ baseForSolving, 50};
    Solvers::NmSimplexSolver<3, LedFinder &> solver( ledFinder, 80, 3, true);
    LedFinder::Parameters p;
    p <<= 1,2,3;
    auto solution = solver.FindMinimun(
            boost::numeric::ublas::zero_vector<double>(3), 100);


    // show the result of those settings
    std::cout << "Parameters: " << solution << '\n';
    auto results = ledFinder.FindLeds( solution);
    Mat thresholded;
    LedFinder::CookBeforeDetection( baseForSolving, thresholded);
    ShowResults( lit, results);
}

int minDist = 1;
int minArea = 1;
int maxArea = 800;
int lowerThreshold = 100;
int upperThreshold = 255;
int lowerHue = 15;
int upperHue = 115;
int blurValue = 2;
Mat tweakBase;
Mat baseImage;

void ShowTweaked( int, void* )
{
    Mat cooked;
    GaussianBlur( tweakBase, cooked, Size{ 1 + 2 * blurValue, 1 + 2 * blurValue}, 0);;
    inRange( cooked,
            Scalar( lowerHue, 0, lowerThreshold),
            Scalar( upperHue, 255, upperThreshold),
            cooked);

    SimpleBlobDetector::Params params;
    params.minDistBetweenBlobs = minDist;
    params.filterByInertia = false;

    params.filterByConvexity = true;
    params.minConvexity = 0.5;
    params.maxConvexity = 1.1;

    params.filterByColor = true;
    params.blobColor = 255;

    params.filterByArea = true;
    params.minArea = minArea;
    params.maxArea = maxArea;

    params.minThreshold = 150;
    params.maxThreshold = 254;

    params.filterByCircularity = true;
    params.minCircularity = .5;
    params.maxCircularity = 1.1;

    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
    std::vector<KeyPoint> features;
    detector->detect(cooked, features);

    std::cout << "Found " << features.size() << " features\n";

    Mat dst;
    drawKeypoints(cooked, features, dst, Scalar::all(-1),
            DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


    imshow(windowName, dst);
    Mat orig;
    drawKeypoints(baseImage, features, orig, Scalar::all(-1),
            DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    imshow( "base image", orig);

}


void TweakParameters( const Mat &base, const Mat &lit)
{
    namedWindow(windowName, WINDOW_AUTOSIZE);
    baseImage = lit;
    imshow( "base image", lit);

    createTrackbar( "min distance",
                    windowName, &minDist,
                    500, ShowTweaked );

    createTrackbar( "min Area",
                    windowName, &minArea,
                    2000, ShowTweaked );
    createTrackbar( "max Area",
                    windowName, &maxArea,
                    2000, ShowTweaked );

    createTrackbar( "lower Treshold",
                    windowName, &lowerThreshold,
                    300, ShowTweaked );
    createTrackbar( "upper Threshold",
                    windowName, &upperThreshold,
                    300, ShowTweaked );

    createTrackbar( "lower Hue",
                    windowName, &lowerHue,
                    300, ShowTweaked );
    createTrackbar( "upper Hue",
                    windowName, &upperHue,
                    300, ShowTweaked );
    createTrackbar( "blur",
                    windowName, &blurValue,
                    10, ShowTweaked );

    cvtColor( lit, tweakBase, CV_BGR2HSV);
    std::vector<Mat> hsv;
//    split(tweakBase, hsv);
//    imshow("h", hsv[0]);
//    imshow("s", hsv[1]);
//    imshow("v", hsv[2]);


    ShowTweaked(0,nullptr);
}

Mat MakeDifferenceImage( const Mat &base, const Mat &lit)
{
    std::vector<Mat> rgb1;
    std::vector<Mat> rgb2;

    split(base, rgb1);
    split( lit, rgb2);

    return (rgb1[2] - rgb2[2]);
}

int main(int argc, char** argv)
{

    if (argc != 3)
    {
        printf("usage: LedMapping <before image> <lit image>\n");
        return -1;
    }

    Mat base = imread(argv[1]);
    Mat lit = imread( argv[2]);
    if (!base.data || !lit.data)
    {
        std::cerr << "No image data!\n";
        return -1;
    }

    //RunNelderMead( base, lit-base);
    //TweakParameters( base, lit);
    //imshow( "diff", MakeDifferenceImage(base, lit));
    waitKey(0);

    return 0;
}

