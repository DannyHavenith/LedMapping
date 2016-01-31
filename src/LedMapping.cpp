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

using namespace cv;

class LedFinder
{
public:
    LedFinder(const Mat &baseImage, unsigned int expectedCount) :
            m_expectedCount{ expectedCount },
            m_baseImage{baseImage}
    {
    }

    typedef std::vector<KeyPoint> KeyPoints;
    typedef boost::numeric::ublas::c_vector<double, 4> Parameters;

    static KeyPoints FindLeds( const Mat &source, const Parameters &parameters)
    {
        using std::vector;
        Mat cooked;

        threshold(source, cooked, 240, 255, THRESH_BINARY);

        SimpleBlobDetector::Params params;
        params.minDistBetweenBlobs = parameters[1]; // min_dist;
        params.filterByInertia = false;
        params.filterByConvexity = false;
        params.filterByColor = false;
        params.filterByCircularity = false;
        params.filterByArea = true;
        params.minArea = parameters[2]; //(float) min_area;
        params.maxArea = parameters[3]; //(float) max_area;

        Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
        vector<KeyPoint> features;
        detector->detect(cooked, features);
        return features;

    }

    /**
     * Function call operator that allows objects of this class to be used
     * as a cost function for minimalization algorithms.
     */
    double operator()(
            const Parameters &parameters) const
    {

        auto features = FindLeds( m_baseImage, parameters);

        // the cost is defined as how far off we are of the expected
        // light count
        if (features.size() > m_expectedCount)
        {
            return (features.size() - m_expectedCount);
        }
        else if (features.empty())
        {
            // there's a huge 'desert' in the parameter landscape where
            // we find no features at all. To keep the simplex moving,
            // randomize the landscape in this zone
            static std::default_random_engine generator;
            static std::uniform_real_distribution<double> distribution(7, 100);
            double noise = distribution(generator);
            return m_expectedCount + noise;
        }
        else
        {
            return m_expectedCount - features.size();
        }
    }

private:
    unsigned int    m_expectedCount;
    Mat             m_baseImage;

};


void ShowResults(Mat &source, const LedFinder::KeyPoints &features)
{
    std::cout << "Found " << features.size() << " features\n";

    Mat dst;
    drawKeypoints(source, features, dst, Scalar::all(-1),
            DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    const std::string window_name = "LED Finder";
//    namedWindow(window_name, WINDOW_AUTOSIZE);
    imshow(window_name, dst);
}

int main(int argc, char** argv)
{

    if (argc != 2)
    {
        printf("usage: LedMapping <Image_Path>\n");
        return -1;
    }

    Mat image = imread(argv[1], 1);
    if (!image.data)
    {
        printf("No image data \n");
        return -1;
    }

    // get the blue component out of the image.
    std::vector<Mat> colors;
    split(image, colors);

    // find the best settings for OpenCV to detect the LEDs.
    const int blue = 2;
    Solvers::NmSimplexSolver<4, LedFinder> solver(LedFinder{ colors[blue], 50 }, 80, 3);
    auto solution = solver.FindMinimun(
            boost::numeric::ublas::zero_vector<double>(4));


    // show the result of those settings
    auto results = LedFinder::FindLeds( colors[blue], solution);
    ShowResults( image, results);
    waitKey(0);

    return 0;
}

