#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>

using namespace cv;
using namespace std;

string source_window = "Source";
Mat src, src_gray, original;
int thresh_har = 150, thresh_suz = 200;
int thresh_knn = 90, window = 50;
int max_thresh = 255;
RNG rng(12345);

void suzuki_callback(int, void*);
void cornerHarris(int, void*);
void calcKnnDistance(int, void*);

int main(int, char** argv)
{
    src = imread(argv[1]);
    original = imread(argv[2]);
    if (src.empty()) {
        cerr << "No image supplied ..." << endl;
        return -1;
    }
    cvtColor(src, src_gray, COLOR_BGR2GRAY);

    namedWindow(source_window, WINDOW_AUTOSIZE);
    createTrackbar("Suzuki threshold:", source_window, &thresh_suz, max_thresh, suzuki_callback);
    suzuki_callback(0, 0);
    
    createTrackbar("Harris threshold: ", source_window, &thresh_har, max_thresh, cornerHarris);
    cornerHarris(0, 0);

    createTrackbar(" Knn threshold:", source_window, &thresh_knn, max_thresh, calcKnnDistance);
    createTrackbar(" Knn window:", source_window, &window, 200, calcKnnDistance);
    calcKnnDistance(0, 0);
    //imshow(source_window, src);

    waitKey(0);
    return (0);
}

void suzuki_callback(int, void*)
{
    Mat canny_output, blurred;
    blur(src_gray, blurred, Size(3, 3));
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    Canny(blurred, canny_output, thresh_suz, thresh_suz * 2, 3);
    findContours(canny_output, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
    Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);
    
    for (size_t i = 0; i < contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, contours, (int)i, color, 2, 8, hierarchy, 0, Point());
    }

    stringstream str;
    str << "Count: " << contours.size();
    putText(drawing, str.str(), Point(10, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    imwrite("contours.png", drawing);

    namedWindow("Contours", WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
}

void cornerHarris(int, void*)
{
    int count = 0;
    Mat dst, dst_norm, dst_norm_scaled;
    dst = Mat::zeros(src.size(), CV_32FC1);
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    
    cornerHarris(src_gray, dst, blockSize, apertureSize, k, BORDER_DEFAULT);
    normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
    convertScaleAbs(dst_norm, dst_norm_scaled);
    
    for (int j = 0; j < dst_norm.rows; j++) {
        for (int i = 0; i < dst_norm.cols; i++) {
            if ((int)dst_norm.at<float>(j, i) > thresh_har) {
                circle(dst_norm_scaled, Point(i, j), 5, Scalar(0), 2, 8, 0);
                count++;
            }
        }
    }

    stringstream str;
    str << "Count: " << count;
    putText(dst_norm_scaled, str.str(), Point(10, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(255, 255, 255));
    imwrite("corners.png", dst_norm_scaled);

    namedWindow("Corners", WINDOW_AUTOSIZE);
    imshow("Corners", dst_norm_scaled);
}

void calcKnnDistance(int, void*)
{
    Mat estimated = src_gray.clone();
    Mat src_clone = src.clone();
    Mat original_grey;
    cvtColor(original, original_grey, COLOR_BGR2GRAY);
    double sum_distance = 0;
    
    //for every cell in estimated
    for (int j = 0; j < estimated.rows; j++) {
        for (int i = 0; i < estimated.cols; i++) {

            //for every cell in original in window
            if (estimated.at<uchar>(j, i) < thresh_knn) {
                
                double min_distance = sqrt(2 * window * window);
                src_clone.at<Vec3b>(j,i)[1] = 255; //visualisation
                for (int l = (j - window) < 0 ? 0 : (j - window); l < original_grey.rows && l < j + window; l++) {

                    for (int m = (i - window) < 0 ? 0 : (i - window); m < original_grey.cols && m < i + window; m++) {

                        if (original_grey.at<uchar>(l, m) < thresh_knn) {
                            double distance = sqrt((j - l) * (j - l) + (i - m) * (i - m));

                            if (distance < min_distance)
                                min_distance = distance;
                        }
                    }
                }
                sum_distance += min_distance;
            }
        }
    }
    
    
    //calc occuped cells in original
    int occuped = 0;
    for (int l = 0; l < original_grey.rows; l++) {
        for (int m = 0; m < original_grey.cols; m++) {
            if (original_grey.at<uchar>(l, m) < thresh_knn) {
                occuped++;
                src_clone.at<Vec3b>(l,m)[2] = 255; //visualisation
            }
        }
    }

    double result = sum_distance / occuped;

    stringstream str;
    str << "KNN distance: " << result;
    putText(src_clone, str.str(), Point(10, 30), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0));
    imshow(source_window, src_clone);
    imwrite("knn_distance.png", src_clone);
}
