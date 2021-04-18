#include <time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/core.hpp>
#include <math.h>
#include <iostream>
#include <cmath>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

/****************************
* 
* 记得给串口赋予权限 chmod 777 /dev/ttyUSB0
* 
* **************************/

//#define NX
#define RED
//#define BLUE
//#define IMSHOW //提高效率可以把这两个注释掉
#define CLOCK
//#define PREDICT

#ifdef NX
#include "serial.h"
#endif

using namespace std;
using namespace cv;

int main()
{
#ifdef NX
    Serial uart(BR115200, WORDLENGTH_8B, STOPBITS_1, PARITY_NONE, HWCONTROL_NONE);
    uart.sOpen("/dev/ttyUSB0");
    VideoCapture capture(0);
#endif
    clock_t start, finish;
    double totaltime, heights[32],sum=0;
    int hi = 0;
    int times = 0;

#ifndef NX
    VideoCapture capture(1);
#endif

    Mat frame, binary, frame1;
    RotatedRect Select_contours[32];
    RotatedRect RA[32], R[32];
    int stateNum = 4;
    int measureNum = 2;
    KalmanFilter KF(stateNum, measureNum, 0);
    //Mat processNoise(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    setIdentity(KF.measurementMatrix);
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));
    setIdentity(KF.errorCovPost, Scalar::all(1));
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));

    while (1)
    {
        hi = 0;//再次确保不会爆数组
#ifdef CLOCK
        start = clock();
#endif
        capture >> frame;
        frame.copyTo(binary);//展示效果
        frame.copyTo(frame1);
        //cvtColor(frame1, frame1, COLOR_BGR2HSV);//用作后面的红蓝判断
        if (!capture.isOpened()) break;

        /*
        //HSV方案
        Mat temp, temp1,temp2, thres_whole;
        vector<cv::Mat> channels;
        cvtColor(frame, temp, COLOR_BGR2HSV);
        split(temp, channels);
        inRange(temp, Scalar(0,43,46),Scalar(11,255,255),temp1);
        inRange(temp,Scalar(156,43,46),Scalar(181,255,255),temp2);
        Mat red;
        red = temp2 | temp1;
        cvtColor(frame,thres_whole,CV_BGR2GRAY);
        threshold(thres_whole,thres_whole,105,255,THRESH_BINARY);

        frame = red & thres_whole;
        imshow("1", frame);
        */

        /*
        //RGB方案
        //blue
        Mat thres_whole;
        vector<Mat> splited;
        split(frame, splited);
        cvtColor(frame, thres_whole, CV_BGR2GRAY);
        threshold(thres_whole, thres_whole, 128, 255, THRESH_BINARY);

        subtract(splited[0], splited[2], frame);
        cvtColor(frame, frame, COLOR_BGR2GRAY);
        threshold(frame, frame, 90, 255, THRESH_BINARY);// blue

        frame = frame & thres_whole;
        imshow("1", frame);
        */

        //亮度方案
        float Image_bright = -102;
        Mat BrightnessLut(1, 256, CV_8UC1);
        for (int i = 0; i < 256; i++)
        {//you can change "Image_bright" to adjust bright level
            BrightnessLut.at<uchar>(i) = saturate_cast<uchar>(i + Image_bright);
        }
        LUT(frame, BrightnessLut, frame);
        cvtColor(frame, frame, COLOR_BGR2GRAY);
        int threshold_value = 105;
        threshold(frame, frame, threshold_value, 255, cv::THRESH_BINARY);

        Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(3, 3), Point(-1, -1));
        morphologyEx(frame, frame, MORPH_OPEN, kernel);

#ifdef IMSHOW
        imshow("double", frame);
#endif

        vector<vector<Point>> contours;
        findContours(frame, contours, RETR_LIST, CHAIN_APPROX_NONE);
        vector<vector<Point>> select_contours;

        //select the right contours
        for (size_t i = 0; i < contours.size(); i++)
        {
            float light_contour_area = contourArea(contours[i]);

            if (light_contour_area < 20 || contours[i].size() <= 5)
            {
                continue;
            }

            drawContours(frame, contours, static_cast<int>(i), Scalar(0), 2);

            //RotatedRect rec = fitEllipse(contours[i]); //椭圆拟合
            RotatedRect rec = minAreaRect(contours[i]); //最小外接矩阵拟合
            //采用最小外接矩阵拟合比椭圆拟合效果要好，因为有的装甲板两条灯条亮度不一样，用椭圆拟合出来的矩形亮的比暗的大很多

            /*
            //因为摄像头的原因，无法在图片预处理就筛出红蓝色，因此只能找到区域后再判断区域中心点的颜色
            //这种方法如果摄像头晃动就会报错
            int b = frame1.at<Vec3b>((int)rec.center.x, (int)rec.center.y)[0];
            int g = frame1.at<Vec3b>((int)rec.center.x, (int)rec.center.y)[1];
            int r = frame1.at<Vec3b>((int)rec.center.x, (int)rec.center.y)[2];
#ifdef RED
            if (((b<0||g<43||r<46)||(b>11||g>255||g>255))||(b < 156 || g < 43 || r < 46) || (b > 181 || g > 255 || g > 255))
            {
                continue;
            }
#endif
#ifdef BLUE
            if ((b < 100 || g < 43 || r < 46) || (b > 124 || g > 255 || g > 255))
            {
                continue;
            }
#endif
*/
            /*
            cv::Point2f* vertices = new cv::Point2f[4];
            rec.points(vertices);
            for (int j = 0; j < 4; j++)
            {
                cv::line(binary, vertices[j], vertices[(j + 1) % 4], cv::Scalar(0, 255, 0), 4);

            }
            continue;
            */

            float angle = abs(rec.angle);
            if (angle > 45)
            {
                angle = 90 - angle;
            }
            if (angle > 30)
            {
                continue;
            }

            const float width_height_ratio_max = 10;
            const float width_height_ratio_min = 2;
            if (abs(rec.angle) < 45)
            {
                if ((rec.size.height / rec.size.width) > width_height_ratio_max)
                {
                    continue;
                }
            }
            else
            {
                if ((rec.size.width / rec.size.height) > width_height_ratio_max)
                {
                    continue;
                }
            }
            select_contours.push_back(contours[i]);
        }

        for (int i = 0; i < select_contours.size(); i++)
        {

            drawContours(frame, select_contours, static_cast<int>(i), Scalar(0), 2);

            if (i == select_contours.size() - 1) continue;
            RotatedRect rect = minAreaRect(select_contours[i]);

            for (int j = i + 1; j < select_contours.size(); j++)
            {
                RotatedRect rectA = minAreaRect(select_contours[j]);

                float r_angle, rA_angle, angle_diff;
                float r_height, r_width, rA_height, rA_width;
                float r_ratio, rA_ratio;
                if (rect.angle < -45)
                {
                    r_angle = 90 + rect.angle;
                    r_width = rect.size.height;
                    r_height = rect.size.width;
                }
                else 
                {
                    r_angle = rect.angle;
                    r_height = rect.size.height;
                    r_width = rect.size.width;
                }
                if (rectA.angle < -45)
                {
                    rA_angle = 90 + rectA.angle;
                    rA_width = rectA.size.height;
                    rA_height = rectA.size.width;
                }
                else
                {
                    rA_angle = rectA.angle;
                    rA_height = rectA.size.height;
                    rA_width = rectA.size.width;
                }
                r_ratio = r_height / r_width;
                rA_ratio = rA_height / rA_width;
                if (r_ratio > 10 || r_ratio < 1 || rA_ratio> 10 || rA_ratio < 1)
                {
                    //cout << r_ratio << "   " << rA_ratio << endl;;
                    continue;
                }

                angle_diff = abs(r_angle - rA_angle);
                if (angle_diff > 10)
                {
                    continue;
                }

                float d = sqrt((rect.center.x - rectA.center.x) * (rect.center.x - rectA.center.x) +
                    (rect.center.y - rectA.center.y) * (rect.center.y - rectA.center.y));
                if (abs(rect.center.x - rectA.center.x) < 0.9 * d)
                {
                    continue;
                }

                const float twobar_height_ratio_max = 3;
                float twobar_height_ratio = max(max(rect.size.width, rect.size.height), max(rectA.size.width, rectA.size.height)) / min(max(rect.size.width, rect.size.height), max(rectA.size.width, rectA.size.height));
                float twobar_width_ratio = max(min(rect.size.width, rect.size.height), min(rectA.size.width, rectA.size.height)) / min(min(rect.size.width, rect.size.height), min(rectA.size.width, rectA.size.height));;

                if (twobar_width_ratio > twobar_height_ratio_max || twobar_height_ratio > twobar_height_ratio_max)
                {
                    continue;
                }
                if (rect.size.area() / rectA.size.area() > 3)
                {
                    continue;
                }

                float board_width = abs(rect.center.x - rectA.center.x);
                float board_height = (max(rect.size.width, rect.size.height) + max(rectA.size.width, rectA.size.height)) / 2;
                float board_ratio = board_width / board_height;
                const float board_ratio_max = 7;
                if (board_ratio > board_ratio_max)
                {
                    continue;
                }

                float board_width_difference = abs(rect.center.x - rectA.center.x);
                float board_height_difference = abs(rect.center.y - rectA.center.y);

                const float board_height_difference_max = 200;
                const float board_width_difference_min = 30;
                if (board_width_difference < board_width_difference_min || board_height_difference>board_height_difference_max)
                {
                    continue;
                }

#ifdef IMSHOW
                cv::Point2f* vertices1 = new cv::Point2f[4];
                rect.points(vertices1);
                for (int j = 0; j < 4; j++)
                {
                    cv::line(binary, vertices1[j], vertices1[(j + 1) % 4], cv::Scalar(0, 255, 0), 4);

                }
                cv::Point2f* vertices2 = new cv::Point2f[4];
                rectA.points(vertices2);
                for (int j = 0; j < 4; j++)
                {
                    cv::line(binary, vertices2[j], vertices2[(j + 1) % 4], cv::Scalar(0, 255, 0), 4);

                }
#endif
                heights[hi] = max(rect.size.height,rect.size.width)+ max(rectA.size.height,rectA.size.width) / 2;
                R[hi] = rect;
                RA[hi] = rectA;
                hi++;//统计找到的装甲板个数
            }
        }

        //选择最近的装甲板进行击打
        double maxh = 0;
        int mark;
        for (int i = 0; i < hi; i++)
        {
            if (heights[i] >= maxh)
            {
                maxh = heights[i];
                mark = i;
            }
        }

        if (hi != 0)
        {
            cv::circle(binary, Point((R[mark].center.x + RA[mark].center.x) / 2,
                (R[mark].center.y + RA[mark].center.y) / 2),
                15, cv::Scalar(0, 0, 255), 4);

            double center_x = (R[mark].center.x + RA[mark].center.x) / 2;
            double center_y = (R[mark].center.y + RA[mark].center.y) / 2;

            Point2f verticesR[4];
            R[mark].points(verticesR);
            Point2f verticesRA[4];
            RA[mark].points(verticesRA);
            float x1, y1, x2, y2, x3, y3, x4, y4;

            if (abs(R[mark].angle) > 45)
            {
                x1 = (verticesR[2].x + verticesR[3].x) / 2;
                y1 = (verticesR[2].y + verticesR[3].y) / 2;
                x4 = (verticesR[1].x + verticesR[0].x) / 2;
                y4 = (verticesR[1].y + verticesR[0].y) / 2;
            }
            else
            {
                x1 = (verticesR[1].x + verticesR[2].x) / 2;
                y1 = (verticesR[1].y + verticesR[2].y) / 2;
                x4 = (verticesR[0].x + verticesR[3].x) / 2;
                y4 = (verticesR[0].y + verticesR[3].y) / 2;
            }

            if (abs(RA[mark].angle) > 45)
            {
                x2 = (verticesRA[2].x + verticesRA[3].x) / 2;
                y2 = (verticesRA[2].y + verticesRA[3].y) / 2;
                x3 = (verticesRA[1].x + verticesRA[0].x) / 2;
                y3 = (verticesRA[1].y + verticesRA[0].y) / 2;
            }
            else
            {
                x2 = (verticesRA[1].x + verticesRA[2].x) / 2;
                y2 = (verticesRA[1].y + verticesRA[2].y) / 2;
                x3 = (verticesRA[0].x + verticesRA[3].x) / 2;
                y3 = (verticesRA[0].y + verticesRA[3].y) / 2;
            }

#ifdef PREDICT
            Mat prediction = KF.predict();
            Point predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));

            measurement.at<float>(0) = (float)center_x;
            measurement.at<float>(1) = (float)center_y;
            KF.correct(measurement);

            circle(binary, predict_pt, 3, Scalar(34, 255, 255), -1);
#endif
            //卡尔曼的预测结果
            //center_x = (int)prediction.at<float>(0);
            //center_y = (int)prediction.at<float>(1);

#ifdef IMSHOW  
            vector<cv::Point2f> imagePoints;             
            imagePoints.push_back(Point2f(x1, y1));
            imagePoints.push_back(Point2f(x2, y2));
            imagePoints.push_back(Point2f(x3, y3));
            imagePoints.push_back(Point2f(x4, y4));

            for (int n = 0; n < imagePoints.size(); n++)
            {
                circle(binary, imagePoints[n], 3, Scalar(255, 0, 0), -1, 8);
            }
#endif
            float boardw_up = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
            float boardw_down = sqrt((x3 - x4) * (x3 - x4) + (y3 - y4) * (y3 - y4));
            float boardw = (boardw_up + boardw_down) / 2;
            float boardh_left = sqrt((x1 - x4) * (x1 - x4) + (y1 - y4) * (y1 - y4));
            float boardh_right = sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
            float boardh = (boardh_left + boardh_right) / 2;

            float board_width = abs(R[mark].center.x - RA[mark].center.x);
            float board_height = (max(R[mark].size.width, R[mark].size.height) + max(RA[mark].size.width, RA[mark].size.height)) / 2;
            float board_ratio = board_width / board_height;
            float xishu;
            float final_distance;
            if (board_ratio < 4)//判断是大装甲板还是小装甲板
            {
                //小装甲板
                xishu = (13.5 / boardw + 5.4 / boardh) / 2;
                final_distance = 10700 / boardw;
            }
            else
            {
                //大装甲板
                xishu = (23.5 / boardw + 5.4 / boardh) / 2;
                final_distance = 18626 / boardw;
            }
            //cout << 10700 / boardw << endl;
            //cout << xishu << endl;
            double distance_to_midboard_x, distance_to_midboard_y;
            distance_to_midboard_x = xishu * (center_x - 320);
            distance_to_midboard_y = xishu * (center_y - 160);

            double angle_x = atan2(distance_to_midboard_x, final_distance);
            double angle_y = atan2(distance_to_midboard_y, final_distance);

            const double P = 3.1415926;
            double final_angle_x = angle_x / P * 180;
            double final_angle_y = angle_y / P * 180;
#ifdef IMSHOW
            cout << "angle_x" << final_angle_x << endl;
            cout << "angle_y" << final_angle_y << endl;
#endif
#ifdef NX
            uart.sSendData(final_angle_x, final_angle_y);
#endif
        }
#ifdef IMSHOW
        imshow("okey", binary);
#endif
        waitKey(1);
#ifdef CLOCK
        finish = clock();
        totaltime = (double)(finish - start) / CLOCKS_PER_SEC;
        sum += totaltime;
        times+=1;
        if (sum > 1)
        {
            cout <<"times"<< times << endl;
            times = 0;
            sum = 0;
        }
        std::cout << "Time" << totaltime << endl;
#endif
        hi = 0;//这个很重要！！！ 让RotatedRect数组重新从0开始存储 不然会爆数组
    }
}
