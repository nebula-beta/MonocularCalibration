#ifndef MONOCULAR_CALIBRATION_H
#define MONOCULAR_CALIBRATION_H


#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;



/**
 * @brief 对相机进行标定，并评价结果
 * 
 **/
class MonocularCalibration
{
public:
    /*
	* imgs : 标定板图片
	* patternSize : 标定板内点的列行数 Size(4, 6)表示有6行4列
	* squareSize: 标定板每个格子的大小, 用来构造世界坐标
	* winSize : 角点亚像素精化时的搜索区域
	*/
    
      /**
     * @brief ...
     * 
     * @param imgs			: 所拍摄的标定板图片
     * @param patternSize	: 棋盘标定板的每行的角点数和每列的角点数（如果参数的数目和实际的数目不一致，那么会导致标定失败）
     * @param squareSize	: 棋盘标定板上的每个方块有多大，以第一个方块为原点，根据方块的大小，就能够计算出其它方块的世界坐标
	 *						 （对于单目相机标定，由于相差一个尺度，所以该参数可以任意，只需要保证两个维度大小一致即可）
     * @param winSize		: 在多大的范围内进行角点亚像素精华
     */
    
    MonocularCalibration(vector<Mat> &imgs, Size patternSize, Size squareSize = Size(10, 10), Size winSize = Size(4, 4));
    /* MonocularCalibration(vector<Mat> &img, Size patternSize, Size squareSize, Size winSize, Size zeroZone, */ 
			    /* TermCriteria criteria = TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1)); */

    void getReProjError();
    bool startCalibrate();
	// 是否显示算法流程中的输出信息
    void onLog();
    void offLog();

	// 是否将角点画到图像上，然后显示出来
    void onShowImage();
    void offShowImage();

private:
    bool getIthChessboardCornerSubPix(int ith, InputOutputArray corners);	
    void getChessboardWorldPoints();
    void getChessboardCornerPoints();
	
private:
    vector<Mat> imgs;	
    Size patternSize;
    Size squareSize;
    Size winSize;
    
    bool isLog;
    bool isShowImage;

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> cornerPoints;
    Mat cameraMatrix;
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    int n;	// 所拍摄的标定板图像数
    int m;	// 寻找角点失败的图像数

    double reProjError;
};

#endif //MONOCULAR_CALIBRATION_H
