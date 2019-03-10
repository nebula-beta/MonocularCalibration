#ifndef MONOCULAR_CALIBRATION_H
#define MONOCULAR_CALIBRATION_H


#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


/*
 * 找到标定板上的角点，专门为标定写的函数
 * bool findChessboardCorners( InputArray image, Size patternSize, OutputArray corners, int flags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE )
 * image : 输入的棋盘图片，　灰色图或者彩色图
 * patternsize : 棋盘内部角点的行列数(是角点的行列数，而不是棋盘的行列数)
 * corners : 输出的棋盘角点
 */


/*
 * 用来作为迭代算法的终止条件
 * TermCriteria(int _type, int _maxCount, double _epsilon)
 * _type : 迭代的类型　Termcriteria::COUNT(或者Termcriteria::MAX_ITER), Termcriteria::EPS, Termcriteria::MAX_ITER | Termcriteria::EPS 
 *		　分别表示终止条件达到最大次数、迭代到满足阈值终止，　两者都作为迭代条件
 *
 *	_maxCount : 最大迭代次数
 *	_epsilon : 迭代精度
 */

/*
 * 角点精化，　亚像素级角点
 * cornerSubPix( Inputarray image, InputOutputArray corners, Size winSize, Size zeroZone, Termcriteria criteria )
 * image : 输入图像
 * corners : 输入角点的初始坐标，以及精华后的坐标用于输入，　所以类型是Inputoutputarray
 * winSize : 搜索窗口边长的一半，　如果winSize(5, 5) 则一个大小为(5 * 2 + 1) * (5 * 2 + 1) = 11 * 11的搜索窗口将被使用
 * zeroZone : 
 * criteria : 角点精华迭代过程中的终止条件。
 */

/*
 * void drawchessboardcorners( InputOutputArray image, Size patternSize, Inputarray corners, bool patternWasFound)
 *	image : 灰度图或者彩色图
 *	patternSize : 标定棋盘内角点的行列数
 *	corners : 角点坐标
 *	patternWasFound : 角点是否全部找到的标志，　为findChessboardCorners的返回值？
 */


/*
 * double calibrateCamera( InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, Size imageSize, 
 *						   CV_OUT InputOutputArray cameraMatrix, CV_OUT InputOutputArray distCoeffs, 
 *						   OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs, int flags, 
 *						   TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON) );
 * 
 * objectPoints : 世界坐标系中的三维点vector<vector<Point3f> > objectPoints, 根据棋盘上单个黑白矩阵的大小，　计算出（初始化）每一个内角点的世界坐标
 * imagePoints : 每一个内角点所对应的像素坐标，　vector<vector<Point2f>> imagePoints
 * imageSize :　为图像的像素尺寸大小，　在计算相机内参和畸变矩阵时需要用到该参数
 * cameraMatrix : 相机的内参矩阵, Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0))
 * distCoeffs : 畸变矩阵, Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0));
 * rvecs : 旋转向量 vector<Mat>rvecs 每张图片对应的旋转矩阵
 * tvecs : 位移向量 vector<Mat>tvecs;
 * flags : 所采用的算法
 *		CV_CALIB_USE_INTRINSIC_GUESS : 使用该参数时，　cameraMatrix中应该有fx, fy, u0, v0的估计值
 *		CV_CALIB_FIX_PRINCIPAL_POINT : 
 *		CV_CALIB_FIX_ASPECT_RATION
 *		CV_CALIB_ZERO_TANGENT_DIST
 *		CV_CALIB_RATIONAL_MODE : 
 *	criteria : 迭代终止条件
 */




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
