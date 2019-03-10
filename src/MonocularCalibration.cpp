#include "MonocularCalibration.h"

MonocularCalibration::MonocularCalibration(vector<Mat> &imgs, Size patternSize, Size squareSize, Size winSize)
{
    this->imgs = imgs;
    this->patternSize = patternSize;
    this->squareSize = squareSize;
    this->winSize = winSize;
    isLog = false;
    isShowImage = false;
    n = imgs.size();
    m = 0;
}

bool MonocularCalibration::getIthChessboardCornerSubPix(int ith, InputOutputArray corners)
{
    if(isLog)
    {
	    cout << "get " << ith << "th chessboardCornerSubPix...";
    }

	// 寻找标定板角点
    bool found = findChessboardCorners(imgs[ith], patternSize, corners);
    if(found)
    {
	    Mat grayImg = imgs[ith];
	    if(grayImg.type() != CV_8UC1)
		    cvtColor(imgs[ith], grayImg, COLOR_BGR2GRAY);

		// 角点进行亚像精化
	    find4QuadCornerSubpix(grayImg, corners, winSize);

	    if(isShowImage)
	    {
		    drawChessboardCorners(imgs[ith], patternSize, corners, found);
		    namedWindow("camera", CV_WINDOW_NORMAL);//CV_WINDOW_NORMAL就是0
		    imshow("camera", imgs[ith]);
            waitKey(10);
	    }
    }
    return found;
}

void MonocularCalibration::getChessboardWorldPoints()
{
    int count = n - m;
    vector<Point3f> tmpPoints;
    if(isLog)
    {
	    cout << endl << "start getChessboardWorldPoints...";
    }
    for(int i = 0; i < patternSize.height; ++i)
    {
	    for(int j = 0; j < patternSize.width; ++j)
	    {
		    tmpPoints.push_back( Point3f( j * squareSize.width, i * squareSize.height, 0 ) );
	    }
    }
    for(int k = 0; k < count; ++k)
    {
	    objectPoints.push_back(tmpPoints);
    }
    if(isLog)
    {
	    cout << " Successfully..." << endl;
    }
}
void MonocularCalibration::getChessboardCornerPoints()
{
    if(isLog)
    {
	    cout << "start getChessboardCornerPoints..." << endl << endl;
    }
    for(int i = 0; i < n; ++i)
    {
	    vector<Point2f> corners;
	    bool found = getIthChessboardCornerSubPix(i, corners);
	    if(!found)
	    {
		    m++;
			cout << " failed." << endl;
	    }
	    else
	    {
			cout << " successfully." << endl;
		    cornerPoints.push_back(corners);
	    }
    }
    if(isLog)
    {
	    cout << endl << "end getChessboardCornerPoints..." << endl;
		cout << "result : total " << imgs.size() << " images, " << n - m<< " succeed " << m << " fail " << endl;
    }
}
bool MonocularCalibration::startCalibrate()
{
    getChessboardCornerPoints();
	// 如果成功找到角点的图像不超过3张，那么无法进行标定，标定失败
    if(n - m < 3)
	    return false;

    getChessboardWorldPoints();
    calibrateCamera( objectPoints, cornerPoints, imgs[0].size(), cameraMatrix, distCoeffs, rvecs, tvecs );
    cout << endl;
    getReProjError();


    cout << endl;
	cout << "Intrinsic parameters : " << endl;
    cout << cameraMatrix << endl;
	cout << "Distortion parameter : " << endl;
    cout << distCoeffs << endl;

}


void MonocularCalibration::getReProjError()
{
    reProjError = 0;
    double err = 0;
    for(int i = 0; i < n - m; ++i)
    {
	    const vector<Point3f> &tmpObjPoints = objectPoints[i];
	    const vector<Point2f> &tmpCornerPoints = cornerPoints[i];
	    vector<Point2f> reProPoints;
	    projectPoints(tmpObjPoints, rvecs[i], tvecs[i], cameraMatrix, distCoeffs, reProPoints);

	    Mat cornerPointMat(tmpCornerPoints);
	    Mat reProPointMat(reProPoints);
	    /* Mat cornerPointMat(1, reProPoints.size(), CV_32FC2); */
	    /* Mat reProPointMat(1, reProPoints.size(), CV_32FC2); */
	    /* for(int j = 0; j < reProPoints.size(); ++j) */
	    /* { */
	    /* 	cornerPointMat.at<Vec2f>(0, j) = Vec2f(tmpCornerPoints[j].x, tmpCornerPoints[j].y); */
	    /* 	reProPointMat.at<Vec2f>(0, j) = Vec2f(reProPoints[j].x, reProPoints[j].y); */
	    /* } */
	    err = norm(cornerPointMat, reProPointMat, NORM_L2);
	    err /= (patternSize.width * patternSize.height); //即每个点的平均误差
	    if(isLog)
	    {
		    cout << i << "th images average error is " << err << endl;
	    }
	    reProjError += err;
    }
    reProjError /= (n - m);
    if(isLog)
    {
	    cout << "average re-projection error is " << reProjError << endl;
    }

}
void MonocularCalibration::onLog()
{
    isLog = true;
}
void MonocularCalibration::offLog()
{
    isLog = false;
}

void MonocularCalibration::onShowImage()
{
    isShowImage = true;
}
void MonocularCalibration::offShowImage()
{
    isShowImage = false;
}
