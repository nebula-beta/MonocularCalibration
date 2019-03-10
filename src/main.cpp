#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>

#include "MonocularCalibration.h"


using namespace std;
using namespace cv;




void getImages(string configutationFile, vector<Mat>& images)
{
    ifstream fin(configutationFile);
    string filename;
    while(getline(fin, filename))
    {
	    Mat img;
	    img = imread(filename);
	    images.push_back(img);
    }
}

void getImages(const vector<cv::String>& images_url, vector<cv::Mat>& images)
{
    for(size_t i = 0; i < images_url.size(); ++i)
    {
		Mat img = imread(images_url[i]);
		images.push_back(img);
    }
    
}
int main(int argc, char* argv[])
{
	if(argc != 5)
	{
		printf("usage : ./calib [chessboard_width] [chessboard_height] [imgs_directory] [file_extension]\n");
		exit(-1);
	}

	int board_width = atoi(argv[1]);
	int board_height = atoi(argv[2]);
	string imgs_directory = argv[3];
	string file_extension = argv[4];
    
    vector<Mat> images;
    vector<cv::String> images_url;

	string tmp  = imgs_directory[imgs_directory.size() - 1] == '/' ? "*." : "/*.";
	string path = imgs_directory + tmp + file_extension;

	cv::glob(path, images_url);

   
    getImages(images_url, images);

    MonocularCalibration calib(images, Size(board_width, board_height));
    calib.onLog();
    calib.onShowImage();
    calib.startCalibrate();

    return 0;
}
