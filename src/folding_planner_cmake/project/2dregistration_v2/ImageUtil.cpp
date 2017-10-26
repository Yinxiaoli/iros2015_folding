#include "ImageUtil.h"

using namespace std;

ImageUtil::ImageUtil()
{

}

SImage<double, 1> ImageUtil::ConvertToSImage(cv::Mat input)
{
	// Rotate image 90 degree clockwise to compromise SImage conversion.
	// It looks like that convert to SImage will rotate image 90 degree counter-clockwise.
	rotate_image_90n(input, input, 90);

	SImage<double, 1> simage;
	initImage(input.cols, input.rows, simage);

	for(int i = 0; i < input.cols; i++)
		for (int j = 0; j < input.rows; j++)
		{
			cv::Scalar intensity = input.at<uchar>(i, j);
			double intensityd = (double)intensity[0];
			simage.ptr[j*input.cols + i] = intensityd;
		}
		return simage;
}


SImage<double, 1> ImageUtil::ConvertToSImage(char* file_name)
{
	cv::Mat img_load = cv::imread(file_name, CV_LOAD_IMAGE_GRAYSCALE);
	return ConvertToSImage(img_load);
}


void ImageUtil::rotate_image_90n(cv::Mat &src, cv::Mat &dst, int angle)
{   
	if(src.data != dst.data)
	{
		src.copyTo(dst);
	}

	angle = ((angle / 90) % 4) * 90;

	//0 : flip vertical; 1 flip horizontal
	bool const flip_horizontal_or_vertical = angle > 0 ? 1 : 0;
	int const number = std::abs(angle / 90);          

	for(int i = 0; i != number; ++i)
	{
		cv::transpose(dst, dst);
		cv::flip(dst, dst, flip_horizontal_or_vertical);
	}
}


void ImageUtil::ImageShow(char* file_name)
{
		cv::Mat img = cv::imread(file_name);
		if (img.empty())
		{
			cout << "error";
			return;
		}
		cv::imshow("my picture", img);
		cv::waitKey();

		return;
}

void ImageUtil::ImageShow(cv::Mat img)
{
	if (img.empty())
	{
		cout << "error";
		return;
	}
	cv::imshow("my picture", img);
	cv::waitKey();

	return;
}

void ImageUtil::ImageShow(cv::Mat img, int sec)
{
	if (img.empty())
	{
		cout << "error";
		return;
	}
	cv::imshow("my picture", img);
	cv::waitKey(sec);

	return;
}

