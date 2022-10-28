#include <iostream>
#include "Strategy.h"
#include "Image_Process.h"

int focus = 50;

int main(int argc, char** argv)
{
	//定义焦距（高斯核大小） 
	/*std::vector<int> focus =
	{
		12,
		10,
		9,
		9,
		7,
		8,
		7,
		7,
		6,
		5,
		4,
		3,
		2,
		1,
		0,
		1,
		2,
		3,
		4,
		5,
		6,
		5,
		3,
		2,
		3,
		4,
		6,
		8,
		9,
		10
	};
	
	cv::Mat temp = cv::imread("img.bmp");
	//使用高斯模糊创建不同焦距下的图片
	for(int i = 0;i < 30;i++)
	{
		char result_name[30];
		sprintf_s(result_name, "%d.bmp", i);
		cv::Mat result = Gaussian_filter(temp, focus[i], focus[i], 1, 30.0);
		cv::imwrite(result_name, result);
	}*/

	//1.0 使用图片集合进行对焦（离散）

	//图片文件名称
	/*std::vector<std::string> img_name =
	{
		"0.bmp",
		"1.bmp",
		"2.bmp",
		"3.bmp",
		"4.bmp",
		"5.bmp",
		"6.bmp",
		"7.bmp",
		"8.bmp",
		"9.bmp",
		"10.bmp",
		"11.bmp",
		"12.bmp",
		"13.bmp",
		"14.bmp",
		"15.bmp",
		"16.bmp",
		"17.bmp",
		"18.bmp",
		"19.bmp",
		"20.bmp",
		"21.bmp",
		"22.bmp",
		"23.bmp",
		"24.bmp",
		"25.bmp",
		"26.bmp",
		"27.bmp",
		"28.bmp",
		"29.bmp",
	};*/
	//创建class，存储图片集合和焦距集合，并包含若干成员函数
	//Image_Process img(img_name, "focus.txt");

	//测试插值图片和反差计算是否正确
	/*for(size_t i = 0;i < img.focus.size();i++)
	{
		for(int j = 0;j < 10;j++)
		{
			std::cout << img(i + 1.0 + 0.1 * j) << std::endl;
		}
	}*/

	//下降单纯形法求解
	/*float start[4] = { 18.9, 18.0, 18.1, 18.3 };
	float focus = DownHill<4>(start, img);*/
	//爬山算法求解
	//float start = 19.9;
	//float focus = ClimbHill(start, img);

	//打印结果
	//std::cout << focus << std::endl;
	//cv::imshow("Difference", img.FocusToImg(focus));

	//2.0 使用图片和深度图进行对焦（连续）

	//由原图和深度图创建虚拟相机对象，包含若干成员函数
	//Camera camera("color1.png", "depth1.png");

	//下降单纯形法求解
	//float start[4] = { 49.0, 50.0, 48.0, 47.0 };
	//float focus = DownHill<4>(start, camera);
	//爬山算法求解
	//float start = 80.0;
	//float focus = ClimbHill(start, camera);

	//打印结果
	//std::cout << focus << std::endl;
	//cv::imshow("Difference", camera.FocusCamera(focus));

	//for(int i = 1;i <= 100;i++)
	//{
		//std::cout << camera(i * 1.0) << std::endl;
	//}

	cv::Mat img = cv::imread("P3.jpg");

	cv::imwrite("bumpmipmap1.jpg", MipMap(img, 1));
	cv::imwrite("bumpmipmap2.jpg", MipMap(img, 2));
	cv::imwrite("bumpmipmap3.jpg", MipMap(img, 3));
	cv::imwrite("bumpmipmap4.jpg", MipMap(img, 4));
	cv::imwrite("bumpmipmap5.jpg", MipMap(img, 5));
	cv::imwrite("bumpmipmap6.jpg", MipMap(img, 6));

	std::cout << "Press any key to quit" << '\n';
	cv::waitKey(1000000);
	cv::destroyAllWindows();

	return 0;
}
