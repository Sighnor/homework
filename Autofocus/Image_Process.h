#pragma once

#include <fstream>
#include <string>
#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\objdetect\objdetect.hpp>
#include <opencv2\imgproc\types_c.h>

float Core(int s, int t, float k, float theta2);

cv::Mat Gaussian_filter(cv::Mat &img1, int m, int n, float k, float theta2);

cv::Mat Clamp_Img(cv::Mat &img1, cv::Mat &img2, const float &k);//对图像进行插值

cv::Mat MipMap(cv::Mat &img1, int level);

float Difference(cv::Mat &img1, float l, float r, float t, float b);//计算反差值

class Image_Process
{
  public:
    Image_Process(std::vector<std::string> img_name, std::string focus_txt)//构造函数，传入图像文件的名称和焦距文件的名称
    {
      for(size_t i = 0;i < img_name.size();i++)
      {
          img.push_back(cv::imread(img_name[i]));
      }

      for(size_t i = 0;i < img.size();i++)
      {
          cv::cvtColor(img[i], img[i], cv::COLOR_BGR2GRAY);//转化为灰度图
      }

      float temp_focus;
      std::ifstream file;
      file.open(focus_txt);

      while (file >> temp_focus)
      {
        focus.push_back(temp_focus);//读取焦距
      }

	    file.close();

      img.size() == focus.size()? std::cout << "Match!" << std::endl : std::cout << "Not Match!" << std::endl;
    }
    //可通过map优化？不过对性能影响不大，因为性能主要消耗在图像处理上
    float Clamp_Focus(const float &my_focus, size_t &t1, size_t &t2);//插值寻找最合适焦距

    cv::Mat FocusToImg(const float &my_focus);//由焦距得到对应图像

    float operator()(const float &my_focus)//重载运算符，输入焦距得到反差值
    {
      cv::Mat img = FocusToImg(my_focus);
      return Difference(img, left_Range, right_Range, top_Range, below_Range);
    }

    std::vector<cv::Mat> img;
    std::vector<float> focus;
    //对焦区域范围
    float left_Range = 0.30;
    float right_Range = 0.55;
    float top_Range = 0.10;
    float below_Range = 0.35; 
};

inline float Image_Process::Clamp_Focus(const float &my_focus, size_t &t1, size_t &t2)//得到插值的焦距下标和比例
{
  float k;
  float floor = -1000.1;
  float ceil = 1000.1;
  for(size_t i = 0;i < focus.size();i++)
  {
    if(focus[i] <= my_focus && focus[i] > floor)//向下取
    {
      t1 = i;
      floor = focus[i];
    }
    else if(focus[i] > my_focus && focus[i] < ceil)//向上取
    {
      t2 = i;
      ceil = focus[i];
    }
  }

  if(ceil > 1000.0f)//没有找到更大的焦距
  {
    t2 = t1;
    k = 1.0;
  }
  else if(floor < -1000.0f)//没有找到更小的焦距
  {
    t1 = t2;
    k = 0.0;
  }
  else
  {
    k = (ceil - my_focus) / (ceil - floor);//插值因子
  }

  return k;
}

inline cv::Mat Image_Process::FocusToImg(const float &my_focus)//由焦距得到图像
{
  size_t t1 = 0;
  size_t t2 = 0;
  float k = Clamp_Focus(my_focus, t1, t2);//得到插值因子和对应图像下标

  return Clamp_Img(img[t1], img[t2], k); 
}

class Camera
{
  public:
    Camera(std::string color_name, std::string depth_name)//构造函数，传入颜色图和深度图的名称
    {
      buffer_color = cv::imread(color_name);
      cv::Mat depth = cv::imread(depth_name);
      for(size_t i = 0;i < depth.rows;i++)
      {
        for(size_t j = 0;j < depth.cols;j++)
        {
          buffer_depth.push_back(depth.at<cv::Vec3b>(i, j)[0]);//存储深度值
        }
      }
      //std::cout << float(depth.at<cv::Vec3b>(10, depth.cols - 35)[0]) << std::endl;
      //cv::imshow("temp", buffer_color);
      //std::cout << depth.rows << ',' << depth.cols << std::endl;
    }

    cv::Mat FocusCamera(const float &my_focus);

    float operator()(const float &my_focus)//重载运算符，输入焦距得到反差值
    {
      cv::Mat img = FocusCamera(my_focus);
      //cv::imshow("temp", img);
      cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);//转化为灰度图
      return Difference(img, left_Range, right_Range, top_Range, below_Range);
    }

    cv::Mat buffer_color;
    std::vector<float> buffer_depth;
    //对焦区域范围
    float left_Range = 0.6;
    float right_Range = 1.0;
    float top_Range = 0.0;
    float below_Range = 0.33; 
};

inline cv::Mat Camera::FocusCamera(const float &my_focus)
{
  int m = 15;
  int k = 1;
  int theta2 = 100;
  int width = buffer_color.rows;
  int height = buffer_color.cols;
  cv::Mat result = buffer_color.clone();
  std::vector<float> my_core((m + 1) * (m + 1));
  std::fill(my_core.begin(), my_core.end(), float(0));

  for (int a = 0; a <= m; a++)
  {
      for (int b = 0; b <= m; b++)
      {
          my_core[a * (m + 1) + b] = Core(a, b, k, theta2);//预计算高斯核
      }
  }

#pragma omp parallel for
  for (int x = 0; x < width; x++)
  {
      for (int y = 0; y < height; y++)
      {
        int my_depth = buffer_depth[x * height + y];//深度值
        int diff = std::max(sqrt(abs(my_focus - my_depth)) - 1.5f, 0.01f);//深度与焦距差值
        int neg_xRadius = -std::min(x, diff);
        int neg_yRadius = -std::min(y, diff);
        int pos_xRadius = std::min(width - 1 - x, diff);
        int pos_yRadius = std::min(height - 1 - y, diff);
        float B = 0.0f;
        float G = 0.0f;
        float R = 0.0f;
        float sum_Core = 0.0f;
        for (int a = neg_xRadius; a <= pos_xRadius; a++)
        {
            for (int b = neg_yRadius; b <= pos_yRadius; b++)
            {
              int temp_depth = buffer_depth[(x + a) * height + y + b];//当前像素点深度与核中像素点深度差值
              size_t pos = abs(a) * (m + 1) + abs(b);
              float temp_core = my_core[pos] * exp(-0.1 * sqrt(abs(my_depth - temp_depth)));//深度差值作为一个核加入
              B += buffer_color.at<cv::Vec3b>(x + a, y + b)[0] * temp_core;
              G += buffer_color.at<cv::Vec3b>(x + a, y + b)[1] * temp_core;
              R += buffer_color.at<cv::Vec3b>(x + a, y + b)[2] * temp_core;
              sum_Core += temp_core;
            }
        }
        result.at<cv::Vec3b>(x, y)[0] = B / sum_Core;
        result.at<cv::Vec3b>(x, y)[1] = G / sum_Core;
        result.at<cv::Vec3b>(x, y)[2] = R / sum_Core;
      }
  }
  return result;
}