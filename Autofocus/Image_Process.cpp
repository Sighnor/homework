#include "Image_Process.h"
//高斯核
float Core(int s, int t, float k, float theta2)
{
    float distance2 = s * s + t * t;
    return k * exp(-distance2 / (2 * theta2));
}
//高斯滤波
cv::Mat Gaussian_filter(cv::Mat &img1, int m, int n, float k, float theta2)
{
    int width = img1.rows;
    int height = img1.cols;
    cv::Mat img2 = img1.clone();
    std::vector<float> my_core((m + 1) * (n + 1));
    std::fill(my_core.begin(), my_core.end(), float(0));

    for (int a = 0; a <= m; a++)
    {
        for (int b = 0; b <= n; b++)
        {
            my_core[a * (n + 1) + b] = Core(a, b, k, theta2);//预计算高斯核
        }
    }

#pragma omp parallel for
    for (int x = 0; x < width; x++)
    {
        for (int y = 0; y < height; y++)
        {
            //边界判断，防止溢出
            int neg_xRadius = -std::min(x, m);
            int neg_yRadius = -std::min(y, n);
            int pos_xRadius = std::min(width - 1 - x, m);
            int pos_yRadius = std::min(height - 1 - y, n);
            float B = 0.0f;
            float G = 0.0f;
            float R = 0.0f;
            float sum_Core = 0.0f;
            for (int a = neg_xRadius; a <= pos_xRadius; a++)
            {
                for (int b = neg_yRadius; b <= pos_yRadius; b++)
                {
                    size_t pos = abs(a) * (n + 1) + abs(b);
                    B += img1.at<cv::Vec3b>(x + a, y + b)[0] * my_core[pos];
                    G += img1.at<cv::Vec3b>(x + a, y + b)[1] * my_core[pos];
                    R += img1.at<cv::Vec3b>(x + a, y + b)[2] * my_core[pos];
                    sum_Core += my_core[pos];
                }
            }
            img2.at<cv::Vec3b>(x, y)[0] = B / sum_Core;
            img2.at<cv::Vec3b>(x, y)[1] = G / sum_Core;
            img2.at<cv::Vec3b>(x, y)[2] = R / sum_Core;
        }
    }
    return img2;
}

cv::Mat MipMap(cv::Mat &img1, int level)
{
    int size = pow(2, level);
    cv::Mat img2(img1.rows / size, img1.cols / size, CV_8UC3);
    #pragma omp parallel for
    for (int x = 0; x < img2.rows; x++)
    {
        for (int y = 0; y < img2.cols; y++)
        {
            float B = 0.0f;
            float G = 0.0f;
            float R = 0.0f;
            float sum_Core = 0.0f;
            for (int a = x * size; a < (x + 1) * size; a++)
            {
                for (int b = y * size; b < (y + 1) * size; b++)
                {
                    B += img1.at<cv::Vec3b>(a, b)[0];
                    G += img1.at<cv::Vec3b>(a, b)[1];
                    R += img1.at<cv::Vec3b>(a, b)[2];
                    sum_Core += 1;
                }
            }
            img2.at<cv::Vec3b>(x, y)[0] = B / sum_Core;
            img2.at<cv::Vec3b>(x, y)[1] = G / sum_Core;
            img2.at<cv::Vec3b>(x, y)[2] = R / sum_Core;
        }
    }
    return img2;
}

//插值图像函数
cv::Mat Clamp_Img(cv::Mat &img1, cv::Mat &img2, const float &k)
{
    if(k >= 0.98)//直接输出图像，不进行插值
    {
        return img1.clone();
    }
    else if(k <= 0.02)
    {
        return img2.clone();
    }
    else//图像插值
    {
        int width = img1.rows;
        int height = img1.cols;
        cv::Mat img3 = img1.clone();

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < height; y++)
            {
                img3.at<uchar>(x, y) *= k;
                img3.at<uchar>(x, y) += (1.0 - k) * img2.at<uchar>(x, y);
            }
        }
        
        return img3;
    }
}
//反差计算函数
float Difference(cv::Mat &img1, float l, float r, float t, float b)
{
    int m = 4;
    int n = 4;
    int width = img1.rows;
    int height = img1.cols;
    cv::Mat img2 = img1.clone();

#pragma omp parallel for
    for (int x = int(width * t); x < int(width * b); x++)
    {
        for (int y = int(height * l); y < int(height * r); y++)
        {
            //边界判断，防止溢出
            int neg_xRadius = -std::min(x, m);
            int neg_yRadius = -std::min(y, n);
            int pos_xRadius = std::min(width - 1 - x, m);
            int pos_yRadius = std::min(height - 1 - y, n);
            float k = img1.at<uchar>(x, y);
            float delta = 0.0f;
            float sum_Core = 0.0f;
            for (int a = neg_xRadius; a <= pos_xRadius; a++)
            {
                for (int b = neg_yRadius; b <= pos_yRadius; b++)
                {
                    float temp = 1.0 + abs(img1.at<uchar>(x + a, y + b) - k) / 255.0f;
                    delta += log(temp) / temp * 255.0f;
                    sum_Core += 1.0;
                }
            }
            img2.at<uchar>(x, y) = delta / sum_Core;
        }
    }

    cv::imshow("img2", img2);

    float sum = 0.0;
    float core = 0.0;
    //对对焦区域计算平均反差
    for (int x = int(width * t); x < int(width * b); x++)
    {
        for (int y = int(height * l); y < int(height * r); y++)
        {
            sum += img2.at<uchar>(x, y);
            core += 1.0;
        }
    }
    return sum / core;
}
