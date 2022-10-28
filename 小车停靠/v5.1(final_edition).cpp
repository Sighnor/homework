#include <stdlib.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cv.h>
#include <highgui.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include <boost/thread.hpp>
#include <geometry_msgs/Twist.h>

int fov = 60;//y轴视场角
int width_scene = 672;//宽度
int height_scene = 376;//高度
float scale;//FOV对应的tan
float imageAspectRatio;//x,y比例
float distance_x = 120;//两相机之间的像素距离
float eye_pos_y = 500;//相机的像素高度

int left_theta = 0;
int right_theta = 0;

std::vector<float> ray_x;
std::vector<float> left_ray_inv_y;
std::vector<float> left_ray_z;
std::vector<float> right_ray_inv_y;
std::vector<float> right_ray_z;
std::vector<int> my_length;

int min_H_B = 99;//色调最低阈值
int max_H_B = 140;//色调最高阈值
int min_H_G = 35;//色调最低阈值
int max_H_G = 99;//色调最高阈值
int min_S = 35;//饱和度最低阈值
int min_V = 35;//强度最低阈值

float p_vec = 0;//速度的比例
float i_vec = 0;//速度的积分
float d_vec = 0;//速度的微分
float p_omega = 0;//角度的比例
float i_omega = 0;//角度的积分
float d_omega = 0;//角度的微分

float kp_vec = 0.0020;
float ki_vec = 0;
float kd_vec = 0;
float kp_omega = 0.50;
float ki_omega = 0;
float kd_omega = 0;

float turn_omega = 0;
int range_boundingbox = 10;
float center_x = 0;
float center_y = 0;
float angle = 0;
int total_height = 0;
int total_width = 0;
int frame_count = 0;
int time_count = 40;
float last_vec;
float last_omega;
int flag_find = 0;
int flag_lost = 0;
float vec_rotate = 0.2;
float turn_change = 0;
int turn_flag = 0;

ros::Publisher cmd_pub;
geometry_msgs::Twist twist;

cv::Mat filter(cv::Mat before)//滤波
{
    cv::Mat after = before.clone();
    std::vector<int> my_left(before.rows);//存储每行最左侧像素位置
    std::vector<int> my_right(before.rows);//存储每行最右侧像素位置

        //对每行插值处理断点
        for (int i = 0; i < before.rows; i++)
        {
            int color = 0;//颜色，0代表黑色，1代表有颜色
            int change = 0;//颜色微分
            //对于每个断点，寻找其左侧有效像素和右侧有效像素，利用距离进行插值
            int begin_x = -1;
            int end_x = -1;
            for (int j = 0; j < before.cols; j++)
            {
                if(before.at<uchar>(i, j * 3 + 0) != 0 || before.at<uchar>(i, j * 3 + 1) != 0 || before.at<uchar>(i, j * 3 + 2) != 0)
                {
                    change = 1 - color;
                    color = 1;
                }
                else
                {
                    change = - color;
                    color = 0;
                }
                if(change == -1)//颜色由彩色变成黑色
                {
                    begin_x = j - 1;//该点 - 1即左侧有效像素
                    my_right[i] = j;//该点有可能为最右侧像素
                }
                else if(change == 1)//颜色由黑色变成彩色
                {
                    if(begin_x == -1)//如果这是第一个有效像素，则它是最左侧像素s
                    {
                        my_left[i] = j;
                    }
                    else//它为右侧有效像素
                    {
                        end_x = j;
                        for(int k = begin_x + 1; k < end_x; k++)//对中间断点进行插值
                        {
                            after.at<uchar>(i, k * 3 + 0) = (end_x - k) / float(end_x - begin_x) * before.at<uchar>(i, begin_x * 3 + 0) + (k - begin_x) / float(end_x - begin_x) * before.at<uchar>(i, end_x* 3 + 0);
                            after.at<uchar>(i, k * 3 + 1) = (end_x - k) / float(end_x - begin_x) * before.at<uchar>(i, begin_x * 3 + 1) + (k - begin_x) / float(end_x - begin_x) * before.at<uchar>(i, end_x* 3 + 1);
                            after.at<uchar>(i, k * 3 + 2) = (end_x - k) / float(end_x - begin_x) * before.at<uchar>(i, begin_x * 3 + 2) + (k - begin_x) / float(end_x - begin_x) * before.at<uchar>(i, end_x* 3 + 2);
                        }
                    }
                }
            }
            int j = before.cols - 1;//处理最右边像素为终点情况
            if(before.at<uchar>(i, j * 3 + 0) != 0 || before.at<uchar>(i, j * 3 + 1) != 0 || before.at<uchar>(i, j * 3 + 2) != 0)
            {
                my_right[i] = j;
            }
        }
        //对每列进行插值
        cv::Mat my_img = after.clone();
        int j = after.cols / 2;//中间列开始
        int color = 0;
        int change = 0;
        int top_y = -1;
        int below_y = -1;
        //寻找断行进行处理
        for (int i = 0; i < after.rows; i++)
        {
            if(after.at<uchar>(i, j * 3 + 0) != 0 || after.at<uchar>(i, j * 3 + 1) != 0 || after.at<uchar>(i, j * 3 + 2) != 0)
            {
                change = 1 - color;
                color = 1;
            }
            else
            {
                change = - color;
                color = 0;
            }
            if(change == -1)
            {
                top_y = i - 1;
            }
            else if(change == 1 && top_y != -1)
            {
                below_y = i;
                //对每个断行插值
                for(int k = top_y + 1; k < below_y; k++)
                {
                    int long_width = my_right[top_y] - my_left[top_y];//上方有效行长度
                    int short_width = my_right[below_y] - my_left[below_y];//下方有效行长度s
                    float top_height = k - top_y;//到上方的距离
                    float below_height = below_y - k;//到下方的距离s
                    float y1 = below_height / ((top_height + below_height));
                    float y2 = 1 - y1;
                    float left_x = my_left[top_y] * y1 + my_left[below_y] * y2;//插值得到最左侧坐标
                    float right_x = my_right[top_y] * y1 + my_right[below_y] * y2;//插值得到最右侧坐标s
                    float x_width = right_x - left_x;//当前行长度
                    for(int n = int(left_x + 1); n < int(right_x); n++)
                    {
                        float ratio_x = (n - left_x) / x_width;//该点在当前行比例
                        int top_x = my_left[top_y] + long_width * ratio_x;//用比例算出上方行对应点
                        int below_x = my_left[below_y] + short_width * ratio_x;//用比例算出下方行对应点
                        //插值计算像素
                        my_img.at<uchar>(k, n * 3 + 0) = after.at<uchar>(top_y, top_x * 3 + 0) * y1 + after.at<uchar>(below_y, below_x * 3 + 0) * y2;
                        my_img.at<uchar>(k, n * 3 + 1) = after.at<uchar>(top_y, top_x * 3 + 1) * y1 + after.at<uchar>(below_y, below_x * 3 + 1) * y2;
                        my_img.at<uchar>(k, n * 3 + 2) = after.at<uchar>(top_y, top_x * 3 + 2) * y1 + after.at<uchar>(below_y, below_x * 3 + 2) * y2;
                    }
                }
            }
        }
    return my_img;
}

bool Inside_boundingbox(int x, int y, cv::Rect bounding_box)
{
    return (x > bounding_box.tl().x - range_boundingbox && x < bounding_box.br().x + range_boundingbox
    && y > bounding_box.tl().y - range_boundingbox && y < bounding_box.br().y + range_boundingbox) 
    ? true : false;
}

cv::Rect Find_rect(cv::Mat img)//寻找最小外接矩形
{
    int num_boundingbox = 0;
    std::vector<int> num_pixel;
    std::vector<cv::Rect> my_boundingbox;
    int max_pixel = 1200;
    int my_i = -1;
    for(int i = 0; i < img.rows; i++)  
    {  
        uchar* temp = img.ptr<uchar>(i);
        for(int j = 0; j < img.cols; j++)   
        {  
            if(temp[j] > 0)
            {
                int flag = 0;
                for(int k = 0;k < num_boundingbox && flag == 0; k++)
                {
                    if(Inside_boundingbox(j, i, my_boundingbox[k]))
                    {
                        flag = 1;
                        int left_x = my_boundingbox[k].tl().x;
                        int right_x = my_boundingbox[k].br().x;
                        int top_y = my_boundingbox[k].tl().y;
                        int below_y = my_boundingbox[k].br().y;
                        if(right_x < j)
                        {
                            right_x = j;
                        }
                        else if(left_x > j)
                        {
                            left_x = j;
                        }
                        if(below_y < i)
                        {
                            below_y = i;
                        }
                        else if(top_y > i)
                        {
                            top_y = i;
                        }
                        my_boundingbox[k] = cv::Rect(cv::Point(left_x, top_y), cv::Point(right_x, below_y));
                        num_pixel[k]++;
                    }
                }
                if(flag == 0)
                {
                    my_boundingbox.push_back(cv::Rect(cv::Point(j, i), cv::Point(j, i)));
                    num_pixel.push_back(1);
                    num_boundingbox++;
                }
            }
        }  
    }
    for(int i  = 0; i < num_boundingbox; i++)
    {
        float ratio_height_width = my_boundingbox[i].height / float(my_boundingbox[i].width);
        if(num_pixel[i] > max_pixel && ratio_height_width < 1.5f && ratio_height_width > 0.66f && my_boundingbox[i].height < 220 && my_boundingbox[i].width < 220)
        {
            my_i = i;
            max_pixel = num_pixel[i];
        }
    } 
    if (my_i != -1)
    {
        return my_boundingbox[my_i];
    }
    else
    {
        return cv::Rect(cv::Point(0, 0),cv::Point(720, 720));
    }
}

cv::RotatedRect Find_minrect(cv::Rect bounding_box, cv::Mat img)
{
    std::vector<cv::Point> contours;

        for(int i = bounding_box.tl().y; i < bounding_box.br().y; i++)
        {
            uchar* temp = img.ptr<uchar>(i);
            for(int j = bounding_box.tl().x; j < bounding_box.br().x; j++)
            { 
                if(temp[j] > 0)
                {
                    contours.push_back(cv::Point(j, i));
                }
            }
        }

    return cv::minAreaRect(cv::Mat(contours));
}

void pid_control(cv::Mat BGR_img, char name[])
{
    cv::Mat temp_img;
    cvtColor(BGR_img, temp_img, CV_BGR2GRAY);
    threshold(temp_img, temp_img, 20, 255, CV_THRESH_BINARY); //二值化
    cv::Rect bounding_box = Find_rect(temp_img);
    cv::rectangle(BGR_img, bounding_box, cv::Scalar(0, 255, 0), 1);

    if(bounding_box.area() < 50000 && flag_lost < 2)
    {
        cv::RotatedRect my_bounding_box = Find_minrect(bounding_box, temp_img); //定义最小外接矩形
        cv::Point2f rect[4];

        std::cout << "angle = " << my_bounding_box.angle << '\n';

        flag_find = 1;

        if(frame_count < 50)
        {
            my_bounding_box.points(rect);
            for(int j=0; j<4; j++)
            {
                line(BGR_img, rect[j], rect[(j+1)%4], cv::Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
            }
            total_height += my_bounding_box.size.height;
            total_width += my_bounding_box.size.width;
            frame_count++;
        }
        else if(frame_count == 50)
        {
            total_height = total_height / 50;
            total_width = total_width / 50;
            frame_count++;
        }
        else
        {
            if((my_bounding_box.size.height > 0.50 * total_height && my_bounding_box.size.height < 2.00 * total_height
            && my_bounding_box.size.width > 0.50 * total_width && my_bounding_box.size.width < 2.00 * total_width) ||
	        (my_bounding_box.size.width > 0.50 * total_height && my_bounding_box.size.width < 2.00 * total_height
            && my_bounding_box.size.height > 0.50 * total_width && my_bounding_box.size.height < 2.00 * total_width))
            {
                my_bounding_box.points(rect);
                for(int j=0; j<4; j++)
                {
                    line(BGR_img, rect[j], rect[(j+1)%4], cv::Scalar(0, 0, 255), 2, 8);  //绘制最小外接矩形每条边
                }
                center_x = my_bounding_box.center.x;
                center_y = my_bounding_box.center.y;
                angle = my_bounding_box.angle;
            }

            float distance_tl = abs((BGR_img.rows - center_y) * cos(angle * M_PI / 180.0) - (BGR_img.cols / 2 - center_x) * sin(angle * M_PI / 180.0));
            float distance_tr = abs((BGR_img.rows - center_y) * sin(angle * M_PI / 180.0) + (BGR_img.cols / 2 - center_x) * cos(angle * M_PI / 180.0));
            int temp_y;

            if(distance_tl < distance_tr)
            {
                temp_y = center_y + (BGR_img.cols / 2 - center_x) * tan(angle * M_PI / 180.0);
            }
            else
            {
                temp_y = center_y + (center_x - BGR_img.cols / 2) / tan(angle * M_PI / 180.0);
            }

            cv::line(BGR_img, cv::Point(BGR_img.cols / 2, BGR_img.rows), cv::Point(BGR_img.cols / 2, temp_y), cv::Scalar(0, 0, 255), 1);
            cv::line(BGR_img, cv::Point(center_x, center_y), cv::Point(BGR_img.cols / 2, temp_y), cv::Scalar(255, 0, 0), 1);

            float distance_ot = (BGR_img.rows - temp_y) * (BGR_img.rows - temp_y);
            float distance_tc = ((center_x - BGR_img.cols / 2) * (center_x - BGR_img.cols / 2) + (center_y - temp_y) * (center_y - temp_y));
            int center_flag = center_x > BGR_img.cols / 2 ? -1 : 1;

            float temp_turn_change;

            if(turn_flag == 1)
            {
                turn_omega = 0;
            }
            else if(turn_omega != 0)
            {
                temp_turn_change = distance_ot - distance_tc;
            }
            else if(temp_y >= BGR_img.rows || temp_y <= center_y)
            {
                turn_omega = center_flag;
                temp_turn_change = 0;
            }
            else if(distance_ot < distance_tc)
            {
                turn_omega = center_flag;
                temp_turn_change = distance_ot - distance_tc;
            }
            else if(distance_ot >= distance_tc)
            {
                turn_omega = -center_flag;
                temp_turn_change = distance_ot - distance_tc;
            }

            if((angle < -87 || angle > -3) && center_x > BGR_img.cols / 2 - 25 && center_x < BGR_img.cols / 2 + 25)
            {
                std::cout << "move forward!" << '\n';
                turn_flag = 1;
                turn_omega = 0;
            }
            else if(center_x > BGR_img.cols / 2 - 25 + my_length[center_y] || center_x < BGR_img.cols / 2 + 25 - my_length[center_y])
            {
                std::cout << "must turn around!" << '\n';
                //turn_flag = 0;  
                //turn_change = 0;
                turn_omega = center_flag; 
            }
            else if(turn_change * temp_turn_change < 0)
            {
                std::cout << "Bezier change!" << '\n';
                turn_flag = 1;
                turn_change = 0;
                turn_omega = 0;
            }
            else
            {
                turn_change = temp_turn_change;
            }

            if(turn_omega != 0)
            {
                p_omega += turn_omega * vec_rotate;
                std::cout << "turn around for Bezier!" << ',' << p_omega << '\n';
            }
            else if((angle < -87 || angle > -3) && center_x > BGR_img.cols / 2 - 25 && center_x < BGR_img.cols / 2 + 25)
            {

                p_vec += 40;
                std::cout << "move straight!" << '\n';
            }
            else
            {
                for(float t = 0;t < 1;t += 0.02)
                {
                    float my_x = t * t * BGR_img.cols / 2 + 2 * t * (1 - t) * BGR_img.cols / 2 + (1 - t) * (1 - t) * center_x;
                    float my_y = t * t * BGR_img.rows + 2 * t * (1 - t) * temp_y + (1 - t) * (1 - t) * center_y;
                    cv::circle(BGR_img, cv::Point(my_x, my_y), 1, cv::Scalar(255, 255, 255));
                }
                float t = 0.9;
                float my_x = t * t * BGR_img.cols / 2 + 2 * t * (1 - t) * BGR_img.cols / 2 + (1 - t) * (1 - t) * center_x;
                float my_y = t * t * BGR_img.rows + 2 * t * (1 - t) * temp_y + (1 - t) * (1 - t) * center_y;
                //std::cout << "x = " << my_x << ',' << "y = " << my_y << ',' << "temp_y = " << temp_y << '\n' << '\n';
                if(temp_y > BGR_img.rows - 20)
                {
                    p_vec += 0;
                    /*if(my_x < BGR_img.cols/2)
                    {
                        p_omega += vec_rotate;
                    }
                    else
                    {
                        p_omega += -vec_rotate;
                    }*/
		    p_omega +=  1.2f * atan((BGR_img.cols/2 - my_x) / float(BGR_img.rows - my_y));
                    std::cout << "turn Bezier!" << '\n';
                }
                else
                {
                    p_vec += 3 * (BGR_img.rows - my_y);
                    p_omega +=  atan((BGR_img.cols/2 - my_x) / float(BGR_img.rows - my_y));
                    std::cout << "run Bezier!" << '\n';
                }
            }
	    }
    }
    else
    {
        if(frame_count > 50)
        {
            flag_lost++;
            if(time_count > 0)
            {
                std::cout << "go!" << '\n';
		if(last_vec == 0)
		{	
			p_vec += 40;
		}                
		p_vec += last_vec;
                p_omega += last_omega;
                time_count--;
            }
            else
            {
                std::cout << "stop!" << '\n';
                p_vec += 0;
                p_omega += 0;
            }
        }
        else
        {
            std::cout << "turn to find!" << '\n';
            p_omega -= vec_rotate;
        }
    }

    cv::imshow(name, BGR_img);
    
    /*std::cout << center_x << ',' << center_y << '\n';
    std::cout << p_vec << ',' << i_vec << ',' << d_vec << '\n';
    std::cout << p_omega << ',' << i_omega << ',' << d_omega << '\n' << '\n';*/
}

cv::Mat color_cut(cv::Mat img)
{
        cv::Mat BGR_img = img.clone();
        cv::Mat HSV_img;
        cv::cvtColor(BGR_img, HSV_img, cv::COLOR_BGR2HSV);
        for(int y = 0; y < HSV_img.rows; y++)//颜色阈值滤波
        {
            for(int x = 0; x < HSV_img.cols; x++)
            {
                        int temp_H = HSV_img.at<uchar>(y, x * 3 + 0);
                        int temp_S = HSV_img.at<uchar>(y, x * 3 + 1);
                        int temp_V = HSV_img.at<uchar>(y, x * 3 + 2);
                        if(temp_V < min_V || temp_S < min_S)
                        {
                            BGR_img.at<uchar>(y, x * 3 + 0) = 0; 
                            BGR_img.at<uchar>(y, x * 3 + 1) = 0; 
                            BGR_img.at<uchar>(y, x * 3 + 2) = 0; 
                        }
                        else
                        {
                            if((temp_H >= min_H_B && temp_H <= max_H_B) || (temp_H >= min_H_G && temp_H <= max_H_G))
                            {

                            }
                            else
                            {
                                BGR_img.at<uchar>(y, x * 3 + 0) = 0; 
                                BGR_img.at<uchar>(y, x * 3 + 1) = 0; 
                                BGR_img.at<uchar>(y, x * 3 + 2) = 0; 
                            } 
                        }
            }
        } 
        return BGR_img;
}

void rebuild_2D(float eye_pos_y, float ratio, int height, int width, cv::Mat img)
{    
        cv::Mat left_BGR_img(width, width, CV_8UC3, cv::Scalar(0,0,0));//初始化为黑色
 	    cv::Mat right_BGR_img(width, width, CV_8UC3, cv::Scalar(0,0,0));//初始化为黑色
        for(int y = img.rows / 2; y < img.rows; y++)//在下半边寻找
        {
                uchar* temp = img.ptr<uchar>(y);
                float left_distance = - eye_pos_y * left_ray_inv_y[y * width_scene];//距离
                int left_temp_y = (width + ratio * left_distance * left_ray_z[y * width_scene + img.cols / 4]);//重构y坐标
                for(int x = 0; x < img.cols / 2; x++)
                {
                    int left_temp_x = (ratio * (left_distance * ray_x[y * width_scene + x] - distance_x / 2) + width / 2.0f);//重构x坐标
                    if(left_temp_x > 0 && left_temp_x < width && left_temp_y > 0 && left_temp_y < width)
                    {
                        
                        left_BGR_img.at<uchar>(left_temp_y, left_temp_x * 3 + 0) = temp[3 * x];
                        left_BGR_img.at<uchar>(left_temp_y, left_temp_x * 3 + 1) = temp[3 * x + 1];
                        left_BGR_img.at<uchar>(left_temp_y, left_temp_x * 3 + 2) = temp[3 * x + 2]; 
                    }
                }
		        float right_distance = - eye_pos_y * right_ray_inv_y[y * width_scene];//距离
                int right_temp_y = (width + ratio * right_distance * right_ray_z[y * width_scene + 3 * img.cols / 4]);//重构y坐标
                for(int x = img.cols / 2; x < img.cols; x++)
                {
                    int right_temp_x = (ratio * (right_distance * ray_x[y * width_scene + x] + distance_x / 2) + width / 2.0f);//重构x坐标
                    if(right_temp_x > 0 && right_temp_x < width && right_temp_y > 0 && right_temp_y < width)
                    {                                
                        right_BGR_img.at<uchar>(right_temp_y, right_temp_x * 3 + 0) = temp[3 * x];
                        right_BGR_img.at<uchar>(right_temp_y, right_temp_x * 3 + 1) = temp[3 * x + 1];
                        right_BGR_img.at<uchar>(right_temp_y, right_temp_x * 3 + 2) = temp[3 * x + 2]; 
                    }
                }
        }
	cv::imshow("left_before", left_BGR_img);  
        left_BGR_img = filter(left_BGR_img);//特殊均值滤波，处理图像断线
	    right_BGR_img = filter(right_BGR_img); 
	    cv::imshow("left_2DRebuild", left_BGR_img);  
	    cv::imshow("right_2DRebuild", right_BGR_img); 
        left_BGR_img = color_cut(left_BGR_img);
	    right_BGR_img = color_cut(right_BGR_img);
	    char left_name[10] = "left";
	    char right_name[10] = "right";
        pid_control(left_BGR_img, left_name);//pid控制
        //pid_control(right_BGR_img, right_name);//pid控制
}

void control(int temp_height, int temp_width, float temp_ratio, void* usrdata)
{
	    cv::Mat img = *(cv::Mat*)(usrdata);
        rebuild_2D(eye_pos_y, temp_ratio, temp_height, temp_width, img);//重构函数
}

void sendCmd() 
{  
    last_vec = p_vec;
    last_omega = p_omega;

    //pid控制
    float my_vec = kp_vec * p_vec + ki_vec * i_vec + kd_vec * d_vec;
    float my_omega = kp_omega * p_omega + ki_omega * i_omega + kd_omega * d_omega;

        //std::cout << "vec = " << my_vec << ',' << "omega = " << my_omega << '\n' << '\n';

        if(p_vec > 0.15)//限制线速度
        {
            my_vec = 0.15 ;
        }
        if(my_omega > 0.5)//限制角速度
        {
            my_omega = 0.5;
        }

    p_vec = 0;//速度的比例
    i_vec = 0;//速度的积分
    d_vec = 0;//速度的微分
    p_omega = 0;//角度的比例
    i_omega = 0;//角度的积分
    d_omega = 0;//角度的微分

    twist.linear.x = my_vec;//线速度
    twist.linear.y = 0;
    twist.linear.z = 0;
    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = my_omega;//角速度
    ROS_WARN("Run");//调试使用，打印输出（可以看程序执行到哪里）
    cmd_pub.publish(twist); //发布消息  
}

void left_translate(int pos, void* userdata)
{
    float my_theta = 0.1f * (pos - 180 - left_theta);
    for (int j = 0; j < height_scene; j++) 
    {
        for (int i = 0; i < width_scene; i++) 
        {
            float temp_y = 1 / left_ray_inv_y[j * width_scene + i];
            float temp_z = left_ray_z[j * width_scene + i];
            left_ray_inv_y[j * width_scene + i] = 1 / (cos(my_theta * 3.14159f / 180) * temp_y - sin(my_theta * 3.14159f / 180) * temp_z);
            left_ray_z[j * width_scene + i] = sin(my_theta * 3.14159f / 180) * temp_y + cos(my_theta * 3.14159f / 180) * temp_z;
        }
    }
    left_theta = pos - 180;
}

void right_translate(int pos, void* userdata)
{
    float my_theta = 0.1f * (pos - 180 - right_theta);
    for (int j = 0; j < height_scene; j++) 
    {
        for (int i = 0; i < width_scene; i++) 
        {
            float temp_y = 1 / right_ray_inv_y[j * width_scene + i];
            float temp_z = right_ray_z[j * width_scene + i];
            right_ray_inv_y[j * width_scene + i] = 1 / (cos(my_theta * 3.14159f / 180) * temp_y - sin(my_theta * 3.14159f / 180) * temp_z);
            right_ray_z[j * width_scene + i] = sin(my_theta * 3.14159f / 180) * temp_y + cos(my_theta * 3.14159f / 180) * temp_z;
        }
    }
    right_theta = pos - 180;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"ColorMove");//初始化 ROS 节点
    ros::NodeHandle n;
    cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);//定义速度发布器

    cv::VideoCapture capture;
	capture.open(1); //打开zed相机，如果要打开笔记本上的摄像头，需要改为0
	cv::waitKey(100);
    cv::Mat img(height_scene, 2 * width_scene, CV_8UC3, cv::Scalar(0,0,0));

    scale = tan(fov * 0.5 * 3.1415926f / 180.0f);//FOV对应的tan
    imageAspectRatio = width_scene / (float)height_scene;//x,y比例

    for (int j = 0; j < height_scene; j++) 
    {
        for (int i = 0; i < width_scene; i++) 
        {
            float x = 2.12f * (2 * (i + 0.5) / (float)width_scene - 1) *
                      imageAspectRatio * scale;
            float y = 2.12f * (1 - 2 * (j + 0.5) / (float)height_scene) * scale;
            ray_x.push_back(x);
            left_ray_inv_y.push_back(1/(float)y);
            left_ray_z.push_back(-2.12f);
	        right_ray_inv_y.push_back(1/(float)y);
            right_ray_z.push_back(-2.12f);
        }
    }

    int key = 0;
    int left_theta = 90;
    int right_theta = 90;

    left_translate(left_theta, &img);
    right_translate(right_theta, &img);	

    cv::imshow("img", img);
    cv::createTrackbar("min_H_B", "img", &min_H_B, 255);
    cv::createTrackbar("max_H_B", "img", &max_H_B, 255);
    cv::createTrackbar("min_H_G", "img", &min_H_G, 255);
    cv::createTrackbar("max_H_G", "img", &max_H_G, 255);
    cv::createTrackbar("min_S", "img", &min_S, 100);
    cv::createTrackbar("min_V", "img", &min_V, 100);
    cv::createTrackbar("left_angle", "img", &left_theta, 360, left_translate, &img);
    cv::createTrackbar("right_angle", "img", &right_theta, 360, right_translate, &img);

    int temp_height = eye_pos_y / scale;//暂时没用
    int temp_width = 720;//重构图像宽度
    float temp_ratio = (temp_width * 0.50f) / (2 * eye_pos_y * imageAspectRatio);//比例，最近点在图中占0.25倍长度

    float temp_scale = scale * imageAspectRatio;
    for(int i = 0;i < temp_width;i++)
    {
        int length = (temp_width - i) * temp_scale;
        if(length > temp_width / 2)
        {
            my_length.push_back(temp_width / 2);
        }
        else
        {
            my_length.push_back(length);
        }
    }

    while (key != 27) 
    {
        capture.read(img);
        cv::imshow("img", img);
	    //std::cout << img.rows << ',' << img.cols << '\n';
        control(temp_height, temp_width, temp_ratio, &img);
	    sendCmd();
        key = cv::waitKey(50);
    }

    return 0;
}
