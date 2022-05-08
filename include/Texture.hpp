//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;

        auto u_img = u * width;
        auto v_img = (1 - v) * height;

        float u0_ = std::floor(u_img);
        float u1_ = std::min(std::ceil(u_img), (float)width);
        float u_0 = std::floor(v_img);
        float u_1 = std::min(std::ceil(v_img), (float)height);

        float s = (u_img - u0_) / (u1_ - u0_);
        float t = (v_img - u_0) / (u_1 - u_0);
        //opencv中高坐标与uv坐标相反
        auto color_1 = image_data.at<cv::Vec3b>(u_0, u0_);
        auto color_2 = image_data.at<cv::Vec3b>(u_1, u0_);
        auto color_3 = image_data.at<cv::Vec3b>(u_0, u1_);
        auto color_4 = image_data.at<cv::Vec3b>(u_1, u1_);

        Eigen::Vector3f color1(color_1[0], color_1[1], color_1[2]);
        Eigen::Vector3f color2(color_2[0], color_2[1], color_2[2]);
        Eigen::Vector3f color3(color_3[0], color_3[1], color_3[2]);
        Eigen::Vector3f color4(color_4[0], color_4[1], color_4[2]);

        auto U0 =(1 - s) * color1 + s * color3;
        auto U1 =(1 - s) * color2 + s * color4;

        auto color = (1 - t) * U0 + t * U1;
        return color;
    }

};
#endif //RASTERIZER_TEXTURE_H
