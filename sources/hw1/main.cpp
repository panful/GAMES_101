#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <numbers>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926535898;

/// @brief 绕任意轴axis顺时针旋转angle度（注意此处是角度）
/// @param axis
/// @param angle
/// @return
Eigen::Matrix4f get_rotation(Vector3f axis, float angle)
{
    Eigen::Matrix4f mat { Eigen::Matrix4f::Identity() };

    angle = angle / 180.0f * std::numbers::pi_v<float>;

    axis.normalize();
    float X = axis.x();
    float Y = axis.y();
    float Z = axis.z();

    auto cos0 = std::cos(angle);
    auto sin0 = std::sin(angle);

    mat(0, 0) = cos0 + X * X * (1 - cos0);
    mat(1, 0) = X * Y * (1 - cos0) - Z * sin0;
    mat(2, 0) = X * Z * (1 - cos0) + Y * sin0;

    mat(0, 1) = Y * Z * (1 - cos0) + Z * sin0;
    mat(1, 1) = cos0 + Y * Y * (1 - cos0);
    mat(2, 1) = Y * Z * (1 - cos0) - Z * sin0;

    mat(0, 2) = X * Z * (1 - cos0) - Y * sin0;
    mat(1, 2) = Y * Z * (1 - cos0) + X * sin0;
    mat(2, 2) = cos0 + Z * Z * (1 - cos0);

    return mat;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    // View矩阵，其实就是一个平移矩阵
    Eigen::Matrix4f translate;
    // clang-format off
    translate << 1, 0, 0, -eye_pos[0], 
                 0, 1, 0, -eye_pos[1], 
                 0, 0, 1, -eye_pos[2], 
                 0, 0, 0, 1;
    // clang-format off

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.


    Eigen::Matrix4f rotation;
    float sinAngle = sin(rotation_angle), cosAngle = cos(rotation_angle);
    // clang-format off
    // 绕X轴旋转的矩阵
    // rotation << 1, 0,         0,        0, 
    //             0, cosAngle, -sinAngle, 0, 
    //             0, sinAngle,  cosAngle, 0, 
    //             0, 0,         0,        1;

    // 绕Y轴旋转的矩阵
    // rotation << cosAngle, 0, sinAngle, 0, 
    //             0,        1, 0,        0, 
    //            -sinAngle, 0, cosAngle, 0, 
    //             0,        0, 0,        1;

    // 绕Z轴旋转的矩阵
    // rotation << cosAngle, -sinAngle, 0, 0, 
    //             sinAngle,  cosAngle, 0, 0, 
    //             0,         0,        1, 0, 
    //             0,         0,        0, 1;
    // clang-format on

    // 绕 (1,1,1) 旋转
    rotation = get_rotation(Eigen::Vector3f { 0.f, 0.f, 1.f }, rotation_angle);
    model *= rotation;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    // 透视投影，需要将视锥体压扁为立方体，再做正交变换（平移、缩放）
    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Identity();
    // clang-format off
    persp2ortho << zNear, 0,     0,             0, 
                   0,     zNear, 0,             0, 
                   0,     0,     zNear + zFar, -zNear * zFar, 
                   0,     0,     1,             0;
    // clang-format on

    double halfEyeRadian = eye_fov * MY_PI / 2.0 / 180.0;
    double top           = zNear * tan(halfEyeRadian);
    double bottom        = -top;
    double right         = top * aspect_ratio;
    double left          = -right;

    // 正交投影，通过平移缩放，将视锥体变换为原点：(0,0,0),大小：[2,2,2]即xyz的范围为[-1,1]
    Eigen::Matrix4f orthoScale = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f orthoTrans = Eigen::Matrix4f::Identity();

    // clang-format off
    orthoScale << 2 / (float)(right - left), 0,                         0,                  0, 
                  0,                         2 / (float)(top - bottom), 0,                  0, 
                  0,                         0,                         2 / (zNear - zFar), 0, 
                  0,                         0,                         0,                  1;

    orthoTrans << 1, 0, 0, -(float)(right + left) / 2, 
                  0, 1, 0, -(float)(top + bottom) / 2, 
                  0, 0, 1, -(zNear + zFar) / 2, 
                  0, 0, 0,  1;
    // clang-format on

    Eigen::Matrix4f matrixOrtho = orthoScale * orthoTrans;
    projection                  = matrixOrtho * persp2ortho;

    return projection;
}

/*
 * 命令行执行程序：Windows >hw1.exe Linux >hw1
 * 退出程序：Esc
 *./hw1.exe 循环运行程序，创建一个窗口显示，且你可以使用A键和D键旋转三角形。
 *./hw1.exe −r 20 运行程序并将三角形旋转20度,然后将结果存在output.png中
 *./hw1.exe −r 20 image.png 运行程序并将三角形旋转20度，然后将结果存在image.png中。
 */

int main(int argc, const char** argv)
{
    float angle          = 0.f;
    bool command_line    = false;
    std::string filename = "output.png";

    if (argc >= 3)
    {
        command_line = true;
        angle        = std::stof(argv[2]); // -r by default

        if (argc == 4)
        {
            filename = std::string(argv[3]);
        }
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = { 0, 0, 5 };

    std::vector<Eigen::Vector3f> pos { { 2, 0, -2 }, { 0, 2, -2 }, { -2, 0, -2 } };

    std::vector<Eigen::Vector3i> ind { { 0, 1, 2 } };

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key         = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    // 按下Esc退出循环
    while (key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1f, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a')
        {
            angle += 1;
        }
        else if (key == 'd')
        {
            angle -= 1;
        }
    }

    return 0;
}
