/*
 * 0.Eigen和OpenCV环境测试

 * 101.Eigen中向量的常见运算
 * 102.Eigen中矩阵的常见运算
 * 103.设置矩阵的元素
 * 104.缩放、平移、旋转矩阵的生成
 *
 * 201.OpenCV写图片
 */

// Eigen 使用笔记：
// http://zhaoxuhui.top/blog/2019/08/21/eigen-note-1.html
// https://zxl19.github.io/eigen-note/

#define TEST201

#ifdef TEST0
#include <Eigen/Core>
#include <iostream>
#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat cv_mat {};
    Eigen::Matrix<int, 3, 3> eigen_mat {};

    std::cout << "Test OpenCV Eigen\n";
}
#endif // TEST0

#ifdef TEST101

#include <Eigen/Dense>
#include <iostream>

int main()
{
    Eigen::Vector3f v1(1.f, 2.f, 3.f);
    Eigen::Vector3f v2(4.f, 5.f, 6.f);

    // 打印向量，默认列向量
    std::cout << "-----------------------------------------------\n"
              << "Eigen::Vector3f(1.f, 2.f, 3.f) = \n"
              << v1 << '\n'
              << "Eigen::Vector3f(4.f, 5.f, 6.f) = \n"
              << v2 << '\n';

    // 向量和标量运算，Eigen中向量不能与标量使用 + - 运算符
    std::cout << "-----------------------------------------------\n"
              << "(1.f, 2.f, 3.f) * 3 = \n"
              << v1 * 3 << '\n'
              << "(1.f, 2.f, 3.f) / 3 = \n"
              << v1 / 3 << '\n'
              << '\n';

    // 向量与向量相加、相减；向量与向量的点积；向量与向量的叉积；向量与向量不能使用 * 运算符
    std::cout << "-----------------------------------------------\n"
              << "(1.f, 2.f, 3.f) - (4.f, 5.f, 6.f) = \n"
              << v1 - v2 << '\n'
              << "(1.f, 2.f, 3.f) + (4.f, 5.f, 6.f) = \n"
              << v1 + v2 << '\n'
              << "(1.f, 2.f, 3.f) . (4.f, 5.f, 6.f) = \n"
              << v1.dot(v2) << '\n'
              << "(1.f, 2.f, 3.f) x (4.f, 5.f, 6.f) = \n"
              << v1.cross(v2) << '\n';
}

#endif // TEST101

#ifdef TEST102

#include <Eigen/Dense>
#include <iostream>

int main()
{
    Eigen::Matrix3d m1; // 默认初始化为0
    Eigen::Matrix3d m2;

    std::cout << "--------------------------------------------\n"
              << m1 << "\n"
              << "--------------------------------------------\n"
              << m2 << '\n';

    m1 << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0; // 先行后列，(1,2,3)为第一行
    m2 << 1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0;

    std::cout << "--------------------------------------------\n"
              << m1 << "\n"
              << "--------------------------------------------\n"
              << m2 << '\n';

    // 矩阵与矩阵相加
    std::cout << "--------------------------------------------\n" << m1 << "\n+\n" << m2 << "\n=\n" << m1 + m2 << '\n';
    // 矩阵与矩阵相乘
    std::cout << "--------------------------------------------\n" << m1 << "\n*\n" << m2 << "\n=\n" << m1 * m2 << '\n';
    // 矩阵与标量相乘
    std::cout << "--------------------------------------------\n" << m1 << "\n*\n" << 2 << "\n=\n" << m1 * 2 << '\n';

    // 矩阵和向量相乘，矩阵必须在前面，因为Eigen中向量默认为列向量
    Eigen::Vector3d vec({ 1.0, 2.0, 3.0 });
    std::cout << "--------------------------------------------\n" << m1 << "\n*\n" << vec << "\n=\n" << m1 * vec << '\n';
}

#endif // TEST102

#ifdef TEST103

#include <Eigen/Dense>
#include <iostream>

int main()
{
    // 创建一个单位矩阵
    Eigen::Matrix4d mat { Eigen::Matrix4d::Identity() };
    std::cout << "----------------\n" << mat << '\n';

    // 设置矩阵的[0,0]元素为2，[M,N]第M行，第N列
    mat(0, 0) = 2;
    std::cout << "----------------\n" << mat << '\n';

    // 设置矩阵的所有元素值为0
    mat = Eigen::Matrix4d::Zero();
    std::cout << "----------------\n" << mat << '\n';

    // 设置矩阵的所有元素值为1
    mat = Eigen::Matrix4d::Ones();
    std::cout << "----------------\n" << mat << '\n';
}

#endif // TEST103

#ifdef TEST104

#include <Eigen/Dense>
#include <iostream>
#include <numbers>

int main()
{
    // 缩放
    {
        Eigen::Vector4d vec { 1, 2, 0, 1 };
        Eigen::Matrix4d mat { Eigen::Matrix4d::Identity() };

        mat(0, 0) = 2.0; // x缩放为原来的2倍
        mat(1, 1) = 0.5; // y缩放为原来的0.5倍
        mat(2, 2) = 1.0; // z不缩放

        std::cout << "--------------------------\n" << mat << '\n';
        std::cout << mat * vec << '\n';
    }

    // 平移
    {
        Eigen::Vector4d vec { 1, 2, 0, 1 };
        Eigen::Matrix4d mat { Eigen::Matrix4d::Identity() };

        mat(0, 3) = 1;  // 向+x平移1个单位
        mat(1, 3) = 2;  // 向+y平移2个单位
        mat(2, 3) = -1; // 向-z平移1个单位

        std::cout << "--------------------------\n" << mat << '\n';
        std::cout << mat * vec << '\n';
    }

    // 旋转
    {
        Eigen::Vector4d vec { 1, 0, 0, 1 };
        Eigen::Matrix4d mat { Eigen::Matrix4d::Identity() };

        double angle = 45.0 / 180.0 * std::numbers::pi; // 旋转的弧度（逆时针）
        double X     = 0.0;                             // XYZ组成旋转的轴
        double Y     = 0.0;
        double Z     = 1.0;

        auto cos0 = std::cos(angle);
        auto sin0 = std::sin(angle);

        // 注意：Eigen是列向量 glm是行向量
        mat(0, 0) = cos0 + X * X * (1 - cos0);
        mat(1, 0) = X * Y * (1 - cos0) - Z * sin0;
        mat(2, 0) = X * Z * (1 - cos0) + Y * sin0;

        mat(0, 1) = Y * Z * (1 - cos0) + Z * sin0;
        mat(1, 1) = cos0 + Y * Y * (1 - cos0);
        mat(2, 1) = Y * Z * (1 - cos0) - Z * sin0;

        mat(0, 2) = X * Z * (1 - cos0) - Y * sin0;
        mat(1, 2) = Y * Z * (1 - cos0) + X * sin0;
        mat(2, 2) = cos0 + Z * Z * (1 - cos0);

        std::cout << "--------------------------\n" << mat << '\n';
        std::cout << mat * vec << '\n';
    }
}

#endif // TEST104

#ifdef TEST201

#include <opencv2/opencv.hpp>

int main()
{
    // 创建一个空的黑色图像，大小为100x100
    cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));

    // 设置图像为绿色（OpenCV使用BGR颜色通道顺序）
    cv::Scalar green_color(0, 255, 0);
    image.setTo(green_color);

    // 将图像写入文件
    if (cv::imwrite("green_image.png", image))
    {
        std::cout << "image write success\n";
    }

    // 在窗口中显示图片
    cv::imshow("OpenCV Window", image);
    cv::waitKey();           // 等待按下任意键盘上的按钮，返回值是按下的按钮，参数是延时（单位ms）
    cv::destroyAllWindows(); // 销毁窗口
}

#endif // TEST201
