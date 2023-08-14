//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <stdexcept>

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return { id };
}

// Bresenham's line drawing algorithm
// Code taken from a stack overflow answer: https://stackoverflow.com/a/16405254
void rst::rasterizer::draw_line(Eigen::Vector3f begin, Eigen::Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    // 直线的颜色 BGR
    Eigen::Vector3f line_color = { 255.f, 255.f, 255.f };

    int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

    dx  = static_cast<int>(x2 - x1);
    dy  = static_cast<int>(y2 - y1);
    dx1 = static_cast<int>(fabs(dx));
    dy1 = static_cast<int>(fabs(dy));
    px  = 2 * dy1 - dx1;
    py  = 2 * dx1 - dy1;

    if (dy1 <= dx1)
    {
        if (dx >= 0)
        {
            x  = static_cast<int>(x1);
            y  = static_cast<int>(y1);
            xe = static_cast<int>(x2);
        }
        else
        {
            x  = static_cast<int>(x2);
            y  = static_cast<int>(y2);
            xe = static_cast<int>(x1);
        }

        Eigen::Vector3f point = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 1.0f);
        set_pixel(point, line_color);

        for (i = 0; x < xe; i++)
        {
            x = x + 1;
            if (px < 0)
            {
                px = px + 2 * dy1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    y = y + 1;
                }
                else
                {
                    y = y - 1;
                }
                px = px + 2 * (dy1 - dx1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 1.0f);
            set_pixel(point, line_color);
        }
    }
    else
    {
        if (dy >= 0)
        {
            x  = static_cast<int>(x1);
            y  = static_cast<int>(y1);
            ye = static_cast<int>(y2);
        }
        else
        {
            x  = static_cast<int>(x2);
            y  = static_cast<int>(y2);
            ye = static_cast<int>(y1);
        }
        Eigen::Vector3f point = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 1.0f);
        set_pixel(point, line_color);

        for (i = 0; y < ye; i++)
        {
            y = y + 1;
            if (py <= 0)
            {
                py = py + 2 * dx1;
            }
            else
            {
                if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0))
                {
                    x = x + 1;
                }
                else
                {
                    x = x - 1;
                }
                py = py + 2 * (dx1 - dy1);
            }
            //            delay(0);
            Eigen::Vector3f point = Eigen::Vector3f(static_cast<float>(x), static_cast<float>(y), 1.0f);
            set_pixel(point, line_color);
        }
    }
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

void rst::rasterizer::draw(rst::pos_buf_id pos_buffer, rst::ind_buf_id ind_buffer, rst::Primitive type)
{
    if (type != rst::Primitive::Triangle)
    {
        throw std::runtime_error("Drawing primitives other than triangle is not implemented yet!");
    }
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];

    float f1 = (100.f - 0.1f) / 2.0f;
    float f2 = (100.f + 0.1f) / 2.0f;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;

        Eigen::Vector4f v[] = { mvp * to_vec4(buf[i[0]], 1.0f), mvp * to_vec4(buf[i[1]], 1.0f), mvp * to_vec4(buf[i[2]], 1.0f) };

        // 透视除法
        for (auto& vec : v)
        {
            vec /= vec.w();
        }

        // 将NDC坐标转换为屏幕坐标[0,w][0,h]，不需要对w进行操作
        for (auto& vert : v)
        {
            vert.x() = 0.5f * width * (vert.x() + 1.0f);
            vert.y() = 0.5f * height * (vert.y() + 1.0f);
            vert.z() = vert.z() * f1 + f2; // z坐标暂时没用，三角形是2D的
        }

        // 设置三角形的三个顶点（屏幕坐标）
        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
        }

        // 设置顶点颜色，暂时用不到
        t.setColor(0, 255.0, 0.0, 0.0);
        t.setColor(1, 0.0, 255.0, 0.0);
        t.setColor(2, 0.0, 0.0, 255.0);

        rasterize_wireframe(t);
    }
}

void rst::rasterizer::rasterize_wireframe(const Triangle& t)
{
    // 绘制三角形的三条边
    draw_line(t.c(), t.a());
    draw_line(t.c(), t.b());
    draw_line(t.b(), t.a());
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f { 0, 0, 0 });
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    // old index: auto ind = point.y() + point.x() * width;
    // 如果像素坐标超出屏幕大小，直接返回不绘制该像素
    // if (point.x() < 0 || point.x() >= width || point.y() < 0 || point.y() >= height)
    // {
    //     return;
    // }

    auto ind = static_cast<int>((height - point.y()) * width + point.x());
    if (ind >= width * height || ind < 0)
    {
        std::cerr << "Index is error for pixels\n";
        return;
    }

    frame_buf[ind] = color;
}
