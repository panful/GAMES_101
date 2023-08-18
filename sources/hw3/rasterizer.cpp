//
// Created by goksu on 4/6/19.
//

#include "rasterizer.hpp"
#include <algorithm>
#include <math.h>
#include <opencv2/opencv.hpp>

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

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return { id };
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector4f* _v)
{
    Vector3f v[3];
    for (int i = 0; i < 3; i++)
    {
        v[i] = { _v[i].x(), _v[i].y(), 1.0 };
    }

    Vector3f f0, f1, f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);

    Vector3f p((float)x, (float)y, 1.f);
    if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
    {
        return true;
    }
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v)
{
    float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y())
        / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
    float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y())
        / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
    float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y())
        / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
    return { c1, c2, c3 };
}

void rst::rasterizer::draw(std::vector<Triangle*>& TriangleList)
{
    float f1 = (50 - 0.1f) / 2.0f;
    float f2 = (50 + 0.1f) / 2.0f;

    Eigen::Matrix4f mvp = projection * view * model;
    for (const auto& t : TriangleList)
    {
        Triangle newtri = *t;

        std::array<Eigen::Vector4f, 3> mm { (view * model * t->v[0]), (view * model * t->v[1]), (view * model * t->v[2]) };

        std::array<Eigen::Vector3f, 3> viewspace_pos;

        std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) { return v.template head<3>(); });

        Eigen::Vector4f v[] = { mvp * t->v[0], mvp * t->v[1], mvp * t->v[2] };
        // Homogeneous division
        for (auto& vec : v)
        {
            vec.x() /= vec.w();
            vec.y() /= vec.w();
            vec.z() /= vec.w();
        }

        Eigen::Matrix4f inv_trans = (view * model).inverse().transpose();
        Eigen::Vector4f n[]
            = { inv_trans * to_vec4(t->normal[0], 0.0f), inv_trans * to_vec4(t->normal[1], 0.0f), inv_trans * to_vec4(t->normal[2], 0.0f) };

        // Viewport transformation
        for (auto& vert : v)
        {
            vert.x() = 0.5f * width * (vert.x() + 1.0f);
            vert.y() = 0.5f * height * (vert.y() + 1.0f);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            // screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            // view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        newtri.setColor(0, 148, 121.0, 92.0);
        newtri.setColor(1, 148, 121.0, 92.0);
        newtri.setColor(2, 148, 121.0, 92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(
    float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(
    float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

// Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& view_pos)
{
    auto min_x = static_cast<int>(std::min({ t.a().x(), t.b().x(), t.c().x() }));
    auto min_y = static_cast<int>(std::min({ t.a().y(), t.b().y(), t.c().y() }));
    auto max_x = static_cast<int>(std::max({ t.a().x(), t.b().x(), t.c().x() }));
    auto max_y = static_cast<int>(std::max({ t.a().y(), t.b().y(), t.c().y() }));

    for (int x = min_x; x < max_x; x++)
    {
        for (int y = min_y; y < max_y; y++)
        {
            if (insideTriangle(x, y, t.v))
            {
                auto [alpha, beta, gamma] = computeBarycentric2D((float)x, (float)y, t.v);
                auto uv                   = interpolate(alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1.f);

                Eigen::Vector3f v0 = t.v[0].head<3>();
                Eigen::Vector3f v1 = t.v[1].head<3>();
                Eigen::Vector3f v2 = t.v[2].head<3>();

                auto pos = interpolate(alpha, beta, gamma, v0, v1, v2, 1.f);

                Eigen::Vector3f color = texture.value().getColor(uv.x(), uv.y());

                set_pixel(Eigen::Vector2i(x, y), color, pos.z());
            }
        }
    }

    // TODO: From your HW3, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture :
    // nullptr); Use: payload.view_pos = interpolated_shadingcoords; Use: Instead of passing the triangle's color directly to the frame buffer, pass
    // the color to the shaders first to get the final color; Use: auto pixel_color = fragment_shader(payload);
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

    texture = std::nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height - y) * width + x;
}

void rst::rasterizer::set_pixel(const Vector2i& point, const Eigen::Vector3f& color, float depth)
{
    int ind = point.y() * width + point.x();
    if (ind < 0 || ind >= frame_buf.size())
    {
        return;
    }

    if (depth < depth_buf[ind])
    {
        depth_buf[ind] = depth;
        frame_buf[ind] = color;
    }
}

void rst::rasterizer::set_vertex_shader(std::function<Eigen::Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(std::function<Eigen::Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}
