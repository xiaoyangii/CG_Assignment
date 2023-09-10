// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <tuple>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v3 = { 
        x * 1.0f,
        y * 1.0f,
        1.0f
    };
    // 计算三角形的两个边向量
    Vector3f e1 = v3 - _v[0];
    Vector3f e2 = v3 - _v[1];
    Vector3f e3 = v3 - _v[2];

    // 利用Eigen的库 叉乘计算法向量
    bool normal_vector1 = e1.cross(e2).z() > 0;
    bool normal_vector2 = e2.cross(e3).z() > 0;
    bool normal_vector3 = e3.cross(e1).z() > 0;
    return (normal_vector1*normal_vector2 > 0 && normal_vector2*normal_vector3 > 0);
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    float min_x, max_x, min_y, max_y;
    min_x = max_x = 0.0f;
    min_y = max_y = 0.0f;
    for (int i = 0;i < 3;i++) {
        min_x = std::min(v[i].x(), min_x);
        max_x = std::max(v[i].x(), max_x);
        min_y = std::min(v[i].y(), min_y);
        max_y = std::max(v[i].y(), max_y);
    }

    // If so, use the following code to get the interpolated z value.
    //float alpha, beta, gamma;
    //std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    // iterate through the pixel

    // Define the super-sampling level
    //int superSamplingLevel = 2;
    int superSamplingLevel = 2;
    float superSamplingInv = 1.0f / superSamplingLevel;
    int superSamplingTotal = superSamplingLevel * superSamplingLevel;


    for (int x = min_y; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {

            Vector3f accumulatedColor(0.0f, 0.0f, 0.0f);
            float accumulatedDepth = 0.0f;

            // Iterate through the sub-pixels for super-sampling
            for (int sx = 0; sx < superSamplingLevel; sx++) {
                for (int sy = 0; sy < superSamplingLevel; sy++) {
                    float subPixel_x = x + (sx + 0.5f) / superSamplingInv; // superSamplingLevel
                    float subPixel_y = y + (sy + 0.5f) / superSamplingInv; // superSamplingLevel

                    // Check if the current sub-pixel is inside the triangle
                    if (insideTriangle(subPixel_x, subPixel_y, t.v)) {
                        // Calculate the interpolated z value
                        float alpha, beta, gamma;
                        std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // Get the color of the triangle
                        Vector3f pixelColor = t.getColor();

                        // Accumulate the color and depth for super-sampling
                        accumulatedColor += pixelColor;
                        accumulatedDepth += z_interpolated;
                    }
                }
            }

            // Calculate the final color by averaging the super-sampled colors
            // accumulatedColor /= (superSamplingLevel * superSamplingLevel);
            // accumulatedDepth /= (superSamplingLevel * superSamplingLevel);
            accumulatedColor /= superSamplingTotal;
            accumulatedDepth /= superSamplingTotal;
            int index = get_index(x, y);

            // Get the current depth from the depth buffer
            float currentDepth = depth_buf[index];

            // Check if the current pixel is closer to the camera
            if (accumulatedDepth <= currentDepth) {
                // Update the depth buffer
                depth_buf[index] = accumulatedDepth;

                // Set the current pixel (use the set_pixel function) to the final color
                set_pixel(Vector3f(x, y, depth_buf[index]), accumulatedColor);
            }
        }
    }
    

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
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
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
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on