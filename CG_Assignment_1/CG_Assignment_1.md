# CG_Assignment_1

**完成情况**

- [x] 正确地提交所有必须的文件，且代码能够编译运行。
- [x] 正确测试点是否在三角形内。
- [x] 正确实现三角形栅格化算法。
- [x] 正确实现 z-buffer 算法, 将三角形按顺序画在屏幕上。
- [x] 用 super-sampling 处理 Anti-aliasing。

## 一、输入坐标(x, y)与三角形顶点，测试坐标是否在三角形内

### 1.static bool insideTriangle() 函数代码

```
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
```

### 2.主要功能

1. **边向量计算**：计算从点 `(x, y)` 到三角形各个顶点的三个边向量 `e1`、`e2` 和 `e3`，这些向量表示了点到各个顶点的距离和方向。 
2. **法向量计算**：利用Eigen库的叉乘功能，计算了三个边向量之间的叉乘结果，从而得到法向量。 
3. **法向量方向比较**：检查法向量的 `z` 分量是否大于0。如果法向量的 `z` 分量大于0，表示点 `(x, y)` 位于三角形的前半部分，否则位于后半部分。 
4. **综合判断**：如果点 `(x, y)` 位于三角形内部，那么三个法向量的方向比较结果应该都是正的，即 `normal_vector1 * normal_vector2 > 0 && normal_vector2 * normal_vector3 > 0`。 
5. **返回结果**：函数返回一个布尔值，表示点 `(x, y)` 是否在三角形内部。

## 二、三角形光栅化和超采样抗锯齿处理

### 1.rasterize_triangle() 函数代码

```
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

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    // iterate through the pixel

    // 定义 super-sampling level eg.2*2 (4*4 效果更好)
    // int superSamplingLevel = 2;
    int superSamplingLevel = 2;
    float superSamplingInv = 1.0f / superSamplingLevel;
    int superSamplingTotal = superSamplingLevel * superSamplingLevel;


    for (int x = min_y; x < max_x; x++) {
        for (int y = min_y; y < max_y; y++) {

            Vector3f accumulatedColor(0.0f, 0.0f, 0.0f); //累积颜色值
            float accumulatedDepth = 0.0f; //累计深度值

            // 遍历子像素以进行 super-sampling
            for (int sx = 0; sx < superSamplingLevel; sx++) {
                for (int sy = 0; sy < superSamplingLevel; sy++) {
                    float subPixel_x = x + (sx + 0.5f) / superSamplingInv; // superSamplingLevel
                    float subPixel_y = y + (sy + 0.5f) / superSamplingInv; // superSamplingLevel

                    // 检查当前子像素是否在三角形内
                    if (insideTriangle(subPixel_x, subPixel_y, t.v)) {
                        // Calculate the interpolated z value
                        float alpha, beta, gamma;
                        std::tie(alpha, beta, gamma) = computeBarycentric2D(x, y, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // 获得三角形颜色
                        Vector3f pixelColor = t.getColor();

                        // 累积超级采样的颜色和深度
                        accumulatedColor += pixelColor;
                        accumulatedDepth += z_interpolated;
                    }
                }
            }

            // accumulatedColor /= (superSamplingLevel * superSamplingLevel);
            // accumulatedDepth /= (superSamplingLevel * superSamplingLevel);
            // 通过对超级采样的颜色求平均值来计算最终颜色
            accumulatedColor /= superSamplingTotal;
            accumulatedDepth /= superSamplingTotal;
            int index = get_index(x, y);

            // 从深度缓冲区获取当前深度
            float currentDepth = depth_buf[index];

            // 检查当前像素是否更靠近相机
            if (accumulatedDepth <= currentDepth) {
                // 更新深度缓冲区
                depth_buf[index] = accumulatedDepth;

                // 将当前像素 Set_pixel 设置为最终颜色
                set_pixel(Vector3f(x, y, depth_buf[index]), accumulatedColor);
            }
        }
    }
    

}
```



三角形光栅化器, z-buffer算法用于将三维三角形投影到屏幕空间并在屏幕上渲染。它还包括超采样(super-sampling)抗锯齿(Anti-aliasing)处理, 对每个像素进行 2 * 2 采样，以减少锯齿和提高图形渲染质量。

### 2.主要功能

1. **找到三角形的边界框**：首先，计算给定三角形在屏幕空间中的边界框。这是通过迭代三角形的顶点并找到最小和最大的x和y坐标来完成的。

2. **超采样抗锯齿**：在每个像素位置，使用超采样技术进行抗锯齿处理。它将当前像素划分为多个子像素，并在每个子像素上进行颜色和深度的采样。

3. **深度插值**：对于每个子像素，检查是否在三角形内。如果是，则进行深度插值以计算子像素的深度值。通过计算子像素在三角形内的重心坐标，然后使用重心坐标插值顶点深度

4. **颜色融合**：在超采样的子像素内，根据子像素在三角形内的位置和深度值，从三角形获取颜色信息，并对颜色进行累积。

5. **最终颜色计算**：在完成超采样抗锯齿处理后，将累积的颜色除以子像素的总数，获得最终的像素颜色。

6. **深度缓冲更新**：在得到最终颜色和深度后，判断当前像素的深度值是否更接近相机。如果是，更新深度缓冲区中的深度值。

7. **像素绘制**：最后使用 `set_pixel` 函数将最终的颜色设置到帧缓冲区中。

### 3.性能优化

​	进行了一些性能优化，包括减少不必要的循环迭代、重复计算的次数以及使用变量来减少计算。
