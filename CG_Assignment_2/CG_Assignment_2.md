# CG_Assignment_2

## **完成情况**

- [x] 正确地提交所有必须的文件，且代码能够编译运行
- [x] De Casteljau 算法：对于给定的控制点，你的代码能够产生正确的 Bézier 曲线
- [x] 实现对 Bézier 曲线的反走样

## **recursive_bezier**

```c++
cv::Point2f recursive_bezier(const std::vector<cv::Point2f>& control_points, float t)
{
    if (control_points.size() == 1)
    {
        return control_points[0];
    }
    else
    {
        std::vector<cv::Point2f> new_control_points;
        for (size_t i = 0; i < control_points.size() - 1; ++i)
        {
            cv::Point2f new_point = (1 - t) * control_points[i] + t * control_points[i + 1];
            new_control_points.push_back(new_point);
        }
        return recursive_bezier(new_control_points, t);
    }

    //return cv::Point2f();

}
```

`recursive_bezier` 函数实现了 de Casteljau 递归算法，具体功能如下：

- **输入参数**：
  - `control_points`：一个包含控制点的 `std::vector<cv::Point2f>`。这些控制点定义了 Bézier 曲线的形状
  - `t`：一个浮点数，表示要计算 Bézier 曲线上的点的位置，范围在 0 到 1 之间
- **递归计算**：`recursive_bezier` 函数使用 de Casteljau 算法递归地计算 Bézier 曲线上的点
- **输出结果**：最终，它返回 Bézier 曲线上给定 `t` 处的点坐标（`cv::Point2f` 类型）

## **bezier**

```c++
void bezier(const std::vector<cv::Point2f>& control_points, cv::Mat& window)
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    // Iterate through all t = 0 to t = 1 with small steps
    for (double t = 0.0; t <= 1.0; t += 0.001)
    {
        // Call recursive_bezier to get the point on the curve at t
        cv::Point2f point = recursive_bezier(control_points, t);

        // Calculate integer pixel coordinates
        int x = static_cast<int>(point.x);
        int y = static_cast<int>(point.y);

        // Calculate fractional parts of coordinates
        double xf = point.x - x;
        double yf = point.y - y;

        // Perform 2x2 anti-aliasing
        for (int dx = 0; dx <= 1; ++dx)
        {
            for (int dy = 0; dy <= 1; ++dy)
            {
                int px = x + dx;
                int py = y + dy;

                if (px >= 0 && px < window.cols && py >= 0 && py < window.rows)
                {
                    double weight = (1 - std::abs(px - point.x)) * (1 - std::abs(py - point.y));
                    window.at<cv::Vec3b>(py, px)[1] += static_cast<uchar>(weight * 255);
                }
            }
        }
    }
}
```

`bezier` 函数的目的是绘制 Bézier 曲线。具体功能如下：

- **输入参数**：
  - `control_points`：一个包含控制点的 `std::vector<cv::Point2f>`。这些控制点定义了 Bézier 曲线的形状
  - `window`：一个 OpenCV 的 `cv::Mat` 对象，代表绘图窗口。曲线将绘制在这个窗口上
- **迭代计算**：`bezier` 函数通过迭代来计算曲线上的点。它从 `t` 值为 0 到 1 的范围内，以小步长递增的方式遍历
- **计算点位置**：对于每个 `t` 值，它调用了 `recursive_bezier` 函数来计算在 Bézier 曲线上的点
- **绘制曲线**：然后，它使用得到的点坐标将一条绿色线绘制在 `window` 上。你可以通过 `window.at<cv::Vec3b>(point.y, point.x)[1] = 255` 来将绿色线绘制在屏幕上