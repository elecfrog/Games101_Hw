#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float rotateDegree = rotation_angle / 180.0f * MY_PI;
    model << cos(rotateDegree), -sin(rotateDegree), 0, 0,
        sin(rotateDegree), cos(rotateDegree), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f init_model_matrix()
{
    Eigen::Matrix4f init_model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    init_model<< 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return init_model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    Eigen::Matrix4f orthMat = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f persMat = Eigen::Matrix4f::Identity();


    float fovDegree = eye_fov / 180.0f * MY_PI;
    float n = -zNear;
    float f = -zFar;
    float t = tan(fovDegree / 2) * abs(n);
    float r = t * aspect_ratio;
    float b = -t;
    float l = -r;

    Eigen::Matrix4f pers2Orth = Eigen::Matrix4f::Identity();
    pers2Orth << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    Eigen::Matrix4f transOfOrth = Eigen::Matrix4f::Identity();
    transOfOrth << 1, 0, 0, -(l + r) / 2,
        0, 1, 0, -(b + t) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f scalOfOrth = Eigen::Matrix4f::Identity();
    scalOfOrth << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;

    orthMat = scalOfOrth * transOfOrth;
    persMat = orthMat * pers2Orth;
    projection = persMat;


    return projection;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {

    // std::cout << axis << std::endl;

    Eigen::Matrix4f ans = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f one;
    one << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    float alpha = angle / 180 * MY_PI;
    float cs = cos(alpha), sn = sin(alpha);
    float x = axis[0], y = axis[1], z = axis[2];
    Eigen::Matrix3f cross;
    cross << 0, -z, y, z, 0, -x, -y, x, 0;

    Eigen::Matrix3f tmp = cs * one + (1 - cs) * axis * axis.transpose() + sn * cross;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (i < 3 && j < 3)
                ans(i, j) = tmp(i, j);
            else if (i == 3 && j == 3)
                ans(i, j) = 1;
            else
                ans(i, j) = 0;
        }
    }
    return ans;
}

int main(int argc, const char** argv)
{
    float angle = 0;

    // 命令行调试参数设置
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
    }

    // 定义了一个确定宽高参数的屏幕
    rst::rasterizer r(700, 700);

    // 人眼（相机）的坐标位置
    Eigen::Vector3f eye_pos = {0, 0, 5};

    // 三角形三个定点的坐标
    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    // 这三个点的索引值
    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    // 通过loadposition与load_indices将坐标点和索引值赋给光栅化器（rasterizer)
    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    Eigen::Vector3f axis = { 2, 0, -2 };
    axis.normalize();

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    r.set_model(init_model_matrix());
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
            r.set_model(get_model_matrix(angle));
        }
        else if (key == 'd') {
            angle -= 10;
            r.set_model(get_model_matrix(angle));
        }
        if (key == 'q') {
            angle += 10;
            r.set_model(get_rotation(axis, angle));
        }
        else if (key == 'e') {
            angle -= 10;
            r.set_model(get_rotation(axis, angle));
        }
    }

    return 0;
}
