#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926;

// 视图变换
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 
        0, 1, 0, -eye_pos[1], 
        0, 0, 1,-eye_pos[2], 
        0, 0, 0, 1;

    view = translate * view;

    return view;
}
// 输入一个旋转角度，进行模型变换
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

// 初始化一个单位矩阵
Eigen::Matrix4f init_model_matrix()
{
    Eigen::Matrix4f init_model = Eigen::Matrix4f::Identity();

    init_model<< 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    return init_model;
}

// 透视投影变换
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float n = -zNear, f = -zFar;
    float t = tan((eye_fov / 180.0f * MY_PI)/ 2) * abs(n);
    float b = -t;
    float r = t * aspect_ratio;
    float l = -r;

    Eigen::Matrix4f orth_mat = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f pers_mat = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f p2o_mat = Eigen::Matrix4f::Identity();
    p2o_mat << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;

    Eigen::Matrix4f o_transl = Eigen::Matrix4f::Identity();
    o_transl << 1, 0, 0, -(l + r) / 2,
        0, 1, 0, -(b + t) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f o_linear = Eigen::Matrix4f::Identity();
    o_linear << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;

    //orth_mat = o_linear * o_transl;
    //pers_mat = o_linear * o_transl * p2o_mat;
    projection = o_linear * o_transl * p2o_mat;

    return projection;
}

Eigen::Matrix4f get_rotation(Eigen::Vector3f axis, float angle) {

    // std::cout << axis << std::endl;

    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    //one << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    float alpha = angle / 180 * MY_PI;
    float cos_alpha = cos(alpha), sin_alpha = sin(alpha);
    float n_x = axis[0], n_y = axis[1], n_z = axis[2];
    Eigen::Matrix3f N;
    N << 0, -n_z, n_y, 
        n_z, 0, -n_x, 
        -n_y, n_x, 0;

    Eigen::Matrix3f tr = cos_alpha * I + (1 - cos_alpha) * axis * axis.transpose() + sin_alpha * N;

    // TODO: 可以写一个Matrix3f to 4f 的函数
    model << tr(0, 0), tr(0, 1), tr(0, 2), 0,
        tr(1, 0), tr(1, 1), tr(1, 2), 0,
        tr(2, 0), tr(2, 1), tr(2, 2), 0,
        0, 0, 0, 1;
    return model;
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
        
        // 清除整个显示屏幕
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
