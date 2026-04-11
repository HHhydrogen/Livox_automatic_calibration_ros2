/*
The mapping algorithm is an advanced implementation of the following open source project:
  [blam](https://github.com/erik-nelson/blam). 
Modifier: livox               dev@livoxtech.com


Copyright (c) 2015, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <memory>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_utils/Transform3.h>
#include <point_cloud_mapper/PointCloudMapper.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;
namespace gu = geometry_utils;

#define PI (3.1415926535897932346f)

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress(double percentage)
{
    int val = (int)(percentage * 100);
    int lpad = (int)(percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}

std::string itos(int i)
{
    std::stringstream s;
    s << i;
    return s.str();
}

int countFrames(const std::string& frames_dir, int first_frame)
{
    int count = 0;
    while (true)
    {
        const std::string frame_path = frames_dir + "/" + itos(first_frame + count) + ".pcd";
        if (access(frame_path.c_str(), F_OK) != 0)
        {
            break;
        }
        ++count;
    }
    return count;
}

int main(int argc, char** argv)
{
    std::cout << "开始标定..." << std::endl;

    std::string data_root;
    if (argc > 1)
    {
        data_root = argv[1];
    }
    else
    {
        try
        {
            data_root = ament_index_cpp::get_package_share_directory("livox_automatic_calibration_ros2") + "/data";
        }
        catch (const std::exception&)
        {
            data_root = "../data";
        }
    }

    const std::string frames_dir = data_root + "/Target_LiDAR_Frames";
    const std::string map_path = data_root + "/H-LiDAR-Map-data/H_LiDAR_Map.pcd";
    const std::string t_matrix_path = data_root + "/T_Matrix.txt";
    const std::string init_matrix_path = data_root + "/Init_Matrix.txt";
    const std::string calib_output_path = data_root + "/calib_data.txt";
    setenv("LIVOX_CALIB_DATA_DIR", data_root.c_str(), 1);

    //================== 步骤1：读取基准地图 =====================//

    std::cout << "正在读取基准地图..." << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr H_LiDAR_Map(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *H_LiDAR_Map) == -1)
    {
        PCL_ERROR("读取基准地图失败\n");
        return (-1);
    }
    std::cout << "已加载基准地图点数：" << H_LiDAR_Map->size() << std::endl;

    // 将地图点云写入地图容器
    PointCloudMapper maps;
    maps.Initialize();
    pcl::PointCloud<pcl::PointXYZ>::Ptr unused(new pcl::PointCloud<pcl::PointXYZ>);
    maps.InsertPoints(H_LiDAR_Map, unused.get());

    //================== 步骤2：读取基准轨迹与外参初值 =====================//

    ifstream T_Mat_File(t_matrix_path.c_str());
    Eigen::Matrix4f T_Matrix = Eigen::Matrix4f::Identity();

    ifstream initFile(init_matrix_path.c_str());
    if (!T_Mat_File.is_open())
    {
        std::cerr << "无法打开轨迹文件：" << t_matrix_path << std::endl;
        return -1;
    }
    if (!initFile.is_open())
    {
        std::cerr << "无法打开初值矩阵文件：" << init_matrix_path << std::endl;
        return -1;
    }

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f init_guess_0 = Eigen::Matrix4f::Identity();

    for (int mat_i = 0; mat_i != 4; mat_i++)
    {
        for (int mat_j = 0; mat_j != 4; mat_j++)
        {
            initFile >> init_guess(mat_i, mat_j);
        }
    }

    init_guess_0 = init_guess;
    //================== 步骤3：读取目标雷达帧数据 =====================//

    int framenumbers = countFrames(frames_dir, 100000);
    if (framenumbers <= 0)
    {
        std::cerr << "未找到连续编号的 PCD 帧：" << frames_dir << "（应从 100000.pcd 开始）" << std::endl;
        return -1;
    }
    int frame_count = 100000;
    int cframe_count = 0;
    cout << "已加载目标雷达帧数：" << framenumbers << endl;

    //=================================
    // 配置 ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr ICP_output_cloud(new pcl::PointCloud<pcl::PointXYZ>); // 占位输出，接口必需
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setTransformationEpsilon(0.0000000001); // 变换收敛阈值
    icp.setMaxCorrespondenceDistance(10);
    icp.setMaximumIterations(35);
    icp.setRANSACIterations(0);
    icp.setMaximumOptimizerIterations(50); // 默认值为 20

    //=================================
    // 配置可视化（默认关闭，避免无图形环境崩溃）
    const char* viewer_flag = std::getenv("LIVOX_CALIB_ENABLE_VIEWER");
    const bool enable_viewer = (viewer_flag != nullptr && std::string(viewer_flag) == "1");
    std::unique_ptr<pcl::visualization::PCLVisualizer> viewer_final;
    if (enable_viewer)
    {
        viewer_final.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer_final->setBackgroundColor(0, 0, 0);
    }
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> map_color(H_LiDAR_Map, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> match_color(H_LiDAR_Map, 0, 255, 0);

    if (enable_viewer)
    {
        viewer_final->addPointCloud<pcl::PointXYZ>(H_LiDAR_Map, map_color, "target cloud");
        viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
        viewer_final->addPointCloud<pcl::PointXYZ>(H_LiDAR_Map, match_color, "match cloud"); // 显示匹配点云
    }

    //=================================
    // 准备保存标定数据

    ofstream fout(calib_output_path.c_str());
    if (!fout.is_open())
    {
        std::cerr << "无法写入标定输出文件：" << calib_output_path << std::endl;
        return -1;
    }
    fout.setf(ios::fixed, ios::floatfield);
    fout.precision(7);

    //=================================
    //              开始处理
    //=================================
    while (true)
    {
        if (enable_viewer && viewer_final->wasStopped())
        {
            break;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr frames(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(frames_dir + "/" + itos(frame_count) + ".pcd", *frames) == -1)
        {
            PCL_ERROR("读取目标雷达帧失败\n");
            return (-1);
        }
        //std::cout << "当前帧点数：" << frames->size() << std::endl;

        // 读取基准轨迹矩阵
        for (int mat_i = 0; mat_i != 4; mat_i++)
        {
            for (int mat_j = 0; mat_j != 4; mat_j++)
            {
                if (!(T_Mat_File >> T_Matrix(mat_i, mat_j)))
                {
                    std::cerr << "T_Matrix.txt 数据不足，帧号：" << frame_count << std::endl;
                    return -1;
                }
            }
        }

        //================== 步骤4：执行标定 =====================//

        pcl::PointCloud<pcl::PointXYZ>::Ptr trans_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*frames, *trans_output_cloud, init_guess); // Tiny_T * init_guess 用于迭代更新
        pcl::transformPointCloud(*trans_output_cloud, *final_output_cloud, T_Matrix);

        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbors_L(new pcl::PointCloud<pcl::PointXYZ>); // 邻域匹配点
        maps.ApproxNearestNeighbors(*final_output_cloud, neighbors_L.get());

        // 轨迹矩阵求逆
        
        gu::Transform3 inverse_mat;
        inverse_mat.translation = gu::Vec3(T_Matrix(0, 3), T_Matrix(1, 3), T_Matrix(2, 3));
        inverse_mat.rotation = gu::Rot3(T_Matrix(0, 0), T_Matrix(0, 1), T_Matrix(0, 2),
                                        T_Matrix(1, 0), T_Matrix(1, 1), T_Matrix(1, 2),
                                        T_Matrix(2, 0), T_Matrix(2, 1), T_Matrix(2, 2));

        const gu::Transform3 estimate = gu::PoseInverse(inverse_mat); // 根据配置参数得到积分位姿
        const Eigen::Matrix<double, 3, 3> T_Matrix_Inverse_R = estimate.rotation.Eigen();
        const Eigen::Matrix<double, 3, 1> T_Matrix_Inverse_T = estimate.translation.Eigen();

        Eigen::Matrix4d T_Matrix_Inverse;
        T_Matrix_Inverse.block(0, 0, 3, 3) = T_Matrix_Inverse_R;
        T_Matrix_Inverse.block(0, 3, 3, 1) = T_Matrix_Inverse_T;

        //====== 核心步骤：局部 ICP ======//
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr neighbors_trans(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*neighbors_L, *neighbors_trans, T_Matrix_Inverse);

        // 执行 ICP，得到本帧微小变换 Tiny_T
        icp.setInputSource(trans_output_cloud); // 当前帧
        icp.setInputTarget(neighbors_trans);    // 当前帧邻域地图
        icp.align(*ICP_output_cloud);
        const Eigen::Matrix4f Tiny_T = icp.getFinalTransformation();

        //std::cout << "匹配得分: " << icp.getFitnessScore() << std::endl;

        if (icp.getFitnessScore() > 1)
        {
            //std::cout<<"匹配质量差，跳过该帧"<<std::endl;
            init_guess = init_guess_0;
            //continue;
        }
        else
        {
            Eigen::Matrix4f Final_Calib_T = Eigen::Matrix4f::Identity();

            Final_Calib_T = Tiny_T * init_guess;
            //std::cout << Final_Calib_T.matrix() << std::endl;
            init_guess = Final_Calib_T;

            //===== 输出欧拉角 =====//
            gu::Vector3 EulerAngle;
            gu::Rot3 rot_mat(Final_Calib_T(0, 0), Final_Calib_T(0, 1), Final_Calib_T(0, 2),
                             Final_Calib_T(1, 0), Final_Calib_T(1, 1), Final_Calib_T(1, 2),
                             Final_Calib_T(2, 0), Final_Calib_T(2, 1), Final_Calib_T(2, 2));
            EulerAngle = rot_mat.GetEulerZYX();
            const Eigen::Matrix<double, 3, 1> EulerAngle_T = EulerAngle.Eigen();
            //std::cout<<"欧拉角: "<<EulerAngle_T(0,0)<<" "<<EulerAngle_T(1,0)<<" "<<EulerAngle_T(2,0)<<std::endl;

            if (icp.getFitnessScore() < 0.1)
                fout << frame_count - 100000 << " " << icp.getFitnessScore() << " " << Final_Calib_T(0, 3) << " " << Final_Calib_T(1, 3) << " " << Final_Calib_T(2, 3) << " " << EulerAngle_T(0, 0) << " " << EulerAngle_T(1, 0) << " " << EulerAngle_T(2, 0) << endl; // x, y, z, roll, pitch, yaw
        }

        frame_count++;
        cframe_count++;

        printProgress((double)cframe_count / (double)framenumbers);
        if (enable_viewer)
        {
            viewer_final->updatePointCloud<pcl::PointXYZ>(final_output_cloud, match_color, "match cloud");
            viewer_final->spinOnce(10);
        }

        if (cframe_count == framenumbers)
        {
            std::cout << "\n标定匹配完成。" << std::endl;
            break;
        }
    }

    return 0;
}
