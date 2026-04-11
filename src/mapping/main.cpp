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



#include <blam_slam/BlamSlam.h>
#include "common.h"

// PCL 相关头文件
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/registration/gicp.h>

#include <geometry_utils/Transform3.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <cstdlib>
#include <sys/stat.h>
#include <unistd.h>
#include <memory>
#include <ament_index_cpp/get_package_share_directory.hpp>
using namespace std;

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

void printProgress (double percentage)
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf ("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush (stdout);
}

namespace gu = geometry_utils;

typedef pcl::POINT_TYPE PointT;
typedef pcl::PointCloud<PointT> PointCloud;

using namespace pcl::visualization;
BlamSlam LivoxSLAM;

long long frameNumber = 0;

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

    const std::string frames_dir = data_root + "/Base_LiDAR_Frames";
    const std::string t_matrix_path = data_root + "/T_Matrix.txt";
    const std::string map_output_dir = data_root + "/H-LiDAR-Map-data";
    const std::string map_output_path = map_output_dir + "/H_LiDAR_Map.pcd";
    setenv("LIVOX_CALIB_DATA_DIR", data_root.c_str(), 1);
  
    //================== 步骤1：读取基准雷达帧数据 =====================//
    int framenumbers = countFrames(frames_dir, 100000);
    if (framenumbers <= 0)
    {
        std::cerr << "未找到连续编号的 PCD 帧：" << frames_dir << "（应从 100000.pcd 开始）" << std::endl;
        return -1;
    }
    int frame_count = 100000;
    int cframe_count = 0;
    cout << "开始构建局部地图..." << endl;
    cout << "已加载基准雷达帧数：" << framenumbers << endl;

    ofstream fout(t_matrix_path.c_str());
    if (!fout.is_open())
    {
        std::cerr << "无法写入轨迹文件：" << t_matrix_path << std::endl;
        return -1;
    }

    const char* viewer_flag = std::getenv("LIVOX_CALIB_ENABLE_VIEWER");
    const bool enable_viewer = (viewer_flag != nullptr && std::string(viewer_flag) == "1");
    std::unique_ptr<pcl::visualization::CloudViewer> viewer;
    if (enable_viewer)
    {
        viewer.reset(new pcl::visualization::CloudViewer("点云可视化窗口"));
    }
    
    LivoxSLAM.Initialize();

    //================== 步骤2：构建子地图 =====================//

    while (true)
    {
        if (enable_viewer && viewer->wasStopped())
        {
            break;
        }

        while (frame_count < framenumbers + 100000)
        
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr frames(new pcl::PointCloud<pcl::PointXYZ>);
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(frames_dir + "/" + itos(frame_count) + ".pcd", *frames) == -1)
            {
                PCL_ERROR("读取 PCD 帧失败\n");
                return (-1);
            }
            
            std::vector<int> mapping;
            
            
            LivoxSLAM.ProcessPointCloudMessage(frames);

            const gu::Transform3 estimate = LivoxSLAM.localization_.GetIntegratedEstimate();
            const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
            const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();
            Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
            
            tf.block(0, 0, 3, 3) = R;
            tf.block(0, 3, 3, 1) = T;
            fout << tf.matrix() << endl;

            if (enable_viewer)
            {
                viewer->showCloud(LivoxSLAM.mapper_.map_data_);
            }
            
            frame_count++;
            cframe_count++;
           
            printProgress ((double)cframe_count/(double)framenumbers);
        }

        std::cout << "\n正在保存子地图..." << std::endl;
        mkdir(map_output_dir.c_str(), 0755);
        pcl::PCDWriter writer;
        writer.write(map_output_path, *LivoxSLAM.mapper_.map_data_);
        std::cout << "建图完成！" << std::endl;
        fout.close();
        break;
       
    }

    return 0;
}
