#include "fitline.h"
#include "ransac.h"
#include <cstdlib>
#include <cstdio>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>
#define PI (3.1415926535897932346f)

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

        const std::string calib_data_path = data_root + "/calib_data.txt";

        //========Read calibration data========//

        double id, score, x = 0.0, y = 0.0, z = 0.0, roll = 0.0, yaw = 0.0, pitch = 0.0;
        double x_0 = 0.0, y_0 = 0.0, z_0 = 0.0, roll_0 = 0.0, yaw_0 = 0.0, pitch_0 = 0.0;

        ifstream calib_FileA(calib_data_path.c_str());
        if (!calib_FileA.is_open())
        {
                cerr << "Cannot open " << calib_data_path << endl;
                return -1;
        }

        int filecount = 0;
        while (calib_FileA >> id >> score >> x >> y >> z >> roll >> pitch >> yaw)
        {
                filecount++;
        }
        calib_FileA.close();

        if (filecount <= 0)
        {
                cerr << "No valid calibration records in " << calib_data_path << endl;
                return -1;
        }

        cout << "Read " << filecount << " data" << endl;

        ifstream calib_File(calib_data_path.c_str());
        if (!calib_File.is_open())
        {
                cerr << "Cannot reopen " << calib_data_path << endl;
                return -1;
        }
        int count = 0;

        std::vector<Point2D32f> points_x(filecount);
        std::vector<Point2D32f> points_y(filecount);
        std::vector<Point2D32f> points_z(filecount);

        std::vector<Point2D32f> points_roll(filecount);
        std::vector<Point2D32f> points_pitch(filecount);
        std::vector<Point2D32f> points_yaw(filecount);

        while ((calib_File >> id >> score >> x >> y >> z >> roll >> pitch >> yaw) && count < filecount)
        {
                points_x[count].x = id;
                points_x[count].y = x;

                points_y[count].x = id;
                points_y[count].y = y;

                points_z[count].x = id;
                points_z[count].y = z;

                points_roll[count].x = id;
                points_roll[count].y = roll;

                points_pitch[count].x = id;
                points_pitch[count].y = pitch;

                points_yaw[count].x = id;
                points_yaw[count].y = yaw;

                x_0+=x;
                y_0+=y;
                z_0+=z;
                roll_0+=roll;
                pitch_0+=pitch;
                yaw_0+=yaw;
                
                count++;
        }
        x_0=x_0/count;
        y_0=y_0/count;
        z_0=z_0/count;
        roll_0=roll_0/count;
        pitch_0=pitch_0/count;
        yaw_0=yaw_0/count;

        // 当前版本 RANSAC 拟合逻辑被注释，输出均值作为最终参数。
        x = x_0;
        y = y_0;
        z = z_0;
        roll = roll_0;
        pitch = pitch_0;
        yaw = yaw_0;

     /*   float lines[4] = {0.0}; //line parameters

        int numForEstimate = 5;
        float successProbability = 0.9999f;
        float maxOutliersPercentage = 0.9; //(float)outlierCnt/COUNT; 0.9
        float a, b;

        Ransac(points_x, filecount, lines, numForEstimate, successProbability, maxOutliersPercentage);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        printf("x ransac fit(including outliers): a: %f  x: %f\n", a, b);
        x = b;

        Ransac(points_y, filecount, lines, numForEstimate, successProbability, maxOutliersPercentage);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        printf("y ransac fit(including outliers): a: %f  y: %f\n", a, b);
        y = b;

        Ransac(points_z, filecount, lines, numForEstimate, successProbability, maxOutliersPercentage);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        printf("z ransac fit(including outliers): a: %f  z: %f\n", a, b);
        z = b;

        Ransac(points_roll, filecount, lines, numForEstimate, successProbability, maxOutliersPercentage);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        printf("roll ransac fit(including outliers): a: %f  roll: %f\n", a, b);
        roll = b;

        Ransac(points_pitch, filecount, lines, numForEstimate, successProbability, maxOutliersPercentage);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        printf("pitch ransac fit(including outliers): a: %f  pitch: %f\n", a, b);
        pitch = b;

        Ransac(points_yaw, filecount, lines, numForEstimate, successProbability, maxOutliersPercentage);
        a = lines[1] / lines[0];
        b = lines[3] - a * lines[2];
        printf("yaw ransac fit(including outliers): a: %f  yaw: %f\n", a, b);
        yaw = b;
*/
        calib_File.close();

        cout << endl;
        cout << "RANSAC Result (x,y,z,roll,pitch,yaw): " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << endl;

        Eigen::Matrix<double, 3, 1> T;

        //EulerAngles to RotationMatrix
        ::Eigen::Vector3d ea0(yaw, pitch, roll);
        ::Eigen::Matrix3d R;
        R = ::Eigen::AngleAxisd(ea0[0], ::Eigen::Vector3d::UnitZ()) * ::Eigen::AngleAxisd(ea0[1], ::Eigen::Vector3d::UnitY()) * ::Eigen::AngleAxisd(ea0[2], ::Eigen::Vector3d::UnitX());

        // cout << R << endl << endl;

        T << x, y, z;
        Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
        tf.block(0, 0, 3, 3) = R;
        tf.block(0, 3, 3, 1) = T;

        cout << "----------------------------------" << endl;
        cout << "Result Matrix:" << endl;
        cout << tf << endl;

        return 0;
}
