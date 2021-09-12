#include "radar_lidar_kf.hh"
#include <fstream>
#include <sstream>
#include "ground_truth_package.h"
#include "measurement_package.h"
#include "tools.h"
#include <vector>
#include <string>
#include <iostream>
int main(){
    // read all data
    // loop for all data, for each one, use
    // process_management(data)
    
    //read measurement data
    std::ifstream in_file("/home/ugv-yu/bryan/program_language_basic/tracking-with-Extended-Kalman-Filter/data/sample-laser-radar-measurement-data-1.txt", std::ios::in);
    std::ofstream out_file("output.txt", std::ios::out);
    std::vector<MeasurementPackage> measurement_pack_list;
    std::vector<GroundTruthPackage> gt_pack_list;
    std::string line;
    while(std::getline(in_file, line)){
        std::string sensor_type;
        MeasurementPackage meas_package;
        GroundTruthPackage gt_package;
        std::istringstream iss(line);
        long long timestamp;

        iss >> sensor_type;
        if(sensor_type.compare("L") == 0){
            // Laser measurement
            meas_package.sensor_type_ = MeasurementPackage::LASER;
            meas_package.raw_measurements_ = Eigen::VectorXd(2);
            float x;
            float y;
            iss >> x;
            iss >> y;
            meas_package.raw_measurements_ << x, y;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        else if(sensor_type.compare("R") == 0){
            // radar measurement
            meas_package.sensor_type_ = MeasurementPackage::RADAR;
            meas_package.raw_measurements_ = Eigen::VectorXd(3);
            float ro;
            float phi;
            float ro_dot;
            iss >> ro;
            iss >> phi;
            iss >> ro_dot;
            meas_package.raw_measurements_ << ro, phi, ro_dot;
            iss >> timestamp;
            meas_package.timestamp_ = timestamp;
            measurement_pack_list.push_back(meas_package);
        }
        // read ground truth data to compare later
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;
        gt_package.gt_values_ = Eigen::VectorXd(4);
        gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
        gt_pack_list.push_back(gt_package);
    }
    RadarLidarKF rl_kf;
    // used to compute the RMSE later
    std::vector<Eigen::VectorXd> estimations;
    std::vector<Eigen::VectorXd> ground_truth;
    size_t N = measurement_pack_list.size();
    for(size_t k = 0; k < N; ++k){
        auto& meas_pk = measurement_pack_list[k];
        if(meas_pk.sensor_type_ == MeasurementPackage::LASER){
            LidarSensor ls;
            ls.time_for_simulation = meas_pk.timestamp_;
            ls.time_stamp = std::chrono::system_clock::now();
            ls.x = meas_pk.raw_measurements_(0);
            ls.y = meas_pk.raw_measurements_(1);
            rl_kf.process_management(ls);
        }
        else if(meas_pk.sensor_type_ == MeasurementPackage::RADAR){
            RadarSensor rs;
            rs.time_for_simulation = meas_pk.timestamp_;
            rs.time_stamp = std::chrono::system_clock::now();
            float rho = meas_pk.raw_measurements_[0];      // range: radial distance from origin
            float phi = meas_pk.raw_measurements_[1];      // bearing: angle between rho and x axis
            float rho_dot = meas_pk.raw_measurements_[2];  // radial velocity: change of rho
            rs.x = rho * cos(phi);
            rs.y = rho * sin(phi);
            rs.dx = rho_dot * cos(phi);
            rs.dy = rho_dot * sin(phi);
            rl_kf.process_management(rs);
        }
        // output the estimation
        out_file << rl_kf.kf_.x_(0) << "\t";
        out_file << rl_kf.kf_.x_(1) << "\t";
        out_file << rl_kf.kf_.x_(2) << "\t";
        out_file << rl_kf.kf_.x_(3) << "\t";
        // output the measurements
        if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::LASER) {
            // output the estimation
            out_file << measurement_pack_list[k].raw_measurements_(0) << "\t";
            out_file << measurement_pack_list[k].raw_measurements_(1) << "\t";
        } else if (measurement_pack_list[k].sensor_type_ == MeasurementPackage::RADAR) {
            // output the estimation in the cartesian coordinates
            float ro = measurement_pack_list[k].raw_measurements_(0);
            float phi = measurement_pack_list[k].raw_measurements_(1);
            out_file << ro * cos(phi) << "\t"; // p1_meas
            out_file << ro * sin(phi) << "\t"; // ps_meas
        }
        // output the ground truth packages
        out_file << gt_pack_list[k].gt_values_(0) << "\t";
        out_file << gt_pack_list[k].gt_values_(1) << "\t";
        out_file << gt_pack_list[k].gt_values_(2) << "\t";
        out_file << gt_pack_list[k].gt_values_(3) << "\n";

        estimations.push_back(rl_kf.kf_.x_);
        ground_truth.push_back(gt_pack_list[k].gt_values_);
    }

    Tools tools;
    std::cout << "Accuracy - RMSE:" << std::endl << tools.CalculateRMSE(estimations, ground_truth) << std::endl;

    // close files
    if (out_file.is_open()) {
        out_file.close();
    }

    if (in_file.is_open()) {
        in_file.close();
    }
    
    return 0;
}
