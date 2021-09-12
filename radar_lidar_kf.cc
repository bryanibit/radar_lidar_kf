#include "radar_lidar_kf.hh"

RadarLidarKF::RadarLidarKF(){
    // assign value to R H
    is_initialized_ = false;
    pre_time_for_simulation = 0;
    previous_time_stamp = std::chrono::system_clock::now();
    R_laser_ = Eigen::MatrixXd(2, 2);
    R_radar_ = Eigen::MatrixXd(4, 4);
    H_laser_ = Eigen::MatrixXd(2, 4);
    H_radar_ = Eigen::MatrixXd(4, 4);
    //measurement covariance matrix - laser
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    R_radar_ << 0.0225, 0,0,0,
                0, 0.0225,0,0,
                0,0, 20, 0,
                0, 0, 0, 20;
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    H_radar_ << 1,0,0,0,
                0,1,0,0,
                0,0,1,0,
                0,0,0,1;
    kf_.P_ = Eigen::MatrixXd(4, 4);
    kf_.P_ << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1000, 0,
            0, 0, 0, 1000;

    kf_.F_ = Eigen::MatrixXd(4, 4);
    kf_.F_ << 1, 0, 1, 0,
            0, 1, 0, 1,
            0, 0, 1, 0,
            0, 0, 0, 1;
    noise_ax = 9.0;
    noise_ay = 9.0;
}
template<typename T>
void RadarLidarKF::kf_predict_process(const T& sensor){
    // assigen R H to kf_
#if SIMULATION
    double dt = (sensor.time_for_simulation - pre_time_for_simulation) / 1000000.0;
    pre_time_for_simulation = sensor.time_for_simulation;
#else    
    double dt = (sensor.time_stamp - previous_time_stamp).count();
    previous_time_stamp = sensor.time_stamp;
#endif
    double dt_2 = dt * dt;
    double dt_3 = dt_2 * dt;
    double dt_4 = dt_3 * dt;

    // Modify the F matrix so that the time is integrated
    kf_.F_(0, 2) = dt;
    kf_.F_(1, 3) = dt;

  //set the process covariance matrix Q
    kf_.Q_ = Eigen::MatrixXd(4, 4);
    kf_.Q_ << dt_4/4*noise_ax,   0,                dt_3/2*noise_ax,  0,
             0,                 dt_4/4*noise_ay,  0,                dt_3/2*noise_ay,
             dt_3/2*noise_ax,   0,                dt_2*noise_ax,    0,
             0,                 dt_3/2*noise_ay,  0,                dt_2*noise_ay;

    kf_.Predict();

}
void RadarLidarKF::process_management(const RadarSensor& rs){
#if SIMULATION
    if(!is_initialized_){
        kf_.x_ = Eigen::VectorXd(4);
        kf_.x_ << rs.x, rs.y, 0, 0;  // x, y, vx, vy
        pre_time_for_simulation = rs.time_for_simulation;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
#endif
    kf_predict_process(rs);
    // call kf_ function
    kf_.H_ = H_radar_;
    kf_.R_ = R_radar_;
    Eigen::VectorXd meas_radar(4);
    meas_radar << rs.x, rs.y, rs.dx, rs.dy;
    kf_.Update(meas_radar);
}
void RadarLidarKF::process_management(const LidarSensor& ls){
#if SIMULATION
    if(!is_initialized_){
        kf_.x_ = Eigen::VectorXd(4);
        kf_.x_ << ls.x, ls.y, 0, 0;  // x, y, vx, vy
        pre_time_for_simulation = ls.time_for_simulation;
        // done initializing, no need to predict or update
        is_initialized_ = true;
        return;
    }
#endif    
    kf_predict_process(ls);
    kf_.H_ = H_laser_;
    kf_.R_ = R_laser_;
    Eigen::VectorXd meas_laser(2);
    meas_laser << ls.x, ls.y;
    kf_.Update(meas_laser);
}
