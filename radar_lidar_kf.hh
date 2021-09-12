#include "Eigen/Dense"
#include <chrono>
#include "kalman_filter.hh"
#define SIMULATION 1
struct RadarSensor{
#if SIMULATION
    long long time_for_simulation;
#endif
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time_stamp;
    double x;
    double y;
    double dx; // velocity
    double dy;
};
struct LidarSensor{
#if SIMULATION
    long long time_for_simulation;
#endif
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time_stamp;
    double x;
    double y;
};

class RadarLidarKF{
    public:
        RadarLidarKF();
        ~RadarLidarKF() = default;
        void process_management(const RadarSensor&);
        void process_management(const LidarSensor&);
        KalmanFilter kf_;
    private:
        bool is_initialized_;
#if SIMULATION
        long long int pre_time_for_simulation;
#endif
        std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> previous_time_stamp;
        Eigen::MatrixXd R_laser_;    // laser measurement noise
        Eigen::MatrixXd R_radar_;    // radar measurement noise
        Eigen::MatrixXd H_laser_;    // measurement function for laser
        Eigen::MatrixXd H_radar_;         // measurement function for radar
        double noise_ax;
        double noise_ay;
        
        template<typename T>
        void kf_predict_process(const T&);
};