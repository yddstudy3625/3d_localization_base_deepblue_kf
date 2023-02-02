/*
 * @Description: 数据预处理模块，包括时间同步、点云去畸变等
 * @Author: Ren Qian
 * @Date: 2020-02-10 08:38:42
 */
#include "lidar_localization/data_pretreat/data_pretreat_flow.hpp"

#include "glog/logging.h"
#include "lidar_localization/global_defination/global_defination.h"

namespace lidar_localization {
DataPretreatFlow::DataPretreatFlow(ros::NodeHandle& nh, std::string cloud_topic) {
    std::string config_file_path =
      WORK_SPACE_PATH + "/config/pretreat/pretreat.yaml";

    YAML::Node config_node = YAML::LoadFile(config_file_path);
    std::string lidar_topic = config_node["lidarTopic"].as<std::string>();
    std::string imu_topic = config_node["imuTopic"].as<std::string>();
    std::string gps_topic = config_node["gpsTopic"].as<std::string>();
    std::string vel_topic = config_node["velocityTopic"].as<std::string>();

    std::string lidar_frame = config_node["lidarFrame"].as<std::string>();
    std::string imu_frame = config_node["imuFrame"].as<std::string>();

    // subscribers:
    // a. velodyne measurement:
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, lidar_topic, 100000);
    // b. OXTS IMU:
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh, imu_topic, 1000000);
    // c. OXTS velocity:
    velocity_sub_ptr_ = std::make_shared<VelocitySubscriber>(nh, vel_topic, 1000000);
    // d. OXTS GNSS:
    gnss_sub_ptr_ = std::make_shared<GNSSSubscriber>(nh, gps_topic, 1000000);
    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh, imu_frame, lidar_frame);

    // publishers:
    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh, cloud_topic, lidar_frame, 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh, "/synced_imu", imu_frame, 100);
    pos_vel_pub_ptr_ = std::make_shared<PosVelPublisher>(nh, "/synced_pos_vel", "/map", imu_frame, 100);
    gnss_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "/synced_gnss", "/map", lidar_frame, 100);

    // motion compensation for lidar measurement:
    distortion_adjust_ptr_ = std::make_shared<DistortionAdjust>();
}

bool DataPretreatFlow::Run() {
    if (!ReadData())
        return false;

    // if (!InitCalibration()) 
    //     return false;

    // LOG(INFO) << " Read S 22222222222222222 " << std::endl;

    if (!InitGNSS())
        return false;

    // LOG(INFO) << " Read S 33333333333333333 --> cloud imu vel gnss size: " << cloud_data_buff_.size() << "," << imu_data_buff_.size() << "," << velocity_data_buff_.size() << "," << gnss_data_buff_.size() << std::endl;

    while(HasData()) {
        //  LOG(INFO) << " Read S 444444444444444 " << std::endl;
        if (!ValidData())
            continue;
        // LOG(INFO) << " Read S 55555555555555555555555555555555555 " << std::endl;
        TransformData();
        PublishData();
    }

    return true;
}

bool DataPretreatFlow::ReadData() {
    static std::deque<IMUData> unsynced_imu_;
    static std::deque<VelocityData> unsynced_velocity_;
    static std::deque<GNSSData> unsynced_gnss_;

    // fetch lidar measurements from buffer:
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    imu_sub_ptr_->ParseData(unsynced_imu_);
    velocity_sub_ptr_->ParseData(unsynced_velocity_);
    gnss_sub_ptr_->ParseData(unsynced_gnss_);

    // LOG(INFO) << " Read laser->imu->vel->gnss size : " << cloud_data_buff_.size() << "," << unsynced_imu_.size() << ","
    //     << unsynced_velocity_.size() << "," << unsynced_gnss_.size() << ","   << std::endl;

    if (cloud_data_buff_.size() == 0)
        return false;

    // use timestamp of lidar measurement as reference:
    double cloud_time = cloud_data_buff_.front().time;
    // sync IMU, velocity and GNSS with lidar measurement:
    // find the two closest measurement around lidar measurement time
    // then use linear interpolation to generate synced measurement:
    bool valid_imu = IMUData::SyncData(unsynced_imu_, imu_data_buff_, cloud_time);
    bool valid_velocity = VelocityData::SyncData(unsynced_velocity_, velocity_data_buff_, cloud_time);
    bool valid_gnss = GNSSData::SyncData(unsynced_gnss_, gnss_data_buff_, cloud_time);

    // only mark lidar as 'inited' when all the three sensors are synced:
    static bool sensor_inited = false;
    if (!sensor_inited) {
        if (!valid_imu || !valid_velocity || !valid_gnss) {
            LOG(INFO) << " valid_imu -> valid_velocity -> valid_gnss: " << valid_imu << "," << valid_velocity << "," << valid_gnss << std::endl;
            cloud_data_buff_.pop_front();
            return false;
        }
        sensor_inited = true;
    }

    // LOG(INFO) << " Read endddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd " << std::endl;
    return true;
}

bool DataPretreatFlow::InitCalibration() {
    // lookup imu pose in lidar frame:
    static bool calibration_received = false;
    if (!calibration_received) {
        if (lidar_to_imu_ptr_->LookupData(lidar_to_imu_)) {
            calibration_received = true;
        }
    }

    return calibration_received;
}

bool DataPretreatFlow::InitGNSS() {
    static bool gnss_inited = false;
    if (!gnss_inited) {
        GNSSData gnss_data = gnss_data_buff_.front();
        gnss_data.InitOriginPosition();
        gnss_inited = true;
    }

    return gnss_inited;
}

bool DataPretreatFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (imu_data_buff_.size() == 0)
        return false;
    if (velocity_data_buff_.size() == 0)
        return false;
    if (gnss_data_buff_.size() == 0)
        return false;

    return true;
}

bool DataPretreatFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_velocity_data_ = velocity_data_buff_.front();
    current_gnss_data_ = gnss_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    double diff_velocity_time = current_cloud_data_.time - current_velocity_data_.time;
    double diff_gnss_time = current_cloud_data_.time - current_gnss_data_.time;
    //
    // this check assumes the frequency of lidar is 10Hz:
    //
    if (diff_imu_time < -0.05 || diff_velocity_time < -0.05 || diff_gnss_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_imu_time > 0.05) {
        imu_data_buff_.pop_front();
        return false;
    }

    if (diff_velocity_time > 0.05) {
        velocity_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    velocity_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;
}

bool DataPretreatFlow::TransformData() {
    // a. get reference pose:
    gnss_pose_ = Eigen::Matrix4f::Identity();
    // get position from GNSS
    current_gnss_data_.UpdateXYZ();
    gnss_pose_(0,3) = current_gnss_data_.local_E;
    gnss_pose_(1,3) = current_gnss_data_.local_N;
    // gnss_pose_(2,3) = current_gnss_data_.local_U;
    gnss_pose_(2,3) = 4.36;
    // get orientation from IMU:
    gnss_pose_.block<3,3>(0,0) = current_imu_data_.GetOrientationMatrix();
    // this is lidar pose in GNSS/map frame:
    // gnss_pose_ *= lidar_to_imu_;

    // b. set synced pos vel
    pos_vel_.pos.x() = current_gnss_data_.local_E;
    pos_vel_.pos.y() = current_gnss_data_.local_N;
    pos_vel_.pos.z() = current_gnss_data_.local_U;

    pos_vel_.vel.x() = current_velocity_data_.linear_velocity.x;
    pos_vel_.vel.y() = current_velocity_data_.linear_velocity.y;
    pos_vel_.vel.z() = current_velocity_data_.linear_velocity.z;

    // c. motion compensation for lidar measurements:
    // current_velocity_data_.TransformCoordinate(lidar_to_imu_);
    // distortion_adjust_ptr_->SetMotionInfo(0.1, current_velocity_data_);
    // distortion_adjust_ptr_->AdjustCloud(current_cloud_data_.cloud_ptr, current_cloud_data_.cloud_ptr);

    return true;
}

bool DataPretreatFlow::PublishData() {
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.time);

    pos_vel_pub_ptr_->Publish(pos_vel_, current_cloud_data_.time);
    
    //
    // this synced odometry has the following info:
    //
    // a. lidar frame's pose in map
    // b. lidar frame's velocity 
    gnss_pub_ptr_->Publish(gnss_pose_, current_velocity_data_, current_cloud_data_.time);

    
    return true;
}
}