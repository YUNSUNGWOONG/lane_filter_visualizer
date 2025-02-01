#ifndef LIDARLANEDETECTOR_H
#define LIDARLANEDETECTOR_H
#define PCL_NO_PRECOMPILE

#include <iostream>
#include <cmath>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "point_os1.h"

/*
차선 검출하는 코드
*/

class LaneDetector : public rclcpp::Node {
public:
    LaneDetector() : Node("lane_detector") {
		// 파라미터 선언
		this->declare_parameter<int>("channel_threshold", 10);
		this->declare_parameter<float>("angle_threshold", 5.0);
		this->declare_parameter<int>("intensity_threshold", 100);
		this->declare_parameter<int>("lane_candidate", 3);
		this->declare_parameter<float>("lane_width", 3.5);
		this->declare_parameter<float>("x_std_dev_threshold", 0.5);
		this->declare_parameter<float>("y_std_dev_threshold", 0.5);

		try{
			initParams();
            initializeSubscriberPublisher();
        } catch (const std::exception& e){    
            RCLCPP_ERROR(this->get_logger(), "Initialization failed %s", e.what());
            rclcpp::shutdown();
        }

	}

private:
    struct Line {
        float a;
        float b;
    };
    typedef ouster_ros::OS1::PointOS1 PointT;
    typedef std::vector<PointT> VectorT;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    int channel_threshold_;
    float angle_threshold_;
    int intensity_threshold_;
    int lane_candidate_;
    float lane_width_;
    float x_std_dev_threshold_;
    float y_std_dev_threshold_;
    Line prev_line_[7];
    int is_line_[7];

    void initParams();
	void initializeSubscriberPublisher();
	
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void extractDrivableRegion(const pcl::PointCloud<PointT> *cloud, std::vector<VectorT> *drivable_region, pcl::PointCloud<PointT> *filtered_cloud);
    void extractRoadMarkLine(const std::vector<VectorT> *drivable_region, std::vector<VectorT> *road_mark_line);
    void extractRoadLine(const std::vector<VectorT> *road_mark_line, Line prev_line[], int is_line[], pcl::PointCloud<PointT> *filtered_cloud);
    void publish(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &marker_pub, const int is_line[], const Line prev_line[]);
};

void LaneDetector::initParams() {
    this->get_parameter("channel_threshold", channel_threshold_);
    this->get_parameter("angle_threshold", angle_threshold_);
    this->get_parameter("intensity_threshold", intensity_threshold_);
    this->get_parameter("lane_candidate", lane_candidate_);
    this->get_parameter("lane_width", lane_width_);
    this->get_parameter("x_std_dev_threshold", x_std_dev_threshold_);
    this->get_parameter("y_std_dev_threshold", y_std_dev_threshold_);

    // 초기화
    prev_line_[0].a = 0.0;
    prev_line_[0].b = 1.5 * lane_width_;
    prev_line_[1].a = 0.0;
    prev_line_[1].b = 1.0 * lane_width_;
    prev_line_[2].a = 0.0;
    prev_line_[2].b = 0.5 * lane_width_;
    prev_line_[3].a = 0.0;
    prev_line_[3].b = 0.0;
    prev_line_[4].a = 0.0;
    prev_line_[4].b = -0.5 * lane_width_;
    prev_line_[5].a = 0.0;
    prev_line_[5].b = -1.0 * lane_width_;
    prev_line_[6].a = 0.0;
    prev_line_[6].b = -1.5 * lane_width_;

    for (int i = 0; i < 7; ++i) {
        is_line_[i] = 0;
    }
}

// void LaneDetector::initializeSubscriberPublisher() {
//     // QoS 설정
//     rclcpp::QoS qos_settings(10);
//     qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  // BEST_EFFORT 설정
//     qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);       // VOLATILE 설정

//     // 포인트 클라우드 구독자 생성
//     pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//         "/ouster/points",  // 토픽 이름
//         rclcpp::SensorDataQoS(),
//         std::bind(&LaneDetector::cloudCallback, this, std::placeholders::_1)
//     );

//     // 필터링된 포인트 클라우드 발행자 생성
//     pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
//         "/filtered_points",
//         10
//     );

//     // 마커 발행자 생성
//     marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
//         "/road_line",
//         10
//     );
// }

void LaneDetector::initializeSubscriberPublisher() {
    // QoS 설정 - REIABLE 사용
    rclcpp::QoS qos_settings(10);
	qos_settings.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);  
    qos_settings.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    // Ouster LiDAR 데이터 구독
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/ouster/points",
        qos_settings,
        std::bind(&LaneDetector::cloudCallback, this, std::placeholders::_1)
    );

    // 필터링된 포인트 클라우드 발행
    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_points",
        10
    );

    // 마커 발행
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/road_line",
        10
    );
}


// void LaneDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
//     pcl::PointCloud<PointT> cloud;
//     pcl::PointCloud<PointT> filtered_cloud;

//     std::vector<VectorT> drivable_region(1024);
//     std::vector<VectorT> road_mark_line(1024);

//     sensor_msgs::msg::PointCloud2 filtered_cloud_msg;

//     pcl::fromROSMsg(*cloud_msg, cloud);

//     extractDrivableRegion(&cloud, &drivable_region, &filtered_cloud);
//     extractRoadMarkLine(&drivable_region, &road_mark_line);
//     extractRoadLine(&road_mark_line, prev_line_, is_line_, &filtered_cloud);
//     publish(marker_pub_, is_line_, prev_line_);

//     pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
//     filtered_cloud_msg.header.frame_id =  "os_sensor"; 
//     filtered_cloud_msg.header.stamp = this->now();
    

//     pc_pub_->publish(filtered_cloud_msg);
    
// }

void LaneDetector::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr filtered_cloud(new pcl::PointCloud<PointT>);

    std::vector<VectorT> drivable_region(1024);
    std::vector<VectorT> road_mark_line(1024);
    sensor_msgs::msg::PointCloud2 filtered_cloud_msg;

    // PointCloud2 → PCL 변환 (Ouster와 호환되도록)
    try {
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "PointCloud conversion failed: %s", e.what());
        return;
    }

    extractDrivableRegion(pcl_cloud.get(), &drivable_region, filtered_cloud.get());
    extractRoadMarkLine(&drivable_region, &road_mark_line);
    extractRoadLine(&road_mark_line, prev_line_, is_line_, filtered_cloud.get());
    publish(marker_pub_, is_line_, prev_line_);

    pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
    filtered_cloud_msg.header.frame_id = "os_sensor";  // 실제 Ouster 프레임 확인 후 수정
    filtered_cloud_msg.header.stamp = this->now();
    
    pc_pub_->publish(filtered_cloud_msg);
}


void LaneDetector::extractDrivableRegion(const pcl::PointCloud<PointT> *cloud, std::vector<VectorT> *drivable_region, pcl::PointCloud<PointT> *filtered_cloud) {
    for (int a = 0; a < 1024; a++) {
        for (int c = 63; c > channel_threshold_; c--) {
            auto angle = (cloud->points[a * 64 + c - 1].z - cloud->points[a * 64 + c].z) /
                         sqrt(powf(cloud->points[a * 64 + c].x - cloud->points[a * 64 + c - 1].x, 2) +
                              powf(cloud->points[a * 64 + c].y - cloud->points[a * 64 + c - 1].y, 2)) * 180.0 / M_PI;
            if (angle < angle_threshold_) {
                (*drivable_region)[a].push_back(cloud->points[a * 64 + c]);
            } else {
                break;
            }
        }
        filtered_cloud->points.insert(filtered_cloud->points.end(), (*drivable_region)[a].begin(), (*drivable_region)[a].end());
    }
}

//extractRoadMarkLine(&drivable_region, &road_mark_line);
void LaneDetector::extractRoadMarkLine(
    const std::vector<VectorT> *drivable_region,
    std::vector<VectorT> *road_mark_line) 
{
    for (size_t a = 0; a < drivable_region->size(); ++a) {
        for (const auto &point : (*drivable_region)[a]) {
            if (point.intensity > intensity_threshold_) {
                (*road_mark_line)[a].push_back(point);
            }
        }
    }
}

//extractRoadLine(&road_mark_line, prev_line_, is_line_, &filtered_cloud);
void LaneDetector::extractRoadLine(
    const std::vector<VectorT> *road_mark_line, 
    Line prev_line[], 
    int is_line[], 
    pcl::PointCloud<PointT> *filtered_cloud) 
{
    std::vector<VectorT> closest_point(lane_candidate_ * 2 - 1);
    float min_distance = 0.25;
    float distance = 0.0;
    int index = -1;

    // 각 차선에 가장 가까운 점 찾기
    for (int a = 0; a < 1024; a++) {
        for (size_t i = 0; i < (*road_mark_line)[a].size(); i++) {
            min_distance = 0.25;
            index = -1;
            for (int j = 0; j < lane_candidate_ * 2 - 1; j++) {
                distance = fabs(prev_line[j].a * (*road_mark_line)[a][i].x -
                                (*road_mark_line)[a][i].y + prev_line[j].b) /
                           sqrt(powf(prev_line[j].a, 2) + 1);
                if (distance < min_distance) {
                    min_distance = distance;
                    index = j;
                }
            }
            if (index != -1) {
                closest_point[index].push_back((*road_mark_line)[a][i]);
            }
        }
    }

    // 초기화
    for (int i = 0; i < 7; ++i) {
        is_line[i] = 0;
    }

    int even_count = 0;
    int odd_count = 0;

    // 통계 계산
    for (int i = 0; i < lane_candidate_ * 2 - 1; i++) {
        float y_total = 0.0, y_mean = 0.0, y_square_total = 0.0, y_std_dev = 0.0;
        float x_total = 0.0, x_mean = 0.0, x_square_total = 0.0, x_std_dev = 0.0;

        if (!closest_point[i].empty()) {
            for (const auto &point : closest_point[i]) {
                y_total += point.y;
                x_total += point.x;
            }
            y_mean = y_total / closest_point[i].size();
            x_mean = x_total / closest_point[i].size();

            for (const auto &point : closest_point[i]) {
                y_square_total += powf(point.y - y_mean, 2);
                x_square_total += powf(point.x - x_mean, 2);
            }

            y_std_dev = sqrt(y_square_total / (closest_point[i].size() - 1));
            x_std_dev = sqrt(x_square_total / (closest_point[i].size() - 1));

            prev_line[i].b = y_mean;

            if (fabs(prev_line[i].b - y_mean) < 0.4 && x_std_dev > x_std_dev_threshold_ &&
                y_std_dev < y_std_dev_threshold_) {
                is_line[i] = 1;
                if (i % 2 == 0) {
                    even_count++;
                } else {
                    odd_count++;
                }
            }
        }
    }

    // 가장 많이 발견된 라인 유지
    if (even_count > odd_count) {
        for (int i = 1; i < lane_candidate_ * 2 - 1; i += 2) {
            is_line[i] = 0;
        }
    } else {
        for (int i = 0; i < lane_candidate_ * 2 - 1; i += 2) {
            is_line[i] = 0;
        }
    }
}


void LaneDetector::publish(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &marker_pub,
                           const int is_line[], const Line prev_line[]) {
    for (int i = 0; i < lane_candidate_ * 2 - 1; i++) {
        if (is_line[i] == 1) {
            visualization_msgs::msg::Marker line;
            std::vector<geometry_msgs::msg::Point> line_points;
            geometry_msgs::msg::Point temp_point;
            for (int j = -10; j < 11; j++) {
                temp_point.x = j;
                temp_point.y = prev_line[i].b;
                temp_point.z = -2.0;
                line_points.push_back(temp_point);
            }
            line.header.frame_id = "os1_lidar/lidar";
            line.header.stamp = this->now();
            line.ns = "lines";
            line.id = i;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.scale.x = 0.2;
            line.color.b = 1.0;
            line.color.a = 1.0;
            line.points = line_points;

            marker_pub->publish(line);
        }
    }
}

#endif
