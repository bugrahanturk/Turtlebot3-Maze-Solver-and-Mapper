#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <termios.h>
#include <geometry_msgs/Twist.h>
#include <unistd.h>

class Mapper 
{
public:
    Mapper() : nh_("~"), cozunurluk_(0.05), window_size_(800) 
    {
        // ROS Aboneliklerini tanimlayalim
        scan_sub_ = nh_.subscribe("/scan", 10, &Mapper::scanCallback, this);

        // Haritayi olusturalim
        map_ = cv::Mat::zeros(window_size_, window_size_, CV_8UC3);
        cv::namedWindow("Map");
    }

    void spin() 
    {
        ros::Rate rate(10);
        while (ros::ok()) 
        {
            ros::spinOnce();
            cv::imshow("Map", map_);
            cv::waitKey(1);
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    tf::TransformListener tf_listener_;
    cv::Mat map_;
    double cozunurluk_;
    int window_size_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) 
    {
        try 
        {
            // Her lazer sensoru olcumunu isleyelim
            for (size_t i = 0; i < scan->ranges.size(); ++i) 
            {
                if (scan->ranges[i] < scan->range_min || scan->ranges[i] > scan->range_max)
                    {
                        continue;
                    }
                    

                // `base_scan` cercevesinde kartezyen koordinatlari hesaplayalim
                double aci = scan->angle_min + i * scan->angle_increment;
                double x = scan->ranges[i] * cos(aci);
                double y = scan->ranges[i] * sin(aci);

                // `base_scan` -> `base_footprint` cerceve donusumu yapalim
                tf::StampedTransform base_to_footprint;
                tf_listener_.lookupTransform("base_footprint", "base_scan", ros::Time(0), base_to_footprint);

                tf::Vector3 point_base(x, y, 0.0);
                tf::Vector3 point_footprint = base_to_footprint * point_base;

                // `base_footprint` -> `odom` cerceve donusumu
                tf::StampedTransform footprint_to_odom;
                tf_listener_.lookupTransform("odom", "base_footprint", ros::Time(0), footprint_to_odom);

                tf::Vector3 point_odom = footprint_to_odom * point_footprint;

                // Harita koordinatlari
                int map_x = (point_odom.x() / cozunurluk_) + window_size_ / 2;
                int map_y = (point_odom.y() / cozunurluk_) + window_size_ / 2;

                if (map_x >= 0 && map_x < window_size_ && map_y >= 0 && map_y < window_size_) 
                {
                    cv::circle(map_, cv::Point(map_x, map_y), 1, cv::Scalar(255, 255, 255), -1);
                }
            }
        } 
        catch (tf::TransformException& ex) 
        {
            ROS_WARN("%s", ex.what());
        }
    }
};

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "my_mapper");

    Mapper mapper;
    mapper.spin();

    return 0;
}
