#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rcutils/logging_macros.h"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 700; // For reference purposes.
const double WHEEL_RADIUS = 0.045; // Wheel radius in meters
const double WHEEL_BASE = 0.35; // Center of left tire to center of right tire
const int TICKS_PER_METER = 2560; // Calculated is 2786 ticks, but a lower number makes the system more accurate (naybe due to slippage?)

// Distance both wheels have traveled in meters between readings
double distanceLeft = 0.0;
double distanceRight = 0.0;

double cumulativeDistanceLeft = 0.0;
double cumulativeDistanceRight = 0.0;

int32_t lastCountL = 0;
int32_t lastCountR = 0;

nav_msgs::msg::Odometry odomNew;
nav_msgs::msg::Odometry odomOld;

const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

bool broadcast_transform = false;

rclcpp::Time lastJointStateTimeStamp;
rclcpp::Time lastOdomStamp;

class DiffDriveController : public rclcpp::Node
{
  public:
    DiffDriveController()
    : Node("diff_drive_controller")
    {
        this->declare_parameter("broadcast_transform", false);
        this->get_parameter("broadcast_transform", broadcast_transform);

        lastJointStateTimeStamp = this->now();
        lastOdomStamp = lastJointStateTimeStamp;

    	odomNew.header.frame_id = "odom";
    	odomNew.child_frame_id = "base_footprint";
        odomNew.pose.pose.position.z = 0;
        odomNew.pose.pose.orientation.x = 0;
        odomNew.pose.pose.orientation.y = 0;
        odomNew.twist.twist.linear.x = 0;
        odomNew.twist.twist.linear.y = 0;
        odomNew.twist.twist.linear.z = 0;
        odomNew.twist.twist.angular.x = 0;
        odomNew.twist.twist.angular.y = 0;
        odomNew.twist.twist.angular.z = 0;
        odomOld.pose.pose.position.x = initialX;
        odomOld.pose.pose.position.y = initialY;
        odomOld.pose.pose.orientation.z = initialTheta;

    	auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());

        subscriptionR_ = this->create_subscription<std_msgs::msg::Int32>(
        "right_tick_publisher", default_qos, std::bind(&DiffDriveController::topic_callback_right, this, _1));

        subscriptionL_ = this->create_subscription<std_msgs::msg::Int32>(
        "left_tick_publisher", default_qos, std::bind(&DiffDriveController::topic_callback_left, this, _1));

        initialPose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "initial_2d", default_qos, std::bind(&DiffDriveController::set_initial_2d, this, _1));

        auto sensor_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  		//sensor_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  		//sensor_qos.depth = 50;
  		//sensor_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
  		//sensor_qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", sensor_qos);
        publisherQuat_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", sensor_qos);
        publisherJS_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", sensor_qos);

        timer_ = this->create_wall_timer(66ms, std::bind(&DiffDriveController::timer_callback, this));

        if (broadcast_transform) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            RCLCPP_INFO(this->get_logger(), "Broadcasting transform");
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Will NOT broadcast transform");
        }

    }

  private:

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriptionR_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscriptionL_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initialPose_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisherQuat_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherJS_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void publish_joint_state() const {

        rclcpp::Time now = this->now();

        double secondsDiff = now.seconds() - lastJointStateTimeStamp.seconds();

        //RCLCPP_INFO(this->get_logger(), "JS seconds diff: '%f'", secondsDiff);
        sensor_msgs::msg::JointState combined_joint_state;

        combined_joint_state.header.stamp = now;
        //combined_joint_state.header.frame_id = "base_link";
        //right is pushed on first, and we need to consistent from here on out in this method
        combined_joint_state.name.push_back("drivewhl_r_joint");
        combined_joint_state.name.push_back("drivewhl_l_joint");


        //we have distance traveled in meters, so convert back to total ticks
        double totalClicksLeft = cumulativeDistanceLeft*TICKS_PER_METER;
        double totalClicksRight = cumulativeDistanceRight*TICKS_PER_METER;

        //position is total radians the wheel has spun
        combined_joint_state.position.push_back(totalClicksRight/TICKS_PER_REVOLUTION*2.0*PI);
        combined_joint_state.position.push_back(totalClicksLeft/TICKS_PER_REVOLUTION*2.0*PI);

        combined_joint_state.velocity.push_back(2.0 * PI * (distanceRight*TICKS_PER_METER/secondsDiff)/TICKS_PER_REVOLUTION);
        combined_joint_state.velocity.push_back(2.0 * PI * (distanceLeft*TICKS_PER_METER/secondsDiff)/TICKS_PER_REVOLUTION);

        publisherJS_->publish(combined_joint_state);


        lastJointStateTimeStamp = now;

    }

    void sendTransform() {

        tf2::Quaternion q;

        q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

        geometry_msgs::msg::TransformStamped transform_stamped;

        transform_stamped.header.stamp    = this->get_clock()->now();
        transform_stamped.header.frame_id = "odom";
        transform_stamped.child_frame_id  = "base_footprint";

        transform_stamped.transform.translation.x = odomNew.pose.pose.position.x;
        transform_stamped.transform.translation.y = odomNew.pose.pose.position.y;
        transform_stamped.transform.translation.z = odomNew.pose.pose.position.z;

        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform_stamped);

    }

    void update_odam() const {
     //	auto message = nav_msgs::msg::Odometry();

      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // Calculate the average distance
        double cycleDistance = (distanceRight + distanceLeft) / 2.0;



        // Calculate the number of radians the robot has turned since the last cycle
        double cycleAngle = asin((distanceRight-distanceLeft)/WHEEL_BASE);

        //RCLCPP_INFO(this->get_logger(), "radians turned: '%f'", cycleAngle);

        // Average angle during the last cycle
        double avgAngle = cycleAngle/2.0 + odomOld.pose.pose.orientation.z;

        if (avgAngle > PI) {
            avgAngle -= 2.*PI;
        }
        else if (avgAngle < -PI) {
            avgAngle += 2.*PI;
        }
        else{}

        // Calculate the new pose (x, y, and theta)
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
        odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

        // Prevent lockup from a single bad cycle
        if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
             || isnan(odomNew.pose.pose.position.z)) {
            odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
            odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
            odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
        }

        // Make sure theta stays in the correct range
        if (odomNew.pose.pose.orientation.z > PI) {
            odomNew.pose.pose.orientation.z -= 2. * PI;
        }
        else if (odomNew.pose.pose.orientation.z < -PI) {
            odomNew.pose.pose.orientation.z += 2. * PI;
        }
        else{}

        // Compute the velocity
        rclcpp::Time now = this->now();

        odomNew.header.stamp = now;

        double timeDiff = now.seconds() - lastOdomStamp.seconds();

        //RCLCPP_INFO(this->get_logger(), "Time diff: '%f'", timeDiff);

        odomNew.twist.twist.linear.x = cycleDistance/timeDiff;
        odomNew.twist.twist.angular.z = cycleAngle/timeDiff;

        // Save the pose data for the next cycle
        odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
        odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
        odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
        odomOld.header.stamp = odomNew.header.stamp;

        publisher_->publish(odomNew);

        lastOdomStamp = now;
    }

    void publish_quat() {

        tf2::Quaternion q;

        q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

        nav_msgs::msg::Odometry quatOdom;
        quatOdom.header.stamp = odomNew.header.stamp;
        quatOdom.header.frame_id = "odom";
        quatOdom.child_frame_id = "base_footprint";
        quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
        quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
        quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
        quatOdom.pose.pose.orientation.x = q.x();
        quatOdom.pose.pose.orientation.y = q.y();
        quatOdom.pose.pose.orientation.z = q.z();
        quatOdom.pose.pose.orientation.w = q.w();
        quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
        quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
        quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
        quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
        quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
        quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

        for(int i = 0; i<36; i++) {
            if(i == 0 || i == 7 || i == 14) {
              quatOdom.pose.covariance[i] = .01;
             }
             else if (i == 21 || i == 28 || i== 35) {
               quatOdom.pose.covariance[i] += 0.1;
             }
             else {
               quatOdom.pose.covariance[i] = 0;
             }
        }

        publisherQuat_->publish(quatOdom);
    }

    // Calculate the distance the left wheel has traveled since the last cycle
    void Calc_Left(const std_msgs::msg::Int32& leftCount) const
    {


        int32_t leftCountInt = leftCount.data;
        if(leftCountInt != 0 && lastCountL != 0) {

            int32_t leftTicks = leftCountInt - lastCountL;

            if (leftTicks > 10000) {
              leftTicks = 0 - (65535 - leftTicks);
            }
            else if (leftTicks < -10000) {
              leftTicks = 65535-leftTicks;
            }
            else{}
            distanceLeft = (double)leftTicks/TICKS_PER_METER;
            cumulativeDistanceLeft += distanceLeft;
            //RCLCPP_INFO(this->get_logger(), "Distance left: '%f'", distanceLeft);
        }

        lastCountL = leftCountInt;
    }

    // Calculate the distance the right wheel has traveled since the last cycle
    void Calc_Right(const std_msgs::msg::Int32 rightCount) const
    {

        int32_t rightCountInt = rightCount.data;
        if(rightCountInt != 0 && lastCountR != 0) {

            int32_t rightTicks = rightCountInt - lastCountR;

            if (rightTicks > 10000) {
              rightTicks = 0 - (65535 - rightTicks);
            }
            else if (rightTicks < -10000) {
              rightTicks = 65535 - rightTicks;
            }
            else{}
            distanceRight = (double)rightTicks/TICKS_PER_METER;
            cumulativeDistanceRight += distanceRight;
            //RCLCPP_INFO(this->get_logger(), "Distance right: '%f'", distanceRight);
        }
        lastCountR = rightCount.data;
    }

    void timer_callback()
    {
     	this->update_odam();
     	this->publish_quat();
     	if (broadcast_transform) {
            this->sendTransform();
     	}

     	this->publish_joint_state();
    }

    void topic_callback_right(const std_msgs::msg::Int32& msg) const
    {
    	static bool showedMessageR = false;

      if (!showedMessageR) {
      	showedMessageR = true;
      	RCLCPP_INFO(this->get_logger(), "rec'd right tick (showing once): '%ld'", (long int)msg.data);
      }
      this->Calc_Right(msg);
    }

    void topic_callback_left(const std_msgs::msg::Int32& msg) const
    {
    	static bool showedMessageL = false;
      //RCLCPP_INFO(this->get_logger(), "rec'd left tick: '%ld'", (long int)msg.data);

      if (!showedMessageL) {
      	showedMessageL = true;
      	RCLCPP_INFO(this->get_logger(), "rec'd left tick (showing once): '%ld'", (long int)msg.data);
      }
      this->Calc_Left(msg);
    }
    void set_initial_2d(const geometry_msgs::msg::PoseStamped& rvizClick) const
    {
        odomNew.pose.pose.position.x = rvizClick.pose.position.x;
        odomNew.pose.pose.position.y = rvizClick.pose.position.y;
        odomNew.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DiffDriveController>());
  rclcpp::shutdown();
  return 0;
}
