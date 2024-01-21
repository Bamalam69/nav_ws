#include <iostream>
#include <unistd.h>
#include <math.h>
#include <stdlib.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Inertial Labs source header
#include "ILDriver.h"

// Adding message type headers
#include "inertiallabs_interfaces/msg/sensor_data.hpp"
#include "inertiallabs_interfaces/msg/ins_data.hpp"
#include "inertiallabs_interfaces/msg/gps_data.hpp"
#include "inertiallabs_interfaces/msg/gnss_data.hpp"
#include "inertiallabs_interfaces/msg/marine_data.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// // Publishers
// struct Context
// {
// 	// std::shared_ptr<rclcpp::Publisher<inertiallabs_interfaces::msg::SensorData>> publisher_sensor_data;
// 	// std::shared_ptr<rclcpp::Publisher<inertiallabs_interfaces::msg::InsData>> publisher_ins_data;
// 	// std::shared_ptr<rclcpp::Publisher<inertiallabs_interfaces::msg::GpsData>> publisher_gps_data;
// 	// std::shared_ptr<rclcpp::Publisher<inertiallabs_interfaces::msg::GnssData>> publisher_gnss_data;
// 	// std::shared_ptr<rclcpp::Publisher<inertiallabs_interfaces::msg::MarineData>> publisher_marine_data;

// 	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Imu>> publisher_sensor_data;
// 	std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> publisher_ins_data;
// 	std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>> publisher_gps_data;

// 	std::string imu_frame_id;

// 	rclcpp::Node::SharedPtr node;
// };

// void publish_device(IL::INSDataStruct * data, void * contextPtr)
// {
// 	Context * context = reinterpret_cast<Context *>(contextPtr);
// 	static int seq = 0;
// 	seq++;

// 	// inertiallabs_interfaces::msg::SensorData msg_sensor_data;
// 	// inertiallabs_interfaces::msg::InsData msg_ins_data;
// 	// inertiallabs_interfaces::msg::GpsData msg_gps_data;
// 	// inertiallabs_interfaces::msg::GnssData msg_gnss_data;
// 	// inertiallabs_interfaces::msg::MarineData msg_marine_data;

// 	rclcpp::Time timestamp = context->node->now();

// 	sensor_msgs::msg::Imu msg_sensor_data;
// 	nav_msgs::msg::Odometry msg_odo_data;
// 	sensor_msgs::msg::NavSatFix msg_gps_data;

// 	msg_sensor_data.header.stamp = timestamp;
// 	msg_sensor_data.header.frame_id = context->imu_frame_id;
// 	msg_sensor_data.orientation.x = data->Quat[1];
// 	msg_sensor_data.orientation.y = data->Quat[2];
// 	msg_sensor_data.orientation.z = data->Quat[3];
// 	msg_sensor_data.orientation.w = data->Quat[0];
// 	msg_sensor_data.angular_velocity.x = data->Gyro[0];
// 	msg_sensor_data.angular_velocity.y = data->Gyro[1];
// 	msg_sensor_data.angular_velocity.z = data->Gyro[2];
// 	msg_sensor_data.linear_acceleration.x = data->Acc[0];
// 	msg_sensor_data.linear_acceleration.y = data->Acc[1];
// 	msg_sensor_data.linear_acceleration.z = data->Acc[2];
// 	context->publisher_sensor_data->publish(msg_sensor_data);

// 	// msg_odo_data.header.stamp = timestamp;
// 	// msg_odo_data.header.frame_id = context->imu_frame_id;
// 	// // msg_ins_data.child_frame_id = context->imu_frame_id;
// 	// msg_odo_data.pose.pose.orientation.x = data->Quat[1];
// 	// msg_odo_data.pose.pose.orientation.y = data->Quat[2];
// 	// msg_odo_data.pose.pose.orientation.z = data->Quat[3];
// 	// msg_odo_data.pose.pose.orientation.w = data->Quat[0];

// 	// msg_odo_data.twist.twist.angular.x = data->Gyro[0];
// 	// msg_odo_data.twist.twist.angular.y = data->Gyro[1];
// 	// msg_odo_data.twist.twist.angular.z = data->Gyro[2];
// 	// msg_odo_data.twist.twist.linear.x = data->Acc[0];
// 	// msg_odo_data.twist.twist.linear.y = data->Acc[1];
// 	// msg_odo_data.twist.twist.linear.z = data->Acc[2];
// 	// context->publisher_ins_data->publish(msg_ins_data);

// 	msg_gps_data.header.stamp = timestamp;
// 	msg_gps_data.header.frame_id = context->imu_frame_id;
// 	msg_gps_data.latitude = data->LatGNSS;
// 	msg_gps_data.longitude = data->LonGNSS;
// 	msg_gps_data.altitude = data->AltGNSS;
// 	context->publisher_gps_data->publish(msg_gps_data);

// 	// if (context->publisher_sensor_data->get_subscription_count() > 0) {
// 		// msg_sensor_data.header.seq = seq;
// 		// msg_sensor_data.header.stamp = timestamp;
// 		// msg_sensor_data.header.frame_id = context->imu_frame_id;
// 		// msg_sensor_data.mag.x = data->Mag[0];
// 		// msg_sensor_data.mag.y = data->Mag[1]; // indices used to be 0...
// 		// msg_sensor_data.mag.z = data->Mag[2]; // 'fixed', assuming that was a bug.
// 		// msg_sensor_data.accel.x = data->Acc[0];
// 		// msg_sensor_data.accel.y = data->Acc[1];
// 		// msg_sensor_data.accel.z = data->Acc[2];
// 		// msg_sensor_data.gyro.x = data->Gyro[0];
// 		// msg_sensor_data.gyro.y = data->Gyro[1];
// 		// msg_sensor_data.gyro.z = data->Gyro[2];
// 		// msg_sensor_data.temp = data->Temp;
// 		// msg_sensor_data.vinp = data->VSup;
// 		// msg_sensor_data.pressure = data->hBar;
// 		// msg_sensor_data.barometric_height = data->pBar;
// 		// context->publisher_sensor_data->publish(msg_sensor_data);
// 	// }

// 	// if (context->publisher_ins_data->get_subscription_count() > 0) {
// 		// msg_ins_data.header.seq = seq;
// 		// msg_ins_data.header.stamp = timestamp;
// 		// msg_ins_data.header.frame_id = context->imu_frame_id;
// 		// msg_ins_data.ypr.x = data->Heading;
// 		// msg_ins_data.ypr.y = data->Pitch;
// 		// msg_ins_data.ypr.z = data->Roll;
// 		// msg_ins_data.oriquat.w = data->Quat[0];
// 		// msg_ins_data.oriquat.x = data->Quat[1];
// 		// msg_ins_data.oriquat.y = data->Quat[2];
// 		// msg_ins_data.oriquat.z = data->Quat[3];
// 		// msg_ins_data.llh.x = data->Latitude;
// 		// msg_ins_data.llh.y = data->Longitude;
// 		// msg_ins_data.llh.z = data->Altitude;
// 		// msg_ins_data.vel_enu.x = data->VelENU[0];
// 		// msg_ins_data.vel_enu.y = data->VelENU[1];
// 		// msg_ins_data.vel_enu.z = data->VelENU[2];
// 		// msg_ins_data.gps_ins_time = data->GPS_INS_Time;
// 		// msg_ins_data.gps_imu_time = data->GPS_IMU_Time;
// 		// msg_ins_data.gps_msow.data = data->ms_gps;
// 		// msg_ins_data.solution_status.data = data->INSSolStatus;
// 		// msg_ins_data.usw = data->USW;
// 		// msg_ins_data.pos_std.x = data->KFLatStd;
// 		// msg_ins_data.pos_std.y = data->KFLonStd;
// 		// msg_ins_data.pos_std.z = data->KFAltStd;
// 		// msg_ins_data.heading_std = data->KFHdgStd;
// 		// context->publisher_ins_data->publish(msg_ins_data);
// 	// }

// 	// if (context->publisher_gps_data->get_subscription_count() > 0) {
// 		// msg_gps_data.header.seq = seq;
// 		// msg_gps_data.header.stamp = timestamp;
// 		// msg_gps_data.header.frame_id = context->imu_frame_id;
// 		// msg_gps_data.llh.x = data->LatGNSS;
// 		// msg_gps_data.llh.y = data->LonGNSS;
// 		// msg_gps_data.llh.z = data->AltGNSS;
// 		// msg_gps_data.horspeed = data->V_Hor;
// 		// msg_gps_data.speeddir = data->Trk_gnd;
// 		// msg_gps_data.verspeed = data->V_ver;
// 		// context->publisher_gps_data->publish(msg_gps_data);
// 	// }

// 	// if (context->publisher_gnss_data->get_subscription_count() > 0) {
// 		// msg_gnss_data.header.seq = seq;
// 		// msg_gnss_data.header.stamp = timestamp;
// 		// msg_gnss_data.header.frame_id = context->imu_frame_id;
// 		// msg_gnss_data.gnss_info_1 = data->GNSSInfo1;
// 		// msg_gnss_data.gnss_info_2 = data->GNSSInfo2;
// 		// msg_gnss_data.number_sat = data->SVsol;
// 		// msg_gnss_data.gnss_velocity_latency = data->GNSSVelLatency;
// 		// msg_gnss_data.gnss_angles_position_type = data->AnglesType;
// 		// msg_gnss_data.gnss_heading = data->Heading_GNSS;
// 		// msg_gnss_data.gnss_pitch = data->Pitch_GNSS;
// 		// msg_gnss_data.gnss_gdop = data->GDOP;
// 		// msg_gnss_data.gnss_pdop = data->PDOP;
// 		// msg_gnss_data.gnss_hdop = data->HDOP;
// 		// msg_gnss_data.gnss_vdop = data->VDOP;
// 		// msg_gnss_data.gnss_tdop = data->TDOP;
// 		// msg_gnss_data.new_gnss_flags = data->NewGPS;
// 		// msg_gnss_data.diff_age = data->DiffAge;
// 		// msg_gnss_data.pos_std.x = data->LatGNSSStd;
// 		// msg_gnss_data.pos_std.y = data->LonGNSSStd;
// 		// msg_gnss_data.pos_std.z = data->AltGNSSStd;
// 		// msg_gnss_data.heading_std = data->HeadingGNSSStd;
// 		// msg_gnss_data.pitch_std = data->PitchGNSSStd;
// 		// context->publisher_gnss_data->publish(msg_gnss_data);
// 	// }

// 	// if (context->publisher_marine_data->get_subscription_count() > 0) {
// 		// msg_marine_data.header.seq = seq;
// 		// msg_marine_data.header.stamp = timestamp;
// 		// msg_marine_data.header.frame_id = context->imu_frame_id;
// 		// msg_marine_data.heave = data->Heave;
// 		// msg_marine_data.surge = data->Surge;
// 		// msg_marine_data.sway = data->Sway;
// 		// msg_marine_data.heave_velocity = data->Heave_velocity;
// 		// msg_marine_data.surge_velocity = data->Surge_velocity;
// 		// msg_marine_data.sway_velocity = data->Sway_velocity;
// 		// msg_marine_data.significant_wave_height = data->significant_wave_height;
// 		// context->publisher_marine_data->publish(msg_marine_data);
// 	// }
// }

class InertialLabsINS : public rclcpp::Node
{
public:
	InertialLabsINS()
	: Node("il_ins"),
	  imu_frame_id("imu_link"),
	  publisher_sensor_data(create_publisher<sensor_msgs::msg::Imu>("/imu/data", 1)),
	  publisher_odo_data(create_publisher<nav_msgs::msg::Odometry>("/odometry/data", 1)),
	  publisher_gps_data(create_publisher<sensor_msgs::msg::NavSatFix>("/gps/data", 1))
	{
		// Command line variables
		std::string port;
		if (!this->get_parameter("ins_url", port)) {
			// port = "serial:/dev/ttyUSB0:460800";
			port = "serial:/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_DO00H0RN-if00-port0:115200";
		}

		int ins_output_format;
		if (!this->get_parameter("ins_output_format", ins_output_format)) {
			// ins_output_format = 0x52;
			ins_output_format = 102; // Defaults to INS OPVT & Raw IMU data format...
		}

		RCLCPP_INFO(this->get_logger(), "connecting to INS at URL %s\n", port.c_str());

		int il_err = ins.connect(port.c_str());
		if (il_err != 0) {
			RCLCPP_FATAL(
				this->get_logger(),
				"Could not connect to the INS on this URL %s\n",
				port.c_str()
			);
			exit(EXIT_FAILURE);
		}

		// Check if the INS is started and stop it
		if (ins.isStarted()) {
			ins.stop();
		}

		// Get device information and parameters
		auto devInfo = ins.getDeviceInfo();
		auto devParams = ins.getDeviceParams();
		std::string SN(reinterpret_cast<const char *>(devInfo.IDN), 8);

		// Print device information
		RCLCPP_INFO(this->get_logger(), "Found INS S/N %s", SN.c_str());
		imu_frame_id = "imu_link";

		// Start the INS with the specified output format
		il_err = ins.start(ins_output_format);
		if (il_err != 0) {
			RCLCPP_FATAL(this->get_logger(), "Could not start the INS: %i", il_err);
			ins.disconnect();
			exit(EXIT_FAILURE);
		}

		// Set the callback function for publishing
		ins.setCallback(std::bind(&InertialLabsINS::publish_device, this, std::placeholders::_1));

		// Print information
		RCLCPP_INFO(this->get_logger(), "publishing at %d Hz", devParams.dataRate);
		RCLCPP_INFO(this->get_logger(), "rostopic echo the topics to see the data");
	}

	~InertialLabsINS()
	{
		// Stop the INS
		RCLCPP_INFO(this->get_logger(), "Stopping INS... ");
		ins.stop();

		// Disconnect from the INS
		RCLCPP_INFO(this->get_logger(), "Disconnecting... ");
		ins.disconnect();

		RCLCPP_INFO(this->get_logger(), "Done.");
	}

private:
	// Callback
	void publish_device(IL::INSDataStruct * data)
	{
		rclcpp::Time timestamp = rclcpp::Clock().now();

		sensor_msgs::msg::Imu msg_sensor_data;
		nav_msgs::msg::Odometry msg_odo_data;
		sensor_msgs::msg::NavSatFix msg_gps_data;

		msg_sensor_data.header.stamp = timestamp;
		msg_sensor_data.header.frame_id = imu_frame_id;

		// Need to convert the euler angles to quaternion ROS2:
		// Need to remap the domain of Pitch:
		double pitchDeg = data->Pitch;
		double rollDeg = data->Roll;
		double headingDeg = data->Heading;

		double deg2rad = M_PI / 180.0;
		double pitchRad = pitchDeg * deg2rad;
		double rollRad = rollDeg * deg2rad;
		double headingRad = headingDeg * deg2rad;

		RCLCPP_INFO(this->get_logger(), "Pitch: %f, Roll: %f, Heading: %f", pitchDeg, rollDeg, headingDeg);

		tf2::Quaternion q;
		q.setRPY(rollRad, pitchRad, headingRad);
		msg_sensor_data.orientation = tf2::toMsg(q);
		msg_sensor_data.orientation_covariance[0] = 0.01;
		msg_sensor_data.orientation_covariance[4] = 0.01;
		msg_sensor_data.orientation_covariance[8] = 0.01;

		// Angular velocity:
		// msg_sensor_data.angular_velocity.x = data->Gyro[0] * deg2rad; // * 0.01745329251
		// msg_sensor_data.angular_velocity.y = data->Gyro[1] * deg2rad; // * 0.01745329251
		// msg_sensor_data.angular_velocity.z = data->Gyro[2] * deg2rad; // * 0.01745329251

		// Angular velocity covariance:
		msg_sensor_data.angular_velocity_covariance[0] = -1.0; //0.0005;
		// msg_sensor_data.angular_velocity_covariance[4] = 0.0005;
		// msg_sensor_data.angular_velocity_covariance[8] = 0.0005;

		// Linear acceleration:
		// msg_sensor_data.linear_acceleration.x = data->Acc[0]; // * 78.610420842
		// msg_sensor_data.linear_acceleration.y = data->Acc[1]; // * 78.610420842
		// msg_sensor_data.linear_acceleration.z = data->Acc[2]; // * 78.610420842

		// Linear acceleration covariance:
		msg_sensor_data.linear_acceleration_covariance[0] = -1; //0.1000;
		// msg_sensor_data.linear_acceleration_covariance[4] = 0.1000;
		// msg_sensor_data.linear_acceleration_covariance[8] = 0.1000;
		publisher_sensor_data->publish(msg_sensor_data);

		msg_odo_data.header.stamp = timestamp;
		msg_odo_data.header.frame_id = imu_frame_id;
		msg_odo_data.twist.twist.linear.x = data->VelENU[0];
		msg_odo_data.twist.twist.linear.y = data->VelENU[1];
		msg_odo_data.twist.twist.linear.z = data->VelENU[2];
		publisher_odo_data->publish(msg_odo_data);

		msg_gps_data.header.stamp = timestamp;
		msg_gps_data.header.frame_id = imu_frame_id;
		msg_gps_data.latitude = data->LatGNSS;
		msg_gps_data.longitude = data->LonGNSS;
		msg_gps_data.altitude = data->AltGNSS;
		publisher_gps_data->publish(msg_gps_data);
	}

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_sensor_data;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_odo_data;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr publisher_gps_data;

	rclcpp::Rate loop_rate{100.0};
	IL::Driver ins;
	std::string imu_frame_id{"imu_link"};
};

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<InertialLabsINS>());
	rclcpp::shutdown();

	return 0;

	// NEW ^^
	// ---------
	// OLD:

	// ros::init(argc, argv, "il_ins");
	// ros::NodeHandle n;
	// ros::NodeHandle np("~");
	// ros::Rate r(100);       // 100 hz
	// std::string port;
	// IL::Driver ins;
	// int ins_output_format;
	// std::string imu_frame_id;
	// Context context;

	// Command line variables
	// np.param<std::string>("ins_url", port, "serial:/dev/ttyUSB0:460800");
	// np.param<int>("ins_output_format", ins_output_format, 0x52);

	// Initializing Publishers
	// context.publishers[0] = np.advertise<inertiallabs_msgs::sensor_data>(
	// 	"/Inertial_Labs/sensor_data",
	// 	1);
	// context.publishers[1] = np.advertise<inertiallabs_msgs::ins_data>("/Inertial_Labs/ins_data", 1);
	// context.publishers[2] = np.advertise<inertiallabs_msgs::gps_data>("/Inertial_Labs/gps_data", 1);
	// context.publishers[3] = np.advertise<inertiallabs_msgs::gnss_data>("/Inertial_Labs/gnss_data", 1);
	// context.publishers[4] = np.advertise<inertiallabs_msgs::marine_data>(
	// 	"/Inertial_Labs/marine_data",
	// 	1);

	// ROS_INFO("connecting to INS at URL %s\n", port.c_str());

	// auto il_err = ins.connect(port.c_str());
	// if (il_err != 0) {
	// 	ROS_FATAL(
	// 	"Could not connect to the INS on this URL %s\n",
	// 	port.c_str()
	// 	);
	// 	exit(EXIT_FAILURE);
	// }

	// if (ins.isStarted()) {
	// 	ins.stop();
	// }
	// auto devInfo = ins.getDeviceInfo();
	// auto devParams = ins.getDeviceParams();
	// std::string SN(reinterpret_cast<const char *>(devInfo.IDN), 8);
	// ROS_INFO("Found INS S/N %s\n", SN.c_str());
	// context.imu_frame_id = SN;
	// il_err = ins.start(ins_output_format);
	// if (il_err != 0) {
	// 	ROS_FATAL("Could not start the INS: %i\n", il_err);
	// 	ins.disconnect();
	// 	exit(EXIT_FAILURE);
	// }
	// ins.setCallback(&publish_device, &context);
	// ROS_INFO("publishing at %d Hz\n", devParams.dataRate);
	// ROS_INFO("rostopic echo the topics to see the data");
	// ros::spin();
	// std::cout << "Stopping INS... " << std::flush;
	// ins.stop();
	// std::cout << "Disconnecting... " << std::flush;
	// ins.disconnect();
	// std::cout << "Done." << std::endl;
	// return 0;
}
