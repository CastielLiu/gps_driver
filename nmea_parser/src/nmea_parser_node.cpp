
#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <iostream>
#include <gps_msgs/Gpchc.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define DEG2RAD 0.017453292

class GpsDriver
{
public:
	GpsDriver()
	{
		msg_ = new uint8_t[500];
	}
	~GpsDriver()
	{
		delete [] msg_;
	}
	
	bool openSerial(const std::string& port,int baudrate)
	{
		serial_ = new serial::Serial(port,baudrate,serial::Timeout::simpleTimeout(10)); 

		if (!serial_->isOpen())
		{
			std::stringstream output;
		    output << "Serial port: " << port << " failed to open." << std::endl;
			delete serial_;
			serial_ = NULL;
			return false;
		} 
		else 
		{
			std::stringstream output;
			output << "Serial port: " << port << " opened successfully." << std::endl;
		}

		serial_->flush();
		return true;
	}

	void closeSerial()
	{
		if(serial_ != NULL)
		{
			delete serial_;
			serial_ = NULL;
		}
	}
	
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
	
		pub_gpchc_ = nh.advertise<gps_msgs::Gpchc>(nh_private.param<std::string>("gpchc_topic","/gps/gpchc"), 10);
		pub_imu_   = nh.advertise<sensor_msgs::Imu>(nh_private.param<std::string>("imu_topic","/gps/imu"), 10);
		pub_twist_ = nh.advertise<geometry_msgs::TwistStamped>(nh_private.param<std::string>("velocity_topic","/gps/vel"), 10);
		pub_fix_   = nh.advertise<sensor_msgs::NavSatFix>(nh_private.param<std::string>("navsatfix_topic","/gps/fix"), 10);
		
		std::string port_name = nh_private.param<std::string>("port_name","/dev/ttyUSB0");
		int baudrate = nh_private.param<int>("baudrate",230400);
		if(!openSerial(port_name,baudrate))
			return false;
			
		start_time_ = ros::Time::now().toSec();
		return true;
	}
	
	void parseGpchc()
	{
		gps_msgs::Gpchc gpchc;
		gpchc.header.stamp = ros::Time::now();
		
		char *tok;
		//time
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.GPSWeek = atoi(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.GPSTime = atof(tok);
		
		//attitude
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Heading = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Pitch = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Roll = atof(tok);
		
		//3d-gyro
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.gyro_x = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.gyro_y = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.gyro_z = atof(tok);
		
		//3d-accelerate
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.acc_x = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.acc_y = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.acc_z = atof(tok);
		
		//location
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Lattitude = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Longitude = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Altitude = atof(tok);
		
		//velocity
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Ve = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Vn = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Vu = atof(tok);
		
		//antenna
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Baseline = atof(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.NSV1 = atoi(tok);
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.NSV2 = atoi(tok);
		
		
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Status = atoi(tok);
		
		tok = strtok(NULL, ",");
		if(tok == NULL) return;
		gpchc.Age = atoi(tok);
		
		tok = strtok(NULL, "*");
		if(tok == NULL) return;
		gpchc.Warming = atoi(tok);
		pub_gpchc_.publish(gpchc);
		
		sensor_msgs::NavSatFix fix;
		fix.header.stamp = gpchc.header.stamp;
		fix.header.frame_id = "imu_link";
		fix.latitude = gpchc.Lattitude;
		fix.longitude = gpchc.Longitude;
		fix.altitude = gpchc.Altitude;
		pub_fix_.publish(fix);
		
		geometry_msgs::TwistStamped twist_stamped;
		twist_stamped.header.stamp = gpchc.header.stamp;
		twist_stamped.header.frame_id = "imu_link";
		twist_stamped.twist.linear.x = gpchc.Ve;
		twist_stamped.twist.linear.y = gpchc.Vn;
		twist_stamped.twist.linear.z = gpchc.Vu;
		twist_stamped.twist.angular.x = gpchc.gyro_x;
		twist_stamped.twist.angular.y = gpchc.gyro_y;
		twist_stamped.twist.angular.z = gpchc.gyro_z;
		pub_twist_.publish(twist_stamped);
		
		sensor_msgs::Imu imu;
		imu.header.stamp = gpchc.header.stamp;
		imu.header.frame_id = "imu_link";
		tf2::Quaternion q;
		q.setRPY(gpchc.Roll*DEG2RAD, gpchc.Pitch*DEG2RAD, gpchc.Heading*DEG2RAD);
		q.normalize();
		imu.orientation = tf2::toMsg(q);
		imu.angular_velocity.x = gpchc.gyro_x;
		imu.angular_velocity.y = gpchc.gyro_y;
		imu.angular_velocity.z = gpchc.gyro_z;
		imu.linear_acceleration.x = gpchc.acc_x;
		imu.linear_acceleration.y = gpchc.acc_y;
		imu.linear_acceleration.z = gpchc.acc_z;
		pub_imu_.publish(imu);
		
	}
	
	void msgClassify(int len)
	{
//		if(ros::Time::now().toSec() - start_time_ > 120.0)
//		{
//			ROS_ERROR("gps driver trial timeout...");
//			return;
//		}
			
		
		char *tok = strtok((char*)msg_, ",");
		if(std::string(tok) == "$GPCHC")
			parseGpchc();
//		else if(std::string(tok) == "$GPGGA")
//			parseGpgga();
	}
	
	void parseDataFlow(const uint8_t* buf, int len)
	{
		static int index = 0;
		
		for(int i=0; i<len; ++i)
		{
			uint8_t c = buf[i];
			if(index == 0)
			{
				if('$' == c)
					msg_[index++] = c;
			}
			else
			{
				msg_[index++] = c;
				if('\n' == c)
				{
					msgClassify(index);
					index = 0;
				}
			}
			
		}
	}
	
	void run()
	{
		const int max_len = 200;
		uint8_t buf[max_len];
		ros::Rate loop_rate(50);
		
		int len;
		while(ros::ok())
		{
			try
			{
				len = serial_->read(buf, max_len);
			}
			catch(std::exception &e)
			{
				ROS_ERROR("Error reading from serial port: %s",e.what());
			}
			
			if(len > 0)
				parseDataFlow(buf, len);
			
			loop_rate.sleep();
		}
	}


private:
	serial::Serial *serial_;
	ros::Publisher pub_gpchc_;
	ros::Publisher pub_imu_;
	ros::Publisher pub_twist_;
	ros::Publisher pub_fix_;
	
	uint8_t *msg_;
	double start_time_;
};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"gps_driver_node");
	
	GpsDriver driver;
	
	if(driver.init())
		driver.run();
	
	return 0;
}



