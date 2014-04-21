#ifndef IMUREADER_H
#define IMUREADER_H

#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/system/error_code.hpp>
#include <boost/thread.hpp>

#include <iostream>

#include <MadgwickAHRS.h>

class IMUReader
{
public:

	//IMU data format
	struct SensorData
	{
		unsigned long time;
		float gyro[3];
		float acce[3];
		float comp[3];
		friend std::ostream & operator<<(std::ostream &s, const SensorData &data) {
			s << data.time << "," << data.gyro[0] << "," << data.gyro[1] << "," << data.gyro[2] << ","\
				<< data.acce[0] << "," << data.acce[1] << "," << data.acce[1] << "," \
				<< data.comp[0] << "," << data.comp[1] << "," << data.comp[2] << std::endl;
			return s;
		}

	};
public:
	IMUReader() :ahrs_(5.0 / 1000000,0.1f),z_bias_(0.f){ recv_byte_ = 0; serial_ready_ = true; };
	void stop()
	{

		if (serial_&&serial_->is_open())
		{
			serial_->cancel();
			serial_->close();
			serial_.reset();
		}
		
		io_service_.stop();
		io_service_.reset();
		
	
		
	}
	//initialize the Serial_Port, the IMU module is provided by jiangtao, arduino driver is needed
	//the communication protocol: 
	//  1. Host send a character to IMU
	//  2. IMU send a frame of data
	bool init(std::string port_name);

	//get one frame of IMU data
	bool getDataOnce(SensorData &data);

	//run this right after init(), to avoid drift of z axis.
	bool calibration(int);
	bool isBusy()
	{
		return !serial_ready_;
	}
	AHRS::MadgwickAHRS ahrs_;

private:
	

	boost::asio::io_service io_service_;
	boost::shared_ptr<boost::asio::serial_port> serial_;
	int recv_byte_;
	
	float z_bias_;
	unsigned char buffer[40];
	
	bool serial_ready_,data_ready_;
	
	boost::thread t;

	

	
};
#endif
