#include <IMUReader.h>

#include <iostream>


bool IMUReader::init(std::string port_name)
{
	boost::system::error_code ec;
	serial_ = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(io_service_));
	serial_->open(port_name, ec);
	if (ec)
	{
		std::cerr << "wrong opening port: " << ec.message().c_str() << std::endl;
		return false;
	}
	else
	{
		serial_->set_option(boost::asio::serial_port_base::baud_rate(115200));
		serial_->set_option(boost::asio::serial_port_base::character_size(8));
		serial_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
		serial_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
		serial_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
		std::cerr << "open ok!\n"<<sizeof(int)<<std::endl;
	}

	t = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service_));
	

	recv_byte_ = 0;
	boost::this_thread::sleep(boost::posix_time::milliseconds(3000));

	
	IMUReader::SensorData data;
	getDataOnce(data);

	
	
	return true;
}

bool IMUReader::getDataOnce(IMUReader::SensorData &data)
{
	if (!serial_)
		return false;

	
	if (!serial_ready_)
		return false;
	
	
	
	char send_buffer[] = { ' ' };
	recv_byte_ = 0;

	serial_->write_some(boost::asio::buffer(send_buffer));

	while (recv_byte_ < 40)
	{
		recv_byte_+=serial_->read_some(boost::asio::buffer(buffer + recv_byte_, 40));

	}
	
	if (recv_byte_ == 40)
	{

		data.time = *(reinterpret_cast<unsigned int*>(buffer));
		data.gyro[0] = *(reinterpret_cast<float*>(buffer + 4)) / 180 * 3.1415926;
		data.gyro[1] = *(reinterpret_cast<float*>(buffer + 8)) / 180 * 3.1415926;
		data.gyro[2] = *(reinterpret_cast<float*>(buffer + 12)) / 180 * 3.1415926 - z_bias_;

		data.acce[0] = *(reinterpret_cast<float*>(buffer + 16));
		data.acce[1] = *(reinterpret_cast<float*>(buffer + 20));
		data.acce[2] = *(reinterpret_cast<float*>(buffer + 24));

		data.comp[0] = *(reinterpret_cast<float*>(buffer + 28));
		data.comp[1] = *(reinterpret_cast<float*>(buffer + 32));
		data.comp[2] = *(reinterpret_cast<float*>(buffer + 36));
	}
	
	serial_ready_ = true;
	return recv_byte_==40;

}

bool IMUReader::calibration(int iterN)
{
	IMUReader::SensorData data;
	float z_bias_tmp_ = 0.f;
	for (int i = 0; i < iterN; i++)
	{
		getDataOnce(data);
		z_bias_tmp_ += data.gyro[2] / static_cast<float>(iterN);
		
	}
	z_bias_ = z_bias_tmp_;
	std::cout << z_bias_ << std::endl;
	return true;
}