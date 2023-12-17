#include <unistd.h>
#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <cerrno>
#include <string>
#include <cstdio>
#include "../../SerialPort.h"
#include <rclcpp/rclcpp.hpp>

namespace IL {

	SerialPort::SerialPort()
		: fd(-1)
		, timeout(1000)
	{
	}

	SerialPort::~SerialPort()
	{
		close();
	}

	int SerialPort::open(const char* url)
	{
		std::string urlStr(url);
		std::string pathStr;
		tcflag_t baudrate;
		size_t pos = urlStr.find(':');
		if (pos != std::string::npos) {
			std::string param = urlStr.substr(pos + 1);
			if (param == "4800") baudrate = B4800;
			else if (param == "9600") baudrate = B9600;
			else if (param == "19200") baudrate = B19200;
			else if (param == "38400") baudrate = B38400;
			else if (param == "57600") baudrate = B57600;
			else if (param == "115200") baudrate = B115200;
			else if (param == "230400") baudrate = B230400;
			else if (param == "460800") baudrate = B460800;
			else if (param == "921600") baudrate = B921600;
			else if (param == "2000000") baudrate = B2000000;
			else  return 2;
			pathStr = urlStr.substr(0, pos);
		}
		else {
			baudrate = B115200;
			pathStr = urlStr;
		}

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "SerialPort.opening... %s...", pathStr.c_str());

		fd = ::open(pathStr.c_str(), O_NOCTTY | O_RDWR);
		if (fd < 0) return -errno;
		struct termios config;
		if (tcgetattr(fd, &config) < 0)	return 1;
		cfmakeraw(&config);
		config.c_cflag = baudrate & ~(CSIZE | PARENB);
		config.c_cflag |= CS8 | CREAD | CLOCAL;
		config.c_lflag |= IEXTEN;
		if (tcsetattr(fd, TCSANOW, &config) < 0) return 3;
		return 0;
	}

	bool SerialPort::isOpen()
	{
		return fd >= 0;
	}

	void SerialPort::close()
	{
		if (isOpen())
		{
			::close(fd);
			fd = -1;
		}
	}

	int SerialPort::read(char* buf, unsigned int size)
	{
    	// Check if the file descriptor is associated with a terminal device
		if (!isatty(fd)) return -1;

		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Checked file descriptor...");

		
		pollfd fds; // Declare a pollfd structure to monitor the file descriptor
		fds.fd = fd; // Set the file descriptor to monitor		
		fds.events = POLLIN; // Set the events to monitor (POLLIN for data to read)
		fds.revents = 0; // Clear the revents field

		// Call poll to wait for an event on the file descriptor
		// The function will wait for the specified timeout
		int result = poll(&fds, 1, timeout);

		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Poll result: %d", result);

		// If poll returns a negative value, an error occurred
		// Return the negated error number
		if (result < 0) return -errno;

		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GOT HERE 8888: %d", result);

		// If poll returns 0, the timeout expired without any events
		// Return 0 to indicate no data was read
		if (!result) return 0;

		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "GOT HERE 9999: %d", result);

		// Read data from the file descriptor into the buffer
		// The amount of data read is the lesser of the size of the buffer and the amount of data available
		result = ::read(fd, buf, size);

		// RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read result: %d", result);

		// If read returns a negative value, an error occurred
		// Return the negated error number
		if (result < 0) return -errno;

		// Return the number of bytes read
		return result;

		// if (!isatty(fd)) return -1;
		// pollfd fds;
		// fds.fd = fd;
		// fds.events = POLLIN;
		// fds.revents = 0;
		// int result = poll(&fds, 1, timeout);
		// if (result < 0)	return -errno;
		// if (!result) return 0;
		// result = ::read(fd, buf, size);
		// if (result < 0)	return -errno;
		// return result;
	}

	int SerialPort::write(char* buf, unsigned int size)
	{
		int result = ::write(fd, buf, size);
		if (result < 0)	return -errno;
		return result;
	}
}