#ifndef _MTDEVICE_H__
#define _MTDEVICE_H__ 1

#include <ros/ros.h>
#include <math.h>
#include <sensor_msgs/Imu.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include <boost/shared_ptr.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <cplus_xsens_driver/MtType.hpp>
#include <cplus_xsens_driver/mt_message.h>
#include <cplus_xsens_driver/packet_counter.h>


#define MAX_MESSAGE_LENGTH 128
#define BAG_ABANDON 800

typedef unsigned char uchar;

namespace xsens_device
{
struct MTMessage{
	uchar bid;
	uchar mid;
	uchar len;
	uchar data[MAX_MESSAGE_LENGTH];
	uchar checksum;
};
struct MTPacket{
	uchar id[2];       
	uchar len;
	uchar data[MAX_MESSAGE_LENGTH];
};
struct MTData2Data{
	uint32_t packet_counter;
	uint32_t sample_time_fine;
	uint32_t sample_time_coarse;
	double sample_time;
	uint32_t realtime_sec;
	uint32_t realtime_nsec;
	double realtime;
	float quaternion[4];
	float acceleration[3];
	float turn_rate[3];
};
class MTDevice
{
public:
     MTDevice();
     ~MTDevice();
     void parse();
     MTMessage message_;
     MTData2Data mtdata2_;
    
     ros::NodeHandle nh;
     ros::Publisher imu_data_pub_;
     ros::Publisher imu_packet_counter_pub_ ; 

     sensor_msgs::Imu imu_message_;
     cplus_xsens_driver::packet_counter packet_counter_;	

private:
     void read();
     void parseMTPacket(const MTPacket& packet);
     void receiveCallback(const boost::system::error_code &ec, size_t bytes_transferred);
     float getData2Float(const uchar *src);
     boost::asio::io_service io_srv_;
     boost::shared_ptr<boost::asio::serial_port> port_;
     boost::shared_ptr<boost::thread> thrd_io_srv_;

     boost::circular_buffer<uchar>  buf_;
     uchar raw_buf_[MAX_MESSAGE_LENGTH];

     ros::Time begin_;
     double diff_tosec;
     int step_;
     int begin_num_flag;
     int begin_flag_;
     int pub_flag_;
     int packet_counter_begin;
		
     uint32_t loop_;
     uint32_t short_counter_;
};


MTDevice::MTDevice():buf_(8 * MAX_MESSAGE_LENGTH, 0), step_(0)
    {
        buf_.clear();

        loop_ = 0;
        short_counter_ = 0;

        imu_data_pub_ = nh.advertise<sensor_msgs::Imu>("imu_data",100);
        imu_packet_counter_pub_ = nh.advertise<cplus_xsens_driver::packet_counter>("packet_counter",100);

        port_.reset(new boost::asio::serial_port(io_srv_, "/dev/ttyUSB0"));
        port_->set_option(boost::asio::serial_port_base::baud_rate(460800));
        port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

        boost::asio::basic_serial_port<boost::asio::serial_port_service >::native_type native = port_->native();
        struct serial_struct serial;
        ioctl(native,TIOCGSERIAL,&serial);
        serial.flags |= ASYNC_LOW_LATENCY;
        ioctl(native,TIOCSSERIAL,&serial);

        io_srv_.post(boost::bind(&MTDevice::read,this));
        thrd_io_srv_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &io_srv_)));
        begin_num_flag = 1;
        begin_flag_ = 1;
        pub_flag_ = 0;

    }
    
MTDevice::~MTDevice() {port_->close();}


float MTDevice::getData2Float(const uchar* src){
	float ret;
	uint8_t* dest = (uint8_t*) &ret;
	dest[0] = src[3];
	dest[1] = src[2];
	dest[2] = src[1];
	dest[3] = src[0];

	return ret;
}

void MTDevice::read()
{
  if (port_.get() == NULL || !port_->is_open()) {
    std::cout << "Port not opened\n" << std::endl;
    return;
  }
  port_->async_read_some(
      boost::asio::buffer(raw_buf_, sizeof(raw_buf_)),
      boost::bind(&MTDevice::receiveCallback, this,
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));
}

    void MTDevice::parse() {
        size_t old_size = 0;
        
        /*
        * step:
        * 0: Preamble, BID, MID & LEN
        * 1: DATA & Checksum
        */

        //check all of the message readed
        if(buf_.empty())
        {
            ROS_INFO("EMPTY");
        }
        while (!buf_.empty() && old_size != buf_.size()) {
            old_size = buf_.size();
            switch (step_) {
            case 0:
                while (buf_.size() >= 4 && buf_.front() != 0xfa){
                    buf_.pop_front();
                }
                if (buf_.front() == 0xfa && buf_.size() >= 4) {
                    buf_.pop_front();
                    message_.bid = buf_.front();
                    buf_.pop_front();
                    message_.mid = buf_.front();
                    buf_.pop_front();
                    message_.len = buf_.front();
                    buf_.pop_front();

                    if (message_.len == 0xff) {
                        printf("Extened format not supported\n");
                        step_ = 0;
                    }

                    step_ = 1;
                }
                break;
            case 1:
                if (buf_.size() > message_.len) {
                    for (int i = 0; i < message_.len; ++i) {
                        message_.data[i] = buf_.front();
                        buf_.pop_front();
                    }

                    message_.checksum = buf_.front();
                    buf_.pop_front();

                    int checksum = 0;
                    checksum += message_.bid;
                    checksum += message_.mid;
                    checksum += message_.len;
                    for (int i = 0; i < message_.len; ++i)
                        checksum += message_.data[i];
                    checksum += message_.checksum;
                    if ((checksum & 0xff) != 0x00) {
                        ROS_DEBUG("/** Wrong checksum:%02x **/\n", checksum);
                        ROS_DEBUG("BID=%02x, MID=%02x, len=%02x(%d), checksum=%02x\n", message_.bid, message_.mid, message_.len, message_.len, message_.checksum);
                        for (int i = 0; i < message_.len; ++i) {
                            printf("%02x ", message_.data[i]);
                        }
                        printf("\n");
                        printf("/** END **/\n");
                    }
                    else {
                        for (int i = 0; i < message_.len;) {
                            MTPacket packet;
                            if (message_.len - i < 2) {
                                printf("not enough id\n");
                                break;
                            }
                            packet.id[0] = message_.data[i++];
                            packet.id[1] = message_.data[i++];

                            if (message_.len - i < 1) {
                                printf("not eough len\n");
                                break;
                            }
                            packet.len = message_.data[i++];

                            if (message_.len - i < packet.len) {
                                printf("not enough data: %d - 1 - %d, %d\n", message_.len, i, packet.len);
                                break;
                            }
                            memcpy(packet.data, &message_.data[i], packet.len);
                            i += packet.len;

                            parseMTPacket(packet);
   
                        }   
                    }
                    step_ = 0;
                }
                break;
            default:
                printf("\n");
                break;
            }
        }
    }

    void MTDevice::receiveCallback(const boost::system::error_code &ec, size_t bytes_transferred) {
        if (ec) {
            std::cout << "read error: " << ec.message() << std::endl;
            read();
            return;
        }else{
		ROS_DEBUG("bytes_transferred=%lu",bytes_transferred);
	}

        for (unsigned int i = 0; i < bytes_transferred; ++i) {
            buf_.push_back(raw_buf_[i]);
        }
        parse();
        read();
    }

    void MTDevice::parseMTPacket(const MTPacket& packet) {
        enum xsens_imu::GroupId group;

        if (message_.mid == xsens_imu::kMtData2)     
        {
            group = xsens_imu::GroupId(packet.id[0]);

            if (group == xsens_imu::GroupId::kTimestamp){
                if (packet.id[1] == 0x20) {
                    mtdata2_.packet_counter = packet.data[0] * 256 + packet.data[1];
                    if ( mtdata2_.packet_counter < short_counter_ )
                        loop_++;
                    short_counter_ = mtdata2_.packet_counter;
                    
                    packet_counter_.packet_counter = (loop_ << 16) | mtdata2_.packet_counter;
                    
                    ROS_INFO ("packet_counter=%d", packet_counter_.packet_counter);
                    if (begin_num_flag)
                    {
                        packet_counter_begin = packet_counter_.packet_counter;
                        begin_num_flag = false;
                    }

                    //It's not stable at first, so abandon some packets at beginning and then publish the topic.
                    if ((packet_counter_.packet_counter - packet_counter_begin) > BAG_ABANDON )
                        pub_flag_ = true;
                }

                //Sample Time Fine
                if ((packet.id[1]&0xF0) == 0x60) {
                        mtdata2_.sample_time_fine = ((uint32_t)packet.data[0] << 24) + 
                                    ((uint32_t)packet.data[1] << 16) + 
                                    ((uint32_t)packet.data[2] <<  8) + 
                                    ((uint32_t)packet.data[3] <<  0);
                }
                //Sample Time Coarse
                if ((packet.id[1]&0xF0) == 0x70)
                {
                    //since the system time is not accurate, it's better to use the sample time from IMU.
                     mtdata2_.sample_time_coarse = ((uint32_t)packet.data[0] << 24) + 
                                      ((uint32_t)packet.data[1] << 16) + 
                                      ((uint32_t)packet.data[2] <<  8) + 
                                      ((uint32_t)packet.data[3] <<  0);
		mtdata2_.sample_time =
		    (mtdata2_.sample_time_coarse + (mtdata2_.sample_time_fine % 10000) / 10000.0);
                    if ( pub_flag_ && begin_flag_)
                    {
                        begin_ = ros::Time::now();
                        diff_tosec = begin_.toSec() - mtdata2_.sample_time;
                        begin_flag_ = false;
                    }
                    mtdata2_.realtime = mtdata2_.sample_time + diff_tosec;
                    mtdata2_.realtime_sec = (uint32_t)floor(mtdata2_.realtime);
                    mtdata2_.realtime_nsec = (mtdata2_.realtime - mtdata2_.realtime_sec)*pow(10,9);
                    packet_counter_.head.stamp.sec = mtdata2_.realtime_sec;
                    packet_counter_.head.stamp.nsec = mtdata2_.realtime_nsec;
                    packet_counter_.sample_fine_time = mtdata2_.sample_time;
		    // Fill packet counter
		    int loop = std::round((400 * packet_counter_.sample_fine_time - mtdata2_.packet_counter) / (1 << 16));
		    mtdata2_.packet_counter += loop * (1 << 16);
		    packet_counter_.packet_counter = mtdata2_.packet_counter;
		    ROS_DEBUG("packet_counter=%d", packet_counter_.packet_counter);
                }
            }

            if (group == xsens_imu::GroupId::kOrientationData )
            {
                if ((packet.id[1]&0xF0) == 0x10){
                    mtdata2_.quaternion[0] = getData2Float(packet.data);
                    mtdata2_.quaternion[1] = getData2Float(packet.data+4);
                    mtdata2_.quaternion[2] = getData2Float(packet.data+8);
                    mtdata2_.quaternion[3] = getData2Float(packet.data+12);
                }
                else
                {
                    ROS_INFO("no orientation data\n");
                }
            
            }

            if (group == xsens_imu::GroupId::kAcceleration)
            {	
                if ((packet.id[1]&0xF0) == 0x20)
                {
                    mtdata2_.acceleration[0] = getData2Float(packet.data);
                    mtdata2_.acceleration[1] = getData2Float(packet.data+4);
                    mtdata2_.acceleration[2] = getData2Float(packet.data+8);
                }
                else
                {
                    ROS_INFO("no acceleration data\n");
                }
                
            }

            if (group == xsens_imu::GroupId::kAngularVelocity)
            {
                if ((packet.id[1]&0xF0) == 0X20)
                {
                    mtdata2_.turn_rate[0] = getData2Float(packet.data);
                    mtdata2_.turn_rate[1] = getData2Float(packet.data+4);
                    mtdata2_.turn_rate[2] = getData2Float(packet.data+8);

                }
                else
                {
                    ROS_INFO("NO GRY DATA\n");
                }
            }

            if (packet.id[0] == xsens_imu::GroupId::kStatus)
            {
                if ((packet.id[1]&0xF0) == 0X20)
                {
                    imu_message_.header.stamp = packet_counter_.head.stamp;
                    imu_message_.header.frame_id = "xsens_link";
                    imu_message_.orientation.x = mtdata2_.quaternion[0];
                    imu_message_.orientation.y = mtdata2_.quaternion[1];
                    imu_message_.orientation.z = mtdata2_.quaternion[2];
                    imu_message_.orientation.w = mtdata2_.quaternion[3];

                    imu_message_.linear_acceleration.x = mtdata2_.acceleration[0];
                    imu_message_.linear_acceleration.y = mtdata2_.acceleration[1];
                    imu_message_.linear_acceleration.z = mtdata2_.acceleration[2];

                    imu_message_.angular_velocity.x = mtdata2_.turn_rate[0];
                    imu_message_.angular_velocity.y = mtdata2_.turn_rate[1];
                    imu_message_.angular_velocity.z = mtdata2_.turn_rate[2];
                    
                    if (pub_flag_)
                    {
                        imu_data_pub_.publish(imu_message_);
                        imu_packet_counter_pub_.publish(packet_counter_);
                    }
                }
            }
        }	


    }
}// namespace xsens_device

#endif // !_MTDEVICE_H__

