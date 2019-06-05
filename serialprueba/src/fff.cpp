#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64MultiArray.h>
#include <boost/bind.hpp>
#include <exception>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>



using namespace boost::asio;

const double_t VOLTAJEREF = 5.0;
const uint16_t NUMOUTS = 16; //15
const uint16_t NUMINS  = 16; //16
int s = 1;
std::string msgio;
double p1 = 0;
double p2 = 0;
double p3 = 0;
double p4 = 0;
std::string ret;
boost::mutex mutex;

struct HexCharStruct
{
  unsigned char c;
  HexCharStruct(unsigned char _c) : c(_c) { }
};

inline std::ostream& operator<<(std::ostream& o, const HexCharStruct& hs)
{
  return (o << std::hex << (int)hs.c);
}

inline HexCharStruct hex(unsigned char _c)
{
  return HexCharStruct(_c);
}


///Configuracion del puerto serie
std::string port = "/dev/ttyUSB0";                                      /*!< COMM port name.                  */
int         baudrate = 38400;                                              /*!< COMM baud rate.                  */
io_service  io;                                                            /*!< Internal communication.          */
serial_port handle(io, port);                                              /*!< Serial port.                     */
//

void addCRC_ccitt(std::string &msgcrc) {
  boost::crc_basic<16>  crc_ccitt1( 0x1021, 0x0000, 0, false, false );
  crc_ccitt1.process_bytes(msgcrc.data(), msgcrc.length());
  msgcrc.append(1, (int)crc_ccitt1.checksum() % 256);
  msgcrc.append(1, (int)crc_ccitt1.checksum() / 256);

}

bool send(std::string cmd) {
  addCRC_ccitt(cmd);


  try {
	mutex.lock();
    boost::asio::write(handle, boost::asio::buffer(cmd.c_str(), cmd.size()));
	mutex.unlock();
  } catch (std::exception &e) {
    std::cerr << "Could not send command '" << cmd << "' towards port " << port << "." << std::endl;
    return false;
  }
  std::cout << "Enviando: ";
  for (int xnt = 0; xnt < cmd.length(); xnt++) {
    std::cout << hex(cmd[xnt]) << " ";
  }
  std::cout << std::endl;

  return true;
}

bool recv(std::string& ret) {
  bool isOk = false;
  try {
    boost::asio::streambuf resp;
    boost::asio::read_until(handle, resp, 'R');
    boost::asio::streambuf::const_buffers_type buf = resp.data();
    ret = std::string(boost::asio::buffers_begin(buf), boost::asio::buffers_begin(buf) + resp.size());
    isOk = true;
  }
  catch (std::exception e) {
    std::cout << isOk << "error read " << e.what() << std::endl;
  }
  return isOk;
}
bool openCommunication() {
  std::string ROmsg = "RO";
  std::string ROresp;
  ROmsg.append<int>(1, 0x00);
  send(ROmsg);
  recv(ROresp);

  return ROresp.find(0x06, 0) != std::string::npos;
}

void KeepCommunication() {
  using boost::this_thread::get_id;
  std::string RCmsg = "RC";
  RCmsg.append<int>(1, 0x01);
  RCmsg.append<int>(1, 0x00);
  RCmsg.append<int>(1, 0xff);
  while (ros::ok()) {

    std::cout << "thread 1: ";
    send(RCmsg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
  }
}
void Buzz() {
  using boost::this_thread::get_id;
  std::string RCmsg = "RE";
  RCmsg.append<int>(1, 0x0a);
  RCmsg.append<int>(1, 0xff);
  RCmsg.append<int>(1, 0xff);
  std::string RSmsg = "RS";
  RSmsg.append<int>(1, 0x0a);
  RSmsg.append<int>(1, 0xff);
  RSmsg.append<int>(1, 0xff);
  while (ros::ok()) {

    std::cout << "thread 2: ";
    send(RCmsg);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(300));
    std::cout << "thread 2: ";
    send(RSmsg);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
  }
}
void getData(){
using boost::this_thread::get_id;
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x10);
  RCmsg.append<int>(1, 0x00);

  while (ros::ok()) {

    std::cout << "thread 4: ";
    send(RCmsg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
  }
}

void readbd() {
  using boost::this_thread::get_id;
  while (ros::ok()){
boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    if (recv(ret)) {
      msgio.append(ret);
      std::cout << get_id() << ":  "<<std::endl;
	
      
      while (msgio.find('R', 0) != std::string::npos and msgio.length()>=5) {

	
        if (msgio.substr(0, 2) == "RO" and msgio[2]==0x06) {
          std::cout << "iniciado: ";
          msgio.erase(0, 5);
          for (int xnt = 0; xnt < 5; xnt++) {
            std::cout << hex(msgio[xnt]) << " ";
          }
          std::cout << std::endl;
        }
        else if ( msgio.substr(0, 2) == "RE" and msgio[2]==0x06) {
          std::cout << "ejecutando: ";
	for (int xnt = 0; xnt < 5; xnt++) {
        std::cout << hex(msgio[xnt]) << " ";
      }
      std::cout << std::endl;
          msgio.erase(0, 5);
        } else if (msgio.substr(0, 2) == "RC" and msgio[2]==0x06) {
          
	std::cout << "manteniendo: ";
	for (int xnt = 0; xnt < 5; xnt++) {
        	std::cout << hex(msgio[xnt]) << " ";
      		}
		std::cout << std::endl;
		msgio.erase(0, 5);
        }else if (msgio.substr(0, 2) == "RG"and msgio[2]==0x06) {
          
	std::cout << "Info: ";
	for (int xnt = 0; xnt < 7+(int)msgio[3]; xnt++) {
        	std::cout << hex(msgio[xnt]) << " ";
      		}
		std::cout << std::endl;
		
	for(int cnt=0;cnt<(int)msgio[3]/4;cnt++) {
	std::cout<<(int32_t)msgio[5+4*cnt]<<std::endl;
	std::cout<<(int32_t)msgio[6+4*cnt]*256<<std::endl;
	std::cout<<(int32_t)msgio[7+4*cnt]*512<<std::endl;
	std::cout<<(int32_t)msgio[8+4*cnt]*1024<<std::endl;
	   }
	msgio.erase(0, 7+(int)msgio[3]);
        
	}else {
	std::cout<<"Eroor" << std::endl;
		
		for (int xnt = 0; xnt < msgio.length(); xnt++) {
        	std::cout << hex(msgio[xnt]) << " ";
      		}
		std::cout << std::endl;
          msgio.erase(0, msgio.find('R', 1));
        }
      }
    }
  }
}

//~ void Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
void Callback(const std_msgs::String::ConstPtr& msg) //PARA RECIBIR UN TIPO STRING
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  //  if (msg->data.size() >= NUMOUTS){
  //    send(msg->data.substr(0, NUMOUTS));
  //  }
  send(msg->data);

  //~ char msgChar[NUMOUTS];
  //~ for (uint16_t k = 0; k < NUMOUTS; k++) {
  //~ msgChar[k] = (char) (msg->data[k]*255.0/5.0 + 1.0);
  //~ }
  //~ std::to_string()
  //~ ROS_INFO("I heard: [%s]", msgChar);
  //~ send(std::string(msgChar)); //Le mando el tipo String
}

int main(int argc, char* argv[]) {

  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;

  ros::Publisher voltaje_pub = nh.advertise<std_msgs::Float64MultiArray>("/arduino/inputs", 1);

  ros::Rate loop_rate(100); //100

  ros::init(argc, argv, "listener");
  ros::Subscriber sub = nh.subscribe("/arduino/outputs", 1000, Callback);


  std_msgs::String msg;
  std_msgs::Float64MultiArray msg_v;
  std::string rot;

  /*
     Todos estos parametros son del puerto serie sincronizados con el arduino
  */


  handle.set_option( serial_port_base::baud_rate( baudrate ) );
  handle.set_option( serial_port_base::character_size( 8 ) );
  handle.set_option( serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  handle.set_option( serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  handle.set_option( serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));


  //boost::this_thread::sleep(boost::posix_time::seconds(2));  ///Espera un tiempo
  while(openCommunication()){}
  
  boost::thread t1(KeepCommunication);
  //boost::thread t2(Buzz);
  boost::thread t3(readbd);
  boost::thread t4(getData);
  t1.join();

  //t2.join();





  while (ros::ok()) {

    KeepCommunication();
    if (recv(ret)) {
      msgio.append(ret);
      bool check = true;
      while (check) {
        try {
          msgio.erase(0, msgio.find_first_of('R'));


          for (int xnt = 0; xnt < msgio.length(); xnt++) {
            std::cout << hex(msgio[xnt]) << " ";
          }
          std::cout << std::endl;
          msgio.erase(0, 2);
          for (int xnt = 0; xnt < msgio.length(); xnt++) {
            std::cout << hex(msgio[xnt]) << " ";
          }
          std::cout << std::endl;
          check = (msgio.find_first_of('R') != std::string::npos);
          std::cout << check << std::endl;

        }
        catch (std::exception e) {
          std::cout << "error" << e.what() << std::endl;
        }
      }
      rot = "RE";
      rot.append<int>(1, 0x01);
      rot.append<int>(1, 0x02);
      rot.append<int>(1, 0xff);
      send(rot);

      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}

