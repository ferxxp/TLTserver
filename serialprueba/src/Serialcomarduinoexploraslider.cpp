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

using namespace boost::asio;
#define CRC16 0x1021
const double_t VOLTAJEREF = 5.0;
const uint16_t NUMOUTS = 16; //15
const uint16_t NUMINS  = 16; //16
int s = 1;
std::string msgio;
double p1 = 0;
double p2 = 0;

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

 unsigned short crc16(const unsigned char* data_p, unsigned char length){
    unsigned char x;
    unsigned short crc = 0x1021;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
}
static const unsigned short CRC_TABLE[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7
,  0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
,  0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6
,  0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE
,  0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485
,  0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D
,  0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4
,  0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC
,  0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823
,  0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B
,  0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12
,  0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A
,  0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41
,  0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49
,  0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70
,  0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78
,  0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F
,  0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067
,  0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E
,  0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256
,  0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D
,  0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405
,  0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C
,  0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634
,  0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB
,  0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3
,  0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A
,  0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92
,  0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9
,  0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1
,  0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8
,  0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

///Configuracion del puerto serie
std::string port = "/dev/ttyACM0";                                      /*!< COMM port name.                  */
int         baudrate = 38400;                                              /*!< COMM baud rate.                  */
io_service  io;                                                            /*!< Internal communication.          */
serial_port handle(io, port);                                              /*!< Serial port.                     */
//
//unsigned short CalculateChecksum(const unsigned char* pAdr, int len)
//{
//  if (len < 0)
//  {
//    ASSERT(FALSE);
//    return 0;
//  }
//  unsigned short crc = 0;
//  while (len--)
//  {
//    crc = static_cast<unsigned short>(CRC_TABLE[((crc >> 8) ^ *pAdr++) &
//                                      0xFF] ^ (crc << 8));
//  }
//  return crc;
//}



bool send(std::string cmd) {
  try {
    boost::asio::write(handle, boost::asio::buffer(cmd.c_str(), cmd.size()));
  } catch (std::exception &e) {
    std::cerr << "Could not send command '" << cmd << "' towards port " << port << "." << std::endl;
    return false;
  }
  std::cout << "Enviando: " << cmd << std::endl;

  return true;
}

bool recv(std::string& ret) {

  bool isOk = true;
  boost::asio::streambuf resp;
  boost::asio::read_until(handle, resp, '\0');
  boost::asio::streambuf::const_buffers_type buf = resp.data();
  ret = std::string(boost::asio::buffers_begin(buf), boost::asio::buffers_begin(buf) + resp.size());

  return isOk;
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

  std::string ret;
  std_msgs::String msg;
  std_msgs::Float64MultiArray msg_v;

  /*
     Todos estos parametros son del puerto serie sincronizados con el arduino
  */


  handle.set_option( serial_port_base::baud_rate( baudrate ) );
  handle.set_option( serial_port_base::character_size( 8 ) );
  handle.set_option( serial_port_base::parity(boost::asio::serial_port_base::parity::none));
  handle.set_option( serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  handle.set_option( serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));

  std::string INIT = "Preparando el sistema...";
  send(INIT);
  boost::this_thread::sleep(boost::posix_time::seconds(2));  ///Espera un tiempo

  while (ros::ok()) {

    ///WRITE MODE
    std::string test = "CA";
    send(test); ///enviar al arduino lo que reciba del subscriptor

    ///READ MODE
    recv(ret); ///recibe del arduino y publicas ret, que es de tipo String
    std::cout << " tamaño ret es: " << ret.size() << std::endl;
    msgio.append(ret);
    //std::cout << "ret es: "<< ret<<std::endl;


    while (msgio.find('\0') != std::string::npos) {

      try {
	msgio.erase(0, msgio.find_first_of('C'));
	std::cout << msgio.substr(0, 2) << std::endl;
        p1=((double)msgio[2])*256;
	p2=((double)msgio[3]<0?(double)msgio[3]+256:(double)msgio[3]);
        std::cout<<"p1*256: " << p1 << std::endl;
	std::cout<<"p2:" << p2 << std::endl;
	std::cout<<"total: " << p1+p2 << std::endl;
	for(int xnt=0;xnt<msgio.length();xnt++){
		std::cout<<hex(msgio[xnt]);	
	}
	std::cout<<std::endl;
	//std::cout<<crc16(msgio,msgio.length())<<std::endl;
	
        msgio.erase(0, 6);


      }
      catch (std::exception e) {
        std::cout << "error"<<e.what() << std::endl;
      }
	msgio.erase(0, msgio.find_first_of('\0') + 1);
    }


    if (true) {
      msg_v.data.push_back(p1+p2);///Dentro del vector msg_v tengo que añadir los valores que vaya convirtiendo de string a double


      voltaje_pub.publish(msg_v);
      msg_v.data.clear(); /// limpio el vector para que cada vez no sea más grande

    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

