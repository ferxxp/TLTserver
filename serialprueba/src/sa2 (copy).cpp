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
#include <netinet/in.h>
#include <boost/endian/conversion.hpp>
#include <queue>

using namespace boost::asio;

const double_t VOLTAJEREF = 5.0;
const uint16_t NUMOUTS = 16; //15
const uint16_t NUMINS  = 16; //16
int s = 1;
std::string msgio;
std::queue <int> colaRG;
std::queue <int> colaRT;
double nbits1=0;
double nbits2=0;
std::string ret;
boost::mutex mutex;
int32_t position[6];
int32_t memPositions[156];
uint16_t velocidades[6];
int32_t posdest[6];
bool moveDone[6];

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
std::string int32toLEStr(int32_t num){
  std::string sinteger;
  sinteger.append<int>(1,num % 256 );
  num=num/256;
  sinteger.append<int>(1,num % 256 );
  num=num/256;
  sinteger.append<int>(1,num % 256 );
  num=num/256;
  sinteger.append<int>(1,num % 256 );
  return sinteger;

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
bool sends(std::string cmd) {
  addCRC_ccitt(cmd);


  try {
    mutex.lock();
    boost::asio::write(handle, boost::asio::buffer(cmd.c_str(), cmd.size()));
    mutex.unlock();
  } catch (std::exception &e) {
    std::cerr << "Could not send command '" << cmd << "' towards port " << port << "." << std::endl;
    return false;
  }
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

    //std::cout << "thread 1: ";
    sends(RCmsg);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
  }
}
void Buzz() {
  using boost::this_thread::get_id;
  std::string REmsg = "RE";
  REmsg.append<int>(1, 0x00);
  REmsg.append<int>(1, 0x09);
  REmsg.append<int>(1, 0xff);
  std::string RSmsg = "RS";
  RSmsg.append<int>(1, 0x00);
  RSmsg.append<int>(1, 0xff);
  std::string RE1msg = "RE";
  RE1msg.append<int>(1, 0x01);
  RE1msg.append<int>(1, 0x09);
  RE1msg.append<int>(1, 0xff);
  std::string RS1msg = "RS";
  RS1msg.append<int>(1, 0x01);
  RS1msg.append<int>(1, 0xff);
  std::string REmsg2 = "RE";
  REmsg2.append<int>(1, 0x00);
  REmsg2.append<int>(1, 0x01);
  REmsg2.append<int>(1, 0xff);
  std::string RSmsg2 = "RS";
  RSmsg2.append<int>(1, 0x00);
  RSmsg2.append<int>(1, 0xff);
  std::string RE1msg2 = "RE";
  RE1msg2.append<int>(1, 0x01);
  RE1msg2.append<int>(1, 0x01);
  RE1msg2.append<int>(1, 0xff);
  std::string RS1msg2 = "RS";
  RS1msg2.append<int>(1, 0x01);
  RS1msg2.append<int>(1, 0xff);
  while (ros::ok()) {

    std::cout << "thread 2: ";
    send(REmsg);
    send(RE1msg);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
    std::cout << "thread 2: ";
    send(RSmsg);
    send(RS1msg);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
    std::cout << "thread 2: ";
    send(REmsg2);
    send(RE1msg2);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
    std::cout << "thread 2: ";
    send(RSmsg2);
    send(RS1msg2);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
  }
}
void RGCurrPos() {
  using boost::this_thread::get_id;
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x00);
  RCmsg.append<int>(1, 0x20);

  while (ros::ok()) {

    std::cout << "GETTO: ";
    send(RCmsg);
    colaRG.push(1);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(3500));

  }
}
void RGDestPos() {
  using boost::this_thread::get_id;
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x20);
  RCmsg.append<int>(1, 0x30);

  while (ros::ok()) {

    std::cout << "GETTO: ";
    send(RCmsg);
    colaRG.push(4);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(3500));

  }
}
void RGvel() {
  using boost::this_thread::get_id;
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x10);
  RCmsg.append<int>(1, 0x30);

  while (ros::ok()) {

    std::cout << "GETTO: ";
    send(RCmsg);
    colaRG.push(3);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(3500));

  }
}
void RTVel() {
  using boost::this_thread::get_id;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
  int veldas=50;
  while (ros::ok()) {
    std::string RVel = "RT";
    RVel.append<int>(1, 0x04);
    RVel.append<int>(1, 0x00);
    RVel.append<int>(1, 0x11);
    RVel.append<int>(1, 0x30);
    RVel.append<int>(1, veldas);
    RVel.append<int>(1, 0x00);

    std::cout << "GETTO: "<<veldas<<"  ";
    send(RVel);
    colaRT.push(1);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
    veldas=veldas+1;
    if(veldas<50){veldas=50;}

  }
}
void Movepruebas() {
  using boost::this_thread::get_id;
  while (ros::ok()) {
    for(int drive=0;drive<10;drive++){
      for(int potition=0;potition<10;potition++){

        std::string RCmsg = "RE";
        RCmsg.append<int>(1, drive);
        RCmsg.append<int>(1, potition);
        RCmsg.append<int>(1, 0xff);
        std::string RSmsg = "RS";
        RSmsg.append<int>(1, drive);
        RSmsg.append<int>(1, 0x01);



        std::cout << "thread 2 move: ";
        send(RCmsg);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
        std::cout << "thread 2 Stop: ";
        send(RSmsg);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
      }}}
    }
    void getData(){
      using boost::this_thread::get_id;
      std::string RCmsg = "RG";
      RCmsg.append<int>(1, 0x10);
      RCmsg.append<int>(1, 0x00);

      while (ros::ok()) {

        //std::cout << "thread 4: ";
        sends(RCmsg);
        colaRG.push(2);

        boost::this_thread::sleep_for(boost::chrono::milliseconds(2000));
      }
    }
    void moveVelocity(int actuator,int16_t speed,int32_t positiondestiny){
      std::string Rspeed = "RT";
      Rspeed.append<int>(1, 0x04);
      Rspeed.append<int>(1, 0x00);
      Rspeed.append<int>(1, 11+actuator);
      Rspeed.append<int>(1, 0x30);
      Rspeed.append<int>(1, speed%256);
      Rspeed.append<int>(1, speed/256);

    }
    void movetoPos(int actuator,uint16_t speed,int32_t positiondestiny){
      std::string Rspeed = "RT";
      Rspeed.append<int>(1, 0x04);
      Rspeed.append<int>(1, 0x00);
      Rspeed.append<int>(1, 0x11);
      Rspeed.append<int>(1, 0x30);
      Rspeed.append<int>(1, speed%256);
      Rspeed.append<int>(1, speed/256);

    }
    void actuatorFinish(){
      using boost::this_thread::get_id;
      std::string RCmsg = "RG";
      RCmsg.append<int>(1, 0x70);
      RCmsg.append<int>(1, 0x01);
      while (ros::ok()) {
        std::cout << "stateActuators: ";
        send(RCmsg);
        colaRG.push(5);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(300));
      }

    }
    void readbd() {
      using boost::this_thread::get_id;
      bool notenough=true;
      recv(ret);
      msgio.append(ret);
      msgio.erase(0,msgio.find_first_of("R"));
      while (ros::ok()){
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
        if (recv(ret)) {
          msgio.append(ret);
          //std::cout << get_id() << ":  "<<std::endl;


          while (msgio.find('R', 0) != std::string::npos and msgio.length()>=5 and notenough) {
            if (msgio.substr(0, 2) == "RO" and msgio[2]==0x06) {
              std::cout << "iniciado: ";

              for (int xnt = 0; xnt < 5; xnt++) {
                std::cout << hex(msgio[xnt]) << " ";
              }
              std::cout << std::endl;
              msgio.erase(0, 5);
            }
            else if (msgio.substr(0, 2) == "RT" and msgio[2]==0x06) {
              if(colaRT.front()==1){std::cout << "Velocidad aceptada: "<<std::endl;}
              else if(colaRT.front()==2){std::cout << "Posicion aceptada: "<<std::endl;}
              colaRT.pop();
              msgio.erase(0, 5);
            }
            else if ( msgio.substr(0, 2) == "RE" and msgio[2]==0x06) {
              std::cout << "ejecutando: ";
              for (int xnt = 0; xnt < 5; xnt++) {
                std::cout << hex(msgio[xnt]) << " ";
              }
              std::cout << std::endl;
              msgio.erase(0, 5);
            }
            else if (msgio.substr(0, 2) == "RC" and msgio[2]==0x06) {

              //std::cout << "manteniendo: ";
              //for (int xnt = 0; xnt < 5; xnt++) {
              //	std::cout << hex(msgio[xnt]) << " ";
              //	}
              //	std::cout << std::endl;
              msgio.erase(0, 5);
            }
            else if (msgio.substr(0, 2) == "RG"and msgio[2]==0x06) {
              nbits1=(msgio[3]<0)?(double)msgio[3]+256:(double)msgio[3];
              nbits2=((msgio[4]<0)?(double)msgio[4]+256:(double)msgio[4])*256;
              if((double)msgio.length()>=(nbits1+nbits2+7) ){
                if(colaRG.front()==1){
                  colaRG.pop();
                  std::cout << "MEM: "<<(double)msgio.length()<<" "<<nbits1+nbits2+7<<" ; ";
                  std::cout << std::endl;
                  for(int32_t cnt=0;cnt<(nbits1+nbits2)/(4);cnt++) {
                    memPositions[cnt]=(msgio[5+cnt*4]<0?msgio[5+cnt*4]+256:msgio[5+cnt*4])
                    +(msgio[6+cnt*4]<0?msgio[6+cnt*4]+256:msgio[6+cnt*4])*256
                    +(msgio[7+cnt*4]<0?msgio[7+cnt*4]+256:msgio[7+cnt*4])*65536
                    +(msgio[8+cnt*4]<0?msgio[8+cnt*4]+256:msgio[8+cnt*4])*16777216;
                  }msgio.erase(0, 7+nbits1+nbits2);
                }else if(colaRG.front()==2){
                  colaRG.pop();
                  std::cout << "POS: "<<(double)msgio.length()<<" "<<nbits1+nbits2+7<<" ; ";

                  for (double xnt = 0; xnt < 7+nbits1+nbits2; xnt++) {
                    std::cout << hex(msgio[xnt]) << " ";
                  } std::cout<<std::endl;
                  for(int32_t cnt=0;cnt<(nbits1+nbits2)/(4);cnt++) {
                    position[cnt]=(msgio[5+cnt*4]<0?msgio[5+cnt*4]+256:msgio[5+cnt*4])
                    +(msgio[6+cnt*4]<0?msgio[6+cnt*4]+256:msgio[6+cnt*4])*256
                    +(msgio[7+cnt*4]<0?msgio[7+cnt*4]+256:msgio[7+cnt*4])*65536
                    +(msgio[8+cnt*4]<0?msgio[8+cnt*4]+256:msgio[8+cnt*4])*16777216;

                    std::cout<<"Pos "<<cnt<<":"<<(double)position[cnt]<<" mm "<<position[cnt]<<std::endl;
                  }
                  msgio.erase(0, 7+nbits1+nbits2);
                }else if(colaRG.front()==3){
                  colaRG.pop();
                  std::cout << "VEL: "<<(double)msgio.length()<<" "<<nbits1+nbits2+7<<" ; ";

                  for (double xnt = 0; xnt < 7+nbits1+nbits2; xnt++) {
                    std::cout << hex(msgio[xnt]) << " ";
                  } std::cout<<std::endl;
                  for(int32_t cnt=0;cnt<(nbits1+nbits2)/(2);cnt++) {
                    velocidades[cnt]=(msgio[5+cnt*2]<0?msgio[5+cnt*2]+256:msgio[5+cnt*2])
                    +(msgio[6+cnt*2]<0?msgio[6+cnt*2]+256:msgio[6+cnt*2])*256;

                    std::cout<<"vel "<<cnt<<":"<<(double)velocidades[cnt]<<" mm "<<velocidades[cnt]<<std::endl;
                  }
                  msgio.erase(0, 7+nbits1+nbits2);
                }else if(colaRG.front()==4){
                  colaRG.pop();
                  std::cout << "Posdest: "<<(double)msgio.length()<<" "<<nbits1+nbits2+7<<" ; ";

                  for (double xnt = 0; xnt < 7+nbits1+nbits2; xnt++) {
                    std::cout << hex(msgio[xnt]) << " ";
                  } std::cout<<std::endl;
                  for(int32_t cnt=0;cnt<(nbits1+nbits2)/(4);cnt++) {
                    posdest[cnt]=(msgio[5+cnt*4]<0?msgio[5+cnt*4]+256:msgio[5+cnt*4])
                    +(msgio[6+cnt*4]<0?msgio[6+cnt*4]+256:msgio[6+cnt*4])*256
                    +(msgio[7+cnt*4]<0?msgio[7+cnt*4]+256:msgio[7+cnt*4])*65536
                    +(msgio[8+cnt*4]<0?msgio[8+cnt*4]+256:msgio[8+cnt*4])*16777216;

                    std::cout<<"posdest "<<cnt<<":"<<(double)posdest[cnt]<<" mm "<<posdest[cnt]<<std::endl;
                  }
                  msgio.erase(0, 7+nbits1+nbits2);
                }else if(colaRG.front()==5){
                  colaRG.pop();
                  std::cout << "Posdest: "<<(double)msgio.length()<<" "<<nbits1+nbits2+7<<" ; ";

                  for (double xnt = 0; xnt < 7+nbits1+nbits2; xnt++) {
                    std::cout << hex(msgio[xnt]) << " ";
                  } std::cout<<std::endl;
                  for(int32_t cnt=0;cnt<(nbits1+nbits2);cnt++) {
                    std::cout<<"Status "<<(msgio[5+cnt]<0?msgio[5+cnt]+256:msgio[5+cnt]);
                    moveDone[cnt]=((msgio[5+cnt]<0?msgio[5+cnt]+256:msgio[5+cnt]) & ( 1 << 4 )) >> 4;
                    std::cout << moveDone[cnt] <<std::endl;
                  }
                  msgio.erase(0, 7+nbits1+nbits2);
                }
              }else {
                notenough=false;
              }

            }
            else if (msgio.substr(0, 2) == "RS" and msgio[2]==0x06) {

              std::cout << "Parada funcion "<< std::endl;
              msgio.erase(0, 5);
            }
            else {
              std::cout<<"Eroor**************************************************************************************" << std::endl;

              for (int xnt = 0; xnt < msgio.length(); xnt++) {
                std::cout << hex(msgio[xnt]) << " ";
              }
              std::cout << std::endl;
              msgio.erase(0, msgio.find('R', 1));
            }
          }
          notenough=true;
        }
      }
    }
    void sendDestPos(int32_t pDest[6]){
      using boost::this_thread::get_id;
      std::string RTmsg = "RT";
      std::string poDest;
      for(int32_t cnt=0; cnt<6;cnt++){
        poDest.append(int32toLEStr(pDest[cnt]+5));
      }
      RTmsg.append<int>(1,(poDest.length()+2)%256);
      RTmsg.append<int>(1,poDest.length()/256);
      RTmsg.append<int>(1,0x20);
      RTmsg.append<int>(1,0x30);
      RTmsg.append(poDest);
      send(RTmsg);
      colaRT.push(2);
    }
    void holder(){
      while (ros::ok()) {
        sendDestPos(posdest);
        boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
      }
    }

    //~ void Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
    void Callback(const std_msgs::String::ConstPtr& msg){
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

void initialize(){



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
