#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <unistd.h>
#include <boost/thread/thread.hpp>
#include <stdlib.h>
#include <string>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int32.h"
#include <std_msgs/Float64MultiArray.h>
#include <boost/bind.hpp>
#include <exception>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include <netinet/in.h>
#include <queue>
#include <boost/function.hpp>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>

using namespace boost::asio;

const double_t VOLTAJEREF = 5.0;
const uint16_t NUMOUTS = 16; //15
const uint16_t NUMINS  = 16; //16
int s = 1;
std::string msgio;
std::queue <int> colaRG;
std::queue <int> colaRT;
std::queue <int> actuadorRS;
std::string ret;
boost::mutex mutex;
int32_t currposition[6];
int32_t memPositions[156];
uint16_t velocidades[6];
uint16_t currVelocidad[6];
int32_t posdest[6];
uint MachineState;
bool moveDone[6];
std_msgs::Float64MultiArray MSGMOVED;

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

//adaptacion del mensaje *******************************************************
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
std::string uint16toLEStr(uint16_t& num){
  std::string sinteger;
  sinteger.append<int>(1,num % 256 );
  num=num/256;
  sinteger.append<int>(1,num % 256 );
  return sinteger;

}

//recepcion y envio a IOPort****************************************************
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
  // std::string debug="look above";
  // for (int xnt = 0; xnt < cmd.length(); xnt++) {
  //   std::cout << hex(cmd[xnt]) << " ";
  // }
  // std::cout<<std::endl;
  // ROS_INFO(cmd.c_str());
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

//apertura y mantenimiento******************************************************
bool openCommunication() {
  ROS_INFO("Opening communication");
  std::string ROmsg = "RO";
  std::string ROresp;
  ROmsg.append<int>(1, 0x00);
  sends(ROmsg);
  recv(ROresp);

  return ROresp.find(0x06, 0) != std::string::npos;
}
void KeepCommunication() {
  ROS_INFO("Keep communication");
  std::string RCmsg = "RC";
  RCmsg.append<int>(1, 0x01);
  RCmsg.append<int>(1, 0x00);
  RCmsg.append<int>(1, 0xff);
  while (ros::ok()) {
    sends(RCmsg);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
  }
}
void initialize(){



}

//extraccion de datos de la SCU*************************************************
void RGUserPositionDataS() {
  ROS_INFO("Asking for RGUserPositionDataS");
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x00);
  RCmsg.append<int>(1, 0x20);
  sends(RCmsg);
  colaRG.push(1);
}
void RGUserPositionData(int actuador) {
  ROS_INFO("Asking for RGUserPositionData",actuador);
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, actuador);
  RCmsg.append<int>(1, 0x20);
  sends(RCmsg);
  colaRG.push(1);
}
void RGDestPosS(){
  ROS_INFO("Asking for destiny positionS");
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x20);
  RCmsg.append<int>(1, 0x30);
  sends(RCmsg);
  colaRG.push(4);
}
void RGDestPos(int actuador) {
  ROS_INFO("Asking for destiny position",actuador);
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x20+actuador);
  RCmsg.append<int>(1, 0x30);
  sends(RCmsg);
  colaRG.push(4);
  boost::this_thread::sleep_for(boost::chrono::milliseconds(3500));
}
void RGvelS() {
  ROS_INFO("Asking for velocityS");
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x10);
  RCmsg.append<int>(1, 0x30);
  sends(RCmsg);
  colaRG.push(3);
}
void RGvel(int actuador) {
  ROS_INFO("Asking for velocity",actuador);
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x10+actuador);
  RCmsg.append<int>(1, 0x30);
  sends(RCmsg);
  colaRG.push(3);
}
void RGCurrPosS(){
      ROS_INFO("Asking for current positionS");
      std::string RCmsg = "RG";
      RCmsg.append<int>(1, 0x10);
      RCmsg.append<int>(1, 0x00);
      sends(RCmsg);
      colaRG.push(2);
}
void RGCurrPos(int actuador){
  ROS_INFO("Asking for destiny position",actuador);
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x10+actuador);
  RCmsg.append<int>(1, 0x00);
  sends(RCmsg);
  colaRG.push(2);
}
void RGactuatorFinishS(){
  ROS_INFO("Asking for move doneS");
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x70);
  RCmsg.append<int>(1, 0x01);
  sends(RCmsg);
  colaRG.push(5);
}
void RGactuatorFinish(int actuador){
  ROS_INFO("Asking for move done",actuador);
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0x70+actuador);
  RCmsg.append<int>(1, 0x01);
  sends(RCmsg);
  colaRG.push(5);
}
void RGactuatorSpeedS(){
  ROS_INFO("Asking for current velocityS");
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0xF0);
  RCmsg.append<int>(1, 0x00);
  sends(RCmsg);
  colaRG.push(6);
}
void RGactuatorSpeed(int actuador){
  ROS_INFO("Asking for current velocity",actuador);
  std::string RCmsg = "RG";
  RCmsg.append<int>(1, 0xF0+actuador);
  RCmsg.append<int>(1, 0x00);
  sends(RCmsg);
  colaRG.push(6);
}

//transferencia de datos a la SCU***********************************************
void RTDestPosS(int32_t pDest[6]){
  ROS_INFO("Transfering destiny positionS");
  using boost::this_thread::get_id;
  std::string RTmsg = "RT";
  std::string poDest;
  for(int32_t cnt=0; cnt<6;cnt++){
    poDest.append(int32toLEStr(pDest[cnt]));
  }
  RTmsg.append<int>(1,(poDest.length()+2)%256);
  RTmsg.append<int>(1,poDest.length()/256);
  RTmsg.append<int>(1,0x20);
  RTmsg.append<int>(1,0x30);
  RTmsg.append(poDest);
  sends(RTmsg);
  colaRT.push(2);
}
void RTDestPos(int actuador, int32_t pDest){
  ROS_INFO("Transfering destiny positionS",actuador);
  using boost::this_thread::get_id;
  std::string RTmsg = "RT";
  std::string poDest;
  poDest.append(int32toLEStr(pDest));
  RTmsg.append<int>(1,(poDest.length()+2)%256);
  RTmsg.append<int>(1,poDest.length()/256);
  RTmsg.append<int>(1,0x20+actuador);
  RTmsg.append<int>(1,0x30);
  RTmsg.append(poDest);
  sends(RTmsg);
  colaRT.push(2);
}
void RTVelS(uint16_t speed[6]){
  ROS_INFO("Transfering VelocityS");
  std::string RTmsg = "RT";
  std::string speedstr;
  for(int32_t cnt=0; cnt<6;cnt++){
    speedstr.append(uint16toLEStr(speed[cnt]));
  }

  RTmsg.append<int>(1,(speedstr.length()+2)%256);
  RTmsg.append<int>(1,speedstr.length()/256);
  RTmsg.append<int>(1,0x10);
  RTmsg.append<int>(1,0x30);
  RTmsg.append(speedstr);
  sends(RTmsg);
  colaRT.push(2);
}
void RTVel(int actuador, uint16_t speed){
  ROS_INFO("Transfering Velocity",actuador);
  std::string RTmsg = "RT";
  std::string speedstr;
  speedstr.append(uint16toLEStr(speed));
  RTmsg.append<int>(1,(speedstr.length()+2)%256);
  RTmsg.append<int>(1,speedstr.length()/256);
  RTmsg.append<int>(1,0x10+actuador);
  RTmsg.append<int>(1,0x30);
  RTmsg.append(speedstr);
  sends(RTmsg);
  colaRT.push(2);
}


//ejecutar y parar funcion******************************************************
void REMove(int actuador,int dato){
  ROS_INFO("Executing function",dato,actuador);
  std::string REmsg = "RE";
  REmsg.append<int>(1, actuador-1);
  REmsg.append<int>(1, dato);
  REmsg.append<int>(1, 0xff);
  sends(REmsg);
}
void RSMove(int actuador){
  ROS_INFO("Stopping function",actuador);
  std::string RSmsg = "RS";
  RSmsg.append<int>(1, actuador-1);
  RSmsg.append<int>(1, 0x01);
  actuadorRS.push(actuador);
  sends(RSmsg);
}
void moveVelocity(int actuator,int16_t speed,int32_t positiondestiny){
  std::string Rspeed = "RT";
  Rspeed.append<int>(1, 0x04);
  Rspeed.append<int>(1, 0x00);
  Rspeed.append<int>(1, 11+actuator);
  Rspeed.append<int>(1, 0x30);
  Rspeed.append<int>(1, speed%256);
  Rspeed.append<int>(1, speed/256);

}  //incompleted
void movetoPos(int actuador,uint16_t speed,int32_t positiondestiny){
  ROS_ERROR("Starting move command");
  int mask=0;
  if(actuador==1){mask=0x40;}else if(actuador==2){mask=0x10;}
  MachineState=MachineState | mask;
  std::cout<<MachineState<<std::endl;
  RTDestPos(actuador,positiondestiny);
  RTVel(actuador,speed);
  REMove(actuador,0x09);
  RGactuatorFinishS();
  boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  while(moveDone[actuador-1]){};
  RSMove(actuador);
}

//lectura de mensajes**********************************************************
void ROGotmsg(std::string Msg){
  if(Msg[2]!=0x06){
    ROS_ERROR("initialization rejected");
    std::cout<<"ErrorRO ";
    for (int xnt = 0; xnt < 5; xnt++) {
      std::cout<<(hex(Msg[xnt]))<<" ";
    }
    std::cout<<std::endl;
  }else{
    ROS_INFO("initialization completed");
  }
}
void RCGotmsg(std::string Msg){
  if(Msg[2]!=0x06){
    ROS_ERROR("Keep connection error");
    std::cout<<"ErrorRC ";
    for (int xnt = 0; xnt < 5; xnt++) {
      std::cout<<(hex(Msg[xnt]))<<" ";
    }
    std::cout<<std::endl;
  }else{
    ROS_INFO("Keep connection completed");
  }
}
void RTGotmsg(std::string Msg){
  if(Msg[2]!=0x06){
    ROS_ERROR("Could not transfer info");
    std::cout<<"ErrorRT ";
    for (int xnt = 0; xnt < 5; xnt++) {
      std::cout<<(hex(Msg[xnt]))<<" ";
    }
    std::cout<<std::endl;
  }else{
    ROS_INFO("Transfer completed");
    if (colaRT.front()==1) {
      ROS_INFO("Velocity succesfully updated");
    }else if (colaRT.front()==2) {
      ROS_INFO("Position succesfully updated");
    }else{
      ROS_ERROR("No info in transfer");
    }
  }
}
void REGotmsg(std::string Msg){
  if(Msg[2]!=0x06){
    ROS_ERROR("couldn not execute function");
    std::cout<<"ErrorRE ";
    for (int xnt = 0; xnt < 5; xnt++) {
      std::cout<<(hex(Msg[xnt]))<<" ";
    }
    std::cout<<std::endl;
  }else{
    ROS_INFO("Executing function");
  }
}
void RSGotmsg(std::string Msg){
  if(Msg[2]!=0x06){
    ROS_ERROR("couldn not stop function");
    std::cout<<"ErrorRE ";
    for (int xnt = 0; xnt < 5; xnt++) {
      std::cout<<(hex(Msg[xnt]))<<" ";
    }
    std::cout<<std::endl;
  }else{
    ROS_INFO("Stopping function");
    uint mask=0;
    if(actuadorRS.front()==1){mask=0xff-0x40;}else if(actuadorRS.front()==2){mask=0xff-0x10;}
    MachineState=MachineState & mask;
    std::cout<<MachineState<<std::endl;
  }
  actuadorRS.pop();
}
void RGGot1(std::string Msg, double n1, double n2){
  ROS_INFO("Users positions data recieved");
  for(int32_t cnt=0;cnt<(n1+n2)/(4);cnt++) {
    memPositions[cnt]=(Msg[5+cnt*4]<0?Msg[5+cnt*4]+256:Msg[5+cnt*4])
    +(Msg[6+cnt*4]<0?Msg[6+cnt*4]+256:Msg[6+cnt*4])*256
    +(Msg[7+cnt*4]<0?Msg[7+cnt*4]+256:Msg[7+cnt*4])*65536
    +(Msg[8+cnt*4]<0?Msg[8+cnt*4]+256:Msg[8+cnt*4])*16777216;
  }
}
void RGGot2(std::string Msg, double n1, double n2){
  ROS_INFO("Current positions recieved");
  for(int32_t cnt=0;cnt<(n1+n2)/(4);cnt++) {
    currposition[cnt]=(Msg[5+cnt*4]<0?Msg[5+cnt*4]+256:Msg[5+cnt*4])
    +(Msg[6+cnt*4]<0?Msg[6+cnt*4]+256:Msg[6+cnt*4])*256
    +(Msg[7+cnt*4]<0?Msg[7+cnt*4]+256:Msg[7+cnt*4])*65536
    +(Msg[8+cnt*4]<0?Msg[8+cnt*4]+256:Msg[8+cnt*4])*16777216;
  }

}
void RGGot3(std::string Msg, double n1, double n2){
  ROS_INFO("Actuator velocity recieved");
  for(int32_t cnt=0;cnt<(n1+n2)/(2);cnt++) {
    velocidades[cnt]=(Msg[5+cnt*2]<0?Msg[5+cnt*2]+256:Msg[5+cnt*2])
    +(Msg[6+cnt*2]<0?Msg[6+cnt*2]+256:Msg[6+cnt*2])*256;
  }
}
void RGGot4(std::string Msg, double n1, double n2){
  ROS_INFO("Memory positions recieved");
  for(int32_t cnt=0;cnt<(n1+n2)/(4);cnt++) {
    posdest[cnt]=(Msg[5+cnt*4]<0?Msg[5+cnt*4]+256:Msg[5+cnt*4])
    +(Msg[6+cnt*4]<0?Msg[6+cnt*4]+256:Msg[6+cnt*4])*256
    +(Msg[7+cnt*4]<0?Msg[7+cnt*4]+256:Msg[7+cnt*4])*65536
    +(Msg[8+cnt*4]<0?Msg[8+cnt*4]+256:Msg[8+cnt*4])*16777216;
      }
}
void RGGot5(std::string Msg, double n1, double n2){
  ROS_INFO("Actuators status recieved");
  for(int32_t cnt=0;cnt<(n1+n2);cnt++) {
    moveDone[cnt]=((Msg[5+cnt]<0?Msg[5+cnt]+256:Msg[5+cnt]) & ( 1 << 4 )) >> 4;
  }
}
void RGGot6(std::string Msg, double n1, double n2){
  ROS_INFO("Actuators current velocity recieved");
  for(int32_t cnt=0;cnt<(n1+n2)/2;cnt++) {
    currVelocidad[cnt]=(Msg[5+cnt*2]<0?Msg[5+cnt*2]+256:Msg[5+cnt*2])
    +(Msg[6+cnt*2]<0?Msg[6+cnt*2]+256:Msg[6+cnt*2])*256;
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
        if (msgio.substr(0, 2) == "RO") {
          ROGotmsg(msgio.substr(0,5));
          msgio.erase(0,5);
        }
        else if (msgio.substr(0, 2) == "RC") {
          RCGotmsg(msgio.substr(0,5));
          msgio.erase(0, 5);
        }
        else if (msgio.substr(0, 2) == "RT") {
          RTGotmsg(msgio.substr(0,5));
          msgio.erase(0, 5);
        }
        else if (msgio.substr(0, 2) == "RE") {
          REGotmsg(msgio.substr(0,5));
          msgio.erase(0, 5);
        }
        else if (msgio.substr(0, 2) == "RS") {
          RSGotmsg(msgio.substr(0,5));
          msgio.erase(0, 5);
        }
        else if (msgio.substr(0, 2) == "RG") {
          if(msgio[2]!=0x06){
            ROS_ERROR("Could not get info");
            std::cout<<"ErrorRG ";
            for (int xnt = 0; xnt < 5; xnt++) {
              std::cout<<(hex(msgio[xnt]))<<" ";
            }
            std::cout<<std::endl;
            msgio.erase(0,5);
          }
          else{
            double n1=(msgio[3]<0)?(double)msgio[3]+256:(double)msgio[3];
            double n2=((msgio[4]<0)?(double)msgio[4]+256:(double)msgio[4])*256;
            if((double)msgio.length()>=(n1+n2+7) ){

              std::string Msg =msgio.substr(0,7+n1+n2);
              if(colaRG.front()==1){
                RGGot1(Msg,n1,n2);
              }
              else if(colaRG.front()==2){
                RGGot2(Msg,n1,n2);
              }
              else if(colaRG.front()==3){
                RGGot3(Msg,n1,n2);
              }
              else if(colaRG.front()==4){
                RGGot4(Msg,n1,n2);
              }
              else if(colaRG.front()==5){
                RGGot5(Msg,n1,n2);
              }
              else if(colaRG.front()==6){
                RGGot6(Msg,n1,n2);
              }
              else{ROS_INFO("WTF unknown msg for RG");}
              colaRG.pop();
              msgio.erase(0,7+n1+n2);
            }else {
              notenough=false;
            }


          }


        }

        else {
          std::cout<<"Eroor**************************************************************************************" << std::endl;

          for (int xnt = 0; xnt < msgio.length(); xnt++) {
            std::cout << hex(msgio[xnt]) << " ";
          }
          std::cout << std::endl;
          msgio.erase(0, msgio.find('R', 4));
        }
      }
      notenough=true;
    }
  }
}



//~ void Callback(const std_msgs::Float64MultiArray::ConstPtr& msg)
void roslistener(const std_msgs::Float64MultiArray& ROSLdestposition){
  std::cout <<"Empiezo el intento"<< std::endl;
   for(uint16_t cnt=0;cnt<ROSLdestposition.data.size() ;cnt++){
    if(ROSLdestposition.data[cnt]!=0 && ROSLdestposition.data[cnt]!=MSGMOVED.data[cnt] ){
      uint mask=0;
      if(cnt==0){mask=0x40;}else if(cnt==1){mask=0x10;}
      std::cout<<MachineState<<std::endl;
      if(!(MachineState & mask)){
      int32_t gh=ROSLdestposition.data[cnt];
      boost::thread m (movetoPos,cnt+1,50,gh);
      }
    }
   }
   std::cout <<"Acabo el intento"<< std::endl;
  MSGMOVED.data=ROSLdestposition.data;
}



//funciones de prueba***********************************************************
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
void Velocidadoscilante() {
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
void Posicionoscilante() {
  using boost::this_thread::get_id;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
  int veldas=50;
  while (ros::ok()) {
    std::string RVel = "RT";
    RVel.append<int>(1, 0x06);
    RVel.append<int>(1, 0x00);
    RVel.append<int>(1, 0x21);
    RVel.append<int>(1, 0x30);
    RVel.append<int>(1, veldas);
    RVel.append<int>(1, 0x00);
    RVel.append<int>(1, 0x00);
    RVel.append<int>(1, 0x00);

    std::cout << "GETTO: "<<veldas<<"  ";
    send(RVel);
    colaRT.push(1);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
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
void holderM( boost::function<void(int,int16_t,int32_t)> funa, int freq){
  while (ros::ok()) {
    funa(1,100,posdest[0]+50);
    boost::this_thread::sleep_for(boost::chrono::milliseconds(5000));
  }
}
void holdervoid( boost::function<void()> funa){
  while (ros::ok()) {
    funa();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
  }
}
void rospublish(std_msgs::Int32MultiArray msg_v,ros::Publisher ROSPPosition,
  geometry_msgs::TransformStamped tfTLT[2],tf::TransformBroadcaster broadcaster,
  ros::Rate loop_rate){
  while(ros::ok){
  for(int32_t cnt=0;cnt<6;cnt++){
  msg_v.data.push_back(currposition[cnt]);
  }
  tfTLT[0].header.stamp = ros::Time::now();
  tfTLT[0].transform.translation.x = 0;
  tfTLT[0].transform.translation.y = 0;
  tfTLT[0].transform.translation.z= (double)currposition[0]/1000;
  tfTLT[0].transform.rotation = tf::createQuaternionMsgFromYaw(0);
  tfTLT[1].header.stamp = ros::Time::now();
  tfTLT[1].transform.translation.x = 0;
  tfTLT[1].transform.translation.y = 0;
  tfTLT[1].transform.translation.z=(double)currposition[1]/1000;
  tfTLT[1].transform.rotation = tf::createQuaternionMsgFromYaw(0);
  broadcaster.sendTransform(tfTLT[0]);
  broadcaster.sendTransform(tfTLT[1]);
  ROSPPosition.publish(msg_v);
  loop_rate.sleep();
  msg_v.data.clear();
  }
}
void roslistenerinit(ros::Subscriber ROSLPosition){
  while(ros::ok){
    try{
  ros::spin();
}catch(std::exception e){std::cout<<e.what();}
  }
}


    int main(int argc, char* argv[]) {

      ros::init(argc, argv, "talker");

      ros::NodeHandle nh;

      ros::Publisher ROSPPosition = nh.advertise<std_msgs::Int32MultiArray>("/TLT/pos", 1);

      ros::Rate loop_rate(1); //100

      ros::init(argc, argv, "listener");
      ros::Subscriber ROSLPosition = nh.subscribe("/TLT/dest", 1, roslistener);
      tf::TransformBroadcaster broadcaster;
      geometry_msgs::TransformStamped tfTLT[2];
      tfTLT[0].header.frame_id = "bot";
      tfTLT[0].child_frame_id = "mid";
      tfTLT[1].header.frame_id = "mid";
      tfTLT[1].child_frame_id = "top";

      ROS_DEBUG_NAMED("test_only", "Hello %s", "World");
      ROS_DEBUG_STREAM_NAMED("test_only", "Hello " << "World");

      ROS_DEBUG("HELO");


      std_msgs::String msg;
      std_msgs::Int32MultiArray msg_v;
      std::string rot;

      /*
      Todos estos parametros son del puerto serie sincronizados con el arduino
      */
      handle.set_option( serial_port_base::baud_rate(baudrate));
      handle.set_option( serial_port_base::character_size(8));
      handle.set_option( serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      handle.set_option( serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
      handle.set_option( serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));


      //boost::this_thread::sleep(boost::posix_time::seconds(2));  ///Espera un tiempo
      while(openCommunication()){}
      MachineState=MachineState | 0x80;
      for(int abc=0;abc<6;abc++){MSGMOVED.data.push_back(0);}


      boost::thread t1(KeepCommunication);
      boost::thread t2(readbd);
      boost::thread t3(holdervoid,RGCurrPosS);
      boost::thread t4(holdervoid,RGactuatorFinishS);
      boost::thread t5(holdervoid,RGDestPosS);
      boost::thread t6(holdervoid,RGactuatorSpeedS);
      boost::thread t7(rospublish,msg_v,ROSPPosition,tfTLT,broadcaster,loop_rate);
      boost::thread t8(roslistenerinit,ROSLPosition);

      t1.join();
      ROS_INFO("STOP");
      return 0;
    }
