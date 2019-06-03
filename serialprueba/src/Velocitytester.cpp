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
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <boost/bind.hpp>
#include <exception>
#include <boost/crc.hpp>
#include <boost/cstdint.hpp>
#include <boost/cstdfloat.hpp>
#include <netinet/in.h>
#include <queue>
#include <boost/function.hpp>
#include <tf/transform_broadcaster.h>
#include <ros/console.h>
#include <cstdlib>
#include "serialprueba/MachineState.h"
#include "serialprueba/CurrPos.h"
#include "serialprueba/CurrVel.h"
#include "serialprueba/ErrorCode.h"
#include <fstream>
#include <boost/chrono.hpp>

std::vector<boost::float64_t> pos[2];
std::vector<boost::float64_t> vel[2];
bool moving;
std_msgs::Float64MultiArray sendvel;
std_msgs::Float64MultiArray upper;
std_msgs::Float64MultiArray lower;
std::ofstream myfile;
std::ofstream myfile2;

 void testveldown(){
  while (pos[0].back()>0.290 || pos[1].back()>0.290){boost::this_thread::interruption_point();}
  pos[0].clear();
  vel[0].clear();
  pos[1].clear();
  vel[1].clear();
  moving=true;
  boost::chrono::steady_clock::time_point start= boost::chrono::steady_clock::now();
  boost::this_thread::sleep_for(boost::chrono::seconds(1));
  while (pos[0].back()>0.090 || pos[1].back()>0.090){boost::this_thread::interruption_point();}
  boost::chrono::duration<double> timer=boost::chrono::steady_clock::now()-start;
  std::cout<<timer.count()<<std::endl;
  myfile<<pos[0].front() <<" "<<pos[0].back()<< " " <<timer.count()<<" "<<((boost::float64_t)pos[0].front()-pos[0].back())/timer.count()<<"\n";
  myfile<<pos[1].front() <<" "<<pos[1].back()<< " " <<timer.count()<<" "<<((boost::float64_t)pos[1].front()-pos[1].back())/timer.count()<<"\n";
  myfile2<<((boost::float64_t)pos[0].front()-pos[0].back())/timer.count()<<" " <<((boost::float64_t)pos[1].front()-pos[1].back())/timer.count()<<"\n";
}
void testvelup(){
  while (pos[0].back()<0.010 || pos[1].back()<0.010){boost::this_thread::interruption_point();}
  pos[0].clear();
  vel[0].clear();
  pos[1].clear();
  vel[1].clear();
  moving=true;
  boost::chrono::steady_clock::time_point start= boost::chrono::steady_clock::now();
  boost::this_thread::sleep_for(boost::chrono::seconds(1));
  while (pos[0].back()<0.210 || pos[1].back()<0.210){boost::this_thread::interruption_point();}
  boost::chrono::duration<double> timer=boost::chrono::steady_clock::now()-start;
  std::cout<<timer.count()<<std::endl;
  myfile<<pos[0].front() <<" "<<pos[0].back()<< " " <<timer.count()<<" "<<((boost::float64_t)pos[0].front()-pos[0].back())/timer.count()<<"\n";
  myfile<<pos[1].front() <<" "<<pos[1].back()<< " " <<timer.count()<<" "<<((boost::float64_t)pos[1].front()-pos[1].back())/timer.count()<<"\n";
	myfile2<<((boost::float64_t)pos[0].front()-pos[0].back())/timer.count()<<" " << ((boost::float64_t)pos[1].front()-pos[1].back())/timer.count()<<"\n";
}

void roslistenerpos(std_msgs::Float64MultiArray posl){
  pos[0].push_back(posl.data[0]);
  std::cout<<posl.data[0]<<" "<<posl.data[1]<<std::endl;
  pos[1].push_back(posl.data[1]);
}
void roslistenervel(std_msgs::Float64MultiArray vell){
  vel[0].push_back(vell.data[0]);
  vel[1].push_back(vell.data[1]);
}
void rossPin(){
  while(ros::ok){
    try{
  ros::spin();
}
catch(std::exception e){std::cout<<e.what();}
  }
}


using namespace boost::asio;

int main(int argc, char* argv[]) {
  
  ros::init(argc, argv, "tryer");

  myfile.open("InformeVelocidad.txt",std::ofstream::out);
  myfile2.open("Data.txt",std::ofstream::out);

  upper.data.push_back(100);
  lower.data.push_back(-100);
  upper.data.push_back(100);
  lower.data.push_back(-100);

   boost::thread aksjdh(rossPin);

  boost::this_thread::sleep_for(boost::chrono::seconds(3));
  ros::NodeHandle nhj;
  ros::Subscriber ROSLPositionVels = nhj.subscribe("/TLT/pos", 10, roslistenerpos);
  ros::Subscriber ROSLVelocityVels = nhj.subscribe("/TLT/currvel", 10, roslistenervel);
  ros::Publisher ROSPPositionVels = nhj.advertise<std_msgs::Float64MultiArray>("/TLT/des", 1);
  ros::Publisher ROSPVelocityVels = nhj.advertise<std_msgs::Float64MultiArray>("/TLT/vel", 1);

  for(int16_t cnt=25;cnt<120;cnt=cnt+10){
    sendvel.data.push_back(-cnt);
    sendvel.data.push_back(-cnt);
    myfile<<"Probando Velocidad: -"<<cnt<<"\n";
    std::cout<<"Probando Velocidad: -"<<cnt<<std::endl;
	boost::this_thread::sleep_for(boost::chrono::seconds(3));
    while(pos[0].back()<0.290 || pos[1].back()<0.290){
    ROSPVelocityVels.publish(upper);
    boost::this_thread::sleep_for(boost::chrono::seconds(3));
    }
    ROSPVelocityVels.publish(sendvel);
    boost::thread tester(testveldown);
    boost::this_thread::sleep_for(boost::chrono::seconds(5));
    if(!moving){
      tester.interrupt();
      myfile<<"Didnt move \n";
      myfile2<<"0";
    }else{
    tester.join();
    }
    moving=false;
    sendvel.data.clear();
	sendvel.data.push_back(cnt);
    sendvel.data.push_back(cnt);
	std::cout<<"Probando Velocidad: "<<cnt<<std::endl;
	boost::this_thread::sleep_for(boost::chrono::seconds(3));
    while(pos[0].back()>0.010 || pos[1].back()>0.010){
    ROSPVelocityVels.publish(lower);
    boost::this_thread::sleep_for(boost::chrono::seconds(3));
	}
	ROSPVelocityVels.publish(sendvel);
    tester=boost::thread(testvelup);
    boost::this_thread::sleep_for(boost::chrono::seconds(5));
    if(!moving){
      tester.interrupt();
      myfile<<"Didnt move \n";
      myfile2<<"0";
    }else{
    tester.join();
    }
    moving=false;
    sendvel.data.clear();
}
myfile.close();
  return 0;
}
