/*
 * acoustic_modem_state.h
 *
 *  Created on: 20 Jul 2014
 *      Author: nick
 */

#ifndef ACOUSTIC_MODEM_STATE_H_
#define ACOUSTIC_MODEM_STATE_H_

#include <vector>
#include <set>
#include <ros/ros.h>
#include <acomms_sim/AcousticModemData.h>
#include <acomms_sim/acoustic_packet_state.h>
#include <acomms_sim/time_interval.h>

namespace acomms_sim
{

struct classcomp {
  bool operator() (const acomms_sim::TimeInterval& lhs, const acomms_sim::TimeInterval& rhs) const
  {return lhs._endTime > rhs._endTime;}
};

class AcousticModemState
{
protected:
  ros::NodeHandlePtr _nh;
  ros::Subscriber _inboundSub;
  ros::Publisher _outboundPub;
  std::string _platformName;
  int _slot;
  std::set<acomms_sim::AcousticPacketState> _packetSet;
  std::set<acomms_sim::TimeInterval, classcomp> _modemBusySet;

public:
  AcousticModemState(ros::NodeHandle *nh, std::string platformName, int slot, std::string inboundTopic, std::string outboundTopic);
  int getSlot();
  std::string getName();
  std::string getInboundTopic();
  std::string getOutboundTopic();
  //ros::Publisher<AcousticModemData>* getOutboundPub();
  void callback(AcousticModemData msg);
  void publishMessage(AcousticModemData msg);
  void call(AcousticModemData request);
  std::set<AcousticPacketState> getPacketState();
  bool addSendPacket(AcousticModemData msg, AcousticPacketState packet);
  AcousticPacketState addReceivePacker(AcousticPacketState sendPacket, long flightTime);
  std::vector<AcousticPacketState> getNonCollidedReceivedPackets(long updateTime);
  void removeOldModemBusyIntervals(long updateTime);
};

}  // namespace acomms_sim

#endif /* ACOUSTIC_MODEM_STATE_H_ */
