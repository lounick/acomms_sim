/*
 * acoustic_modem_state.cpp
 *
 *  Created on: 20 Jul 2014
 *      Author: nick
 */

#include <acomms_sim/acoustic_modem_state.h>

namespace acomms_sim
{
AcousticModemState::AcousticModemState(ros::NodeHandle *nh, std::string platformName, int slot, std::string inboundTopic, std::string outboundTopic)
{
  _nh = nh;
  _platformName = platformName;
  _slot = slot;
  _inboundSub = nh->subscribe(inboundTopic, 5, callback);
  _outboundPub = nh->advertise<AcousticModemData>(outboundTopic, 5);
}

int AcousticModemState::getSlot()
{
  return _slot;
}

std::string AcousticModemState::getName()
{
  return _platformName;
}

std::string AcousticModemState::getInboundTopic()
{
  return _inboundSub.getTopic();
}

std::string AcousticModemState::getOutboundTopic()
{
  return _outboundPub.getTopic();
}

//ros::Publisher<AcousticModemData>* AcousticModemState::getOutboundPub() //TODO: Probably not correct. Should implement a member  function to access the publisher. Maybe return a pointer?
//{
//
//}

void AcousticModemState::callback(AcousticModemData msg)
{
  //TODO: Implement callback.
}

void AcousticModemState::publishMessage(AcousticModemData msg)
{
  _outboundPub.publish(msg);
}

void AcousticModemState::call(AcousticModemData request)
{
  //TODO: What do we need this for???
}

std::set<AcousticPacketState> AcousticModemState::getPacketState()
{
  return _packetSet;
}

bool AcousticModemState::addSendPacket(AcousticModemData msg, AcousticPacketState packet)
{

}

AcousticPacketState AcousticModemState::addReceivePacker(AcousticPacketState sendPacket, long flightTime)
{

}

std::vector<AcousticPacketState> AcousticModemState::getNonCollidedReceivedPackets(long updateTime)
{

}

void AcousticModemState::removeOldModemBusyIntervals(long updateTime)
{

}
}  // namespace accoms_sim
