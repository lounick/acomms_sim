/*
 * acoustic_packet_state.h
 *
 *  Created on: 18 Jul 2014
 *      Author: nick
 */

#ifndef ACOUSTIC_PACKET_STATE_H_
#define ACOUSTIC_PACKET_STATE_H_

#include <acomms_sim/AcousticModemData.h>
#include <acomms_sim/time_interval.h>

namespace acomms_sim
{
class AcousticPacketState
{
protected:
  static long nextUniquenessValue = 0;

  long _uniquenessValue;

  acomms_sim::AcousticModemData msg;

  int _numFrames;

  long _modemBusyStartTime;

  long _modemBusyEndTime;

  long _acousticStartTime;

  long _acousticEndTime;

  long _collisionWindowStartTime;

  long _collisionWindowEndTime;

  long _flightTime;

  bool _priorCollision;

  bool _transmitCorruption;

  bool _receiveCorruption;

  AcousticPacketState relatedPacket;

  int _networkPosition;

  int _slot;

  static const int NETWORK_POSITION_TX = 1;
  static const int NETWORK_POSITION_RX = 2;
  static const int NETWORK_POSITION_TX_BUSIED = 3;

public:
  float randomNum;

  AcousticPacketState(acomms_sim::AcousticModemData message, int payloadSize, long sendTime, int sendSlot);
  AcousticPacketState(AcousticPacketState state);
  AcousticPacketState copyForReceiver(long flightTime);
  acomms_sim::AcousticModemData getMessage();
  int getSlot();
  long getCollisionWindowStartTime();
  long getCollisionWindowEndTime();
  acomms_sim::TimeInterval getModemBusyTimeInterval();
  int getNetworkPosition();
  void setNetworkPosition(int networkPosition);
  void setCollision(bool priorCollision);
  bool hasCollision();
  int compareTo(AcousticPacketState o);
  int compareOverlap(AcousticPacketState o);
  std::string toString();
  bool hasTransmitCorruption();
  void setTransmitCorruption(bool transmitCorruption);
  bool hasReceiveCorruption();
  void setReceiveCorruption(bool receiveCorruption);
  bool hasCorruption();
};

}// namespace acomms_sim

#endif /* ACOUSTIC_PACKET_STATE_H_ */
