/*
 * acoustic_packet_state.h
 *
 *  Created on: 18 Jul 2014
 *      Author: nick
 */

#ifndef ACOUSTIC_PACKET_STATE_H_
#define ACOUSTIC_PACKET_STATE_H_

#include <acomms_sim/AcousticModemData.h>

namespace acomms_sim
{
class AcousticPacketState
{
protected:
  static long nextUniquenessValue = 0;

  acomms_sim::AcousticModemData msg;

  int numFrames;

  long modemBusyStartTime;

  long modemBusyEndTime;

  long acousticStartTime;

  long acousticEndTime;

  long collisionWindowStartTime;

  long collisionWindowEndTime;

  long flightTime;

  bool priorCollision;

  bool transmitCorruption;

  bool receiveCorruption;

  AcousticPacketState relatedPacket;

  int networkPosition;

  int slot;

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


};

}// namespace acomms_sim

#endif /* ACOUSTIC_PACKET_STATE_H_ */
