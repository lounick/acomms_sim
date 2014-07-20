/*
 * acoustic_packet_state.cpp
 *
 *  Created on: 19 Jul 2014
 *      Author: nick
 */

#include <acomms_sim/acoustic_packet_state.h>

namespace acomms_sim
{

AcousticPacketState::AcousticPacketState(acomms_sim::AcousticModemData message, int payloadSize, long sendTime,
                                         int sendSlot)
{
  // Uniqueness value used purely for compareTo() method.
  _uniquenessValue = ++nextUniquenessValue;
  // The message is technically const, so we shouldn't modify it.
  msg = message;
  _slot = sendSlot;

  _numFrames = (payloadSize + AcousticNetworkSimulator::frameSize - 1) / AcousticNetworkSimulator::frameSize;

  // Busy time is the time the transmission is initialised with the modem.
  // The modem will be busy from this time.
  _modemBusyStartTime = sendTime;

  // WHOI modem can't receive when transmit buffering, so collisions start
  // as soon as the modem cycle command is sent.
  //
  _collisionWindowStartTime = _modemBusyStartTime;

  // The time the modem starts the acoustic transmission.
  _acousticStartTime = _modemBusyStartTime + AcousticNetworkSimulator::modemTxCycleToRequestTimeMs
      + (AcousticNetworkSimulator::modemTxFrameSerialTimeMs * _numFrames);

  // The time the modem finishes the acoustic transmission.
  _acousticEndTime = _acousticStartTime + (AcousticNetworkSimulator::modemTxPerFrameChannelTimeMs * _numFrames);
  // Modem busy period ends when sending complete.
  _modemBusyEndTime = _acousticEndTime;
  // Collisions end when the modem is free.
  _collisionWindowEndTime = _modemBusyEndTime;

  _networkPosition = NETWORK_POSITION_TX;
}

AcousticPacketState::AcousticPacketState(AcousticPacketState state)
{
  _uniquenessValue = ++nextUniquenessValue;
  // Const message pointer is copied, not cloned - message should not be altered.
  msg = state.msg;
  _numFrames = state._numFrames;
  _slot = state._slot;
}

AcousticPacketState AcousticPacketState::copyForReceiver(long flightTime)
{
  AcousticPacketState receiverPacket = new AcousticPacketState(*this);

  receiverPacket._flightTime = flightTime;

  // Acoustic start time and modem busy when receiver first hears packet.
  receiverPacket._acousticStartTime = _acousticStartTime + flightTime;
  receiverPacket._modemBusyStartTime = receiverPacket._acousticStartTime;

  // Acoustic packets apparently shouldn't collide in flight, only at
  // end points (according to Don Brutzman). Thus start time is just
  // when we begin to receive the packet.
  receiverPacket._collisionWindowStartTime = receiverPacket._acousticStartTime;

  // The end is when the acoustic signal finishes at the receiver.
  receiverPacket._acousticEndTime = receiverPacket._acousticStartTime
      + (AcousticNetworkSimulator::modemTxPerFrameChannelTimeMs * _numFrames);
  // TODO: could potentially increase modem busy and collision time slightly over acoustic end.
  receiverPacket._modemBusyEndTime = receiverPacket._acousticEndTime;
  receiverPacket._collisionWindowEndTime = receiverPacket._modemBusyEndTime;

  receiverPacket._networkPosition = NETWORK_POSITION_RX;

  // Tie receive packet to transmit packet, for collision purposes.
  // Yes, this is a bit hacky.
  receiverPacket.relatedPacket = *this;

  receiverPacket._transmitCorruption = _transmitCorruption;

  return receiverPacket;
}

acomms_sim::AcousticModemData AcousticPacketState::getMessage()
{
  return msg;
}

int AcousticPacketState::getSlot()
{
  return _slot;
}

long AcousticPacketState::getCollisionWindowStartTime()
{
  return _collisionWindowStartTime;
}

long AcousticPacketState::getCollisionWindowEndTime()
{
  return _collisionWindowEndTime;
}

acomms_sim::TimeInterval AcousticPacketState::getModemBusyTimeInterval()
{
  return new TimeInterval(_modemBusyStartTime, _modemBusyEndTime);
}

int AcousticPacketState::getNetworkPosition()
{
  return _networkPosition;
}

void AcousticPacketState::setNetworkPosition(int networkPosition)
{
  _networkPosition = networkPosition;
}

void AcousticPacketState::setCollision(bool priorCollision)
{
  _priorCollision = priorCollision;
}

bool AcousticPacketState::hasCollision()
{
  // Receive packet is tied to transmit packet, for collision purposes.
  // Yes, this is a bit hacky.
  return _priorCollision || (relatedPacket != NULL && relatedPacket.hasCollision());
}

int AcousticPacketState::compareTo(AcousticPacketState o) //TODO: Check if this is not good comparator for std::set.
{
  if (_collisionWindowStartTime < o._collisionWindowStartTime)
    return -1;
  else if (_collisionWindowStartTime > o._collisionWindowStartTime)
    return 1;

  if (_collisionWindowEndTime < o._collisionWindowEndTime)
    return -1;
  else if (_collisionWindowEndTime > o._collisionWindowEndTime)
    return 1;

  //System.out.println("Packets have same interval: " + this + ", " + o);

  if (_uniquenessValue < o._uniquenessValue)
    return -1;
  else if (_uniquenessValue > o._uniquenessValue)
    return 1;

  return 0;
}

int AcousticPacketState::compareOverlap(AcousticPacketState o) //TODO: Check if this is not good comparator for std::set.
{
  // Packet is after this.
  if (o._collisionWindowStartTime >= _collisionWindowEndTime)
    return 1;
  // Packet is before this.
  else if (o._collisionWindowEndTime <= _collisionWindowStartTime)
    return -1;
  // Packet overlaps this.
  else
    return 0;
}

std::string AcousticPacketState::toString()
{
  std::stringstream ss;
  ss << "[np " << _networkPosition << ", frames " << _numFrames << ", cs " << _collisionWindowStartTime << ", ce " << _collisionWindowEndTime << "]";
  return ss.str();
}

bool AcousticPacketState::hasTransmitCorruption()
{
  return _transmitCorruption;
}

void AcousticPacketState::setTransmitCorruption(bool transmitCorruption)
{
  _transmitCorruption = transmitCorruption;
}

bool AcousticPacketState::hasReceiveCorruption()
{
  return _receiveCorruption;
}

void AcousticPacketState::setReceiveCorruption(bool receiveCorruption)
{
  _receiveCorruption = receiveCorruption;
}

bool AcousticPacketState::hasCorruption()
{
  return hasTransmitCorruption() || hasReceiveCorruption();
}

}  // namespace acomms_sim
