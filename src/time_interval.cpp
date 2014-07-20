/*
 * time_interval.cpp
 *
 *  Created on: 18 Jul 2014
 *      Author: nick
 */

#include <acomms_sim/time_interval.h>

namespace acomms_sim
{
TimeInterval::TimeInterval()
{
  //Empty constructor
}

TimeInterval::TimeInterval(long startTime, long endTime)
{
  if (startTime > endTime)
  {
    std::stringstream ss;
    ss << "startTime after endTime (" << sTime << ", " << eTime << ")";
    throw std::invalid_argument(ss.str().c_str());
  }
  _startTime = startTime;
  _endTime = endTime;
}

long TimeInterval::getStartTime()
{
  return _startTime;
}

long TimeInterval::getEndTime()
{
  return _endTime;
}

int TimeInterval::compareTo(TimeInterval o) //TODO: Check implementation.
{
  if (_startTime < o._startTime)
    return -1;
  else if (_startTime > o._startTime)
    return 1;

  else if (_endTime < o._endTime)
    return -1;
  else if (_endTime > o._endTime)
    return 1;

  // Else they are the same interval.
  else
    return 0;
}

int TimeInterval::endCompareTo(TimeInterval o)
{
  if (_endTime < o._endTime)
    return -1;
  else if (_endTime > o._endTime)
    return 1;

  else if (_startTime < o._startTime)
    return -1;
  else if (_startTime > o._startTime)
    return 1;

  // More comparison checks, or is that good enough?
  else
    return 0;
}

bool TimeInterval::containsInstant(long instant)
{
  // An instant is contained within a TimeInterval if it is on or after
  // the start time, and before the end time.
  // So, necessarily a zero length TimeInterval can't contain any instants.
  if (_startTime <= instant && instant < _endTime)
    return true;
  else
    return false;
}

int TimeInterval::compareOverlap(TimeInterval o)
{
  // Packet is after this.
  if (o._startTime >= _endTime)
    return 1;
  // Packet is before this.
  else if (o._endTime <= _startTime)
    return -1;
  // Packet overlaps this.
  else
    return 0;
}

std::string TimeInterval::toString()
{
  // Purposefully uses [a,b) notation.
  std::stringstream ss;
  ss << "[" << _startTime << ", " << _endTime << ")";
  return ss.str();
}

}  //namespace acomms_sim
