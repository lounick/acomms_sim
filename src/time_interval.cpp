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

TimeInterval::TimeInterval(long sTime, long eTime)
{
  if (startTime > endTime)
  {
    std::stringstream ss;
    ss << "startTime after endTime (" << sTime << ", " << eTime << ")";
    throw std::invalid_argument(ss.str().c_str());
  }
  startTime = sTime;
  endTime = eTime;
}

long TimeInterval::getStartTime()
{
  return startTime;
}

long TimeInterval::getEndTime()
{
  return endTime;
}

int TimeInterval::compareTo(TimeInterval o) //TODO: Check implementation.
{
  if (startTime < o.startTime)
    return -1;
  else if (startTime > o.startTime)
    return 1;

  else if (endTime < o.endTime)
    return -1;
  else if (endTime > o.endTime)
    return 1;

  // Else they are the same interval.
  else
    return 0;
}

bool TimeInterval::containsInstant(long instant)
{
  // An instant is contained within a TimeInterval if it is on or after
  // the start time, and before the end time.
  // So, necessarily a zero length TimeInterval can't contain any instants.
  if (startTime <= instant && instant < endTime)
    return true;
  else
    return false;
}

int TimeInterval::compareOverlap(TimeInterval o)
{
  // Packet is after this.
  if (o.startTime >= endTime)
    return 1;
  // Packet is before this.
  else if (o.endTime <= startTime)
    return -1;
  // Packet overlaps this.
  else
    return 0;
}

std::string TimeInterval::toString()
{
  // Purposefully uses [a,b) notation.
  std::stringstream ss;
  ss << "[" << startTime << ", " << endTime << ")";
  return ss.str();
}

}//namespace acomms_sim
