/*
 * time_interval.h
 *
 *  Created on: 18 Jul 2014
 *      Author: nick
 */

#ifndef TIME_INTERVAL_H_
#define TIME_INTERVAL_H_

#include <exception>
#include <string>
#include <sstream>

namespace acomms_sim
{
class TimeInterval
{
protected:
  // Start time of the interval
  long _startTime;

  // End time of the interval
  long _endTime;
public:
  TimeInterval();
  TimeInterval(long startTime, long endTime);
  long getStartTime();
  long getEndTime();
  int compareTo(TimeInterval o);
  bool containsInstant(long instant);
  int compareOverlap(TimeInterval o);
  std::string toString();
};

}    //namespace acomms_sim

#endif /* TIME_INTERVAL_H_ */
