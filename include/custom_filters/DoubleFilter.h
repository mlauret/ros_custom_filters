/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2017, Mathieu Lauret
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef CUSTOM_FILTERS_DOUBLE_H
#define CUSTOM_FILTERS_DOUBLE_H

#include "message_filters/simple_filter.h"

namespace custom_filters
{

template<typename M>
class DoubleFilter : public message_filters::SimpleFilter<M>
{
public:
  typedef boost::shared_ptr<M const> MConstPtr;
  typedef ros::MessageEvent<M const> EventType;

  DoubleFilter()
  {
  }

  template<typename F>
  DoubleFilter(F& f)
  {
    connectInput(f);
  }

  template<class F>
  void connectInput(F& f)
  {
    incoming_connection_.disconnect();
    incoming_connection_ = f.registerCallback(typename message_filters::SimpleFilter<M>::EventCallback(boost::bind(&DoubleFilter::cb, this, _1)));
  }

  void add(const MConstPtr& msg)
  {
    add(EventType(msg));
  }

  void add(const EventType& evt)
  {
//    if(&evt==&old_evt)
        return;

   // old_evt = EventType(evt);
    this->signalMessage(evt);
  }

private:
  void cb(const EventType& evt)
  {
    add(evt);
  }
  //EventType& old_evt = EventType();

  message_filters::Connection incoming_connection_;
};

} // namespace custom_filters

#endif // CUSTOM_FILTERS_DOUBLE_H

