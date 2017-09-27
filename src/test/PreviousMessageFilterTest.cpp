/*******************************************************************************
 * Software License Agreement (BSD 3-Clause License)
 *
 * Copyright (c) 2017, Mathieu LAURET
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <custom_filters/PreviousMessageFilter.h>
#include <custom_filters/utils/MessageComparator.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <sstream>

///
/// \brief callback
/// \param msg the ros std_msgs::String message
/// \param counter a pointer to a counter
/// Callback used by all test. It just increment a counter
void callback(const std_msgs::String::ConstPtr& msg, int* counter)
{
  *counter += 1;
}


///
/// \brief TEST
/// Send the same message to increase the counter by 1 only (the first time)
TEST(TestSimple1, previousMessageFilterTest)
{
  //a counter to count the number of callback made
  int counter = 0;
  ros::NodeHandle n;
  //publisher to test the filter
  ros::Publisher pub = n.advertise<std_msgs::String>("/test/string", 10);

  //special subscriber for message_filter
  message_filters::Subscriber<std_msgs::String> sub(n, "/test/string", 10);
  //Send the message from the previous filter (subscriber) to our filter with our callback
  custom_filters::PreviousMessageFilter<std_msgs::String> filter(sub, custom_filters::comparator::stringComparator);
  //End the filter pipeline with a 'normal' callback function (bind is used to send the counter)
  filter.registerCallback(boost::bind(callback, _1, &counter));

  //send the same data to see if the counter increase or not
  std_msgs::String msg;
  msg.data = "coucou";

  for (int i = 0; i < 5; i++)
  {
    pub.publish(msg);
    //TODO : check if a better method than sleep exist to allow roscore to digest the message
    ros::Duration(2.0).sleep();
    ros::spinOnce();
  }

  //if the counter is 1, the test is OK
  ASSERT_EQ(1, counter);
}

///
/// \brief TEST
/// Send different message to increase the counter by 5 (everytime)
TEST(TestSimple5, previousMessageFilterTest)
{
  int counter = 0;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("/test/string", 10);

  message_filters::Subscriber<std_msgs::String> sub(n, "/test/string", 10);
  custom_filters::PreviousMessageFilter<std_msgs::String> filter(sub, custom_filters::comparator::stringComparator);
  filter.registerCallback(boost::bind(callback, _1, &counter));

  std_msgs::String msg;
  for (int i = 0; i < 5; i++)
  {
    std::stringstream ss;
    ss << i;
    msg.data = ss.str();
    pub.publish(msg);
    ros::Duration(2.0).sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(5, counter);
}

TEST(TestSimple3, previousMessageFilterTest)
{
  int counter = 0;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("/test/string", 10);

  message_filters::Subscriber<std_msgs::String> sub(n, "/test/string", 10);
  custom_filters::PreviousMessageFilter<std_msgs::String> filter(sub, custom_filters::comparator::stringComparator);
  filter.registerCallback(boost::bind(callback, _1, &counter));

  std_msgs::String msg;
  //Send different message 3 times
  for (int i = 0; i < 3; i++)
  {
    std::stringstream ss;
    ss << i;
    msg.data = ss.str();
    pub.publish(msg);
    ros::Duration(2.0).sleep();
    ros::spinOnce();
  }

  //send the same message 2 times
  for (int i = 0; i < 2; i++)
  {
    pub.publish(msg);
    ros::Duration(2.0).sleep();
    ros::spinOnce();
  }

  ASSERT_EQ(3, counter);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "previous_message_filter_test");

  return RUN_ALL_TESTS();
}
