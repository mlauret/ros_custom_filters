#include <ros/ros.h>
#include <gtest/gtest.h>
#include <custom_filters/DoubleFilter.h>
#include <std_msgs/String.h>
#include <message_filters/subscriber.h>
#include <boost/range/irange.hpp>

void callback(const std_msgs::String::ConstPtr& msg, int* counter){
 *counter+=1;
}


bool cb(const std_msgs::String::ConstPtr &newMessage, const std_msgs::String::ConstPtr &previsouMessage){
  return newMessage->data!=previsouMessage->data;
}

TEST(TestSimple, doubleFilterTest){
  int counter = 0;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("/test/string", 10);


  message_filters::Subscriber<std_msgs::String> sub(n, "/test/string", 10);
  custom_filters::DoubleFilter<std_msgs::String> filter(sub,cb);
  filter.registerCallback(boost::bind(callback, _1, &counter));


  std_msgs::String msg;
  msg.data="coucou";


  for(int i : boost::irange(1,5)){
  pub.publish(msg);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  }

  ASSERT_EQ(1, counter);
}

TEST(TestSimple2, doubleFilterTest){
  int counter = 0;
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("/test/string", 10);


  message_filters::Subscriber<std_msgs::String> sub(n, "/test/string", 10);
  custom_filters::DoubleFilter<std_msgs::String> filter(sub,cb);
  filter.registerCallback(boost::bind(callback, _1, &counter));


  std_msgs::String msg;
  for(int i : boost::irange(1,5)){
  msg.data=std::to_string(i);
  pub.publish(msg);
  ros::Duration(2.0).sleep();
  ros::spinOnce();
  }

  ASSERT_EQ(4, counter);
}


int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "double_filter_test");

  return RUN_ALL_TESTS();
}
