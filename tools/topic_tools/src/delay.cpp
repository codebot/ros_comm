///////////////////////////////////////////////////////////////////////////////
// delay is a little node that just delays messages between its 'in' and 'out'
// topics. It can be useful when trying to simulate a super long delay in
// userspace, rather than using lower-level tools like "tc"
//
// Copyright (C) 2017, Morgan Quigley
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of Stanford University nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
/////////////////////////////////////////////////////////////////////////////


#include <cstdio>
#include <deque>
#include "topic_tools/shape_shifter.h"
#include "topic_tools/parse.h"

using std::string;
using std::vector;
using namespace topic_tools;

typedef std::pair<ros::Time, boost::shared_ptr<ShapeShifter const> > StampedShapeShifter;
std::deque<StampedShapeShifter> g_shape_shifters;

ros::NodeHandle *g_node = NULL;
bool g_advertised = false;
string g_output_topic;
ros::Publisher g_pub;
ros::Subscriber* g_sub;
ros::Duration g_delay;

void in_cb(const ros::MessageEvent<ShapeShifter>& msg_event)
{
  boost::shared_ptr<ShapeShifter const> const &msg =
      msg_event.getConstMessage();
  boost::shared_ptr<const ros::M_string> const& connection_header =
      msg_event.getConnectionHeaderPtr();

  if (!g_advertised)
  {
    // If the input topic is latched, make the output topic latched, #3385.
    bool latch = false;
    if (connection_header)
    {
      ros::M_string::const_iterator it = connection_header->find("latching");
      if((it != connection_header->end()) && (it->second == "1"))
      {
        ROS_DEBUG("input topic is latched; latching output topic to match");
        latch = true;
      }
    }
    g_pub = msg->advertise(*g_node, g_output_topic, 10, latch);
    g_advertised = true;
    ROS_INFO("advertised as %s", g_output_topic.c_str());
  }
  g_shape_shifters.push_front(std::make_pair(ros::Time::now() + g_delay, msg));
}

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    printf("\nusage: delay SECONDS IN_TOPIC [OUT_TOPIC]\n\n");
    return 1;
  }
  std::string topic_name;
  if(!getBaseName(string(argv[2]), topic_name))
    return 1;
  ros::init(argc, argv, topic_name + string("_delay"),
            ros::init_options::AnonymousName);
  if (argc == 3)
    g_output_topic = string(argv[2]) + string("_delayed");
  else // argc == 3
    g_output_topic = string(argv[3]);
  g_delay = ros::Duration(atof(argv[1]));
  string input_topic = string(argv[2]);
  ros::NodeHandle n;
  g_node = &n;
  
  g_sub = new ros::Subscriber(g_node->subscribe(input_topic, 10, &in_cb));
  while (ros::ok())
  {
    ros::spinOnce();
    if (!g_shape_shifters.empty() &&
        ros::Time::now() > g_shape_shifters.back().first) {
      g_pub.publish(g_shape_shifters.back().second);
      g_shape_shifters.pop_back();
    }
    else
      ros::Duration(0.001).sleep();
  }
  return 0;
}

