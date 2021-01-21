#ifndef GLOBAL_PLAN_LOADER_H
#define GLOBAL_PLAN_LOADER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <fstream>
#include <mid_plan/utils/grid_graph.h>
#include <visualization_msgs/Marker.h>
#include <stack>
#include <cpc_motion_planning/path.h>

class GlobalPlanLoader
{
public:
  GlobalPlanLoader();
  ~GlobalPlanLoader();

private:
  void load_straight_line_mission(const std_msgs::Int32::ConstPtr &msg);

  std::vector<float2> make_straight_path(const float2 &a, const float2 &b)
  {
    std::vector<float2> path;
    float2 delta = b-a;
    float dist = sqrtf(dot(delta,delta));

    if (dist < 0.1f)
    {
      path.push_back(b);
      return path;
    }

    delta = delta/dist;
    for (float l = 0; l <= dist; l+=0.1f)
    {
      path.push_back(a+delta*l);
    }
    return path;
  }

  std::vector<float2> cascade_vector(const std::vector<float2> &A, const std::vector<float2> &B)
  {
    std::vector<float2> AB;
    AB.reserve( A.size() + B.size() ); // preallocate memory
    AB.insert( AB.end(), A.begin(), A.end() );
    AB.insert( AB.end(), B.begin(), B.end() );
    return AB;
  }

  inline float pnt2line_dist(const float2 & c1, const float2 & c2, const float2 & c0)
  {
    float2 a = c1-c0;
    float2 b = c2-c1;

    float a_square = dot(a,a);
    float b_square = dot(b,b);
    float a_dot_b = a.x*b.x + a.y*b.y;

    if (b_square < 1e-3)
      return sqrtf(static_cast<float>(a_square));

    return sqrtf(static_cast<float>(a_square*b_square - a_dot_b*a_dot_b)/static_cast<float>(b_square));
  }


  std::vector<size_t> split_merge(const std::vector<float2> &path)
  {
    std::vector<size_t> split_idx;

    // If there is no path, return the center point
    if (path.size() <= 1)
    {
      return split_idx;
    }

    std::stack<std::pair<unsigned int,unsigned int>> task; // fist is anchor, second is target
    task.push(std::make_pair(path.size() - 1, 0));

    unsigned int check = static_cast<unsigned int>(path.size()) - 1;

    split_idx.push_back(path.size() - 1);
    while (task.size() > 0)
    {
      unsigned int target = task.top().second;
      unsigned int anchor = task.top().first;
      task.pop();
      // find the largest distance
      unsigned int max_id = 0;
      float max_dist = 0;
      for (unsigned int j = target; j< anchor; j++)
      {
        float dist = pnt2line_dist(path[anchor],path[target],path[j]);
        if (dist > max_dist)
        {
          max_dist = dist;
          max_id = j;
        }
      }

      if (max_dist > 0.2f)
      {
        task.push(std::make_pair(max_id, target));
        task.push(std::make_pair(anchor, max_id));
        target = max_id;
      }
      else
      {
        split_idx.push_back(target);
        if (target >= check)
        {
          std::cout<<"path size: "<<path.size()<<std::endl;
          ROS_ERROR("Wrong split sequence");
          exit(-1);
        }
        check = target;
      }
    }
    std::reverse(split_idx.begin(),split_idx.end());
    return split_idx;
  }


  void show_path(const std::vector<float2> &wps)
  {
    // For visulization
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "world";
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    line_strip.color.g = 1.0;
    line_strip.color.a = 1.0;

    for (float2 wp: wps)
    {
      geometry_msgs::Point pnt;
      pnt.x = wp.x;
      pnt.y = wp.y;
      pnt.z = 0;
      line_strip.points.push_back(pnt);
    }
    m_straight_line_vis_pub.publish(line_strip);
  }

private:
  ros::NodeHandle m_nh;
  ros::Subscriber m_straight_line_mission_sub;
  std::vector<float2> m_curr_act_path;
  ros::Publisher m_straight_line_vis_pub;
  ros::Publisher m_glb_path_pub;

};

#endif // GLOBAL_PLAN_LOADER_H
