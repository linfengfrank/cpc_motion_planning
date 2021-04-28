#ifndef UGV_TEB_PLANNER_H
#define UGV_TEB_PLANNER_H
#include <loc_plan/ugv_base_local_planner.h>
#include <cpc_motion_planning/path.h>
#include <std_msgs/Int32.h>
#include <pure_pursuit/pure_pursuit_ctrl.h>
#include <cpc_aux_mapping/nf1_task.h>

#include <teb_local_planner/optimal_planner.h>
#include <teb_local_planner/homotopy_class_planner.h>
#include <teb_local_planner/visualization.h>
#include <teb_local_planner/recovery_behaviors.h>

#define SIMPLE_UGV UGV::UGVModel,UGV::UGVDPControl,UGV::NF1Evaluator,UGV::UGVSwarm<8>
namespace teb_local_planner
{
class TEBLocalPlanner : public UGVLocalMotionPlanner
{
  enum STUCK_SUB_MODE
  {
    RECOVER = 0,
    FULL_STUCK
  };

public:
  TEBLocalPlanner();
  ~TEBLocalPlanner();

protected:
  virtual void do_start();
  virtual void do_normal();
  virtual void do_stuck();
  virtual void do_emergent();
  virtual void do_braking();
  virtual void do_pos_reached();
  virtual void do_fully_reached();
  virtual void do_dropoff();
private:
  void do_recover();
  void do_full_stuck();
private:
  void plan_call_back(const ros::TimerEvent&);
  void nf1_call_back(const cpc_aux_mapping::nf1_task::ConstPtr &msg);
  void cycle_init();
  bool check_tgt_is_same(const UGV::NF1Evaluator::Target &t1, const UGV::NF1Evaluator::Target &t2)
  {
    if (t1.act_id == t2.act_id && t1.path_id == t2.path_id)
      return true;
    else
      return false;
  }

private:
  ros::Subscriber m_nf1_sub;
  ros::Subscriber m_hybrid_path_sub;
  ros::Timer m_planning_timer;
  ros::Publisher m_ref_pub;
  ros::Publisher m_status_pub;
  ros::Publisher m_tgt_reached_pub;
  ros::Publisher m_stuck_plan_request_pub;
  ros::Publisher m_drive_dir_pub;

  bool m_goal_received;
  bool m_task_is_new;
  bool m_use_de;

  UGV::NF1Evaluator::Target m_goal;
  UGV::UGVModel::State m_carrot;
  float m_ref_v, m_ref_w, m_ref_theta;
  cpc_motion_planning::ref_data m_ref_msg;
  int m_v_err_reset_ctt, m_w_err_reset_ctt, m_tht_err_reset_ctt;
  int m_plan_cycle;
  int m_ref_start_idx;
  std::vector<UGV::UGVModel::State> m_traj;
  bool cycle_initialized;
  int m_braking_start_cycle;
  int m_stuck_start_cycle;
  int m_full_start_cycle;
  int m_plan_request_cycle;
  int m_swarm_size;
  int m_batch_num;
  int m_episode_num;
  NF1MapDT *m_nf1_map;
  cpc_motion_planning::path m_stuck_recover_path;
  ros::ServiceClient m_collision_check_client;
  STUCK_SUB_MODE m_stuck_submode;

  //----------
  HomotopyClassPlannerPtr m_planner;
  TebConfig m_cfg;
  TebVisualizationPtr m_visualization;
  UGV::UGVModel::State m_ini_state;

  RobotFootprintModelPtr get_footprint_from_param(const ros::NodeHandle& nh)
  {
    std::string model_name;
    if (!nh.getParam("footprint_model/type", model_name))
    {
      ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
      return boost::make_shared<PointRobotFootprint>();
    }

    // point
    if (model_name.compare("point") == 0)
    {
      ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
      return boost::make_shared<PointRobotFootprint>();
    }

    // circular
    if (model_name.compare("circular") == 0)
    {
      // get radius
      double radius;
      if (!nh.getParam("footprint_model/radius", radius))
      {
        ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                         << "/footprint_model/radius' does not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius <<"m) loaded for trajectory optimization.");
      return boost::make_shared<CircularRobotFootprint>(radius);
    }

    // line
    if (model_name.compare("line") == 0)
    {
      // check parameters
      if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
      {
        ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                         << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      // get line coordinates
      std::vector<double> line_start, line_end;
      nh.getParam("footprint_model/line_start", line_start);
      nh.getParam("footprint_model/line_end", line_end);
      if (line_start.size() != 2 || line_end.size() != 2)
      {
        ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                         << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }

      ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] <<"]m, line_end: ["
                       << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
      return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
    }

    // two circles
    if (model_name.compare("two_circles") == 0)
    {
      // check parameters
      if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius")
          || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
      {
        ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                         << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      double front_offset, front_radius, rear_offset, rear_radius;
      nh.getParam("footprint_model/front_offset", front_offset);
      nh.getParam("footprint_model/front_radius", front_radius);
      nh.getParam("footprint_model/rear_offset", rear_offset);
      nh.getParam("footprint_model/rear_radius", rear_radius);
      ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset <<"m, front_radius: " << front_radius
                      << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
      return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
    }

    // polygon
    if (model_name.compare("polygon") == 0)
    {

      // check parameters
      XmlRpc::XmlRpcValue footprint_xmlrpc;
      if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc) )
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                         << "/footprint_model/vertices' does not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      // get vertices
      if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        try
        {
          Point2dContainer polygon = make_footprint_from_xml(footprint_xmlrpc, "/footprint_model/vertices");
          ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
          return boost::make_shared<PolygonRobotFootprint>(polygon);
        }
        catch(const std::exception& ex)
        {
          ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
          return boost::make_shared<PointRobotFootprint>();
        }
      }
      else
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                         << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }

    }

    // otherwise
    ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
    return boost::make_shared<PointRobotFootprint>();
  }

  Point2dContainer make_footprint_from_xml(XmlRpc::XmlRpcValue& footprint_xmlrpc, const std::string& full_param_name)
  {
     // Make sure we have an array of at least 3 elements.
     if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         footprint_xmlrpc.size() < 3)
     {
       ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                  full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
       throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                                "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
     }

     Point2dContainer footprint;
     Eigen::Vector2d pt;

     for (int i = 0; i < footprint_xmlrpc.size(); ++i)
     {
       // Make sure each element of the list is an array of size 2. (x and y coordinates)
       XmlRpc::XmlRpcValue point = footprint_xmlrpc[ i ];
       if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
           point.size() != 2)
       {
         ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                   "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                    full_param_name.c_str());
         throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
      }

      pt.x() = get_number_from_xml(point[ 0 ], full_param_name);
      pt.y() = get_number_from_xml(point[ 1 ], full_param_name);

      footprint.push_back(pt);
    }
    return footprint;
  }

  double get_number_from_xml(XmlRpc::XmlRpcValue& value, const std::string& full_param_name)
  {
    // Make sure that the value we're looking at is either a double or an int.
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::string& value_string = value;
      ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                 full_param_name.c_str(), value_string.c_str());
       throw std::runtime_error("Values in the footprint specification must be numbers");
     }
     return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
  }

};
}

#endif // UGV_TEB_PLANNER_H
