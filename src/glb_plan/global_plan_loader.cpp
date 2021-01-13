#include "glb_plan/global_plan_loader.h"

GlobalPlanLoader::GlobalPlanLoader()
{
  m_straight_line_mission_sub = m_nh.subscribe("/start_mission", 1, &GlobalPlanLoader::load_straight_line_mission, this);
  m_straight_line_vis_pub = m_nh.advertise<visualization_msgs::Marker>("path_viz",1);
  m_glb_path_pub = m_nh.advertise<cpc_motion_planning::path>("/global_path",1);
}

GlobalPlanLoader::~GlobalPlanLoader()
{

}

void GlobalPlanLoader::load_straight_line_mission(const std_msgs::Int32::ConstPtr &msg)
{
  // Read in the data files
  std::ifstream corridor_file;
  float data[2];
  std::vector<float2> wps, wps_raw;
  corridor_file.open("/home/sp/path.txt");
  std::cout<<"Read in data"<<std::endl;
  while(1)
  {
    if (corridor_file>>data[0]>>data[1])
    {
      wps_raw.push_back((make_float2(data[0],data[1])));
    }
    else
    {
      break;
    }
  }

  // Scan the wps_raw to remove the stopped part
  if (wps_raw.size() <=1 )
    return;

  wps.push_back(wps_raw.front());
  for (size_t i = 1; i < wps_raw.size(); i++)
  {
    float2 d = wps_raw[i] - wps.back();
    if (sqrtf(dot(d,d)) > 0.1)
      wps.push_back(wps_raw[i]);
  }

  // We need at least two waypoint to form a line
  if(wps.size()>1)
  {
    show_path(wps);
    m_curr_act_path.clear();
    for (size_t i=0; i<wps.size()-1;i++)
    {
      m_curr_act_path = cascade_vector(m_curr_act_path,make_straight_path(wps[i],wps[i+1]));
    }
    std::vector<size_t> split_idx = split_merge(m_curr_act_path);

    std::vector<size_t> cusp_idx;
    cusp_idx.push_back(split_idx.front());
    for (size_t i=1; i<split_idx.size()-1;i++)
    {
      float2 a = m_curr_act_path[split_idx[i]] - m_curr_act_path[split_idx[i-1]];
      float2 b = m_curr_act_path[split_idx[i+1]] - m_curr_act_path[split_idx[i]];
      float norm_a = sqrtf(dot(a,a));
      float norm_b = sqrtf(dot(b,b));
      float theta = acosf(dot(a,b)/(norm_a*norm_b));

      if (theta > 0.5*M_PI)
        cusp_idx.push_back(split_idx[i]);
    }
    cusp_idx.push_back(split_idx.back());

    cpc_motion_planning::path glb_path;
    glb_path.request_ctt = -1;

    //--- set the path action
    for (size_t i=0; i<cusp_idx.size()-1;i++)
    {
      std::cout<<cusp_idx[i]<<" "<<cusp_idx[i+1]<<" "<<m_curr_act_path.size()-1<<std::endl;
      cpc_motion_planning::path_action pa;
      for (size_t j=cusp_idx[i]; j<cusp_idx[i+1];j++)
      {
        pa.x.push_back(m_curr_act_path[j].x);
        pa.y.push_back(m_curr_act_path[j].y);
        pa.theta.push_back(0);
        pa.type = cpc_motion_planning::path_action::TYPE_FORWARD;
      }
      glb_path.actions.push_back(pa);
    }
    m_glb_path_pub.publish(glb_path);
  }
}

