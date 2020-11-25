#include "glb_plan/path_smoother.h"

PathSmoother::PathSmoother(GridGraph *gh):
  m_gh(gh)
{

}

void PathSmoother::smooth_path()
{
  for (int i=0; i<120; i++)
  {
    autoResize();
    apply_eb();
  }

  std::vector<float2> pb; //path buffer
  pb = m_path;
  m_path.clear();
  m_path.push_back(pb.front());
  for(size_t i=1; i < pb.size(); i++)
  {
    float2 current_head = m_path.back();
    float d = dist(pb[i],current_head);
    int N = ceil(d/m_gh->getGridStep());
    float2 delta = (pb[i] - current_head)/N;
    for (int j=1;j<=N;j++)
    {
       m_path.push_back(current_head + j*delta);
    }
  }
}

void PathSmoother::apply_eb()
{
  std::vector<float2> pb; //path buffer

  pb = m_path;
  for (size_t i = 1; i<pb.size()-1; i++)
  {
    float2 f1 = get_force(pb[i],pb[i-1]);
    float2 f2 = get_force(pb[i],pb[i+1]);
    float2 f3 = get_gradient(pb[i]);

    float2 f_total = 0.5f*(f1 + f2 + f3);

    float norm = sqrtf(dot(f_total,f_total));
    if (norm > m_gh->getGridStep()*0.5f)
      f_total = f_total/norm*m_gh->getGridStep()*0.5f;
    m_path[i] = m_path[i]+f_total;
  }
}


void PathSmoother::autoResize()
{

  std::vector<float2> pb; //path buffer
  float d = 3*m_gh->getGridStep();

  pb = m_path;
  m_path.clear();

  m_path.push_back(pb.front());
  for(size_t i=1; i < pb.size()-1; i++)
  {
    if (dist(pb[i],m_path.back()) >= d )
      m_path.push_back(pb[i]);
  }
  m_path.push_back(pb.back());

  //handle the case where the last two point are very close
  if (m_path.size() > 2)
  {
    if (dist(m_path.back(),m_path[m_path.size()-2]) < d)
    {
      m_path[m_path.size()-2] = m_path.back();
      m_path.pop_back();
    }
  }

  pb = m_path;
  m_path.clear();

  m_path.push_back(pb.front());
  for(size_t i=1; i < pb.size(); i++)
  {
    if (dist(pb[i],m_path.back()) >= 2*d )
    {
      m_path.push_back(0.5f*(pb[i]+m_path.back()));
      m_path.push_back(pb[i]);
    }
    else
    {
      m_path.push_back(pb[i]);
    }
  }

}
