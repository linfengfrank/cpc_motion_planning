#ifndef CLASSJLT
#define CLASSJLT
//#define DEBUGJLT
#include <sstream>
#include <string>
#include <fstream>
#include <assert.h>
#include <cmath>
#include <cuda.h>
#include <cuda_runtime.h>
#include <cuda_runtime_api.h>

#define JLT_signEpsilon 1e-5f
#define JLT_reachEpsilon 1e-2f
class JLT
{
public:
  enum FRAME
  {
    GLB,
    LOC
  };

  struct State
  {
    float p;
    float v;
    float a;
    __host__ __device__
    State() :p(0), v(0), a(0){}

    __host__ __device__
    ~State(){}
  };

  struct Limit
  {
    float vMax,vMin;
    float aMax,aMin;
    float jMax,jMin;
    __host__ __device__
    Limit() :vMax(1.0),vMin(-1.0),aMax(1.0),aMin(-1.0),jMax(1.0),jMin(-1.0)
    {}
  };

  struct StageParam
  {
    float t[3];
    float p[4];
    float v[4];
    float a[4];
    float j[3];
    __host__ __device__
    StageParam()
    {
      for (int i = 0; i < 3; i++)
      {
        t[i] = 0;
        p[i] = 0;
        v[i] = 0;
        a[i] = 0;
        j[i] = 0;
      }
      p[3] = 0;
      v[3] = 0;
      a[3] = 0;
    }

    __host__ __device__
    StageParam& operator=(StageParam const& rhs)
    {
      for (int i = 0; i < 3; i++)
      {
        t[i] = rhs.t[i];
        p[i] = rhs.p[i];
        v[i] = rhs.v[i];
        a[i] = rhs.a[i];
        j[i] = rhs.j[i];
      }
      p[3] = rhs.p[3];
      v[3] = rhs.v[3];
      a[3] = rhs.a[3];
      return *this;
    }
  };

  struct TPBVPParam
  {
    StageParam p1, p2;
    float T[3];
  };

  struct Param3d
  {
    TPBVPParam val[3];
    float T_longest;
    bool ok;
    __host__ __device__
    Param3d()
    {
      T_longest = 0;
      ok = false;
    }
  };

  //---
public:
  __host__ __device__
  JLT()
  {

  }

  __host__ __device__
  ~JLT()
  {

  }
  //---
  __host__ __device__
  int solveTPBVP(float pr, float vr, State ini, Limit lim, TPBVPParam & P)
  {
    int ok = 0;
    float pf = calculateEndPosition(vr, ini, lim);
    float dist_err = pr - pf;
    int cruise_sign = sign(dist_err);
    float cruise_velocity;
    if (cruise_sign == 1)
      cruise_velocity = lim.vMax;
    else if (cruise_sign == -1)
      cruise_velocity = lim.vMin;
    else
      cruise_velocity = 0;

    StageParam ZCP_U; //ZCP for zero cruise profile (UP stage: from ini to cruise speed)

    solveVelocityTaret(cruise_velocity, ini, lim, ZCP_U);
    State mid;
    mid.p = ZCP_U.p[3];
    mid.v = ZCP_U.v[3];
    mid.a = ZCP_U.a[3];
    pf = calculateEndPosition(vr, mid, lim);

    //Case 1: The cruise_sign is zero, directly slows down to zero shall reach the target
    if(cruise_sign == 0 || fabsf(dist_err) < JLT_reachEpsilon)
    {
      ok = 1;
      StageParam STP; //STP for stop (reach the final veocity directly)
      solveVelocityTaret(vr,ini,lim,STP);
      P.T[0] = STP.t[2];
      P.T[1] = P.T[0];
      P.T[2] = P.T[1];
      P.p1 = STP;

      //Fill up the rest of the trajectory since we have a one stage reach
      mid.p = STP.p[3];
      mid.v = STP.v[3];
      mid.a = STP.a[3];

      StageParam SUP; //SUP for supplementary
      solveVelocityTaret(vr,mid,lim,SUP);
      P.p2 = SUP;
    }
    else
    {
      float tgtDiff = (pf - pr) * static_cast<float>(cruise_sign);
      //Case 2: Undershoot the target, fill up the missing distance with max speed cruise phase
      if (tgtDiff <= -JLT_reachEpsilon)
      {
        float cruise_time = 0;
        ok = 1;
        StageParam ZCP_D; //ZCP for zero cruise profile (Down stage: from max speed to vr)
        mid.p = mid.p + pr - pf;
        solveVelocityTaret(vr, mid, lim, ZCP_D);
        cruise_time = (pr - pf) / cruise_velocity;
#ifdef DEBUGJLT
        if (cruise_time < 0)
          assert(0 && "cruise time smaller than zero");
#endif
        P.T[0] = ZCP_U.t[2];
        P.T[1] = P.T[0] + cruise_time;
        P.T[2] = P.T[1] + ZCP_D.t[2];
        P.p1 = ZCP_U;
        P.p2 = ZCP_D;
      }
      //Case 3: Over shoot the target, run the bisection search for the reachable max speed
      else if(tgtDiff >= JLT_reachEpsilon)
      {
        float tHigh = ZCP_U.t[2];
        float tLow = 0;
        float tProbe = 0;
        float tDiff = 0;
        for (int i = 0; i < 64; i++)
        {
          tProbe = (tHigh + tLow)*0.5f;
          mid = stageRefGen(ZCP_U, tProbe);
          pf = calculateEndPosition(vr, mid, lim);

          tDiff = (pf - pr)* static_cast<float>(cruise_sign);

          if(tDiff <= -JLT_reachEpsilon)
          {
            tLow = tProbe;
          }
          else if(tDiff >= JLT_reachEpsilon)
          {
            tHigh = tProbe;
          }
          else
          {
            ok = 1;
            break;
          }
        }
        StageParam ZCP_D; //ZCP for zero cruise profile (Down stage: from max speed to vr)
        solveVelocityTaret(vr, mid, lim, ZCP_D);
        P.T[0] = tProbe;
        P.T[1] = P.T[0];
        P.T[2] = P.T[1] + ZCP_D.t[2];
        P.p1 = ZCP_U;
        P.p2 = ZCP_D;
      }
      //Case 4: Hit the target with zero cruise profile, thus no further calculation needed
      else
      {
        ok = 1;
        StageParam ZCP_D; //SUP for supplementary
        solveVelocityTaret(vr, mid, lim, ZCP_D);
        P.T[0] = ZCP_U.t[2];
        P.T[1] = P.T[0];
        P.T[2] = P.T[1] + ZCP_D.t[2];
        P.p1 = ZCP_U;
        P.p2 = ZCP_D;
      }
    }
    return ok;
  }
  //---
  __host__ __device__
  State TPBVPRefGen(const TPBVPParam &P, float t, float &u)
  {
    State out;
    if (t <= P.T[0])
    {
      out = stageRefGen(P.p1, t, u);
    }
    else if (t > P.T[0] && t < P.T[1])
    {
      out.p = calcP(t - P.T[0], P.p1.p[3], P.p1.v[3], P.p1.a[3], 0);
      out.v = calcV(t - P.T[0], P.p1.v[3], P.p1.a[3], 0);
      out.a = calcA(t - P.T[0], P.p1.a[3], 0);
      u = 0;
    }
    else
    {
      out = stageRefGen(P.p2, t - P.T[1], u);
    }
    return out;
  }

  //---
  __host__ __device__
  void solveVelocityTaret(float vr, State ini, Limit lim, StageParam& tP)
  {
    //find the end velocity, when acceleration immediately goes to zero
    float vE;
    if (ini.a >=0)
      vE = ini.v + ini.a*fabsf(ini.a/lim.jMin)/2.0f;
    else
      vE = ini.v + ini.a*fabsf(ini.a/lim.jMax)/2.0f;

    //determine the cruise direction
    int d = sign(vr - vE);

    //determine the cruise acceleration
    float ac;
    if (d == 1)
      ac = lim.aMax;
    else if(d == -1)
      ac = lim.aMin;
    else
      ac = 0.0;

    //determine t1, j1, v1
    float t1;
    if (ac - ini.a >= 0)
    {   //increase
      t1 = (ac - ini.a)/lim.jMax;
      tP.j[0] = lim.jMax;
    }
    else
    {   //decrease
      t1 = (ac - ini.a)/lim.jMin;
      tP.j[0] = lim.jMin;
    }
    float v1 = ini.v + ini.a*t1 + 0.5f*tP.j[0]*t1*t1;

    //determine t3, j3, v3bar, v2bar
    float t3;
    if (-ac >= 0)
    {
      //increase
      t3 = -ac/lim.jMax;
      tP.j[2] = lim.jMax;
    }
    else
    {
      //decrease
      t3 = -ac/lim.jMin;
      tP.j[2] = lim.jMin;
    }
    float v3bar = ac*t3 + 0.5f*tP.j[2]*t3*t3;
    float v2bar = vr - v1 - v3bar;

    float t2;
    if (d == 0)
      t2 = 0;
    else
      t2 = v2bar / ac;

    bool numeric_error = false;
    if (t2 < 0)
    {
      if (d==1)
      {
        float tmp_val = (2.0f*(vr-ini.v)+ini.a*ini.a/lim.jMax)/(1.0f/lim.jMax-1.0f/lim.jMin);
        if (tmp_val >= 0)
        {
          float a_norm = sqrtf(tmp_val);
          t1 = (a_norm - ini.a)/lim.jMax;
          t2 = 0.0;
          t3 = -a_norm/lim.jMin;
        }
        else
          numeric_error = true;
      }
      else if (d == -1)
      {
        float tmp_val = (2*(vr-ini.v)+ini.a*ini.a/lim.jMin)/(1.0f/lim.jMin-1.0f/lim.jMax);
        if (tmp_val >= 0)
        {
          float a_norm = -sqrtf(tmp_val);
          t1 = (a_norm - ini.a)/lim.jMin;
          t2 = 0;
          t3 = -a_norm/lim.jMax;
        }
        else
          numeric_error = true;
      }
      else
      {
        assert(0 && "sth wrong here.\n");
      }
    }

    if (!numeric_error)
    {
      tP.t[0] = t1;
      tP.t[1] = t2;
      tP.t[2] = t3;
      completeParam(tP, ini);
    }
    else
    {
      zeroAccParam(ini,lim,tP);
    }
  }

  //---
  __host__ __device__
  void zeroAccParam(const State &ini, const Limit &lim, StageParam& tP)
  {
    float t1,t2,t3;
    if (- ini.a >= 0)
    {   //increase
      t1 = (- ini.a)/lim.jMax;
      tP.j[0] = lim.jMax;
    }
    else
    {   //decrease
      t1 = (- ini.a)/lim.jMin;
      tP.j[0] = lim.jMin;
    }
    tP.j[2] = lim.jMax;
    t2 = 0;
    t3 = 0;

    tP.t[0] = t1;
    tP.t[1] = t2;
    tP.t[2] = t3;

    completeParam(tP, ini);
  }
  //---
  __host__ __device__
  float calculateEndPosition(float vr, State ini, Limit lim)
  {
    StageParam tmP;
    solveVelocityTaret(vr, ini, lim, tmP);
    return tmP.p[3];
  }
  //---
  __host__ __device__
  void completeParam(StageParam& tP, const State& ini)
  {
    //--------------
    tP.a[0] = ini.a;
    tP.v[0] = ini.v;
    tP.p[0] = ini.p;

    //--------------------------
    tP.a[1] = calcA(tP.t[0], tP.a[0], tP.j[0]);
    tP.v[1] = calcV(tP.t[0], tP.v[0], tP.a[0], tP.j[0]);
    tP.p[1] = calcP(tP.t[0], tP.p[0], tP.v[0], tP.a[0], tP.j[0]);

    //--------------------------
    tP.a[2] = calcA(tP.t[1], tP.a[1], tP.j[1]);
    tP.v[2] = calcV(tP.t[1], tP.v[1], tP.a[1], tP.j[1]);
    tP.p[2] = calcP(tP.t[1], tP.p[1], tP.v[1], tP.a[1], tP.j[1]);

    //--------------------------
    tP.a[3] = calcA(tP.t[2], tP.a[2], tP.j[2]);
    tP.v[3] = calcV(tP.t[2], tP.v[2], tP.a[2], tP.j[2]);
    tP.p[3] = calcP(tP.t[2], tP.p[2], tP.v[2], tP.a[2], tP.j[2]);

    //--------------------------
    tP.t[1] = tP.t[1] + tP.t[0];
    tP.t[2] = tP.t[2] + tP.t[1];
  }
  //---
  __host__ __device__
  State stageRefGen(StageParam tP, float t)
  {
    State out;
    if (t < tP.t[0])
    {
      out.p = calcP(t, tP.p[0], tP.v[0], tP.a[0], tP.j[0]);
      out.v = calcV(t, tP.v[0], tP.a[0], tP.j[0]);
      out.a = calcA(t, tP.a[0], tP.j[0]);
    }
    else if (t >= tP.t[0] && t < tP.t[1])
    {
      out.a = calcA(t - tP.t[0], tP.a[1], tP.j[1]);
      out.v = calcV(t - tP.t[0], tP.v[1], tP.a[1], tP.j[1]);
      out.p = calcP(t - tP.t[0], tP.p[1], tP.v[1], tP.a[1], tP.j[1]);
    }
    else if (t >= tP.t[1] && t < tP.t[2])
    {
      out.a = calcA(t - tP.t[1], tP.a[2], tP.j[2]);
      out.v = calcV(t - tP.t[1], tP.v[2], tP.a[2], tP.j[2]);
      out.p = calcP(t - tP.t[1], tP.p[2], tP.v[2], tP.a[2], tP.j[2]);
    }
    else
    {
      out.a = calcA(t - tP.t[2], tP.a[3], 0.0);
      out.v = calcV(t - tP.t[2], tP.v[3], tP.a[3], 0.0);
      out.p = calcP(t - tP.t[2], tP.p[3], tP.v[3], tP.a[3], 0.0);
    }
    return out;
  }
  __host__ __device__
  State stageRefGen(StageParam tP, float t,float &u)
  {
    State out;
    if (t < tP.t[0])
    {
      out.p = calcP(t, tP.p[0], tP.v[0], tP.a[0], tP.j[0]);
      out.v = calcV(t, tP.v[0], tP.a[0], tP.j[0]);
      out.a = calcA(t, tP.a[0], tP.j[0]);
      u = tP.j[0];
    }
    else if (t >= tP.t[0] && t < tP.t[1])
    {
      out.a = calcA(t - tP.t[0], tP.a[1], tP.j[1]);
      out.v = calcV(t - tP.t[0], tP.v[1], tP.a[1], tP.j[1]);
      out.p = calcP(t - tP.t[0], tP.p[1], tP.v[1], tP.a[1], tP.j[1]);
      u = tP.j[1];
    }
    else if (t >= tP.t[1] && t < tP.t[2])
    {
      out.a = calcA(t - tP.t[1], tP.a[2], tP.j[2]);
      out.v = calcV(t - tP.t[1], tP.v[2], tP.a[2], tP.j[2]);
      out.p = calcP(t - tP.t[1], tP.p[2], tP.v[2], tP.a[2], tP.j[2]);
      u = tP.j[2];
    }
    else
    {
      out.a = calcA(t - tP.t[2], tP.a[3], 0.0);
      out.v = calcV(t - tP.t[2], tP.v[3], tP.a[3], 0.0);
      out.p = calcP(t - tP.t[2], tP.p[3], tP.v[3], tP.a[3], 0.0);
      u = 0;
    }
    return out;
  }
  //---
  __host__ __device__
  int sign(float in)
  {
    if (in > JLT_signEpsilon)
      return 1;
    else if (in < -JLT_signEpsilon)
      return -1;
    else
      return 0;
  }
  //---
  __host__ __device__
  inline float calcA(float t, float a0, float j)
  {
    return a0 + j*t;
  }
  //---
  __host__ __device__
  inline float calcV(float t, float v0, float a0, float j)
  {
    return v0 + a0*t + 0.5f*j*t*t;
  }
  //---
  __host__ __device__
  inline float calcP(float t, float x0, float v0, float a0, float j)
  {
    return x0 + v0*t + 0.5f*a0*t *t + 1 / 6.0f * j*t*t*t;
  }
  //---

};
#endif

