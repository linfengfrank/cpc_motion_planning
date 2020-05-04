#include "cpc_motion_planning/JLT.h"
#include <iostream>
#include <cmath>
#include <assert.h>

JLT::JLT()
{
}


JLT::~JLT()
{
}

int JLT::sign(double in)
{
    if (in > signEpsilon)
        return 1;
    else if (in < -signEpsilon)
        return -1;
    else
        return 0;
}

//Draft, can be improved.
void JLT::solveVelocityTaret(double vr, State ini, Limit lim, StageParam& tP)
{
    //find the end velocity, when acceleration immediately goes to zero
    double vE;
    if (ini.a >=0)
        vE = ini.v + ini.a*fabs(ini.a/lim.jMin)/2.0;
    else
        vE = ini.v + ini.a*fabs(ini.a/lim.jMax)/2.0;

    //determine the cruise direction
    int d = sign(vr - vE);

    //determine the cruise acceleration
    double ac;
    if (d == 1)
        ac = lim.aMax;
    else if(d == -1)
        ac = lim.aMin;
    else
        ac = 0.0;

    //determine t1, j1, v1
    double t1;
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
    double v1 = ini.v + ini.a*t1 + 0.5*tP.j[0]*t1*t1;

    //determine t3, j3, v3bar, v2bar
    double t3;
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
    double v3bar = ac*t3 + 0.5*tP.j[2]*t3*t3;
    double v2bar = vr - v1 - v3bar;

    double t2;
    if (d == 0)
        t2 = 0;
    else
        t2 = v2bar / ac;

    if (t2 >= 0)
    {
        tP.t[0] = t1;
        tP.t[1] = t2;
        tP.t[2] = t3;
    }
    else
    {
        if (d==1)
        {
            double a_norm = sqrt((2.0*(vr-ini.v)+ini.a*ini.a/lim.jMax)/(1.0/lim.jMax-1.0/lim.jMin));
#ifdef DEBUGJLT
            if (std::isnan(a_norm))
                assert(0 && "a_norm is nan");
#endif
            t1 = (a_norm - ini.a)/lim.jMax;
            t2 = 0.0;
            t3 = -a_norm/lim.jMin;
        }
        else if (d == -1)
        {
            double a_norm = -sqrt((2*(vr-ini.v)+ini.a*ini.a/lim.jMin)/(1/lim.jMin-1/lim.jMax));
#ifdef DEBUGJLT
            if (std::isnan(a_norm))
                assert(0 && "a_norm is nan");
#endif
            t1 = (a_norm - ini.a)/lim.jMin;
            t2 = 0;
            t3 = -a_norm/lim.jMax;
        }
        else
        {
            assert(0 && "sth wrong here.\n");
        }
        tP.t[0] = t1;
        tP.t[1] = t2;
        tP.t[2] = t3;
    }
    completeParam(tP, ini);
}

//Draft, can be improved.
double JLT::calculateEndPosition(double vr, State ini, Limit lim)
{
    StageParam tmP;
    solveVelocityTaret(vr, ini, lim, tmP);
    return tmP.p[3];
}

void JLT::completeParam(StageParam& tP, const State& ini)
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

JLT::State JLT::stageRefGen(StageParam tP, double t)
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

int JLT::solveTPBVP(double pr, double vr, State ini, Limit lim, TPBVPParam & P)
{
    int ok = 0;
    double pf = calculateEndPosition(vr, ini, lim);
    double dist_err = pr - pf;
    int cruise_sign = sign(dist_err);
    double cruise_velocity;
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
    if(cruise_sign == 0 || fabs(dist_err) < reachEpsilon)
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
        double tgtDiff = (pf - pr) * static_cast<double>(cruise_sign);
        //Case 2: Undershoot the target, fill up the missing distance with max speed cruise phase
        if (tgtDiff <= -reachEpsilon)
        {
            double cruise_time = 0;
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
        else if(tgtDiff >= reachEpsilon)
        {
            double tHigh = ZCP_U.t[2];
            double tLow = 0;
            double tProbe = 0;
            double tDiff = 0;
            for (int i = 0; i < 64; i++)
            {
                tProbe = (tHigh + tLow)*0.5;
                mid = stageRefGen(ZCP_U, tProbe);
                pf = calculateEndPosition(vr, mid, lim);

                tDiff = (pf - pr)* static_cast<double>(cruise_sign);

                if(tDiff <= -reachEpsilon)
                {
                    tLow = tProbe;
                }
                else if(tDiff >= reachEpsilon)
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

JLT::State JLT::TPBVPRefGen(const TPBVPParam& P, double t)
{
    State out;
    if (t <= P.T[0])
    {
        out = stageRefGen(P.p1, t);
    }
    else if (t > P.T[0] && t < P.T[1])
    {
        out.p = calcP(t - P.T[0], P.p1.p[3], P.p1.v[3], P.p1.a[3], 0);
        out.v = calcV(t - P.T[0], P.p1.v[3], P.p1.a[3], 0);
        out.a = calcA(t - P.T[0], P.p1.a[3], 0);
    }
    else
    {
        out = stageRefGen(P.p2, t - P.T[1]);
    }
    return out;
}
