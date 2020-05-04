#ifndef CLASSJLT
#define CLASSJLT
#define DEBUGJLT
#include <sstream>
#include <string>
#include <fstream>
#include <assert.h>
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
        double p;
        double v;
        double a;
        State() :p(0), v(0), a(0){}
    };

    struct Limit
    {
        double vMax,vMin;
        double aMax,aMin;
        double jMax,jMin;
        Limit() :vMax(1.0),vMin(-1.0),aMax(1.0),aMin(-1.0),jMax(1.0),jMin(-1.0)
        {}
    };

    struct StageParam
    {
        double t[3];
        double p[4];
        double v[4];
        double a[4];
        double j[3];
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
        double T[3];
    };

    struct Param3d
    {
        TPBVPParam val[3];
        double T_longest;
        bool ok;
        Param3d()
        {
            T_longest = 0;
            ok = false;
        }
    };

//---
public:
    JLT();
    ~JLT();
    //---
    int solveTPBVP(double pr, double vr, State ini, Limit lim, TPBVPParam & P);
    //---
    State TPBVPRefGen(const TPBVPParam &tP, double t);

private:
    //---
    void solveVelocityTaret(double vr, State ini, Limit lim, StageParam& tP);
    //---
    double calculateEndPosition(double vr, State ini, Limit lim);
    //---
    void completeParam(StageParam& tP, const State& ini);
    //---
    State stageRefGen(StageParam tP, double t);
    //---
    int sign(double in);
    //---
    inline double calcA(double t, double a0, double j)
    {
        return a0 + j*t;
    }
    //---
    inline double calcV(double t, double v0, double a0, double j)
    {
        return v0 + a0*t + 0.5*j*t*t;
    }
    //---
    inline double calcP(double t, double x0, double v0, double a0, double j)
    {
        return x0 + v0*t + 0.5*a0*t *t + 1 / 6.0 * j*t*t*t;
    }
    //---
    const double signEpsilon = 1e-9;
    const double reachEpsilon = 1e-3;

public:
    // a static function to load limit and delta t from a file
    static void loadTimeAndLimFromFile(std::string name, double &dt, Limit &lim)
    {
        std::ifstream infile(name);
        std::string line;
        int idx = 0;
        while (std::getline(infile, line))
        {
            std::istringstream iss(line);
            double _1, _2;
            if (!(iss >> _1 >> _2))
            {
                assert(0 && "JLT Read error.\n");
                break;
            } // error

            switch (idx)
            {
            case 0:
                dt = _1;
                break;
            case 1:
                lim.vMin = _1;
                lim.vMax = _2;
                break;
            case 2:
                lim.aMin = _1;
                lim.aMax = _2;
                break;
            case 3:
                lim.jMin = _1;
                lim.jMax = _2;
                break;
            default:
                assert(0 && "Over loaded JLT.\n");
                break;
            }
            idx++;
        }

        if (idx != 4)
            assert(0 && "JLT Read error.\n");
    }
};
#endif 

