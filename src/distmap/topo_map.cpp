#include "distmap/TopoMap.h"
#include <string.h>
TopoMap::TopoMap(int maxX, int maxY, int maxZ):
    MapBase(),
    _w(maxX),
    _h(maxY),
    _d(maxZ)
{
    _map = new SeenDist[_w*_h*_d];
    memset(_map, 0, _w*_h*_d*sizeof(SeenDist));
}

TopoMap::~TopoMap()
{
    delete [] _map;
}


CUDA_GEO::pos TopoMap::findNearestPoint(CUDA_GEO::pos _p, float tor_min_dist, bool valid_z) const {
    CUDA_GEO::coord _c = transformPosToCoord(_p);
    float _res = this->getGridStep();
    float _c_d = this->distAt(_c, -1) * _res;
    if ( _c_d < -0.1 || _c_d > tor_min_dist ) {
        return _p;
    }
    std::cout << "\033[33m" << "[Path Searcher]: reset point" << "\033[0m"<< std::endl;
    std::multimap<float, CUDA_GEO::coord> _openSet;
    CUDA_GEO::coord _tmp_c = _c;
    float _tmp_c_d = this->distAt(_tmp_c, -1)*_res;
    int max_iteration = 10000;
    _openSet.insert(std::make_pair(_tmp_c_d, _tmp_c));
    while (_tmp_c_d > -0.1 && _tmp_c_d <= tor_min_dist && max_iteration > 0 && !_openSet.empty()) {
        max_iteration--;
        std::multimap<float, CUDA_GEO::coord>::iterator _it;
        _it = _openSet.end();
        --_it;
        _tmp_c_d = _it->first;
        _tmp_c = _it->second;
        _openSet.erase(_it);
        for (int i = -1; i <= 1; i++) {
            for (int j = -1; j <= 1; j++) {
                for (int k = (valid_z? -1: 0); k <= (valid_z? 1: 0); k++) {
                    if (i == 0 && j == 0 && k==0) {
                        continue;
                    }
                    CUDA_GEO::coord _delta_c(i, j, k);
                    float _tmp_tmp_c_d = this->distAt(_delta_c+_tmp_c, -1)*_res;
                    if (_tmp_tmp_c_d > _tmp_c_d || _tmp_tmp_c_d < -0.1) {
                        _openSet.insert(std::make_pair(_tmp_tmp_c_d, _delta_c + _tmp_c));
                    }
                }
            }
        }
    }
    _p = transformCoordToPos(_tmp_c);
    std::cout << "\033[33m" << "[Path Searcher]: (" << max_iteration << ") "<< _p.x << ", " << _p.y  << ", " << _p.z << "\033[0m"<< std::endl;
    return _p;
}

float TopoMap::distAt(const CUDA_GEO::coord & s, const float default_value) const
{
    if (s.x<0 || s.x>=_w ||
            s.y<0 || s.y>=_h ||
            s.z<0 || s.z>=_d)
        return default_value;

    return _map[s.z*_w*_h+s.y*_w+s.x].d;
}

float TopoMap::distAt(const float&x, const float&y, const float&z, const float default_value) const
{
    CUDA_GEO::pos _origin = this->getOrigin();
    float _res = this->getGridStep();
    CUDA_GEO::coord _s;
    _s.x = floorf((x - _origin.x)/_res);
    _s.y = floorf((y - _origin.y)/_res);
    _s.z = floorf((z - _origin.z)/_res);
    return distAt(_s, default_value);
}

void TopoMap::distWithGrad(const CUDA_GEO::pos& pt, float default_value, float& dist, CUDA_GEO::pos& grad) const
{
    float _res = this->getGridStep();
    dist = this->distAt(pt.x, pt.y, pt.z, default_value) * _res;
    grad.x = 0;
    grad.y = 0;
    grad.z = 0;
    for (float _x = pt.x - _res; _x < pt.x + _res; _x+=_res) {
        for (float _y = pt.y - _res; _y < pt.y + _res; _y+=_res) {
            for (float _z = pt.z - _res; _z < pt.z + _res; _z += _res) {
                if ((fabsf(_x - pt.x) < 1e-3f) &&
                (fabsf(_y - pt.y) < 1e-3f) &&
                (fabsf(_z - pt.z) < 1e-3f))
                    continue;
                float dist_n = this->distAt(_x, _y, _z, default_value)*this->getGridStep();
                if (fabsf(dist_n - default_value) < 1e-3f) {
                    continue;
                }
                float d_dist = dist_n - dist;
                float x_2 = (_x - pt.x)*(_x - pt.x);
                float y_2 = (_y - pt.y)*(_y - pt.y);
                float z_2 = (_z - pt.z)*(_z - pt.z)/4.0f;
                float offset_norm = sqrtf(x_2 + y_2 + z_2);
                float scale = d_dist/offset_norm;
                CUDA_GEO::pos _n_grad((_x - pt.x)*scale,
                        (_y - pt.y)*scale,
                        (_z - pt.z)*scale/2.0f);
                grad = grad + _n_grad;
            }
        }
    }
}

CUDA_GEO::pos TopoMap::indexToPos(const CUDA_GEO::coord & s) const
{
    CUDA_GEO::pos Pos;
    CUDA_GEO::pos _origin = this->getOrigin();
    float _res = this->getGridStep();
    CUDA_GEO::coord _s;
    Pos.x = _origin.x + static_cast<float>(s.x) * _res;
    Pos.y = _origin.y + static_cast<float>(s.y) * _res;
    Pos.z = _origin.z + static_cast<float>(s.z) * _res;
    return Pos;
}

bool TopoMap::isSeen(const CUDA_GEO::coord & s, const bool default_value) const
{
    if (s.x<0 || s.x>=_w ||
            s.y<0 || s.y>=_h ||
            s.z<0 || s.z>=_d)
        return default_value;

    return _map[s.z*_w*_h+s.y*_w+s.x].s;
}


bool TopoMap::isSeen(const float&x, const float&y, const float&z, const bool default_value) const
{
    CUDA_GEO::pos _origin = this->getOrigin();
    float _res = this->getGridStep();
    CUDA_GEO::coord _s;
    _s.x = floorf((x - _origin.x)/_res);
    _s.y = floorf((y - _origin.y)/_res);
    _s.z = floorf((z - _origin.z)/_res);
    return isSeen(_s, default_value);
}

void TopoMap::copyData(const cpc_aux_mapping::grid_map &msg)
{
    if (msg.x_size != _w || msg.y_size != _h || msg.z_size != _d)
    {
        printf("Dimension mismatch during map data copying!\n Map is not copied!\n");
        return;
    }

    setMapSpecs(CUDA_GEO::pos(static_cast<float>(msg.x_origin),
                            static_cast<float>(msg.y_origin),
                            static_cast<float>(msg.z_origin)), msg.width);
    memcpy(_map, msg.payload8.data(), sizeof(SeenDist)*_w*_h*_d);
}
