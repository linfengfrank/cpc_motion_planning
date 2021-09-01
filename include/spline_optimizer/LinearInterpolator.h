#ifndef LINEARINTERPOLATOR_H
#define LINEARINTERPOLATOR_H

#include <map>
#include <vector>
#include <mid_plan/grid_graph.h>

 /**
  * Provides a basic interpolation mechanism in C++ using the STL.
  * Maybe not the fastest or most elegant method, but it works (for
  * linear interpolation!!), and is fast enough for a great deal of
  * purposes. It's also super easy to use, so that's a bonus.
  */
class LinearInterpolator {
  public:
    LinearInterpolator() {}

    bool addPath(const std::vector<CUDA_GEO::pos> &path, double &length)
    {
      if (path.size() < 2)
      {
        std::cout << "Path size smaller than 2, cannot interpolate!"<<std::endl;
        return false;
      }

      length = 0;
      addDataPoint(length,path[0]);
      for (int i=0; i<path.size() - 1; i++)
      {
        CUDA_GEO::pos diff = path[i+1] - path[i];
        length += sqrtf(diff.square());
        addDataPoint(length,path[i+1]);
      }
      return true;
    }

    std::vector<CUDA_GEO::pos> interpolate_path(int num, const std::vector<CUDA_GEO::pos> &path)
    {
      data.clear();
      std::vector<CUDA_GEO::pos> inter_path;
      double length = 0;
      if (!addPath(path,length))
      {
        std::cout << "Failed to add the path!"<<std::endl;
        return inter_path;
      }

      double delta = length/(num-1);

      for (int i=0; i<num; i++)
      {
        inter_path.push_back(interpolate(delta*i));
      }
      return inter_path;
    }

    /**
     * Adds a data point to the interpolator for future interpolation
     * @param x The anchor point where this data is located
     * @param d A CUDA_GEO::pos position point
     */
    void addDataPoint(double x, const CUDA_GEO::pos &d) {
      // just add it to the map
      data[x] = d;
    }

    /**
     * Interpolates our data sequence at a given point, for a given column of data
     * @param  x      The anchor point to interpolate at
     * @return y      The interpolated value CUDA_GEO::pos
     */
    CUDA_GEO::pos interpolate(double x) {
      // loop through all the keys in the map
      // to find one that is greater than our intended value
      std::map<double, CUDA_GEO::pos >::iterator it = data.begin();
      bool found = false;
      while(it != data.end() && !found) {
        if(it->first >= x) {
          found = true;
          break;
        }

        // advance the iterator
        it++;
      }

      // check to see if we're outside the data range
      if(it == data.begin()) {
        return data.begin()->second;
      }
      else if(it == data.end()) {
        // move the point back one, as end() points past the list
        it--;
        return it->second;
      }
      // check to see if we landed on a given point
      else if(it->first == x) {
        return it->second;
      }

      // nope, we're in the range somewhere
      // collect some values
      double xb = it->first;
      CUDA_GEO::pos yb = it->second;
      it--;
      double xa = it->first;
      CUDA_GEO::pos ya = it->second;

      // and calculate the result!
      // formula from Wikipedia
      return (ya + (yb - ya) * (x - xa) / (xb - xa));
    }

  private:
    std::map<double, CUDA_GEO::pos > data;
};

#endif // LINEARINTERPOLATOR_H
