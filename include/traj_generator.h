/*
Description: Trajectory generator
     Author: Shihao Feng
      Email: 13247344844@163.com
Update time: 2022-11-14
*/

#ifndef TRAJ_GENERATOR_H
#define TRAJ_GENERATOR_H

#include <iostream>
#include <vector>
using namespace std;
#include <math.h>
#include <Eigen/Dense> 
using namespace Eigen;
#include "pinocchio/algorithm/kinematics.hpp" 
#include "pinocchio/spatial/se3.hpp"
namespace pin = pinocchio;

enum class TrajType{Bezier, Cycloid};

class TimeScale
{
public:
    // TimeScale();
    // ~TimeScale();
    Vector3d calc_s(const double &T, const double &s, int order);
    // Vector3d quick_foot_fall(const double &T, const double &s);

    inline bool check(const double &s)
    {
        if (s >= 0. && s <= 1.) {return true;}
        cout << "[TimeScale]: 's' should in the interval [0,1]\n";
        return false;
    }
};

class TrajGenerator
{
public:
    TrajGenerator();
    // ~TrajGenerator();

    vector<Vector3d> p2p_traj(const Vector3d &q_start, 
                            const Vector3d &q_end, 
                            const double &T, 
                            const double &s,
                            int order = 5);

    pin::SE3 p2p_traj(const pin::SE3 &T_start, 
                    const pin::SE3 &T_end, 
                    const double &T,
                    const double &s, 
                    int order = 5,
                    bool coupled = true); 

    vector<Vector3d> sw_traj(const Vector3d &p_hf_start, 
                            const Vector3d &p_hf_end, 
                            const double &h_gc, 
                            const double &T_sw, 
                            const double &s_sw,
                            TrajType traj_type = TrajType::Cycloid);

private:
    inline size_t _factorial(int num) // 递归方法求阶乘
    {
        if (num ==0) {return 1;}
        return num*_factorial(num-1);
    }

    double _bernstein_ploy(const int &k, const int &n, const double &s);
    
    vector<Vector3d> _bezier_traj(const MatrixX3d &ctrl_points, 
                                const double &T, 
                                const double &s);

    TimeScale _time_scale;
};

#endif