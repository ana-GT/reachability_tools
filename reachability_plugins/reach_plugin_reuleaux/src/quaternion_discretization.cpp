#include <reach_plugin_reuleaux/quaternion_discretization.h>


TesseractDiscretization::TesseractDiscretization()
{

}

/**
 * @function generateQuaternions 
 */
std::vector<Eigen::Quaterniond> TesseractDiscretization::generateQuaternions(const int &_n)
{
    std::vector<Eigen::Vector4d> points;
    points = generatePoints(_n);

    std::vector<Eigen::Quaterniond> qs;
    for(auto pi : points)
        qs.push_back( Eigen::Quaterniond(pi(3), pi(0), pi(1), pi(2)) );
    
    return qs;
}

/**
 * @function generatePoints 
 */
std::vector<Eigen::Vector4d> TesseractDiscretization::generatePoints(const int &_n)
{
    // 1. Generate initial cubes
    std::vector<Cube> C0;
    generateCubes(C0);

   std::vector<Cube> Cn_1;
   std::vector<Cube> Cn;

   Cn = C0;
   for(unsigned int i = 1; i <= _n; ++i)
   {
    Cn_1 = Cn;
    Cn.clear();
    for(unsigned int j = 0; j < Cn_1.size(); ++j)
    {
        std::vector<Cube> subdivided;
        subdivided = Cn_1[j].subdivide();
        Cn.insert(Cn.end(), subdivided.begin(), subdivided.end());
    }
   }

    // Get points
    std::vector<Eigen::Vector4d> res;
    getPointsFromCubes(Cn, res);

    // Normalize
    std::vector<Eigen::Vector4d> pn;

    for(auto rp : res)
        pn.push_back(rp.normalized());

   return pn;

}


/**
 * @function generateSquares 
 */
void TesseractDiscretization::generateCubes( std::vector<Cube> &_C0)
{
    _C0.resize(8);

    _C0[0].setCorners(0, true);
    _C0[1].setCorners(0, false);
    _C0[2].setCorners(1, true);
    _C0[3].setCorners(1, false);
    _C0[4].setCorners(2, true);
    _C0[5].setCorners(2, false);
    _C0[6].setCorners(3, true);
    _C0[7].setCorners(3, false);
}

/**
 * @function getPointsFromSquares 
 */
void TesseractDiscretization::getPointsFromCubes(const std::vector<Cube> &_Cn, 
                                                 std::vector<Eigen::Vector4d> &_points)
{
    _points.clear();
    for(auto ci : _Cn) 
    {
        for(int j = 0; j < 8; ++j)
         addIfNotThere(_points, ci);
    }

}

void TesseractDiscretization::addIfNotThere(std::vector<Eigen::Vector4d> &_points, 
                       const Cube &_ci)
{
    std::vector<Eigen::Vector4d> cs = {_ci.c[0], _ci.c[1], _ci.c[2], _ci.c[3],
                                       _ci.c[4], _ci.c[5], _ci.c[6], _ci.c[7]};

    double eps = 0.001;
    for(const auto px : _points)
    {
        for(std::vector<Eigen::Vector4d>::iterator it = cs.begin();
            it != cs.end();)
        {
          if( (px - (*it)).norm() < eps )
            it = cs.erase(it);
          else if ( (px + (*it)).norm() < eps)
            it = cs.erase(it);
          else
            ++it;     
        }

        if(cs.empty())
            break;    
    }

    // Add if not in list
    for(auto ci : cs)
        _points.push_back(ci);


}                    
                


////////////////////////////////////////////////////////

/**
 * @function CubeSurfaceDiscretization 
 */
CubeSurfaceDiscretization::CubeSurfaceDiscretization()
{

}

/**
 * @function generatePoints 
 */
std::vector<Eigen::Vector3d> CubeSurfaceDiscretization::generatePoints(const int &_n)
{
   // 1. Generate initial squares
   std::vector<Square> C0;
   generateSquares(C0);

   std::vector<Square> Cn_1;
   std::vector<Square> Cn;

   Cn = C0;
   for(unsigned int i = 1; i <= _n; ++i)
   {
    Cn_1 = Cn;
    Cn.clear();
    for(unsigned int j = 0; j < Cn_1.size(); ++j)
    {
        std::vector<Square> subdivided;
        subdivided = Cn_1[j].subdivide();
        Cn.insert(Cn.end(), subdivided.begin(), subdivided.end());
    }
   }

    // Get points
    std::vector<Eigen::Vector3d> res;
    getPointsFromSquares(Cn, res);

    // Normalize
    std::vector<Eigen::Vector3d> pn;

    for(auto rp : res)
        pn.push_back(rp.normalized());

   return pn;
}

/**
 * @function generateSquares 
 */
void CubeSurfaceDiscretization::generateSquares( std::vector<Square> &_C0)
{
    std::vector<Eigen::Vector3d> vs;
    vs.push_back(Eigen::Vector3d(-1, -1, -1)); // v0
    vs.push_back(Eigen::Vector3d(-1, -1, 1)); // v1
    vs.push_back(Eigen::Vector3d(-1, 1, 1)); // v2
    vs.push_back(Eigen::Vector3d(-1, 1, -1)); // v3
    vs.push_back(Eigen::Vector3d(1, -1, -1)); // v4
    vs.push_back(Eigen::Vector3d(1, -1, 1)); //  v5
    vs.push_back(Eigen::Vector3d(1, 1, 1)); // v6
    vs.push_back(Eigen::Vector3d(1, 1, -1)); // v7

    _C0.resize(6);

    _C0[0].setCorners(vs[4], vs[5], vs[6], vs[7]);
    _C0[1].setCorners(vs[0], vs[1], vs[2], vs[3]);
    _C0[2].setCorners(vs[5], vs[1], vs[2], vs[6]);
    _C0[3].setCorners(vs[4], vs[0], vs[3], vs[7]);
    _C0[4].setCorners(vs[4], vs[5], vs[1], vs[0]);
    _C0[5].setCorners(vs[7], vs[6], vs[2], vs[3]);
}

/**
 * @function getPointsFromSquares 
 */
void CubeSurfaceDiscretization::getPointsFromSquares(const std::vector<Square> &_Cn, 
                                                     std::vector<Eigen::Vector3d> &_points)
{
    _points.clear();
    for(auto ci : _Cn) 
      addIfNotThere(_points, ci);
}

/**
 * 
 */
void CubeSurfaceDiscretization::addIfNotThere(std::vector<Eigen::Vector3d> &_points, 
                                              const Square &_si)
{
    std::vector<Eigen::Vector3d> cs = {_si.c[0], _si.c[1], _si.c[2], _si.c[3]};

    double eps = 0.001;
    for(const auto px : _points)
    {
        for(std::vector<Eigen::Vector3d>::iterator it = cs.begin();
            it != cs.end();)
        {
          if( fabs(px(0) - (*it)(0)) > eps ||
              fabs(px(1) - (*it)(1)) > eps ||
              fabs(px(2) - (*it)(2)) > eps )
            ++it;    
          else
            it = cs.erase(it);
 
        }

        if(cs.empty())
            break;    
    }

    // Add if not in list
    for(auto ci : cs)
        _points.push_back(ci);

}
