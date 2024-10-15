#pragma once

#include <vector>
#include <Eigen/Geometry>


struct Cube
{
   Eigen::Vector4d c[8];
   
   void setCorners(int idx, bool _positive)
   {
    std::vector<int> indices(3);

    if(idx == 0)
    { indices = {1, 2, 3}; }
    else if(idx == 1)
    { indices = {0, 2, 3}; }
    else if(idx == 2)
    { indices = {0, 1, 3}; }
    else if( idx == 3)
    { indices = {0, 1, 2}; }

    int count = 0;
    for(int i = 0; i < 2; ++i)
    {
      for(int j = 0; j < 2; ++j)
      {
        for(int k = 0; k < 2; ++k)
        {
          Eigen::Vector4d v;
          v(idx) = _positive? 1 : -1;
          v(indices[0]) = (i % 2 == 0)? 1 : -1;
          v(indices[1]) = (j % 2 == 0)? 1 : -1;
          v(indices[2]) = (k % 2 == 0)? 1 : -1;

          c[count] = v;
          count++;
        } // for k
      } // for j
    } // for i

   } // setCorners

  std::vector<Cube> subdivide()
  {
    // Computer center of cube
    Eigen::Vector4d m;
    m = Eigen::Vector4d::Zero();
    for(int i = 0; i < 8; ++i)
      m = m + c[i];
    
    m = m / 8.0;

    // Computer subdivision
    std::vector<Cube> result(8);
    for(int i = 0; i < 8; ++i)
      result[i].setFromOppositeCorners(c[i], m);

    return result;
  }

  bool setFromOppositeCorners(Eigen::Vector4d v1, Eigen::Vector4d v2)
  {
    // 1. Find the non-changing index
    Eigen::Vector4d diff; diff = v2 - v1;

    double eps = 0.0001;
    int idx;
    std::vector<int> indices(3);
    if( fabs(diff(0)) < eps )
    {  idx = 0; indices = {1, 2, 3}; }
    else if( fabs(diff(1)) < eps)
    {  idx = 1; indices = {0, 2, 3}; }
    else if( fabs(diff(2)) < eps)
    { idx = 2; indices = {0, 1, 3}; }
    else if( fabs(diff(3)) < eps)
    { idx = 3; indices = {0, 1, 2}; }
    else
    {
      printf("SOMETHING IS WRONG! NO SAME INDEX FOR A FACE OF THE CUBE!!! \n");
      return false;
    }


    // Set corners
    c[0] = v1;
    c[7] = v2;

    for(int i = 1; i < 7; ++i)
      c[i] = v1;

    // v1 + (d0, 0, 0)
    c[1](indices[0]) += diff(indices[0]);

    // (0, d1, 0)
    c[2](indices[1]) += diff(indices[1]);  

    // (0, 0, d2)
    c[3](indices[2]) += diff(indices[2]);

    // (d0, d1, 0)
    c[4](indices[0]) += diff(indices[0]);
    c[4](indices[1]) += diff(indices[1]);  

    // (d0, 0, d2)
    c[5](indices[0]) += diff(indices[0]);  
    c[5](indices[2]) += diff(indices[2]);  

    // (0, d1, d2)
    c[6](indices[1]) += diff(indices[1]);  
    c[6](indices[2]) += diff(indices[2]);  

    return true;
  }


};

/**
 * @class TesseractDiscretization 
 */
class TesseractDiscretization
{
  public:
  TesseractDiscretization();
  std::vector<Eigen::Quaterniond> generateQuaternions(const int &_n);

  protected:
  std::vector<Eigen::Vector4d> generatePoints(const int &_n);
  void generateCubes( std::vector<Cube> &_C0);
  void getPointsFromCubes(const std::vector<Cube> &_Cn, 
                          std::vector<Eigen::Vector4d> &_points);
  void addIfNotThere(std::vector<Eigen::Vector4d> &_points, 
                       const Cube &_ci);


};

/**
 * @struct Square 
 */
struct Square
{
  Eigen::Vector3d c[4];
  void setCorners(const Eigen::Vector3d &_c0, const Eigen::Vector3d &_c1,
                  const Eigen::Vector3d &_c2, const Eigen::Vector3d &_c3)
  {
    c[0] = _c0; c[1] = _c1, 
    c[2] = _c2; c[3] = _c3;
  }

  std::vector<Square> subdivide()
  {
    std::vector<Eigen::Vector3d> m;
    m.push_back( 0.5*(c[0] + c[1]) );
    m.push_back( 0.5*(c[1] + c[2]) );
    m.push_back( 0.5*(c[2] + c[3]) );
    m.push_back( 0.5*(c[3] + c[0]) );
    m.push_back( 0.25*(c[0] + c[1] + c[2] + c[3]) ); 

    std::vector<Square> result(4);
    result[0].setCorners(c[0], m[0], m[4], m[3]);
    result[1].setCorners(m[0], c[1], m[1], m[4]);
    result[2].setCorners(m[4], m[1], c[2], m[2]);
    result[3].setCorners(m[3], m[4], m[2], c[3]);

    return result;
}


};

/**
 * 
 */
class CubeSurfaceDiscretization
{
 public:
    CubeSurfaceDiscretization();
    std::vector<Eigen::Vector3d> generatePoints(const int &_n);

 protected:

    void generateSquares( std::vector<Square> &_C0);
    void getPointsFromSquares(const std::vector<Square> &_Cn, 
                          std::vector<Eigen::Vector3d> &_points);
    std::vector<Square> subdivideSquare(const Square &_c);
    void addIfNotThere(std::vector<Eigen::Vector3d> &_points, 
                       const Square &_si);
};

