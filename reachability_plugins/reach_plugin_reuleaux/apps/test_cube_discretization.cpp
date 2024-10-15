
#include <reach_plugin_reuleaux/quaternion_discretization.h>
#include <stdio.h>
#include<fstream>

int main(int argc, char* argv[])
{
    CubeSurfaceDiscretization csd;
    
    int n = 5;
    std::vector<Eigen::Vector3d> points;

    for(int i = 1; i <= n; ++i)
    {
        points.clear();
        points = csd.generatePoints(i);
        printf("Num of points for discretization: %d : %d \n", i, points.size());
    }

    // Store
    std::ofstream fp;
    fp.open("/home/ana/Desktop/test.txt");
    for(auto p : points)
        fp << p(0) << " " << p(1) << " " << p(2) << std::endl; 
    fp.close();

}
