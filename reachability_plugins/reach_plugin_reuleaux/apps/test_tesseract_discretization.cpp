
#include <reach_plugin_reuleaux/quaternion_discretization.h>
#include <stdio.h>
#include<fstream>

int main(int argc, char* argv[])
{
    TesseractDiscretization td;
    
    int n = 2;
    std::vector<Eigen::Quaterniond> qs;

    for(int i = 1; i <= n; ++i)
    {
        qs.clear();
        qs = td.generateQuaternions(i);
        printf("Num of points for discretization: %d : %ld \n", i, qs.size());
    }

    // Store
    /*std::ofstream fp;
    fp.open("/home/ana/Desktop/test.txt");
    for(auto p : points)
        fp << p(0) << " " << p(1) << " " << p(2) << std::endl; 
    fp.close();
*/
    return 0;
}
