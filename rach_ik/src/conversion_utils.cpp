
#include <rach_ik/conversion_utils.h>

void jointStateToVector(const sensor_msgs::msg::JointState &_js,
                        std::vector<double> &_x)
{
    _x.clear();
    for(auto ji : _js.position)
        _x.push_back(ji);
}