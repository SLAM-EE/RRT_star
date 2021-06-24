/* C++ port of RRT from PythonRobotics by author AtsushiSakai
 * Aswin P Ajayan 
 * aswin@ee.iitb.ac.in
 */

#include "../include/RRT.h"

using namespace Planning;


class Node : public RRT_Node{
    public:
        float cost;
        Node(cv::Point2i loc):RRT_Node(loc), cost(0.0){}
        Node(int x, int y):RRT_Node(x, y), cost(0.0){}
};

class RRTStar : public RRT{
    public:

};
 
/*---- main method for simple RRT ---*/
int main(){
    Planning::RRT rrt = Planning::RRT(cv::Point2i(100, 900), cv::Point2i(800, 900));
    rrt.set_MAP("../img/map_basic.png");
    rrt.display_MAP(2000);
    auto path = rrt.planning(true);
    std::cout << "Execution complete" << std::endl;
    return 0;
}

