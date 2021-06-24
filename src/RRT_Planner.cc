#include "../include/RRT_Star.h"

using namespace Planning;
 
#define SHOW_ANIMATION true
/*---- main method for simple RRT Star---*/
int main(){
    RRTStar rrt = RRTStar(cv::Point2i(480, 270), cv::Point2i(600, 178)); //result
    //RRTStar rrt = RRTStar(cv::Point2i(100, 900), cv::Point2i(800, 900));
    //RRTStar rrt = RRTStar(cv::Point2i(100, 900), cv::Point2i(800, 900));
    //RRTStar rrt = RRTStar(cv::Point2i(100, 900), cv::Point2i(800, 400));
    rrt.set_MAP("../img/map_slam.jpg");
    //rrt.set_MAP("../img/map_basic.png");
    //rrt.set_MAP("../img/map_s.png");
    //rrt.add_map_padding(20);
    #if USE_LEBESGUE_MEASURE
    std::cout << "Using measure:";
    rrt.create_coverage_map();
    #endif
    rrt.display_MAP(500);
    auto path = rrt.planning(SHOW_ANIMATION);
    std::cout << "Execution complete" << std::endl;
    return 0;
}

