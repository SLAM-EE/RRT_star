///* C++ port of RRT from PythonRobotics by author AtsushiSakai
// * Aswin P Ajayan 
// * aswin@ee.iitb.ac.in
// */
//
//
//#include <cmath>
//#include <complex>
//#include <cstdint>
//#include <iostream>
//#include <limits>
//#include <opencv2/core.hpp>
//#include <opencv2/core/hal/interface.h>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgcodecs.hpp>
//#include <opencv2/imgproc.hpp>
//#include <stdlib.h>     /* srand, rand */
//#include <time.h> 
//#include <math.h>
//#include <bits/stdc++.h>
//#include <limits>
//#include <opencv2/opencv.hpp>
//
//
//class RRT_Node{
//    /* RRT node*/
//    public:
//        cv::Point2i loc;
//        std::vector<cv::Point2i> path;
//        RRT_Node *parent;
//        RRT_Node(cv::Point2i loc){
//            this->loc = loc;
//            this->parent = NULL;
//        }
//        RRT_Node(int x, int y){
//            this->loc = cv::Point2i(x, y);
//            this->parent = NULL;
//        }
//};
//class  RRT{
//    /*Parameters 
//     * start: Start Position RRT_Node
//     * end: goal location RRT_Node
//     * MAP: binary occupancy GridMap cv:Mat(CV_8UC1) 
//     */
//    public:
//        RRT_Node start, end;
//        int expand_dist, goal_sample_rate, max_iter, max_rand, min_rand=0;
//        float path_resolution;
//
//
//        //imout is used to read in MAP image as well as to 
//        //display the out;
//        cv::Mat imout, MAP;
//
//        std::vector<RRT_Node *> node_list;
//
//        RRT(cv::Point2i start, cv::Point2i goal, int expand_dist=100,
//                int goal_sample_rate=5, int max_iter=500,
//                float path_resolution=25.0)
//            :start(RRT_Node(start)), end(RRT_Node(goal)), expand_dist(expand_dist),
//             goal_sample_rate(goal_sample_rate), max_iter(max_iter),
//             path_resolution(path_resolution){
//                 std::srand(time(NULL));} 
//
//        void set_sim_params(int expand_dist=300, int goal_sample_rate=5,
//                int max_iter=500){
//            this->expand_dist = expand_dist;
//            this->goal_sample_rate = goal_sample_rate;
//            this->max_iter = max_iter;
//        }
//
//        void set_MAP(std::string file_path="../img/map_s.png"){
//            imout = cv::imread(file_path, cv::IMREAD_COLOR);
//            cv::Mat gray;
//            cv::cvtColor(imout, gray, cv::COLOR_BGR2GRAY);
//            MAP.create(imout.rows, imout.cols, CV_8UC1);
//            cv::threshold(gray, MAP, 0, 255, cv::THRESH_OTSU); 
//            this->max_rand = MAP.rows;
//        }
//
//        void display_MAP(int delay_ms=0){
//            cv::imshow("Occupancy Grid Map", MAP);
//            int k = cv::waitKey(delay_ms);
//            cv::destroyAllWindows();
//        }
//        
//        RRT_Node * get_random_node(){
//            RRT_Node *rnd;
//            if(std::rand() % 100 > goal_sample_rate)
//                rnd = new RRT_Node(std::rand() % max_rand,
//                        std::rand() % max_rand);
//            else
//                rnd = new RRT_Node(end);
//            //std::cout<<"x: "<<rnd->loc.x << " y: " << rnd->loc.y<<"\n";
//            return rnd;
//        }
//
//        RRT_Node * get_nearest_node(RRT_Node * rnd_node){
//            //std::vector<double> dist;
//            int dist, min_dist = std::numeric_limits<int>::max();
//            RRT_Node *nearest_node;
//            for(auto node: node_list){
//                cv::Point2i diff = node->loc - rnd_node->loc; 
//                dist = diff.ddot(diff);
//                if(dist < min_dist){
//                    min_dist = dist;
//                    nearest_node = node;
//                }
//            }
//            return nearest_node;
//        }
//
//        RRT_Node * steer(RRT_Node *from_node, RRT_Node *to_node){
//            RRT_Node *new_node;
//            double dist, theta;
//            new_node = new RRT_Node(from_node->loc);
//            cv::Point2f diff = to_node->loc - new_node->loc;
//            dist = std::hypotf(diff.y, diff.x);
//            /*new_node->path.push_back(new_node->loc);
//
//            if(dist > expand_dist) dist = expand_dist; 
//            
//            int n_expand = (int) std::floor(dist / path_resolution);
//            for(int i=0; i < n_expand; i++){
//                new_node->loc.x += (int)std::floor(path_resolution * std::cos(theta));
//                new_node->loc.y += (int)std::floor(path_resolution * std::sin(theta));
//                new_node->path.push_back(new_node->loc);
//            }
//
//            new_node->path.push_back(to_node->loc);
//            new_node->parent = from_node;
//            */
//            if(dist > expand_dist){
//                dist = expand_dist;
//                theta = std::atan2(diff.y, diff.x);
//                new_node->loc.x += (int)std::floor(dist * std::cos(theta));
//                new_node->loc.y += (int)std::floor(dist * std::sin(theta));
//            }
//            new_node->parent = from_node;
//
//            //std::cout<<"x: "<<new_node->loc.x << " y: " << new_node->loc.y<<"\n";
//            return new_node;
//        }
//
//
//        bool check_collision(RRT_Node *from_node, RRT_Node *to_node){
//            int x0 = from_node->loc.x; int y0 = from_node->loc.y;
//            int x1 = to_node->loc.x; int y1 = to_node->loc.y;
//
//
//            //line drawing algorithm 
//            int steep=(abs(y1-y0)>abs(x1-x0))?1:0;
//            if (steep){ 
//                std::swap(x0, y0); std::swap(x1,y1);
//            }
//            if(x0>x1){ 
//                std::swap(x0,x1); std::swap(y0,y1);
//            }
//            
//            int dErr=abs(y1-y0);
//            int yStep=y0>y1?-1:1;
//            int dX=x1-x0;
//            int err=dX>>1;
//            int y=y0; 
//            for(int x=x0;x<=x1;x++){
//            	if(steep){
//            	    if(MAP.at<uint8_t>(x,y)<128) return true;
//            		}
//                else{
//                    if(MAP.at<uint8_t>(y,x)<128) return true;
//                }
//                err-=dErr;
//                if(err<0){
//                    y+=yStep;
//                    err+=dX;
//                }
//            }
//        
//            return false;
//        }
//
//        std::vector<cv::Point> generate_final_course(RRT_Node *cur_node){
//            std::vector<cv::Point> path;
//            path.push_back(end.loc);
//            cur_node = cur_node->parent;
//            while(cur_node->parent !=NULL){
//                path.push_back(cur_node->loc);
//                cur_node = cur_node->parent;
//            }
//            path.push_back(cur_node->loc);
//            return path;
//
//        }
//
//        std::vector<RRT_Node*> planning(bool show_animation=true){
//
//            std::vector<RRT_Node *> path;
//            RRT_Node *rnd_node, *nearest_node, *new_node, *final_node;
//            node_list.push_back(&start);
//            for(int iter=0; iter<max_iter; ++iter){
//                rnd_node = get_random_node();
//                nearest_node = get_nearest_node(rnd_node);
//                new_node = steer(nearest_node, rnd_node); 
//                if(!check_collision(nearest_node, new_node)){
//                    node_list.push_back(new_node);
//                    if(show_animation){
//                        //std::cout << "Iteration: " << iter << "\n";
//                        cv::line(imout, nearest_node->loc, new_node->loc,
//                                cv::Scalar(255, 0, 0), 1, cv::LINE_8);
//                        cv::imshow("RRT in action", imout);
//                        cv::waitKey(24);
//                    }
//                }
//                auto diff = end.loc - new_node->loc;
//                if(std::hypotf(diff.x, diff.y) < this->expand_dist){
//                    final_node = steer(new_node, &end);
//                    if(!check_collision(new_node, final_node)){
//                       auto final_path = generate_final_course(final_node); 
//                       cv::destroyAllWindows();
//                       cv::polylines(imout, final_path, false,
//                                cv::Scalar(0, 0, 0), 2, cv::LINE_8);
//                       cv::imshow("final_path", imout);
//                       cv::waitKey(0);
//                       break;
//                    }
//                }
//            }
//            return path;
//        }
//
//
//};
//
///*** 
// * main method for simple RRT
//int main(){
//    RRT rrt = RRT(cv::Point2i(100, 100), cv::Point2i(800, 800));
//    rrt.set_MAP("../img/map_basic.png");
//    rrt.display_MAP(500);
//    auto path = rrt.planning(true);
//    std::cout << "Execution complete" << std::endl;
//    return 0;
//}
//*/
