/* C++ port of RRT from PythonRobotics by author AtsushiSakai
 * Aswin P Ajayan 
 * aswin@ee.iitb.ac.in
 */
#ifndef RRT_STAR_H
#define RRT_STAR_H
#include "RRT.h"
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../spline/include/Bezier.h"
#include "../spline/include/BSpline.h"

#define USE_LEBESGUE_MEASURE true
#define SHOW_ANIMATION true
namespace Planning{

    class RRT_Node;
    class RRT;

    class Node : public RRT_Node{
        public:
            float tcost;
            Node(cv::Point2i loc):RRT_Node(loc), tcost(0.0){}
            Node(int x, int y):RRT_Node(x, y), tcost(0.0){}
    };
    
    class RRTStar : public RRT{
        private:
            #if USE_LEBESGUE_MEASURE
                cv::Mat coverage;
            #endif
            
        public:
            int neighbour_dist;
            Node goal_node, start_node;
            double EPS;
            //std::vector<Node *> node_list;
            RRTStar(cv::Point2i start, cv::Point2i goal, int expand_dist=40,
    
                    int goal_sample_rate=5, int max_iter=50000,
                    float path_resolution=25.0,int neighbour_dist=100, double EPS=30.0)
                :RRT(start, goal, expand_dist,goal_sample_rate, max_iter,
                    path_resolution), neighbour_dist(neighbour_dist), 
                    goal_node(goal), start_node(start), EPS(EPS){
                    } 
            #if USE_LEBESGUE_MEASURE
            void create_coverage_map(){
                coverage = MAP.clone();    
            }
            #endif
    
            std::vector<RRT_Node *> find_near_nodes(RRT_Node *new_node){
                std::vector<RRT_Node *> near_nodes;
                float nnode = node_list.size();
                float r;
    
                //r1 = neighbour_dist * std::pow(2.8, -1 * nnode/(max_iter * 100));
                
    #if USE_LEBESGUE_MEASURE
                const float coeff = 2.449489742783178; // 2 * (1 + 1/d)^(1/d) ; d=2;
                float gamma_rrt, gamma_rrt_star;
                float mu_X_free = cv::countNonZero(coverage);
                gamma_rrt = std::sqrt(std::log(nnode)/nnode); 
                gamma_rrt_star = coeff * gamma_rrt * mu_X_free;
                r = std::min(gamma_rrt_star, (float)(expand_dist));
    #else
                float r1 = neighbour_dist * (1 - (nnode)/ max_iter);
                r = std::min(r1, (float)(expand_dist));
    #endif
    
                if(nnode < 3) r = neighbour_dist;
                //if(nnode > 200) r = std::min(r1, (float)(expand_dist));
                for(auto node: node_list){
                    auto diff = new_node->loc - node->loc;
                    if(diff.ddot(diff) < r*r){
                        near_nodes.push_back(node);
                    }
                }
    
                return near_nodes;
            }
    
            float calc_new_cost(RRT_Node *from_node, RRT_Node *to_node){
                float cost;
                auto diff = to_node->loc - from_node->loc;
                cost = std::hypotf(diff.y, diff.y);
                cost += from_node->cost;
                return cost;
            }
    
            RRT_Node * choose_parent(std::vector<RRT_Node *> near_nodes,
                                    RRT_Node * new_node){
                RRT_Node *parent_node;
                float cost, min_cost=std::numeric_limits<float>::max();
                if(near_nodes.empty()){
                    //std::cout <<"no near nodes--"; 
                    return NULL;}
                for(auto node: near_nodes){
                    //std::cout << node->loc << ". cost : " << cost <<"\n";
                    auto t_node = new RRT_Node(new_node->loc);
                    t_node->parent = node;
                    if(t_node  && (!check_collision(node, t_node))){
                        if(new_node->loc == node->loc) continue;
                        cost = calc_new_cost(node, t_node);
                        if(cost < min_cost){
                            min_cost = cost;
                            parent_node = node;
                        }
                    }
                }
                if(min_cost > (float)(20 * max_rand_x * max_rand_y)) return NULL;
                //new_node = steer(parent_node, new_node);
                new_node->parent = parent_node;
                new_node->cost = min_cost;
                return new_node;
            }
    
            void propagate_cost_to_leaves(RRT_Node *parent_node){
                for(auto& node: node_list){
                    if(node->parent == parent_node){
                        node->cost = calc_new_cost(parent_node, node);
                        propagate_cost_to_leaves(node);
                    }
                }
                
            }
    
            void rewire(RRT_Node *new_node, std::vector<RRT_Node*> near_nodes){
                //check wether cost from current node to near nodes are less;
                bool improved_cost;
                for(auto& node: near_nodes){
                 //   auto edge_node = steer(new_node, node);
                    
                    if(node == NULL) continue;
                    RRT_Node *edge_node = new RRT_Node(node->loc);
                    edge_node->parent = new_node;
                    edge_node->cost = calc_new_cost(new_node, edge_node);
                    improved_cost = node->cost > edge_node->cost;
                    if(improved_cost && !(check_collision(new_node, edge_node))){
                        node = edge_node;
                        propagate_cost_to_leaves(new_node);
                    }
                        
                }
            }
            void show_animation(){
                cv::Mat img = imout.clone();
                if(iter % 100) return;
                for(auto node: node_list){
                    if(node->parent !=NULL){
                        cv::line(img, node->parent->loc, node->loc,
                                line_col, 1, cv::LINE_8);
                    }
                }
                cv::imshow("RRT Star in action", img);
                cv::waitKey(24);
            }
    
            float get_dist_to_goal(RRT_Node * from_node){
                auto diff = goal_node.loc - from_node->loc;
                return std::hypot(diff.x, diff.y);
            }
            RRT_Node * get_best_goal_node(){
                RRT_Node *safe_goal;
                float min_cost=std::numeric_limits<float>::max();
                for(auto node: node_list){
                    auto dist = get_dist_to_goal(node);
                    if(dist <= EPS){
                        if(!check_collision(node, &goal_node)){
                            if(node->cost < min_cost){
                                min_cost = node->cost;
                                safe_goal = node;
                            }
                        }
                    }
                }
                if(safe_goal->cost == min_cost) return safe_goal;
                return NULL;
            }
    
            cv::Point getPoint(Vector node){
            return cv::Point(node.x, node.y);}
    
    #if USE_LEBESGUE_MEASURE
            void update_coverage(RRT_Node *center){
                cv::circle(coverage, center->loc,
                        (int)((center->cost - center->parent->cost)),
                        0, cv::FILLED);
        #if SHOW_ANIMATION
                    //if(iter % 100==0){
                    //    cv::imshow("coverage_map", coverage);
                    //    cv::waitKey(50);
                    //}
        #endif
            }
    #endif
    
            std::vector<cv::Point>  get_bezier_path(std::vector<cv::Point> path){
                std::vector<cv::Point> smoothened_path;
                Curve* curve = new BSpline();
                curve->set_steps(100);
                smoothened_path.push_back(end.loc);
                for(auto point: path){
                    curve->add_way_point(Vector(point.x, point.y, 0));
                }
                for(int i = 0; i < curve->node_count() ; ++i)
                    smoothened_path.push_back(getPoint(curve->node(i)));
                delete curve;
                smoothened_path.push_back(start.loc);
                return smoothened_path;
    
            }
            void display_simulation_params(){
                std::cout << "#............. Simulation parameters ........#\n";
                std::cout << "max iterations : " << max_iter <<"\n";
                std::cout << "expand distance : " << expand_dist <<"\n";
                std::cout << "neighnour hood distance : " << neighbour_dist <<"\n";
                std::cout << "MAP : " << MAP.type() << " size: " << MAP.size <<"\n";
                std::cout << "sampling bounds : " << max_rand_x << " x " << max_rand_y <<"\n";
    #if USE_LEBESGUE_MEASURE
                std::cout << "Using Lebesgue Measure for rolling of neighbourhod\n";
    #endif
    #if SHOW_ANIMATION
                std::cout<< "You can turn off animations by use SHOW_ANIMATION FLAG\n";
    #else
                std::cout<< "To view all nodes generated use SHOW_ANIMATION FLAG\n";
    #endif
            }
    
            std::vector<cv::Point> planning(bool animation=true){
                display_simulation_params();
                std::vector<cv::Point> final_path;
                RRT_Node *rnd_node, *nearest_node, *new_node;
                std::vector<RRT_Node *> near_nodes;
                node_list.push_back(&start_node);
                for(iter=0; iter<max_iter; ++iter){
                    rnd_node = get_random_node();
                    if(rnd_node == NULL) continue;
                    nearest_node = get_nearest_node(rnd_node);
                    new_node = steer(nearest_node, rnd_node); 
                    if(new_node == NULL) continue;
                    if(!check_collision(nearest_node, new_node)){
                        near_nodes = find_near_nodes(new_node);
    
                        new_node = choose_parent(near_nodes, new_node);
                        if(new_node){
                            node_list.push_back(new_node);
                            rewire(new_node, near_nodes);
    #if USE_LEBESGUE_MEASURE    
                            update_coverage(new_node);
    #endif
                        }
                    }
                    if(new_node){
                        static float min_cost=std::numeric_limits<float>::max();
                        auto safe_goal = get_best_goal_node();
                        if(safe_goal && safe_goal->cost + get_dist_to_goal(safe_goal) < min_cost){
                            //if(check_collision(safe_goal, &goal_node)) continue;
                            min_cost = safe_goal->cost + get_dist_to_goal(safe_goal);
    
                            auto final_path = generate_final_course(safe_goal);
                            cv::polylines(imout, final_path, false,
                                     colors[(++col_i)%5], 1, cv::LINE_8);
                            auto smoothened_path = get_bezier_path(final_path);
                            cv::polylines(imout, smoothened_path, false, colors[col_i % 5], 2);
                            std::cout << "New path found at iter: " << iter << " [Node count "
                                << node_list.size() << " cost : " << min_cost << " ] \n";
    
                            cv::imshow("final_path", imout);
                            cv::waitKey(20);
                            
                        }
                    }
                    if(animation){
                      show_animation();     
                        //if(iter % 500 ==0) std::cout << "Iteration : " << iter << 
                         //               " Node count : " << node_list.size() <<"\n";
                    }
                }
                auto safe_goal = get_best_goal_node();
                if(safe_goal){
                    auto final_path = generate_final_course(safe_goal);
                    cv::polylines(imout, final_path, false,
                             line_col, 1, cv::LINE_8);
                    auto smoothened_path = get_bezier_path(final_path);
                    cv::polylines(imout, smoothened_path, false, 0, 2);
                    cv::imshow("final_path", imout);
                    cv::imwrite("img/out.png", imout);
                    cv::waitKey(0);
                    
                }else{
                    std::cout << "No path found between " << start_node.loc  
                        << " and " << goal_node.loc << "\n";
                }
                return final_path;
            }
    };
} //namespace Planning

#endif /* ifndef RRT_STAR_H */
