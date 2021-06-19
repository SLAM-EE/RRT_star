/* C++ port of RRT from PythonRobotics by author AtsushiSakai
 * Aswin P Ajayan 
 * aswin@ee.iitb.ac.in
 */

#include "../include/RRT.h"
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace Planning;


class Node : public RRT_Node{
    public:
        float tcost;
        Node(cv::Point2i loc):RRT_Node(loc), tcost(0.0){}
        Node(int x, int y):RRT_Node(x, y), tcost(0.0){}
};

class RRTStar : public RRT{
    public:
        int neighbour_dist;
        Node goal_node, start_node;
        //std::vector<Node *> node_list;
        RRTStar(cv::Point2i start, cv::Point2i goal, int expand_dist=30,
                int goal_sample_rate=5, int max_iter=80000,
                float path_resolution=25.0,int neighbour_dist=100)
            :RRT(start, goal, expand_dist,goal_sample_rate, max_iter,
                path_resolution), neighbour_dist(neighbour_dist), 
                goal_node(goal), start_node(start){} 

        std::vector<RRT_Node *> find_near_nodes(RRT_Node *new_node){
            std::vector<RRT_Node *> near_nodes;
            float nnode = node_list.size();
            float r = neighbour_dist ;//* std::sqrt(std::log(nnode+10)/ nnode);

            r = std::min(r, (float)expand_dist);
            r = 40;
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
            if(near_nodes.empty()){std::cout <<"no near nodes--"; return NULL;}
            for(auto node: near_nodes){
                //std::cout << node->loc << ". cost : " << cost <<"\n";
                //auto t_node = steer(node, new_node);
                if(new_node  && (!check_collision(node, new_node))){
                    if(new_node->loc == node->loc || new_node->loc == start.loc) continue;
                    cost = calc_new_cost(node, new_node);
                    if(cost < min_cost){
                        min_cost = cost;
                        parent_node = node;
                    }
                }
            }
            if(min_cost > (float)(2 * max_rand * max_rand)) return NULL;
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
            float improved_cost;
            for(auto& node: near_nodes){
             //   auto edge_node = steer(new_node, node);
                auto edge_node = node;
                if(edge_node == NULL) continue;
                edge_node->cost = calc_new_cost(new_node, node);
                improved_cost = node->cost > edge_node->cost;
                if(improved_cost && !(check_collision(new_node, node))){
                    node = edge_node;
                    propagate_cost_to_leaves(new_node);
                }
                    
            }
        }
        void show_animation(){
            cv::Mat img = imout.clone();
            int i = 0;
            for(auto node: node_list){
                if(node->parent !=NULL){
                    cv::line(img, node->parent->loc, node->loc,
                            cv::Scalar(255, 0, 0), 1, cv::LINE_8);
                }
            }
            //cv::imshow("RRT Star in action", img);
            //cv::waitKey(24);
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
                if(dist <= expand_dist){
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
        std::vector<cv::Point> planning(bool animation=true){
            std::vector<cv::Point> final_path;
            RRT_Node *rnd_node, *nearest_node, *new_node;
            std::vector<RRT_Node *> near_nodes;
            node_list.push_back(&start_node);
            for(int iter=0; iter<max_iter; ++iter){
                rnd_node = get_random_node();
                nearest_node = get_nearest_node(rnd_node);
                new_node = steer(nearest_node, rnd_node); 
                if(!check_collision(nearest_node, new_node)){
                    near_nodes = find_near_nodes(new_node);

                    new_node = choose_parent(near_nodes, new_node);
                    if(new_node){
                        node_list.push_back(new_node);
                        rewire(new_node, near_nodes);
                    }
                }
                if(new_node){
                    auto safe_goal = get_best_goal_node();
                    if(safe_goal){
                        auto final_path = generate_final_course(safe_goal);
                        cv::polylines(imout, final_path, false,
                                 cv::Scalar(0, 0, 0), 2, cv::LINE_8);
                        //cv::imshow("final_path", imout);
                        //cv::waitKey(20);
                        
                    }
                }

                if(animation){
                  show_animation();     
                }
                //    node_list.push_back(new_node);
                //    if(show_animation){
                //        //std::cout << "Iteration: " << iter << "\n";
                //        cv::line(imout, nearest_node->loc, new_node->loc,
                //                cv::Scalar(255, 0, 0), 1, cv::LINE_8);
                //        cv::imshow("RRT in action", imout);
                //        cv::waitKey(24);
                //    }
                //}
                //auto diff = end.loc - new_node->loc;
                //if(std::hypotf(diff.x, diff.y) < this->expand_dist){
                //    final_node = steer(new_node, &end);
                //    if(!check_collision(new_node, final_node)){
                //       final_path = generate_final_course(final_node); 
                //       cv::destroyAllWindows();
                //       cv::polylines(imout, final_path, false,
                //                cv::Scalar(0, 0, 0), 2, cv::LINE_8);
                //       cv::imshow("final_path", imout);
                //       cv::waitKey(0);
                //       break;
                //    }
                if(iter % 500 ==0) std::cout << iter<<"\n";
            }
            auto safe_goal = get_best_goal_node();
                    if(safe_goal){
                        auto final_path = generate_final_course(safe_goal);
                        cv::polylines(imout, final_path, false,
                                 cv::Scalar(0, 0, 0), 2, cv::LINE_8);
                        cv::imshow("final_path", imout);
                        cv::waitKey(0);
                        
                    }
            return final_path;
        }
};



 
/*---- main method for simple RRT ---*/
int main(){
    RRTStar rrt = RRTStar(cv::Point2i(100, 900), cv::Point2i(800, 900));
    rrt.set_MAP("../img/map_basic.png");
    rrt.display_MAP(500);
    auto path = rrt.planning(true);
    std::cout << "Execution complete" << std::endl;
    return 0;
}

