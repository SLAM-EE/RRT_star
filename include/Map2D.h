/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAP2D_H
#define MAP2D_H

#include "FrameDrawer.h"
#include "KeyFrame.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include <condition_variable>

#include <mutex>

namespace ORB_SLAM2
{

class System;
class KeyFrame;


class Map2D
{
public:

    std::condition_variable cv;
    std::mutex mutexStart;
    Map2D(size_t scale, size_t grid_size);
    Map2D();


    int points=0;
   // parameters
    float scale_factor = 10;
    float resize_factor = 1;
    float cloud_max_x = 50;
    float cloud_min_x = -50.0;
    float cloud_max_z = 50;
    float cloud_min_z = -10;
    float free_thresh = 0.55;
    float occupied_thresh = 0.50;
    float thresh_diff = 0.01;
    int visit_thresh = 0;
    float upper_left_x = -1.5;
    float upper_left_y = -2.5;
    const int resolution = 10;
    unsigned int use_local_counters = 0;
    
    float grid_max_x, grid_min_x,grid_max_z, grid_min_z;
    cv::Mat global_occupied_counter, global_visit_counter;
    cv::Mat local_occupied_counter, local_visit_counter;
    cv::Mat local_map_pt_mask;
    cv::Mat grid_map, grid_map_int, grid_map_thresh, grid_map_thresh_resized;
    float norm_factor_x, norm_factor_z;
    int h, w;
    unsigned int n_kf_received;
    bool loop_closure_being_processed = false;

    float kf_pos_x, kf_pos_z;
    int kf_pos_grid_x, kf_pos_grid_z;

 

    // 
    //Thread to create 2D map 
    void Run();
    void setNewKeyFrames(std::vector<ORB_SLAM2::KeyFrame*> keyFrames);
    void getGridMap();


private:

    int frame_count;

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;
    std::vector<ORB_SLAM2::KeyFrame*> mpKeyFrames; 

    // 1/fps in ms
    float mImageWidth, mImageHeight;

    std::mutex mMutexStop;
    
    // some helper functions for creating the map
    void inc_frame();

    void resetGridMap();

    
    float discretizeInX(float inp);
    float discretizeInZ(float inp);
    

};

}


#endif // MAP2D_H
	

