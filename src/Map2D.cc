/**
*/

#include "Map2D.h"
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/saturate.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pangolin/pangolin.h>
#include "matplotlibcpp.h"

#include <mutex>
namespace plt = matplotlibcpp;
namespace ORB_SLAM2
{

    //Constructor
    Map2D::Map2D(size_t scale, size_t grid_size):
        frame_count(0){
    
    }

    Map2D::Map2D():
        frame_count(0){

    
    }
    void Map2D::Run()
    {
        //Main method

        cv::Mat im = cv::imread("/home/dataset/testImage.jpeg", cv::IMREAD_GRAYSCALE);
      	grid_max_x = cloud_max_x*scale_factor;
	    grid_min_x = cloud_min_x*scale_factor;
	    grid_max_z = cloud_max_z*scale_factor;
	    grid_min_z = cloud_min_z*scale_factor;

       	double grid_res_x = grid_max_x - grid_min_x, grid_res_z = grid_max_z - grid_min_z;
       	norm_factor_x = float(grid_res_x - 1) / float(grid_max_x - grid_min_x);
        norm_factor_z = float(grid_res_z - 1) / float(grid_max_z - grid_min_z);
      	h = grid_res_z;
        w = grid_res_x;

        global_occupied_counter.create(h, w, CV_32SC1);
        global_visit_counter.create(h, w, CV_32SC1);
        grid_map.create(h, w, CV_32FC1);
	    grid_map_thresh.create(h, w, CV_8UC1);
	    grid_map_thresh_resized.create(h*resize_factor, w*resize_factor, CV_8UC1);
        grid_map_int = cv::Mat(h, w, CV_8SC1);
        grid_map_display = cv::Mat(h, w, CV_8UC1);



        //cv::imshow("ORB-SLAM2: Current Map", im);
        //cv::waitKey(5);
        //cv::destroyAllWindows();

        while(1)
        {

             
            std::unique_lock<std::mutex> lk(mutexStart);
            cv.wait(lk);
            std::cout << "Mapper Triggered" << endl;
            resetGridMap();
            getGridMap();
            std::cout << "Map Processed" << endl;
              
            //tDisp = new std::thread(std::this->display_map);
        }
    }
    void Map2D::inc_frame(){
        frame_count++;
    }
    void Map2D::setNewKeyFrames(std::vector<ORB_SLAM2::KeyFrame*> keyFrames){
        mpKeyFrames = keyFrames;
    }

    void Map2D::resetGridMap(){
        points = 0;
        float camX, camZ, ptX, ptZ;
        int camIndX, camIndZ, ptIndX, ptIndZ;
        global_visit_counter.setTo(0);
	    global_occupied_counter.setTo(0);
        for(auto keyFrame : mpKeyFrames){
            if(keyFrame->isBad()) continue;

            cv::Mat twc = keyFrame->GetCameraCenter();
            camX = twc.at<float>(0); camZ = twc.at<float>(2);
            camIndX = discretizeInX(camX); camIndZ = discretizeInZ(camZ);
            if(camIndX < 0 || camIndZ < 0) return;

            std::set<ORB_SLAM2::MapPoint*> mapPts = keyFrame->GetMapPoints();

            points++;

            for(auto mapPt : mapPts){
                if(!mapPt || mapPt->isBad()) continue;

                cv::Mat ptPose = mapPt->GetWorldPos();
                if(ptPose.empty()) continue;

                ptX = ptPose.at<float>(0); ptZ = ptPose.at<float>(2);
                ptIndX = discretizeInX(ptX); ptIndZ = discretizeInZ(ptZ);
                if(ptIndX < 0 || ptIndZ < 0) continue;


                ++global_occupied_counter.at<int>(ptIndZ, ptIndX);

               // line drawing alg. @abhineet github
               
                int x0 = camIndX, y0 = camIndZ;
                int x1 = ptIndX, y1 = ptIndZ;

                bool steep = (abs(y1 - y0) > abs(x1 - x0));
                if(steep){
                    std::swap(x0, y0); std::swap(x1, y1);
                }
                if(x0 > x1){
                    std::swap(x0, x1);
                    std::swap(y0, y1);
                }
                int dx = x1 - x0;
                int dy = abs(y1 - y0);
               	double error = 0;
            	double deltaerr = ((double)dy) / ((double)dx);
            	int y = y0;
            	int ystep = (y0 < y1) ? 1 : -1;
            	for (int x = x0; x <= x1; ++x){
            		if (steep) {
            			++global_visit_counter.at<int>(x, y);
            		}
            		else {
            			++global_visit_counter.at<int>(y, x);
            		}
            		error = error + deltaerr;
            		if (error >= 0.5){
            			y = y + ystep;
            			error = error - 1.0;
            		}
            	}



           }//loop on mapPts;


        }//loop on keyframes;
        cout << "No of keyframes processed :" << points << endl;
        cout << "height :" << h << " width :" << w << endl;

    }

    void Map2D::getGridMap() {
    //directly taken from @abhineet github/
	for (int row = 0; row < h; ++row){
		for (int col = 0; col < w; ++col){
			int visits = global_visit_counter.at<int>(row, col);
			int occupieds = global_occupied_counter.at<int>(row, col);

			if (visits <= visit_thresh){
				grid_map.at<float>(row, col) = 0.5;
			}
			else {
				grid_map.at<float>(row, col) = 1.0 - float(occupieds / visits);
			}
			if (grid_map.at<float>(row, col) >= free_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 255;
			}
			else if (grid_map.at<float>(row, col) < free_thresh && grid_map.at<float>(row, col) >= occupied_thresh) {
				grid_map_thresh.at<uchar>(row, col) = 128;
			}
			else {
				grid_map_thresh.at<uchar>(row, col) = 0;
			}
			grid_map_int.at<char>(row, col) = (1 - grid_map.at<float>(row, col)) * 100;
		}
	}
	cv::resize(grid_map_thresh, grid_map_thresh_resized, grid_map_thresh_resized.size());
    inc_frame();
    cv::Mat dst;
    grid_map.convertTo(dst, CV_8UC1, 255.0, 0); 
    cv::imshow("map 2D", dst);





}

    void Map2D::display_map(){
            
    //cv::imshow("grid_map_thresh_resized", grid_map_thresh_resized);
    	//int key = cv::waitKey(1) % 256;
    //std::cout << "image height:" << grid_map_thresh_resized.rows << std::endl;
    //std::cout << "image width:" << grid_map_thresh_resized.cols << std::endl;
   	//printf("saving maps with id: %u\n", frame_count);
    //cv::Mat mask = (grid_map < 0.48);
    cv::Mat mask = (grid_map == 0.5);
    //grid_map.setTo(0.48, mask);
    //cv::Mat mask2 = grid_map > 0.6;
    //grid_map.setTo(0.6, mask2);
    float Amin = *min_element(grid_map.begin<float>(), grid_map.end<float>());
    float Amax = *max_element(grid_map.begin<float>(), grid_map.end<float>());
    cv::Mat display;
    cv::Mat A_scaled = cv::Mat(h, w, CV_32FC1);
    A_scaled = (grid_map - Amin) /(Amax - Amin);
    //A_scaled = (grid_map - 0.48) /(0.6 - 0.48);
    
    int nHistSize = 512;
    float fRange[] = { 0.0f, 1.0f };
    const float* fHistRange = { fRange };
    //
    //cv::Mat matHist;
    ////cv::calcHist(&A_scaled, 1, 0, cv::Mat(), matHist, 1, &nHistSize, &fHistRange);
    //cv::calcHist(&grid_map, 1, 0, cv::Mat(), matHist, 1, &nHistSize, &fHistRange);
    //std::vector<float> vecHist(matHist.begin<float>(), matHist.end<float>());

    //plt::plot(vecHist);
    //plt::show();
    cv::Mat dst;
    grid_map.convertTo(dst, CV_8UC1, 255.0, 0); 

    vector<int> hist(256,0);
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            if (mask.at<int>(r, c)) {
                hist[dst.at<uchar>(r, c)]++;
            }
        }
    }


    int cnz = countNonZero(mask);
    float scale = 255.f / float(cnz);
    vector<uchar> lut(256);
    int sum = 0;
    for (int i = 0; i < hist.size(); ++i) {
        sum += hist[i];
        lut[i] = cv::saturate_cast<uchar>(sum * scale);
    }

    //plt::plot(lut);
    //plt::show();


    unsigned char temp;
    for (int r = 0; r < dst.rows; ++r) {
        for (int c = 0; c < dst.cols; ++c) {
            if (mask.at<int>(r, c)) {
                //grid_map_display.at<uint>(r, c) = cv::saturate_cast<uchar>(lut[dst.at<uchar>(r,c)]);
                dst.at<uint>(r, c) = cv::saturate_cast<uchar>(lut[dst.at<uchar>(r,c)]);
                //cout << dst2.at<int>(r, c);
                //cout << dst2.at<int>(r, c) ;
                //cout << lut[dst.at<uchar>(r,c)];
            }
        }
    }
    //cv::equalizeHist(dst, dst);
    cv::applyColorMap(dst, dst, cv::COLORMAP_JET);
    cv::imwrite("results/grid_map_" + to_string(frame_count) + ".jpg", dst);

    }



    inline float Map2D::discretizeInX(float inp){
        float pos = inp * scale_factor;
        int grid_pos = int(floor((pos - grid_min_x) * norm_factor_x));
        if(grid_pos < 0 || grid_pos >= w) return -1;
        return grid_pos;
    }


    inline float Map2D::discretizeInZ(float inp){
        float pos = inp * scale_factor;
        int grid_pos = int(floor((pos - grid_min_z) * norm_factor_z));
        if(grid_pos < 0 || grid_pos >= h) return -1;
        return grid_pos;

    }



}
