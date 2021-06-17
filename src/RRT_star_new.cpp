#include <cstdint>
#include <iostream>
#include <limits>
#include <stdlib.h>     /* srand, rand */
#include <time.h> 
#include <math.h>
#include <bits/stdc++.h>
#include <limits>
#include <opencv2/opencv.hpp>
#define numNodes 10000
#define xmax 1000
#define ymax 1000

using namespace std;

float EPS=40;
int lamb=3.5;
float r=60;//radius
int N_=0;
class Node
{
  
  public:
  Node()
  { 
      x_cord=0;
      y_cord=0;
      parent=0;
      cost=0;
      
  }
  int x_cord,y_cord,parent;
  float cost;
  
  void generate_node(int x,int y,int p,float c)
  {
      x_cord=x;
      y_cord=y;
      parent=p;
      cost=c;
      
  }
  void get_node(float A[]/*this array will contain [x,y,parent,cost]*/)
 {
     A[0]=(int)x_cord;
     A[1]=(int)y_cord;
     A[2]=(int)parent;
     A[3]=cost;
 }
  
  
  
};

/*void create_obstacle(int A[][xmax])
{
    for(int i=0;i<ymax;i++)
    {
        for(int j=0;j<xmax;j++)
        {
            if(((j>=0.24*xmax) &&(j<=0.26*xmax)&&(i>=0 )&&(i<=0.6*ymax))||((j>=0.74*xmax) &&(j<=0.76*xmax)&&(i>=0.4*ymax )&&(i<ymax)))
        
            {
                A[i][j]=1;
            }
        }
    }
}*/
/*void display_obstacles(int A[][xmax])
{
    for(int i=0;i<ymax;i++)
    {
        for(int j=0;j<xmax;j++)
        {
            cout<<A[i][j];
        }
        cout<<endl;
    }
}*/
float dist(int x1,int y1,int x2,int y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

void steer(int xr,int yr,int xn,int yn,float val,float EPS,int * new_x,int * new_y)
{
    if(val>=EPS)
    {	int f1,f2;
   
  
    f1=int(50*(xr-xn)/val);
    f2=int(50*(yr-yn)/val);
        *new_x=f1+xn;
                *new_y=yn+f2;
    }
    else
    {
                *new_x=xr;
                        *new_y=yr;
    }
}
void swap(int * a,int * b)
{
int temp=*a;
*a=*b;
*b=temp;
}
int no_collision(int x0,int y0,int x1,int y1,cv::Mat MAP)
{

int steep=(abs(y1-y0)>abs(x1-x0))?1:0;
if (steep)
{
swap(&x0,&y0);
swap(&x1,&y1);
}
if(x0>x1)
{
swap(&x0,&x1);
swap(&y0,&y1);
}
int dErr=abs(y1-y0);
int yStep=y0>y1?-1:1;
int dX=x1-x0;
int err=dX>>1;
int y=y0;
for(int x=x0;x<=x1;x++)
{
	if(steep)
	
	{
//	cout<<MAP.at<int>(x,y)<<endl;
	if(MAP.at<uint8_t>(x,y)<128)//i made 
	
	
		return 0;}
else
	{

//        cout<<MAP.at<int>(x,y)<<endl;
        if(MAP.at<uint8_t>(y,x)<128)//i made
        
		return 0;}
err-=dErr;
if(err<0){
y+=yStep;
err+=dX;
}
}
return 1;
    
    
}

void bline(int x0,int y0,int x1,int y1,cv::Mat MAP)
{
int steep=(abs(y1-y0)>abs(x1-x0))?1:0;
if (steep)
{
swap(&x0,&y0);
swap(&x1,&y1);
}
if(x0>x1)
{
swap(&x0,&x1);
swap(&y0,&y1);
}
int dErr=abs(y1-y0);
int yStep=y0>y1?-1:1;
int dX=x1-x0;
int err=dX>>1;
int y=y0;
for(int x=x0;x<=x1;x++){
if(steep)
	MAP.at<int>(x,y)==0;//x&y swap??
else
	MAP.at<int>(y,x)==0;
err-=dErr;
if(err<0){
y+=yStep;
err+=dX;
}
}

}
/*void line(int x1,int y1,int x2,int y2,int obs[][xmax])
{
   int max_x,max_y,min_x,min_y;
    max_x=(x1>x2)?x1:x2;
    max_y=(y1>y2)?y1:y2;
    min_x=(x1>x2)?x2:x1;
    min_y=(y1>y2)?y2:y1;
    float slope=(y2-y1)/((x2-x1)+0.0000001);
    for(int i=min_y;i<=max_y;i++)
    {
        for(int j=min_x;j<=max_x;j++)
        {
            if (((i-slope*(j-x2)-y2)<=lamb)&&((i-slope*(j-x2)-y2)>=(-1*lamb)))
            {
                obs[i][j]=2;
            }
        }
    } 
}*/
int main()
{
    cv::Mat img = cv::imread("../img/map_basic.png", cv::IMREAD_GRAYSCALE);
    cv::Mat imgout = cv::imread("../img/map_basic.png", cv::IMREAD_COLOR);
    //cv::imshow("Display window", img);
    // int k = cv::waitKey(0); // Wait for a keystroke in the window
    //if(k == 'q')
    //{
    //    cv::destroyAllWindows();
    //}
    cv::Mat MAP;
    MAP.create(1000,1000, CV_8UC1);
    cv::threshold(img, MAP, 0, 255, cv::THRESH_OTSU);
    //for(int i = 0; i < 1000; i++){
    //    for(int j = 0; j < 1000; j++){
    //       // cout << imgout.at<int>(i,j);
    //    }cout << endl;
    //}
    //cv::imshow("Display window2", imgout);
    // k = cv::waitKey(0); // Wait for a keystroke in the window
    //if(k == 'q')
    //{
    //    cv::destroyAllWindows();
    //}
    srand (time(NULL));
    Node n[numNodes];
    n[0].generate_node(200,200,INT_MAX,0);//x,y,parent,cost
    N_++;
    int goal_x=800;
    int goal_y=600;
    //int obstacles[ymax][xmax]={0};
   // create_obstacle(obstacles);
    //display_obstacles(obstacles);//till here working
    //n[0].generate_node(0,0,1,1);
    for(long i=0;i<numNodes;i++)
    {	//cout<<"nodes:"<<N_<<endl;
        int x_rand=rand()%xmax;//0 to xmax-1
        int y_rand=rand()%ymax;//0 to ymax-1
        
        int idx; 
       float min_dist=std::numeric_limits<float>::max();
        int global_count=0;//for ndist
        //skipped that  for
        
        float ndist[numNodes]={0};
        
        for(int j=0;j<N_;j++)
        {
            float tmp=dist(n[j].x_cord,n[j].y_cord,x_rand,y_rand);
            ndist[global_count]=tmp;
            global_count++;//keep track of the array's useful size
            if(tmp < min_dist){
                min_dist = tmp;
                idx = j;
            }
        }
     
        //cout<<" value"<<val<<" idx"<<idx<<endl;
        
        //n[idx]----> q_near
        //qnew_x & qnew_y----->q_new.cord
        int qnew_x,qnew_y,qnew_parent;
        float qnew_cost;
        steer(x_rand,y_rand,n[idx].x_cord,n[idx].y_cord,min_dist,EPS,&qnew_x,&qnew_y);
        if (no_collision(qnew_x,qnew_y,n[idx].x_cord,n[idx].y_cord,MAP))//To defined
        {
        //line drawn between qnear and qnew_y-dotted
        qnew_cost=dist(qnew_x,qnew_y,n[idx].x_cord,n[idx].y_cord)+n[idx].cost;
      float q_nearest [numNodes][3]={0};//[x,y,cost]
        
        int neighbour_count=0;
        for(int j=0;j<N_;j++)
        {
            if(no_collision(n[j].x_cord,n[j].y_cord,qnew_x,qnew_y,MAP) && dist(n[j].x_cord,n[j].y_cord,qnew_x,qnew_y)<=r)
            {
                q_nearest[neighbour_count][0]=n[j].x_cord;
                 q_nearest[neighbour_count][1]=n[j].y_cord;
                  q_nearest[neighbour_count][2]=n[j].cost;
                  neighbour_count+=1;
                  
                
            }   
        }
        int qminx=n[idx].x_cord;
        int qminy=n[idx].y_cord;
        
        float cmin=qnew_cost;
        
        for(int k=0;k<neighbour_count;k++)
        {
            if(no_collision(q_nearest[k][0],q_nearest[k][1],qnew_x,qnew_y,MAP)&&(q_nearest[k][2]+dist(q_nearest[k][0],q_nearest[k][1],qnew_x,qnew_y)<cmin))
            {
                qminx=q_nearest[k][0];
                qminy=q_nearest[k][1];
                cmin=q_nearest[k][2]+dist(q_nearest[k][0],q_nearest[k][1],qnew_x,qnew_y);
                //line from qmin to qnew of green
            }
        }
        
        for(int j=0;j<N_;j++)
        {
            if((n[j].x_cord==qminx)&&(n[j].y_cord==qminy))
                
                {
                    qnew_parent=j;
                    
                    break;
                }
                
        }
        n[i].generate_node(qnew_x,qnew_y,qnew_parent,cmin);//cmin??
        N_++;
        if (dist(qnew_x,qnew_y,goal_x,goal_y)<EPS)
        {
            break;
        }
        }
        
    }
    float D[numNodes]={0.0};
    float tmp;
    int count=0,val2=9999999;
    int idx2;
    for(int j=0;j<N_;j++)
    {	
        tmp=dist(n[j].x_cord,n[j].y_cord,goal_x,goal_y);
        //cout<<tmp<<" "<<j<<endl;
        D[count]=tmp;
        count++;
        
    }
    for (int l=0;l<count;l++)//now val and idx contain min dist and the corresponding index
        {
            if(D[l]<val2)
            {
                val2=D[l];
                idx2=l;
            }
        }
    int goal_parent=idx2;
   
    int q_end=count;
    n[count].generate_node(goal_x,goal_y,goal_parent,n[goal_parent].cost+dist(goal_x,goal_y,n[goal_parent].x_cord,n[goal_parent].y_cord));
   N_++;
    count++;
    while(1)
    {
        int start=q_end;
     	cout<<"\n";
        //bline(n[q_end].x_cord,n[q_end].y_cord,n[n[q_end].parent].x_cord,n[n[q_end].parent].y_cord,obstacles);
        cv::line(imgout, cv::Point(n[q_end].x_cord,n[q_end].y_cord),cv::Point(n[n[q_end].parent].x_cord,n[n[q_end].parent].y_cord),cv::Scalar(255,0,0),3,cv::LINE_8);
        //draw line btw 2 Nodes=q_endparent && qend
       
        q_end=n[q_end].parent;
    
        if (q_end==0)
        break;
    }
    //display_obstacles(obstacles);
    cv::imshow("Display path", imgout);
     int m = cv::waitKey(0); // Wait for a keystroke in the window
    if(m == 'q')
    {
        cv::destroyAllWindows();
    }
}

