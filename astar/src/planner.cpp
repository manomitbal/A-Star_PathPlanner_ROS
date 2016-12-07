#include <iostream>
#include <sys/time.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include "nav_msgs/Odometry.h"
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <stack>
#include <queue>

using namespace std;
using namespace ros;

#define M_PI  3.14159265358979323846 /* pi */

geometry_msgs::Pose current_Pose;
geometry_msgs:: Pose2D goal_Pose;
sensor_msgs::LaserScan baseScan;
double roll, pitch, bearing;

bool end = true;

const int n=18; 
const int m=20; 
int mapxy[n][m];
int tmp_map[20][18] = { 
                    {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
                    {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
                    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},                    
                    {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
                    {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
                    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
                    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
                    {0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
                    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1},
                    {0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0},
                    {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                    {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0},
                    {0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0},
                    {0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0},
                    {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1},
                                                            };
int usedcells[n][m]; 
int freecells[n][m]; 
int directionals[n][m]; // 
const int dir = 4; 

static int deltaX[dir]={1, 0, -1, 0};
static int deltaY[dir]={0, 1, 0, -1};


class cell
{
    
    int position_X;
    int position_Y;
    int node;
    int priority;  

    public:
        cell(int xp, int yp, int d, int p) 
        {
            position_X = xp; 
            position_Y = yp; 
            node = d; 
            priority = p;
        }
    
        int getposition_X() const 
        {
            return position_X;
        }
        
        int getposition_Y() const 
        {
            return position_Y;
        }
        
        int getnode() const 
        {
            return node;
        }
        
        int getPriority() const 
        {
            return priority;
        }

        void updatePriority(const int & xDest, const int & yDest)
        {
             priority = node + getHeuristic(xDest, yDest) * 10; //A*
        }
        
        
        const int & getHeuristic(const int & xDest, const int & yDest) const
        {
            static int xDistance, yDistance, dist;
            xDistance = xDest - position_X;
            yDistance = yDest - position_Y;         
            dist = (sqrt(xDistance * xDistance + yDistance * yDistance));
            return(dist);
        }
};


bool operator<(const cell & a, const cell & b)
{
  return a.getPriority() > b.getPriority();
}


string planPath( const int & xStart, const int & yStart, 
                 const int & xFinish, const int & yFinish )
{
    priority_queue<cell> container[2]; 
    int incN; 
    cell* rowcell;
    cell* colcell;
    int i, j, x, y, xdeltaX, ydeltaY;
    char c;
    incN=0;

    
    for(y = 0;y < m;y++)
    {
        for(x = 0;x < n;x++)
        {
            usedcells[x][y]=0;
            freecells[x][y]=0;
        }
    }

    
    rowcell = new cell(xStart, yStart, 0, 0);
    rowcell->updatePriority(xFinish, yFinish);
    container[incN].push(*rowcell);
    freecells[xStart][yStart] = rowcell->getPriority(); 

   
    while(!container[incN].empty())
    {
        
        rowcell=new cell( container[incN].top().getposition_X(), container[incN].top().getposition_Y(), 
                     container[incN].top().getnode(), container[incN].top().getPriority());

        x = rowcell->getposition_X(); 
        y = rowcell->getposition_Y();

        container[incN].pop(); 
        freecells[x][y]=0;
        usedcells[x][y]=1;

       
        if(x == xFinish && y == yFinish) 
        {
            
            string path = "";
            while(!(x == xStart && y == yStart))
            {
                j = directionals[x][y];
                c = '0' + (j + dir / 2) % dir;
                path = c + path;
                x+=deltaX[j];
                y+=deltaY[j];
            }

            
            delete rowcell;
            
            while(!container[incN].empty()) container[incN].pop();           
            return path;
        }

        
        for(i = 0;i < dir;i++)
        {
            xdeltaX = x + deltaX[i]; 
            ydeltaY = y + deltaY[i];

            if(!(xdeltaX < 0 || xdeltaX > n-1 || ydeltaY < 0 || ydeltaY > m-1 || mapxy[xdeltaX][ydeltaY] == 1 || usedcells[xdeltaX][ydeltaY] == 1))
            {
                
                colcell=new cell( xdeltaX, ydeltaY, rowcell->getnode(), 
                             rowcell->getPriority());
                
                colcell->updatePriority(xFinish, yFinish);

                
                if(freecells[xdeltaX][ydeltaY] == 0)
                {
                    freecells[xdeltaX][ydeltaY] = colcell->getPriority();
                    container[incN].push(*colcell);
                    
                    directionals[xdeltaX][ydeltaY] = (i + dir / 2) % dir;
                }
                else if(freecells[xdeltaX][ydeltaY]>colcell->getPriority())
                {
                    
                    freecells[xdeltaX][ydeltaY]=colcell->getPriority();
                    
                    directionals[xdeltaX][ydeltaY] = (i + dir / 2) % dir;

                    while(!(container[incN].top().getposition_X() == xdeltaX && 
                           container[incN].top().getposition_Y() == ydeltaY))
                    {                
                        container[1 - incN].push(container[incN].top());
                        container[incN].pop();       
                    }
                    container[incN].pop(); 
                    
                    
                    if(container[incN].size() > container[1 - incN].size()) 
                        incN = 1 - incN;
                    
                    while(!container[incN].empty())
                    {                
                        container[1 - incN].push(container[incN].top());
                        container[incN].pop();       
                    }
                    
                    incN = 1 - incN;
                    container[incN].push(*colcell); 
                }
                else delete colcell; 
            }
        }
        delete rowcell; 
    }
    return ""; 
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg_in)
{
    baseScan = *msg_in;
}

void current_PoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	current_Pose.position.x = msg->pose.pose.position.x;
	current_Pose.position.y = msg->pose.pose.position.y;
	tf::Quaternion quat(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
	tf::Matrix3x3 m(quat);
	m.getRPY(roll,pitch,bearing);
}


void goalPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)            
{
    end = false;                                                       
    goal_Pose.x = msg->x;                                                 
    goal_Pose.y = msg->y;
    return;
}


float angleError(geometry_msgs::Pose current_Pose, geometry_msgs::Pose2D goal_Pose)
{
 
    float estimatedX = goal_Pose.x - current_Pose.position.x;                                    
    float estimatedY = goal_Pose.y - current_Pose.position.y;                                   
    float dest = atan2f(estimatedY, estimatedX);                                        
    float estimatedTheta = dest - bearing;
    return estimatedTheta;
}


float linearError(geometry_msgs::Pose current_Pose, geometry_msgs::Pose2D goal_Pose)
{
   
    float estimatedX = goal_Pose.x - current_Pose.position.x;                                   
    float estimatedY = goal_Pose.y - current_Pose.position.y;                                   
    float estimatedTheta = angleError(current_Pose, goal_Pose);                           
    float correctedTheta = hypotf(estimatedX, estimatedY)*cos(estimatedTheta); 
    return correctedTheta;
}

int main(int argc, char* argv[])
{
	init(argc, argv, "planner_controller");
	NodeHandle nh;

	Subscriber ComPose_sub = nh.subscribe("target_pose", 5, goalPoseCallback);
    Subscriber current_Pose_sub = nh.subscribe("base_pose_ground_truth", 5, current_PoseCallback);
    
    Publisher Twist_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    Publisher target_pub = nh.advertise<geometry_msgs::Pose2D>("target_pose", 100);
    
    geometry_msgs::Pose2D target;

    Rate loop_rate(10);                                            
    float error_Linear = 0;                                                 
    float error_Angle = 0;                                                 
    geometry_msgs::Twist twistCommand;                                         

    queue<int> coordX;
    queue<int> coordY;

    for(int y=0;y<m;y++)
    {
        for(int x=0;x<n;x++) 
        {
            mapxy[x][y] = tmp_map[y][x];
        }
    }

    double sX, sY, goalX, goalY;
    nh.getParam("goalX", goalX);
    nh.getParam("goalY", goalY);

    int startposX, startposY, endposX, endposY;

    cout << goalX << " " << goalY << endl;
    startposX = 0;
    startposY = 12;
    endposX = goalX + 9;
    endposY = goalY - 8;

    coordX.push(startposX);
    coordY.push(startposY);

    
    string trajectory = planPath(startposX, startposY, endposX, endposY);
    if(trajectory == "") 
        cout<<"An empty trajectory generated!"<<endl;
    
    if(trajectory.length()>0)
    {
        int approx; char c;
        int pos_x = startposX;
        int pos_y = startposY;
        mapxy[pos_x][pos_y]=2;
        for(int i = 0;i < trajectory.length();i++)
        {
            c = trajectory.at(i);
            approx = c-'0'; 
            pos_x = pos_x + deltaX[approx];
            pos_y = pos_y + deltaY[approx];
            mapxy[pos_x][pos_y] = 3;
        
            coordX.push(pos_x);
            coordY.push(pos_y);
        }
        mapxy[pos_x][pos_y]=4;
        
    }

    
    while (ok() && nh.ok())                                        
    {
        spinOnce();
        if(!coordX.empty())
        {
        	target.x = coordX.front() - 8;
        	target.y = 10 - coordY.front();
		    target_pub.publish(target);
        	if(current_Pose.position.x <= target.x + 0.4 && current_Pose.position.y  <= target.y + 0.4 && current_Pose.position.x >= target.x - 0.4 && current_Pose.position.y  >= target.y - 0.4)
        	{
        		coordX.pop();
        		coordY.pop();
        	}
        }

        if (end == false)                                              
        {

            error_Linear = linearError(current_Pose, goal_Pose);                   
            error_Angle = angleError(current_Pose, goal_Pose);                   
            printf("Error linear: %f, Error angular: %f\n", error_Linear, error_Angle);

            if (error_Linear > 0)                                           
            {
                twistCommand.linear.x = 0.9 * error_Linear;                        
            }

            else
            {
                twistCommand.linear.x = 0;                                      
            }

            twistCommand.angular.z = 1.5 * error_Angle;                            
            Twist_pub.publish(twistCommand);                                  
        }
        
        loop_rate.sleep();                                              
    }
}


