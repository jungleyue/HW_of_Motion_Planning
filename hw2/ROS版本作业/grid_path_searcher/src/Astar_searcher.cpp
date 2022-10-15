#include "Astar_searcher.h"

using namespace std;
using namespace Eigen;

void AstarPathFinder::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNodePtr ** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNodePtr * [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNodePtr [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE;k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = gridIndex2coord(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }
}

void AstarPathFinder::resetGrid(GridNodePtr ptr)
{
    ptr->id = 0;
    ptr->cameFrom = NULL;
    ptr->gScore = inf;
    ptr->fScore = inf;
}

void AstarPathFinder::resetUsedGrids()
{   
    for(int i=0; i < GLX_SIZE ; i++)
        for(int j=0; j < GLY_SIZE ; j++)
            for(int k=0; k < GLZ_SIZE ; k++)
                resetGrid(GridNodeMap[i][j][k]);
}

void AstarPathFinder::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x > gl_xu || coord_y > gl_yu || coord_z > gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

vector<Vector3d> AstarPathFinder::getVisitedNodes()
{   
    vector<Vector3d> visited_nodes;
    for(int i = 0; i < GLX_SIZE; i++)
        for(int j = 0; j < GLY_SIZE; j++)
            for(int k = 0; k < GLZ_SIZE; k++){   
                //if(GridNodeMap[i][j][k]->id != 0) // visualize all nodes in open and close list
                if(GridNodeMap[i][j][k]->id == -1)  // visualize nodes in close list only
                    visited_nodes.push_back(GridNodeMap[i][j][k]->coord);
            }

    ROS_WARN("visited_nodes size : %d", visited_nodes.size());
    return visited_nodes;
}

Vector3d AstarPathFinder::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i AstarPathFinder::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d AstarPathFinder::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

inline bool AstarPathFinder::isOccupied(const Eigen::Vector3i & index) const
{
    return isOccupied(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isFree(const Eigen::Vector3i & index) const
{
    return isFree(index(0), index(1), index(2));
}

inline bool AstarPathFinder::isOccupied(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    //cout<<data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z]<<endl;
    return  (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
            (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] == 1));
}

inline bool AstarPathFinder::isFree(const int & idx_x, const int & idx_y, const int & idx_z) const 
{
    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

inline void AstarPathFinder::AstarGetSucc(GridNodePtr currentPtr, vector<GridNodePtr> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    neighborPtrSets.clear();
    edgeCostSets.clear();
    
    int i,j,k;
    Eigen::Vector3i current_index=currentPtr->index;
    GridNodePtr neighborPtr;
    for(i=-1;i<2;i++)
    {
        for(j=-1;j<2;j++)
        {
            for(k=-1;k<2;k++)
            {
                if(i*i+j*j+k*k==0)
                {
                    continue;
                }

                Vector3i idx=current_index+Vector3i(i,j,k);

                if(isOccupied(idx))
                {
                    continue;
                }

                if(idx(0) < 0 || idx(0) >= GLX_SIZE || idx(1) < 0 || idx(1) >= GLY_SIZE || idx(2) < 0 || idx(2) >=GLZ_SIZE)
                {
                    continue;
                }
                
                neighborPtr = GridNodeMap[idx(0)][idx(1)][idx(2)];
                //cout<<idx(2)<<endl;
                
                if (neighborPtr->id == -1)
                {
                    continue;
                }
                
                neighborPtrSets.push_back(neighborPtr);
                
                edgeCostSets.push_back(sqrt(
                    (idx(0) - currentPtr->index(0)) * (idx(0) - currentPtr->index(0)) +
                    (idx(1) - currentPtr->index(1)) * (idx(1) - currentPtr->index(1)) +
                    (idx(2) - currentPtr->index(2)) * (idx(2) - currentPtr->index(2))));
                
            }
        }
    }

}

double AstarPathFinder::getHeu(GridNodePtr node1, GridNodePtr node2)
{
    /* 
    choose possible heuristic function you want
    Manhattan, Euclidean, Diagonal, or 0 (Dijkstra)
    Remember tie_breaker learned in lecture, add it here ?
    */
    //Euclidean_coord
    /*
    double x_distance=node1->coord[0]-node2->coord[0];
    double y_distance=node1->coord[1]-node2->coord[1];
    double z_distance=node1->coord[2]-node2->coord[2];
    return sqrt(x_distance*x_distance+y_distance*y_distance+z_distance*z_distance);
    */
    //Euclidean_index
    /*
    double x_distance=node1->index[0]-node2->index[0];
    double y_distance=node1->index[1]-node2->index[1];
    double z_distance=node1->index[2]-node2->index[2];
    return sqrt(x_distance*x_distance+y_distance*y_distance+z_distance*z_distance);
    */
    //Manhattan
    
    double dis;
    dis = abs(node1->index(0) - node2->index(0)) +
          abs(node1->index(1) - node2->index(1)) +
          abs(node1->index(2) - node2->index(2));
    return dis;
    



}

void AstarPathFinder::AstarGraphSearch(Vector3d start_pt, Vector3d end_pt)
{   
    ros::Time time_1 = ros::Time::now();    

    //index of start_point and end_point
    Vector3i start_idx = coord2gridIndex(start_pt);
    Vector3i end_idx   = coord2gridIndex(end_pt);
    goalIdx = end_idx;
    //position of start_point and end_point
    start_pt = gridIndex2coord(start_idx);
    end_pt   = gridIndex2coord(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNodePtr startPtr = new GridNode(start_idx, start_pt);
    GridNodePtr endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNodePtr currentPtr  = NULL;
    GridNodePtr neighborPtr = NULL;

    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    //STEP 1: finish the AstarPathFinder::getHeu , which is the heuristic function
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;
    startPtr->nodeMapIt=openSet.insert( make_pair(startPtr -> fScore, startPtr));
    /*
    *
    STEP 2 :  some else preparatory works which should be done before while loop
    please write your code below
    *
    *
    */
    

    vector<GridNodePtr> neighborPtrSets;
    vector<double> edgeCostSets;

    // this is the main loop
    while ( !openSet.empty() )
    {  
        //ROS_INFO("000000");
      

        currentPtr = (*openSet.begin()).second;

        openSet.erase(currentPtr->nodeMapIt);
        //ROS_INFO("1111111112");
        if (currentPtr->id == -1)
        {
            ROS_INFO("yes");
            continue;
        }
        currentPtr->id = -1;
        // if the current node is the goal 
        if( currentPtr->index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr->gScore * resolution );            
            return ;
        }
        //ROS_INFO("1111111113");
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     
        //ROS_INFO("1111111111");
        //cout<<neighborPtrSets.size()<<endl;
        for(int i = 0; i < (int)neighborPtrSets.size(); i++)
        {    
            neighborPtr = neighborPtrSets[i];
            if(neighborPtr->id==0)
            { //discover a new node, which is not in the closed set and open set
                neighborPtr->id = 1;
                neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                neighborPtr->fScore = getHeu(neighborPtr, endPtr) + neighborPtr->gScore;
                neighborPtr->cameFrom = currentPtr;
                neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                //ROS_INFO("222222222");
                continue;
            }
            else if(neighborPtr->id==1)
            {
                if(neighborPtr->gScore > currentPtr->gScore + edgeCostSets[i])
                {
                    openSet.erase(neighborPtr->nodeMapIt);
                    neighborPtr->gScore = currentPtr->gScore + edgeCostSets[i];
                    neighborPtr->fScore = getHeu(neighborPtr, endPtr) + neighborPtr->gScore;
                    neighborPtr->cameFrom = currentPtr;
                    neighborPtr->nodeMapIt = openSet.insert(make_pair(neighborPtr->fScore, neighborPtr));
                }
            }
            //ROS_INFO("33333333333");
        }      
    }
    
    //if search fails
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
}


vector<Vector3d> AstarPathFinder::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNodePtr> gridPath;
    /*
    *
    *
    STEP 8:  trace back from the curretnt nodePtr to get all nodes along the path
    please write your code below
    *      
    */
    GridNodePtr Ptr = NULL;
    Ptr=terminatePtr;
    while (Ptr!= NULL)
    {
        gridPath.push_back(Ptr);
        Ptr=Ptr->cameFrom;
    }
    for (auto ptr: gridPath)
        path.push_back(ptr->coord);
        
    reverse(path.begin(),path.end());


    return path;
}