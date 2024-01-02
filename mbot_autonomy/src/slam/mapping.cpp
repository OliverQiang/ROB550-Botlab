#include <slam/mapping.hpp>
#include <utils/grid_utils.hpp>
#include <numeric>
#include <chrono>
#include <list>
using namespace std::chrono;

Mapping::Mapping(float maxLaserDistance, int8_t hitOdds, int8_t missOdds)
: kMaxLaserDistance_(maxLaserDistance)
, kHitOdds_(hitOdds)
, kMissOdds_(missOdds)
, initialized_(false)
{
}


void Mapping::updateMap(const mbot_lcm_msgs::lidar_t& scan,
                        const mbot_lcm_msgs::pose2D_t& pose,
                        OccupancyGrid& map)
{
    /*
        inputs
            scan: new lidar scan
            pose: the pose we're currently at in the map
            map: current occupancy grid to be modified based on the new scan 
        modified
            map
    */

    if (!initialized_)
        previousPose_ = pose;
    

    MovingLaserScan movingScan(scan, previousPose_, pose);

    /// TODO: Update the map's log odds using the movingScan  
    //
    // Hint: Consider both the cells the laser hit and the cells it passed through.
    for(auto& ray : movingScan){
        scoreEndpoint(ray, map);
    }
    for(auto& ray : movingScan){
        scoreRay(ray, map);
    }
    initialized_ = true;
    previousPose_ = pose;
}

void Mapping::scoreEndpoint(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cell that the laser endpoint hits 
    // increase log odds for the occupancy grid cell that the ray ends at
    // i.e. this is a cell with an obstacle

    // ensure the rayRange is a valid distance and not the max distance
    if(ray.range <= kMaxLaserDistance_){
        Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
        Point<int> rayCell;

        rayCell.x = static_cast<int> (ray.range * std::cos(ray.theta) * map.cellsPerMeter()) + rayStart.x;
        rayCell.y = static_cast<int> (ray.range * std::sin(ray.theta) * map.cellsPerMeter()) + rayStart.y;

        if(map.isCellInGrid(rayCell.x, rayCell.y)){
            increaseCellOdds(rayCell.x, rayCell.y, map);
        }    
    }
}

void Mapping::scoreRay(const adjusted_ray_t& ray, OccupancyGrid& map)
{
    /// TODO: Implement how to score the cells that the laser ray passes through  
    // decrease log odds for the occupancy grid cells that the ray passed through
    // i.e. these are cells with no obstacles

    

    // using Bresenham's Algorithm
    std::vector<Point<int>> cells = bresenham(ray, map);
    for(auto& cell : cells){
        if(map.isCellInGrid(cell.x, cell.y)){
            decreaseCellOdds(cell.x, cell.y, map);
        }   
    }

    
}

void Mapping::increaseCellOdds(int x, int y, OccupancyGrid& map)
{
    // if(!initialized_){
    //     //do nothing
    // }else if(127 - map(x,y) > kHitOdds_){
    //     map(x,y) += kHitOdds_;
    // }else{
    //     map(x,y) = 127;
    // }
    if(127 - map(x,y) >= kHitOdds_){
        map(x,y) += kHitOdds_;
    }else{
        map(x,y) = 127;
    }
}

void Mapping::decreaseCellOdds(int x, int y, OccupancyGrid& map)
{
    // if(!initialized_){
    //     //do nothing
    // }else if(map(x,y) + 127 > kMissOdds_ ){
    //     map(x,y) -= kMissOdds_;
    // }else{
    //     map(x,y) = -127;
    // }
    if(map(x,y) + 127 >= kMissOdds_ ){
        map(x,y) -= kMissOdds_;
    }else{
        map(x,y) = -127;
    }
}

/*
Takes the ray and map, and returns a vector of map cells to check
*/
std::vector<Point<int>> Mapping::bresenham(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement the Bresenham's line algorithm to find cells touched by the ray.
    Point<float> rayStart = global_position_to_grid_cell(ray.origin, map);
    // printf(" hitodds %d, miss odds %d \n", kHitOdds_, kMissOdds_);
    // printf("%f \n", kMaxLaserDistance_);
    // rayCell_0
    int x0 = static_cast<int> (rayStart.x);
    int y0 = static_cast<int> (rayStart.y);

    // we don't want to score any cells that are outside of our range
    float range_corrected = ray.range;
    if(ray.range > kMaxLaserDistance_){
        range_corrected = kMaxLaserDistance_;
    }

    //  Point<float> endpoint = global_position_to_grid_cell(
    //     Point<float>(
    //         ray.origin.x + ray.range * 1 * std::cos(ray.theta),
    //         ray.origin.y + ray.range * 1 * std::sin(ray.theta)
    //         ), 
    //         map
    //     );

    Point<float> endpoint = global_position_to_grid_cell(
        Point<float>(
        ray.origin.x + range_corrected  * 0.9 * std::cos(ray.theta),
        ray.origin.y + range_corrected  * 0.9 * std::sin(ray.theta)
        ), 
        map
    );

    int x1 = static_cast<int> (endpoint.x);
    int y1 = static_cast<int> (endpoint.y);

    // int x1 = static_cast<int> (x0 + range_corrected * std::cos(ray.theta) * map.cellsPerMeter());
    // int y1 = static_cast<int> (y0 + range_corrected * std::sin(ray.theta) * map.cellsPerMeter());
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    // find sign values
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int x = x0;
    int y = y0;

    std::vector<Point<int>> cells; 
    cells.push_back(Point<int>(x,y));

    while(x!=x1 || y!=y1){
        int e2 = 2 * err;
        if(e2 >= -dy){
            err = err - dy;
            x = x + sx;
        }
        if(e2 <= dx){
            err = err + dx;
            y = y + sy;
        }
        cells.push_back(Point<int>(x,y));
    }
    return cells;
    
}

std::vector<Point<int>> Mapping::divideAndStepAlongRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Implement an alternative approach to find cells touched by the ray. 
    return {};
    
}
