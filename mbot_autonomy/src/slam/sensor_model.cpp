#include <slam/sensor_model.hpp>
#include <slam/moving_laser_scan.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <utils/grid_utils.hpp>
#include <utils/geometric/point.hpp>
SensorModel::SensorModel(void)
:   sigma_hit_(0.003),
	occupancy_threshold_(0),
	ray_stride_(7),
	max_ray_range_(1000),
    search_range(2),
    offset_quality_weight(3),
    z_hit_(0.6), 
    z_rand_(0.005) 
{
    initialize_bfs_offsets();
}

void SensorModel::initialize_bfs_offsets()
{
    /// TODO: Initialize the BFS offsets based on the search range 
    bfs_offsets_.clear();
    for(int y = -search_range; y <= search_range; ++y){
        for(int x = -search_range; x <= search_range; ++x){
            bfs_offsets_.emplace_back(x, y);
        }
    }
}


float SensorModel::likelihood(const mbot_lcm_msgs::particle_t& sample, 
                               const mbot_lcm_msgs::lidar_t& scan, 
                               const OccupancyGrid& map)
{
    float q = 1.0;

    MovingLaserScan movingScan(scan, sample.parent_pose, sample.pose);
    
    for(const auto& ray : movingScan){
        if (ray.range >= max_ray_range_) continue;

        // float x_z_t = sample.pose.x + (ray.range * std::cos(sample.pose.theta + ray.theta));
        // float y_z_t = sample.pose.y + (ray.range * std::sin(sample.pose.theta + ray.theta));
        
        // float x_z_t = sample.pose.x + ((ray.origin.x + ray.range)* std::cos(sample.pose.theta)) - ((ray.origin.y + ray.range) * std::sin(sample.pose.theta)) + (ray.range * std::cos(sample.pose.theta + ray.theta));
        // float y_z_t = sample.pose.y + ((ray.origin.y + ray.range) * std::cos(sample.pose.theta)) + ((ray.origin.x + ray.range) * std::sin(sample.pose.theta)) + (ray.range * std::sin(sample.pose.theta + ray.theta));
        Point<float> ray_end_point = getRayEndPointOnMap(ray, map);

        Point<float> rayEnd = global_position_to_grid_position(rayEnd, map);

        Point<int> nearestOccupiedCell = gridBFS(rayEnd, map);

        if(nearestOccupiedCell.x >= 0 && nearestOccupiedCell.y >= 0) {
            float distanceToNearestOccupiedCell = distance_between_points(rayEnd, nearestOccupiedCell);
            float z_hit_prob = NormalPdf(distanceToNearestOccupiedCell/ map.metersPerCell());
            float z_rand_prob = 1.0 / max_ray_range_;
            q *= (z_hit_ * z_hit_prob + z_rand_ * z_rand_prob);
        }
    }
    
    return q;
}


float SensorModel::scoreRay(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Compute a score for a given ray based on its end point and the map. 
    // Consider the offset from the nearest occupied cell.  
}

float SensorModel::NormalPdf(const double& x)
{
    return (1.0/(sqrt(2.0 * M_PI)*sigma_hit_))*exp((-0.5*x*x)/(sigma_hit_*sigma_hit_));
}

Point<int> SensorModel::gridBFS(const Point<int> end_point, const OccupancyGrid& map)
{
    /// TODO: Use Breadth First Search to find the nearest occupied cell to the given end point. 
    Point<int> nearestOccupiedCell(-1, -1);
    double minDist = std::numeric_limits<double>::infinity();
    
    for (const auto& offset : bfs_offsets_){
        Point<int> check_point = end_point + offset;
        // printf("logodds : %d \n", map.logOdds(check_point.x, check_point.y));
        if(map.isCellInGrid(check_point.x, check_point.y) &&
           map.logOdds(check_point.x, check_point.y) > occupancy_threshold_){

            double dist = distance_between_points(end_point, check_point);
            if (dist < minDist) {
                minDist = dist;
                nearestOccupiedCell = check_point;
            }
        }
        // printf("min dist: %f: \n", minDist);
    }
    return nearestOccupiedCell;
}

Point<float> SensorModel::getRayEndPointOnMap(const adjusted_ray_t& ray, const OccupancyGrid& map)
{
    /// TODO: Calculate the end point of a given ray on the map 
    Point<double> endpoint(ray.origin.x + ray.range * std::cos(ray.theta), 
                         ray.origin.y + ray.range * std::sin(ray.theta));
    // auto rayEnd = global_position_to_grid_position(endpoint, map);
    return endpoint; 
    
}
