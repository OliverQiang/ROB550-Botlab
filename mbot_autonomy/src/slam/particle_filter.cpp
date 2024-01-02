#include <utils/grid_utils.hpp>
#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <mbot_lcm_msgs/pose2D_t.hpp>
#include <mbot_lcm_msgs/particle_t.hpp>
#include <cassert>
#include <utils/geometric/angle_functions.hpp>


ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles),
  samplingAugmentation_(0.5, 0.9, numParticles),
  distribution_quality(1),
  quality_reinvigoration_percentage(0.1),
  occupancy_threshold_(10)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}


void ParticleFilter::initializeFilterAtPose(const mbot_lcm_msgs::pose2D_t& pose, const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    float sampleWeight = 1.0 / kNumParticles_;
    posteriorPose_ = pose;
    // printf("initialize first set of particles completed ---------");

    for (auto& p : posterior_) {
        
        // if (!map.isCellInGrid(p.pose.x, p.pose.y)) {
        //     std::cerr << "Error: Particle initialized out of map bounds.\n";
        //     continue;
        // }
        // // Ensure that the pose is not in an occupied space
        // auto cellStatus = map.logOdds(p.pose.x, p.pose.y);
        // if (cellStatus > occupancy_threshold_) {
        //     std::cerr << "Error: Particle initialized in an occupied space.\n";
        //     continue;
        // }

        p.pose.x = posteriorPose_.x;
        p.pose.y = posteriorPose_.y;
        p.pose.theta = wrap_to_pi(posteriorPose_.theta);
        p.pose.utime = pose.utime;
        p.parent_pose = p.pose;
        p.weight = sampleWeight;
    }
    
}

void ParticleFilter::initializeFilterRandomly(const OccupancyGrid& map)
{
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////
    
    printf("initializeFilterRandomly not executed !\n");
   

}

void ParticleFilter::resetOdometry(const mbot_lcm_msgs::pose2D_t& odometry)
{
    actionModel_.resetPrevious(odometry);
}


mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilter(const mbot_lcm_msgs::pose2D_t& odometry,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{  
    //  printf("update filter \n");

    if (flag){
        actionModel_.resetPrevious(odometry);
        flag = false;
    }


    bool hasRobotMoved = actionModel_.updateAction(odometry);
    // printf("true");
    if (hasRobotMoved)
    {
        // printf("robot moved = %d \n", hasRobotMoved);
        auto prior = resamplePosteriorDistribution(map);
        auto proposal = computeProposalDistribution(prior);
        posterior_ = computeNormalizedPosterior(proposal, laser, map);
        posteriorPose_ = estimatePosteriorPose(posterior_);
        posteriorPose_.utime = odometry.utime;
    }
    // posteriorPose_ = estimatePosteriorPose(posterior_);

    // posteriorPose_.utime = odometry.utime;
    // printf("posterior Pose x: %f, y: %f, theta: %f\n", posteriorPose_.x, posteriorPose_.y, posteriorPose_.theta);  
    return posteriorPose_;
}

mbot_lcm_msgs::pose2D_t ParticleFilter::updateFilterActionOnly(const mbot_lcm_msgs::pose2D_t& odometry)
{
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    printf("Not executing updateFilterActionOnly");
    // bool hasRobotMoved = actionModel_.updateAction(odometry);

    // if(hasRobotMoved)
    // {
    //     auto prior = resamplePosteriorDistribution();
    //     auto proposal = computeProposalDistribution(prior);
    //     posterior_ = proposal;
    // }

    // posteriorPose_ = odometry;

    // return posteriorPose_;
}


mbot_lcm_msgs::pose2D_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


mbot_lcm_msgs::particles_t ParticleFilter::particles(void) const
{
    mbot_lcm_msgs::particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const OccupancyGrid& map,
                                                           const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////

    // printf("here in resamplePosteriorDistribution \n");
    ParticleList newParticles;

    // Step 1: Generate a random number r in the range [0, M^-1]
    std::random_device rd;
    std::mt19937 generator(rd());
    std::uniform_real_distribution<float> dist(0.0, 1.0/kNumParticles_);

    float r = dist(generator);

    // Step 2: Initialize the cumulative weight c to the weight of the first particle
    double c = posterior_[0].weight;

    // Step 3: Loop through each particle to resample
    int i = 0;
    for(int m = 0; m < kNumParticles_; ++m) {
        // Step 4: The 'wheel' algorithm to select the particle
        float U = r + (m-1) * (1.0/kNumParticles_);
        while(U > c) {
            i = i + 1;
            c = c + posterior_[i].weight;
        }
        newParticles.push_back(posterior_[i]);
    }

    return newParticles;
    // ParticleList samples;
    // int num_particles = kNumParticles_;

    // // if (num_particles < 1 || posterior_.size() < 1) return samples;

    // std::random_device rd{};
    // std::mt19937 gen{rd()};
    // std::uniform_real_distribution<float> distribution(0.0, 1.0 / num_particles);

    // float r = distribution(gen);
    // int idx = 0;
    // float s = posterior_[idx].weight;

    // for (int i = 0; i < num_particles; ++i)
    // {
    //     float u = r + (i * (1. / num_particles));
    //     while (u > s) {
    //         ++idx;
    //         s += posterior_[idx].weight;
    //     }
    //     samples.push_back(posterior_[idx]);
    // }

    // return samples;
}


ParticleList ParticleFilter::resamplePosteriorDistribution(const bool keep_best,
                                                           const bool reinvigorate)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    printf("here in 2nd resamplePosteriorDistribution--------------------------------------- \n");

    // std::vector<mbot_lcm_msgs::particle_t> prior = posterior_;
    // double sampleWeight = 1.0/kNumParticles_;
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::normal_distribution<> dist(0.0,0.01);

    // // replace this with lowvarice algorithm
    // for(auto& p : prior){
    //     p.pose.x = posteriorPose_.x + dist(generator);
    //     p.pose.y = posteriorPose_.y + dist(generator);
    //     p.pose.theta = posteriorPose_.theta + dist(generator);
    //     p.pose.utime = posteriorPose_.utime;
    //     p.parent_pose = posteriorPose_;
    //     p.weight = sampleWeight;
    // }

    // return prior;
    // ParticleList newParticles;

    // // Step 1: Generate a random number r in the range [0, M^-1]
    // std::random_device rd;
    // std::mt19937 generator(rd());
    // std::uniform_real_distribution<> dist(0, 1.0/kNumParticles_);

    // double r = dist(generator);

    // // Step 2: Initialize the cumulative weight c to the weight of the first particle
    // double c = posterior_[0].weight;

    // // Step 3: Loop through each particle to resample
    // int i = 0;
    // for(int m = 0; m < kNumParticles_; ++m) {
    //     // Step 4: The 'wheel' algorithm to select the particle
    //     double U = r + m * (1.0/kNumParticles_);
    //     while(U > c) {
    //         i = i + 1;
    //         c = c + posterior_[i].weight;
    //     }
    //     newParticles.push_back(posterior_[i]);
    // }

    // return newParticles;

}


void ParticleFilter::reinvigoratePriorDistribution(ParticleList& prior)
{
    // Augmentation: if sensor model suspects an average particle quality of
    //      less than 15%, invigorate
    if (distribution_quality < 0.15)  // TODO: make 0.15 a parameter
    {
        // printf("inside reinvogorate");
        int count = 0;
        int max_count = floor(quality_reinvigoration_percentage * prior.size());

        std::random_device rd;
        std::default_random_engine generator(rd());
        auto ud01 = std::uniform_real_distribution<double>(0.0, 1.0);
        int step = std::max<int>(1, floor(ud01(generator) * prior.size() / max_count));

        for (int i = 0; i < max_count; i++)
        {
            prior[i*step] = randomPoseGen_.get_particle();
        }

    }

    // // Augmentation: randomize any unreasonable samples
    // if(map != nullptr)
    // {
    //     for (int i = 0; i < prior.size(); i++)
    //     {
    //         const auto& p = prior[i].pose;
    //         if(!map->isCellInGrid(p.x, p.y) ||
    //           (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0)))
    //         {
    //             std::cout << "\tinvalid sample!!" << ", "
    //                 << !map->isCellInGrid(p.x, p.y) << ", "
    //                 << (map->isCellInGrid(p.x, p.y) && (map->logOdds(p.x, p.y) >= 0.0))
    //                 << std::endl;


    //             prior[i] = randomPoseGen_.get_particle();
    //         }
    //     }
    // }
    // return prior;
}


ParticleList ParticleFilter::computeProposalDistribution(const ParticleList& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    
    std::vector<mbot_lcm_msgs::particle_t> proposal;
    // for each partical in the prior distribution, apply the action
    for(auto& p : prior){
        proposal.push_back(actionModel_.applyAction(p));
    }
    // printf("proposal size %lu",proposal.size());
    return proposal;

}


ParticleList ParticleFilter::computeNormalizedPosterior(const ParticleList& proposal,
                                                        const mbot_lcm_msgs::lidar_t& laser,
                                                        const OccupancyGrid& map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the
    ///////////       particles in the proposal distribution

    ParticleList posterior = ParticleList();
    double sumWeights = 0.0;
    for(auto& p : proposal){
        mbot_lcm_msgs::particle_t weighted = p;
        weighted.weight= sensorModel_.likelihood(weighted, laser, map);
        // printf("weight %f \n", weighted.weight );
        sumWeights += weighted.weight;
        posterior.push_back(weighted);

    }

    for(auto& p : posterior){
        p.weight /= sumWeights;
    }

    return posterior;

}


mbot_lcm_msgs::pose2D_t ParticleFilter::estimatePosteriorPose(const ParticleList& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    // Figure out which pose to take for the posterior pose
    // Weighted average is simple, but could be very bad
    // Maybe only take the best x% and then average.

    mbot_lcm_msgs::pose2D_t pose =  computeParticlesAverage(posterior);

    // mbot_lcm_msgs::pose2D_t pose;

    // double xMean = 0.0;
    // double yMean = 0.0;
    // double cosThetaMean = 0.0;
    // double sinThetaMean = 0.0;
    // for(auto& p : posterior){
    //     xMean += p.weight * p.pose.x;
    //     yMean += p.weight * p.pose.y;
    //     cosThetaMean += p.weight * std::cos(p.pose.theta);
    //     sinThetaMean += p.weight * std::sin(p.pose.theta);
    // }

    // pose.x = xMean;
    // pose.y = yMean;
    // pose.theta = std::atan2(sinThetaMean, cosThetaMean);
    
    // // printf("pose x: %f, y: %f, theta: %f\n", pose.x, pose.y, pose.theta);
    return pose;

}

mbot_lcm_msgs::pose2D_t ParticleFilter::computeParticlesAverage(const ParticleList& particles_to_average)
{
    mbot_lcm_msgs::pose2D_t avg_pose;
    avg_pose.x = 0.0;
    avg_pose.y = 0.0;
    avg_pose.theta = 0.0;
    double sum_weight = 0.0;

    // Aux variables to compute theta average
    double theta_x = 0.0;
    double theta_y = 0.0;
    for (auto &p : particles_to_average)
    {   
        // printf("weight: %f \n", p.weight);
        avg_pose.x += p.weight * p.pose.x;
        avg_pose.y += p.weight * p.pose.y;
        theta_x += p.weight * std::cos(p.pose.theta);
        theta_y += p.weight * std::sin(p.pose.theta);

        sum_weight += p.weight;
    }
    avg_pose.x /= sum_weight;
    avg_pose.y /= sum_weight;
    theta_x /= sum_weight;
    theta_y /= sum_weight;
    avg_pose.theta = std::atan2(theta_y, theta_x);

    return avg_pose;
}
