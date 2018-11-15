/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   8
 * LOCALIZACION CON FILTROS DE PARTICULAS
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "visualization_msgs/Marker.h"
#include "random_numbers/random_numbers.h"
#include "occupancy_grid_utils/ray_tracer.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#define NOMBRE "APELLIDO_PATERNO_APELLIDO_MATERNO"

#define NUMBER_OF_PARTICLES 1000
#define SENSOR_NOISE 0.5
#define MOVEMENT_NOISE 0.1
#define DIST_THRESHOLD 0.2
#define ANGLE_THRESHOLD 0.2

class Robot
{
public:
    Robot();
    void Move(float delta_x, float delta_y, float delta_a);
    sensor_msgs::LaserScan SimulateSense(nav_msgs::OccupancyGrid& map);
    random_numbers::RandomNumberGenerator* rng;
    
    float x;
    float y;
    float a;
};

Robot::Robot()
{
    this->x = 0;
    this->y = 0;
    this->a = 0;
    this->rng = new random_numbers::RandomNumberGenerator();
}

void Robot::Move(float delta_x, float delta_y, float delta_a)
{
    /*
     * TODO:
     * Add the movement given by delta_x, delta_y, delta_a, to the
     * current position and add gaussian noise with zero mean and covariance
     * MOVEMENT_NOISE. Remember to keep the orientation in the interval (-pi, pi]
     * Hint: Use the function 'gaussian' of the RandomNumberGenerator object.
     */
}

sensor_msgs::LaserScan Robot::SimulateSense(nav_msgs::OccupancyGrid& map)
{
    sensor_msgs::LaserScan scanInfo;
    scanInfo.header.frame_id = "laser_link";
    scanInfo.angle_min = -2;
    scanInfo.angle_max = 2;
    scanInfo.angle_increment = 0.07;
    scanInfo.scan_time = 0.1;
    scanInfo.range_min = 0.01;
    scanInfo.range_max = 4.0;
    sensor_msgs::LaserScan simulatedScan;
    geometry_msgs::Pose sensorPose;
    sensorPose.position.x = this->x;
    sensorPose.position.y = this->y;
    sensorPose.orientation.w = cos(this->a*0.5);
    sensorPose.orientation.z = sin(this->a*0.5);
    simulatedScan = *occupancy_grid_utils::simulateRangeScan(map, sensorPose, scanInfo);
    return simulatedScan;
}

std::vector<float> measurement_weights(std::vector<sensor_msgs::LaserScan>& particle_measurements,
				       sensor_msgs::LaserScan real_measurement)
{
    /*
     * TODO:
     * Write the algorithm to get the probability of being resampled for each particle.
     *
     * weight_sum = 0
     * FOREACH Particle-measurement:
     *   weight_i = 0
     *   FOREACH laser_reading:
     *     weight_i = weight_i + exp(-error^2/sensor_noise)
     *   weight_sum = weight_sum + weight_i
     * FOREACH Particle-measurement:
     *   weight_i = weight_i / weight_sum
     * RETURN set of weight_i
     */
    std::vector<float> weights;
    weights.resize(particle_measurements.size());
    
    return weights;
}

int weighted_sample_index(std::vector<float>& weights, float max_weight)
{
    /*
     * TODO:
     * Write a function to randomly sample an integer (index) in the interval [0, n-1]
     * with n = number of weights (size of weights)
     * with a distribution given by the corresponding weights, e.g., if
     * weights = [0.1, 0.5, 0.2, 0.2] then, the index 1 should have the biggest probability
     * if being chosen, indices 2 and 3 should have the same probability and index 0
     * should have the smallest probability. Nevertheless, chosing an index must still be
     * a random process.
     * Hint: check first answer in https://stackoverflow.com/questions/1761626/weighted-random-numbers
     * (that's why you need the max_weight parameter)
     */
}

std::vector<Robot> resample(std::vector<Robot>& robots, std::vector<float>& weights)
{
    std::vector<Robot> resampled_robots;
    resampled_robots.resize(robots.size());

    /*
     * TODO:
     * Write the code to RESAMPLE n particles from the set of particles given by 'robots' where
     * 'n' is the size of 'robots', i.e., you will get the same number of particles but the same particles.
     * To do this:
     * Calculate the maximum weight.
     * FOR i in [0, robots_size):
     *   Chose a weighted random index using the function weighted_sample_index
     *   Chose the robot with this index
     * Return the new set of robots
     */

    return resampled_robots;
}

visualization_msgs::Marker particles_marker(std::vector<Robot>& robots)
{
    visualization_msgs::Marker mrk;
    mrk.header.frame_id = "map";
    mrk.ns = "loc_particles";
    mrk.id = 0;
    mrk.type = visualization_msgs::Marker::POINTS;
    mrk.action = visualization_msgs::Marker::ADD;
    mrk.scale.x = 0.05;
    mrk.scale.y = 0.07;
    mrk.color.a = 1.0;
    mrk.color.r = 1.0;
    mrk.color.g = 0;
    mrk.color.b = 0;
    mrk.points.resize(robots.size());
    for(size_t i=0; i < robots.size(); i++)
    {
        mrk.points[i].x = robots[i].x;
        mrk.points[i].y = robots[i].y;
    }
    return mrk;
}

/*
 * This variable stores the real sensor readings and it will be used to get the weight for each measurement.
 */
sensor_msgs::LaserScan real_scan;
void callback_laser_scan(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    real_scan = *msg;
}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 08 - LOCALIZACION CON FILTROS DE PARTICULAS - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_08");
    ros::NodeHandle n("~");
    ros::Rate loop(10);

    ros::Subscriber sub_scan = n.subscribe("/hardware/scan", 1, callback_laser_scan);
    ros::Publisher  pub_markers  = n.advertise<visualization_msgs::Marker>("/hri/visualization_marker", 1);
    ros::Publisher  pub_test = n.advertise<sensor_msgs::LaserScan>("/laser_test", 1);

     /*
     * Important variables.
     */
    float robot_x; // Stores the current robot position and orientation
    float robot_y; 
    float robot_a;
    float last_robot_x; //Auxiliar variables to calculate the robot movements and decide when to resample.
    float last_robot_y;
    float last_robot_a;
    nav_msgs::OccupancyGrid map; //The map, used to simulate the sensor readings for each particle
    std::vector<Robot> robots;   //The set of particles
    std::vector<sensor_msgs::LaserScan> particle_measurements; //The set of simulated sensor readings for each particle

    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    map = srvGetMap.response.map;
    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan");
    
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;
    robot_x = 0;
    robot_y = 0;
    robot_a = 0;
    tl.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(5));
    tl.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
    last_robot_x = robot_x;
    last_robot_y = robot_y;
    last_robot_a = robot_a;

    random_numbers::RandomNumberGenerator rnd;
    particle_measurements.resize(NUMBER_OF_PARTICLES);
    robots.resize(NUMBER_OF_PARTICLES);

    /*
     * TODO:
     * Initialize the N particles with randomly distrbuted positions and orientations.
     * Positions should be within the map dimensions and orientations should be in (-pi,pi]
     * Hint: Use the function uniformReal of the RandomNumberGenerator class
     */

    while(ros::ok())
    {
         /*
         * Instructions needed to get the current robot position.
         */   
        tl.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(0.5));
        tl.lookupTransform("map", "base_link", ros::Time(0), t);
        robot_x = t.getOrigin().x();
        robot_y = t.getOrigin().y();
        q = t.getRotation();
        robot_a = atan2(q.z(), q.w())*2;

        /*
         * TODO:
         * If the change in position or orientation is greater than DIST_THRESHOLD or ANGLE_THRESHOLD then:
         *   FOREACH particle:
         *     Update position and orientation using the function 'Move'
         *     Get the simulated sensor readings using the function 'SimulateSense' and store them in 'particle_measurements'
         *   Get the weights for each particle using the 'measurement_weights' function.
         *   Resample particles using the 'resample' function.
         *   Change the last_robot position and orientation.
         */
        
        pub_markers.publish(particles_marker(robots));
        pub_test.publish(robots[0].SimulateSense(map));
        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}
