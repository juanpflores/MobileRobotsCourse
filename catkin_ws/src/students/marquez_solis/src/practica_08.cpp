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
    this->x += delta_x + this->rng->gaussian(0, MOVEMENT_NOISE);
    this->y += delta_y + this->rng->gaussian(0, MOVEMENT_NOISE);
    this->a += delta_a + this->rng->gaussian(0, MOVEMENT_NOISE/2);
    if(this->a >   M_PI) this->a -= 2*M_PI;
    if(this->a <= -M_PI) this->a += 2*M_PI;
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
    std::vector<float> weights;
    float weights_sum = 0;
    weights.resize(particle_measurements.size());
    for(size_t i = 0; i < particle_measurements.size(); i++)
    {
	weights[i] = 0;
	for(size_t j=0; j < particle_measurements[i].ranges.size()-1; j++)
	{
	    weights[i] += exp(-pow(real_measurement.ranges[10*j] - particle_measurements[i].ranges[j], 2)/SENSOR_NOISE);
	}
	weights_sum += weights[i];
    }
    for(size_t i = 0; i < particle_measurements.size(); i++)
	weights[i] = weights_sum > 0? weights[i]/weights_sum : 0;
    
    return weights;
}

int weighted_sample_index(std::vector<float>& weights, float max_weight)
{
    random_numbers::RandomNumberGenerator rnd;
    float beta = rnd.uniformReal(0, 1.0);
    for(size_t i = 0; i < weights.size(); i++)
    {
	if(beta < weights[i])
	    return i;
	beta -= weights[i];
    }
    std::cout << "Error!!!!!" << std::endl;
    return -1; 
}

std::vector<Robot> resample(std::vector<Robot>& robots, std::vector<float>& weights)
{
    std::vector<Robot> resampled_robots;
    resampled_robots.resize(robots.size());
    float max_weight = *std::max_element(weights.begin(), weights.end());

    for(size_t i=0; i < robots.size(); i++)
    {
	int idx = weighted_sample_index(weights, max_weight);
	resampled_robots[i].x = robots[idx].x;
	resampled_robots[i].y = robots[idx].y;
	resampled_robots[i].a = robots[idx].a;
    }

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
 * This variable will be used to get the weight for each measurement.
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
     * This variable stores the map and will be used to get the simulated sensor readings foreach particle
     */
    nav_msgs::OccupancyGrid map;
    
    nav_msgs::GetMap srvGetMap;
    ros::service::waitForService("/navigation/localization/static_map");
    ros::ServiceClient srvCltGetMap = n.serviceClient<nav_msgs::GetMap>("/navigation/localization/static_map");
    srvCltGetMap.call(srvGetMap);
    map = srvGetMap.response.map;
    real_scan = *ros::topic::waitForMessage<sensor_msgs::LaserScan>("/hardware/scan");
    
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;
    float robot_x = 0;
    float robot_y = 0;
    float robot_a = 0;
    tl.waitForTransform("map", "base_link", ros::Time::now(), ros::Duration(5));
    tl.lookupTransform("map", "base_link", ros::Time(0), t);
    robot_x = t.getOrigin().x();
    robot_y = t.getOrigin().y();
    q = t.getRotation();
    robot_a = atan2(q.z(), q.w())*2;
    float last_robot_x = robot_x;
    float last_robot_y = robot_y;
    float last_robot_a = robot_a;

    random_numbers::RandomNumberGenerator rnd;
    std::vector<Robot> robots;
    std::vector<sensor_msgs::LaserScan> particle_measurements;
    particle_measurements.resize(NUMBER_OF_PARTICLES);
    robots.resize(NUMBER_OF_PARTICLES);
    for(size_t i = 0; i < robots.size(); i++)
    {
	robots[i].x = rnd.uniformReal(-1,10);
	robots[i].y = rnd.uniformReal(-1,8);
	robots[i].a = rnd.uniformReal(-3.14, 3.14);
    }
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

	float delta_d = sqrt(pow(robot_x - last_robot_x,2) + pow(robot_y - last_robot_y,2));
        float delta_a = fabs(robot_a - last_robot_a);
	if(delta_d > DIST_THRESHOLD || delta_a > ANGLE_THRESHOLD)
	{
	    for(size_t i = 0; i < robots.size(); i++)
	    {
		robots[i].Move(robot_x - last_robot_x, robot_y - last_robot_y,  robot_a - last_robot_a);
	    	particle_measurements[i] = robots[i].SimulateSense(map);
	    }
	    std::vector<float> weights = measurement_weights(particle_measurements, real_scan);
	    robots = resample(robots, weights);

	    last_robot_x = robot_x;
	    last_robot_y = robot_y;
	    last_robot_a = robot_a;
	}
	
	pub_markers.publish(particles_marker(robots));
	pub_test.publish(robots[0].SimulateSense(map));
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
