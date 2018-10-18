/*
 * ROBOTS MÓVILES Y AGENTES INTELIGENTES
 * FACULTAD DE INGENIERÍA, UNAM, 2019-1
 * P R Á C T I C A   5
 * SEGUIMIENTO DE RUTAS
 */

#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/GetPlan.h"
#include "tf/transform_listener.h"
#define NOMBRE "JIMENEZ_JIMENEZ"

/*
 * Variables to store the current robot position and the global goal position.
 */
float global_goal_x;
float global_goal_y;
float robot_x;
float robot_y;
float robot_a;
int aux;

/*
 * global_plan: stores the path to be tracked.
 * cltPlanPath: service client used to call the service for calculating a path. 
 */
nav_msgs::Path global_plan;
ros::ServiceClient clt_plan_path;


void callback_go_to_xya(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    /*
     * Callback function. Any time a new goal xya is received, the goal coordinates and the path are
     * stored in the corresponding variables.
     */
    global_goal_x = msg->data[0];
    global_goal_y = msg->data[1];
    std::cout << "Practica05.->Global goal point received: X=" << global_goal_x << "\tY=" << global_goal_y << std::endl;

    std::cout << "Practica05.->Calling service for path planning..." << std::endl;
    nav_msgs::GetPlan srv;
    srv.request.start.pose.position.x = robot_x;
    srv.request.start.pose.position.y = robot_y;
    srv.request.goal.pose.position.x  = global_goal_x;
    srv.request.goal.pose.position.y  = global_goal_y;
    clt_plan_path.call(srv);
    global_plan = srv.response.plan;
    aux = 1;

}
/*Fucnion para el calculo de los goals parciales
*primero recorre el global_plan hasta que la diferencia de distancias entre
*el robot y el goal parcial sea de 20 cm
*como es referenciado se pasa los valores de las posiciones encontradas al vector goal_parcial
*regresa en donde se quedo el iterador para seguir desde esepunto en proximas iteraaciones
*/
int goal_p(nav_msgs::Path global_plan, float r_x, float r_y, float* goal_parcial, int pos){
    for(int i=pos; i<global_plan.poses.size(); i++){
        float d = sqrt((global_plan.poses[i].pose.position.x - r_x)*(global_plan.poses[i].pose.position.x - r_x) 
                            + (global_plan.poses[i].pose.position.y - r_y)*(global_plan.poses[i].pose.position.y - r_y));
        if(d >= 0.2){
            goal_parcial[0] = global_plan.poses[i].pose.position.x;
            goal_parcial[1] = global_plan.poses[i].pose.position.y;
            return i;
        }
    }

}

int main(int argc, char** argv)
{
    std::cout << "PRÁCTICA 05 - SEGUIMIENTO DE RUTAS - " << NOMBRE << std::endl;
    ros::init(argc, argv, "practica_05");
    ros::NodeHandle n("~");
    ros::Rate loop(20);

    ros::Subscriber sub_goto_xya = n.subscribe("/navigation/go_to_xya", 1, callback_go_to_xya);
    ros::Publisher  pub_cmd_vel  = n.advertise<geometry_msgs::Twist>("/hardware/mobile_base/cmd_vel", 1);
    clt_plan_path = n.serviceClient<nav_msgs::GetPlan>("/navigation/path_planning/a_star_search");
    tf::TransformListener tl;
    tf::StampedTransform t;
    tf::Quaternion q;

    geometry_msgs::Twist msg_cmd_vel;
    			
    float dif_m = 0;
    float goal_parcial[2];
    int p=0;
    float error_m = 1;
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
	 * Write the code necessary to follow the path stored in 'global_path'.
	 * Use the position control for a DIFFERENTIAL base.
	 * Store the linear and angular speed in msg_cmd_vel.
	 */
    
    float alpha = 0.6548;
    float beta  = 0.01;
    float v_max = 0.5;
    float w_max = 0.5;

   
    if(aux==1){
        if(error_m > 0.1){
            if(dif_m <= 0.8){//distancia entre robot y goal parcial para que vuelva a calcular los sig goales parciales
               p = goal_p(global_plan,robot_x,robot_y,goal_parcial,p);//calulo de los goals parciales, 

            }else if(error_m <= 0.22){//chaca si esta cerca del goal global y cconsigue el ultimo punto
                                        //ya que si no se queda oscilando
                goal_parcial[0] = global_plan.poses[global_plan.poses.size()-1].pose.position.x;
                goal_parcial[1] = global_plan.poses[global_plan.poses.size()-1].pose.position.y;
            }
            /*calulo de errores, se usa el goal parcial del punto calculado en la funcion anterior*/
            float error_x = goal_parcial[0] - robot_x;
            float error_y = goal_parcial[1] - robot_y;
            float error_a = atan2(error_y, error_x) - robot_a;
            /* calculo de errores:
            dif_m -> diferencia  o error entre el goal parcial y la posicion del robot
            error_m -> distancia entre el goal global y la posicion del robot*/
            dif_m = sqrt(error_x*error_x + error_y*error_y);
            error_m = sqrt((global_goal_x-robot_x)*(global_goal_x-robot_x) + (global_goal_y-robot_y)*(global_goal_y-robot_y));
            //std::cout<<"d: "<<dif_m<<std::endl;
             /*condiciones para girar al sentido mas proximo pa alcanzar el angulo deseado*/
            if(error_a >  M_PI) error_a -= 2*M_PI;
            if(error_a < -M_PI) error_a += 2*M_PI;
            //valores del caculo para el control diferencial
            msg_cmd_vel.angular.z = w_max * (2 / (1 + exp(-error_a/beta)) - 1);
            msg_cmd_vel.linear.x  = v_max * exp(-error_a*error_a/alpha);
            msg_cmd_vel.linear.y  = 0;
         
             
        
        }else{
            aux = 0; //cambia para no ejecutar lo anterior hasta que se vuelva a llamar al callback
            msg_cmd_vel.linear.x  = 0;
            msg_cmd_vel.linear.y  = 0;
            msg_cmd_vel.angular.z = 0;
            //inicializa valores para poder entrar de nuevo a los if y al file
            p=0;
            error_m = 1;
            dif_m = 0;
            
            

        }
    }

	pub_cmd_vel.publish(msg_cmd_vel);
	ros::spinOnce();
	loop.sleep();
    }
    return 0;
}
