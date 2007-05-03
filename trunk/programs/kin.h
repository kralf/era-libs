/*	Header-file for 
 *      Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    1.5.2007
 */


typedef struct {
  double theta1;
  double theta2;
  double theta3;
  double theta4;
  double theta6;
} t_theta;

typedef struct {
  double x;
  double y;
  double z;

} t_cartesian;

typedef struct {
  double x;
  double y;
  double z;
  double beta1;
  double beta2;
} t_target;




void inverse_kinematics(t_target* target,t_theta* theta);
void forward_kinematics(t_target* target,t_theta* theta);


void theta_init_start_tiks(t_theta* th);
t_theta theta_tiks_to_rad(t_theta* th);
t_theta theta_rad_to_tiks(t_theta* th);
void target_init_starting_values(t_target* target);

void theta_print_rad(t_theta* th);
void theta_print_tiks(t_theta* th);
void target_print(t_target* th);

double sqr(double x);

//double vector_product(t_cartesian* v1, t_cartesian* v2)
//{ vector_product = v1->x*v2->x + v1->y*v2->y + v1->z*v2->z; };
