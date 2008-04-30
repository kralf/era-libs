
typedef struct {
  double theta_1;
  double theta_2;
  double theta_3;
  double theta_4;
  double theta_6;
} t_theta;

typedef struct {
  double x;
  double y;
  double z;
} t_cartesian;

void inverse_kinematics(t_cartesian* target_pos, double* beta_1, double* beta_2,  
			t_theta* theta);


//double vector_product(t_cartesian* v1, t_cartesian* v2)
//{ vector_product = v1->x*v2->x + v1->y*v2->y + v1->z*v2->z; };
