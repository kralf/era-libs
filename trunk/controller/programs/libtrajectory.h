/*	Header-file for 
 *      Trayectory Generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    28.5.2007
 */


/**     \file
 *      \brief 
 *      Trajectory Generation
 *
 *      Monotone Cubic Interpolation is used to create
 *      Velocity Profiles in joint space for given via 
 *      points in tool space.
*/



#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H




/*
void trajectory_auto_angle(double via_points[][6], 
			   int number_of_via_points);

*/



int read_scriptfile(char *filename, double via_points[][7], int *number_of_via_points);

/** Monotone Cubic Interpolation:<br>
 *  Main function 
 */
int mci(double via_points[][7],          /**< (input) array containing the via point positions (tool space) */
	//	double via_points_time[],        /**< (input) array containing the time between two via points */
	int number_of_via_points,       /**< (input) number of via points */
	double theta_vel[][6],           /**< (output) array containing the velocities theta_vel[i][j]
					 *    of the joint j at time i*dt.
					 */
	int *number_vel_intervals,      /**< (output) Number of total time intervals, the velocity profile
					 *    constists of.
					 */
	double dt,                       /**< (input) Lenght of time interval of the velocity profile. */
	int kin_err[][3]
	);




/** Monotone Cubic Interpolation:<br>
 *  Precalculation of the slopes at the via points.
 *  The slopes of the first and last point are set
 *  to zero.
 */
void mci_slope(double via_points[][7],       /**< (input) array containing the via point positions (tool space) */
	       int number_of_via_points,    /**< (input) number of via points */
	       double m[][6]                 /**< (output) array containing the slopes at the via points*/
	       );



/** Monotone Cubic Interpolation:<br>
 *  Evaluate Cubic polynomial  
 *  \return Position at t 
 */
double mci_eval(double p_a, /**< (input) position at t=0 */
	       double p_b, /**< (input) position at t=1 */
	       double m_a, /**< (input) slope at t=0 */
	       double m_b, /**< (input) slope at t=1 */
	       double t    /**< (input) Evaluation takes place at t=[0,1] */
	       );



/** Monotone Cubic Interpolation:<br>
 *  Calculates velocity profile
 */
int mci_theta_vel(double via_points[][7],     /**< (input) array containing the via point positions (tool space) */
		  //		  double via_points_time[],   /**< (input) array containing the time between two via points */
		  int number_of_via_points,  /**< (input) number of via points */
		  double m[][6],              /**< (input) array containing the slopes at the via points*/
		  double theta_vel[][6],      /**< (output) array containing the velocities theta_vel[i][j]
					      *    of the joint j at time i*dt.
					      */
		  int *number_vel_intervals, /**< (output) Number of total time intervals, the velocity profile
					      *    constists of.
					      */
		  double dt,                  /**< (input) Lenght of time interval of the velocity profile. */
		  int kin_err[][3]
		  );



#endif
