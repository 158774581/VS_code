
#include "motion_planning_3rd.h"
#include "stdio.h"





//extern int16 motion_planning1(MOTION_PLANNING_NON_ZERO_PRM* input, MOTION_PLANNING_3RD* m_mp3rd, motion_trajectory_nonzero* motion_tra, double sample_time);
void main_nonzero()
{
	double sample_time = 0.001;
	MOTION_PLANNING_NON_ZERO_PRM input;
	MOTION_PLANNING_3RD motion_3rd;
	motion_trajectory_nonzero motion_tra;
	input.pos = 100;
	input.spd_max = 100;
	input.acc_max = 25;
	input.t_acc = 0.1;
	input.max_itera_time = 30;
	input.pos = -1;
	input.vs = -100;
	input.scale = 1.0;
	input.ve = 10;
	input.secant_eps = 0.001;

	motion_planning_nonzero(&input, &motion_3rd, &motion_tra, sample_time);

	return;
}
