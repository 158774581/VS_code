// force_control.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "dynamic_module.h"
#include "torque_generator.h"
#include "joint_module.h"
#include "cartesian_module.h"
#include "robot_module.h"
#include "lib_scara.h"
#include "matrix.h"
#include "math.h"
#include "trajectory_generator.h"

robot_module gRobotMod;

FILE *fp;
FILE *fp1;
FILE *fp2;
//FILE *fp3;
FILE *fp10;
FILE *fp11;
FILE *fp12;
FILE *fp13;
FILE *fp14;

int gSign;

int16 simulate(cartesian_module * m_cart, joint_module* m_joint, kinematics* m_kine, robot_config_module* p_config,dynamic_module* p_dym_m)
{
	matxx deltF;  //deltF = F_real-F_module
	int i	= 0;
	double cart[6] = { 0,0,0 ,0,0,0};
	double joint[6] = { 0,0,0 ,0,0,0 };
	double cart_vel[6] = { 0,0,0 ,0,0,0 };
	double joint_vel[6] = { 0,0,0 ,0,0,0 };

	matxx_malloc(&deltF, 2, 1);
	(*(deltF.point))[0] = 0;
	(*(deltF.point))[1] = 0;

	cartesian_profile* p_cart = &p_dym_m->dynamic_profile.cart;
	cartsian_impedance_control_law*	p_law = &p_dym_m->control_law.cart_law;
	//cartesian_dynamic* p_cdym								= &p_dym_m->dynamic.cart_dynamic;
	joint_profile* p_joint = &p_dym_m->dynamic_profile.joint;

	matxx_copy(&p_dym_m->dynamic.T, &p_cart->tmp21);
	matxx_k_mac(-1, &p_dym_m->dynamic.C, &p_cart->tmp21);

	matxx_k_mac(-1, &p_dym_m->dynamic.F, &p_cart->tmp21);

	matxx_k_mac(-1, &deltF, &p_cart->tmp21);

	//////////////////////////////matxx_inv(&p_dym_m->dynamic.M, &p_cart->tmp22); ///eazy wrong
													   //matxx_printf(&p_dym->M);
	matxx_multiply(&p_cart->tmp22, &p_cart->tmp21, &p_cart->temp21);
	//p_dym->ctl_rate.current_time = 1000000;

	(*(p_joint->joint_acc.point))[0] = (*(p_cart->temp21.point))[0] *MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;
	(*(p_joint->joint_acc.point))[1] = (*(p_cart->temp21.point))[1] *MOTION_MODULE_UNIT_TRANSFORM_RAD_2_DEGREE;

	(*(p_joint->joint_vel.point))[0] += (*(p_joint->joint_acc.point))[0] * p_joint->sample_time;
	(*(p_joint->joint_vel.point))[1] += (*(p_joint->joint_acc.point))[1] * p_joint->sample_time;

	(*(p_joint->joint_pos.point))[0] += (*(p_joint->joint_vel.point))[0] * p_joint->sample_time +\
		0.5*(*(p_joint->joint_acc.point))[0] * pow(p_joint->sample_time, 2);
	(*(p_joint->joint_pos.point))[1] += (*(p_joint->joint_vel.point))[1] * p_joint->sample_time + \
		0.5*(*(p_joint->joint_acc.point))[1] * pow(p_joint->sample_time, 2);

	for (i = 0; i < 2; i++)
	{
		joint[i]					= (*(p_joint->joint_pos.point))[i];
		joint_vel[i]				= (*(p_joint->joint_vel.point))[i];
		//m_joint[i].joint_pos_cmd	= joint[i];
	}
	ScaraFK(m_cart, m_joint, m_kine, p_config, cart, joint);
	ScaraBasicFJ(m_cart, m_joint, m_kine, p_config, cart_vel, joint_vel);

	for (i = 0; i < 2; i++)
	{
		(*(p_cart->cart_pos.point))[i]		= cart[i];
		(*(p_cart->cart_vel.point))[i]		= cart_vel[i];
		//m_cart->cart_pos_cmd[i]				= cart[i];
		//m_cart->cart_vel_cmd[i]				= cart_vel[i];
	}
	fprintf(fp, "%lf    %lf    %lf    %lf    %lf    %lf   %lf\n", cart[0], cart[1], cart_vel[0], cart_vel[1]\
		,(*(p_cart->ax.point))[0], (*(p_cart->ax.point))[1], 2-cart[0]- cart_vel[0]);
	fprintf(fp1, "%lf    %lf    %lf    %lf    %lf	  %lf    %lf    %lf   %lf\n", joint[0], joint[1], joint_vel[0], joint_vel[1],\
		(*(p_joint->joint_acc.point))[0], (*(p_joint->joint_acc.point))[1], (*(p_joint->aq.point))[0], (*(p_joint->aq.point))[1],  - joint[0] - joint_vel[0]);
	fprintf(fp2, "%lf	%lf    %lf	%lf\n", (*(p_dym_m->dynamic.T.point))[0], (*(p_dym_m->dynamic.T.point))[1], (*(p_dym_m->dynamic.CURm.point))[0], (*(p_dym_m->dynamic.CURm.point))[1]);

	return 0;
}

int16 profile_torque(robot_module* p_RobMod,robot_config_module* p_cfg)
{
	extern  calculate_blending_parameters(motion_block* current, motion_block* last, robot_config_module* m_cfg);
	//extern  calculate_motion_parameters(robot_config_module* m_cfg, motion_block* p_motion);

	fopen_s(&fp10, "C:\\Users\\hqi\\Desktop\\joint_cmd.txt", "w+");
	fopen_s(&fp11, "C:\\Users\\hqi\\Desktop\\cart_cmd.txt", "w+");
	fopen_s(&fp13, "C:\\Users\\hqi\\Desktop\\cart_vel_cmd.txt", "w+");
	fopen_s(&fp12, "C:\\Users\\hqi\\Desktop\\joint_vel_cmd.txt", "w+");
	fopen_s(&fp14, "C:\\Users\\hqi\\Desktop\\备用.txt", "w+");

	Uint8 flag = 0;

	Uint8 i;

	double cart[6];
	double joint[6];
	gSign = 0;
	double force_joint[6] = { 0,60,0,0,0,0 };
	double joint_end_vector_1[6] = { 0,0,1,6,7,8 };
	double joint_end_vector_2[6] = { 0,30,1,60,40,20 };
	double cart_end_vector_1[6] = { 1,2,14,44,33,60 };
	double cart_end_vector_0[6] = { 0,2.8,1,50,80,70 };
	double circle_begin[6] = { 2.8 * cos(0), 2.8 * sin(0),1,10,10,20 };
	double circle_aux[6] = { 2.8*pow(2,0.5) / 2, 2.8* pow(2,0.5) / 2,1,35,45,65 };
	double circle_aux_1[6] = { 2.8*cos(3.1415926 / 2.5),2.8* sin(3.1415926 / 2.5),1,40,50,70 };
	double circle_end[6] = { 2.8 * cos(3.1415926 / 3),2.8 * sin(3.1415926 / 3),1,80,70,21 };

	trajectory_module    m_traj;


	// init robot_config
	p_cfg->cart_dim = 6;
	p_cfg->cart_rotate_dim = 3;
	p_cfg->cart_trans_dim = 3;
	p_cfg->joint_dim = 6;

	trajectory_module    *p_traj = &m_traj;


	cartesian_module     *p_cart = &p_RobMod->cart;
	joint_module		 *p_joint = &p_RobMod->joint[0];
	//----------------set kinematics-----------------
	// init trajectory module
	if (InitTrajectoryModule(p_traj, p_cfg, 0) == -2)
		printf("no buffer pointer");

	//------------------ set block 1 -----------------------------
	motion_block* 	p_motion = p_traj->motion_block_buffer;
	motion_block* 	p_next_motion = p_traj->motion_block_buffer + 1;
	motion_block* 	p_last_motion = p_traj->motion_block_buffer + 2;

	// set motion block and its var 
	//------------------ set block 1 -----------------------------
	p_motion->motion_type = TRAJECTORY_MODULE_MOTION_TYPE_P2P;
	//p_motion->jointp2p_method = JOINTP2P_TYPE_TIME_SAME;
	p_motion->cartline_method = CARTLINE_TYPE_AXIS_ANGLE;
	p_motion->blending.type = TRAJECTORY_MODULE_BLENDING_AUTO;
	p_motion->blending.is_need_blending = 0;

	//------------------ set block 2 -----------------------------
	p_next_motion->motion_type = TRAJECTORY_MODULE_MOTION_TYPE_LIN;
	//p_next_motion->jointp2p_method = JOINTP2P_TYPE_TIME_SAME;
	p_next_motion->cartline_method = CARTLINE_TYPE_AXIS_ANGLE;
	p_next_motion->blending.type = TRAJECTORY_MODULE_BLENDING_AUTO;
	p_next_motion->blending.is_need_blending = 0;
	//------------------ set block 3 -----------------------------
	p_last_motion->motion_type = TRAJECTORY_MODULE_MOTION_TYPE_LIN;
	//p_last_motion->jointp2p_method = JOINTP2P_TYPE_TIME_SAME;
	p_last_motion->cartline_method = CARTLINE_TYPE_AXIS_ANGLE;
	p_last_motion->blending.type = TRAJECTORY_MODULE_BLENDING_AUTO;
	p_last_motion->blending.is_need_blending = 0;

	//------------------ calculate block 1 -----------------------------

	ScaraIK(p_cart, p_joint, &p_RobMod->kinematics, p_cfg, circle_end, joint,1);

	for (i = 0; i < 6; i++)
	{

		p_motion->pose.pose_start[i] = circle_begin[i];  // for cart
		p_motion->pose.pose_aux1[i] = circle_aux[i]; // for circle
		p_motion->pose.pose_end[i] = circle_end[i];  //for cart
		if (i == 3) 	matxx_set_element(&p_motion->pose.joint_end_vector, i, 0, &joint[5]);
		else if (i == 5) 	matxx_set_element(&p_motion->pose.joint_end_vector, i, 0, &joint[3]);
		else matxx_set_element(&p_motion->pose.joint_end_vector, i, 0, &joint[i]);
		matxx_set_element(&p_motion->pose.joint_end_vector, i, 0, &joint_end_vector_2[i]);
		matxx_set_element(&p_motion->pose.joint_start_vector, i, 0, &force_joint[i]);
	}
	p_motion->profile.profile_vel = p_motion->prf_cart_rot.profile_vel = 20.0*0.5;
	p_motion->profile.profile_acc = p_motion->prf_cart_rot.profile_acc = 2.0*0.5;
	p_motion->profile.profile_tacc = p_motion->prf_cart_rot.profile_tacc = 0.1;
	p_motion->profile.profile_dec = p_motion->prf_cart_rot.profile_dec = 2.0*0.5;
	p_motion->profile.profile_tdec = p_motion->prf_cart_rot.profile_tdec = 0.1;
	p_motion->profile.profile_sample_time = p_motion->prf_cart_rot.profile_sample_time = 0.001;
	//set profile for joint
	for (i = 0; i < 6; i++)
	{
		p_motion->prf_joint[i].profile_vel = 20.0*0.5;
		p_motion->prf_joint[i].profile_acc = 2.0*0.5;
		p_motion->prf_joint[i].profile_tacc = 0.1;
		p_motion->prf_joint[i].profile_dec = 2.0*0.5;
		p_motion->prf_joint[i].profile_tdec = 0.1;
		p_motion->prf_joint[i].profile_sample_time = 0.001;
	}
	calculate_motion_parameters(p_cfg, p_motion);
	p_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING;
	//pos_vel_acc_printf(p_motion);

	//calculate_blending_parameters(p_next_motion, p_motion, p_config);


	//------------------ calculate block 2 -----------------------------
	ScaraFK(p_cart, p_joint, &p_RobMod->kinematics, p_cfg, cart, joint_end_vector_1);
	ScaraIK(p_cart, p_joint, &p_RobMod->kinematics, p_cfg, circle_end, joint,1);


	for (i = 0; i < 6; i++)
	{
		if (p_motion->motion_type == TRAJECTORY_MODULE_MOTION_TYPE_P2P)
		{
			if (i == 3) p_next_motion->pose.pose_start[i] = cart[5];
			else if (i == 5) p_next_motion->pose.pose_start[i] = cart[3];
			else p_next_motion->pose.pose_start[i] = cart[i];


		}
		else
			p_next_motion->pose.pose_start[i] = circle_end[i];

		p_next_motion->pose.pose_aux1[i] = circle_aux_1[i];
		p_next_motion->pose.pose_end[i] = cart_end_vector_0[i];
		if (i == 3) 	matxx_set_element(&p_next_motion->pose.joint_start_vector, i, 0, &joint[5]);
		else if (i == 5) 	matxx_set_element(&p_next_motion->pose.joint_start_vector, i, 0, &joint[3]);
		else matxx_set_element(&p_next_motion->pose.joint_start_vector, i, 0, &joint[i]);
		//matxx_set_element(&p_next_motion->pose.joint_start_vector, i, 0, &joint_end_vector_1[i]);
		matxx_set_element(&p_next_motion->pose.joint_end_vector, i, 0, &joint_end_vector_2[i]);
	}

	p_next_motion->profile.profile_vel = p_next_motion->prf_cart_rot.profile_vel = 20.0*0.5;
	p_next_motion->profile.profile_acc = p_next_motion->prf_cart_rot.profile_acc = 2.0 * 0.5;
	p_next_motion->profile.profile_tacc = p_next_motion->prf_cart_rot.profile_tacc = 0.1;
	p_next_motion->profile.profile_dec = p_next_motion->prf_cart_rot.profile_dec = 2.0 * 0.5;
	p_next_motion->profile.profile_tdec = p_next_motion->prf_cart_rot.profile_tdec = 0.1;
	p_next_motion->profile.profile_sample_time = p_next_motion->prf_cart_rot.profile_sample_time = 0.001;
	for (i = 0; i < 6; i++)
	{
		p_next_motion->prf_joint[i].profile_vel = 20.0*0.5;
		p_next_motion->prf_joint[i].profile_acc = 2.0 * 0.5;
		p_next_motion->prf_joint[i].profile_tacc = 0.1;
		p_next_motion->prf_joint[i].profile_dec = 2.0 * 0.5;
		p_next_motion->prf_joint[i].profile_tdec = 0.1;
		p_next_motion->prf_joint[i].profile_sample_time = 0.001;
	}


	calculate_motion_parameters(p_cfg, p_next_motion);
	p_next_motion->state = TRAJECTORY_MODULE_MOTION_BLOCK_STATE_RUNNING;
	//pos_vel_acc_printf(p_next_motion);

	calculate_blending_parameters(p_next_motion, p_motion, p_cfg);
	//printf("blending_start_time %d\n------------------------------\n", p_next_motion->blending.blending_start_time);


	//---------------------------- execute ----------------------------------------------
	//fprintf(fp, "blending_start_time %d\n", p_next_motion->blending.blending_start_time);
	//fprintf(fp, "------------------ lin blend lin ---------------------\n");
	printf("%d",p_motion->traj_joint[1].time_series[7]);
	int times;
	if (p_next_motion->blending.is_need_blending == 0)
		times = 1;
	else
		times = 2;

	while (gSign < times||gDymMod.dynamic_profile.current_time<40000)
	{
		if (gSign < 1)
		{
			TrajectoryGenerator(p_traj, p_cart, p_joint, p_cfg, 0);
			if (gSign == 1 && flag == 1)
			{
				//fprintf(fp, "--------------------- motion 2----------------------------\n");
				flag = 0;
			}
			if (gSign == 0 && flag == 0)
			{
				flag = 1;
				//fprintf(fp, "----------------------- motion 1 ------------------------\n");
			}
			//fprintf(fp, "%d  ", p_traj->current_time);
			for (i = 0; i < 6; i++)
				fprintf(fp10, "%lf  ", p_joint[i].joint_pos_cmd);  //for joint
			fprintf(fp10, "\n");

			for (i = 0; i < 6; i++)
				fprintf(fp12, "%lf  ", p_joint[i].joint_vel_cmd);        // for cart
			fprintf(fp12, "\n");

			for (i = 0; i < 6; i++)
				fprintf(fp13, "%lf  ", p_cart->cart_vel_cmd[i]);        // for cart
			fprintf(fp13, "\n");

			//fprintf(fp1, "%d  :  ", p_traj->current_time);
			for (i = 0; i < 6; i++)
				fprintf(fp11, "%lf  ", p_cart->cart_pos_cmd[i]);        // for cart
			fprintf(fp11, "\n");
		}

		//------------- calculate the torque -------------------

		torque_generator(&gDymMod);
		simulate(&p_RobMod->cart, p_RobMod->joint, &p_RobMod->kinematics, p_cfg, &gDymMod);

	}

	//-------------------------------- calculate lin_blend_p2p -----------------------------------------------
	//p_next_motion->motion_type = TRAJECTORY_MODULE_MOTION_TYPE_LIN;
	//p_next_motion->blending.is_need_blending = 1;
	//p_next_motion->blending.type = TRAJECTORY_MODULE_BLENDING_AUTO;



	//---------------------------------- exexute ----------------------------------------------------

	fclose(fp10);
	fclose(fp11);
	fclose(fp13);
	return 0;

}
	

int main0()
{
	fopen_s(&fp, "C:\\Users\\hqi\\Desktop\\impedance_cart.txt", "w+");
	fopen_s(&fp1, "C:\\Users\\hqi\\Desktop\\impedance_joint.txt", "w+");
	fopen_s(&fp2, "C:\\Users\\hqi\\Desktop\\impedance_torque.txt", "w+");
	//fopen_s(&fp3, "C:\\Users\\hqi\\Desktop\\impedance_cur_m.txt", "w+");
	robot_config_module cfg;
	robot_config_module* p_cfg = &cfg;
	robot_module* p_RobMod;
	extern double gTmp;
	extern double gTmp1;

	p_cfg->prm.location_config = 1;
	p_cfg->locat_tmp = 1;
	p_cfg->cart_dim = 6;
	p_cfg->cart_rotate_dim = 3;
	p_cfg->cart_trans_dim = 3;
	p_cfg->joint_dim = 6;
	//gTmp[6] = 0;
	//gTmp1 = 0;

	p_RobMod	= &gRobotMod;
	p_RobMod->kinematics.prm.lenth_of_joint[0] = 0.25;
	p_RobMod->kinematics.prm.lenth_of_joint[1] = 0.25;


	InitDynaimcModule(&gDymMod);


	(*(gDymMod.dynamic_profile.cart.cart_pos.point))[0] = 0.375;
	(*(gDymMod.dynamic_profile.cart.cart_pos.point))[1] = 0.25*sin(3.1415926/3.0);
	(*(gDymMod.dynamic_profile.joint.joint_pos.point))[0] = 0;
	(*(gDymMod.dynamic_profile.joint.joint_pos.point))[1] = 60;
	(*(gDymMod.dynamic_profile.cart.cart_vel.point))[0] = 0;
	(*(gDymMod.dynamic_profile.cart.cart_vel.point))[1] = 0;
	(*(gDymMod.dynamic_profile.joint.joint_vel.point))[0] = 0;
	(*(gDymMod.dynamic_profile.joint.joint_vel.point))[1] = 0;

	profile_torque(p_RobMod,p_cfg);
	//while (gSign<1)
	//{
	//	torque_generator(&gDymMod);
	//	simulate(&p_RobMod->cart, p_RobMod->joint, &p_RobMod->kinematics, p_cfg,&gDymMod);
	//}

	fclose(fp);
	fclose(fp1);
	fclose(fp2);
	//fclose(fp3);
    return 0;
}

void test_inverse()
{
	matxx x, y;
	double data[300] = { 3 ,1, 41 , 10,1, 20, 21 ,22, 3 , 55 ,1, 11 , 11, 10, 10, 1 ,42, 3  ,3 ,1,5 , 1, 0, 0, 1 ,2, 1 , 5 ,1, 1 , 1, 0, 0, 1 ,2, 31,1,6,5,1,5\
		, 3, 55, 1, 11, 11, 10, 10, 1, 1, 10, 20, 21, 22, 3, 55, 10, 10, 1, 42, 3, 3, 1, 5, 1, 0, 0, 1 , 6, 3, 1, 5, 1, 0, 0, 1, 3, 55, 1, 11, 11, 10, 10, 1, 42, 3, 3, 1\
		,1, 11, 10, 10, 1, 1, 1, 20, 201, 22, 3, 55, 10, 10, 1, 42, 3, 3, 1, 5, 1, 0, 0, 1, 6, 3, 1, 5, 1, 0, 0, 1, 3, 55, 1, 11, 11, 10, 10, 1, 42, 3, 3, 1,\
		10, 1, 1, 1, 20, 21, 22, 3, 55, 10, 10, 1, 42, 30, 3, 1, 5, 1, 0, 400, 1, 6, 3, 1, 5, 1, 0, 0, 1, 3, 55, 1, 11, 11, 10, 10, 1, 42, 3, 3, 1 \
		, 12, 11, 10, 10, 1, 1, 1, 20, 201, 22, 3, 55, 10, 10, 1, 42, 3, 3, 1, 5, 1, 0, 0, 1, 6, 3, 1, 5, 1, 0, 0, 1, 3, 55, 1, 11, 11, 10, 10, 1, 42, 3, 3, 1, \
		11, 1, 1, 1, 20, 21, 22, 3, 55, 10, 10, 1, 42, 30, 3, 1, 5, 1, 0, 400, 1, 6, 3, 1, 5, 1, 0, 0, 1, 3, 55, 1, };

	matxx_malloc(&x, 9, 9);
	matxx_malloc(&y, 9, 9);
	matxx_assign(&x, data, 81);

	//matxx_printf(&x);

	//for (i=0;i<10;i++)
//	matxx_inv(&x, &y);


	matxx_printf(&y);


	return;

}