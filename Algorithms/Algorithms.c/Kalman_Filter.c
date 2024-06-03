#include "Kalman_Filter.h"

float A[DIM_N][DIM_N] = {[0][0]=1,[0][1]=SAMPLE_PERIOD,[1][0]=0,[1][1]=0};
float B[DIM_N][DIM_M] = {0};	
float H[DIM_Q][DIM_N] = {[0][0]=1,[1][1]=0};
float Q[DIM_N][DIM_N] = {[0][0]=0.0001f,[1][1]=0.0001f};
float R[DIM_Q][DIM_Q] = {[0][0]=0.01f,[1][1]=0.0002f};
float Xplus[DIM_N][1] = {[0][0]=0.0001f,[1][0]=0.0001f};
float Xminus[DIM_N][1] = {0};
float AXminus_B[DIM_N][1] = {0};
float Pplus[DIM_N][DIM_N] = {[0][0]=0.01f,[1][1]=0.01f};
float Pminus[DIM_N][DIM_N] = {0};
float Y[DIM_Q][1] = {0};
float K[DIM_N][DIM_N] = {0};
float I[DIM_N][DIM_N] = {[0][0]=1,[1][1]=1};

float AT[DIM_N][DIM_N] = {0};
float HT[DIM_N][DIM_Q] = {0};
float APplus[DIM_N][DIM_N] = {0};
float APplusAT[DIM_N][DIM_N] = {0};
float APplusAT_Q[DIM_N][DIM_N] = {0};
float PminusHT[DIM_N][DIM_Q] = {0};
float HPminus[DIM_Q][DIM_N] = {0};
float HPminusHT[DIM_Q][DIM_Q] = {0};
float HPminusHT_R[DIM_Q][DIM_Q] = {0};
float Inv_HPminusHT_R[DIM_Q][DIM_Q] = {0};
float HXminus[DIM_Q][1] = {0};
float Y_HXminus[DIM_Q][1] = {0};
float K_Y_HXminus[DIM_N][1] = {0};
float KH[DIM_N][DIM_N] = {0};
float I_KH[DIM_N][DIM_N] = {0};

arm_matrix_instance_f32 A_Instance;
arm_matrix_instance_f32 B_Instance;
arm_matrix_instance_f32 H_Instance; 
arm_matrix_instance_f32 Q_Instance;
arm_matrix_instance_f32 R_Instance;
arm_matrix_instance_f32 Xplus_Instance;
arm_matrix_instance_f32 Xminus_Instance;
arm_matrix_instance_f32 AXminus_B_Instance;
arm_matrix_instance_f32 Pplus_Instance;
arm_matrix_instance_f32 Pminus_Instance;
arm_matrix_instance_f32 Y_Instance;
arm_matrix_instance_f32 K_Instance;
arm_matrix_instance_f32 I_Instance;
arm_matrix_instance_f32 AT_Instance;
arm_matrix_instance_f32 HT_Instance;
arm_matrix_instance_f32 APplus_Instance;
arm_matrix_instance_f32 APplusAT_Instance;
arm_matrix_instance_f32 PminusHT_Instance;
arm_matrix_instance_f32 HPminus_Instance;
arm_matrix_instance_f32 HPminusHT_Instance;
arm_matrix_instance_f32 HPminusHT_R_Instance;
arm_matrix_instance_f32 Inv_HPminusHT_R_Instance;
arm_matrix_instance_f32 HXminus_Instance;
arm_matrix_instance_f32 Y_HXminus_Instance;
arm_matrix_instance_f32 K_Y_HXminus_Instance;
arm_matrix_instance_f32 KH_Instance;
arm_matrix_instance_f32 I_KH_Instance;

void Kalman_Filter_Init(void)
{
	arm_mat_init_f32(&A_Instance,DIM_N,DIM_N,&A[0][0]);
	arm_mat_init_f32(&B_Instance,DIM_N,DIM_M,&B[0][0]);
	arm_mat_init_f32(&H_Instance,DIM_Q,DIM_N,&H[0][0]);
	arm_mat_init_f32(&Q_Instance,DIM_N,DIM_N,&Q[0][0]);
	arm_mat_init_f32(&R_Instance,DIM_Q,DIM_Q,&R[0][0]);
	arm_mat_init_f32(&Xplus_Instance,DIM_N,1,&Xplus[0][0]);
	arm_mat_init_f32(&Xminus_Instance,DIM_N,1,&Xminus[0][0]);
	arm_mat_init_f32(&AXminus_B_Instance,DIM_N,1,&AXminus_B[0][0]);
	arm_mat_init_f32(&Pplus_Instance,DIM_N,DIM_N,&Pplus[0][0]);
	arm_mat_init_f32(&Pminus_Instance,DIM_N,DIM_N,&Pminus[0][0]);
	arm_mat_init_f32(&K_Instance,DIM_N,DIM_N,&K[0][0]);
	arm_mat_init_f32(&I_Instance,DIM_N,DIM_N,&I[0][0]);
	arm_mat_init_f32(&AT_Instance,DIM_N,DIM_N,&AT[0][0]);
	arm_mat_init_f32(&HT_Instance,DIM_N,DIM_Q,&HT[0][0]);
	arm_mat_init_f32(&APplus_Instance,DIM_N,DIM_N,&APplus[0][0]);
	arm_mat_init_f32(&APplusAT_Instance,DIM_N,DIM_N,&APplusAT[0][0]);
	arm_mat_init_f32(&PminusHT_Instance,DIM_N,DIM_Q,&PminusHT[0][0]);
	arm_mat_init_f32(&HPminus_Instance,DIM_Q,DIM_N,&HPminus[0][0]);
	arm_mat_init_f32(&HPminusHT_Instance,DIM_Q,DIM_Q,&HPminusHT[0][0]);
	arm_mat_init_f32(&HPminusHT_R_Instance,DIM_Q,DIM_Q,&HPminusHT_R[0][0]);
	arm_mat_init_f32(&Inv_HPminusHT_R_Instance,DIM_Q,DIM_Q,&Inv_HPminusHT_R[0][0]);
	arm_mat_init_f32(&HXminus_Instance,DIM_Q,1,&HXminus[0][0]);
	arm_mat_init_f32(&Y_HXminus_Instance,DIM_Q,1,&Y_HXminus[0][0]);
	arm_mat_init_f32(&K_Y_HXminus_Instance,DIM_N,1,&K_Y_HXminus[0][0]);
	arm_mat_init_f32(&KH_Instance,DIM_N,DIM_N,&KH[0][0]);
	arm_mat_init_f32(&I_KH_Instance,DIM_N,DIM_N,&I_KH[0][0]);

	arm_mat_trans_f32(&A_Instance,&AT_Instance);
	arm_mat_trans_f32(&H_Instance,&HT_Instance);
}

void Kalman_Filter_Update(void)
{
	Y[0][0] = Chassis.Chassis_Coord.Forward_Speed;
	B[1][0] = Chassis.Chassis_Coord.Forward_Acceleration;
	//Y[1][0] = Chassis.Chassis_Coord.Forward_Acceleration;
	arm_mat_init_f32(&Y_Instance,DIM_Q,1,&Y[0][0]);

	arm_mat_mult_f32(&A_Instance,&Xplus_Instance,&Xminus_Instance);
	arm_mat_add_f32(&Xminus_Instance,&B_Instance,&AXminus_B_Instance);
	arm_mat_mult_f32(&A_Instance,&Pplus_Instance,&APplus_Instance);
	arm_mat_mult_f32(&APplus_Instance,&AT_Instance,&APplusAT_Instance);
	arm_mat_add_f32(&APplusAT_Instance,&Q_Instance,&Pminus_Instance);
	arm_mat_mult_f32(&Pminus_Instance,&HT_Instance,&PminusHT_Instance);
	arm_mat_mult_f32(&H_Instance,&PminusHT_Instance,&HPminusHT_Instance);
	arm_mat_add_f32(&HPminusHT_Instance,&R_Instance,&HPminusHT_R_Instance);
	arm_mat_inverse_f32(&HPminusHT_R_Instance,&Inv_HPminusHT_R_Instance);
	arm_mat_mult_f32(&PminusHT_Instance,&Inv_HPminusHT_R_Instance,&K_Instance);
	arm_mat_mult_f32(&H_Instance,&AXminus_B_Instance,&HXminus_Instance);
	arm_mat_sub_f32(&Y_Instance,&HXminus_Instance,&Y_HXminus_Instance);
	arm_mat_mult_f32(&K_Instance,&Y_HXminus_Instance,&K_Y_HXminus_Instance);
	arm_mat_add_f32(&AXminus_B_Instance,&K_Y_HXminus_Instance,&Xplus_Instance);
	arm_mat_mult_f32(&K_Instance,&H_Instance,&KH_Instance);
	arm_mat_sub_f32(&I_Instance,&KH_Instance,&I_KH_Instance);
	arm_mat_mult_f32(&I_KH_Instance,&Pminus_Instance,&Pplus_Instance);
	
	Chassis.Chassis_Coord.Forward_Speed_KF = Xplus[0][0];
}

float First_Order_Kalman_Filter(Kalman_Filter_t *KF, float Measurement)
{
	KF->Current_P = KF->Prev_P + KF->Q;
	KF->K = KF->Current_P / (KF->Current_P + KF->R);
	KF->Output = KF->Output + KF->K*(Measurement - KF->Output);
	KF->Prev_P = (1 - KF->K)*KF->Current_P;
	
	return KF->Output;
}
