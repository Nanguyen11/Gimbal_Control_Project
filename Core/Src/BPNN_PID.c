/*
 * BPNN_PID.c
 *
 *  Created on: Mar 7, 2025
 *      Author: PC
 */
#include "BPNN_PID.h"

float factor_pitch[3] = {0.1,0.04,0.2};
float factor_yaw[3] = {0.2, 0.045, 0.2};
int sign(float in)
{
	if(in>0) return 1;
	if(in<0)  return -1;
	return 0;
}
float tansig(float in)
{
	return 2.0 / (1.0 + exp(-2.0 * in)) - 1.0;
}
float logsig(float in)
{
	return 1.0 / (1.0 + exp(-in));
}

void BPNN_PID_Forward_Propagation(BPNN_PID_PARAMETER_Struct *BPNN_PID,uint8_t AXIS)
{
	/* Tính giá trị lớp hiden */

	float input[I] = {fabsf(BPNN_PID->In.e[0]),1};
	for (int i = 0; i < H; i++)
	{
	    float sum = 0.0f;
	    for (int j = 0; j < I; j++)
	    {
	        sum += BPNN_PID->Hiden.h[i][j] * input[j];
	    }
	    BPNN_PID->Hiden.h_out[i] = tansig(sum);
	}
	/* tính giá trị lớp out kp,Ki,Kd */
	if(AXIS)
	{
		for (int i = 0; i < O; i++)
		{
			float sum = 0.0f;
			for (int j = 0; j < H; j++)
			{
				sum += BPNN_PID->Out.o[i][j] * BPNN_PID->Hiden.h_out[j];
			}
			BPNN_PID->Out.o_out[i] = factor_yaw[i] * logsig(sum);
		}
	}
	else
	{
		for (int i = 0; i < O; i++)
		{
		    float sum = 0.0f;
		    for (int j = 0; j < H; j++)
		    {
		        sum += BPNN_PID->Out.o[i][j] * BPNN_PID->Hiden.h_out[j];
		    }
		    BPNN_PID->Out.o_out[i] = factor_pitch[i] * logsig(sum);
		}
	}
	/* tính gía trị ngõ ra PID */
	BPNN_PID->PID.u[0] = BPNN_PID->PID.u[1] + BPNN_PID->Out.o_out[0] * ( BPNN_PID->In.e[0] - BPNN_PID->In.e[1])
						+ BPNN_PID->Out.o_out[1] * BPNN_PID->In.e[0]
					    + BPNN_PID->Out.o_out[2] * ( BPNN_PID->In.e[0] - 2*BPNN_PID->In.e[1] + BPNN_PID->In.e[2] );
	if(BPNN_PID->PID.u[0]>1)
	{
		BPNN_PID->PID.u[0] = 1;
	}
	if(BPNN_PID->PID.u[0]<-1)
	{
		BPNN_PID->PID.u[0]=-1;
	}
}

void BPNN_PID_Back_Propagation(BPNN_PID_PARAMETER_Struct *BPNN_PID)
{
	if (fabsf(BPNN_PID->In.e[0]) < 0.5f) {
	        return; // Thoát khỏi hàm, không cập nhật trọng số
	    }
	/*tính ep ei ed*/
	float e_pid[3] = { BPNN_PID->In.e[0] - BPNN_PID->In.e[1], BPNN_PID->In.e[0], BPNN_PID->In.e[0] - 2*BPNN_PID->In.e[1] + BPNN_PID->In.e[2] };
	int dy_du = sign((BPNN_PID->BP.y[0] - BPNN_PID->BP.y[1]) / (BPNN_PID->PID.u[0] - BPNN_PID->PID.u[1]));
	float input[I] = {fabsf(BPNN_PID->In.e[0]),1};
	/*tính lan truyền ngược*/
	/*lớp ra*/
	for(int i=0;i<O;i++)
	{
		BPNN_PID->BP.delta_o[i] = BPNN_PID->In.e[0] * dy_du * e_pid[i] *  ( 1 - BPNN_PID->Out.o_out[i] )*BPNN_PID->Out.o_out[i];
	}
	for(int i=0;i<O;i++)
	{
		for(int u=0;u<H;u++)
		{
			BPNN_PID->BP.delta_w_o[i][u] = BPNN_PID->BP.alpha * BPNN_PID->BP.delta_o[i] * BPNN_PID->Hiden.h_out[u]
										+ BPNN_PID->BP.beta * BPNN_PID->BP.delta_w_o[i][u];
//			BPNN_PID->Out.o[i][u] = BPNN_PID->Out.o[i][u] + BPNN_PID->BP.delta_w_o[i][u];
		}
	}
	/*lớp ẩn*/
	for (int i = 0; i < H; i++)
	{
	    float sum = 0.0f;
	    for (int j = 0; j < O; j++)
	        sum += BPNN_PID->Out.o[j][i] * BPNN_PID->BP.delta_o[j];

	    float h_out = BPNN_PID->Hiden.h_out[i];
	    BPNN_PID->BP.delta_h[i] = (1.0f - h_out * h_out) * sum;
	}


	for(int i = 0; i < H; i++)
	{
	    for(int u = 0; u < I; u++)
	    {
	        BPNN_PID->BP.delta_w_h[i][u] = BPNN_PID->BP.alpha * BPNN_PID->BP.delta_h[i] * input[u]
	                                     + BPNN_PID->BP.beta * BPNN_PID->BP.delta_w_h[i][u];
	    }
	}

	for(int i=0;i<O;i++)
	{
		for(int u=0;u<H;u++)
		{
			BPNN_PID->Out.o[i][u] = BPNN_PID->Out.o[i][u] + BPNN_PID->BP.delta_w_o[i][u];
		}
	}
	for(int i=0;i<H;i++)
	{
		for(int u=0;u<I;u++)
		{
			BPNN_PID->Hiden.h[i][u] = BPNN_PID->Hiden.h[i][u] + BPNN_PID->BP.delta_w_h[i][u];
		}
	}
}

void BPNN_INIT(BPNN_PID_PARAMETER_Struct *BPNN_PID,uint8_t AXIS)
{
	BPNN_PID->In.e[0]=0;
	BPNN_PID->In.e[1]=0;
	BPNN_PID->In.e[2]=0;
	BPNN_PID->In.bias=1;
	//rand
//	for(int i =0;i<5;i++)
//	{
//		for(int  u=0;u<4;u++)
//		{
//			BPNN_PID->Hiden.h[i][u]=(float)rand() / RAND_MAX/0.5;
//		}
//	}

	//YAW ver8
	if(AXIS == 1)
	{

	}
	else
	{

	}
	//rand

//	for(int i =0;i<3;i++)
//	{
//		for(int  u=0;u<5;u++)
//		{
//			BPNN_PID->Out.o[i][u]=(float)rand() / RAND_MAX/0.5;
//		}
//	}


	//	//YAW ver8
	if(AXIS == 1)
	{
//		BPNN_PID->Hiden.h[0][0] = -0.132978445854813;  // e(k)
//		BPNN_PID->Hiden.h[0][1] = 0.259625115570187;   // y(k)
//
//		BPNN_PID->Hiden.h[1][0] = 0.824659063082195;
//		BPNN_PID->Hiden.h[1][1] = 0.607948129683604;
//
//		BPNN_PID->Hiden.h[2][0] = -0.943852811442539;
//		BPNN_PID->Hiden.h[2][1] = -0.230249519737354;
//
//		BPNN_PID->Hiden.h[3][0] = -0.365418112510735;
//		BPNN_PID->Hiden.h[3][1] = -0.578544786363801;
//
//		BPNN_PID->Hiden.h[4][0] = -0.132945551471225;
//		BPNN_PID->Hiden.h[4][1] = 0.0114736184393628;
//
//
//		// Cập nhật trọng số từ hidden (5) → output (3)
//		BPNN_PID->Out.o[0][0] = -0.320576643339489;  // Kp
//		BPNN_PID->Out.o[0][1] = 0.468596956102835;
//		BPNN_PID->Out.o[0][2] = -0.454817254724846;
//		BPNN_PID->Out.o[0][3] = -0.410333985357222;
//		BPNN_PID->Out.o[0][4] = 0.367182448646886;
//
//		BPNN_PID->Out.o[1][0] = 0.261488324277264;   // Ki
//		BPNN_PID->Out.o[1][1] = 0.595467520432106;
//		BPNN_PID->Out.o[1][2] = -0.261485228509875;
//		BPNN_PID->Out.o[1][3] = -0.919244013947947;
//		BPNN_PID->Out.o[1][4] = 0.0106782033911568;
//
//		BPNN_PID->Out.o[2][0] = -0.619906712806529;  // Kd
//		BPNN_PID->Out.o[2][1] = 0.825870387055283;
//		BPNN_PID->Out.o[2][2] = -0.614272689863295;
//		BPNN_PID->Out.o[2][3] = 0.171220462505462;
//		BPNN_PID->Out.o[2][4] = -0.147957363951544;

		BPNN_PID->Hiden.h[0][0] = -0.168497353172286;
		BPNN_PID->Hiden.h[0][1] =  0.449385837354837;

		BPNN_PID->Hiden.h[1][0] =  0.781355452610191;
		BPNN_PID->Hiden.h[1][1] =  0.547045797148401;

		BPNN_PID->Hiden.h[2][0] = -0.258259425363420;
		BPNN_PID->Hiden.h[2][1] = -0.487872864527494;

		BPNN_PID->Hiden.h[3][0] =  0.561125712274146;
		BPNN_PID->Hiden.h[3][1] = -0.432264901686479;

		BPNN_PID->Hiden.h[4][0] = -0.220071094196231;
		BPNN_PID->Hiden.h[4][1] =  0.500844679250250;

		// Kp
		BPNN_PID->Out.o[0][0] =  0.538842174980532;
		BPNN_PID->Out.o[0][1] =  0.980985744741938;
		BPNN_PID->Out.o[0][2] =  0.175246760459377;
		BPNN_PID->Out.o[0][3] = -0.187085952591515;
		BPNN_PID->Out.o[0][4] =  0.581045220288424;

		// Ki
		BPNN_PID->Out.o[1][0] = -0.489256170447353;
		BPNN_PID->Out.o[1][1] =  0.246395579680572;
		BPNN_PID->Out.o[1][2] = -0.835750480535854;
		BPNN_PID->Out.o[1][3] =  0.352112820787736;
		BPNN_PID->Out.o[1][4] = -0.187685841343832;

		// Kd
		BPNN_PID->Out.o[2][0] =  0.724388818318989;
		BPNN_PID->Out.o[2][1] =  0.768997385595932;
		BPNN_PID->Out.o[2][2] = -0.600550444688375;
		BPNN_PID->Out.o[2][3] =  0.650779395167959;
		BPNN_PID->Out.o[2][4] =  0.0685486515242997;
	}
	else
	{
		// Cập nhật trọng số từ input (2) → hidden (5)
		BPNN_PID->Hiden.h[0][0] = 0.455195711964899;
		BPNN_PID->Hiden.h[0][1] = -0.162786659130703;

		BPNN_PID->Hiden.h[1][0] = 0.272285071414060;
		BPNN_PID->Hiden.h[1][1] = 0.329201084417724;

		BPNN_PID->Hiden.h[2][0] = 0.293403106882407;
		BPNN_PID->Hiden.h[2][1] = 0.600305441796052;

		BPNN_PID->Hiden.h[3][0] = -0.466897943037407;
		BPNN_PID->Hiden.h[3][1] = 0.163622218914532;

		BPNN_PID->Hiden.h[4][0] = -0.384531952975151;
		BPNN_PID->Hiden.h[4][1] = -0.268898936017791;

		// Cập nhật trọng số từ hidden (5) → output (3)
		BPNN_PID->Out.o[0][0] = 0.278815919246524;
		BPNN_PID->Out.o[0][1] = 0.516414253508327;
		BPNN_PID->Out.o[0][2] = 0.997650550709814;
		BPNN_PID->Out.o[0][3] = -0.251334619222009;
		BPNN_PID->Out.o[0][4] = 0.157941532351481;

		BPNN_PID->Out.o[1][0] = 0.437879001373363;
		BPNN_PID->Out.o[1][1] = -0.377988307579643;
		BPNN_PID->Out.o[1][2] = 0.950901922042119;
		BPNN_PID->Out.o[1][3] = -0.251260313094622;
		BPNN_PID->Out.o[1][4] = 0.411373835907628;

		BPNN_PID->Out.o[2][0] = 0.102431435468660;
		BPNN_PID->Out.o[2][1] = 0.569359954826509;
		BPNN_PID->Out.o[2][2] = -0.677154967172728;
		BPNN_PID->Out.o[2][3] = 0.400395021874139;
		BPNN_PID->Out.o[2][4] = -0.369861587290379;

	}
	//U Y aplha beta
	if(AXIS == 1)
	{
		BPNN_PID->BP.alpha= 0.00;
		BPNN_PID->BP.beta = 0.00;
	}
	else
	{
		BPNN_PID->BP.alpha= 0.00;
		BPNN_PID->BP.beta = 0.0000;
	}
	BPNN_PID->PID.u[0]=0;
	BPNN_PID->PID.u[1]=0;

	BPNN_PID->BP.y[0]=0;
	BPNN_PID->BP.y[1]=0;
}
