/*
 * BPNN_PID.h
 *
 *  Created on: Mar 7, 2025
 *      Author: PC
 */

#ifndef INC_BPNN_PID_H_
#define INC_BPNN_PID_H_

#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#define I 2
#define H 5
#define O 3


typedef struct INPUT_LAYER_T
{
	float	e[3];
	int		bias;
}INPUT_LAYER_Struct;

typedef struct
{
	float	h[H][I];
	float	h_out[H];
}HIDEN_LAYER_Struct;

typedef struct
{
	float	o[O][H];
	float	o_out[O];
}OUTPUT_LAYER_Struct;

typedef struct
{
	float	u[2];
}PID_Struct;

typedef struct
{
	float	y[2];
	float	beta;
	float	alpha;
	float	delta_o[O];
	float	delta_w_o[O][H];
	float	delta_h[H];
	float	delta_w_h[H][I];
}BACKPROPAGATION_Struct;


typedef struct
{
	INPUT_LAYER_Struct 		In;
	HIDEN_LAYER_Struct 		Hiden;
	OUTPUT_LAYER_Struct		Out;
	PID_Struct				PID;
	BACKPROPAGATION_Struct	BP;
}BPNN_PID_PARAMETER_Struct;

void BPNN_PID_Forward_Propagation(BPNN_PID_PARAMETER_Struct *BPNN_PID,uint8_t AXIS);
void BPNN_PID_Back_Propagation(BPNN_PID_PARAMETER_Struct *BPNN_PID);
float tansig(float in);
float logsig(float in);
int sign(float in);
void BPNN_INIT(BPNN_PID_PARAMETER_Struct *BPNN_PID,uint8_t AXIS);
#endif /* INC_BPNN_PID_H_ */
