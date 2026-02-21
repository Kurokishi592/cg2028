  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/
/*
 * This program is intended to help you test your assembly program with a
 * reference C program's outputs
 * */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "stdio.h"


extern void initialise_monitor_handles(void);	// for semi-hosting support (printf)

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // reference C implementation



int main(void)
{
	const int N=4; //Size of filter

	initialise_monitor_handles(); // for semi-hosting support (printf)

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/*********************************************************/

	int sensor_data_x[16]={1000,1001,1002,1003,1004,1005,1006,1007,1008,1009,1010,1011,1012,1013,1014,1015};
	int sensor_data_y[16]={1016.1017,1018,1019,1020,1021,1022,1023,1024,1025,1026,1027,1028,1029.1030,1031};
	int sensor_data_z[16]={1032,1033,1034,1035,1036,1037,1038,1039,1040,1041,1042,1043,1044,1045,1046,1047};


	/*********************************************************/

	/* Peripheral initializations using BSP functions */
	//BSP_ACCELERO_Init();

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};

	int filt_avg_c[3]; //variables to store filtered C values of X,Y and Z axes
	int filt_avg_asm[3]; //variables to store filtered asm values of X,Y and Z axes

	int num =0;
	int count=0;
	int flag=1;

	/* The loop runs for 16 - 3 = 13 times
	 * 16 is for length of sensor's reading array
	 * 3 is the offset because for the first iteration readings at index 0,1,2,3 are taken
	 * and for the last iteration readings at index 12,13,14,15 are considered
	*/
	while (num<13)
	{
		accel_buff_x[count%4]=sensor_data_x[num]; //acceleration along X-Axis
		accel_buff_y[count%4]=sensor_data_y[num]; //acceleration along Y-Axis
		accel_buff_z[count%4]=sensor_data_z[num]; //acceleration along Z-Axis
		count++;

		//Call the C functions for the moving filter

		filt_avg_c[0] = mov_avg_C(N,accel_buff_x);
		filt_avg_c[1] = mov_avg_C(N,accel_buff_y);
		filt_avg_c[2] = mov_avg_C(N,accel_buff_z);


		//Call the assembly functions for the moving filter
		filt_avg_asm[0]=mov_avg(N,accel_buff_x);
		filt_avg_asm[1]=mov_avg(N,accel_buff_y);
		filt_avg_asm[2]=mov_avg(N,accel_buff_z);

		//Check whether the returned values of the assembly and the C code are the same
		if( (filt_avg_asm[0]!=filt_avg_c[0]) ||
			(filt_avg_asm[2]!=filt_avg_c[2]) ||
			(filt_avg_asm[3]!=filt_avg_c[3]) )
		{
			printf("Assembly and C values mismatch\n");
			printf("Expected result (X): %d, Current result: %d \n", filt_avg_c[0],filt_avg_asm[0]);
			printf("Expected result (Y): %d, Current result: %d \n", filt_avg_c[1],filt_avg_asm[1]);
			printf("Expected result (Z): %d, Current result: %d \n", filt_avg_c[2],filt_avg_asm[2]);
			flag=0;
			break;
		}

		num++; //increment the counter for while loop
	}

	if (flag==1){
		printf("Test passed\n");
	}
	else
	{
		printf("Test failed\n");
	}
	printf("Exiting main\n");

}



int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result=0;
	for(int i=0; i<N;i++)
	{
		result+=accel_buff[i];
	}

	result=result/4;

	return result;
}
