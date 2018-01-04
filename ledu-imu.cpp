#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <math.h>

#include <iostream>
#include <chrono>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "vector.h"

#define	SPI_CHAN		0
#define SENSOR_SIZE		24

// System constants
#define gyroMeasError 3.14159265358979f * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta

void filterUpdate(float* q, float deltat, float w_x, float w_y, float w_z, float a_x, float a_y, float a_z)
{
	// Local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion derrivative from gyroscopes elements
	float f_1, f_2, f_3; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	// Axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * q[0];
	float halfSEq_2 = 0.5f * q[1];
	float halfSEq_3 = 0.5f * q[2];
	float halfSEq_4 = 0.5f * q[3];
	float twoSEq_1 = 2.0f * q[0];
	float twoSEq_2 = 2.0f * q[1];
	float twoSEq_3 = 2.0f * q[2];

	// Normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
		
	// Compute the objective function and Jacobian
	f_1 = twoSEq_2 * q[3] - twoSEq_1 * q[2] - a_x;
	f_2 = twoSEq_1 * q[1] + twoSEq_3 * q[3] - a_y;
	f_3 = 1.0f - twoSEq_2 * q[1] - twoSEq_3 * q[2] - a_z;
		
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * q[3];
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	// Compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2;
	// Normalise the gradient
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 /= norm;
	SEqHatDot_2 /= norm;
	SEqHatDot_3 /= norm;
	SEqHatDot_4 /= norm;	
	// Compute the quaternion derrivative measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// Compute then integrate the estimated quaternion derrivative
	q[0] += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	q[1] += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	q[2] += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	q[3] += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// Normalise quaternion
	norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		
	q[0] /= norm;
	q[1] /= norm;
	q[2] /= norm;
	q[3] /= norm;
}

int main(int argc, char **argv)
{
	int freqkHz, delayus;
	
	float q[4] = {1,0,0,0};
	vector acc(0,0,0);
    vector gyro(0,0,0);
    vector mag(0,0,0);
    
    //float roll, pitch;
	
	if (argc < 3)
	{
		freqkHz = 1000;
		delayus = 20000;
	}
	else
	{
		sscanf(argv[1], "%d", &freqkHz);
		sscanf(argv[2], "%d", &delayus);
	}

	wiringPiSetup();
	wiringPiSPISetupMode(SPI_CHAN, freqkHz*1000, 1);

	unsigned char empty = 0;
	int emptyCnt = 0;

	// first empty ledu's transmit buffer
	do
	{
		wiringPiSPIDataRW(SPI_CHAN, &empty, 1);
		
		if(empty == 0xff)
			emptyCnt++;
			
		usleep(1000);
			
	}while(emptyCnt < 10);

	unsigned char initData[] = {36, 77, 60, 0, 102, 102};

	wiringPiSPIDataRW(SPI_CHAN, initData, sizeof(initData));

	sleep(1);

	auto start = std::chrono::steady_clock::now();

	for(;;)
	{
		auto last_start = start;
		start = std::chrono::steady_clock::now();
		std::chrono::nanoseconds duration = start - last_start;
		float dt = duration.count() / 1e9;
		
		unsigned char data[SENSOR_SIZE] = {36, 77, 60, 0, 102, 102};

		wiringPiSPIDataRW(SPI_CHAN, data, SENSOR_SIZE);

		if (data[0] != 36)
		{
			for (int i=0; i<SENSOR_SIZE; i++)
				printf("%d ", data[i]);
			
			printf("\n");
			return -1;
		}

		acc[0] = ((int16_t)(data[6] << 8) | data[5]) / 512.0;
		acc[1] = ((int16_t)(data[8] << 8) | data[7]) / 512.0;
		acc[2] = ((int16_t)(data[10] << 8) | data[9]) / 512.0;

		gyro[0] = ((int16_t)(data[12] << 8) | data[11]) / 16.4 * M_PI/180.0;
		gyro[1] = ((int16_t)(data[14] << 8) | data[13]) / 16.4 * M_PI/180.0;
		gyro[2] = ((int16_t)(data[16] << 8) | data[15]) / 16.4 * M_PI/180.0;

		filterUpdate(q, dt, gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2]);

		//printf("%f\t%f\t%f\t%f\n", q[0], q[1], q[2], q[3]);

		quaternion rotation(q[0], q[1], q[2], q[3]);
		
		rotation*= quaternion(0, 1, 0, 0);
		rotation.normalize();
		
		std::cout << rotation.toRotationMatrix();
		std::cout << "  " << acc << "  " << mag << std::endl;

		/*roll  = atan2(2.0 * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * 180.0/M_PI;
		pitch = -asin(2.0 * (q[1] * q[3] - q[0] * q[2])) * 180.0/M_PI;

		printf("Roll: %+09.3f \t Pitch: %+09.3f\n", roll, pitch);*/

		usleep(delayus);
	}

	return 0;
}
