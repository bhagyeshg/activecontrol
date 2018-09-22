/* UAV Smart Wing Control Program	-	Testing
 *
 * -Bhagyesh Govilkar
 * 
 * Function daq is data acquisition only. PID control occurs in the "control" function. Use estimateLift function to estimate lift and print data to file and to console in printdata.
 * All functions should be called from within the while loop in the main function to avoid recursion.
 *
 * TEST SEQUENCE:
 * Gust response with FP model guesses....
 * Gust response with second guesses....
 * Gust response with no control....
 * */


#include <wiringPi.h> //wiringPi library
#include <wiringPiI2C.h> //I2C library
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>
#include <math.h>
#include <time.h>

uint8_t data[4]; // 4 byte word
uint8_t data2[4]; // 4 byte word
uint8_t data3[4]; // 4 byte word
uint8_t data4[4]; // 4 byte word

unsigned int pres; // raw pressure
double p_out; // pressure in kPa
unsigned int pres2; // raw pressure
double p_out2; // pressure in kPa
unsigned int pres3; // raw pressure
double p_out3; // pressure in kPa
double p_o3l;
unsigned int pres4; // raw pressure
double p_out4; // pressure in kPa
double rho = 1.225; //density in kgm^-3
double airspeed;
double aas;
double ap1;
double ap2;
double ap3;
int i=0;
double L;
double Kp=0.2064; // set proportional gain
//double Kp=1.189;
double Ki=2.965; // set integral gain
//double Ki=15.9;
double Kd=-0.00066872; // set derivative gain
//double Kd=0.02109;
double e;
double el;
double int_e;
double P;
double I;
double D;
double output;
double dt = 0.01;
double dt1;
double dt2;
double w1 = 0.025;
double w2 = 0.025;
double w3 = 0.025;
int fd;
int fd2;
int angle;
int pos = 0;
int counter=0;
double start_time;
struct timespec gettime_now;
double init_time;
uint8_t ch_1 = 0x01; // address for pitot probe reaƒding
uint8_t ch_2 = 0x02; // address for 1st pressure sensor
uint8_t ch_3 = 0x04; // address for 2nd pressure sensor
uint8_t ch_4 = 0x08; // address for 3rd pressure sensor
FILE *fp = NULL;

void main(){ //acquire pressure reading from pitot probe and calculate airspeed
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	init_time = gettime_now.tv_sec+(gettime_now.tv_nsec)/1000000000;
	fp = fopen("data.csv" ,"a");
	fprintf(fp,"\n-----------GUST RESPONSE ACTUAL 2-----------\n");
	fflush(fp);
	if (wiringPiSetup () == -1)  {                   // setup to use Wiring pin numbers
		fprintf (stdout, "oops: %s\n", strerror (errno)) ;
		return 1 ;
	}
	softServoSetup (0, 1, 2, 3, 4, 5, 6, 7) ;
	int fd = wiringPiI2CSetup (0x28) ; // 0x28 I2C device address
	int fd2 = wiringPiI2CSetup (0x77) ; // 0x77 I2C device address
	while(1){
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		start_time = gettime_now.tv_nsec;//record time
		counter+=1;
		daq(fd, fd2);
		estimateLift();
		control();
		printdata();
		clock_gettime(CLOCK_REALTIME, &gettime_now);//record time again
		if(gettime_now.tv_nsec - start_time>0){
			dt = (gettime_now.tv_nsec - start_time)/1000000000;//calculate loop time in seconds
		}

	}
}

void daq(int fd, int fd2){ //acquire data and average 50 consecutive readings from each sensor
	while(i<40){
		wiringPiI2CWrite (fd2, ch_1) ; // select channel
		read(fd, &data, 4); //request 4 byte data from device
		pres = (((int)(data[0] & 0x3f)) << 8) | data[1]; // acquiring pressure data in number of "counts"
		p_out = 1.058*pres-8620 ; //conversion and calibration to give pressure in kPa
		airspeed = sqrt(fabs(2*p_out/rho));
		aas+=airspeed-2.5;
		i+=1;
	}
	aas=aas/40;
	i=0;
	
	while(i<40){
		wiringPiI2CWrite (fd2, ch_2) ; // select channel
		read(fd, &data2, 4); //request 4 byte data from device
		pres2 = (((int)(data2[0] & 0x3f)) << 8) | data2[1]; // acquiring pressure data
		p_out2 = 1.098*pres2-8914 ; //conversion and calibration to give pressure in Pa
		ap1+=p_out2;
		i+=1;
	}
	ap1=ap1/40;
	i=0;
	
	while(i<40){
		wiringPiI2CWrite (fd2, ch_3) ; // select channel
		read(fd, &data3, 4); //request 4 byte data from device
		pres3 = (((int)(data3[0] & 0x3f)) << 8) | data3[1]; // acquiring pressure data
		p_out3 = 0.9036*pres3-7371 ; //conversion and calibration to give pressure in kPa
		ap2+=p_out3;
		i+=1;
	}
	ap2=ap2/40;
	i=0;
	
	while(i<40){
		wiringPiI2CWrite (fd2, ch_4) ; // select channel
		read(fd, &data4, 4); //request 4 byte data from device
		pres4 = (((int)(data4[0] & 0x3f)) << 8) | data4[1]; // acquiring pressure data
		p_out4 = 1.146*pres4-9357 ; //conversion and calibration to give pressure in kPa
		ap3+=p_out4;
		i+=1;
	}
	ap3=ap3/40;
	i=0;
}

void estimateLift(){ //to estimate lift using acquired pressure data
	L=(w1*ap1+w2*ap2+w3*ap3);
}

void control(){ //run PID iteration and actuate
	e=L-3.2;
	P = Kp*e;
	if(output<-1){
		output=-1;
	}
	else if(output>1){
		output=1;
	}
	else{
		int_e += e;
	}
	I = Ki*int_e*dt;
	D = Kd*(e-el)/dt;
	output = (P+I+D);
	el=e;
	angle = -550*output+550;
	softServoWrite(0,angle);
	/*if(counter>=200){ //uncomment for step response
		softServoWrite(0,0);
	}
	else{
		softServoWrite(0,1100);
	}*/
}

void printdata(){ //print data to console and file, set all average values to 0
	//printf("%f ;%d ; %f ; %f ; %f ; %f\n",(double)clock()/CLOCKS_PER_SEC, angle, ap1 , ap2 , ap3, L);
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	printf("%f ; %f ; %d\n", gettime_now.tv_sec+((double)gettime_now.tv_nsec)/1000000000 - init_time, L, angle);
	clock_gettime(CLOCK_REALTIME, &gettime_now);
	fprintf(fp,"%f ;%d ; %f ; %f ; %f ; %f\n",gettime_now.tv_sec+((double)gettime_now.tv_nsec)/1000000000 - init_time, angle, ap1 , ap2 , ap3, L);
	fflush(fp);
	aas=0;
	ap1=0;
	ap2=0;
	ap3=0;
	i=0;
}
