//add use of two core 
//add pio monitoring off more than 1 pin (for interfaces,spi,i2c)
//add sampling freq choosing?
//add 

//trigger pin is variable {0,1,2,3}
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <time.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

//MOTOR
#define L_IN1 6
#define L_IN2 7
#define L_IN3 8
#define L_IN4 9

#define P_IN1 18
#define P_IN2 19
#define P_IN3 20
#define P_IN4 21
#define MOT_SZ 4

//UART
#define TX 16
#define RX 17
#define MY_UART_RX 6
#define UART_ID uart0
#define BAUD_RATE 9600
#define COM_SIZE 20

//ESP 1S
#define ESP_RESET 29
#define ESP_PWR  28

//GYROSCOPE MPU-6500
#define SRC_LEN 1
#define DST_LEN 1
#define I2C i2c0
#define I2C_SDA_PIN 0
#define I2C_SCL_PIN 1

//GYRO reg addreses
#define PWR_MGMT_1_ADDR 107
#define PWR_MGMT_2_ADDR 108
#define ACCEL_XOUT_H 59 //59-64 ACCEL
#define GYRO_XOUT_H 67 //67 to 72 GYRO

//CONNECT ADO PIN TO GND
#define GYRO_I2C_ADDR 0x68

//GYRO reg values
#define PWR_MGMT_1_RESET 0b10000001
#define PWR_MGMT_1_VAL 0b00000001

#define PWR_MGMT_2_VAL 0b00000000

#define PI 3.141592654


struct motor{
	int in1;
	int in2;
	int pwm;
}typedef motor;

enum commands {MOT_ON,MOT_OFF,SET_MOT} typedef commands;
char com_str[COM_SIZE]={0};
char buf[COM_SIZE]={0};
int buf_count=0;
volatile int is_command=0;

volatile int pid_a=0;
volatile int pid_b=0;
volatile int pid_c=0;

int mot_pins[]={L_IN1,L_IN2,L_IN3,L_IN4,P_IN1,P_IN2,P_IN3,P_IN4};


double accel_magnitude(double x,double y,double z){
    return sqrt(x*x + y*y + z*z);
}

double roll(double x,double y,double z){
    return atan2(y,sqrt(x*x + z*z))*(180/PI);
}

double pitch(double x,double y,double z){
    return atan2(-x,sqrt(y*y + z*z))*(180/PI);
}
 
int write_reg(int reg_addr,uint8_t data,int dev_addr){
        uint8_t src[2]={0}; 
        src[0]=reg_addr;
        src[1]=data;
        int ret=i2c_write_blocking(I2C,dev_addr,src,2,false);
        return ret;
}

int read_reg(int reg_addr,uint8_t *data,size_t sz,int dev_addr){
        uint8_t src[1]={0}; 
        src[0]=reg_addr;
        int ret=i2c_write_blocking(I2C,dev_addr,src,1,false);
        if(ret<0)puts("read write error");

        ret=i2c_read_blocking(i2c0,dev_addr,data,sz,false);
        return ret;	
}

void on_uart_rx() {

        while (uart_is_readable(UART_ID) ) {
	    buf[buf_count] = uart_getc(UART_ID);
	    if(buf[buf_count]=='\n'){
	    	memset(com_str,0,COM_SIZE);
		memcpy(com_str,buf,COM_SIZE);
		is_command=1;
		buf_count=0;
		memset(buf,0,COM_SIZE);
		puts("got_command");
		
	    }
	    else{
	    	buf_count++;
	    } 
 	    if(buf_count>=COM_SIZE){
	        memset(buf,0,COM_SIZE);
	        buf_count=0;
	    }
	}
}

void set_pwm(motor mot){
    int slice_num = pwm_gpio_to_slice_num(mot.in1);
    int channel=pwm_gpio_to_channel(mot.in1);
    int pwm=mot.pwm>100?100:mot.pwm;

    pwm_set_chan_level(slice_num, channel, pwm);
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    // Set the PWM running
}



void turn_motor_on(motor* mot,int sz){
	for(int i=0;i<sz;i++){
	        int slice_num = pwm_gpio_to_slice_num(mot[i].in1);
	        int channel=pwm_gpio_to_channel(mot[i].in1);
    		int pwm=mot[i].pwm>100?100:mot[i].pwm;

    		pwm_set_chan_level(slice_num, channel, pwm);
	        pwm_set_enabled(slice_num, true);
	}
}

void turn_motor_off(motor* mot,int sz){
	for(int i=0;i<sz;i++){
	        int slice_num = pwm_gpio_to_slice_num(mot[i].in1);
	        int channel=pwm_gpio_to_channel(mot[i].in1);
    		pwm_set_chan_level(slice_num, channel, 0);
		sleep_ms(1);
	        pwm_set_enabled(slice_num, false);
	}
}

void exec_command(motor* mot,int sz){
    char fun_buf[COM_SIZE]={0};
    memcpy(fun_buf,com_str,COM_SIZE);
    printf("fun buf %s\n",fun_buf);

    printf("exec com\n");
    for(int i=0;i<3;i++){
	    if(fun_buf[i]=='*'){
		printf("got com begg\n");
		int com=0;
		int val=0;
		int opt=0;
		sscanf(&fun_buf[i+1],"%d:%d:%d",&com,&opt,&val);
		if(com==SET_MOT){
			if(opt<sz && opt>=0){
				printf("setting mot pwm\n");
				mot[opt].pwm=val;
				set_pwm(mot[opt]);
			}
		}
		if(com==MOT_ON){
			printf("mot on\n");
			turn_motor_on(mot,sz);
		}
		if(com==MOT_OFF){
			printf("mot off\n");
			turn_motor_off(mot,sz);
		}
		break;
	   }
    }
}
void setup(){
    //motors setup
    for(int i=0;i<sizeof(mot_pins)/sizeof(mot_pins[0]);i++){
	gpio_init(mot_pins[i]);
        if(i%2==1)gpio_set_dir(mot_pins[i], GPIO_OUT);
	if(i%2==0){
 		unsigned int slice_num = pwm_gpio_to_slice_num(mot_pins[i]);
		gpio_set_function(mot_pins[i], GPIO_FUNC_PWM);
 		pwm_set_wrap(slice_num, 99);
	}
    
        gpio_put(mot_pins[i], 0);
    }
    //ESP 1S setup
    gpio_init(ESP_RESET);
    gpio_set_dir(ESP_RESET, GPIO_OUT);
    gpio_put(ESP_RESET, 1);
    gpio_init(ESP_PWR);
    gpio_set_dir(ESP_PWR, GPIO_OUT);
    gpio_put(ESP_PWR, 1);
    
    uart_init(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);

    // Set our data format
    //uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    uart_set_fifo_enabled(UART_ID, true);

    // Set up a RX interrupt
    // We need to set up the handler first
    // Select correct interrupt for the UART we are using
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    // And set up and enable the interrupt handlers
    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    // Now enable the UART to send interrupts - RX only
    uart_set_irq_enables(UART_ID, true, false);
    // Set the TX and RX pins by using the function select on the GPIO
    // Set datasheet for more information on function select
    gpio_set_function(TX, GPIO_FUNC_UART);
    gpio_set_function(RX, GPIO_FUNC_UART);

	//I2C INIT
    i2c_init(i2c0,100000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    printf("sda=%d,scl=%d\n",I2C_SDA_PIN,I2C_SCL_PIN);
    //start measurment

    //gyro init
    if(write_reg(PWR_MGMT_1_ADDR,PWR_MGMT_1_VAL,GYRO_I2C_ADDR)<0)puts("write error");
    if(write_reg(PWR_MGMT_2_ADDR,PWR_MGMT_2_VAL,GYRO_I2C_ADDR)<0)puts("write error");
    if(write_reg(PWR_MGMT_2_ADDR,PWR_MGMT_2_VAL,GYRO_I2C_ADDR)<0)puts("write error");

	//reset gyro
	if(write_reg(PWR_MGMT_1_ADDR,PWR_MGMT_1_RESET,GYRO_I2C_ADDR)<0)puts("write error");

	

        

}

/*
int pid(int* pid_mem,double pitch_corr,double roll_corr,motor *motors,int mot_sz,uint64_t *time){
	printf("pitch:%f\n",pitch);
        printf("roll:%f\n",roll);
	uint64_t now=time_us_64();
	//pid magic
	
	
	motors[FRONT_LEFT]->pwm;
	motors[FRONT_RIGHT]->pwm;
	motors[BACK_LEFT]->pwm;
	motors[BACK_RIGHT]->pwm;

	set_motor(motors[FRONT_LEFT]);
	set_motor(motors[FRONT_RIGHT]);
	set_motor(motors[BACK_LEFT]);
	set_motor(motors[BACK_RIGHT]);
	*time=time_us_64();
}
*/
void motors_test(motor *motors, int sz){
	for(int i=0;i<sz;i++){
		motors[i].pwm=50;
		set_pwm(motors[i]);
	}
	turn_motor_on(motors,sz);
	sleep_ms(4000);
	
	for(int i=0;i<sz;i++){
		turn_motor_off(motors,sz);
	}
}
int main(){
	
	stdio_init_all();
	
	// setup gpio
	puts("setup");

	setup();
	motor motors[MOT_SZ];
	int k=0;
        for(int i=0;i<MOT_SZ;i++){
		motors[i].in1=mot_pins[k];
		k++;
		motors[i].in2=mot_pins[k];
		k++;
		motors[i].pwm=0;
	}

	int a=0,b=0,c=0;

	while(1){
		/*
		if(is_command==1){
		    printf("is com\n");
		    is_command=0;
		    exec_command(motors,MOT_SZ);
		}
		*/
		uint8_t gyro_data[6]={0};
		uint8_t accel_data[6]={0};
		if(read_reg(GYRO_XOUT_H,gyro_data,6,GYRO_I2C_ADDR)<0)puts("read error");
		if(read_reg(ACCEL_XOUT_H,accel_data,6,GYRO_I2C_ADDR)<0)puts("read error");;
		int16_t x=0,y=0,z=0;
		x|=accel_data[0]<<8;
		x|=accel_data[1];
		y|=accel_data[2]<<8;
		y|=accel_data[3];
		z|=accel_data[4]<<8;
		z|=accel_data[5];
		double accel_x=x/(double)16/(double)1000;
		double accel_y=y/(double)16/(double)1000;
		double accel_z=z/(double)16/(double)1000;
		
		printf("accel x:%f y:%f z:%f\n",accel_x,accel_y,accel_z);
		x=0;
		y=0;
		z=0;
		x|=gyro_data[0]<<8;
		x|=gyro_data[1];
		y|=gyro_data[2]<<8;
		y|=gyro_data[3];
		z|=gyro_data[4]<<8;
		z|=gyro_data[5];
		double gyro_x=x/(double)16/(double)1000;
		double gyro_y=y/(double)16/(double)1000;
		double gyro_z=z/(double)16/(double)1000;
		
		printf("gyro x:%f y:%f z:%f\n",gyro_x,gyro_y,gyro_z);
		// printf("y:%f\n",y);
		// printf("z:%f\n",z);
		// printf("pitch:%f\n",pitch(x,y,z));
		// printf("roll:%f\n",roll(x,y,z))s;
	
		//pid(a,b,c,pitch(x,y,z),roll(x,y,z),motors,MOT_SZ);

		puts("");
		sleep_ms(500);
	
	}

}

