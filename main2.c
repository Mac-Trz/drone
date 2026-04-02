//add use of two core 
//add pio monitoring off more than 1 pin (for interfaces,spi,i2c)
//add sampling freq choosing?
//add 

//trigger pin is variable {0,1,2,3}

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <limits.h>
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "pico/stdlib.h"

#define L_IN1 6
#define L_IN2 7
#define L_IN3 8
#define L_IN4 9

#define P_IN1 18
#define P_IN2 19
#define P_IN3 20
#define P_IN4 21

#define TX 16
#define RX 17
#define UART_ID uart0
#define BAUD_RATE 9600
#define ESP_RESET 29
#define ESP_PWR  28
#define COM_SIZE 20
struct motor{
	int in1;
	int in2;
}typedef motor;
enum commands {MOT_ON,MOT_OFF,SET_MOT} typedef commands;
char com_str[COM_SIZE]={0};
char buf[COM_SIZE]={0};
int buf_count=0;
int is_command=0;

int mot_pins[]={L_IN1,L_IN2,L_IN3,L_IN4,P_IN1,P_IN2,P_IN3,P_IN4};
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
	printf("%s",buf);
}


void set_pwm(motor mot,int pwm){
 
    // Find out which PWM slice is connected to GPIO 0 (it's slice 0)
    uint slice_num = pwm_gpio_to_slice_num(0);
 
    // Set period of 4 cycles (0 to 3 inclusive)
    pwm_set_wrap(slice_num, 3);
    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 1);
    // Set the PWM running
    pwm_set_enabled(slice_num, true);
}


void exec_command(){
    char fun_buf[COM_SIZE]={0};
    memcpy(fun_buf,com_str,COM_SIZE);
    printf("exec com\n");
    for(int i=0;i<3;i++){
	    if(fun_buf[i]=='*'){
		printf("got com begg\n");
		int com=0;
		int val=0;
		int opt=0;
		sscanf(&fun_buf[i+1],"%d:%d:%d",&com,&opt,&val);
		if(com==SET_MOT){
			printf("setting mot pwm\n");
			//set_mot_pwm(mot[opt],val);
		}
		if(com==MOT_ON){
			printf("mot on\n");
			//set_mot_pwm(mot[opt],val);
		}
		if(com==MOT_OFF){
			printf("mot off\n");
			//set_mot_pwm(mot[opt],val);
		}
		break;
	   }
    }
}
void setup(){
    for(int i=0;i<sizeof(mot_pins)/sizeof(mot_pins[0]);i++){
	gpio_init(mot_pins[i]);
        gpio_set_dir(mot_pins[i], GPIO_OUT);
        gpio_put(mot_pins[i], 0);
    }
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


}
void turn_motor_on(motor mot){
        gpio_put(mot.in1, 1);
        gpio_put(mot.in2, 0);
}
void motor_brake(motor mot){
        gpio_put(mot.in1, 1);
        gpio_put(mot.in2, 1);
}

int main(){
	sleep_ms(2000);
	puts("init");
	
	stdio_init_all();
	
	// setup gpio
	puts("setup");

	setup();
	motor motors[4];
	int k=0;
        for(int i=0;i<4;i++){
		motors[i].in1=mot_pins[k];
		k++;
		motors[i].in2=mot_pins[k];
		k++;
	}


	while(1){
		    printf("blink\n");

		if(is_command==1){
		    printf("is com\n");

			is_command=0;
			exec_command();
		}
		sleep_ms(1000);
	}
}

