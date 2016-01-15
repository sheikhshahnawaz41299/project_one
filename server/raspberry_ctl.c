#include <stdio.h>
#include <stdlib.h>
#include "cmd_manager.h"
#include <wiringPi.h>
#include "cmd_define.h"

/* cmd    WIRINGPI_CMD_TYPE    LED_ON */
static int led_on(void* cmd_data, unsigned int data_len, void *p_ret) 
{
	digitalWrite(7, HIGH);
	return 0;
}
static void led_on_info(struct cmd_node_t *cn) {
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
    printf("cmd num in list:%d\n", cn->cnt_in_list);
    printf("cmd desc:led on.\n\n");
}

/* cmd    WIRINGPI_CMD_TYPE    LED_OFF */
static int led_off(void* cmd_data, unsigned int data_len, void *p_ret) 
{
	digitalWrite(7, LOW);
	return 0;
}
static void led_off_info(struct cmd_node_t *cn) {
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
    printf("cmd num in list:%d\n", cn->cnt_in_list);
    printf("cmd desc:led off.\n\n");
}

/* cmd    WIRINGPI_CMD_TYPE    WIRINGPI_INIT */
static int wiringPi_init(void* cmd_data, unsigned int data_len, void *p_ret){
	wiringPiSetup();

	/* led pin mode init */
	pinMode(7, OUTPUT);

	return 0;
}
static void wiringPi_init_info(struct cmd_node_t *cn) {
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
    printf("cmd num in list:%d\n", cn->cnt_in_list);
    printf("cmd desc:wirint pi init.\n\n");
}

/* cmd    WIRINGPI_CMD_TYPE    WIRINGPI_EXIT */
static int wiringPi_exit(void* cmd_data, unsigned int data_len, void *p_ret) {
	return 0;	
}
static void wiringPi_exit_info(struct cmd_node_t *cn) {
    printf("cmd type:%d\ncmd:%d\n", cn->cmd_type, cn->cmd);
    printf("cmd num in list:%d\n", cn->cnt_in_list);
    printf("cmd desc:wirint pi exit.\n\n");
}


/* module init function */
int add_led_module_cmds(void *pdata) {
	cmd_register(WIRINGPI_CMD_TYPE, LED_ON, led_on, led_on_info);
	cmd_register(WIRINGPI_CMD_TYPE, LED_OFF, led_off, led_off_info);
	cmd_register(WIRINGPI_CMD_TYPE, WIRINGPI_INIT, wiringPi_init, wiringPi_init_info);
	cmd_register(WIRINGPI_CMD_TYPE, WIRINGPI_EXIT, wiringPi_exit, wiringPi_exit_info);
	
	return 0;
}
