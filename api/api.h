/*
 * api.h
 *
 *  Created on: Jun 5, 2021
 *      Author: onias
 */

#ifndef API_API_H_
#define API_API_H_

#include "main.h"
#include "states.h"
#include "string.h"
#include <stdbool.h>
#include <math.h>

#define API_TRUE 1U
#define API_FALSE 0U

#define API_LOW 0U
#define API_HIGH 1U

#define LOW 0U
#define HIGH 1U

#define max(a,b) a>=b? a : b
#define min(a,b) a<=b? a : b

#define container_of_c(ptr, type, member) ({                      \
        const typeof( ((type *)0)->member ) *__mptr = (ptr);     \
        (type *)( (char *)__mptr - offsetof(type,member) );      \
        })

#define container_of_cpp(ptr, type, member) (type*)((char*)(ptr) - offsetof(type, member))


/*
 * how to add a new peripheral to the code:
 * - Create a folder that represents the high level object being created (i.e. imu, eeprom etc)
 * - Inside such folder, create generic .c and .h files that represent all generic attributes of such object
 * - Inside that header file, include "api.h" and "states.h", they hold not only the error functions but also the possible states that peripheral can have
 * 	-- Create a struct that holds all the already mentioned parameters as well as the state
 * 	-- implement in the .h and .c all high level functions
 * - If justified, create specific ICs libraries (.c and .h files) and create a second struct specific for that type of IC
 * 	-- This specific struct shall inherit the functions and parameters of the higher level struct, as one of the parameters of the second struct is the first.
 * 	-- To add polymorphism is a bit trickier and involves created a struct of pointers to functions in order that the high level function can call the lower level
 * 	   function if the parameter is of such type
 *
 * 	example:
 * 		TODO: WRITE DOWN EXAMPLE ...
 *
 * */

/*
 *structure to represent a gpio pin. Params are pin number and port number
 * */
typedef struct {
	uint16_t gpio_pin;
	GPIO_TypeDef *gpio_port;
} io_pin_t;

typedef struct {
	TIM_HandleTypeDef *htim;
	uint32_t Channel;
	io_pin_t *pin;
} pwm_t;

typedef enum{
	IRQ_MODE_RISING_FALLING,
	IRQ_MODE_RISING,
	IRQ_MODE_FALLING
} irq_mode_e;

typedef struct {
	io_pin_t *irqPin;
	void (*irq)(io_pin_t * const irqPin);
	void *obj;
	irq_mode_e irqMode;
	uint8_t value;
} irq_t;

typedef io_pin_t irqMap_t;


typedef struct {
	imu_status_e 			imu_state;
	eeprom_status_e 		eeprom_state;
	display_7seg_status_e 	display_7seg_state;
} bsp_status_t;



#endif /* API_API_H_ */
