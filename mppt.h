
/*
 * mppt.h
 *
 *  Created on: Dec 20, 2024
 *      Author: BARUN
 */

#ifndef SRC_BOP_MPPT_MPPT_H_
#define SRC_BOP_MPPT_MPPT_H_

#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"

#define BDZ_MPPT 1

extern float mppt_inputV, mppt_inputA;
extern float mppt_batV, mppt_batA, mppt_batSOC;
extern float mppt_batCC_Set, mppt_batCV_Set, mppt_batCC_Read, mppt_batCV_Read;
extern float mppt_loadV, mppt_loadA;

void set_MPPT_CC(void);
void set_MPPT_CV(void);
uint16_t read_MPPT_CC(void);
uint16_t read_MPPT_CV(void);
uint16_t read_MPPT_INPUT_V(void);
uint16_t read_MPPT_INP_A(void);
uint16_t read_MPPT_BAT_V(void);
uint16_t read_MPPT_BAT_A(void);
uint16_t read_MPPT_BAT_SOC(void);
uint16_t read_MPPT_LOAD_V(void);
uint16_t read_MPPT_LOAD_A(void);

#endif /* SRC_BOP_MPPT_MPPT_H_ */
