/*
 * mySemaphores.h
 *
 *  Created on: May 28, 2018
 *      Author: blake
 */

#ifndef MYSEMAPHORES_H_
#define MYSEMAPHORES_H_

#include "portmacro.h"
#include "projdefs.h"

SemaphoreHandle_t linTaskSemaphore;
BaseType_t linTaskHigherPriorityTaskWoken;

#endif /* MYSEMAPHORES_H_ */
