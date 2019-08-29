/*
 * flash.h
 *
 *  Created on: 28 авг. 2019 г.
 *      Author: nb
 */

#ifndef FLASH_H_
#define FLASH_H_

//#include <stdbool.h>
#include <stdint.h>

void flash_lock();
void flash_unlock();
void flash_erase_page(uint32_t address);
void flash_write(uint32_t address,uint16_t data);
uint16_t flash_read(uint32_t address);


#endif /* FLASH_H_ */
