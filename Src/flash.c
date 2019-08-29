/*
 * flash.c
 *
 *  Created on: 28 авг. 2019 г.
 *      Author: nb
 */


#include "flash.h"
#include "stm32f103xb.h"

/*
    st_address = FLASH_BASE + p * 1024;
           flash_unlock();
           flash_erase_page(st_address);
           flash_lock();


   st_address = FLASH_BASE + p * 1024;
   flash_unlock();
   uint16_t tmp;
   for(tmp=0;tmp<1024;tmp+=4)
           flash_write(st_address+tmp,0x54534554);

   flash_lock();


*/

uint8_t flash_ready(void) {
        return !(FLASH->SR & FLASH_SR_BSY);
}



void flash_unlock(void) {
          FLASH->KEYR = FLASH_KEY1;
          FLASH->KEYR = FLASH_KEY2;
}

void flash_lock() {
        FLASH->CR |= FLASH_CR_LOCK;
}

void flash_erase_page(uint32_t address) {
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR|= FLASH_CR_STRT;
    while(!flash_ready())
        ;
    FLASH->CR&= ~FLASH_CR_PER;
}

void flash_write(uint32_t address, uint16_t data) {

        FLASH->CR |= FLASH_CR_PG;
        while(!flash_ready()) {};
    *(__IO uint16_t*)address = data;
        while(!flash_ready()) {};
/*
        address+=2;
        data>>=16;
    *(__IO uint16_t*)address = (uint16_t)data;
        while(!flash_ready())
                ;
*/
    FLASH->CR &= ~(FLASH_CR_PG);

}


uint16_t flash_read(uint32_t address) {
        return (*(__IO uint16_t*) address);
}


