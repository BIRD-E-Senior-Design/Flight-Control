#include <stdio.h>
#include "sensors/flow.h"

//CONSTANTS
#define PROD_ID 0x0
#define DELTA_XL_ADDR 0x03
#define DELTA_YL_ADDR 0x05
#define INV_PROD_ID 0x5F

uint16_t init_buffer1[59] = {
  0x7F00,
  0x61AD,
  0x7F03,
  0x4000,
  0x7F05,
  0x41B3,
  0x43F1,
  0x4514,
  0x5B32,
  0x5F34,
  0x7B08,
  0x7F06,
  0x441B,
  0x40BF,
  0x4E3F,
  0x7F08,
  0x6520,
  0x6A18,
  0x7F09,
  0x4FAF,
  0x5F40,
  0x4880,
  0x4980,
  0x5777,
  0x6078,
  0x6178,
  0x6208,
  0x6350,
  0x7F0A,
  0x4560,
  0x7F00,
  0x4D11,
  0x5580,
  0x741F,
  0x751F,
  0x4A78,
  0x4B78,
  0x4408,
  0x4550,
  0x64FF,
  0x651F,
  0x7F14,
  0x6567,
  0x6608,
  0x6370,
  0x7F15,
  0x4848,
  0x7F07,
  0x410D,
  0x4314,
  0x4B0E,
  0x450F,
  0x4442,
  0x4C80,
  0x7F10,
  0x5B02,
  0x7F07,
  0x4041,
  0x7000
};

uint16_t init_buffer2[17] = {
  0x3244,
  0x7F07,
  0x4040,
  0x7F06,
  0x62F0,
  0x6300,
  0x7F0D,
  0x48C0,
  0x6FD5,
  0x7F00,
  0x5BA0,
  0x4EA8,
  0x5A50,
  0x4080,
  0x7F00,
  0x5A10,
  0x5400
};

flow_fifo_t flow_buffer;


//BUFFER INTERFACE
bool fifo_push_flow(flow_fifo_t* fifo, flow_measurement val) {

}

bool fifo_pop_flow(flow_fifo_t* fifo, flow_measurement* dest) {

}

//wont lie this is some magic number voodoo, same as the ToF there is no documentation or explanation for the init sequence
void init_flow() {
    uint8_t data;
    uint16_t config_data = (0x3a << 8) | 0x5a;

    spi_read_blocking(spi1,PROD_ID,&data,1);
    printf("Prod ID: %d\n",data);
    spi_read_blocking(spi1,INV_PROD_ID,&data,1);
    printf("Inv Prod ID: %d\n",data);

    spi_write16_blocking(spi1,&config_data,1);
    sleep_ms(5);

    for (int i = 0; i < 59; i++) {
        spi_write16_blocking(spi1,init_buffer1,1);
    }
    sleep_ms(10);
    for (int i = 0; i < 17; i++) {
        spi_write16_blocking(spi1,init_buffer2,1);
    }
}   