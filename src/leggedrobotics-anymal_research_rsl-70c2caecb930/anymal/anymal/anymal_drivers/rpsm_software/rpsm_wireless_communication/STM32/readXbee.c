/*!
 * @file    readXbee.c
 * @author  Russell Buchanan
 * @date    Oct 31, 2016
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>
#include "../include/rpsm_wireless_communication/readXbee.h"

int xbee_init (char *port, struct termios *tty) {

    int fd=open(port,O_RDWR | O_NOCTTY | O_SYNC | O_NONBLOCK);

    if (fd < 0){
        printf("Failed to open USB device\n");
        return -1;
    }
    else {
        if(tcgetattr(fd, tty)!=0){
            printf("tcgetattr() failed to get USB settings\n");
            return -1;
        } 
        else{
                cfmakeraw(tty);

                // Max speed for S2 Xbees
                cfsetispeed(tty, B57600);
                cfsetospeed(tty, B57600);
                
                // 10ms timeout if 1 byte not read
                tty->c_cc[VMIN]   =  1;
                tty->c_cc[VTIME]  =  1;

                tcflush(fd, TCIOFLUSH);

                if (tcsetattr(fd, TCSANOW, tty) != 0) 
                {
                    printf("Error setting USB settings: %s\n", strerror(errno));
                    return -1;
                }
            }
        }
        
    return fd;
}

int xbee_read_byte(int fd, uint8_t *buffer){

    int n = read(fd,buffer,1);

    if(n==1){
        return 1;
    }
    //printf("Error Reading Byte: %s\n", strerror(errno));
    return 0;
}

void parse_button_state(uint8_t *buffer){

    uint8_t battery = (*buffer       & 0x80) >> 7;

    uint8_t rail24V = (*buffer       & 0x40) >> 6;
    uint8_t rail15V = (*buffer       & 0x20) >> 5;
    uint8_t rail12V = (*buffer       & 0x10) >> 4;
    uint8_t rail5V  = (*buffer       & 0x08) >> 3;

    uint8_t leg0    = (*buffer       & 0x04) >> 2;
    uint8_t leg1    = (*buffer       & 0x02) >> 1;
    uint8_t leg2    = (*buffer       & 0x01);
    uint8_t leg3    = (*(buffer + 1) & 0x04) >> 2;

    uint8_t aux0    = (*(buffer + 1) & 0x02) >> 1;
    uint8_t aux1    = (*(buffer + 1) & 0x01);


    printf("Battery: %x\n", battery);

    printf("rail24V: %x\n", rail24V);
    printf("rail15V: %x\n", rail15V);
    printf("rail12V: %x\n", rail12V);
    printf("rail5V: %x\n", rail5V);

    printf("leg0: %x\n", leg0);
    printf("leg1: %x\n", leg1);
    printf("leg2: %x\n", leg2);
    printf("leg3: %x\n", leg3);

    printf("aux0: %x\n", aux0);
    printf("aux1: %x\n", aux1);

}


int xbee_read (int fd, uint8_t *buffer, int length) {
    //Reads data from Xbee
    //Returns: -1 for corrupted message, 1 for valid message, 0 for no message
    const static uint8_t starting_mark = 0xAA;
    uint8_t byte = 0;
    uint8_t crcLeft = 0;
    uint8_t crcRight = 0;
    uint16_t crc;
    uint8_t i,n;

    n = xbee_read_byte(fd,&byte);
    if(byte == starting_mark){
        n = xbee_read_byte(fd,&byte);
        if(byte == starting_mark){
            n = xbee_read_byte(fd,&byte);
            if(byte == starting_mark){



                buffer[0] = starting_mark;
                buffer[1] = starting_mark;
                buffer[2] = starting_mark;

                //Read Main Message
                for (i=0; i<length-3; i++){
                    n = xbee_read_byte(fd,buffer+i+3);
                }

                // Get CRC read LSBs first then MSBs
                n = xbee_read_byte(fd,&crcRight);
                n = xbee_read_byte(fd,&crcLeft);
                crc = ((uint16_t)crcLeft << 8) | crcRight;

                //Check for valid CRC to confirm valid message
                if (!check_crc(crc,buffer,length)){
                    return -1;
                }

                return 1;

            }
        }
    }

    return 0;
}

int check_crc(uint16_t crc, uint8_t *data, int size){

    uint16_t testcrc = gen_crc16(data, size);

    if (testcrc == crc){
        return 1;
    } else{
        return 0;
    }

}

uint16_t gen_crc16(uint8_t *data, int size)
{
    int bit_flag;

    uint16_t out = 0;
    int bits_read = 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> bits_read) & 1; // item a) work from the least significant bits

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;

    }

    // item b) "push out" the last 16 bits
    int i;
    for (i = 0; i < 16; ++i) {
        bit_flag = out >> 15;
        out <<= 1;
        if(bit_flag)
            out ^= CRC16;
    }

    // item c) reverse the bits
    uint16_t crc = 0;
    i = 0x8000;
    int j = 0x0001;
    for (; i != 0; i >>=1, j <<= 1) {
        if (i & out) crc |= j;
    }

    return crc;
}



int xbee_write (int fd, uint8_t *buffer, int length) {

    int n;
    
    uint16_t crc = gen_crc16(buffer, length);

    n = write(fd, buffer, length);
    n = write(fd, &crc, 2);

    return 1;

}

void xbee_close(int fd){
	close(fd);
}