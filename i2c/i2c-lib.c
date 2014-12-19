/*
 * File:   i2c-lib.c   for Microchip PIC XC8
 */

#include <stdio.h>
#include <stdlib.h>

#include <xc.h>
#include "i2c-lib.h"

#ifndef _XTAL_FREQ
    /* 例：4MHzの場合、4000000 をセットする */
    #define _XTAL_FREQ 20000000
#endif

// I2Cバスを有効化
void i2c_enable(void)
{
    SSPSTAT = 0b00000000;      // I2C 400kHz
    SSPADD = 29;               // I2Cバス Baud rate,  48MHz/((SSP1ADD + 1)*4) = 400kHz
    SSPCON1 = 0b00101000;      // I2C有効, Master Mode
}

// I2Cバスを無効化
void i2c_disable(void){
    SSPCON1 = 0b00001000;      // I2C無効, Master Mode
}

// I2C書き込みサイクルの開始（Start Conditionの発行）
void i2c_start(void){
    SSPCON2bits.SEN = 1;       //  Start Condition Enabled bit
    i2c_wait();
}

// I2C書き込みサイクルの開始（Repeat Start Conditionの発行）
void i2c_repeat_start(void){
    SSPCON2bits.RSEN = 1;      //  Start Condition Enabled bit
    i2c_wait();
}

// I2C書き込みサイクルの終了（Stop Conditionの発行）
void i2c_stop(void){
    SSPCON2bits.PEN = 1;       // Stop Condition Enable bit
    i2c_wait();
}

// I2C通信がビジー状態を脱するまで待つ
void i2c_wait(void){
    while ( ( SSPCON2 & 0x1F ) || ( SSPSTAT & 0x04 ) );

}

// I2Cバスにデータを送信（1バイト分）
void i2c_send_byte(const unsigned char data){
    SSPBUF = data;
    i2c_wait();
}

//I2Cバスからデータ受信
// ack=1 : 受信後ACKを送信し、次のデータを送るようスレーブデバイスに指示
// ack=0 : 受信後NO_ACKを送信し、これ以上受信しないことをスレーブデバイスに指示
unsigned char i2c_read_byte(const char ack){
    SSPCON2bits.RCEN = 1;
    i2c_wait();
    unsigned char data = SSPBUF;
    i2c_wait();

    if(ack) SSPCON2bits.ACKDT = 0;     // ACK
    else SSPCON2bits.ACKDT = 1;        // NO_ACK

    SSPCON2bits.ACKEN = 1;

    i2c_wait();
    return data;
}

