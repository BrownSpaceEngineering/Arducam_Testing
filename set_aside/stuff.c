// TODO: There's a lot of important stuff in main() (for instance, Arducam_bus_detect) that we have neglected...

void singleCapture(int CS);
void ArduCAM_Init();
void set_format(unsigned char fmt);
void flush_fifo(int CS);
void start_capture(int CS);
unsigned char get_bit(unsigned char addr, unsigned char bit, int CS);
unsigned int read_fifo_length(int CS);
void set_fifo_burst();
void write_reg(unsigned char addr, unsigned char data, int CS);
unsigned char bus_write(int address,int value,int CS);
unsigned char read_reg(unsigned char addr,int CS);
unsigned char bus_read(int address,int CS);
void CS_HIGH(int CS);
void CS_LOW(int CS);
void delay_us(int micros);
unsigned long get_microsecond_timestamp();
void digitalWrite(int pin, int val);
char spiSendReceive(char send);
void pioInit();
struct sensor_reg {
    unsigned int reg;
    unsigned int val;
};
int wrSensorRegs8_8(const struct sensor_reg reglist[]);
unsigned char wrSensorReg8_8(int regID, int regDat);
void sccb_bus_start(void);
unsigned char sccb_bus_write_byte(unsigned char data);
void pinMode(int pin, int function);


// Constants
#define ARDUCHIP_TRIG      		0x41  //Trigger source
#define CAP_DONE_MASK      		0x08
#define CAM_CS1                 17
#define FIFO_SIZE1				0x42
#define FIFO_SIZE2				0x43
#define FIFO_SIZE3				0x44
#define BMP 	    0
#define JPEG	    1
#define RAW         2
#define JPEG_BUF_SIZE   2*1024*1024 //2M
#define OV2640  	5
#define OV2640_160x120 		0	//160x120
#define OV2640_176x144 		1	//176x144
#define OV2640_320x240 		2	//320x240
#define OV2640_352x288 		3	//352x288
#define OV2640_640x480		4	//640x480
#define OV2640_800x600 		5	//800x600
#define OV2640_1024x768		6	//1024x768
#define OV2640_1280x1024	7	//1280x1024
#define OV2640_1600x1200	8	//1600x1200
#define ARDUCHIP_FIFO      		0x04  //FIFO and I2C control
#define FIFO_CLEAR_MASK    		0x01
#define FIFO_START_MASK    		0x02
#define BURST_FIFO_READ			0x3C  //Burst FIFO read operation

// Globals
unsigned char m_fmt = JPEG;
unsigned int length = 0;
char readbuf[JPEG_BUF_SIZE];
unsigned char sensor_addr = 0;

// High(ish)-level functions
void singleCapture(int CS){ // Will be passed CAM_CS1
    int i , count;
    //Flush the FIFO
    flush_fifo(CS);
    // ADDED - set JPEG format
    set_format(JPEG);
    //Start capture
    start_capture(CS);
    while(!get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK, CS)){;}
    length = read_fifo_length(CS);
    count = length;
    i = 0 ;
    CS_LOW(CS); // API CALL
    set_fifo_burst();//Set fifo burst mode
    while (count--) {
        readbuf[i++] = spiSendReceive(0x00); // API CALL
    }
    count = 0;
    CS_HIGH(CS); // API CALL
}

// Hardcode for OV2640
void ArduCAM_Init() {
    wrSensorReg8_8(0xff, 0x01);
    wrSensorReg8_8(0x12, 0x80);
    if(m_fmt == JPEG) {
        wrSensorRegs8_8(OV2640_JPEG_INIT);
        wrSensorRegs8_8(OV2640_YUV422);
        wrSensorRegs8_8(OV2640_JPEG);
        wrSensorReg8_8(0xff, 0x01);
        wrSensorReg8_8(0x15, 0x00);
        wrSensorRegs8_8(OV2640_320x240_JPEG);
    } else {
        wrSensorRegs8_8(OV2640_QVGA);
    }
    if (model == OV2640)
        OV2640_set_JPEG_size(OV2640_320x240);
}

// Medium-level functions
void set_format(unsigned char fmt) {
    if (fmt == BMP)
        m_fmt = BMP;
    else if(fmt == RAW)
        m_fmt = RAW;
    else
        m_fmt = JPEG;
}
void flush_fifo(int CS) {
    write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK, CS);
}
void start_capture(int CS) {
    write_reg(ARDUCHIP_FIFO, FIFO_START_MASK, CS);
}
unsigned char get_bit(unsigned char addr, unsigned char bit, int CS) {
    unsigned char temp;
    temp = read_reg(addr, CS);
    temp = temp & bit;
    return temp;
}
unsigned int read_fifo_length(int CS) {
    unsigned int len1,len2,len3,len=0;
    len1 = read_reg(FIFO_SIZE1, CS);
    len2 = read_reg(FIFO_SIZE2, CS);
    len3 = read_reg(FIFO_SIZE3, CS) & 0x7f;
    len = ((len3 << 16) | (len2 << 8) | len1) & 0x07fffff;
    return len;
}
void set_fifo_burst() {
    spiSendReceive(BURST_FIFO_READ); // API CALL
}

// Low-level functions
void write_reg(unsigned char addr, unsigned char data, int CS) {
    bus_write(addr | 0x80, data, CS);
}
unsigned char bus_write(int address,int value,int CS) {
    CS_LOW(CS); // API call - set CS pin low
    spiSendReceive(address); // API call
    spiSendReceive(value); // API call
    CS_HIGH(CS); // API call
    return 1;
}
unsigned char read_reg(unsigned char addr,int CS) {
    unsigned char readData;
    readData = bus_read(addr & 0x7F, CS);
    return readData;
}
unsigned char bus_read(int address,int CS) {
    unsigned char value;
    CS_LOW(CS); // API call
    spiSendReceive(address); // API call
    value = spiSendReceive(0x00); // API call
    CS_HIGH(CS); // API call
    return value;
}

// STUFF THAT WILL BE REPLACED BY OUR ACTUAL API
#define HIGH  1
#define LOW   0
#define SPI0CSbits (* (volatile spi0csbits*) (spi + 0))
#define SPI0FIFO (* (volatile unsigned int *) (spi + 1))
#define GPSET    ((volatile unsigned int *) (gpio + 7))
#define GPCLR    ((volatile unsigned int *) (gpio + 10))
#define BLOCK_SIZE (4*1024)
#define PROT_READ       0x01    /* [MC2] pages can be read */
#define PROT_WRITE      0x02    /* [MC2] pages can be written */
#define MAP_SHARED      0x0001          /* [MF|SHM] share changes */
#define GPIO_BASE               (BCM2835_PERI_BASE + 0x200000)
#define SPI0_BASE			    (BCM2835_PERI_BASE + 0x204000)
#define MAP_FAILED      ((void *)-1)    /* [MF|SHM] mmap failed */
typedef struct
{
    unsigned CS 		:2;
    unsigned CPHA		:1;
    unsigned CPOL		:1;
    unsigned CLEAR 		:2;
    unsigned CSPOL		:1;
    unsigned TA 		:1;
    unsigned DMAEN		:1;
    unsigned INTD 		:1;
    unsigned INTR 		:1;
    unsigned ADCS		:1;
    unsigned REN 		:1;
    unsigned LEN 		:1;
    unsigned LMONO 		:1;
    unsigned TE_EN		:1;
    unsigned DONE		:1;
    unsigned RXD		:1;
    unsigned TXD		:1;
    unsigned RXR 		:1;
    unsigned RXF 		:1;
    unsigned CSPOL0 	:1;
    unsigned CSPOL1 	:1;
    unsigned CSPOL2 	:1;
    unsigned DMA_LEN	:1;
    unsigned LEN_LONG	:1;
    unsigned 			:6;
}spi0csbits;
volatile unsigned int *gpio; //pointer to base of gpio
volatile unsigned int *spi;  //pointer to base of spi registers
void CS_HIGH(int CS) {
    delay_us(1);
    digitalWrite(CS, HIGH);
}
void CS_LOW(int CS) {
    delay_us(1);
    digitalWrite(CS, LOW);
}
void delay_us(int micros) {
//    unsigned long nowtime = get_microsecond_timestamp();
//    while((get_microsecond_timestamp() - nowtime)<micros/2){;}
    // TODO: This will require time.h or equivalent (for now, just dummy code)
    int i = 0;
    while (i < 10000000) i++;
    return;
}
unsigned long get_microsecond_timestamp() {
//    struct timespec t;
//    if (clock_gettime(CLOCK_MONOTONIC_RAW, &t) != 0) {
//        return 0;
//    }
//    return (unsigned long) t.tv_sec * 1000000 + t.tv_nsec / 1000;
}
void digitalWrite(int pin, int val) {
    int reg = pin / 32;
    int offset = pin % 32;

    if (val) GPSET[reg] = 1 << offset;
    else     GPCLR[reg] = 1 << offset;
}

char spiSendReceive(char send) {
    SPI0FIFO = send;            // send data to slave
    while(!SPI0CSbits.DONE);	// wait until SPI transmission complete
    return SPI0FIFO;            // return received data
}
void pioInit() {
    int  mem_fd;
    void *reg_map;

    // /dev/mem is a psuedo-driver for accessing memory in the Linux filesystem
    if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
        printf("can't open /dev/mem \n");
        exit(-1);
    }

    reg_map = mmap(
            NULL,             //Address at which to start local mapping (null means don't-care)
            BLOCK_SIZE,       //Size of mapped memory block
            PROT_READ|PROT_WRITE,// Enable both reading and writing to the mapped memory
            MAP_SHARED,       // This program does not have exclusive access to this memory
            mem_fd,           // Map to /dev/mem
            GPIO_BASE);       // Offset to GPIO peripheral

    gpio = (volatile unsigned *)reg_map;

    reg_map = mmap(
            NULL,             //Address at which to start local mapping (null means don't-care)
            BLOCK_SIZE,       //Size of mapped memory block
            PROT_READ|PROT_WRITE,// Enable both reading and writing to the mapped memory
            MAP_SHARED,       // This program does not have exclusive access to this memory
            mem_fd,           // Map to /dev/mem
            SPI0_BASE);       // Offset to SPI peripheral

    if (reg_map == MAP_FAILED) {
        printf("spi mmap error %d\n", (int)reg_map);
        close(mem_fd);
        exit(-1);
    }

    spi = (volatile unsigned *)reg_map;
    close(mem_fd);
}

// I don't even know what this does...
int wrSensorRegs8_8(const struct sensor_reg reglist[]) {
    int err = 0;
    unsigned int reg_addr = 0;
    unsigned int reg_val = 0;
    const struct sensor_reg *next = reglist;
    while ((reg_addr != 0xff) | (reg_val != 0xff))
    {
        reg_addr =next->reg;
        reg_val = next->val;
        err = wrSensorReg8_8(reg_addr, reg_val);
        delay_ms(10);
        next++;
    }

    return err;
}
unsigned char wrSensorReg8_8(int regID, int regDat) {
    delay_us(10);
    sccb_bus_start();
    if(sccb_bus_write_byte(sensor_addr) == 0)
    {
        sccb_bus_stop();
        return 1;
    }
    delay_us(10);
    if(sccb_bus_write_byte(regID) == 0)
    {
        sccb_bus_stop();
        return 2;
    }
    delay_us(10);
    if(sccb_bus_write_byte(regDat)==0)
    {
        sccb_bus_stop();
        return 3;
    }
    sccb_bus_stop();
    return 0;
}
#define sda_port        2
#define scl_port        3
#define SCCB_SID_H()      digitalWrite(sda_port,HIGH)   //SDA	H
#define SCCB_SID_L()      digitalWrite(sda_port,LOW)    //SDA	H
#define SCCB_SIC_H()      digitalWrite(scl_port,HIGH)	 	//SCL H
#define SCCB_SIC_L()      digitalWrite(scl_port,LOW)		 	//SCL H
#define INPUT  0
#define OUTPUT 1
#define SCCB_DATA_IN      pinMode(sda_port, INPUT);
#define SCCB_DATA_OUT     pinMode(sda_port, OUTPUT);
#define GPFSEL    ((volatile unsigned int *) (gpio + 0))
unsigned char I2C_TIM  = 30;
void sccb_bus_start(void) {
    SCCB_SID_H();
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SID_L();
    delay_us(I2C_TIM);
    SCCB_SIC_L();
    delay_us(I2C_TIM);
}
void sccb_bus_stop(void) {
    SCCB_SID_L();
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    SCCB_SID_H();
    delay_us(I2C_TIM);
}
unsigned char sccb_bus_write_byte(unsigned char data) {
    unsigned char i;
    unsigned char tem;
    for(i = 0; i < 8; i++) {
        if((data<<i) & 0x80) {
            SCCB_SID_H();
        }
        else {
            SCCB_SID_L();
        }
        delay_us(I2C_TIM);
        SCCB_SIC_H();
        delay_us(I2C_TIM);
        SCCB_SIC_L();
    }
    SCCB_DATA_IN;
    delay_us(I2C_TIM);
    SCCB_SIC_H();
    delay_us(I2C_TIM);
    if(SCCB_SID_STATE) {
        tem = 0;
    }
    else {
        tem = 1;
    }

    SCCB_SIC_L();
    delay_us(I2C_TIM);
    SCCB_DATA_OUT;
    return tem;
}
void pinMode(int pin, int function) {
    int reg      =  pin/10;
    int offset   = (pin%10)*3;
    GPFSEL[reg] &= ~((0b111 & ~function) << offset);
    GPFSEL[reg] |=  ((0b111 &  function) << offset);
}
