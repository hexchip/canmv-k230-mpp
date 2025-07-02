/* Copyright (c) 2023, Canaan Bright Sight Co., Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/mman.h>
#include <pthread.h>
#include <fcntl.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <poll.h>
#include "k_type.h"

#define JAMLINK_DEVICE   "/dev/jamlink0"
#define JAMLINK_SAMPLE_DEBUG 

static pthread_t pthread_handle2 = RT_NULL;
static volatile char flag;
static pthread_cond_t cond;
static pthread_mutex_t mutex;
#if 0
/* jamlink clock */
enum jamlink_clock {
    JAMLINK_50M = 0,
    JAMLINK_25M,
	JAMLINK_16_7M,
	JAMLINK_12_5M,
	JAMLINK_10M = 4,
	JAMLINK_6_25M = 7,
	JAMLINK_5M = 9,
	JAMLINK_3_125M = 15
};

#else
#define	JAMLINK_SET_MODE        	_IOW('J', 0, int)
#define	JAMLINK_INIT           		_IOW('J', 1, int)
#define	JAMLINK_RESET          		_IOW('J', 2, int)
#define	JAMLINK_DI_RESET      	 	_IOW('J', 3, int)
#define	JAMLINK_DO_RESET       		_IOW('J', 4, int)
#define	JAMLINK_FIFO_RESET     		_IOW('J', 5, int)
#define	JAMLINK_SET_CLK        		_IOW('J', 6, int)
#define	JAMLINK_SET_CO_TH      		_IOW('J', 7, int)
#define	JAMLINK_GET_RX_CNT     		_IOW('J', 8, int)
#define	JAMLINK_GET_TX_CNT          _IOW('J', 9, int)
#define	JAMLINK_GET_STATUS          _IOW('J', 10, int)
#define	JAMLINK_GET_ECC           	_IOW('J', 11, int)
#define	JAMLINK_CLEAR_ECC           _IOW('J', 12, int)
#define	JAMLINK_SET_RX_INT_TH       _IOW('J', 13, int)
#define	JAMLINK_SET_RO_GPIO         _IOW('J', 14, int)

#define	JAMLINK_FIFO           		_IOW('J', 15, int)
#define	JAMLINK_DMA           		_IOW('J', 16, int)

#endif

#define JAMLINK_LOOP_MAX_CNT 128
#define JAMLINK_ID 0x00a3199b
#define SLAVE_CNT 1
#define JAMLINK_SEND_BUFF_SIZE 23

static int jamlink_check_data(k_char *src, k_char *dst, k_u32 length) 
{
    int value = 0,i;
    for (i = 0; i < length; i++) {
        if (src[i] != dst[i]) {
            printf("compare error. src[%3d]:%02x, dst[%3d]:%02x\r\n", i, src[i], i, dst[i]);
            value = 1;
        }
    }

    return value;
}

static k_u32 word_rev(k_u32 num)
{
	k_u32 data = 0;

	for (uint8_t index = 0; index < 32; index++) {
		data <<= 1;
		if (num & 0x01)
			data++;
		num >>= 1;
	}
	return data;
}


static void jamlink_fill_write_buff(k_u32 *buffer,rt_size_t count)
{
    int i;
    k_u32 buf[256] = {0};

    for(i = 0;i < count;i++)
    {
        buf[i] = i;
    }

    for(i = 0;i < count ;i++)
    {
        buffer[i + 3] = buf[i];      
    }
    buffer[0] &= ~ 0xff;
    buffer[0] |= count;
}

static void jamlink_send_read_work_cmd(int fd)
{
    k_u32 jamlink_read_work_table[] = {
        0xff040417, 0x0, 0x0
    };
    k_u32 tx_read_byte_size = sizeof(jamlink_read_work_table);

#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_read_word_size = sizeof(jamlink_read_work_table)/sizeof(jamlink_read_work_table[0]);


    printf("D: send:");
	for (int i = 0; i < tx_read_word_size; i++)
		printf(" %08x", jamlink_read_work_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_read_work_table, tx_read_byte_size);
	while(ioctl(fd, JAMLINK_GET_TX_CNT, 0) > 0){
        ;
    }

}

static void jamlink_write_read_work_test(int fd)
{  
    k_u32 rcv_buf[256];

    k_u32 jamlink_write_work_table[JAMLINK_SEND_BUFF_SIZE + 3] = {
        0xff020404, 0x0, 0x0, 1, 2, 3, 4
    };
    k_u32 rx_word_size = JAMLINK_SEND_BUFF_SIZE + 4;
    k_u32 tx_write_byte_size = sizeof(jamlink_write_work_table);
    k_u32 rx_byte_size = rx_word_size * 4;
 
    jamlink_fill_write_buff(jamlink_write_work_table, JAMLINK_SEND_BUFF_SIZE);

#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_write_word_size = sizeof(jamlink_write_work_table)/sizeof(jamlink_write_work_table[0]);

    printf("D: send:");
	for (int i = 0; i < tx_write_word_size; i++)
		printf(" %08x", jamlink_write_work_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_write_work_table, tx_write_byte_size);
	while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

    jamlink_send_read_work_cmd(fd);

    while(ioctl(fd, JAMLINK_GET_RX_CNT, NULL) < rx_word_size){
        ;
    }
	read(fd, rcv_buf, rx_byte_size);

#ifdef JAMLINK_SAMPLE_DEBUG
	printf("D: recv:");
	for (int i = 0; i < rx_word_size; i++)
		printf(" %08x", rcv_buf[i]);
	printf("\n");   
#endif

    if (jamlink_check_data((k_char *)&jamlink_write_work_table[3], (k_char *)&rcv_buf[4], JAMLINK_SEND_BUFF_SIZE * 4)) {
        printf("D: JamLink write read work test fail!\n\n");
    }else{
        printf("D: JamLink write read work test success!\n\n");
    }

}

static void jamlink_write_read_reg_test(int fd)
{  
	k_u32 tmp_buf[100];
    k_u32 jamlink_read_reg_table[] = {
        0xff040401, 0x50c, 0x0
    };
    k_u32 jamlink_write_reg_table[] = {
        0xff020401, 0x50c, 0x0, 0x22
    };
    k_u32 tx_read_word_size = sizeof(jamlink_read_reg_table)/sizeof(jamlink_read_reg_table[0]);
    k_u32 rx_word_size = tx_read_word_size + 1 + 1;
    k_u32 tx_read_byte_size = sizeof(jamlink_read_reg_table);
    k_u32 tx_write_byte_size = sizeof(jamlink_write_reg_table);
    k_u32 rx_byte_size = tx_read_byte_size + 4 + 4;

#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_write_word_size = sizeof(jamlink_write_reg_table)/sizeof(jamlink_write_reg_table[0]);

    printf("D: send:");
	for (int i = 0; i < tx_write_word_size; i++)
		printf(" %08x", jamlink_write_reg_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_write_reg_table, tx_write_byte_size);
	while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

#ifdef JAMLINK_SAMPLE_DEBUG
    printf("D: send:");
	for (int i = 0; i < tx_read_word_size; i++)
		printf(" %08x", jamlink_read_reg_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_read_reg_table, tx_read_byte_size);
	while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

    while(ioctl(fd, JAMLINK_GET_RX_CNT, NULL) < rx_word_size){
        ;
    }
	read(fd, tmp_buf, rx_byte_size);

#ifdef JAMLINK_SAMPLE_DEBUG
	printf("D: recv:");
	for (int i = 0; i < rx_word_size; i++)
		printf(" %08x", tmp_buf[i]);
	printf("\n");   
#endif

    if(jamlink_write_reg_table[3] != tmp_buf[4])
	    printf("D: JamLink write read reg test fail!\n\n");
    else
        printf("D: JamLink write read reg test success!\n\n");

}

static void jamlink_read_reg_test(int fd)
{  
	k_u32 tmp_buf[100];
    k_u32 jamlink_read_id_table[] = {
        0xff040401, 0x1fc, 0x0
    };
    k_u32 tx_word_size = sizeof(jamlink_read_id_table)/sizeof(jamlink_read_id_table[0]);
    k_u32 rx_word_size = tx_word_size + 1 + 1;
    k_u32 tx_byte_size = sizeof(jamlink_read_id_table);
    k_u32 rx_byte_size = tx_byte_size + 4 + 4;

	write(fd, jamlink_read_id_table, tx_byte_size);
    while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

    while(ioctl(fd, JAMLINK_GET_RX_CNT, NULL) < rx_word_size){
        ;
    }
	read(fd, tmp_buf, rx_byte_size);

#ifdef JAMLINK_SAMPLE_DEBUG
	printf("D: recv:");
	for (int i = 0; i < rx_word_size; i++)
		printf(" %08x", tmp_buf[i]);
	printf("\n");   
#endif

    if(tmp_buf[rx_word_size - 1] != JAMLINK_ID)
        printf("D: JamLink read reg test fail!\n\n");
    else
	    printf("D: JamLink read reg test success!\n\n");

}

static void jamlink_enum_test(int fd)
{
    k_u32 rx_buff[256];
    k_u32 jamlink_init_table[4] = {
	    0xff000401, 0x0, 0x0,0x0
    };
    k_u32 tx_word_size = sizeof(jamlink_init_table)/sizeof(jamlink_init_table[0]);
    k_u32 rx_word_size = tx_word_size;
    k_u32 tx_byte_size = sizeof(jamlink_init_table);
    k_u32 rx_byte_size = tx_byte_size;

#ifdef JAMLINK_SAMPLE_DEBUG
    printf("jamlink_enum_test\n"); 
    for (int i = 0; i < tx_word_size; i++)
		printf(" %08x", jamlink_init_table[i]);
    printf("\n"); 
#endif

    write(fd, jamlink_init_table, tx_byte_size);
    while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

    while(ioctl(fd, JAMLINK_GET_RX_CNT, NULL) < rx_word_size){
        ;
    }
    read(fd,rx_buff,rx_byte_size);

#ifdef JAMLINK_SAMPLE_DEBUG
    for (int i = 0; i < rx_word_size; i++)
		printf(" %08x", rx_buff[i]);
    printf("\n");
#endif

    if(word_rev(rx_buff[3]) != SLAVE_CNT)
        printf("jamlink init test fail\n\n");
    else
        printf("jamlink init test succuss\n\n");
}

static void jamlink_dma_write_read_work_test(int fd)
{
    k_u32 jamlink_write_work_table[JAMLINK_SEND_BUFF_SIZE + 3] = {
        0xff020404, 0x0, 0x0, 1, 2, 3, 4
    };
    k_u32 tx_write_byte_size = sizeof(jamlink_write_work_table);

    k_u32 jamlink_read_work_table[] = {
        0xff040417, 0x0, 0x0
    };
    k_u32 tx_read_byte_size = sizeof(jamlink_read_work_table);
 
    k_u32 rcv_buf[256];
    k_u32 rx_word_size = JAMLINK_SEND_BUFF_SIZE + 4;
    k_u32 rx_byte_size = rx_word_size * 4;
    struct pollfd fds[2];


    jamlink_fill_write_buff(jamlink_write_work_table, JAMLINK_SEND_BUFF_SIZE);

#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_write_word_size = sizeof(jamlink_write_work_table)/sizeof(jamlink_write_work_table[0]);

    printf("D: send:");
	for (int i = 0; i < tx_write_word_size; i++)
		printf(" %08x", jamlink_write_work_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_write_work_table, tx_write_byte_size);
    while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }


#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_read_word_size = sizeof(jamlink_read_work_table)/sizeof(jamlink_read_work_table[0]);

    printf("D: send:");
	for (int i = 0; i < tx_read_word_size; i++)
		printf(" %08x", jamlink_read_work_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_read_work_table, tx_read_byte_size);
    while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

    fds[1].fd = fd;
    fds[1].events = POLLIN;

    poll(&fds[1], 1, -1);
	read(fd, rcv_buf, rx_byte_size);

#ifdef JAMLINK_SAMPLE_DEBUG
	printf("D: recv:");
	for (int i = 0; i < rx_word_size; i++)
		printf(" %08x", rcv_buf[i]);
	printf("\n");   
#endif

    if (jamlink_check_data((k_char *)&jamlink_write_work_table[3], (k_char *)&rcv_buf[4], JAMLINK_SEND_BUFF_SIZE * 4)) {
        printf("D: JamLink write read work test fail!\n\n");
    }else{
        printf("D: JamLink write read work test success!\n\n");
    }

}

static void jamlink_rx_intc_timer_test(int fd)
{
    k_u32 jamlink_write_work_table[JAMLINK_SEND_BUFF_SIZE + 3] = {
        0xff020404, 0x0, 0x0, 1, 2, 3, 4
    };
    k_u32 tx_write_byte_size = sizeof(jamlink_write_work_table);

    k_u32 jamlink_read_work_table[] = {
        0xff040417, 0x0, 0x0
    };
    k_u32 tx_read_byte_size = sizeof(jamlink_read_work_table);
 
    k_u32 rcv_buf[256];
    k_u32 rx_word_size = JAMLINK_SEND_BUFF_SIZE + 4;
    k_u32 rx_byte_size = rx_word_size * 4;
    struct pollfd fds[2];


    jamlink_fill_write_buff(jamlink_write_work_table, JAMLINK_SEND_BUFF_SIZE);

#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_write_word_size = sizeof(jamlink_write_work_table)/sizeof(jamlink_write_work_table[0]);

    printf("D: send:");
	for (int i = 0; i < tx_write_word_size; i++)
		printf(" %08x", jamlink_write_work_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_write_work_table, tx_write_byte_size);
    while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

#ifdef JAMLINK_SAMPLE_DEBUG
    k_u32 tx_read_word_size = sizeof(jamlink_read_work_table)/sizeof(jamlink_read_work_table[0]);

    printf("D: send:");
	for (int i = 0; i < tx_read_word_size; i++)
		printf(" %08x", jamlink_read_work_table[i]);
	printf("\n");
#endif

	write(fd, jamlink_read_work_table, tx_read_byte_size);
    while(ioctl(fd, JAMLINK_GET_TX_CNT, NULL) > 0){
        ;
    }

    fds[1].fd = fd;
    fds[1].events = POLLIN;

    poll(&fds[1], 1, -1);
	read(fd, rcv_buf, rx_byte_size);

#ifdef JAMLINK_SAMPLE_DEBUG
	printf("D: recv:");
	for (int i = 0; i < rx_word_size; i++)
		printf(" %08x", rcv_buf[i]);
	printf("\n");   
#endif

    if (jamlink_check_data((k_char *)&jamlink_write_work_table[3], (k_char *)&rcv_buf[4], JAMLINK_SEND_BUFF_SIZE * 4)) {
        printf("D: JamLink rx interrupt timer test fail!\n\n");
    }else{
        printf("D: JamLink rx interrupt timer test success!\n\n");
    }

}

static void *thread2_entry(void *parameter)
{
    int fd;
    int gpio_low = 0;
    int gpio_high = 1;
    k_u32 dma_flag = JAMLINK_DMA;
    k_u32 fifo_flag = JAMLINK_FIFO;
    k_u32 rx_intr_value = 10;

    printf(" [app] open jamlink.....\n");
    fd = open(JAMLINK_DEVICE, O_RDWR);
    if (fd < 0)
    {
        printf("open dev jamlink failed!\n");
        return NULL;
    }

    ioctl(fd, JAMLINK_INIT, NULL);
    jamlink_enum_test(fd);

    ioctl(fd, JAMLINK_SET_RO_GPIO, &gpio_low);
    ioctl(fd, JAMLINK_SET_RO_GPIO, &gpio_high);
    ioctl(fd, JAMLINK_SET_RO_GPIO, &gpio_low);
    jamlink_read_reg_test(fd);
    jamlink_write_read_reg_test(fd);
    jamlink_write_read_work_test(fd);

    printf("dma mode\n");
    ioctl(fd, JAMLINK_SET_MODE, &dma_flag);
    jamlink_dma_write_read_work_test(fd);

    printf("fifo mode\n");
    ioctl(fd, JAMLINK_SET_MODE, &fifo_flag);
    ioctl(fd, JAMLINK_SET_RX_INT_TH, &rx_intr_value);
    jamlink_rx_intc_timer_test(fd);

    printf("end\n");
    while(1)
    {
        pthread_mutex_lock(&mutex);
        // if (poll(fds, 1, -1) > 0 && fds[0].revents & POLLIN)
        if(1)
        {
            pthread_mutex_unlock(&mutex); 
            pthread_cond_signal(&cond);
        }
    }

    close(fd);
    return NULL;
}


int main()
{
    flag = 0;

    pthread_mutex_init(&mutex, NULL);
    pthread_cond_init(&cond, NULL);
    pthread_create(&pthread_handle2, NULL, thread2_entry, NULL);


    while(1)
    {
        if (!pthread_cond_wait(&cond, &mutex))
        {
            if (flag)
            {

                // checkdata(send, buf, 512);
                // checkdata(send, &buf[512], 512);
                flag = 0;
            }

            pthread_mutex_unlock(&mutex); 
        }
    }


    pthread_join(pthread_handle2, NULL);

    pthread_mutex_destroy(&mutex);

    return 0;
}
