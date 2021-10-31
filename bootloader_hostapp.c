#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <termios.h>
#include "Bootloader.h"
 
#define FILE_SIZE 20480 /* 20kb alloc */  
#define SIZE_1024 1024

#define BL_CMD_UNLOCK 0xa0
#define BL_CMD_DATA 0xa1
#define BL_CMD_VERIFY 0xa2
#define BL_CMD_RESET 0xa3
#define BL_CMD_BNKSWAP_RESET 0xa4

#define BL_RESP_OK 0x50
#define BL_RESP_ERROR 0x51
#define BL_RESP_INVALID 0x52
#define BL_RESP_CRC_OK 0x53
#define BL_RESP_CRC_FAIL 0x54

#define BL_GUARD 0x5048434D

#define ERASE_SIZE 256

#define BOOTLOADER_SIZE 2048

//devices = {"SAME7X" : [8192, 8192]}

int open_serial_port(const char *device, uint32_t baudrate);
int write_port(int fd, uint8_t *buffer, size_t size);
ssize_t read_port(int fd, uint8_t *buffer, size_t size);
void crc32_tab_gen();
int32_t crc32 (uint8_t *data);
int unlock_command(uint32_t addr, uint8_t fsize, int fd);
uint32_t check_option(int opt);
int create_data_blocks(uint8_t *fdata, uint8_t fsize, uint32_t);
int send_verif_command(uint32_t crc);
int send_reboot_command();

uint32_t crc_table[256];


// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
int open_serial_port(const char * device, uint32_t baud_rate)
{
    struct termios options;

    int fd = open(device, O_RDWR | O_NOCTTY);

    if (fd == -1)
    {
        perror(device);
        return -1;
    }
 
    // Flush away any bytes previously read or written.
    int result = tcflush(fd, TCIOFLUSH);
    if (result)
    {
        perror("tcflush failed");  // just a warning, not a fatal error
    }

    // Get the current configuration of the serial port.
    result = tcgetattr(fd, &options);
    if (result)
    {
        perror("tcgetattr failed");
        close(fd);
        return -1;
    }

    // Turn off any options that might interfere with our ability to send and
    // receive raw binary bytes.
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    // Set up timeouts: Calls to read() will return as soon as there is
    // at least one byte available or when 100 ms has passed.
    options.c_cc[VTIME] = 1;
    options.c_cc[VMIN] = 0;

    // This code only supports certain standard baud rates. Supporting
    // non-standard baud rates should be possible but takes more work.
    switch (baud_rate)
    {
        case 4800:   cfsetospeed(&options, B4800);   break;
        case 9600:   cfsetospeed(&options, B9600);   break;
        case 19200:  cfsetospeed(&options, B19200);  break;
        case 38400:  cfsetospeed(&options, B38400);  break;
        case 115200: cfsetospeed(&options, B115200); break;
        default:
            fprintf(stderr,
                    "warning: baud rate %u is not supported, using 9600.\n",
                    baud_rate);
                    cfsetospeed(&options, B9600);
            break;
    }

    cfsetispeed(&options, cfgetospeed(&options));

    result = tcsetattr(fd, TCSANOW, &options);

    if (result)
    {
        perror("tcsetattr failed");
        close(fd);
        return -1;
    }

    return fd;
}
 
// Writes bytes to the serial port, returning 0 on success and -1 on failure.
int write_port(int fd, uint8_t * buffer, size_t size)
{
    ssize_t result = write(fd, buffer, size);
    if (result != (ssize_t)size)
    {
        perror("failed to write to port");
        return -1;
    }

    return 0;
}
 
// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
ssize_t read_port(int fd, uint8_t * buffer, size_t size)
{
    size_t received = 0;
    while (received < size)
    {
        ssize_t r = read(fd, buffer + received, size - received);
        if (r < 0)
        {
            perror("failed to read from port");
            return -1;
        }
        if (r == 0)
        {
            // Timeout
            break;
        }

        received += r;
    }
 
    return received;
} 

void crc32_tab_gen()
{
    uint32_t result;
    uint32_t value;
    int a, b;

    for (a = 0; a < 256; a++) {
        value = a;

        for (b = 0; b < 8; b++) {
            if (value & 1) {    
                value >> 1;              
                value ^= 0xedb88320;
            }
            else {
                value = value >> 1;
            }
        }
             
        crc_table[a] = value;  
    }  
}

int32_t crc32 (uint8_t *data)
{
    uint32_t crc;
    int      i;
   
    crc = 0xffffffff;

    for (i = 0; i < 256; i++) {
        crc = crc_table[(crc ^ data[i]) && 0xff] ^ (crc >> 8);
    }

    return crc;
}

int unlock_command(uint32_t addr, uint8_t fsize, int fd)
{
    uint8_t  data[1024];
    uint8_t  buffer[1];

    printf("Unlocking \n");

    /* BL_CMD_UNLOCK */
    data[0] = BL_GUARD;
    data[1] = BL_GUARD >> 8;
    data[2] = BL_GUARD >> 16;
    data[3] = BL_GUARD >> 24;

    /*  data size */
    data[4] = 8;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    /* unlock command */
    data[8] = BL_CMD_UNLOCK;

    /* start addr */
    data[9] = addr;
    data[10] = addr >> 8;
    data[11] = addr >> 16;
    data[12] = addr >> 24;

    /* image size */  
   
    data[13] = fsize;
    data[14] = fsize >> 8;
    data[15] = fsize >> 16;
    data[16] = fsize >> 24;


    write_port(fd, data, 17);

    while (read_port(fd, buffer, 1) == 0);
   
    if (buffer != BL_RESP_OK) {
        printf("invalid unlock response code\n");
        return 1;
    }

    return 0;
}

uint32_t check_option(int opt)
{
    uint32_t addr;

    switch (opt) {
        case 'a':
            if (optarg % 1024) != 0) {
                printf("invalid address \n");
            }  
            else {
                addr = optarg;
            }
            break;

        case 's':
            break;
        case 'c':
            break;
        default:
            printf("not in option\n");
            return 1;
    }

    return addr;
}

int create_data_blocks(uint8_t *fdata, uint8_t fsize, uint32_t address)
{
    uint8_t  data[1024];
    uint8_t  buffer[1];
    int      i, idx, didx;
	uint32_t addr;

    printf("Verification \n)";

    /* BL_GUARD */
    data[0] = BL_GUARD;
    data[1] = BL_GUARD >> 8;
    data[2] = BL_GUARD >> 16;
    data[3] = BL_GUARD >> 24;

    /*  data size */
    data[4] = 256 + 4;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    /* unlock command */
    data[8] = BL_CMD_DATA;

	addr = addr + blk;
    data[9] = addr;
    data[10] = addr >> 8;
    data[11] = addr >> 16;
    data[12] = addr >> 24;

    idx = 13;

    datasize = fsize;
    i = 0;

	while (datasize > 0)
	{
	    for (; i < ERASE_SIZE, i++)
		{
			datasize--;
			data[idx] = fdata[i];
		}

		write_port(fd, data, ERASE_SIZE + idx);

		while (read_port(fd, buffer, 1) == 0);
	   
		if (buffer != BL_RESP_CRC_OK) {
			printf("invalid response code\n");
			return 1;
		}

		idx = idx + ERASE_SIZE;
        idx++;

        /*  data size */
        data[idx] = ERASE_SIZE + 4;
		idx++;
        data[idx] = 0;
		idx++;
        data[idx] = 0;
		idx++;
        data[idx] = 0;
		idx++;

        /* unlock command */
        data[idx] = BL_CMD_DATA;
		idx++;

	    addr = addr + ERASE_SIZE;
        data[idx] = addr;
		idx++;
        data[idx] = addr >> 8;
		idx++;
        data[idx] = addr >> 16;
		idx++;
        data[idx] = addr >> 24;
		idx++;
	}

    return 0;
}

int send_verif_command(uint32_t crc)
{
    uint8_t  data[1024];
    uint8_t  buffer[1];

    printf("Verification \n)";

    /* BL_GUARD */
    data[0] = BL_GUARD;
    data[1] = BL_GUARD >> 8;
    data[2] = BL_GUARD >> 16;
    data[3] = BL_GUARD >> 24;

    /*  data size */
    data[4] = 4;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    /* unlock command */
    data[8] = BL_CMD_VERIFY;

    /* crc */
    data[9] = crc;
    data[10] = crc >> 8;
    data[11] = crc >> 16;
    data[12] = crc >> 24;


    write_port(fd, data, 13);

    while (read_port(fd, buffer, 1) == 0);
   
    if (buffer != BL_RESP_CRC_OK) {
        printf("invalid verify command response code\n");
        return 1;
    }

    printf("Success \n");    

    return 0;
}

int send_reboot_command()
{
    uint8_t  data[1024];
    uint8_t  buffer[1];

    printf("Swapping Bank and Rebooting \n");


    /* BL_GUARD */
    data[0] = BL_GUARD;
    data[1] = BL_GUARD >> 8;
    data[2] = BL_GUARD >> 16;
    data[3] = BL_GUARD >> 24;

    /*  data size */
    data[4] = 16;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;

    /* unlock command */
    data[8] = BL_CMD_BKSWAP_RESET;


    write_port(fd, data, 9);

    while (read_port(fd, buffer, 1) == 0);
   
    if (buffer != BL_RESP_OK) {
        printf("reset fail \n");
        return 1;
    }

    printf("Reboot Done! \n";

    return 0;
}



int main(int argc, char *argv[])
{
    FILE     *fp;
    uint8_t  fdata[FILE_SIZE]; /* 20kb alloc for file content */
    int      opt, optval;
    uint8_t  fsize, data[1024], buffer[1];
    uint32_t crc32, crc, addr;
    size_t   received;

    const char * device = "/dev/ttyUSB0";
    uint32_t baud_rate = 115200;
 
    int fd = open_serial_port(device, baud_rate);
   
    if (fd < 0) {
        return 1;
    }
       
    opt = getopt(argc, argv, "asc"); /* addr, swbnk, comm */
    fp = fopen(arv[1], "r");

    addr = check_option(opt)

    /* get data and size */
    while (1) {
        fdata[i] = fgetc(fp);

        if (feof(file))
            break;

        i++;    
    }

    fsize = i;

    crc32_tab_gen();
    crc = crc(fdata);
   

    /* should be ready for fw update once it gets signal from device */
    while (1)
    {
        printf("Waiting for device Firmware Update signal\n");

        /* give way to exit the program */
        if (getchar() == 'x') {  
            fclose(fp);  
            close(fd); 
            return 0;
        }
       
        /* check signal for fw update */
        read_port(fd, buffer, 1);

        if (buffer[0] == 'd') {

            /* update firmware */
            printf("Unlocking\n");

            /* unlock command */
            if (unlock_command(addr, fsize, fd) != 0) {
                printf("Unlock Command Error \n");
                close(fd); 
                fclose(fp);
                return 1;
            }

            /* create data blocks of ERASE_SIZE each */
            if (create_data_blocks(fdata, fsize, addr) != 0) {
                printf("create data block invalid response \n");
                close(fd); 
                fclose(fp);
                return 1;
            }

            /* Send verification command */
            if (send_verif_command(crc) != 0) {            
                printf("invalid verify command response code\n");
                close(fd); 
                fclose(fp);
                return 1;
   
            }
            /* Send Reboot command */
            if (send_reboot_command() != 0) {
                printf("Reset Fail\n");
                close(fd); 
                fclose(fp);
                return 1;
            }

        }
    }            
}



