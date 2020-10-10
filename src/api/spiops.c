/*
 ============================================================================
 Name        : spiops.c
 Author      : B. Eschrich
 Version     :
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates SPI bus operations
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

#include <sys/file.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <wiringPiSPI.h>

#include "spiops.h"

/*GCC specific: not all static's used here */
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

/* SPI device initialization parameters */
#define PP_SPI_BUS_SPEED		500000

/* SPI_CS_HIGH 	->  chip select active high
 SPI_CPHA 	->  clock phase
 SPI_CPOL		->  clock polarity
 SPI_RX_QUAD	->
 SPI_TX_QUAD	->
 SPI_RX_DUAL  ->
 SPI_TX_DUAL  ->
 SPI_NO_CS 	-> One dev/bus per device, no Chip Select
 SPI_READY	-> slave pulls low to pauses
 SPI_LSB_FIRST-> per-word bits-on-wire
 SPI_3WIRE	-> SI/SO signals shared
 SPI_LOOP		-> loopback mode

 SPI_MODE_0 	(0|0) / (original MicroWire) /
 SPI_MODE_1	(0|SPI_CPHA)
 SPI_MODE_2	(SPI_CPOL|0)
 SPI_MODE_3	(SPI_CPOL|SPI_CPHA)
 */
#define PP_SPI_OP_MODE		SPI_MODE_0

/* Usinf MODE32 setup */
// #define SPI_USE_OP_MODE_32

/**
 * Dump data bytes
 */
static void
spi_dump (const void* src, size_t size, const size_t line_size, const char* prefix)
{
    int i = 0;
    size_t length = size;
    const unsigned char* address = src;
    const unsigned char* line = address;
    unsigned char c;
    printf ("%s | ", prefix);
    while (length-- > 0)
    {
        printf ("%02X ", *address++);
        if (!(++i % line_size) || (length == 0 && i % line_size))
        {
            if (length == 0)
            {
                while (i++ % line_size)
                {
                    printf ("__ ");
                }
            }
            printf (" | "); /* right close */
            while (line < address)
            {
                c = *line++;
                printf ("%c", (c < 33 || c == 255) ? 0x2E : c);
            }
            printf ("\n");
            if (length > 0)
            {
                printf ("%s | ", prefix);
            }
        }
    }
}

/**
 * Report SPI error
 */
static int
spi_error (int code, const char* message, ...)
{
    va_list argp;
    char buffer[4096];
    va_start(argp, message);
    memset(buffer, 0, sizeof(buffer));
    vsnprintf (buffer, sizeof(buffer)-1, message, argp);
    va_end(argp);
    fprintf (stderr, __FILE__ ": [#%d] %s", code, buffer);
    return (code * -1);
}

/**
 *
 */
static int
spi_open (spidev_t* spidev)
{
    int ret;
    uint32_t maxspeed;

    if ((ret = open (spidev->path, O_RDWR)) <= 0)
    {
        return -ENODEV;
    }

    spidev->fd = ret;

    /* Setup transfer mode */
    if ((ret = ioctl (spidev->fd, SPI_IOC_WR_MODE, &spidev->mode)))
    {
        return spi_error (1020, "Can't setup SPI transfer mode. ret=%d\n", ret);
    }
    if ((ret = ioctl (spidev->fd, SPI_IOC_RD_MODE, &spidev->mode)))
    {
        return spi_error (1021, "Can't read SPI transfer mode. ret=%d\n", ret);
    }

    /* Setup bits per word */
    if ((ret = ioctl (spidev->fd, SPI_IOC_WR_BITS_PER_WORD, &spidev->bitspw)))
    {
        return spi_error (1000, "Can't setup bits per word. ret=%d\n", ret);
    }
    if ((ret = ioctl (spidev->fd, SPI_IOC_RD_BITS_PER_WORD, &spidev->bitspw)))
    {
        return spi_error (1001, "Can't read bits per word. ret=%d\n", ret);
    }

    /* Read current max speed in Hz */
    if ((ret = ioctl (spidev->fd, SPI_IOC_RD_MAX_SPEED_HZ, &maxspeed)))
    {
        return spi_error (1010, "Can't read max speed hz. ret=%d\n", ret);
    }

    /* Setup max speed in Hz */
    if ((ret = ioctl (spidev->fd, SPI_IOC_WR_MAX_SPEED_HZ, &spidev->speed)))
    {
        return spi_error (1011, "Can't setup max speed hz. ret=%d\n", ret);
    }
    if ((ret = ioctl (spidev->fd, SPI_IOC_RD_MAX_SPEED_HZ, &spidev->speed)))
    {
        return spi_error (1012, "Can't read max speed hz. ret=%d\n", ret);
    }

#ifdef SPI_USE_OP_MODE_32
    /* SPI full transfer mode */
    if ((ret = ioctl(spidev->fd, SPI_IOC_WR_MODE32, &spidev->mode)) == -1)
    {
        return spi_error(1031, "Can't setup SPI mode 32\n");
    }
    if((ret = ioctl(spidev->fd, SPI_IOC_RD_MODE32, &spidev->mode)) == -1)
    {
        return spi_error(1032, "Can't read SPI mode 32\n");
    }
#endif

#ifdef TRACE_SPI_SETUP
    fprintf (stdout, __FILE__ ":spi_open():\n");
    fprintf (stdout, "- SPI OP mode...: %x\n", spidev->mode);
    fprintf (stdout, "- Bits per word.: %d\n", spidev->bitspw);
    fprintf (stdout, "- Read delay....: %d\n", spidev->rdelay);
    fprintf (stdout, "- Write delay...: %d\n", spidev->wdelay);
    fprintf (stdout, "- CS change.....: %d\n", spidev->cschg);
    fprintf (stdout, "- Xfer speed....: %d Hz (%d KHz)\n", spidev->speed, spidev->speed / 1000);
    fprintf (stdout, "- Max speed.....: %d Hz (%d KHz)\n", maxspeed, maxspeed / 1000);
#endif // TRACE_SPI_OPS

    //if (maxspeed > spidev->speed)
    //    spidev->speed = maxspeed;

    usleep (1000);
    return 0;
}

/**
 *
 */
static int
spi_close (spidev_t* spidev)
{
    if (spidev && spidev->fd)
    {
        close (spidev->fd);
        spidev->fd = 0;
    }
    return 0;
}

/**
 *
 */
static int
spi_send (const spidev_t* spidev, const uint8_t* buffer, const size_t size)
{
    struct spi_ioc_transfer spi;
    int ret;

#ifdef TRACE_SPI_OPS
    spi_dump (buffer, size, 16, "SPI-TX");
#endif

    memset (&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long) buffer;
    spi.rx_buf = (unsigned long) NULL;
    spi.len = size;
    spi.delay_usecs = spidev->wdelay;
    spi.speed_hz = spidev->speed;
    spi.bits_per_word = spidev->bitspw;
    spi.cs_change = spidev->cschg;

    /* adapted from py_spidev/spidev_module */
#ifdef SPI_USE_OP_MODE_32
    spi.tx_nbits = 0;
    spi.rx_nbits = 0;
#endif

    if ((ret = ioctl (spidev->fd, SPI_IOC_MESSAGE(1), &spi)) < 1)
    {
        return spi_error (1300, "spi_send(): Can't send spi message. ret=%d\n", ret);
    }

    return 0;
}

/**
 *
 */
static int
spi_fetch (const spidev_t* spidev, uint8_t* buffer, const size_t size)
{
    struct spi_ioc_transfer spi;
    int ret;

    memset (&spi, 0, sizeof(spi));
    spi.tx_buf = (unsigned long) NULL;
    spi.rx_buf = (unsigned long) buffer;
    spi.len = size;
    spi.delay_usecs = spidev->rdelay;
    spi.speed_hz = spidev->speed;
    spi.bits_per_word = spidev->bitspw;
    spi.cs_change = spidev->cschg;

    /* adapted from py_spidev/spidev_module */
#ifdef SPI_USE_OP_MODE_32
    spi.tx_nbits = 0;
    spi.rx_nbits = 0;
#endif

    if ((ret = ioctl (spidev->fd, SPI_IOC_MESSAGE(1), &spi)) < 1)
    {
        return spi_error (1100, "spi_fetch(): Can't fetch spi response. ret=%d\n", ret);
    }

    if (spidev->mode & SPI_CS_HIGH)
    {
        ret = read (spidev->fd, buffer, 0);
    }

#ifdef TRACE_SPI_OPS
    spi_dump (buffer, size, 16, "SPI-RX");
#endif

    return 0;
}

/**
 *
 */
static int
spi_xfer (const spidev_t* spidev, const board_command_t* command)
{
    int ret, i = 0;
    uint8_t byte[1] = {0};

#ifdef TRACE_SPI_COMMAND
    fprintf (stdout, __FILE__ ":spi_xfer(): address=0x%02x command=0x%02x p1=0x%02x p2=0x%02x\n",
             command->address,
             command->command,
             command->param1,
             command->param2);
#endif

    uint8_t tx[4] = { //-
        command->address, //-
        command->command, //-
        command->param1,  //-
        command->param2,  //-
    };
    size_t size = sizeof(tx);

    if ((ret = spi_send (spidev, tx, size)))
    {
        return ret;
    }

    if (command->size > 0 && command->result)
    {
        memset(command->result, 0, command->size);

        for (i = 0; i < command->size; i++)
        {
            if ((ret = spi_fetch (spidev, &byte[0], 1)) < 0)
            {
                return ret;
            }

            /* stop at zero terminator */
            if (((byte[0] == 0) && command->stopAt0x0))
            {
                break;
            }

            command->result[i] = byte[0];
        }
    }

    return 0;
}

/**
 *
 */
int
spi_init (const char* path, spidev_t* spidev)
{
    if (path == NULL || strlen (path) == 0 || spidev == NULL)
    {
        return spi_error (-EINVAL, __FILE__ ":spi_init(): Invalid parameters");
    }

    spidev->path = path;
    spidev->ops.open = &spi_open;
    spidev->ops.close = &spi_close;
    spidev->ops.xfer = &spi_xfer;
    spidev->ops.send = &spi_send;
    spidev->ops.fetch = &spi_fetch;
    spidev->ops.dump = &spi_dump;
    spidev->mode = PP_SPI_OP_MODE;
    spidev->speed = PP_SPI_BUS_SPEED;
    spidev->wdelay = 60;
    spidev->rdelay = 20;
    spidev->bitspw = 8; /* 8 bits per word */
    spidev->cschg = 1; /* change chip select */

    return spi_open (spidev);
}
