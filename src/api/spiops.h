/*
 ============================================================================
 Name        : spiops.c
 Author      : B. Eschrich
 Version     :
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates SPI bus operations
 ============================================================================
 */
#ifndef _spiops_h
#define _spiops_h

// generic operation delay
#define SPI_IO_DELAY 1000

/* Internal structures */
typedef struct spidev_t spidev_t;

/* SPI command structure
 * channel      - SPI bus channel to use
 * address      - PI Plates board address
 * command      - PI Plates board command
 * param1       - Command parameter 1
 * param2       - Command parameter 2
 * size         - Return buffer size
 * result       - Pointer to the return buffer
 * stopAt0x0    - Stop string read operation if 0x0 found
 */
typedef struct
{
    uint8_t address;
    uint8_t command;
    uint8_t param1;
    uint8_t param2;
    size_t size;
    uint8_t* result;
    uint8_t stopAt0x0;
} board_command_t;

/* SPI I/O operations */
typedef struct
{
    /**
     * Open SPI device
     */
    int (*open)(spidev_t*);
    /**
     * Close SPI device
     */
    int (*close)(spidev_t*);
    /**
     * Send command to PI-Plates board and get response
     */
    int (*xfer)(const spidev_t*, const board_command_t*);
    /**
     * Send command to PI-Plates board without response
     */
    int (*send)(const spidev_t*, const uint8_t* buffer, const size_t size);
    /**
     * Fetch pending data from PI-Plates
     */
    int (*fetch)(const spidev_t*, uint8_t*, const size_t);
    /**
     * Dump buffer
     * @param buffer to dump
     * @param size of buffer
     * @param length of line
     * @param prefix to show
     */
    void (*dump) (const void*, const size_t, const size_t, const char*);
} spi_ops_t;

typedef struct spidev_t
{
    /**
     * SPI operation functions
     */
    spi_ops_t ops;

    /**
     * File handle of SPI device
     */
    int fd;

    /**
     * Path of userland SPI device (/dev/spi...)
     */
    const char* path;

    /**
     * SPI internals
     */
    uint32_t mode;
    uint32_t speed;
    uint32_t rdelay;
    uint32_t wdelay;
    uint8_t bitspw;
    uint8_t cschg;

} spidev_t;

/**
 * Initialize SPI device access
 */
int spi_init(const char* path, spidev_t* spidev);

#endif // _spiops_h
