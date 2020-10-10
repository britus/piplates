/*
 ============================================================================
 Name        : pptest.c
 Author      : B. Eschrich
 Version     : 1.00
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates RELAYplate and DAQCplate control
 ============================================================================
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

// GPIO/SPI library
#include <wiringPi.h>

// Out SPI bus operations
#include "../api/spiops.h"

// PI Plates RELAYplate API
#include "../api/ppapi.h"

#define DAQC_BOARD_ADDR  0
#define RELAY_BOARD_ADDR 1

/* Possible SPI devices
 * To activate SPI devices configure in /boot/config.txt
 * following lines:
 * # SPI Support overlay
 * dtoverlay=spi-bcm2835-overlay
 * # Additional SPI with Chip Select
 * dtoverlay=spi0-cs
 * dtoverlay=spi0-hw-cs
 * dtoverlay=spi1-1cs
 * dtoverlay=spi1-2cs
 * dtoverlay=spi1-3cs
 * # SPI bus on
 * dtparam=spi=on
 */
const char* spi_devices[] = { // -
    "/dev/spidev0.0", // -
    "/dev/spidev0.1", // -
    "/dev/spidev1.0", // -
    "/dev/spidev1.1", // -
    "/dev/spidev2.0", // -
    "/dev/spidev2.1", // -
    "/dev/spidev3.0", // -
    "/dev/spidev3.1", // -
    "/dev/spidev4.0", // -
    "/dev/spidev4.1", // -
    NULL, // -
};

static int
testRelayBoard ()
{
    board_t* board;
    config_t config;
    uint8_t devId;

    puts ("=================================================");
    puts ("RELAYplate test");
    puts ("=================================================");

    board = NULL;
    for (devId = 0; spi_devices[devId] != NULL; devId++)
    {
        if (initConfig (spi_devices[devId], 3, 6, 4, 24, &config) == 0)
        {
            if ((board = initBoard(PP_BOARD_TYPE_RELAY, RELAY_BOARD_ADDR, &config)))
            {
                fprintf (stdout, "RELAYplate using SPI device '%s'\n", spi_devices[devId]);
                break;
            }
        }
    }

    /* nothing found */
    if (board == NULL)
    {
        puts ("RELAYplate not found.");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    printBoardInfo(board);

    // switch off green board LED
    if (updateLED (board, 0, STATE_OFF) < 0)
    {
        puts ("LED switch off failed.");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    uint8_t state;
    int relay = 4;

    for (relay = 1; relay < PP_MAX_RELAYS; relay++)
    {
        printf ("Switch ON  relay #%d ", relay);
        if (relayON (board, relay) < 0)
        {
            puts ("\nRelay ON failed.");
            cleanupBoard(board);
            return EXIT_FAILURE;
        }

        usleep (220000);

        if (getRelayState (board, &state) < 0)
        {
            puts ("\nGet relay state failed.");
            cleanupBoard(board);
            return EXIT_FAILURE;
        }
        printf ("states: 0x%x\n", state);

        printf ("Switch OFF relay #%d ", relay);
        if (relayOFF (board, relay) < 0)
        {
            puts ("\nRelay OFF failed.");
            cleanupBoard(board);
            return EXIT_FAILURE;
        }

        usleep (220000);

        if (getRelayState (board, &state) < 0)
        {
            puts ("\nGet relay state failed.");
            cleanupBoard(board);
            return EXIT_FAILURE;
        }
        printf ("states: 0x%x\n", state);
    }

    uint8_t relays = BIT2_STATE_ON | BIT4_STATE_ON | BIT6_STATE_ON;
    if (updateRelays (board, relays) < 0)
    {
        puts ("Switch relays by bit mask failed.");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    usleep (589000);

    // switch off two relays
    relays = relays & ~BIT4_STATE_ON;
    relays = relays & ~BIT6_STATE_ON;

    // switch on another two relays
    relays |= BIT5_STATE_ON;
    relays |= BIT7_STATE_ON;

    if (updateRelays (board, relays) < 0)
    {
        puts ("Switch relays by bit mask failed.");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    usleep (589000);

    if (updateRelays (board, 0) < 0)
    {
        puts ("Switch relays by bit mask failed.");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    // switch on green board LED
    if (updateLED (board, 0, STATE_ON) < 0)
    {
        puts ("LED switch on failed.");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    cleanupBoard(board);
    return EXIT_SUCCESS;
}

int
testDAQCBoard ()
{
    config_t config;
    board_t* board;
    uint8_t devId;

    puts ("=================================================");
    puts ("DAQCplate test");
    puts ("=================================================");

    board = NULL;
    for (devId = 0; spi_devices[devId] != NULL; devId++)
    {
        /* Initialize GPIO / SPI addresses for the DAQCplate-1 */
        if (initConfig (spi_devices[devId], 3, 6, 4, 8, &config) == 0)
        {
            /* Initialize all available RELAYplate boards */
            if ((board = initBoard (PP_BOARD_TYPE_DAQC1, DAQC_BOARD_ADDR, &config)))
            {
                fprintf (stdout, "Using SPI device '%s'\n", spi_devices[devId]);
                break;
            }
        }

        /* Initialize GPIO / SPI addresses for the DAQCplate-2 */
        if (initConfig (spi_devices[devId], 3, 6, 4, 32, &config) == 0)
        {
            /* Initialize all available RELAYplate boards */
            if ((board = initBoard (PP_BOARD_TYPE_DAQC2, DAQC_BOARD_ADDR, &config)))
            {
                fprintf (stdout, "Using SPI device '%s'\n", spi_devices[devId]);
                break;
            }
        }
    }

    /* nothing found */
    if (board == NULL)
    {
        puts ("DAQCplate or DAQC2plate not found");
        return EXIT_FAILURE;
    }

    printBoardInfo (board);

    updateLED (board, 0, STATE_OFF);
    updateLED (board, 1, STATE_OFF);

    sleep (1);

    updateLED (board, 0, STATE_TOGGLE);

    sleep (1);

    updateLED (board, 0, STATE_OFF);
    updateLED (board, 1, STATE_TOGGLE);

    uint16_t flags = 0;
    if (getINTflags (board, &flags) < 0)
    {
        puts ("getINTflags() failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("INT flags.........: 0x%04x\n", flags);

    /*
     float range = 0.0f;

     if(getRange(board, 0, 'i', &range) < 0)
     {
     puts("getRange(0, i) failed");
     return EXIT_FAILURE;
     }
     printf("Range(0,i)........: %4.2f\n", range);

     if(getRange(board, 0, 'c', &range) < 0)
     {
     puts("getRange(0, c) failed");
     return EXIT_FAILURE;
     }
     printf("Range(0,c)........: %4.2f\n", range);
     */

    /*	--> Both calls failing ?!
     if (getRange(board, 1, 'i', &range) < 0)
     {
     puts("getRange(1, i) failed");
     return EXIT_FAILURE;
     }
     printf("Range(1,i)........: %4.2f\n", range);

     if (getRange(board, 1, 'c', &range) < 0)
     {
     puts("getRange(1, c) failed");
     return EXIT_FAILURE;
     }
     printf("Range(1,c)........: %4.2f\n", range);
     */

    uint8_t dout = BIT2_STATE_ON | BIT5_STATE_ON;
    if (setDigitalOut (board, dout) < 0)
    {
        puts ("setDigitalOut() failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    uint8_t doutstates = 0;
    if (getDOUTbyte (board, &doutstates) < 0)
    {
        puts ("getDOUTbyte() failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("DOUT states.......: 0x%x\n", doutstates);

    if (enableDINint (board, 2, 'r') < 0)
    {
        puts ("enableDINint() failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    if (enableDINint (board, 4, 'r') < 0)
    {
        puts ("enableDINint() failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }

    uint8_t dinstates = 0;
    if (getDINall (board, &dinstates) < 0)
    {
        puts ("getDINall() failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("DIN states........: 0x%x\n", dinstates);

    float pwm = 0.0f;

    //50%
    if (setPWM (board, 0, 50) < 0)
    {
        puts ("setPWM(0) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    if (getPWM (board, 0, &pwm) < 0)
    {
        puts ("getPWM(0) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("PWM(0)............: %3.2f\n", pwm);

    //70%
    if (setPWM (board, 1, 70) < 0)
    {
        puts ("setPWM(1) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    if (getPWM (board, 1, &pwm) < 0)
    {
        puts ("getPWM(1) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("PWM(1)............: %3.2f\n", pwm);

    float dac = 0.0f;

    // 2.5 Volt
    if (setDAC (board, 0, 2.5f) < 0)
    {
        puts ("setDAC(0) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    if (getDAC (board, 0, &dac) < 0)
    {
        puts ("getDAC(0) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("DAC(0)............: %3.2f\n", dac);

    //4.0 Volt
    if (setDAC (board, 1, 4.0f) < 0)
    {
        puts ("setDAC(1) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    if (getDAC (board, 1, &dac) < 0)
    {
        puts ("getDAC(1) failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    printf ("DAC(1)............: %3.2f\n", dac);

    /* DAQC1plate specific */
    if (board->type == PP_BOARD_TYPE_DAQC1)
    {
        uint8_t swstates = 0;
        if (getSWstate (board, &swstates) < 0)
        {
            puts ("getSWstate() failed");
            cleanupBoard(board);
            return EXIT_FAILURE;
        }
        printf ("SW states.........: 0x%x\n", swstates);
    }

    int i;
    float adcAll[8];

    if (getADCall (board, adcAll, sizeof(adcAll)) < 0)
    {
        puts ("getADCall failed");
        cleanupBoard(board);
        return EXIT_FAILURE;
    }
    for (i = 0; i < 8; i++)
    {
        printf ("ADCall[%d].........: %3.2f\n", i, adcAll[i]);
    }

    float data = 0.0f;
    for (i = 0; i < 9; i++) /* include board Vcc channel=8 */
    {
        if (getADC (board, i, &data) < 0)
        {
            puts ("getADC() failed");
            cleanupBoard(board);
            return EXIT_FAILURE;
        }
        printf ("ADC[%d]............: %3.2f\n", i, data);
    }

    cleanupBoard(board);
    return EXIT_SUCCESS;
}

int
main (void)
{
    int ret;

    /* wiringPI pin layout!! */
    wiringPiSetup ();

    if ((ret = testRelayBoard ()) != EXIT_SUCCESS)
    {
        return ret;
    }

    if ((ret = testDAQCBoard ()) != EXIT_SUCCESS)
    {
        return ret;
    }

    puts ("Finish!");

    return EXIT_SUCCESS;
}
