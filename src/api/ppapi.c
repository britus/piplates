/*
 ============================================================================
 Name        : ppapi.c
 Author      : B. Eschrich
 Version     : 1.00
 Copyright   : Copyright (c) 2016-2017 by B. Eschrich (EoF)
 Description : PI-Plates RELAYplate, DAQCplate and MOTORplate API
 ============================================================================
 */

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <errno.h>
#include <time.h>

// GPIO/SPI library
#include <wiringPi.h>

// Project version informations
#include "../version.h"

// Out SPI bus operations
#include "spiops.h"

// PI Plates specific
#include "ppapi.h"

/*GCC specific: not all static's used here */
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"

// #define TRACE_DAQC_OP

#define PP_BOARD_ADDR(b) (b->config.boardBaseAddr + b->config.address)

typedef struct pp_context_t
{
    /**
     * The SPI device for communication with the PI-Plates
     */
    spidev_t spidev;

    /**
     * The PI-Plate board definition
     */
    board_t board;

    /**
     * Pointer to next board or NULL if this the last
     */
    struct pp_context_t *next;

} pp_context_t;

/* API Version */
static version_t g_version =
{
    .major = PP_MAJOR,
    .minor = PP_MINOR,
    .build = PP_BUILD,
    .revision = PP_REVISION,
};

static pp_context_t *g_list_head = NULL;

/* ========================================================================
 * Internal functions
 * ======================================================================== */

/**
 *
 */
inline static void
pp_verify_ptr (const void* p)
{
    if (p == NULL)
    {
        errno = EFAULT;
        perror (__FILE__ ": NULL pointer access! Terminate process.\n");
        abort ();
    }
}

/**
 *
 */
inline static const uint8_t
pp_is_type (const pp_context_t *ppb, const uint8_t type)
{
    return ppb->board.type == type;
}

/**
 *
 */
inline static const uint8_t
pp_is_daqc (const pp_context_t* ppb)
{
    return pp_is_type (ppb, PP_BOARD_TYPE_DAQC1) ||
           pp_is_type (ppb, PP_BOARD_TYPE_DAQC2);
}

/**
 *
 */
inline static const uint8_t
pp_is_daqc1 (const pp_context_t* ppb)
{
    return pp_is_type (ppb, PP_BOARD_TYPE_DAQC1);
}

/**
 *
 */
inline static const uint8_t
pp_is_daqc2 (const pp_context_t* ppb)
{
    return pp_is_type (ppb, PP_BOARD_TYPE_DAQC2);
}

/**
 *
 */
inline static const uint8_t
pp_is_relay (const pp_context_t* ppb)
{
    return pp_is_type (ppb, PP_BOARD_TYPE_RELAY);
}

/**
 *
 */
inline static const uint8_t
pp_is_motor (const pp_context_t* ppb)
{
    return pp_is_type (ppb, PP_BOARD_TYPE_MOTOR);
}

/**
 *
 */
inline static const uint8_t
pp_verify_type (const board_type_t type)
{
    switch (type)
    {
        case PP_BOARD_TYPE_RELAY:
            return 1;
        case PP_BOARD_TYPE_DAQC1:
            return 1;
        case PP_BOARD_TYPE_DAQC2:
            return 1;
        case PP_BOARD_TYPE_MOTOR:
            return 1;
        default:
            break;
    }
    return 0;
}

/**
 *
 */
inline static const uint8_t
pp_verify_relay (const uint8_t relay)
{
    if ((relay >= 1) && (relay < PP_MAX_RELAYS))
    {
        return 1;
    }
    fprintf (stderr, __FILE__ ": Relay number #%d is out of range. Must be between 1 and 7.\n", relay);
    return 0;
}

/**
 * Returns true if context list is empty
 */
static uint8_t
pp_is_list_empty ()
{
    return g_list_head == NULL;
}

/**
 * Returns the number of context nodes
 */
static int
pp_count_of_contexts ()
{
    int length = 0;
    pp_context_t* node;

    for (node = g_list_head; node != NULL; node = node->next)
    {
        length++;
    }

    return length;
}

/**
 * Creates new context node as first node. Existing context
 * node will be the next one of the created context node.
 * @return A pointer to the new context node
 */
static pp_context_t*
pp_new_context ()
{
    pp_context_t* link = (pp_context_t*) malloc (sizeof(pp_context_t));
    if (link == NULL)
    {
        errno = ENOMEM;
        perror (__FILE__);
        return NULL;
    }

    memset (link, 0, sizeof(pp_context_t));

    /* point to the old first node */
    link->next = g_list_head;

    /* head now the new one */
    g_list_head = link;

    return link;
}

/**
 * Deletes first context node from list and return its pointer
 * and the caller must free the context node. The head becomes
 * the next of the deleted context node.
 * @return Remove context node that must be free
 */
static pp_context_t*
pp_delete_context ()
{
    /* save reference of first context */
    pp_context_t* link = g_list_head;

    /* mark next to first context */
    if (link)
        g_list_head = link->next;

    /* return removed context */
    return link;
}

/**
 * Find given board in the board context list
 * @return The pointer of the context node
 */
static pp_context_t*
pp_find_board (const board_t* board)
{
    pp_context_t* node = g_list_head;

    /* if context list is empty */
    if (node == NULL)
        return NULL;

    pp_verify_ptr (board);

    while (node->board.type != board->type && node->board.config.address != board->config.address)
    {
        /* if node already the last one */
        if (node->next == NULL)
        {
            return NULL;
        }

        /* go to the next context node */
        node = node->next;
    }

    return node;
}

/**
 *
 */
static void
pp_free_context(pp_context_t* context)
{
    if (context)
    {
        spidev_t* dev = &context->spidev;
        if (dev->ops.close) {
            dev->ops.close(dev);
        }
        context->next = NULL;
        free(context);
    }
}

/**
 *
 */
inline static const uint8_t
pp_verify_state (const uint8_t state)
{
    if ((state >= STATE_OFF) && (state <= STATE_ALL))
    {
        return 1;
    }
    fprintf (stderr, __FILE__ ": Object state value #%d is out of range.\n", state);
    return 0;
}

/**
 *
 */
inline static const uint8_t
pp_verify_dig_in (const uint8_t channel)
{
    if ((channel >= 0) && (channel < PP_MAX_DIGITAL_IN))
    {
        return 1;
    }

    fprintf (stderr, __FILE__ ": Digital input channel value out of range. Must be in the range of 0 to 7\n");
    return 0;
}

/**
 *
 */
inline static const uint8_t
pp_verify_analog_in (const uint8_t channel)
{
    if ((channel >= 0) && (channel <= PP_MAX_ANALOG_IN))
    {
        return 1;
    }

    fprintf (stderr, __FILE__ ": Analog input channel value out of range. Must be in the range of 0 to 8\n");
    return 0;
}

/**
 *
 */
inline static const uint8_t
pp_verify_dig_out (const uint8_t channel)
{
    if ((channel >= 0) && (channel < PP_MAX_DIGITAL_OUT))
    {
        return 1;
    }

    fprintf (stderr, __FILE__ ": Digital output channel value out of range. Must be in the range of 0 to 6\n");
    return 0;
}

/**
 *
 */
inline static const uint8_t
pp_verify_led (const uint8_t led, const uint8_t maxleds)
{
    if ((led >= 0) && (led < maxleds))
    {
        return 1;
    }

    fprintf (stderr, __FILE__ ": Invalid LED value. Value must be 0 to %d\n", maxleds-1);
    return 0;
}

/**
 * Wait for DAQC2 ack signal through GPIO BCM=23 wiringPI=4 headerPin=16
 * Note - known collision:
 *   This GPIO also used by WaveShare 4G-HAT-SMI7600x flight mode GPIO
 * @param board Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
inline static const int
pp_wait_daqc2_ack (const pp_context_t* ppb)
{
    struct timespec start;
    struct timespec now;
    uint8_t timedout = 0;

    /* fprintf(stdout, "DAQC2:> [WAIT-DAQC2-ACK]\n"); */

    /* start timer */
    clock_gettime (CLOCK_REALTIME, &start);

    /* wait for acknowledge */
    while (1)
    {
        clock_gettime (CLOCK_REALTIME, &now);

        if (digitalRead (ppb->board.config.pinAckSignal) != 1)
        {
            break;
        }

        if ((now.tv_nsec - start.tv_nsec) > PP_CMD_TIMOUT)
        {
            timedout = 1;
            break;
        }
    }

    if (timedout)
    {
        fprintf(stdout, "DAQC2:> [WAIT-DAQC2-TIMEOUT]\n");
        return -ETIMEDOUT;
    }

    /* fprintf(stdout, "DAQC2:> Got ACK in %ld ns\n", now.tv_nsec - start.tv_nsec); */

    return 0;
}

/**
 * Enable frame signal to transmit commands to the
 * board through the SPI bus.
 * @param board Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
inline static const int
pp_enable_frame (const pp_context_t* ppb)
{
    pp_verify_ptr (ppb);

    // enable SPI frame transfer
    digitalWrite (ppb->board.config.pinFrameControl, HIGH);

    // time to system
    usleep (SPI_IO_DELAY);

    // check bit has raised
    if (!digitalRead (ppb->board.config.pinFrameControl))
    {
        fprintf (stderr, __FILE__ ": digitalWrite() failed!\n");
        return -1;
    }

    return 0;
}

/**
 * Disable frame signal to prevent transmission
 * through the SPI bus.
 * @param board Handle of the PI-Plates board
 * @return 0 success otherwise signal an error
 */
inline static const int
pp_disable_frame (const pp_context_t* ppb)
{
    pp_verify_ptr (ppb);

    // enable SPI frame transfer
    digitalWrite (ppb->board.config.pinFrameControl, LOW);

    // time to system
    usleep (SPI_IO_DELAY);

    // check bit has released
    if (digitalRead (ppb->board.config.pinFrameControl))
    {
        fprintf (stderr, __FILE__ ": digitalWrite() failed!\n");
        return -1;
    }

    return 0;
}

inline static const int
pp_daqc2_fetch(const spidev_t* spidev, uint8_t* result, const size_t size, const uint8_t stopAt0x0)
{
    uint8_t* buf;
    int idx, ret;
    uint8_t csbyte = 0;
    int csum = 0;
    size_t bread = 0;

    /* for string terminator to get checksum byte */
    uint8_t mark = 0;

    /* fetch checksum byte too (one byte more as origin size) */
    size_t nsize = size + 1;

    if ((buf = (uint8_t*) malloc(nsize)) == NULL)
    {
        return -ENOMEM;
    }

    memset(buf, 0, nsize);

    /* fetch response bytewise */
    for (idx = 0; idx < nsize; idx++)
    {
        if ((ret = spidev->ops.fetch (spidev, &buf[idx], 1)))
        {
            free(buf);
            return ret;
        }

        /* response is a string - terminate fetch after checksum byte */
        if (stopAt0x0 && buf[idx] == 0)
        {
            mark = 1;
            bread = idx;
            continue;
        }

        /* exclude CS byte */
        if (idx < size && !mark)
        {
            csum += buf[idx];
            bread = idx;
        }
        else
        {
            csbyte = buf[idx];
        }

        /* got cs byte in case of a string */
        if (mark)
        {
            break;
        }
    }

    /* adjust bytes read */
    bread++;

    int v1 = (~csbyte & 0xFF);
    int v2 = (csum & 0xFF);
    if (v1 != v2)
    {
        fprintf(stderr, "DAQC2:> CHECKSUM failed: v1(~%02X & 0xFF)=%02X v2(%02X & 0xFF)=%X\n",
                csbyte, v1, csum, v2);
        fprintf(stdout, "DAQC2:> cmd_rsp_space=%d | bread=%d | csbyte=0x%02X | csum=%d\n",
                size, bread, csbyte, csum);
        spidev->ops.dump(buf, nsize, 16, "DAQC2:");
        free(buf);
        return -EINVAL;
    }

#ifdef TRACE_DAQC_OP
    fprintf(stdout, "DAQC2:> Got data - cSum=%d csByte=0x%02X bread=%d\n", csum, csbyte, bread);
    spidev->ops.dump(buf, nsize, 16, "DAQC2:");
#endif

    /* copy as needed */
    bread = (size > bread ? bread : size);
    memcpy(result, buf, bread);

    /*fprintf(stdout, "DAQC2:> CHECKSUM OK\n");*/

    free(buf);
    return 0;
}

/**
 *
 */
static int
pp_send_command (const pp_context_t* ppb, const board_command_t* command)
{
    const spidev_t *spidev;
    board_command_t daqc2cmd;
    int ret = 0;

    pp_verify_ptr (ppb);
    pp_verify_ptr (command);

    spidev = &ppb->spidev;

    if (pp_enable_frame (ppb))
    {
        fprintf (stderr, __FILE__ ":pp_send_command(): enable spi frame failed.\n");
        return -1;
    }

    if (!pp_is_daqc2 (ppb))
    {
        if ((ret = spidev->ops.xfer (spidev, command)) != 0)
        {
            fprintf (stderr, __FILE__ ":pp_send_command(): spi xfer failed.\n");
        }
    }
    else
    {
        memcpy(&daqc2cmd, command, sizeof(board_command_t));
        daqc2cmd.size = 0;    /* don't fetch any response!*/
        daqc2cmd.result = 0;  /* don't fetch any response!*/

#ifdef TRACE_DAQC_OP
        fprintf(stdout, "DAQC2:> address=0x%02X | command=0x%02X | param1=%d | param2=%d\n",
                daqc2cmd.address, daqc2cmd.command, daqc2cmd.param1, daqc2cmd.param2);
#endif

        if ((ret = spidev->ops.xfer (spidev, &daqc2cmd)))
        {
            fprintf (stderr, __FILE__ ":pp_send_command(): spi xfer failed.\n");
        }

        /* if response in origin cmd requested, now fetch it */
        if (ret == 0 && command->size > 0 && command->result)
        {
            if (pp_wait_daqc2_ack(ppb) == 0)
            {
                if ((ret = pp_daqc2_fetch(spidev, command->result, command->size, command->stopAt0x0)))
                {
                    fprintf (stderr, __FILE__ ":pp_send_command(): pp_daqc2_fetch() failed.\n");
                }
            }
        }
    }

    if (pp_disable_frame (ppb))
    {
        fprintf (stderr, __FILE__ ":pp_send_command(): disable spi frame failed.\n");
        return -1;
    }

    return ret;
}

/* ========================================================================
 * Generic PI-Plates functions
 * ======================================================================== */

/**
 * Print board informations for given PI-Plates board.
 * @param board The board handle
 */
void
printBoardInfo (const board_t* board)
{
    pp_verify_ptr (board);

    char id[64] = "";
    char revision[10] = "";
    int wmajor, wminor, wgpioly;
    const version_t* ppcver = getAPIVersion();
    printf ("\n");
    printf ("[Pi-Plates API] ---------------------------------\n");
    printf ("C API version.....: %d.%d.%d (%d)\n\n",
            ppcver->major, ppcver->minor, ppcver->build, ppcver->revision);
    printf ("[GPIO config] -----------------------------------\n");
    printf ("Interrupt pin.....: %d\n", board->config.pinInterrupt);
    printf ("Frame control pin.: %d\n", board->config.pinFrameControl);
    printf ("Acknowledge pin...: %d\n", board->config.pinAckSignal);
    printf ("Board base address: %X\n", board->config.boardBaseAddr);
    printf ("Board address.....: %X\n", PP_BOARD_ADDR(board));
    printf ("SPI device........: %s\n\n", board->config.spidev);
    printf ("[Board specific] --------------------------------\n");
    printf ("Type..............: %d\n", board->type);
    printf ("Address...........: %d\n", board->config.address);

    if (getID (board, id, sizeof(id)-1) == 0)
    {
        printf ("Board ID..........: %s\n", id);
    }
    if (getHWRevision (board, revision, sizeof(revision)) == 0)
    {
        printf ("Hardware Revision.: %s\n", revision);
    }
    if (getFWRevision (board, revision, sizeof(revision)) == 0)
    {
        printf ("Firmware Revision.: %s\n", revision);
    }
    if (board->type == PP_BOARD_TYPE_DAQC1 || board->type == PP_BOARD_TYPE_DAQC2)
    {
        if (readBoardVcc (board) == 0)
        {
            printf ("Board Vcc.........: %2.2f Volt\n", board->vcc);
        }
    }
    printf ("\n");
}

/**
 * Print C API version string
 */
void
printAPIVersion ()
{
    const version_t* v = getAPIVersion ();
    printf ("PI-Plates C API %d.%d %d-%d - Copyright (c) 2016-2020 by EoF Systems\n\n",
            v->major, v->minor, v->build, v->revision);
}

/**
 * Retrieve C API version information
 */
const version_t*
getAPIVersion ()
{
    return &g_version;
}

/**
 * Initialize GPIO/SPI configuration structure to communicate
 * with the PI-Plates boards.
 * Note that the pin numbers follows your wiringPi GPIO layout
 * initialization i.e. wiringPiSetup() or wiringPiGPIOSetup()
 * @param spidev Use constant PP_SPI_IO_CHANNEL for default
 * @param gpioINT Interrupt signal pin (wiringPi=3)
 * @param gpioFrame SPI frame signal pin (wiringPi=6)
 * @param gpioAck DAQC2 acknowledge signal pin (wiringPi=4)
 * @param boardBaseAddr The PI-Plates board address
 * @param config Pointer to the configuration structure
 * @return 0 if succsess otherwise signal an error
 */
int
initConfig (const char* spidev, const uint8_t gpioINT, const uint8_t gpioFrame, const uint8_t gpioAck, const uint8_t boardBaseAddr, config_t* config)
{
    pp_verify_ptr (config);
    config->spidev = spidev;
    config->pinInterrupt = gpioINT;
    config->pinFrameControl = gpioFrame;
    config->pinAckSignal = gpioAck;
    config->boardBaseAddr = boardBaseAddr;
    config->state = 0xbe;
    return 0;
}

/**
 * Initialize the PI-Plate boards by specified board type. Each available board becomes
 * a board_t handle allocated in a global board list. You must call initConfig(...) first
 * to get the configuration for the board GPIO/SPI communication.
 * @param type One of the predefied board types (RELAY=1, DAQC=2 or MOTOR=3)
 * @return 0 if succsess otherwise signal an error
 */
board_t*
initBoard (uint8_t type, const uint8_t address, const config_t* config)
{
    uint8_t index, addr;
    pp_context_t* context;

    /* sane check */
    pp_verify_ptr (config);

    if (config->state != 0xbe)
    {
        fprintf (stderr, __FILE__ ": You must call 'initConfig()' first.\n");
        return NULL;
    }

    if (pp_verify_type (type) == 0)
    {
        fprintf (stderr, __FILE__ ": Invalid PI-Plates board type\n");
        return NULL;
    }

    /* Initialize GPIO for SPI frame control */
    if (config->pinFrameControl == 0)
    {
        fprintf (stderr, __FILE__ ": GPIO pin for SPI frame control is required.\n");
        return NULL;
    }

    pinMode (config->pinFrameControl, OUTPUT);
    digitalWrite (config->pinFrameControl, LOW);

    /* Initialize interrupt control */
    if (config->pinInterrupt)
    {
        pinMode (config->pinInterrupt, INPUT);
        pullUpDnControl (config->pinInterrupt, PUD_UP);
    }

    /* Initialize acknowledge signal for DAQC2 */
    if (config->pinAckSignal)
    {
        pinMode (config->pinAckSignal, INPUT);
        pullUpDnControl (config->pinAckSignal, PUD_UP);
    }

    /* time to system */
    usleep (SPI_IO_DELAY);

    /* create context node */
    context = pp_new_context ();
    if (context == NULL)
        return NULL;

    /* setup PI-Plates board context structure */
    memcpy (&context->board.config, config, sizeof(config_t));
    context->board.config.address = address;
    context->board.type = type;

    /* initialize SPI bus access ...*/
    if (spi_init (config->spidev, &context->spidev))
    {
        pp_free_context (pp_delete_context ());
        return NULL;
    }

    /* search board with specific type and address address */
    if (getAddress (&context->board, &addr))
    {
        fprintf (stderr, __FILE__ ": Unable to get board address.\n");
        pp_free_context (pp_delete_context ());
        return NULL;
    }

    if ((addr - config->boardBaseAddr) != address)
    {
        fprintf (stderr, __FILE__ ": Unknown hardware found at %s addr=%X boardBaseAddr=%X address=%X\n",
                 context->spidev.path, addr, config->boardBaseAddr, config->address);
        pp_free_context (pp_delete_context ());
        return NULL;
    }

    /* reset the board */
    if (reset (&context->board))
    {
        fprintf (stderr, __FILE__ ": Board reset failed\n");
        pp_free_context (pp_delete_context ());
        return NULL;
    }

    /* DAQC1/DAQC2 specific initialization */
    if (pp_is_daqc (context))
    {

#ifdef TRACE_DAQC_OP
        fprintf(stdout, "DAQCx:> Read calibration data.....\n");
#endif

        if (readCalibration (&context->board))
        {
            fprintf (stderr, __FILE__ ": Read calibration data failed\n");
            pp_free_context (pp_delete_context ());
            return NULL;
        }

#ifdef TRACE_DAQC_OP
        fprintf(stdout, "DAQCx:> Calibration data loaded.\n");
        context->spidev.ops.dump(context->board.cal_data, sizeof(context->board.cal_data), 16, "CALDAT:");
#endif

        /* set default VCC value */
        context->board.vcc = 0;

        /* wait for power up VCC value */
        while (1)
        {
            if (readBoardVcc (&context->board))
            {
                fprintf (stderr, __FILE__ ": Read board Vcc failed\n");
                pp_free_context (pp_delete_context ());
                return NULL;
            }
            if (context->board.vcc > 3.0f)
            {
                break;
            }
        }

        setDigitalOut(&context->board, 0);
        setPWM(&context->board, 0, 0.0f);
        setPWM(&context->board, 1, 0.0f);
        setDAC(&context->board, 0, 0.0f);
        setDAC(&context->board, 1, 0.0f);
    }

    return &context->board;
}

/**
 * Releases given PI-Plates board
 * @param board The PI-Plates board handle
 */
void
cleanupBoard(const board_t* board)
{
    pp_context_t* node;
    pp_context_t* ctx;

    if (board && (ctx = pp_find_board(board)))
    {
        /* unlink given board from context list */
        if (ctx == g_list_head)
        {
            g_list_head = ctx->next;
        }
        else for (node = g_list_head; node != NULL; node = node->next)
        {
            if (node->next == ctx)
            {
                node->next = ctx->next;
                break;
            }
        }

        pp_free_context (ctx);
    }
}

/**
 * Retrieve the SPI board address. To test a valid board address
 * substract *pAddress - board->config.boardBaseAddress. The result
 * must the same as board->address. That indicate that the given board
 * address in the address field of the board_t structure is valid.
 * @param board Handle of the PI-Plates board
 * @param pAddress Pointer to retrieve the SPI board address
 * @return 0 success otherwise signal an error
 */
int
getAddress (const board_t* board, uint8_t* pAddress)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return 0;

    /* reset return */
    pp_verify_ptr (pAddress);
    *pAddress = 0;

    board_command_t bc = {
        .address = PP_BOARD_ADDR(board),
        .command = 0x00,
        .param1 = 0,
        .param2 = 0,
        .result = pAddress,
        .size = sizeof(uint8_t),
        .stopAt0x0 = 0,
    };

    return pp_send_command (ctx, &bc);
}

/**
 *
 */
int
reset (const board_t* board)
{
    pp_context_t* ctx;
    int ret;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x0F,
       .param1 = 0,
       .param2 = 0,
       .size = 0,
    };

    ret = pp_send_command (ctx, &bc);

    /* wait power up */
    if (ret == 0) {
        usleep (1000000);
    }

    return ret;
}

/**
 *
 */
int
getHWRevision (const board_t* board, char* revision, const size_t size)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (revision);
    memset (revision, 0, size);

    if (size < 10)
    {
        fprintf (stderr, "Buffer size %d to small to return revision.\n", (int) size);
        return -1;
    }

    uint8_t value = 0;
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x02,
       .param1 = 0,
       .param2 = 0,
       .result = &value,
       .size = sizeof(uint8_t),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    float whole = (float) (value >> 4);
    float point = (float) (value & 0x0F);
    float rev = whole + point / 10;
    snprintf (revision, size, "%2.2f", rev);

    return 0;
}

/**
 *
 */
int
getFWRevision (const board_t* board, char* revision, const size_t size)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (revision);
    memset (revision, 0, size);

    if (size < 10)
    {
        fprintf (stderr, "Buffer size %d to small to return revision.\n", (int) size);
        return -1;
    }

    uint8_t value = 0;
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x03,
       .param1 = 0,
       .param2 = 0,
       .result = &value,
       .size = sizeof(uint8_t),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    float whole = (float) (value >> 4);
    float point = (float) (value & 0x0F);
    float rev = whole + point / 10;
    snprintf (revision, size, "%2.2f", rev);

    return 0;
}

/**
 *
 */
int
getID (const board_t* board, char* id, const size_t size)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (id);
    memset (id, 0, size);

    if (size < 21)
    {
        fprintf (stderr, __FILE__ ": Buffer size %d to small to return the ID.\n", (int) size);
        return -1;
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x01,
       .param1 = 0,
       .param2 = 0,
       .result = (uint8_t*)id,
       .size = size,
       .stopAt0x0 = 1,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/* ========================================================================
 * PI-Plates - LED functions
 * ======================================================================== */

inline static const uint8_t
pp_led_state_to_command(const uint8_t state)
{
    switch (state)
    {
        case STATE_ON:
        {
            return 0x60;
        }
        case STATE_OFF:
        {
            return 0x61;
        }
        case STATE_TOGGLE:
        {
            return 0x62;
        }
        default:
        {
            fprintf (stderr, __FILE__ ": Invalid state value %d.\n", state);
            return 0;
        }
    }

    return 0;
}

static const int
pp_update_daqc1_led(pp_context_t* ctx, const uint8_t led, const uint8_t state)
{
    const board_t* board = &ctx->board;
    uint8_t command;

    if (pp_verify_led (led, 2) == 0)
    {
        return -1;
    }

    command = pp_led_state_to_command(state);

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = command,
       .param1 = led,
       .param2 = 0,
       .size = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

static const int
pp_update_daqc2_led(pp_context_t* ctx, const uint8_t led, const uint8_t state)
{
    const board_t* board = &ctx->board;
    uint8_t ledval = (state == STATE_OFF ? 0 : led);

    if (pp_verify_led (led, 8) == 0)
    {
        return -1;
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x60,
       .param1 = ledval,
       .param2 = 0,
       .size = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

static const int
pp_update_relay_led(pp_context_t* ctx, const uint8_t led, const uint8_t state)
{
    const board_t* board = &ctx->board;
    uint8_t command;

    if (pp_verify_led (led, 1) == 0)
    {
        return -1;
    }

    command = pp_led_state_to_command(state);

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = command,
       .param1 = led,
       .param2 = 0,
       .size = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int
updateLED (const board_t* board, const uint8_t led, const uint8_t state)
{
    int ret = 0;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_verify_state (state) == 0)
    {
        return -1;
    }

    switch (board->type)
    {
        case PP_BOARD_TYPE_RELAY:
            ret = pp_update_relay_led(ctx, led, state);
            break;
        case PP_BOARD_TYPE_DAQC1:
            ret = pp_update_daqc1_led(ctx, led, state);
            break;
        case PP_BOARD_TYPE_DAQC2:
            ret = pp_update_daqc2_led(ctx, led, state);
            break;
        default:
            ret = -1;
            break;
    }

    return ret ;
}

/**
 *
 */
int
getLEDState (const board_t* board, const uint8_t led, uint8_t* state)
{
    pp_context_t* ctx;
    uint8_t maxleds;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (state);
    *state = 0;

    switch (board->type)
    {
        case PP_BOARD_TYPE_RELAY:
            maxleds = 1;
            break;
        case PP_BOARD_TYPE_DAQC1:
            maxleds = 2;
            break;
        case PP_BOARD_TYPE_DAQC2:
            maxleds = 8;
            break;
        default:
            maxleds = 1;
            break;
    }

    if (pp_verify_led (led, maxleds) == 0)
    {
        return -1;
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x63,
       .param1 = led,
       .param2 = 0,
       .result = state,
       .size = sizeof(uint8_t),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/* ========================================================================
 * PI-Plates - RELAYplate functions
 * ======================================================================== */

/**
 *
 */
int
updateRelay (const board_t* board, const uint8_t relay, const uint8_t state)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (!pp_is_relay (ctx))
    {
        return -1;
    }
    if (pp_verify_state (state) == 0)
    {
        return -1;
    }
    if (state != STATE_ALL)
    {
        if (pp_verify_relay (relay) == 0)
        {
            return -1;
        }
    }
    else if (relay > 127)
    {
        fprintf (stderr, "Relay argument #%d out of range. Must be between 0 and 127\n", relay);
        return 0;
    }

    int command = 0x00;

    switch (state)
    {
        case STATE_OFF:
        {
            command = 0x11;
            break;
        }
        case STATE_ON:
        {
            command = 0x10;
            break;
        }
        case STATE_TOGGLE:
        {
            command = 0x12;
            break;
        }
        case STATE_ALL:
        {
            command = 0x13;
            break;
        }
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = command,
       .param1 = relay,
       .param2 = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int
getRelayState (const board_t* board, uint8_t* state)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (!pp_is_relay (ctx))
    {
        return -1;
    }

    // reset return
    pp_verify_ptr (state);
    *state = 0;

    uint8_t result = 0;
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x14,
       .param1 = 0,
       .param2 = 0,
       .result = &result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    // return relay states
    *state = result;

    return 0;
}

/**
 *
 */
int
relayON (const board_t* board, const uint8_t relay)
{
    return updateRelay (board, relay, STATE_ON);
}

/**
 *
 */
int
relayOFF (const board_t* board, const uint8_t relay)
{
    return updateRelay (board, relay, STATE_OFF);
}

/**
 *
 */
int
toggleRelay (const board_t* board, const uint8_t relay)
{
    return updateRelay (board, relay, STATE_TOGGLE);
}

/**
 *
 */
int
updateRelays (const board_t* board, const uint8_t mask)
{
    return updateRelay (board, mask, STATE_ALL);
}

/* ========================================================================
 * PI-Plates - DAQCplate functions
 * ======================================================================== */

/**
 *
 */
int
getProgMemory (const board_t* board, const uint32_t address, char* data, const size_t size)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (data);
    memset (data, 0, size);

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    if (size < 3)
    {
        fprintf (stderr, "Buffer size %d to small to return the data value.\n", (int) size);
        return -1;
    }

    uint8_t result[2];
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x0F,
       .param1 = address >> 8,
       .param2 = address & 0xff,
       .result = result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    int value = (256 * result[0] + result[1]);
    snprintf (data, size, "%02X", value);

    return strlen (data);
}

//=======================================
// DAQC2 flash memory functions
//=======================================

int
getCalByte (const board_t* board, const uint8_t ptr, uint8_t* data)
{
    uint8_t result = 0;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (!pp_is_daqc2 (ctx))
    {
        return -1;
    }

    if (ptr > 0xff)
    {
        fprintf (stderr, "Calibration pointer is out of range. Must be in the range of 0 to 255\n");
        return -1;
    }

    // reset return
    pp_verify_ptr(data);
    (*data) = 0;

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0xFD,
       .param1 = 2,
       .param2 = ptr,
       .result = &result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    (*data) = result;
    return 0;
}

static int
pp_daqc2_get_calibration(pp_context_t* ctx)
{
    int ret;
    uint8_t i, j;
    uint8_t values[6];
    uint16_t csign;
    board_t* wrb;

    /* mutable */
    wrb = (board_t*) &ctx->board;

    for (i=0; i < PP_MAX_CAL_DATA; i++)
    {
        for (j=0; j < 6; j++)
        {
            if ((ret = getCalByte(&ctx->board, 6 * i + j, &values[j])))
            {
                return ret;
            }
        }

        /* -------------------------------------------------------------- *
         * 16 bit signed slope calibration values - range is +/-4%        *
         * -------------------------------------------------------------- */

        /* get signed bit */
        csign = values[0] & 0x80;

        wrb->cal_data[i].scale = 0.04 * ((values[0] & 0x7F) * 256 + values[1]) / 32767;
        if (csign != 0)
        {
            wrb->cal_data[i].scale *= -1;
        }
        wrb->cal_data[i].scale += 1;

        /* -------------------------------------------------------------- *
         * 16 bit signed offset calibration values - range is +/- 0.1     *
         * -------------------------------------------------------------- */

        /* get signed bit */
        csign = values[2] & 0x80;

        wrb->cal_data[i].offset = 0.2 * ((values[2] & 0x7F) * 256 + values[3]) / 32767;
        if (csign != 0)
        {
             wrb->cal_data[i].offset *= -1;
        }

        /* -------------------------------------------------------------- *
         * 16 bit signed DAC calibration values - range is +/-4%          *
         * -------------------------------------------------------------- */

        /* get signed bit */
        csign = values[4] & 0x80;

        wrb->cal_data[i].value = 0.04 * ((values[4] & 0x7F) * 256 + values[5]) / 32767;
        if (csign != 0)
        {
            wrb->cal_data[i].value *= -1;
        }
        wrb->cal_data[i].value += 1;
    }

    return 0;
}

/* simulated calibration values for DAQC1. This
 * can be used to set special adjust factor per
 * analog channel too.
 */
static int
pp_daqc1_get_calibration(pp_context_t* ctx)
{
    int ret;
    uint8_t i;
    board_t* wrb;

    /* mutable */
    wrb = (board_t*) &ctx->board;
    for (i=0; i < PP_MAX_CAL_DATA; i++)
    {
        /* this part must be 1 if its zero */
        if (wrb->cal_data[i].scale == 0)
            wrb->cal_data[i].scale = 1;
        if (wrb->cal_data[i].value == 0)
            wrb->cal_data[i].value = 1;
        /* this must be always zero */
        wrb->cal_data[i].offset = 0;
    }

    return 0;
}

/**
 * Reads the calibration data from DAQC2plate flash memory
 */
int
readCalibration (const board_t* board)
{
    int ret = 0;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    switch (board->type)
    {
        case PP_BOARD_TYPE_DAQC1:
        {
            ret = pp_daqc1_get_calibration(ctx);
            break;
        }
        case PP_BOARD_TYPE_DAQC2:
        {
            ret = pp_daqc2_get_calibration(ctx);
            break;
        }
        default:
        {
            ret--;
            break;
        }
    }

    return ret;
}

//=======================================
// ADC Functions
//=======================================

inline static const float
pp_adc_to_phy(pp_context_t* ctx, const uint8_t channel, const uint8_t value1, const uint8_t value2)
{
    daqc_cal_data_t* cal = ctx->board.cal_data;
    float result = (256 * value1 + value2);

    switch (ctx->board.type)
    {
        case PP_BOARD_TYPE_DAQC1:
        {
            result = (result * 4.096f) / 1024;
            /* dedicated board Vcc channel */
            if (channel == 8)
            {
                result = result * 2.0;
            }
            else {
                result = result * cal[channel].scale + cal[channel].offset;
            }

            break;
        }

        case PP_BOARD_TYPE_DAQC2:
        {
            /* COMPAT: dedicated board Vcc channel */
            if (channel == 8)
            {
                result = result * 5.0 * 2.4 / 65536;
            }
            else
            {
                result = (result * 24.0 / 65536) - 12.0;
                result = result * cal[channel].scale + cal[channel].offset;
            }

            break;
        }
        default:
        {
            result = 0;
            break;
        }
    }

    return result;
}

/**
 *
 */
int
getADC (const board_t* board, const uint8_t channel, float* data)
{
    float value;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (data);
    (*data) = 0;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }
    if (pp_verify_analog_in (channel) == 0)
    {
        return -1;
    }

    uint8_t result[2] = { 0, 0 };
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x30,
       .param1 = channel,
       .param2 = 0,
       .result = result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    (*data) = pp_adc_to_phy(ctx, channel, result[0], result[1]);

    return 0;
}

/**
 *
 */
int
getADCall (const board_t* board, float data[], const size_t size)
{
    uint8_t channel;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    // reset return
    pp_verify_ptr (data);
    memset (data, 0, size);

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    if (size < (sizeof(float) * 8))
    {
        fprintf (stderr, "ERROR: Parameter data must be an array of float with 8 elements.\n");
        return -1;
    }

    uint8_t result[16] = {};
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x31,
       .param1 = 0,
       .param2 = 0,
       .result = result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    for (channel = 0; channel < 8; channel++)
    {
        data[channel] = pp_adc_to_phy(ctx, channel, result[2 * channel], result[2 * channel + 1]);
    }

    return 0;
}

//=======================================
// Digital Input Functions
//=======================================

/**
 *
 */
int
getDINbit (const board_t* board, const uint8_t bit, uint8_t* state)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    if (pp_verify_dig_in (bit) == 0)
    {
        return -1;
    }

    pp_verify_ptr (state);
    (*state) = 0;

    uint8_t result = 0;
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x20,
       .param1 = bit,
       .param2 = 0,
       .result = &result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    (*state) = result;
    return 0;
}

/**
 *
 */
int
getDINall (const board_t* board, uint8_t* states)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr(states);
    (*states) = 0;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x25,
       .param1 = 0,
       .param2 = 0,
       .result = states,
       .size = sizeof(uint8_t),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    // Digital input must be set to GND to signal an input.
    // If noting is set all 8 bits are high. So we negate.
    uint8_t value = (*states);
    (*states) = ~value;

    return 0;
}

/**
 *
 */
int
enableDINint (const board_t* board, const uint8_t bit, const unsigned char edge)
{
    uint8_t cmd;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }
    if (pp_verify_dig_in (bit) == 0)
    {
        return -1;
    }

    switch (edge)
    {
        case 'f':
        case 'F':
        {
            cmd = 0x21;
            break;
        }
        case 'r':
        case 'R':
        {
            cmd = 0x22;
            break;
        }
        default:
        {
            cmd = 0x23;
            break;
        }
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = cmd,
       .param1 = bit,
       .param2 = 0,
       .result = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int
disableDINint (const board_t* board, const uint8_t bit)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }
    if (pp_verify_dig_in (bit) == 0)
    {
        return -1;
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x24,
       .param1 = bit,
       .param2 = 0,
       .result = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

//=======================================
// Hybrid Input Functions
//=======================================

/**
 *
 */
int
getRange (const board_t* board, const uint8_t channel, const unsigned char units, float* data)
{
    pp_context_t* ctx;
    const spidev_t* spidev;
    int rc;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr (data);
    *data = 0.0f;

    if (pp_is_daqc (ctx) == 0)
    {
        return -1;
    }

    const unsigned char c = tolower (units);
    if (c != 'i' && c != 'c')
    {
        fprintf (stderr, __FILE__ ": Incorrect units parameter. Must be 'c' or 'i'.\n");
        return -1;
    }

    spidev = &ctx->spidev;

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x80,
       .param1 = channel,
       .param2 = 0,
       .result = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_enable_frame (ctx) != 0)
    {
        return -1;
    }

    if ((rc = ctx->spidev.ops.xfer (spidev, &bc)) < 0)
    {
        pp_disable_frame (ctx);
        return rc;
    }

    usleep (70000);

    uint8_t result[2] = { 0, 0 };

    bc.command = 0x81;
    bc.result = result;
    bc.size = sizeof(result);

    if ((rc = ctx->spidev.ops.xfer (spidev, &bc)) < 0)
    {
        pp_disable_frame (ctx);
        return rc;
    }

    if (pp_disable_frame (ctx) != 0)
    {
        return -1;
    }

    float range = result[0] * 256 + result[1];
    if (range == 0.0f)
    {
        fprintf (stderr, __FILE__ ": Sensor #%d failue.\n", channel);
        return -1;
    }

    if (c == 'c')
    {
        range = range / 58.326;
    }
    else if (c == 'i')
    {
        range = range / 148.148;
    }

    *data = round (range);
    usleep (SPI_IO_DELAY * 1000);

    return 0;
}

//=======================================
// Switch Functions
//=======================================

/**
 *
 */
int
getSWstate (const board_t* board, uint8_t* state)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr (state);
    (*state) = 0;

    if (pp_is_daqc1(ctx))
    {
        uint8_t result = 0;
        board_command_t bc = {
           .address = PP_BOARD_ADDR(board),
           .command = 0x50,
           .param1 = 0,
           .param2 = 0,
           .result = &result,
           .size = sizeof(result),
           .stopAt0x0 = 0,
        };

        if (pp_send_command (ctx, &bc) != 0)
        {
            return -1;
        }

        // return state value
        (*state) = result;
    }

    return 0;
}

/**
 *
 */
int
enableSWint (const board_t* board)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc1(ctx))
    {
        board_command_t bc = {
           .address = PP_BOARD_ADDR(board),
           .command = 0x51,
           .param1 = 0,
           .param2 = 0,
           .result = 0,
           .size = 0,
           .stopAt0x0 = 0,
        };

        if (pp_send_command (ctx, &bc) != 0)
        {
            return -1;
        }
    }

    return 0;
}

/**
 *
 */
int
disableSWint (const board_t* board)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc1(ctx))
    {
        board_command_t bc = {
           .address = PP_BOARD_ADDR(board),
           .command = 0x52,
           .param1 = 0,
           .param2 = 0,
           .result = 0,
           .size = 0,
           .stopAt0x0 = 0,
        };

        if (pp_send_command (ctx, &bc) != 0)
        {
            return -1;
        }

    }

    return 0;
}

/**
 *
 */
int
enableSWpower (const board_t* board)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc1(ctx))
    {
        board_command_t bc = {
           .address = PP_BOARD_ADDR(board),
           .command = 0x53,
           .param1 = 0,
           .param2 = 0,
           .result = 0,
           .size = 0,
           .stopAt0x0 = 0,
        };

        if (pp_send_command (ctx, &bc) != 0)
        {
            return -1;
        }
    }

    return 0;
}

/**
 *
 */
int
disableSWpower (const board_t* board)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc1(ctx))
    {
        board_command_t bc = {
           .address = PP_BOARD_ADDR(board),
           .command = 0x54,
           .param1 = 0,
           .param2 = 0,
           .result = 0,
           .size = 0,
           .stopAt0x0 = 0,
        };

        if (pp_send_command (ctx, &bc) != 0)
        {
            return -1;
        }
    }

    return 0;
}

// ============================================
// Digital Output Functions
// ============================================

/**
 *
 */
int
updateDOUT (const board_t* board, const uint8_t bit, const uint8_t state)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }
    if (pp_verify_state (state) == 0)
    {
        return -1;
    }
    if (state != STATE_ALL)
    {
        if (pp_verify_dig_out (bit) == 0)
        {
            return -1;
        }
    }
    else if (bit > 127)
    {
        fprintf (stderr, __FILE__ ": Digital output value #%d out of range. Must be between 0 and 127\n", bit);
        return 0;
    }

    int command = 0x00;
    switch (state)
    {
        case STATE_OFF:
        {
            command = 0x11;
            break;
        }
        case STATE_ON:
        {
            command = 0x10;
            break;
        }
        case STATE_TOGGLE:
        {
            command = 0x12;
            break;
        }
        case STATE_ALL:
        {
            command = 0x13;
            break;
        }
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = command,
       .param1 = bit,
       .param2 = 0,
       .size = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int
digitalOutON (const board_t* board, const uint8_t bit)
{
    return updateDOUT (board, bit, STATE_ON);
}

/**
 *
 */
int
digitalOutOFF (const board_t* board, const uint8_t bit)
{
    return updateDOUT (board, bit, STATE_OFF);
}

/**
 *
 */
int
digitalOutToggle (const board_t* board, const uint8_t bit)
{
    return updateDOUT (board, bit, STATE_TOGGLE);
}

/**
 *
 */
int
setDigitalOut (const board_t* board, const uint8_t bitMask)
{
    return updateDOUT (board, bitMask, STATE_ALL);
}

/**
 *
 */
int
getDOUTbyte (const board_t* board, uint8_t* value)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr (value);
    (*value) = 0;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    uint8_t result = 0;
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x14,
       .param1 = 0,
       .param2 = 0,
       .result = &result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    (*value) = result;
    return 0;
}

// ============================================
// Frequency Counter Functions (DAQC2 only)
// ============================================

// ============================================
// PWM and DAC Output Functions
// ============================================

static int
pp_daqc1_set_pwm (const pp_context_t* ctx, const uint8_t channel, float value)
{
    const board_t* board = &ctx->board;
    uint8_t hibyte, lobyte;
    uint16_t regval;

    regval = round (value * PP_MAX_DAC_BITRES / 100);
    hibyte = regval >> 8;
    lobyte = regval - (hibyte << 8);

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x40 + channel,
       .param1 = hibyte,
       .param2 = lobyte,
       .result = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

static int
pp_daqc2_set_pwm (const pp_context_t* ctx, const uint8_t channel, float value)
{
    const board_t* board = &ctx->board;
    uint16_t regval;

    regval = (uint16_t) (value * PP_MAX_DAC_BITRES / 100 + 0.5);

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0xC1,
       .param1 = (channel << 7) + (regval >> 8),
       .param2 = regval & 0xFF,
       .result = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

int
setPWM (const board_t* board, const uint8_t channel, float value)
{
    int ret;
    uint16_t regval;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    /* DAQC1 & DAQC2 = 2 */
    if (!(channel >= 0 && channel < PP_MAX_PWM_CHANNELS))
    {
        fprintf (stderr, __FILE__ ": PWM channel %d must be 0 or 1\n", channel);
        return -1;
    }

    /* value in percent */
    if (value < 0 || value > 100)
    {
        fprintf (stderr, __FILE__ ": PWM value %3.0f out of range - must be between 0 and 100%%\n", value);
        return -1;
    }

    switch (board->type)
    {
        case PP_BOARD_TYPE_DAQC1:
        {
            ret = pp_daqc1_set_pwm(ctx, channel, value);
            break;
        }

        case PP_BOARD_TYPE_DAQC2:
        {
            if ((ret = pp_daqc2_set_pwm (ctx, channel, value)) == 0)
            {
                ((board_t*) board)->pwm_data[channel].pwm = value;
            }
            break;
        }

        default: {
            ret = -1;
        }
    }

    return 0;
}

static int
pp_daqc1_get_pwm (const pp_context_t* ctx, const uint8_t channel, float* data)
{
    const board_t* board = &ctx->board;
    uint8_t result[2] = {};
    float value;

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x40 + channel + 2,
       .param1 = 0,
       .param2 = 0,
       .result = result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    value = 256 * result[0] + result[1];

    /* make percent value */
    (*data) = round((value * 100) / PP_MAX_DAC_BITRES);

    return 0;
}

/**
 *
 */
int
getPWM (const board_t* board, const uint8_t channel, float* data)
{
    int ret = 0;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr (data);
    (*data) = 0.0f;

    /* DAQC1 & DAQC2 = 2 */
    if (!(channel >= 0 && channel < PP_MAX_PWM_CHANNELS))
    {
        fprintf (stderr, __FILE__ ": PWM channel %d must be 0 or 1\n", channel);
        return -1;
    }

    switch (board->type)
    {
        case PP_BOARD_TYPE_DAQC1:
        {
            ret = pp_daqc1_get_pwm(ctx, channel, data);
            break;
        }

        case PP_BOARD_TYPE_DAQC2:
        {
            (*data) = board->pwm_data[channel].pwm;
            ret = 0;
            break;
        }

        default: {
            ret = -1;
        }
    }

    return ret;
}

/**
 *
 */
int
setDAC (const board_t* board, const uint8_t channel, float value)
{
    uint16_t regval;
    uint16_t calval;
    uint8_t maxchnl;
    uint8_t hibyte = 0;
    uint8_t lobyte = 0;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    /* board powered up? */
    if (board->vcc == 0.0f)
    {
        fprintf (stderr, __FILE__ ": Board Vcc value is zero!\n");
        return -1;
    }

    /* DAQC1=2 DAQC2=4 */
    maxchnl = (pp_is_daqc1(ctx)
                    ? PP_MAX_DAC1_CHANNELS
                    : PP_MAX_DAC2_CHANNELS);
    if (!(channel >= 0 && channel < maxchnl))
    {
        fprintf (stderr, __FILE__ ": DAC channel %d must be between 0 and %d\n",
                 channel, maxchnl);
        return -1;
    }

    if (value < 0.0f || value > PP_MAX_DAC_VOLT)
    {
        fprintf (stderr, __FILE__ ": DAC value %f out of range - must be between 0 and %2.2f volts\n",
                 value, PP_MAX_DAC_VOLT);
        return -1;
    }

    switch (board->type)
    {
        case PP_BOARD_TYPE_DAQC1:
        {
            regval = (uint16_t) value / board->vcc * 1024;
            break;
        }

        case PP_BOARD_TYPE_DAQC2:
        {
            /* get board calibration value */
            calval = board->cal_data[channel].value;
            regval = (uint16_t) value * calval * 1000;

            /* overflow? */
            if (regval > 4095)
            {
                regval = 4095;
            }
            break;
        }

        default: {
            return -1;
        }
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x40 + channel,
       .param1 = regval >> 8,
       .param2 = regval - (hibyte << 8),
       .result = 0,
       .size = 0,
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int
getDAC (const board_t* board, const uint8_t channel, float* data)
{
    uint8_t result[2] = {};
    uint8_t offset;
    uint8_t maxchnl;
    uint16_t regval;
    float value;
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr (data);
    (*data) = 0.0f;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    /* board powered up? */
    if (board->vcc == 0.0f)
    {
        fprintf (stderr, __FILE__ ": Board Vcc value is zero!\n");
        return -1;
    }

    /* DAQC1=2 DAQC2=4 */
    maxchnl = (pp_is_daqc1(ctx)
                    ? PP_MAX_DAC1_CHANNELS
                    : PP_MAX_DAC2_CHANNELS);
    if (!(channel >= 0 && channel < maxchnl))
    {
        fprintf (stderr, __FILE__ ": DAC channel %d must be between 0 and %d\n",
                 channel, maxchnl);
        return -1;
    }

    /* DAQC1=2 DAQC2=4 */
    offset = (pp_is_daqc1(ctx)
                    ? PP_MAX_DAC1_CHANNELS
                    : PP_MAX_DAC2_CHANNELS);

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x40 + channel + offset,
       .param1 = 0,
       .param2 = 0,
       .result = result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    switch (board->type)
    {
        case PP_BOARD_TYPE_DAQC1: {
            regval = 256 * result[0] + result[1];
            value = regval * 100 / PP_MAX_DAC_BITRES;
            (*data) = value * PP_MAX_DAC_VOLT / 100;
            break;
        }

        case PP_BOARD_TYPE_DAQC2: {
            (*data) = (256 * result[0] + result[1]) / 1000;
            break;
        }

        default: {
            (*data) = 0;
        }
    }

    return 0;
}

/**
 *
 */
int
readBoardVcc (const board_t* board)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    float value = 0.0f;
    if (getADC (board, 8, &value) != 0)
    {
        return -1;
    }

    ((board_t*)board)->vcc = value;
    return 0;
}

// ============================================
// Interrupt Functions
// ============================================

/**
 *
 */
int
updateINT (const board_t* board, const uint8_t state)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }
    if (pp_verify_state (state) == 0)
    {
        return -1;
    }

    int command = 0x00;
    switch (state)
    {
        case STATE_OFF:
        {
            command = 0x05;
            break;
        }
        case STATE_ON:
        {
            command = 0x04;
            break;
        }
        default:
        {
            fprintf (stderr, __FILE__ ": INT state #%d not valid. Must be 0 or 1\n", state);
            return -1;
        }
    }

    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = command,
       .param1 = 0,
       .param2 = 0,
       .size = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    return 0;
}

/**
 *
 */
int
enableINT (const board_t* board)
{
    return updateINT (board, STATE_ON);
}

/**
 *
 */
int
disableINT (const board_t* board)
{
    return updateINT (board, STATE_OFF);
}

/**
 *
 */
int
getINTflags (const board_t* board, uint16_t* flags)
{
    pp_context_t* ctx;

    pp_verify_ptr (board);
    if ((ctx = pp_find_board (board)) == NULL)
        return -1;

    pp_verify_ptr (flags);
    (*flags) = 0;

    if (pp_is_daqc(ctx) == 0)
    {
        return -1;
    }

    uint8_t result[2] = {};
    board_command_t bc = {
       .address = PP_BOARD_ADDR(board),
       .command = 0x06,
       .param1 = 0,
       .param2 = 0,
       .result = result,
       .size = sizeof(result),
       .stopAt0x0 = 0,
    };

    if (pp_send_command (ctx, &bc) != 0)
    {
        return -1;
    }

    (*flags) = (256 * result[0] + result[1]);
    return 0;
}
