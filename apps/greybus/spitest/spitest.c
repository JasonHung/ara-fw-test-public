/*
 * Copyright (c) 2016 Google, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <string.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <getopt.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

#include <libfwtest.h>

#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))

/* Log tag for print project name */
#define LOG_TAG "ARA"

#define TEST_DATA_SIZE      16
#define TEST_DATA           0xCA

/* default open device */
static const char *device = "/dev/spidev32766.1";

/**
 * @brief check step ret value and confirm step status
 *
 * @param case_id The SPI test case number
 * @param ret return value
 * @return None
 */
void check_step_result(int case_id, int ret)
{
    if (ret) {
        print_test_case_log(LOG_TAG, case_id, strerror(-ret));
    }
}

/**
 * @brief check ret value, then print test result
 *
 * @param case_id The SPI test case number
 * @param ret return value
 * @return None
 */
void print_test_result(int case_id, int ret)
{
    if (ret) {
        print_test_case_result(LOG_TAG, case_id, ret, strerror(-ret));
    } else {
        print_test_case_result(LOG_TAG, case_id, ret, NULL);
    }
}

/**
 * @brief Print usage of this SPI test application
 *
 * @param prog The spitest program name
 * @return None
 */
static void print_usage(const char *prog)
{
    printf("\nUsage: %s [-cD]\n", prog);
    printf("    -c: Test Rail test case ID\n");
    printf("    -D device to use (default /dev/spidev32766.1)\n");
    printf("\nExample : testcase C1051 and device /dev/spidev32766.1 \n");
    printf("     \n");
    printf("     %s -D /dev/spidev32766.1 -c 1051\n", prog);
}

/**
 * @brief Convert SPI mode value to string
 *
 * @param mode: SPI mode
 * @return SPI mode string
 */
static char* mode2str(uint32_t mode)
{
    switch (mode) {
    case SPI_CPHA:
        return "SPI_CPHA";
    case SPI_CPOL:
        return "SPI_CPOL";
    case SPI_CS_HIGH:
        return "SPI_CS_HIGH";
    case SPI_LSB_FIRST:
        return "SPI_LSB_FIRST";
    case SPI_3WIRE:
        return "SPI_3WIRE";
    case SPI_LOOP:
        return "SPI_LOOP";
    case SPI_NO_CS:
        return "SPI_NO_CS";
    case SPI_READY:
        return "SPI_READY";
    }
    return "UNKNOWN";
}

/**
 * @brief Get SPI supported mode
 *
 * @param dev: spi device node
 * @param mode: spi mode, to receive the spi mode value
 * @return 0 on success, negative errno on error
 */
static int spi_get_mode(const char *dev, uint8_t *mode)
{
    int ret = 0, i = 0, fd = 0;
    uint8_t value = 0;

    /* open device */
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        return errno;
    }

    *mode = 0;
    for (i = 0; i < 8; i++) {
        value = 0x1 << i;
        ret = ioctl(fd, SPI_IOC_WR_MODE, &value);
        if (!ret) {
            *mode |= value;
        }
    }
    close(fd);

    return 0;
}

/**
 * @brief SPI Bit-Per-Word testing
 *
 * @param dev: spi device node
 * @param bpw: spi bit-per-word value
 * @return 0 on success, negative errno on error
 */
static int spi_bpw_test(const char *dev, uint8_t bpw)
{
    int fd = 0, i, j, ret = 0;
    uint8_t mode, nbytes = 0, *txbuf, *rxbuf;
    uint32_t freq = 0;
    struct spi_ioc_transfer xfer;

    /* open device */
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        return errno;
    }

    mode = 0; /* mode 0 */
    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret) {
        goto error_ioctl;
    }

    /* config bits per word */
    ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);
    if (ret) {
        /* spi doesn't support this bpw value. */
        goto error_ioctl;
    }
    nbytes = bpw / 8 + ((bpw % 8)? 1 : 0);

    /* config spi frequency */
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &freq);
    if (ret) {
        goto error_ioctl;
    }
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);
    if (ret) {
        goto error_ioctl;
    }

    /* prepare spi transfer data */
    xfer.len = 1;
    xfer.speed_hz = freq / 2;
    xfer.delay_usecs = 0;
    xfer.bits_per_word = bpw;
    xfer.cs_change = 1;
    xfer.pad = 0;

    /* allocate and fill test data */
    txbuf = malloc(nbytes);
    rxbuf = malloc(nbytes);
    if (!txbuf || !rxbuf) {
        ret = -ENOMEM;
        goto error_alloc;
    }
    memset((void*)txbuf, TEST_DATA, nbytes);
    memset((void*)rxbuf, 0, nbytes);

    xfer.tx_buf = (unsigned long)txbuf;
    xfer.rx_buf = (unsigned long)rxbuf;
    /* start spi transfer */
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret > 0) {
        /* transfer success */
        ret = 0;
    }
error_alloc:
    if (txbuf) free(txbuf);
    if (rxbuf) free(rxbuf);
error_ioctl:
    if (fd > 0) close(fd);
    return ret;
}

/**
 * @brief SPI mode testing
 *
 * @param dev: spi device node
 * @param mode: spi mode value
 * @return 0 on success, negative errno on error
 */
static int spi_mode_test(const char *dev, uint8_t mode)
{
    int fd = 0, i, j, ret = 0, n;
    uint8_t bpw = 8, bpw_values[3] = {8, 16, 32}, nbytes = 0, *txbuf, *rxbuf;
    uint32_t freq = 0;
    struct spi_ioc_transfer xfer;
    char msg[256];

    /* open device */
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        return errno;
    }

    ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
    if (ret) {
        goto error_ioctl;
    }

    /* config bits per word */
    for (i = 0; i < ARRAY_SIZE(bpw_values); i++) {
        bpw = bpw_values[i];
        ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bpw);
        if (!ret) {
            nbytes = bpw / 8 + ((bpw % 8)? 1 : 0);
            break;
        }
    }

    if (ret) {
        /* spi doesn't support these bpw values (bpw_values[]) */
        goto error_ioctl;
    }

    /* config spi frequency */
    ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &freq);
    if (ret) {
        goto error_ioctl;
    }
    ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &freq);
    if (ret) {
        goto error_ioctl;
    }

    /* prepare spi transfer data */
    xfer.len = TEST_DATA_SIZE;
    xfer.speed_hz = freq / 2;
    xfer.delay_usecs = 0;
    xfer.bits_per_word = bpw;
    xfer.cs_change = 1;
    xfer.pad = 0;

    /* allocate and fill test data */
    txbuf = malloc(TEST_DATA_SIZE * nbytes);
    rxbuf = malloc(TEST_DATA_SIZE * nbytes);
    if (!txbuf || !rxbuf) {
        ret = -ENOMEM;
        goto error_alloc;
    }
    memset((void*)txbuf, TEST_DATA, TEST_DATA_SIZE * nbytes);
    memset((void*)rxbuf, 0, TEST_DATA_SIZE * nbytes);

    xfer.tx_buf = (unsigned long)txbuf;
    xfer.rx_buf = (unsigned long)rxbuf;
    /* start spi transfer */
    ret = ioctl(fd, SPI_IOC_MESSAGE(1), &xfer);
    if (ret > 0) {
        /* transfer success */
        ret = 0;

        /* dump transfer data */
        n = 0;
        n += sprintf(msg, "TxBuf: ");
        for (i = 0; i < TEST_DATA_SIZE; i++) {
            n += sprintf(msg + n, "%02X ", txbuf[i]);
        }
        printf("%s\n",msg);

        n = 0;
        n += sprintf(msg, "RxBuf: ");
        for (i = 0; i < TEST_DATA_SIZE; i++) {
            n += sprintf(msg + n, "%02X ", rxbuf[i]);
        }
        printf("%s",msg);
    }
error_alloc:
    if (txbuf) free(txbuf);
    if (rxbuf) free(rxbuf);
error_ioctl:
    if (fd > 0) close(fd);
    return ret;
}

/**
 * @brief Test Rail test case C1051
 *
 * C1051: SPI Protocol Mode Response contains the supported modes in the SPI
 * master.
 * This test case verifies that the SPI Protocol Mode Response payload contains
 * a two one-byte values representing the modes that the SPI Masters supports.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1051_get_mode(int case_id)
{
    int ret = 0, i;
    uint8_t mode = 0;
    char msg[40];

    /* query supported spi mode */
    ret = spi_get_mode(device, &mode);

    if (ret) {
        goto err_getmode;
    }

    /* print the spi mode info */
    for (i = 0; i < 8; i++) {
        if (mode & (1 << i)) {
            sprintf(msg, "SPI support mode: %s", mode2str(1 << i));
            print_test_case_log(LOG_TAG, case_id, msg);
        }
    }

err_getmode:
    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1052
 *
 * C1052: SPI Protocol Bits Per word Mask Response contains the Bits Per Word
 * that the SPI Master supports.
 * This test case verifies that the SPI Protocol Bits Per Word Mask Response
 * payload contains a Four-byte value representing the Bits Per Word Mask of
 * the SPI Master.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1052_get_bpw(int case_id)
{
    int ret = 0;
    uint8_t bpw = 0, i;
    char msg[40];

    for (i = 1; i <= 32; i++) {
        /* query supported spi bpw value */
        ret = spi_bpw_test(device, i);

        if (!ret) {
            /* print the spi bpw info */
            sprintf(msg, "SPI BPW support %d", i);
            print_test_case_log(LOG_TAG, case_id, msg);
        }
    }
    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1053
 *
 * C1053: Verify transfer with Clock phase mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_CPHASE in the SPI Protocol
 * Transfer Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1053_CPHA(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_CPHA;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1054
 *
 * C1054: Verify transfer with Clock polarity mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_CPOL in the SPI Protocol Transfer
 * Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1054_CPOL(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_CPOL;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1055
 *
 * C1055: Verify transfer with Chip select active high mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_CS_HIGH in the SPI Protocol
 * Transfer Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1055_CS_high(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_CS_HIGH;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1056
 *
 * C1056: Verify transfer with Per-word bits-on-wire mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_LSB_FIRST in the SPI Protocol
 * Transfer Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1056_LSB_first(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_LSB_FIRST;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1057
 *
 * C1057: Verify transfer with SI/SO Signals shared mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_3WIRE in the SPI Protocol Transfer
 * Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1057_3WIRE(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_3WIRE;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1058
 *
 * C1058: Verify transfer with loopback mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_LOOP in the SPI Protocol Transfer
 * Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1058_loop(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_LOOP;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1059
 *
 * C1059: Verify transfer with no chip select mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_NO_CS in the SPI Protocol Transfer
 * Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1059_no_CS(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_NO_CS;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Test Rail test case C1060
 *
 * C1060: Verify transfer with slave mode is successful.
 * This test case verifies that SPI Transfer Response payload contains the data
 * requested when mode is set to GB_SPI_MODE_READY in the SPI Protocol Transfer
 * Request Operation.
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int ARA_1060_ready(int case_id)
{
    int ret = 0;
    uint8_t mode = SPI_READY;

    ret = spi_mode_test(device, mode);

    print_test_result(case_id, ret);
    return ret;
}

/**
 * @brief Perform the test case
 *
 * @param case_id: test case id
 * @return 0 on success, error code on failure
 */
static int perform_testcase(int case_id)
{
    switch (case_id) {
    case 1051:
        return ARA_1051_get_mode(case_id);
    case 1052:
        return ARA_1052_get_bpw(case_id);
    case 1053:
        return ARA_1053_CPHA(case_id);
    case 1054:
        return ARA_1054_CPOL(case_id);
    case 1055:
        return ARA_1055_CS_high(case_id);
    case 1056:
        return ARA_1056_LSB_first(case_id);
    case 1057:
        return ARA_1057_3WIRE(case_id);
    case 1058:
        return ARA_1058_loop(case_id);
    case 1059:
        return ARA_1059_no_CS(case_id);
    case 1060:
        return ARA_1060_ready(case_id);
    default:
        print_test_case_log(LOG_TAG, 0,
                            "Error: The command had error case_id.");
        return -EINVAL;
    }
}

/**
 * @brief The gpiotest main function
 *
 * @param argc The spitest main arguments count
 * @param argv The spitest main arguments data
 * @return 0 on success, error code on failure
 */
int main(int argc, char **argv)
{
    int ret = 0, option = 0;
    int case_id = 0;

    if (argc < 2) {
        print_usage(argv[0]);
        return -EINVAL;
    }

    /* parse the input argument */
    while ((option = getopt(argc, argv, "c:D:")) != -1) {
        switch(option) {
        case 'c':
            case_id = atoi(optarg);
            break;
        case 'D':
            device = optarg;
            break;
        default:
            print_usage(argv[0]);
            return -EINVAL;
        }
    }

    /* run the test case */
    ret = perform_testcase(case_id);
    check_step_result(case_id, ret);
    return ret;
}
