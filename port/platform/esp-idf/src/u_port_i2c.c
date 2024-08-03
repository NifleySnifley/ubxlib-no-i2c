/*
 * Copyright 2019-2024 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 /** @file
  * @brief Implementation of the port I2C API for the ESP-IDF platform.
  */

#include "stddef.h"
#include "stdint.h"
#include "stdbool.h"

#include "u_cfg_sw.h"
#include "u_compiler.h" // U_ATOMIC_XXX() macros

#include "u_error_common.h"

#include "u_port.h"
#include "u_port_debug.h"
#include "u_port_os.h"
#include "u_port_i2c.h"

  // #include "driver/i2c.h"

  /* ----------------------------------------------------------------
   * COMPILE-TIME MACROS
   * -------------------------------------------------------------- */

#ifndef U_PORT_I2C_MAX_NUM
   /** The number of I2C HW blocks that are available on ESP32.
    */
# define U_PORT_I2C_MAX_NUM 2
#endif

    /** Make a 7-bit address with a read bit.
     */
#define U_PORT_7BIT_ADDRESS_READ(__ADDRESS__) (((__ADDRESS__) << 1) | I2C_MASTER_READ)

     /** Make a 7-bit address with a write bit.
      */
#define U_PORT_7BIT_ADDRESS_WRITE(__ADDRESS__) (((__ADDRESS__) << 1) | I2C_MASTER_WRITE)

      /** Create a header to indicate 10-bit address transmission with a read bit.
       */
#define U_PORT_10BIT_HEADER_READ(__ADDRESS__) ((((__ADDRESS__) & (0x0300)) >> 7) | 0xF0 | I2C_MASTER_READ)

       /** Create a header to indicate 10-bit address transmission with a write bit.
        */
#define U_PORT_10BIT_HEADER_WRITE(__ADDRESS__) ((((__ADDRESS__) & (0x0300)) >> 7) | 0xF0 | I2C_MASTER_WRITE)

        /** Get the portion of a 10 bit address that will be sent first (which
         * is the same whether reading or writing).
         */
#define U_PORT_10BIT_ADDRESS(__ADDRESS__) ((__ADDRESS__) & 0xFF)

#ifndef U_PORT_I2C_ESP32X3_CLOCK_SOURCE
         /** For ESP32 the I2C clock source is the APB clock (80 MHz)
          * and this code doesn't care, however for ESP32x3 the clock
          * source can be selected between the crystal/XTAL (40 MHz) and
          * the RC network which drives the RTC (17.5 MHz); the I2C
          * timeout value is calculated differently depending on which
          * source is employed.  The crystal is the default: switch to
          * the RC network by setting this #define to
          * I2C_SCLK_SRC_FLAG_LIGHT_SLEEP.
          */
# define U_PORT_I2C_ESP32X3_CLOCK_SOURCE 0
#endif

#if U_PORT_I2C_ESP32X3_CLOCK_SOURCE == I2C_SCLK_SRC_FLAG_LIGHT_SLEEP
# define U_PORT_I2C_CLOCK_PERIOD_NS 57
#else
# define U_PORT_I2C_CLOCK_PERIOD_NS 25
#endif

          /** The maximum value that an ESP32X3 I2C timeout
           * register can take.
           */
#define U_PORT_I2C_ESP32X3_TIMEOUT_REGISTER_MAX 22

           /* ----------------------------------------------------------------
            * TYPES
            * -------------------------------------------------------------- */

            /** Structure of the things we need to keep track of per I2C instance.
             */
typedef struct {
    int32_t pinSda;
    int32_t pinSdc;
    int32_t clockHertz; // This also used as a flag to indicate "in use"
    bool adopted;
} uPortI2cData_t;

/* ----------------------------------------------------------------
 * VARIABLES
 * -------------------------------------------------------------- */

 /** Mutex to ensure thread-safety.
  */
static uPortMutexHandle_t gMutex = NULL;

/** I2C device data.
 */
static uPortI2cData_t gI2cData[U_PORT_I2C_MAX_NUM];

/** Variable to keep track of the number of I2C interfaces open.
 */
static volatile int32_t gResourceAllocCount = 0;

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

 // Convert a millisecond timeout to a value that can be passed to
 // i2c_set_timeout()
static int32_t timeoutMsToEsp32(int32_t timeoutMs) {
    int32_t timeoutEsp32 = -1;

#ifdef CONFIG_IDF_TARGET_ESP32
    // Not the X3 case, good 'ole ESP32, nice and simple, units
    // of one cycle of the 80 MHz APB clock.
    timeoutEsp32 = timeoutMs * 80000;
#else
    int32_t y;
    // On ESP32X3 and similar the timeout is a power of two times
    // the chosen source clock period, so 2^x * U_PORT_I2C_CLOCK_PERIOD_NS;
    // if the 40 MHz crystal is chosen as SCLK then you have
    // 2^x * 25 ns, where x can be a maximum value of 22, so the
    // largest timeout value is 2^22 * 25ns = 104.9ms.
    for (size_t x = 0; (x < U_PORT_I2C_ESP32X3_TIMEOUT_REGISTER_MAX) &&
        (timeoutEsp32 < 0); x++) {
        y = (1UL << x) * U_PORT_I2C_CLOCK_PERIOD_NS / 1000000;
        if (y >= timeoutMs) {
            timeoutEsp32 = x;
        }
    }
#endif

    return timeoutEsp32;
}

// Convert a value returned by i2c_get_timeout() into milliseconds.
static int32_t timeoutEsp32ToMs(int32_t timeoutEsp32) {
    int32_t timeoutMs = -1;

#ifdef CONFIG_IDF_TARGET_ESP32
    // Not the X3 case, good 'ole ESP32.
    timeoutMs = timeoutEsp32 / 80000;
#else
    timeoutMs = (1UL << timeoutEsp32) * U_PORT_I2C_CLOCK_PERIOD_NS / 1000000;
#endif

    return timeoutMs;
}

// Close an I2C instance.
static void closeI2c(int32_t index) {
    if (gI2cData[index].clockHertz > 0) {
        if (!gI2cData[index].adopted) {
            i2c_driver_delete(index);
        }
        gI2cData[index].clockHertz = -1;
        U_ATOMIC_DECREMENT(&gResourceAllocCount);
    }
}

// Send an I2C message, returning zero on success else negative error code.
static int32_t send(int32_t handle, uint16_t address,
    const char* pData, size_t size, bool noStop) {
    return -1;
}

// Receive an I2C message, returning number of bytes received on success else
// negative error code.
static int32_t receive(int32_t handle, uint16_t address, char* pData, size_t size) {
    return -1;
}

// Open an I2C instance; unlike the other static functions
// this does all the mutex locking etc.
static int32_t openI2c(int32_t i2c, int32_t pinSda, int32_t pinSdc,
    bool controller, bool adopt) {
    return -1;
}

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

 // Initialise I2C handling.
int32_t uPortI2cInit() {
    int32_t errorCode = (int32_t)U_ERROR_COMMON_SUCCESS;

    if (gMutex == NULL) {
        errorCode = uPortMutexCreate(&gMutex);
        if (errorCode == 0) {
            for (size_t x = 0; x < sizeof(gI2cData) / sizeof(gI2cData[0]); x++) {
                gI2cData[x].pinSda = -1;
                gI2cData[x].pinSdc = -1;
                gI2cData[x].clockHertz = -1;
            }
        }
    }

    return errorCode;
}

// Shutdown I2C handling.
void uPortI2cDeinit() {
    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        // Shut down any open instances
        for (size_t x = 0; x < sizeof(gI2cData) / sizeof(gI2cData[0]); x++) {
            closeI2c(x);
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
        uPortMutexDelete(gMutex);
        gMutex = NULL;
    }
}

// Open an I2C instance.
int32_t uPortI2cOpen(int32_t i2c, int32_t pinSda, int32_t pinSdc,
    bool controller) {
    return openI2c(i2c, pinSda, pinSdc, controller, false);
}

// Adopt an I2C instance.
int32_t uPortI2cAdopt(int32_t i2c, bool controller) {
    return openI2c(i2c, -1, -1, controller, true);
}

// Close an I2C instance.
void uPortI2cClose(int32_t handle) {
    if ((gMutex != NULL) && (handle >= 0) &&
        (handle < sizeof(gI2cData) / sizeof(gI2cData[0]))) {

        U_PORT_MUTEX_LOCK(gMutex);

        closeI2c(handle);

        U_PORT_MUTEX_UNLOCK(gMutex);
    }
}

// Close an I2C instance and attempt to recover the I2C bus.
int32_t uPortI2cCloseRecoverBus(int32_t handle) {
    int32_t errorCode = (int32_t)U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCode = (int32_t)U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0)) {
            errorCode = (int32_t)U_ERROR_COMMON_NOT_SUPPORTED;
            if (!gI2cData[handle].adopted) {
                closeI2c(handle);
                // Nothing to do - bus recovery is done as required
                // on ESP-IDF; return "not supported" to indicate this
                errorCode = (int32_t)U_ERROR_COMMON_NOT_SUPPORTED;
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCode;
}

// Set the I2C clock frequency.
int32_t uPortI2cSetClock(int32_t handle, int32_t clockHertz) {
    return -1;
}

// Get the I2C clock frequency.
int32_t uPortI2cGetClock(int32_t handle) {
    int32_t errorCodeOrClock = (int32_t)U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrClock = (int32_t)U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0)) {
            errorCodeOrClock = (int32_t)U_ERROR_COMMON_NOT_SUPPORTED;
            if (!gI2cData[handle].adopted) {
                errorCodeOrClock = gI2cData[handle].clockHertz;
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrClock;
}

// Set the timeout for I2C.
int32_t uPortI2cSetTimeout(int32_t handle, int32_t timeoutMs) {
    return -1;
}

// Get the timeout for I2C.
int32_t uPortI2cGetTimeout(int32_t handle) {
    return -1;
}

// Send and/or receive over the I2C interface as a controller.
int32_t uPortI2cControllerExchange(int32_t handle, uint16_t address,
    const char* pSend, size_t bytesToSend,
    char* pReceive, size_t bytesToReceive,
    bool noInterveningStop) {
    int32_t errorCodeOrLength = (int32_t)U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrLength = (int32_t)U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) &&
            ((pSend != NULL) || (bytesToSend == 0)) &&
            ((pReceive != NULL) || (bytesToReceive == 0))) {
            errorCodeOrLength = (int32_t)U_ERROR_COMMON_SUCCESS;
            if (pSend != NULL) {
                errorCodeOrLength = send(handle, address, pSend, bytesToSend,
                    noInterveningStop);
            }
            if ((errorCodeOrLength == (int32_t)U_ERROR_COMMON_SUCCESS) &&
                (pReceive != NULL)) {
                errorCodeOrLength = receive(handle, address, pReceive, bytesToReceive);
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrLength;
}

/** \deprecated please use uPortI2cControllerExchange() instead. */
// Send and/or receive over the I2C interface as a controller.
int32_t uPortI2cControllerSendReceive(int32_t handle, uint16_t address,
    const char* pSend, size_t bytesToSend,
    char* pReceive, size_t bytesToReceive) {
    int32_t errorCodeOrLength = (int32_t)U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCodeOrLength = (int32_t)U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) &&
            ((pSend != NULL) || (bytesToSend == 0)) &&
            ((pReceive != NULL) || (bytesToReceive == 0))) {
            errorCodeOrLength = (int32_t)U_ERROR_COMMON_SUCCESS;
            if (pSend != NULL) {
                errorCodeOrLength = send(handle, address, pSend, bytesToSend, false);
            }
            if ((errorCodeOrLength == (int32_t)U_ERROR_COMMON_SUCCESS) &&
                (pReceive != NULL)) {
                errorCodeOrLength = receive(handle, address, pReceive, bytesToReceive);
            }
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCodeOrLength;
}

/** \deprecated please use uPortI2cControllerExchange() instead. */
// Perform a send over the I2C interface as a controller.
int32_t uPortI2cControllerSend(int32_t handle, uint16_t address,
    const char* pSend, size_t bytesToSend,
    bool noStop) {
    int32_t errorCode = (int32_t)U_ERROR_COMMON_NOT_INITIALISED;

    if (gMutex != NULL) {

        U_PORT_MUTEX_LOCK(gMutex);

        errorCode = (int32_t)U_ERROR_COMMON_INVALID_PARAMETER;
        if ((handle >= 0) && (handle < sizeof(gI2cData) / sizeof(gI2cData[0])) &&
            (gI2cData[handle].clockHertz > 0) &&
            ((pSend != NULL) || (bytesToSend == 0))) {
            errorCode = send(handle, address, pSend, bytesToSend, noStop);
        }

        U_PORT_MUTEX_UNLOCK(gMutex);
    }

    return errorCode;
}

// Get the number of I2C interfaces currently open.
int32_t uPortI2cResourceAllocCount() {
    return U_ATOMIC_GET(&gResourceAllocCount);
}

// End of file
