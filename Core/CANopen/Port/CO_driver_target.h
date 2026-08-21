/*
 * RTOS-CAN target definitions for CANopenNode Phase 1.
 *
 * This header intentionally does not bind CANopenNode to STM32 FDCAN yet.
 * Phase 1 only makes CANopenNode core compile in the project.
 */

#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Minimal CiA 301 configuration prepared for a basic node:
 * NMT, heartbeat producer/consumer, emergency producer/history,
 * SDO server, SYNC consumer and PDO objects.
 */
#define CO_CONFIG_GLOBAL_FLAG_CALLBACK_PRE (0)
#define CO_CONFIG_GLOBAL_RT_FLAG_CALLBACK_PRE (0)
#define CO_CONFIG_GLOBAL_FLAG_TIMERNEXT CO_CONFIG_FLAG_TIMERNEXT
#define CO_CONFIG_GLOBAL_FLAG_OD_DYNAMIC (0)

#define CO_CONFIG_NMT (CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_HB_CONS (CO_CONFIG_HB_CONS_ENABLE | CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_NODE_GUARDING (0)
#define CO_CONFIG_EM (CO_CONFIG_EM_PRODUCER | CO_CONFIG_EM_HISTORY | CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_EM_ERR_STATUS_BITS_COUNT (10U * 8U)
#define CO_CONFIG_SDO_SRV (CO_CONFIG_SDO_SRV_SEGMENTED | CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_SDO_CLI (0)
#define CO_CONFIG_TIME (0)
#define CO_CONFIG_SYNC (CO_CONFIG_SYNC_ENABLE | CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_PDO (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_PDO_SYNC_ENABLE | \
                       CO_CONFIG_PDO_OD_IO_ACCESS | CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_STORAGE (0)
#define CO_CONFIG_LEDS (0)
#define CO_CONFIG_GFC (0)
#define CO_CONFIG_SRDO (0)
#define CO_CONFIG_LSS (0)
#define CO_CONFIG_GTW (0)
#define CO_CONFIG_CRC16 (0)
#define CO_CONFIG_FIFO (0)
#define CO_CONFIG_TRACE (0)
#define CO_CONFIG_DEBUG (0)

#ifdef __cplusplus
extern "C" {
#endif

#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) (x)
#define CO_SWAP_32(x) (x)
#define CO_SWAP_64(x) (x)

typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

typedef struct {
    uint32_t ident;
    uint8_t dlc;
    uint8_t data[8];
} CO_CANrxMsg_t;

#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t *)(msg))->ident))
#define CO_CANrxMsg_readDLC(msg) ((uint8_t)(((CO_CANrxMsg_t *)(msg))->dlc))
#define CO_CANrxMsg_readData(msg) ((const uint8_t *)(((CO_CANrxMsg_t *)(msg))->data))

typedef struct {
    uint16_t ident;
    uint16_t mask;
    void *object;
    void (*CANrx_callback)(void *object, void *message);
} CO_CANrx_t;

typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

typedef struct {
    void *CANptr;
    CO_CANrx_t *rxArray;
    uint16_t rxSize;
    CO_CANtx_t *txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;

typedef struct {
    void *addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
    void *addrNV;
} CO_storage_entry_t;

#define CO_LOCK_CAN_SEND(CAN_MODULE)
#define CO_UNLOCK_CAN_SEND(CAN_MODULE)
#define CO_LOCK_EMCY(CAN_MODULE)
#define CO_UNLOCK_EMCY(CAN_MODULE)
#define CO_LOCK_OD(CAN_MODULE)
#define CO_UNLOCK_OD(CAN_MODULE)

#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew) \
    do {                   \
        CO_MemoryBarrier(); \
        rxNew = (void *)1L; \
    } while (0)
#define CO_FLAG_CLEAR(rxNew) \
    do {                     \
        CO_MemoryBarrier();   \
        rxNew = NULL;         \
    } while (0)

#ifdef __cplusplus
}
#endif

#endif /* CO_DRIVER_TARGET_H */
