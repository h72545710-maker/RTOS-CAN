#include "canopen_port_fdcan.h"
#include "301/CO_driver.h"

#define CANOPEN_CANID_MASK 0x07FFU
#define CANOPEN_FLAG_RTR 0x0800U

void CANopen_PortFdcan_Phase1CompileAnchor(void)
{
}

void CO_CANsetConfigurationMode(void *CANptr)
{
    (void)CANptr;
}

void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL) {
        CANmodule->CANnormal = false;
    }
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t *CANmodule,
                                   void *CANptr,
                                   CO_CANrx_t rxArray[],
                                   uint16_t rxSize,
                                   CO_CANtx_t txArray[],
                                   uint16_t txSize,
                                   uint16_t CANbitRate)
{
    if ((CANmodule == NULL) || (rxArray == NULL) || (txArray == NULL)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CANmodule->CANptr = CANptr;
    CANmodule->rxArray = rxArray;
    CANmodule->rxSize = rxSize;
    CANmodule->txArray = txArray;
    CANmodule->txSize = txSize;
    CANmodule->CANerrorStatus = 0U;
    CANmodule->CANnormal = false;
    CANmodule->useCANrxFilters = false;
    CANmodule->bufferInhibitFlag = false;
    CANmodule->firstCANtxMessage = true;
    CANmodule->CANtxCount = 0U;
    CANmodule->errOld = 0U;
    (void)CANbitRate;

    for (uint16_t i = 0U; i < rxSize; i++) {
        rxArray[i].ident = 0U;
        rxArray[i].mask = 0xFFFFU;
        rxArray[i].object = NULL;
        rxArray[i].CANrx_callback = NULL;
    }

    for (uint16_t i = 0U; i < txSize; i++) {
        txArray[i].ident = 0U;
        txArray[i].DLC = 0U;
        txArray[i].bufferFull = false;
        txArray[i].syncFlag = false;
    }

    return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL) {
        CANmodule->CANnormal = false;
    }
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t *CANmodule,
                                    uint16_t index,
                                    uint16_t ident,
                                    uint16_t mask,
                                    bool_t rtr,
                                    void *object,
                                    void (*CANrx_callback)(void *object, void *message))
{
    if ((CANmodule == NULL) || (object == NULL) || (CANrx_callback == NULL) || (index >= CANmodule->rxSize)) {
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    CO_CANrx_t *buffer = &CANmodule->rxArray[index];
    buffer->ident = (ident & CANOPEN_CANID_MASK) | (rtr ? CANOPEN_FLAG_RTR : 0U);
    buffer->mask = (mask & CANOPEN_CANID_MASK) | CANOPEN_FLAG_RTR;
    buffer->object = object;
    buffer->CANrx_callback = CANrx_callback;

    return CO_ERROR_NO;
}

CO_CANtx_t *CO_CANtxBufferInit(CO_CANmodule_t *CANmodule,
                               uint16_t index,
                               uint16_t ident,
                               bool_t rtr,
                               uint8_t noOfBytes,
                               bool_t syncFlag)
{
    if ((CANmodule == NULL) || (index >= CANmodule->txSize)) {
        return NULL;
    }

    CO_CANtx_t *buffer = &CANmodule->txArray[index];
    buffer->ident = (ident & CANOPEN_CANID_MASK) | (rtr ? CANOPEN_FLAG_RTR : 0U);
    buffer->DLC = noOfBytes;
    buffer->bufferFull = false;
    buffer->syncFlag = syncFlag;

    return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer)
{
    (void)CANmodule;
    (void)buffer;
    return CO_ERROR_INVALID_STATE;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule)
{
    if (CANmodule != NULL) {
        CANmodule->bufferInhibitFlag = false;
    }
}

void CO_CANmodule_process(CO_CANmodule_t *CANmodule)
{
    (void)CANmodule;
}
