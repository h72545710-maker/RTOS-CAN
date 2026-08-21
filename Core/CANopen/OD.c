#define OD_DEFINITION
#include "OD.h"

static OD_entry_t OD_list[] = {
    {0x1001U, 1U, ODT_VAR, NULL, NULL},
    {0x1003U, 1U, ODT_ARR, NULL, NULL},
    {0x1005U, 1U, ODT_VAR, NULL, NULL},
    {0x1006U, 1U, ODT_VAR, NULL, NULL},
    {0x1014U, 1U, ODT_VAR, NULL, NULL},
    {0x1016U, 2U, ODT_ARR, NULL, NULL},
    {0x1017U, 1U, ODT_VAR, NULL, NULL},
    {0x1200U, 4U, ODT_REC, NULL, NULL},
    {0x1400U, 4U, ODT_REC, NULL, NULL},
    {0x1600U, 1U, ODT_REC, NULL, NULL},
    {0x1800U, 6U, ODT_REC, NULL, NULL},
    {0x1A00U, 1U, ODT_REC, NULL, NULL},
    {0x0000U, 0U, 0U, NULL, NULL},
};

OD_entry_t OD_entry_H1001 = {0x1001U, 1U, ODT_VAR, NULL, NULL};
OD_entry_t OD_entry_H1003 = {0x1003U, 1U, ODT_ARR, NULL, NULL};
OD_entry_t OD_entry_H1005 = {0x1005U, 1U, ODT_VAR, NULL, NULL};
OD_entry_t OD_entry_H1006 = {0x1006U, 1U, ODT_VAR, NULL, NULL};
OD_entry_t OD_entry_H1014 = {0x1014U, 1U, ODT_VAR, NULL, NULL};
OD_entry_t OD_entry_H1016 = {0x1016U, 2U, ODT_ARR, NULL, NULL};
OD_entry_t OD_entry_H1017 = {0x1017U, 1U, ODT_VAR, NULL, NULL};
OD_entry_t OD_entry_H1200 = {0x1200U, 4U, ODT_REC, NULL, NULL};
OD_entry_t OD_entry_H1400 = {0x1400U, 4U, ODT_REC, NULL, NULL};
OD_entry_t OD_entry_H1600 = {0x1600U, 1U, ODT_REC, NULL, NULL};
OD_entry_t OD_entry_H1800 = {0x1800U, 6U, ODT_REC, NULL, NULL};
OD_entry_t OD_entry_H1A00 = {0x1A00U, 1U, ODT_REC, NULL, NULL};

static OD_t OD_object = {
    .size = (uint16_t)((sizeof(OD_list) / sizeof(OD_list[0])) - 1U),
    .list = OD_list,
};

OD_t *OD = &OD_object;
