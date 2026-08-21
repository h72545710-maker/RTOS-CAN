#ifndef OD_H
#define OD_H

#include "301/CO_ODinterface.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OD_CNT_NMT 1
#define OD_CNT_EM 1
#define OD_CNT_EM_PROD 1
#define OD_CNT_HB_PROD 1
#define OD_CNT_HB_CONS 1
#define OD_CNT_ARR_1016 1
#define OD_CNT_SDO_SRV 1
#define OD_CNT_SYNC 1
#define OD_CNT_RPDO 1
#define OD_CNT_TPDO 1

extern OD_t *OD;

extern OD_entry_t OD_entry_H1001;
extern OD_entry_t OD_entry_H1003;
extern OD_entry_t OD_entry_H1005;
extern OD_entry_t OD_entry_H1006;
extern OD_entry_t OD_entry_H1014;
extern OD_entry_t OD_entry_H1016;
extern OD_entry_t OD_entry_H1017;
extern OD_entry_t OD_entry_H1200;
extern OD_entry_t OD_entry_H1400;
extern OD_entry_t OD_entry_H1600;
extern OD_entry_t OD_entry_H1800;
extern OD_entry_t OD_entry_H1A00;

#define OD_ENTRY_H1001 (&OD_entry_H1001)
#define OD_ENTRY_H1003 (&OD_entry_H1003)
#define OD_ENTRY_H1005 (&OD_entry_H1005)
#define OD_ENTRY_H1006 (&OD_entry_H1006)
#define OD_ENTRY_H1014 (&OD_entry_H1014)
#define OD_ENTRY_H1016 (&OD_entry_H1016)
#define OD_ENTRY_H1017 (&OD_entry_H1017)
#define OD_ENTRY_H1200 (&OD_entry_H1200)
#define OD_ENTRY_H1400 (&OD_entry_H1400)
#define OD_ENTRY_H1600 (&OD_entry_H1600)
#define OD_ENTRY_H1800 (&OD_entry_H1800)
#define OD_ENTRY_H1A00 (&OD_entry_H1A00)

#ifdef __cplusplus
}
#endif

#endif /* OD_H */
