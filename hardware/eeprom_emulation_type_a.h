/*
 * eeprom_emulation_type_a.h — TI EEPROM Emulation Type A
 * 记录大小改为 256 字节（数据负载 248 字节）
 */

#ifndef EEPROM_EMULATION_TYPE_A_H_
#define EEPROM_EMULATION_TYPE_A_H_

#include <stdbool.h>
#include <stdint.h>
#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifndef EEPROM_EMULATION_ADDRESS
#define EEPROM_EMULATION_ADDRESS                                    (0x0001F800)
#endif
#define EEPROM_EMULATION_SECTOR_ACCOUNT                                      (2)
#define EEPROM_EMULATION_RECORD_SIZE                                       (256)  /* 改为 256B */
#define EEPROM_EMULATION_DATA_SIZE            (EEPROM_EMULATION_RECORD_SIZE - 8)
#define EEPROM_EMULATION_RECORD_ACCOUNT    (1024 / EEPROM_EMULATION_RECORD_SIZE)
#define EEPROM_EMULATION_ACTIVE_RECORD_NUM_MIN                               (1)
#define EEPROM_EMULATION_ACTIVE_RECORD_NUM_MAX  (EEPROM_EMULATION_RECORD_ACCOUNT)
#define EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN                               (1)
#define EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX  (EEPROM_EMULATION_SECTOR_ACCOUNT)

#define EEPROM_EMULATION_WRITE_OK                       ((uint32_t) 0x00000000U)
#define EEPROM_EMULATION_WRITE_ERROR                    ((uint32_t) 0x00000001U)
#define EEPROM_EMULATION_FORMAT_ERROR                   ((uint32_t) 0x00000010U)
#define EEPROM_EMULATION_INIT_OK                        ((uint32_t) 0x00000000U)
#define EEPROM_EMULATION_INIT_ERROR                     ((uint32_t) 0x00000002U)

extern uint32_t gActiveRecordAddress;
extern uint32_t gNextRecordAddress;
extern uint16_t gActiveRecordNum;
extern uint16_t gActiveSectorNum;
extern bool gEEPROMTypeASearchFlag;
extern bool gEEPROMTypeAEraseFlag;
extern bool gEEPROMTypeAFormatErrorFlag;

uint32_t EEPROM_TypeA_writeData(uint32_t *data);
uint32_t EEPROM_TypeA_init(uint32_t *data);
void     EEPROM_TypeA_readData(uint32_t *data);
bool     EEPROM_TypeA_repairFormat(uint32_t *data);
void     EEPROM_TypeA_searchCheck(void);
bool     EEPROM_TypeA_eraseLastSector(void);
bool     EEPROM_TypeA_eraseNonActiveSectors(void);
bool     EEPROM_TypeA_eraseAllSectors(void);

#endif
