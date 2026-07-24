/*
 * eeprom_emulation_type_a.c — TI EEPROM Emulation Type A
 * 来源：TI SDK，记录大小改为 256 字节。
 * 未改动算法逻辑，仅修改了 RECORD_SIZE 宏值。
 */

#include "eeprom_emulation_type_a.h"

uint32_t gActiveRecordAddress = EEPROM_EMULATION_ADDRESS;
uint32_t gNextRecordAddress   = EEPROM_EMULATION_ADDRESS;
uint16_t gActiveRecordNum     = 0;
uint16_t gActiveSectorNum     = EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN;

bool gEEPROMTypeASearchFlag      = 0;
bool gEEPROMTypeAEraseFlag       = 0;
bool gEEPROMTypeAFormatErrorFlag = 0;

uint32_t EEPROM_TypeA_writeData(uint32_t *data)
{
    uint32_t *NextRecordPointer = (void *) gNextRecordAddress;
    uint32_t HeaderArray64[]    = {0x0000ffff, 0xffffffff};
    DL_FLASHCTL_COMMAND_STATUS FlashAPIState;

    if (*NextRecordPointer != 0xffffffff) {
        return EEPROM_EMULATION_FORMAT_ERROR;
    }

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, gNextRecordAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
#ifdef __MSPM0_HAS_ECC__
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
        FLASHCTL, gNextRecordAddress, &HeaderArray64[0]);
#else
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64(
        FLASHCTL, gNextRecordAddress, &HeaderArray64[0]);
#endif
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED)
        return EEPROM_EMULATION_WRITE_ERROR;

#ifdef __MSPM0_HAS_ECC__
    FlashAPIState = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, (gNextRecordAddress + 8), data,
        EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t),
        DL_FLASHCTL_REGION_SELECT_MAIN);
#else
    FlashAPIState =
        DL_FlashCTL_programMemoryFromRAM(FLASHCTL, (gNextRecordAddress + 8),
            data, EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t),
            DL_FLASHCTL_REGION_SELECT_MAIN);
#endif
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED)
        return EEPROM_EMULATION_WRITE_ERROR;

    HeaderArray64[1] = 0x0000ffff;
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, gNextRecordAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
#ifdef __MSPM0_HAS_ECC__
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
        FLASHCTL, gNextRecordAddress, &HeaderArray64[0]);
#else
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64(
        FLASHCTL, gNextRecordAddress, &HeaderArray64[0]);
#endif
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED)
        return EEPROM_EMULATION_WRITE_ERROR;

    gActiveRecordNum++;
    if (gActiveRecordNum > EEPROM_EMULATION_ACTIVE_RECORD_NUM_MAX) {
        gActiveRecordNum      = EEPROM_EMULATION_ACTIVE_RECORD_NUM_MIN;
        gEEPROMTypeAEraseFlag = 1;
        gActiveSectorNum++;
        if (gActiveSectorNum > EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX) {
            gActiveSectorNum = EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN;
        }
    }

    if (gEEPROMTypeASearchFlag == 1) {
        HeaderArray64[0] = 0x00000000;
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(
            FLASHCTL, gActiveRecordAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
#ifdef __MSPM0_HAS_ECC__
        FlashAPIState = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
            FLASHCTL, gActiveRecordAddress, &HeaderArray64[0]);
#else
        FlashAPIState = DL_FlashCTL_programMemoryFromRAM64(
            FLASHCTL, gActiveRecordAddress, &HeaderArray64[0]);
#endif
        if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED)
            return EEPROM_EMULATION_WRITE_ERROR;
    } else {
        gEEPROMTypeASearchFlag = 1;
    }

    gActiveRecordAddress = gNextRecordAddress;
    if (gActiveRecordNum >= EEPROM_EMULATION_ACTIVE_RECORD_NUM_MAX &&
        gActiveSectorNum >= EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX) {
        gNextRecordAddress = EEPROM_EMULATION_ADDRESS;
    } else {
        gNextRecordAddress =
            gActiveRecordAddress + EEPROM_EMULATION_RECORD_SIZE;
    }
    return EEPROM_EMULATION_WRITE_OK;
}

uint32_t EEPROM_TypeA_init(uint32_t *data)
{
    bool FlashAPIState;
    EEPROM_TypeA_searchCheck();

    if (gEEPROMTypeASearchFlag == 1) {
        EEPROM_TypeA_readData(data);
        if (gEEPROMTypeAFormatErrorFlag == 1) {
            FlashAPIState = EEPROM_TypeA_repairFormat(data);
            if (FlashAPIState == false) return EEPROM_EMULATION_INIT_ERROR;
            gEEPROMTypeAFormatErrorFlag = 0;
        } else {
            FlashAPIState = EEPROM_TypeA_eraseNonActiveSectors();
            if (FlashAPIState == false) return EEPROM_EMULATION_INIT_ERROR;
        }
    } else {
        FlashAPIState = EEPROM_TypeA_eraseAllSectors();
        if (FlashAPIState == false) return EEPROM_EMULATION_INIT_ERROR;
        gActiveRecordAddress = EEPROM_EMULATION_ADDRESS;
        gNextRecordAddress   = EEPROM_EMULATION_ADDRESS;
        gActiveRecordNum     = 0;
        gActiveSectorNum     = EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN;
    }
    return EEPROM_EMULATION_INIT_OK;
}

void EEPROM_TypeA_readData(uint32_t *data)
{
    uint32_t *ReadRecordPointer;
    uint32_t ReadRecordAddress;

    ReadRecordAddress = gActiveRecordAddress + 8;
    for (uint16_t num = 0; num < EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t);
         num++) {
        ReadRecordPointer = (void *) ReadRecordAddress;
        data[num]         = *ReadRecordPointer;
        ReadRecordAddress += 4;
    }
}

bool EEPROM_TypeA_repairFormat(uint32_t *data)
{
    uint32_t FormatRepairAddress;
    uint32_t HeaderArray64[] = {0x0000ffff, 0xffffffff};
    DL_FLASHCTL_COMMAND_STATUS FlashAPIState;

    if (false == EEPROM_TypeA_eraseNonActiveSectors()) return false;

    if (gActiveSectorNum == EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX) {
        FormatRepairAddress = EEPROM_EMULATION_ADDRESS;
    } else {
        FormatRepairAddress =
            EEPROM_EMULATION_ADDRESS + gActiveSectorNum * 1024;
    }

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, FormatRepairAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
#ifdef __MSPM0_HAS_ECC__
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
        FLASHCTL, FormatRepairAddress, &HeaderArray64[0]);
#else
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64(
        FLASHCTL, FormatRepairAddress, &HeaderArray64[0]);
#endif
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED) return false;

#ifdef __MSPM0_HAS_ECC__
    FlashAPIState = DL_FlashCTL_programMemoryBlockingFromRAM64WithECCGenerated(
        FLASHCTL, (FormatRepairAddress + 8), data,
        EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t),
        DL_FLASHCTL_REGION_SELECT_MAIN);
#else
    FlashAPIState =
        DL_FlashCTL_programMemoryFromRAM(FLASHCTL, (FormatRepairAddress + 8),
            data, EEPROM_EMULATION_DATA_SIZE / sizeof(uint32_t),
            DL_FLASHCTL_REGION_SELECT_MAIN);
#endif
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED) return false;

    HeaderArray64[1] = 0x0000ffff;
    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, FormatRepairAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
#ifdef __MSPM0_HAS_ECC__
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64WithECCGenerated(
        FLASHCTL, FormatRepairAddress, &HeaderArray64[0]);
#else
    FlashAPIState = DL_FlashCTL_programMemoryFromRAM64(
        FLASHCTL, FormatRepairAddress, &HeaderArray64[0]);
#endif
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED) return false;

    gActiveRecordNum = EEPROM_EMULATION_ACTIVE_RECORD_NUM_MIN;
    if (gActiveSectorNum == EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX) {
        gActiveSectorNum = EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN;
    } else {
        gActiveSectorNum += 1;
    }
    gActiveRecordAddress = FormatRepairAddress;
    gNextRecordAddress   = gActiveRecordAddress + EEPROM_EMULATION_RECORD_SIZE;

    if (false == EEPROM_TypeA_eraseLastSector()) return false;
    return true;
}

void EEPROM_TypeA_searchCheck(void)
{
    uint16_t SearchRecordNum, SearchSectorNum;
    uint32_t Temp0, Temp1;
    bool SectorState;
    uint32_t SearchRecordAddress  = EEPROM_EMULATION_ADDRESS;
    uint32_t *SearchRecordPointer = (void *) SearchRecordAddress;

    gEEPROMTypeASearchFlag      = 0;
    gEEPROMTypeAFormatErrorFlag = 0;

    for (SearchSectorNum = EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN;
         SearchSectorNum <= EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX;
         SearchSectorNum++) {
        SectorState = 0;
        for (SearchRecordNum = EEPROM_EMULATION_ACTIVE_RECORD_NUM_MIN;
             SearchRecordNum <= EEPROM_EMULATION_ACTIVE_RECORD_NUM_MAX;
             SearchRecordNum++) {
            SearchRecordPointer = (void *) SearchRecordAddress;
            Temp0               = *SearchRecordPointer;
            SearchRecordPointer = (void *) (SearchRecordAddress + 4);
            Temp1               = *SearchRecordPointer;

            if (Temp0 == 0xffffffff && Temp1 == 0xffffffff) {
                SectorState = 1;
            } else if (Temp0 == 0x00000000 && Temp1 == 0x0000ffff &&
                       SectorState == 0) {
            } else if (Temp0 == 0x0000ffff && Temp1 == 0x0000ffff &&
                       SectorState == 0) {
                gActiveRecordAddress = SearchRecordAddress;
                gActiveRecordNum     = SearchRecordNum;
                gActiveSectorNum     = SearchSectorNum;
                if (gActiveRecordNum >=
                        EEPROM_EMULATION_ACTIVE_RECORD_NUM_MAX &&
                    gActiveSectorNum >=
                        EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX) {
                    gNextRecordAddress = EEPROM_EMULATION_ADDRESS;
                } else {
                    gNextRecordAddress =
                        gActiveRecordAddress + EEPROM_EMULATION_RECORD_SIZE;
                }
                gEEPROMTypeASearchFlag = 1;
            } else if (Temp0 == 0x0000ffff && Temp1 == 0xffffffff &&
                       SectorState == 0) {
                gEEPROMTypeAFormatErrorFlag = 1;
            } else {
                gEEPROMTypeAFormatErrorFlag = 1;
            }
            SearchRecordAddress += (EEPROM_EMULATION_RECORD_SIZE);
        }
    }
}

bool EEPROM_TypeA_eraseLastSector(void)
{
    uint32_t EraseSectorAddress;
    DL_FLASHCTL_COMMAND_STATUS FlashAPIState;
    if (gActiveSectorNum > EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MIN) {
        EraseSectorAddress =
            EEPROM_EMULATION_ADDRESS + 1024 * (gActiveSectorNum - 2);
    } else {
        EraseSectorAddress =
            EEPROM_EMULATION_ADDRESS +
            1024 * (EEPROM_EMULATION_ACTIVE_SECTOR_NUM_MAX - 1);
    }

    DL_FlashCTL_executeClearStatus(FLASHCTL);
    DL_FlashCTL_unprotectSector(
        FLASHCTL, EraseSectorAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
    FlashAPIState = DL_FlashCTL_eraseMemoryFromRAM(
        FLASHCTL, EraseSectorAddress, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
    if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED) return false;
    return true;
}

bool EEPROM_TypeA_eraseNonActiveSectors(void)
{
    uint32_t EraseSectorAddress;
    DL_FLASHCTL_COMMAND_STATUS FlashAPIState;
    uint16_t num;
    for (num = 0; num < EEPROM_EMULATION_SECTOR_ACCOUNT; num++) {
        if (num != (gActiveSectorNum - 1)) {
            EraseSectorAddress = EEPROM_EMULATION_ADDRESS + 1024 * num;
            DL_FlashCTL_executeClearStatus(FLASHCTL);
            DL_FlashCTL_unprotectSector(
                FLASHCTL, EraseSectorAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
            FlashAPIState = DL_FlashCTL_eraseMemoryFromRAM(
                FLASHCTL, EraseSectorAddress, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
            if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED)
                return false;
        }
    }
    return true;
}

bool EEPROM_TypeA_eraseAllSectors(void)
{
    uint32_t EraseSectorAddress;
    DL_FLASHCTL_COMMAND_STATUS FlashAPIState;

    for (uint16_t num = 0; num < EEPROM_EMULATION_SECTOR_ACCOUNT; num++) {
        EraseSectorAddress = EEPROM_EMULATION_ADDRESS + 1024 * num;
        DL_FlashCTL_executeClearStatus(FLASHCTL);
        DL_FlashCTL_unprotectSector(
            FLASHCTL, EraseSectorAddress, DL_FLASHCTL_REGION_SELECT_MAIN);
        FlashAPIState = DL_FlashCTL_eraseMemoryFromRAM(
            FLASHCTL, EraseSectorAddress, DL_FLASHCTL_COMMAND_SIZE_SECTOR);
        if (FlashAPIState == DL_FLASHCTL_COMMAND_STATUS_FAILED) return false;
    }
    return true;
}
