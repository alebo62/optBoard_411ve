#include "main.h"

extern uint32_t SectorError;
extern uint32_t *pOption;
extern uint32_t options[];
extern uint32_t Address;
extern FLASH_EraseInitTypeDef EraseInitStruct;
extern uint32_t AddressW;
extern const uint8_t option_cnt;

void erase_flash(void)
{
	HAL_FLASH_Unlock();
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = 4;
  EraseInitStruct.NbSectors = 1;
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    Error_Handler();
  }
	__HAL_FLASH_DATA_CACHE_DISABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

  __HAL_FLASH_DATA_CACHE_RESET();
  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();
	HAL_FLASH_Lock();
}

void detect_free(void)
{
	AddressW = FA_RPT_CH;
  while((*(__IO uint32_t*)AddressW) != 0xFFFFFFFF)
		AddressW += 4;
}

void write_flash(void)
{
	pOption = options;
	Address = AddressW; 
	HAL_FLASH_Unlock();
	while (Address < (AddressW + option_cnt * 4))
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, *pOption) == HAL_OK)
    {
      Address += 4;
			pOption++;
    }
    else
    { 
      Error_Handler();
    }
  }
	HAL_FLASH_Lock();
}

void read_flash(void)
{
	pOption = options;
  Address = AddressW;
	while (Address < (FA_RPT_CH + option_cnt * 4))
	{
		*pOption = *(__IO uint32_t*)Address;
		Address = Address + 4;
		pOption++;
	} 
}