#include "mycan.h"
#include "oled.h"
#include "can.h"
#include <string.h>
#include <stdio.h>


CAN_TxHeaderTypeDef TxHeader;
CAN_FilterTypeDef CAN_FilterStructure;


HAL_StatusTypeDef CAN_Send_Message(uint32_t id, uint8_t* data, uint8_t length, 
                                  uint32_t ide_type, uint32_t rtr_type)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8] = {0};
    uint32_t TxMailbox;
    
    // 数据安全检查
    if (data == NULL || length == 0 || length > 8) 
    {
        return HAL_ERROR;
    }
    
    // 复制数据
    memcpy(TxData, data, length);
    
    // 配置报文头
    TxHeader.StdId = (ide_type == CAN_ID_STD) ? id : 0;
    TxHeader.ExtId = (ide_type == CAN_ID_EXT) ? id : 0;
    TxHeader.IDE = ide_type;
    TxHeader.RTR = rtr_type;
    TxHeader.DLC = length;
    TxHeader.TransmitGlobalTime = DISABLE;

  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, data, &TxMailbox)  != SET)
  {
    OLED_NewFrame();
    OLED_PrintString(0,0,"filed",&font16x16,OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }else
  {
    OLED_NewFrame();
    OLED_PrintString(0,0,"ok:22",&font16x16,OLED_COLOR_NORMAL);
    OLED_ShowFrame();
  }

  return HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}


// 使用示例

//配置过滤器
void can_filter(void)
{
// ����һ��32λ����ģʽ�Ĺ��������������б�׼����֡
CAN_FilterStructure.FilterBank = 0;                    // ʹ�ù�������0
CAN_FilterStructure.FilterMode = CAN_FILTERMODE_IDMASK; // ����ģʽ
CAN_FilterStructure.FilterScale = CAN_FILTERSCALE_32BIT; // 32λ�߶�
CAN_FilterStructure.FilterIdHigh = 0x0000;             // ��ʶ����16λ
CAN_FilterStructure.FilterIdLow = 0x0000;              // ��ʶ����16λ
CAN_FilterStructure.FilterMaskIdHigh = 0x0000;         // �����16λ
CAN_FilterStructure.FilterMaskIdLow = 0x0000;          // �����16λ
CAN_FilterStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0; // ʹ��FIFO0
CAN_FilterStructure.FilterActivation = ENABLE;         // ʹ�ܹ�����

    if (HAL_CAN_ConfigFilter(&hcan, &CAN_FilterStructure) != HAL_OK) {
        return;
    }
}

void can_recive(uint32_t *ID, uint8_t *Length, uint8_t *Data)
{

}
