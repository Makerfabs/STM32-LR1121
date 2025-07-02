#include "dht11.h"
#include <stddef.h>
#include "stm32l4xx_hal.h"
#include <stdio.h>
#include <stdint.h>
#include "delay.h"
#include "lorawan_commissioning.h"
#include "lr1121_modem_board.h"
#include "apps_utilities.h"
#include "lr1121_modem_helper.h"
#include "lr1121_modem_system_types.h"

uint8_t Data[5];
int flag=0;

void DHT_GPIO_INPUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin=DHT11_PIN;
    GPIO_InitStructure.Mode=GPIO_MODE_INPUT;
    GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_IO,&GPIO_InitStructure);
}

void DHT_GPIO_OUTPUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.Pin=DHT11_PIN;
    GPIO_InitStructure.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(DHT11_IO,&GPIO_InitStructure);
}

//获取一个字节
char DHT_Read_Byte(void)
{
	unsigned char i = 0;
	unsigned char Read_Data = 0;
	uint32_t timeout = 0;

	for(i=0;i<8;i++) //1个数据就是1个字节byte，1个字节byte有8位bit
	{
		// 等待低电平结束，添加超时保护
		timeout = 0;
		while(HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) == 0 && timeout < 100) 
		{
			delay_us(1);
			timeout++;
		}
		if(timeout >= 100) return 0; // 超时返回0
		
		delay_us(30); //延迟30us是为了区别数据0和数据1，0只有26~28us

		Read_Data <<= 1; //左移

		if(HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) == 1) //如果过了30us还是高电平的话就是数据1
		{
			Read_Data |= 1; //数据+1
		}

		// 等待高电平结束，添加超时保护
		timeout = 0;
		while(HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) == 1 && timeout < 100) 
		{
			delay_us(1);
			timeout++;
		}
		if(timeout >= 100) return 0; // 超时返回0
	}

	return Read_Data;
}

void DHT11_Start()
{
    uint32_t timeout = 0;
    DHT_GPIO_OUTPUT();
    HAL_GPIO_WritePin(DHT11_IO, DHT11_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(DHT11_IO, DHT11_PIN, GPIO_PIN_RESET);
    //HAL_Delay(20);
    delay_ms(20);//拉低总线至少 18ms
    HAL_GPIO_WritePin(DHT11_IO, DHT11_PIN, GPIO_PIN_SET);
    delay_us(30);  // 等待30us
    DHT_GPIO_INPUT();
    
    timeout = 0;
    while(HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) && (timeout < 100)) {  //等待DHT11响应低电平
        delay_us(1);
        timeout++;
    }
    if(timeout >= 100) return;  // 超时退出
    
    timeout = 0;
    while(!HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) && (timeout < 100)) { //等待DHT11拉高总线
        delay_us(1);
        timeout++;
    }
    if(timeout >= 100) return;  // 超时退出
    
    timeout = 0;
    while(HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) && (timeout < 100)) {  //等待DHT11拉低总线，开始传送数据
        delay_us(1);
        timeout++;
    }
    if(timeout >= 100) return;  // 超时退出
}

void DHT_Read()
{
    uint8_t timeout_flag = 0;
    uint32_t timeout = 0;

    HAL_DBG_TRACE_INFO("DHT11 read start\n");

    for(int i = 0; i < 5; i++)
    {
        Data[i] = 0;
    }

    DHT11_Start();
    
    // 检查是否超时退出
    // 如果DHT11_Start正常完成，DHT11_IO的引脚应该是低电平
    if(HAL_GPIO_ReadPin(DHT11_IO, DHT11_PIN) == 1)
    {
        // 如果是高电平，可能是DHT11_Start中的某个超时保护触发了
        return; // 直接返回，flag已经设置为0
    }
    
    DHT_GPIO_INPUT();

    for(int i = 0; i < 5; i++)
    {
        Data[i] = DHT_Read_Byte();
    }
    
    // 校验和检查
    if((Data[0]+Data[1]+Data[2]+Data[3]) == Data[4])
    {
        flag = 1; // 读取成功
        HAL_DBG_TRACE_INFO("DHT11 read success\n");
    }
    else
    {
        flag = 0; // 读取失败
        HAL_DBG_TRACE_INFO("DHT11 read failed\n");
    }
}
