#include "Display_EPD_W21_spi.h"

//E-paper GPIO initialization
void EPD_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Povoľ takty portov (ak ich nepovoľuješ v CubeMX)
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // CS pin (PA15)
    GPIO_InitStruct.Pin = EPD_CS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(EPD_CS_GPIO_Port, &GPIO_InitStruct);

    // DC pin (PB4)
    GPIO_InitStruct.Pin = EPD_DC_Pin;
    HAL_GPIO_Init(EPD_DC_GPIO_Port, &GPIO_InitStruct);

    // RST pin (PB6)
    GPIO_InitStruct.Pin = EPD_RST_Pin;
    HAL_GPIO_Init(EPD_RST_GPIO_Port, &GPIO_InitStruct);

    // MOSI pin (PB5) - **len ak používaš softvérový SPI (bitbanging)**!
    GPIO_InitStruct.Pin = EPD_MOSI_Pin;
    HAL_GPIO_Init(EPD_MOSI_GPIO_Port, &GPIO_InitStruct);

    // CLK pin (PB3) - **len ak používaš softvérový SPI (bitbanging)**!
    GPIO_InitStruct.Pin = EPD_SCK_Pin;
    HAL_GPIO_Init(EPD_SCK_GPIO_Port, &GPIO_InitStruct);

    // BUSY pin (PB7) - vstup s pull-up
    GPIO_InitStruct.Pin = EPD_BUSY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(EPD_BUSY_GPIO_Port, &GPIO_InitStruct);
}



//SPI write byte
void SPI_Write(unsigned char value)
{				   			 
	unsigned char i;
  EPD_W21_CLK_0;  
	for(i=0;i<8;i++)
	{ 
		if(value&0x80)
		  EPD_W21_MOSI_1 ;
		else
		  EPD_W21_MOSI_0 ;
		EPD_W21_CLK_1;  
	  EPD_W21_CLK_0;  
		value=value<<1;
  }
}

//SPI write command
void EPD_W21_WriteCMD(unsigned char command)
{
	EPD_W21_CS_0;
	EPD_W21_DC_0;  // D/C#   0:command  1:data
	SPI_Write(command);
	EPD_W21_CS_1;
}
//SPI write data
void EPD_W21_WriteDATA(unsigned char datas)
{
	EPD_W21_CS_0;
	EPD_W21_DC_1;  // D/C#   0:command  1:data
	SPI_Write(datas);
	EPD_W21_CS_1;
}






