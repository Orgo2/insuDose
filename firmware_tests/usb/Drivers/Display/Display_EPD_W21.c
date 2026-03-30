#include "Display_EPD_W21_spi.h"
#include "Display_EPD_W21.h"

typedef enum
{
  EPD_ASYNC_JOB_NONE = 0,
  EPD_ASYNC_JOB_WHITE_CLEAR,
  EPD_ASYNC_JOB_GUI_FRAME
} epd_async_job_t;

typedef enum
{
  EPD_ASYNC_STATE_IDLE = 0,
  EPD_ASYNC_STATE_RESET_LOW_DELAY,
  EPD_ASYNC_STATE_RESET_HIGH_DELAY,
  EPD_ASYNC_STATE_WAIT_IDLE_BEFORE_SWRESET,
  EPD_ASYNC_STATE_WAIT_IDLE_AFTER_SWRESET,
  EPD_ASYNC_STATE_WAIT_IDLE_AFTER_INIT,
  EPD_ASYNC_STATE_WAIT_IDLE_AFTER_UPDATE
} epd_async_state_t;

static volatile epd_async_job_t s_epd_async_job = EPD_ASYNC_JOB_NONE;
static volatile epd_async_state_t s_epd_async_state = EPD_ASYNC_STATE_IDLE;
static uint32_t s_epd_async_deadline_ms = 0u;
static uint32_t s_epd_async_ready_after_ms = 0u;
static const uint8_t *s_epd_async_image = 0;

static bool epd_async_tick_expired(uint32_t deadline_ms);
static void epd_async_begin(epd_async_job_t job, const uint8_t *image);
static void epd_async_write_white_frame(void);
static void epd_async_write_gui_frame(const uint8_t *image);
static void epd_async_send_full_init_sequence(void);
static void epd_async_send_gui_init_sequence(void);
static void epd_async_send_full_update(void);

//Delay Functions
void delay_xms(unsigned int xms)
{
    HAL_Delay(xms);
}

static bool epd_async_tick_expired(uint32_t deadline_ms)
{
  return ((int32_t)(HAL_GetTick() - deadline_ms) >= 0);
}

static void epd_async_begin(epd_async_job_t job, const uint8_t *image)
{
  s_epd_async_job = job;
  s_epd_async_image = image;
  s_epd_async_state = EPD_ASYNC_STATE_RESET_LOW_DELAY;
  s_epd_async_deadline_ms = HAL_GetTick() + 10u;
  EPD_W21_RST_0;
}

static void epd_async_send_full_init_sequence(void)
{
  EPD_W21_WriteCMD(0x01); //Driver output control
  EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);
  EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
  EPD_W21_WriteDATA(0x00);

  EPD_W21_WriteCMD(0x11); //data entry mode
  EPD_W21_WriteDATA(0x01);

  EPD_W21_WriteCMD(0x44); //set Ram-X address start/end position
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(EPD_WIDTH/8-1);

  EPD_W21_WriteCMD(0x45); //set Ram-Y address start/end position
  EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);
  EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(0x00);

  EPD_W21_WriteCMD(0x3C); //BorderWavefrom
  EPD_W21_WriteDATA(0x05);

  EPD_W21_WriteCMD(0x21); //Display update control
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(0x80);

  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
  EPD_W21_WriteDATA(0x80);

  EPD_W21_WriteCMD(0x4E); //set RAM x address count to 0
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteCMD(0x4F); //set RAM y address count
  EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);
  EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
}

static void epd_async_send_gui_init_sequence(void)
{
  EPD_W21_WriteCMD(0x01); //Driver output control
  EPD_W21_WriteDATA((EPD_HEIGHT+112-1)%256);
  EPD_W21_WriteDATA((EPD_HEIGHT+112-1)/256);
  EPD_W21_WriteDATA(0x01); //Show mirror

  EPD_W21_WriteCMD(0x11); //data entry mode
  EPD_W21_WriteDATA(0x01);

  EPD_W21_WriteCMD(0x44); //set Ram-X address start/end position
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(EPD_WIDTH/8-1);

  EPD_W21_WriteCMD(0x45); //set Ram-Y address start/end position
  EPD_W21_WriteDATA((EPD_HEIGHT+112-1)%256);
  EPD_W21_WriteDATA((EPD_HEIGHT+112-1)/256);
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(0x00);

  EPD_W21_WriteCMD(0x3C); //BorderWavefrom
  EPD_W21_WriteDATA(0x05);

  EPD_W21_WriteCMD(0x21); //Display update control
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteDATA(0x80);

  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
  EPD_W21_WriteDATA(0x80);

  EPD_W21_WriteCMD(0x4E); //set RAM x address count to 0
  EPD_W21_WriteDATA(0x00);
  EPD_W21_WriteCMD(0x4F); //set RAM y address count
  EPD_W21_WriteDATA((EPD_HEIGHT+112-1)%256);
  EPD_W21_WriteDATA((EPD_HEIGHT+112-1)/256);
}

static void epd_async_write_white_frame(void)
{
  unsigned int i;

  EPD_W21_WriteCMD(0x24);
  for (i = 0; i < EPD_ARRAY; i++) {
    EPD_W21_WriteDATA(0xff);
  }
}

static void epd_async_write_gui_frame(const uint8_t *image)
{
  unsigned int width;
  unsigned int height;
  unsigned int i;
  unsigned int j;

  if (image == 0) {
    return;
  }

  width = (EPD_WIDTH % 8 == 0) ? (EPD_WIDTH / 8) : (EPD_WIDTH / 8 + 1);
  height = EPD_HEIGHT;

  EPD_W21_WriteCMD(0x24);
  for (j = 0; j < height; j++) {
    for (i = 0; i < width; i++) {
      EPD_W21_WriteDATA(image[i + j * width]);
    }
  }
}

static void epd_async_send_full_update(void)
{
  EPD_W21_WriteCMD(0x22); //Display Update Control
  EPD_W21_WriteDATA(0xF7);
  EPD_W21_WriteCMD(0x20); //Activate Display Update Sequence
}

bool EPD_StartWhiteScreen_White_Async(void)
{
  if ((s_epd_async_state != EPD_ASYNC_STATE_IDLE) ||
      !epd_async_tick_expired(s_epd_async_ready_after_ms)) {
    return false;
  }

  epd_async_begin(EPD_ASYNC_JOB_WHITE_CLEAR, 0);
  return true;
}

bool EPD_StartDisplay_Async(const uint8_t *image)
{
  if ((image == 0) || (s_epd_async_state != EPD_ASYNC_STATE_IDLE) ||
      !epd_async_tick_expired(s_epd_async_ready_after_ms)) {
    return false;
  }

  epd_async_begin(EPD_ASYNC_JOB_GUI_FRAME, image);
  return true;
}

bool EPD_IsAsyncBusy(void)
{
  return (s_epd_async_state != EPD_ASYNC_STATE_IDLE);
}

bool EPD_IsAsyncStop2Safe(void)
{
  switch (s_epd_async_state)
  {
    case EPD_ASYNC_STATE_WAIT_IDLE_BEFORE_SWRESET:
    case EPD_ASYNC_STATE_WAIT_IDLE_AFTER_SWRESET:
    case EPD_ASYNC_STATE_WAIT_IDLE_AFTER_INIT:
    case EPD_ASYNC_STATE_WAIT_IDLE_AFTER_UPDATE:
      return true;

    default:
      return false;
  }
}

void EPD_AsyncTask(void)
{
  switch (s_epd_async_state)
  {
    case EPD_ASYNC_STATE_IDLE:
      return;

    case EPD_ASYNC_STATE_RESET_LOW_DELAY:
      if (!epd_async_tick_expired(s_epd_async_deadline_ms)) {
        return;
      }
      EPD_W21_RST_1;
      s_epd_async_deadline_ms = HAL_GetTick() + 10u;
      s_epd_async_state = EPD_ASYNC_STATE_RESET_HIGH_DELAY;
      return;

    case EPD_ASYNC_STATE_RESET_HIGH_DELAY:
      if (!epd_async_tick_expired(s_epd_async_deadline_ms)) {
        return;
      }
      s_epd_async_state = EPD_ASYNC_STATE_WAIT_IDLE_BEFORE_SWRESET;
      return;

    case EPD_ASYNC_STATE_WAIT_IDLE_BEFORE_SWRESET:
      if (isEPD_W21_BUSY != 0) {
        return;
      }
      EPD_W21_WriteCMD(0x12); //SWRESET
      s_epd_async_state = EPD_ASYNC_STATE_WAIT_IDLE_AFTER_SWRESET;
      return;

    case EPD_ASYNC_STATE_WAIT_IDLE_AFTER_SWRESET:
      if (isEPD_W21_BUSY != 0) {
        return;
      }
      if (s_epd_async_job == EPD_ASYNC_JOB_WHITE_CLEAR) {
        epd_async_send_full_init_sequence();
      } else if (s_epd_async_job == EPD_ASYNC_JOB_GUI_FRAME) {
        epd_async_send_gui_init_sequence();
      } else {
        s_epd_async_state = EPD_ASYNC_STATE_IDLE;
        s_epd_async_job = EPD_ASYNC_JOB_NONE;
      }
      s_epd_async_state = EPD_ASYNC_STATE_WAIT_IDLE_AFTER_INIT;
      return;

    case EPD_ASYNC_STATE_WAIT_IDLE_AFTER_INIT:
      if (isEPD_W21_BUSY != 0) {
        return;
      }
      if (s_epd_async_job == EPD_ASYNC_JOB_WHITE_CLEAR) {
        epd_async_write_white_frame();
      } else if (s_epd_async_job == EPD_ASYNC_JOB_GUI_FRAME) {
        epd_async_write_gui_frame(s_epd_async_image);
      } else {
        s_epd_async_state = EPD_ASYNC_STATE_IDLE;
        s_epd_async_job = EPD_ASYNC_JOB_NONE;
        return;
      }
      epd_async_send_full_update();
      s_epd_async_state = EPD_ASYNC_STATE_WAIT_IDLE_AFTER_UPDATE;
      return;

    case EPD_ASYNC_STATE_WAIT_IDLE_AFTER_UPDATE:
      if (isEPD_W21_BUSY != 0) {
        return;
      }
      EPD_W21_WriteCMD(0x10); //Enter deep sleep
      EPD_W21_WriteDATA(0x01);
      s_epd_async_ready_after_ms = HAL_GetTick() + 100u;
      s_epd_async_state = EPD_ASYNC_STATE_IDLE;
      s_epd_async_job = EPD_ASYNC_JOB_NONE;
      s_epd_async_image = 0;
      return;

    default:
      s_epd_async_state = EPD_ASYNC_STATE_IDLE;
      s_epd_async_job = EPD_ASYNC_JOB_NONE;
      s_epd_async_image = 0;
      return;
  }
}

////////////////////////////////////E-paper demo//////////////////////////////////////////////////////////
//Busy function
void Epaper_READBUSY(void)
{ 
  while (isEPD_W21_BUSY != 0)
  {
    __WFI();
  }
}
//Full screen update initialization
void EPD_HW_Init(void)
{
	EPD_W21_RST_0;  // Module reset   
	delay_xms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 
	
	Epaper_READBUSY();   
	EPD_W21_WriteCMD(0x12);  //SWRESET
	Epaper_READBUSY();   
		
	EPD_W21_WriteCMD(0x01); //Driver output control      
	EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
	EPD_W21_WriteDATA(0x00);

	EPD_W21_WriteCMD(0x11); //data entry mode       
	EPD_W21_WriteDATA(0x01);

	EPD_W21_WriteCMD(0x44); //set Ram-X address start/end position   
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(EPD_WIDTH/8-1);    

	EPD_W21_WriteCMD(0x45); //set Ram-Y address start/end position          
	EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00); 

	EPD_W21_WriteCMD(0x3C); //BorderWavefrom
	EPD_W21_WriteDATA(0x05);	

	EPD_W21_WriteCMD(0x21); //  Display update control
	EPD_W21_WriteDATA(0x00);		
  EPD_W21_WriteDATA(0x80);	
	
  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
	EPD_W21_WriteDATA(0x80);	

	EPD_W21_WriteCMD(0x4E);   // set RAM x address count to 0;
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteCMD(0x4F);   // set RAM y address count to 0X199;    
	EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
  Epaper_READBUSY();
	
}
//Fast update initialization
void EPD_HW_Init_Fast(void)
{
	EPD_W21_RST_0;  // Module reset   
	delay_xms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 
  
	EPD_W21_WriteCMD(0x12);  //SWRESET
	Epaper_READBUSY();   
 	
  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
	EPD_W21_WriteDATA(0x80);	
	  	
	EPD_W21_WriteCMD(0x22); // Load temperature value
	EPD_W21_WriteDATA(0xB1);		
  EPD_W21_WriteCMD(0x20);	
  Epaper_READBUSY();   

	EPD_W21_WriteCMD(0x1A); // Write to temperature register
	EPD_W21_WriteDATA(0x64);		
  EPD_W21_WriteDATA(0x00);	
				  	
	EPD_W21_WriteCMD(0x22); // Load temperature value
	EPD_W21_WriteDATA(0x91);		
  EPD_W21_WriteCMD(0x20);	
	Epaper_READBUSY();   
}
//Fast update initialization
void EPD_HW_Init_4Gray(void)
{
	EPD_W21_RST_0;  // Module reset   
	delay_xms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 
  
	EPD_W21_WriteCMD(0x12);  //SWRESET
	Epaper_READBUSY();   
 	
  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
	EPD_W21_WriteDATA(0x80);	
	  	
	EPD_W21_WriteCMD(0x22); // Load temperature value
	EPD_W21_WriteDATA(0xB1);		
  EPD_W21_WriteCMD(0x20);	
  Epaper_READBUSY();   

	EPD_W21_WriteCMD(0x1A); // Write to temperature register
	EPD_W21_WriteDATA(0x5A); //4 Gray		
  EPD_W21_WriteDATA(0x00);	
				  	
	EPD_W21_WriteCMD(0x22); // Load temperature value
	EPD_W21_WriteDATA(0x91);		
  EPD_W21_WriteCMD(0x20);	
	Epaper_READBUSY();   
	
	EPD_W21_WriteCMD(0x3C); //BorderWavefrom
	EPD_W21_WriteDATA(0x05);	
	
	EPD_W21_WriteCMD(0x2C);     //VCOM Voltage
	EPD_W21_WriteDATA(0x08);    

}
//////////////////////////////Display Update Function///////////////////////////////////////////////////////
//Full screen update update function
void EPD_Update(void)
{   
  EPD_W21_WriteCMD(0x22); //Display Update Control
  EPD_W21_WriteDATA(0xF7);   
  EPD_W21_WriteCMD(0x20); //Activate Display Update Sequence
  Epaper_READBUSY();   

}
//Fast update  update function
void EPD_Update_Fast(void)
{   
  EPD_W21_WriteCMD(0x22); //Display Update Control
  EPD_W21_WriteDATA(0xC7);   
  EPD_W21_WriteCMD(0x20); //Activate Display Update Sequence
  Epaper_READBUSY();   

}
//Partial update update function
void EPD_Part_Update(void)
{
	EPD_W21_WriteCMD(0x22); //Display Update Control
	EPD_W21_WriteDATA(0xFF);   
	EPD_W21_WriteCMD(0x20); //Activate Display Update Sequence
	Epaper_READBUSY(); 			
}
//////////////////////////////Display Data Transfer Function////////////////////////////////////////////
//Full screen update display function
void EPD_WhiteScreen_ALL(const unsigned char *datas)
{
   unsigned int i;	
  EPD_W21_WriteCMD(0x24);   //write RAM for black(0)/white (1)
  for(i=0;i<EPD_ARRAY;i++)
   {               
     EPD_W21_WriteDATA(datas[i]);
   }
   EPD_Update();	 
}
//Fast update display function
void EPD_WhiteScreen_ALL_Fast(const unsigned char *datas)
{
   unsigned int i;	
  EPD_W21_WriteCMD(0x24);   //write RAM for black(0)/white (1)
   for(i=0;i<EPD_ARRAY;i++)
   {               
     EPD_W21_WriteDATA(datas[i]);
   } 
	 
   EPD_Update_Fast();	 
}

//Clear screen display
void EPD_WhiteScreen_White(void)
{
 unsigned int i;
 EPD_W21_WriteCMD(0x24);   //write RAM for black(0)/white (1)
 for(i=0;i<EPD_ARRAY;i++)
 {
		EPD_W21_WriteDATA(0xff);
	}
	EPD_Update();
}
//Display all black
void EPD_WhiteScreen_Black(void)
{
 unsigned int i;
 EPD_W21_WriteCMD(0x24);   //write RAM for black(0)/white (1)
 for(i=0;i<EPD_ARRAY;i++)
 {
		EPD_W21_WriteDATA(0x00);
	}
	EPD_Update();
}
//Partial update of background display, this function is necessary, please do not delete it!!!
void EPD_SetRAMValue_BaseMap( const unsigned char * datas)
{
	unsigned int i;   	
  EPD_W21_WriteCMD(0x24);   //Write Black and White image to RAM
  for(i=0;i<EPD_ARRAY;i++)
   {               
     EPD_W21_WriteDATA(datas[i]);
   }
  EPD_W21_WriteCMD(0x26);   //Write Black and White image to RAM
  for(i=0;i<EPD_ARRAY;i++)
   {               
     EPD_W21_WriteDATA(datas[i]);
   }
   EPD_Update();		 
	 
}
//Partial update display
void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE)
{
	unsigned int i;  
	unsigned int x_end,y_end;
	
	x_start=x_start/8; //x address start
	x_end=x_start+PART_LINE/8-1; //x address end
	y_start=y_start; //Y address start
	y_end=y_start+PART_COLUMN-1; //Y address end
	
	EPD_W21_RST_0;  // Module reset   
	delay_xms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 	
	EPD_W21_WriteCMD(0x3C); //BorderWavefrom,
	EPD_W21_WriteDATA(0x80);	
	
	EPD_W21_WriteCMD(0x44);       // set RAM x address start/end
	EPD_W21_WriteDATA(x_start);  //x address start
	EPD_W21_WriteDATA(x_end);   //y address end   
	EPD_W21_WriteCMD(0x45);    // set RAM y address start/end
	EPD_W21_WriteDATA(y_start%256);  //y address start2 
	EPD_W21_WriteDATA(y_start/256); //y address start1 
	EPD_W21_WriteDATA(y_end%256);  //y address end2 
	EPD_W21_WriteDATA(y_end/256); //y address end1   

	EPD_W21_WriteCMD(0x4E);        // set RAM x address count to 0;
	EPD_W21_WriteDATA(x_start);   //x start address
	EPD_W21_WriteCMD(0x4F);      // set RAM y address count to 0X127;    
	EPD_W21_WriteDATA(y_start%256);//y address start2
	EPD_W21_WriteDATA(y_start/256);//y address start1
	
	
	 EPD_W21_WriteCMD(0x24);   //Write Black and White image to RAM
   for(i=0;i<PART_COLUMN*PART_LINE/8;i++)
   {                         
     EPD_W21_WriteDATA(datas[i]);
   } 
	 EPD_Part_Update();

}
//Full screen partial update display
void EPD_Dis_PartAll(const unsigned char * datas)
{
	unsigned int i;  
	unsigned int PART_COLUMN, PART_LINE;
	PART_COLUMN=EPD_HEIGHT,PART_LINE=EPD_WIDTH;

	EPD_W21_RST_0;  // Module reset   
	delay_xms(10); //At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 	
	EPD_W21_WriteCMD(0x3C); //BorderWavefrom,
	EPD_W21_WriteDATA(0x80);	


	EPD_W21_WriteCMD(0x24);   //Write Black and White image to RAM
	 for(i=0;i<PART_COLUMN*PART_LINE/8;i++)
	 {                         
		 EPD_W21_WriteDATA(datas[i]);
	 } 
	 EPD_Part_Update();

}
//Deep sleep function   
void EPD_DeepSleep(void)
{  	
  EPD_W21_WriteCMD(0x10); //Enter deep sleep
  EPD_W21_WriteDATA(0x01); 
  delay_xms(100);
}

//Partial update write address and data
void EPD_Dis_Part_RAM(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE)
{
	unsigned int i;  
	unsigned int x_end,y_end;
	
	x_start=x_start/8; //x address start
	x_end=x_start+PART_LINE/8-1; //x address end
	
	y_start=y_start-1; //Y address start
	y_end=y_start+PART_COLUMN-1; //Y address end
	
	EPD_W21_RST_0;  // Module reset   
	delay_xms(10);//At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 	
	EPD_W21_WriteCMD(0x3C); //BorderWavefrom,
	EPD_W21_WriteDATA(0x80);	
	
	EPD_W21_WriteCMD(0x44);       // set RAM x address start/end
	EPD_W21_WriteDATA(x_start);  //x address start
	EPD_W21_WriteDATA(x_end);   //y address end   
	EPD_W21_WriteCMD(0x45);     // set RAM y address start/end
	EPD_W21_WriteDATA(y_start%256);  //y address start2 
	EPD_W21_WriteDATA(y_start/256); //y address start1 
	EPD_W21_WriteDATA(y_end%256);  //y address end2 
	EPD_W21_WriteDATA(y_end/256); //y address end1   

	EPD_W21_WriteCMD(0x4E);   // set RAM x address count to 0;
	EPD_W21_WriteDATA(x_start);   //x start address
	EPD_W21_WriteCMD(0x4F);   // set RAM y address count to 0X127;    
	EPD_W21_WriteDATA(y_start%256); //y address start2
	EPD_W21_WriteDATA(y_start/256); //y address start1
		
	EPD_W21_WriteCMD(0x24);   //Write Black and White image to RAM
  for(i=0;i<PART_COLUMN*PART_LINE/8;i++)
   {                         
     EPD_W21_WriteDATA(datas[i]);
   } 
}
//Clock display
void EPD_Dis_Part_Time(unsigned int x_startA,unsigned int y_startA,const unsigned char * datasA,
	                       unsigned int x_startB,unsigned int y_startB,const unsigned char * datasB,
												 unsigned int x_startC,unsigned int y_startC,const unsigned char * datasC,
												 unsigned int x_startD,unsigned int y_startD,const unsigned char * datasD,
											   unsigned int x_startE,unsigned int y_startE,const unsigned char * datasE,
												 unsigned int PART_COLUMN,unsigned int PART_LINE
	                      )
{
	EPD_Dis_Part_RAM(x_startA,y_startA,datasA,PART_COLUMN,PART_LINE);
	EPD_Dis_Part_RAM(x_startB,y_startB,datasB,PART_COLUMN,PART_LINE);
	EPD_Dis_Part_RAM(x_startC,y_startC,datasC,PART_COLUMN,PART_LINE);
	EPD_Dis_Part_RAM(x_startD,y_startD,datasD,PART_COLUMN,PART_LINE);
	EPD_Dis_Part_RAM(x_startE,y_startE,datasE,PART_COLUMN,PART_LINE);
	EPD_Part_Update();
}												 




////////////////////////////////Other newly added functions////////////////////////////////////////////
//Display rotation 180 degrees initialization
void EPD_HW_Init_180(void)
{
	EPD_W21_RST_0;  // Module reset   
	delay_xms(10); //At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 
	
	Epaper_READBUSY();   
	EPD_W21_WriteCMD(0x12);  //SWRESET
	Epaper_READBUSY();   
	
	EPD_W21_WriteCMD(0x3C); //BorderWavefrom
	EPD_W21_WriteDATA(0x05);
	
	EPD_W21_WriteCMD(0x01); //Driver output control      
	EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);
	EPD_W21_WriteDATA(0x00); 

	EPD_W21_WriteCMD(0x11); //data entry mode       
	EPD_W21_WriteDATA(0x02);

	EPD_W21_WriteCMD(0x44); //set Ram-X address start/end position   
	EPD_W21_WriteDATA(EPD_WIDTH/8-1);    
	EPD_W21_WriteDATA(0x00);  

	EPD_W21_WriteCMD(0x45); //set Ram-Y address start/end position          
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00); 
  EPD_W21_WriteDATA((EPD_HEIGHT-1)%256);  
	EPD_W21_WriteDATA((EPD_HEIGHT-1)/256);

	EPD_W21_WriteCMD(0x21); //  Display update control
	EPD_W21_WriteDATA(0x00);		
  EPD_W21_WriteDATA(0x80);	
	
  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
	EPD_W21_WriteDATA(0x80);	

	EPD_W21_WriteCMD(0x4E);   // set RAM x address count to 0;
	EPD_W21_WriteDATA(EPD_WIDTH/8-1);  
	EPD_W21_WriteCMD(0x4F);   // set RAM y address count to 0X199;    
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00);
  Epaper_READBUSY();
}
// GUI initialization
void EPD_HW_Init_GUI(void)
{
  EPD_W21_RST_0;  // Module reset   
	delay_xms(10); //At least 10ms delay 
	EPD_W21_RST_1;
	delay_xms(10); //At least 10ms delay 
	
	Epaper_READBUSY();   
	EPD_W21_WriteCMD(0x12);  //SWRESET
	Epaper_READBUSY();   
		
	EPD_W21_WriteCMD(0x01); //Driver output control      
	EPD_W21_WriteDATA((EPD_HEIGHT+112-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT+112-1)/256);
	EPD_W21_WriteDATA(0x01);//Show mirror

	EPD_W21_WriteCMD(0x11); //data entry mode       
	EPD_W21_WriteDATA(0x01);

	EPD_W21_WriteCMD(0x44); //set Ram-X address start/end position   
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(EPD_WIDTH/8-1);    

	EPD_W21_WriteCMD(0x45); //set Ram-Y address start/end position          
	EPD_W21_WriteDATA((EPD_HEIGHT+112-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT+112-1)/256);
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteDATA(0x00); 

	EPD_W21_WriteCMD(0x3C); //BorderWavefrom
	EPD_W21_WriteDATA(0x05);	

	EPD_W21_WriteCMD(0x21); //  Display update control
	EPD_W21_WriteDATA(0x00);		
  EPD_W21_WriteDATA(0x80);	
	
  EPD_W21_WriteCMD(0x18); //Read built-in temperature sensor
	EPD_W21_WriteDATA(0x80);	

	EPD_W21_WriteCMD(0x4E);   // set RAM x address count to 0;
	EPD_W21_WriteDATA(0x00);
	EPD_W21_WriteCMD(0x4F);   // set RAM y address count to 0X199;    
	EPD_W21_WriteDATA((EPD_HEIGHT+112-1)%256);    
	EPD_W21_WriteDATA((EPD_HEIGHT+112-1)/256);
  Epaper_READBUSY();
}

//GUI display
void EPD_Display(unsigned char *Image)
{
	unsigned int Width, Height,i,j;
	Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
	Height = EPD_HEIGHT;

	EPD_W21_WriteCMD(0x24);
	for ( j = 0; j < Height; j++) {
			for ( i = 0; i < Width; i++) {
				 EPD_W21_WriteDATA(Image[i + j * Width]);
			}
	}
	EPD_Update();		 
}



//4 Gray///////////////////////////////////////////////
unsigned char In2bytes_Out1byte_RAM1(unsigned char data1,unsigned char data2)
{
  unsigned int i; 
	unsigned char TempData1,TempData2;
	unsigned char outdata=0x00;
	TempData1=data1;
	TempData2=data2;
	
    for(i=0;i<4;i++)
     { 
        outdata=outdata<<1;
        if( ((TempData1&0xC0)==0xC0) || ((TempData1&0xC0)==0x40))
           outdata=outdata|0x01;
        else 
          outdata=outdata|0x00;
        TempData1=TempData1<<2;
     }

    for(i=0;i<4;i++)
     { 
        outdata=outdata<<1;
         if((TempData2&0xC0)==0xC0||(TempData2&0xC0)==0x40)
           outdata=outdata|0x01;
        else 
          outdata=outdata|0x00;
        TempData2=TempData2<<2;
     }
		 return outdata;
}
unsigned char In2bytes_Out1byte_RAM2(unsigned char data1,unsigned char data2)
{
  unsigned int i; 
  unsigned char TempData1,TempData2;
	unsigned char outdata=0x00;
TempData1=data1;
TempData2=data2;
	
    for(i=0;i<4;i++)
     { 
        outdata=outdata<<1;
        if( ((TempData1&0xC0)==0xC0) || ((TempData1&0xC0)==0x80))
           outdata=outdata|0x01;
        else 
          outdata=outdata|0x00;
        TempData1=TempData1<<2;
     }

    for(i=0;i<4;i++)
     { 
        outdata=outdata<<1;
         if((TempData2&0xC0)==0xC0||(TempData2&0xC0)==0x80)
           outdata=outdata|0x01;
        else 
          outdata=outdata|0x00;
        TempData2=TempData2<<2;
     }
		 return outdata;
}

void EPD_WhiteScreen_ALL_4Gray(const unsigned char *datas)
{
   unsigned int i;
	 unsigned char tempOriginal;   
	
	
    EPD_W21_WriteCMD(0x24);   //write RAM for black(0)/white (1)
   for(i=0;i<EPD_ARRAY*2;i+=2)
   {               
		tempOriginal= In2bytes_Out1byte_RAM1( *(datas+i),*(datas+i+1));
		 EPD_W21_WriteDATA(~tempOriginal); 
   } 
	 EPD_W21_WriteCMD(0x26);   //write RAM for black(0)/white (1)
   for(i=0;i<EPD_ARRAY*2;i+=2)
   {               
		tempOriginal= In2bytes_Out1byte_RAM2( *(datas+i),*(datas+i+1));
		 EPD_W21_WriteDATA(~tempOriginal); 
   }
   EPD_Update_Fast();	 	 
}


/***********************************************************
						end file
***********************************************************/

