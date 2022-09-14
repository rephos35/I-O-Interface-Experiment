//---------------------------------------------Includes-------------------------------------------------//

#include "ht32.h"
#include "ht32_board.h"
#include "ht32_board_config.h"

//---------------------------------------------Private variables-------------------------------------------------//

TM_TimeBaseInitTypeDef MCTM_TimeBaseInitStructure;
TM_OutputInitTypeDef MCTM_OutputInitStructure;
MCTM_CHBRKCTRInitTypeDef MCTM_CHBRKCTRInitStructure;

//---------------------------------------------Global variables-------------------------------------------------//

u32 ring=0;
u32 initial=1;
u32 up=0, down=0;
//---------------------------------------------function-------------------------------------------------//

void delay(float x);
void RING1(float *freq);
void RING2(float *freq);
void RING3(float *freq);
void LED(int R, int G, int B);
void WET(void);
void DIS1(void);
void DIS2(void);
void DIS3(void);
//??
#define HTCFG_MCTM_RELOAD                           (48000000/2000)
#define HTCFG_MCTM_DEAD_TIME                        (72)

//---------------------------------------------RING-----------------------------------------------------//
__IO uint32_t wMctmUpdateDownCounter = 0;

void PWM_Configuration(int Frequency){
	
	CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
  TM_TimeBaseInitTypeDef TM_TimeBaseInitStructure;
  TM_OutputInitTypeDef TM_OutputInitStructure;
  uint32_t wCRR = 0, wPSCR = 0;

  /* Enable PCLK of BUZZER and AFIO                                                                         */
  BUZZER_TM_CLK(CKCUClock) = 1;
  CKCUClock.Bit.AFIO       = 1;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);

  /* Configure the BUZZER_GPIO_PIN as TM channel output AFIO function                                       */
  HT32F_DVB_GPxConfig(BUZZER_GPIO_ID, BUZZER_AFIO_PIN, BUZZER_AFIO_MODE);

  /* Compute CRR and PSCR value                                                                             */
	wCRR = (SystemCoreClock / Frequency) - 1;
  while ((wCRR / (wPSCR + 1)) > 0xFFFF)
  {
    wPSCR++;
  }
  wCRR = wCRR / (wPSCR + 1);
	
  /* Init BUZZER TM time-base                                                                               */
  TM_TimeBaseInitStructure.CounterReload = wCRR;
  TM_TimeBaseInitStructure.Prescaler = wPSCR;
  TM_TimeBaseInitStructure.RepetitionCounter = 0;
  TM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
  TM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
  TM_TimeBaseInit(BUZZER_TM, &TM_TimeBaseInitStructure);

  /* Clear Update Event Interrupt flag                                                                      */
  TM_ClearFlag(BUZZER_TM, TM_FLAG_UEV);

  /* Init BUZZER TM Channel x to output PWM waveform with 33% duty                                          */
  TM_OutputInitStructure.Channel = BUZZER_TM_CHANNEL;
  TM_OutputInitStructure.OutputMode = TM_OM_PWM2;
  TM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
  TM_OutputInitStructure.ControlN = TM_CHCTL_DISABLE;//TM_CHCTL_ENABLE;
  TM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;
  TM_OutputInitStructure.PolarityN = TM_CHP_NONINVERTED;
  TM_OutputInitStructure.IdleState = MCTM_OIS_LOW;
  TM_OutputInitStructure.IdleStateN = MCTM_OIS_HIGH;
  TM_OutputInitStructure.Compare = ((wCRR + 1) * (100 - 33)) / 100;
  TM_OutputInit(BUZZER_TM, &TM_OutputInitStructure);


  /* Enable interrupt of BUZZER TM update event                                                             */
  NVIC_EnableIRQ(BUZZER_IRQn);
  TM_IntConfig(BUZZER_TM, TM_INT_UEV, ENABLE);

  #if (LIBCFG_MCTM0)
  if (BUZZER_TM == HT_MCTM0)
  {
    /* BUZZER TM Channel Main Output enable                                                                   */
    MCTM_CHMOECmd(BUZZER_TM, ENABLE);
  }
  #endif

  /* BUZZER TM counter enable                                                                               */
  TM_Cmd(BUZZER_TM, ENABLE);	
}
#define BEE_NBR 2        
#define FREQ_DO 262
#define FREQ_RE 294
#define FREQ_MI 330
#define FREQ_FA 349
#define FREQ_SO 392
#define FREQ_LA 440
#define FREQ_SI 494
#define FREQ_DO_H 523
#define RE 578
#define SO 692

float freq1[2] = {FREQ_RE, FREQ_RE};
float freq2[2] = {SO, RE};
float freq3[2] = {SO, RE};

//---------------------------------------------RING-----------------------------------------------------//

//---------------------------------------------WET-----------------------------------------------------//
u32 Humidity, Temperature;
u32 wet=0, tem=0;
void delay_us(u32 us)
{
	u32 i;
	SYSTICK_ClockSourceConfig(SYSTICK_SRC_STCLK);          //选择外部参考时钟作为SysTick时钟源。8MHZ
	SYSTICK_SetReloadValue(SystemCoreClock / 8 / 1000000); // 重装计数初值
	SYSTICK_IntConfig(DISABLE);                            // 是否开启中断
	SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);             //清空定时器
	SYSTICK_CounterCmd(SYSTICK_COUNTER_ENABLE);            //使能
	for( i = 0;i < us;i++ )
	{
		while( !( (SysTick->CTRL) & (1<<16) ) ); 
	}
 
	SYSTICK_CounterCmd(SYSTICK_COUNTER_DISABLE); //关闭
	SYSTICK_CounterCmd(SYSTICK_COUNTER_CLEAR);	 //复位清零
}
void delay_ms(u16 ms){ //mS毫秒级延时程序 	  
	while( ms-- != 0){
		delay_us(1000);	//调用1000微秒的延时
	}
}
void delay_s(u16 s){ //S秒级延时程序	 		  	  
	while( s-- != 0){
		delay_ms(1000);	//调用1000毫秒的延时
	}
} 

char Data[5];
u16 temperature;
u16 humidity;

char read_data()
{
  char data = 0;
  char i;

  //需讀取8bit
  for(i=0; i<8; i++)
  {
    //等待位元起始訊號結束
    while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_7) == RESET);
    //等待1/0數據通用時間
    delay_us(33);
    //判斷高位超時，超時為1，已回歸低位(下一bit起始訊號)為0
    if(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_7) == SET)
      data |= (1<<(7-i)); //高位在前，低位在后
    //若數數据為1，需等待其回歸低位(下一bit起始訊號)
    while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_7) == SET);
  }
  return data;
}
void start_read_DHT(u16 *temp,u16 *humi)
{
  u8 i;
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_7, GPIO_DIR_OUT);
  //開始訊號
  //延遲要大于18ms，以便 DHT11 能讀到開始訊號
  GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_7, RESET);
  delay_ms(18);
  GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_7, SET);
  delay_us(30);
  
  //切換為接收  GPIO output=>input
  //其餘設定請於GPIO_Configuration()內設定
  GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_7, GPIO_DIR_IN);
	GPIO_PullResistorConfig(HT_GPIOA, GPIO_PIN_7, GPIO_PR_DISABLE);
	GPIO_InputConfig(HT_GPIOA, GPIO_PIN_7, ENABLE);
  //等待確認DHT11響應訊號
  while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_7) == SET);
  delay_us(40);
  //等待確認DHT11響應訊號歸位
  while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_7) == RESET);
  delay_us(40);
  //確認開始傳送資料
  while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_7) == SET);

  for(i=0;i<5;i++){
    //讀取封包
    //數據格式： 8bit濕度整數位+8bit濕度小數位
    //        +8bit溫度整數位+8bit溫度小數位
    //        +8bit校驗和
    Data[i] = read_data();
  }
	if((Data[0]+Data[1]+Data[2]+Data[3])==Data[4])
  {
       *humi=(Data[0]*256+Data[1]);
       *temp=(Data[2]*256+Data[3]);
  }
  //接收完畢切換為傳送 GPIO input=>output
  GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_7, GPIO_DIR_OUT);
  //無訊號時常態高位
  GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_7, SET);
}



//---------------------------------------------WET-----------------------------------------------------//

//---------------------------------------------EXTI-----------------------------------------------------//

void EXTI_configuration(void)
{
  EXTI_InitTypeDef EXTI_InitStruct;

  AFIO_EXTISourceConfig(AFIO_EXTI_CH_12, AFIO_ESS_PB);
  EXTI_InitStruct.EXTI_Channel=EXTI_CHANNEL_12;
  EXTI_InitStruct.EXTI_Debounce=EXTI_DEBOUNCE_ENABLE;//
  EXTI_InitStruct.EXTI_DebounceCnt=32000;// 
	//for 1ms, 1ms*32MHz=32000
  EXTI_InitStruct.EXTI_IntType=EXTI_POSITIVE_EDGE;
  EXTI_Init(&EXTI_InitStruct);
  EXTI_IntConfig(EXTI_CHANNEL_12, ENABLE);

}
void NVIC_configuration(void){
  NVIC_EnableIRQ(EXTI12_IRQn);
	NVIC_SetPriority(EXTI4_15_IRQn, 0);
 
}


//---------------------------------------------EXTI-----------------------------------------------------//

//---------------------------------------------DIS-----------------------------------------------------//
vu32 BFTM_OverCnt;
float Distance1, Distance2, Distance3;
void BFTM_Configuration(void){
  //若不設定則溢位時間約89s，溢位後計時歸零

  //單次溢位時間10s
  int Compare = SystemCoreClock * 10;
  //設定溢位時間
  BFTM_SetCompare(HT_BFTM0,Compare-1);
  //不啟用單次模式
  BFTM_OneShotModeCmd(HT_BFTM0, DISABLE);
  //BFTM開啟
  BFTM_EnaCmd(HT_BFTM0, ENABLE);
}

void BFTM_ReStart(void){
  BFTM_OverCnt = 0;
  BFTM_SetCounter(HT_BFTM0, 0);
  BFTM_EnaCmd(HT_BFTM0, ENABLE);
}
void BFTM_end(void){//最長計時約89s後溢位
  BFTM_EnaCmd(HT_BFTM0, DISABLE);
}
float BFTM_us(void){
  // Reload=SYSTICK          => 1s
  // Reload=SYSTICK/1000     => 1ms
  // Reload=SYSTICK/1000000  => 1us
  float count = BFTM_GetCounter(HT_BFTM0);
  return count/(SystemCoreClock/1000000) + BFTM_OverCnt*1000000*10;;
}




//---------------------------------------------DIS-----------------------------------------------------//


void CKCU_configuration(void)
{
  CKCU_PeripClockConfig_TypeDef CKCUClock = {{0}};
	CKCUClock.Bit.PA = 1;
	CKCUClock.Bit.PB = 1;
	CKCUClock.Bit.PC = 1;
	CKCUClock.Bit.PD = 1;
	CKCUClock.Bit.AFIO = 1;
	CKCUClock.Bit.EXTI = 1;
	CKCUClock.Bit.MCTM0 = 1;
	CKCUClock.Bit.BFTM0 = 1;
  CKCU_PeripClockConfig(CKCUClock, ENABLE);
}

void GPIO_configuration(void)
{
  //WET 
	AFIO_GPxConfig(GPIO_PA, AFIO_PIN_7, AFIO_FUN_GPIO);
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_7, GPIO_DIR_OUT);
	

	//EXTI:BUTTON
  AFIO_GPxConfig(GPIO_PB,AFIO_PIN_12,AFIO_FUN_GPIO);
  GPIO_DirectionConfig(HT_GPIOB, GPIO_PIN_12, GPIO_DIR_IN);
  GPIO_PullResistorConfig(HT_GPIOB, GPIO_PIN_12, GPIO_PR_DISABLE);
  GPIO_InputConfig(HT_GPIOB, GPIO_PIN_12, ENABLE);
	
	
	//LED+MCTM 
	AFIO_GPxConfig(GPIO_PC, AFIO_PIN_1, AFIO_FUN_MCTM_GPTM);//C1  MT0 R
  AFIO_GPxConfig(GPIO_PB, AFIO_PIN_2, AFIO_FUN_MCTM_GPTM);//B2  MT2 G 
	AFIO_GPxConfig(GPIO_PD, AFIO_PIN_3, AFIO_FUN_MCTM_GPTM);//D3 MT3 B
  AFIO_GPxConfig(GPIO_PB, AFIO_PIN_4, AFIO_FUN_MCTM_GPTM);//MT_BRK
	
	
	//MOTOR
  AFIO_GPxConfig(GPIO_PC, AFIO_PIN_14, AFIO_FUN_GPIO);    
  GPIO_DirectionConfig(HT_GPIOC, GPIO_PIN_14, GPIO_DIR_OUT);
 
	
	//DIS
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_1, GPIO_DIR_IN);//echo 
	GPIO_PullResistorConfig(HT_GPIOA,GPIO_PIN_1,GPIO_PR_UP);
	GPIO_InputConfig(HT_GPIOA, GPIO_PIN_1,ENABLE);
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_0, GPIO_DIR_OUT);//trig 
	
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_2, GPIO_DIR_IN);
	GPIO_PullResistorConfig(HT_GPIOA,GPIO_PIN_2,GPIO_PR_UP);
	GPIO_InputConfig(HT_GPIOA, GPIO_PIN_2,ENABLE);
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_3, GPIO_DIR_OUT);
	
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_14, GPIO_DIR_IN);
	GPIO_PullResistorConfig(HT_GPIOA,GPIO_PIN_14,GPIO_PR_UP);
	GPIO_InputConfig(HT_GPIOA, GPIO_PIN_14,ENABLE);
	GPIO_DirectionConfig(HT_GPIOA, GPIO_PIN_15, GPIO_DIR_OUT);
	
	//RING
	//GPIO_PA, GPIO_PIN_10
	
}

void MCTM_configuration(void)
{
	
	//LED
	/* MCTM Time Base configuration                                                                            */
  MCTM_TimeBaseInitStructure.CounterReload = HTCFG_MCTM_RELOAD - 1;
  MCTM_TimeBaseInitStructure.Prescaler = 0;
  MCTM_TimeBaseInitStructure.RepetitionCounter = 0;
  MCTM_TimeBaseInitStructure.CounterMode = TM_CNT_MODE_UP;
  MCTM_TimeBaseInitStructure.PSCReloadTime = TM_PSC_RLD_IMMEDIATE;
  TM_TimeBaseInit(HT_MCTM0, &MCTM_TimeBaseInitStructure);

  /* MCTM Channel 0 output configuration                                                                    */
  MCTM_OutputInitStructure.Channel = TM_CH_0;
  MCTM_OutputInitStructure.OutputMode = TM_OM_PWM2;  //因三色LED 輸出，需用PWM2
  MCTM_OutputInitStructure.Control = TM_CHCTL_ENABLE;
  MCTM_OutputInitStructure.ControlN = TM_CHCTL_ENABLE;
  MCTM_OutputInitStructure.Polarity = TM_CHP_NONINVERTED;//TM_CHP_NONINVERTED
  MCTM_OutputInitStructure.PolarityN = TM_CHP_NONINVERTED;
  MCTM_OutputInitStructure.IdleState = MCTM_OIS_LOW;
  MCTM_OutputInitStructure.IdleStateN = MCTM_OIS_HIGH;
	
  MCTM_OutputInitStructure.Channel = TM_CH_0;
  MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * 0/255;
  TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);

  MCTM_OutputInitStructure.Channel = TM_CH_2;
  MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * 0/255;
  TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);

	
	MCTM_OutputInitStructure.Channel = TM_CH_3;
  MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * 0/255;
  TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);
	
	
  /* MCTM Off State, lock, Break, Automatic Output enable, dead time configuration                          */
  MCTM_CHBRKCTRInitStructure.OSSRState = MCTM_OSSR_STATE_ENABLE;
  MCTM_CHBRKCTRInitStructure.OSSIState = MCTM_OSSI_STATE_ENABLE;
  MCTM_CHBRKCTRInitStructure.LockLevel = MCTM_LOCK_LEVEL_2;
  MCTM_CHBRKCTRInitStructure.Break0 = MCTM_BREAK_DISABLE;  ///!!!!!!!!記得要關掉
  MCTM_CHBRKCTRInitStructure.Break0Polarity = MCTM_BREAK_POLARITY_LOW;
  MCTM_CHBRKCTRInitStructure.AutomaticOutput = MCTM_CHAOE_ENABLE;
  MCTM_CHBRKCTRInitStructure.DeadTime = HTCFG_MCTM_DEAD_TIME;
  MCTM_CHBRKCTRInitStructure.BreakFilter = 0;
  MCTM_CHBRKCTRConfig(HT_MCTM0, &MCTM_CHBRKCTRInitStructure);

  /* MCTM counter enable                                                                                    */
  TM_Cmd(HT_MCTM0, ENABLE);

  /* MCTM Channel Main Output enable                                                                        */
  MCTM_CHMOECmd(HT_MCTM0, ENABLE);
	
	
	
}

int main(void)
{
	int i;
//	u32 Data1 = 0, Data2 = 0,Data3 = 0,Data4 = 0;
//	u32 Data_10 = 1;
	
	RETARGET_Configuration();
	CKCU_configuration();
	GPIO_configuration();
	EXTI_configuration();
  NVIC_configuration();
////	ADC_configuration();
	MCTM_configuration();
  	
	BFTM_Configuration();
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_0, RESET);
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_3, RESET);
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_5, RESET);
	GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 0);
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_7, SET);
	delay_s(2);
//	ring=2;
//	up=0, down=1;
//	tem=0;
//	wet=2;
	while (1){
		  WET();	
		  DIS1();
			printf("\r\nDis1:%3.2f",Distance1); 	
			DIS2();
			printf("\r\nDis2:%3.2f",Distance2);
			DIS3();
			printf("\r\nDis3:%3.2f",Distance3);
			printf("\r\nup: %d",up);
			printf("\r\ndown: %d",down);
			printf("\r\nwet: %d",wet);
			printf("\r\ntem: %d",tem);		
			printf("\r\nring: %d",ring);
			printf("\r\ninitial: %d",initial);	
			printf("\r\n");	
		 
//		  
//			
////		Data1 = ADC_GetConversionData(HT_ADC,0);//wet
////		Data2 = ADC_GetConversionData(HT_ADC,1);//dis1 up
////		Data3 = ADC_GetConversionData(HT_ADC,2);//dis2 down
////		Data4 = ADC_GetConversionData(HT_ADC,3);//dis3 door

		
		
		if(Distance2<15)
			up=1;
		else
			up=0;
	
		if(Distance3<15) 
			down=1;
		else
			down=0;
		

		
		
		if(up==0&&down==0&&initial!=0){	//no people
			if((wet==1||wet==2) && Distance1<6){	
				ring=1;
				GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 0);	
			}else if((wet==1||wet==2) && tem==0){ 
				ring=0;
				GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 1);//on
			}else if(wet==1 && tem==1){
				ring=4;
				GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 1);//on	
			}else if(tem==1){
				ring=4;
				GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 0);
			}else{
				ring=0;
				GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 0);
			}
		}else{
			//ring can't set
			GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 0);
			if(ring==4){
				ring=0;
			}else if(ring==2){
				ring=3;
			}
			if(up==0&&down==1&&initial==1){ //fall down
				ring=2;
				initial=0;
			}
		}
		
		
		
		switch(ring){ 
			case 0:
				//nothing
				LED(0,0,0);
				break;
			case 1:
				//ring wet
				LED(50,230,0);
				RING1(freq1);
				LED(0,0,0);
				ring=0;
				break;
			case 2:
				//ring=3;
				//ring H
			  LED(255,220,0);
				RING2(freq2);
			  RING2(freq2);
				RING2(freq2);
				RING2(freq2);
				RING2(freq2);
					delay(50000);
				break;
			case 3:
				//ring L
				LED(255,0,0);
			  RING3(freq3);
				RING3(freq3);
				RING3(freq3);
				delay(50000);
				break;
			default:
				LED(0,255,230);
				break;
		}
		delay_s(1);
	}
}
void WET(void){
	start_read_DHT(&temperature,&humidity);
	Humidity=humidity/100;
	Temperature=temperature/100;
	printf("\r\nH: %d%%",Humidity);
	printf("\r\nT: %dC",Temperature);
	if(Humidity>240)
		wet=1;//very wet
	else if(Humidity>230)
		wet=2;//little wet
	else 
		wet=0;

	if(Temperature<68)
		tem=1;
	else
		tem=0;
}


void DIS1(void){
	
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_0, SET);
	BFTM_ReStart();
	while(BFTM_us()<10);
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_0, RESET);
	while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_1) == RESET);
	BFTM_ReStart();
	while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_1) == SET);
	BFTM_end();
	Distance1 = BFTM_us() / 58.0;
}
void DIS2(void){
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_3, SET);
	BFTM_ReStart();
	while(BFTM_us()<10);
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_3, RESET);
	while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_2) == RESET);
	BFTM_ReStart();
	while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_2) == SET);
	BFTM_end();
	Distance2 = BFTM_us() / 58.0;
}
void DIS3(void){
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_15, SET);
	BFTM_ReStart();
	while(BFTM_us()<10);
	GPIO_WriteOutBits(HT_GPIOA, GPIO_PIN_15, RESET);
	while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_14) == RESET);
	BFTM_ReStart();
	while(GPIO_ReadInBit(HT_GPIOA, GPIO_PIN_14) == SET);
	BFTM_end();
	Distance3 = BFTM_us() / 58.0;
}

void delay(float x){
  int i;
  for(i=0; i < x; ++i);//10000000
}

void RING1(float *freq){
	int i=0,wBeeIndex = 0;
	for (wBeeIndex = 0; wBeeIndex < BEE_NBR; wBeeIndex++){
		PWM_Configuration(freq[wBeeIndex]);
    TM_ChannelConfig(BUZZER_TM, BUZZER_TM_CHANNEL, TM_CHCTL_ENABLE);
    wMctmUpdateDownCounter = (freq[wBeeIndex]*5)/100;
    while (wMctmUpdateDownCounter);
		
		delay(100000);
		
    TM_ChannelConfig(BUZZER_TM, BUZZER_TM_CHANNEL, TM_CHCTL_DISABLE);
    wMctmUpdateDownCounter = (freq[wBeeIndex]*5)/100;
    while (wMctmUpdateDownCounter);
		delay(70000);
  }
}

void RING2(float *freq){
	int i=0,wBeeIndex = 0;
	for (wBeeIndex = 0; wBeeIndex < BEE_NBR; wBeeIndex++){
		PWM_Configuration(freq[wBeeIndex]);
    TM_ChannelConfig(BUZZER_TM, BUZZER_TM_CHANNEL, TM_CHCTL_ENABLE);
    wMctmUpdateDownCounter = (freq[wBeeIndex]*5)/100;
    while (wMctmUpdateDownCounter);
		
		delay(100000);
		
    TM_ChannelConfig(BUZZER_TM, BUZZER_TM_CHANNEL, TM_CHCTL_DISABLE);
    wMctmUpdateDownCounter = (freq[wBeeIndex]*5)/100;
    while (wMctmUpdateDownCounter);
		delay(50000);
  }
}

void RING3(float *freq){
	int i=0,wBeeIndex = 0;
	for (wBeeIndex = 0; wBeeIndex < BEE_NBR; wBeeIndex++){
		PWM_Configuration(freq[wBeeIndex]);
    TM_ChannelConfig(BUZZER_TM, BUZZER_TM_CHANNEL, TM_CHCTL_ENABLE);
    wMctmUpdateDownCounter = (freq[wBeeIndex]*5)/100;
    while (wMctmUpdateDownCounter);
		
		delay(50000);
		
    TM_ChannelConfig(BUZZER_TM, BUZZER_TM_CHANNEL, TM_CHCTL_DISABLE);
    wMctmUpdateDownCounter = (freq[wBeeIndex]*5)/100;
    while (wMctmUpdateDownCounter);
		delay(10000);
  }
}
void LED(int R, int G, int B){ //+time not yet
	MCTM_OutputInitStructure.Channel = TM_CH_0;
  MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * R / 255;
  TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);
	MCTM_OutputInitStructure.Channel = TM_CH_2;
  MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * G / 255;
  TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);
	MCTM_OutputInitStructure.Channel = TM_CH_3;
  MCTM_OutputInitStructure.Compare = HTCFG_MCTM_RELOAD * B / 255;
  TM_OutputInit(HT_MCTM0, &MCTM_OutputInitStructure);
}
	

