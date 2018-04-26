
#include "stm32f10x_conf.h"
#include "pwm.h"

vu32 freq=1000000;
vu32 freq_arr=72000000/100000-1;

#define FREQQQ_ARR_MAX 72000000/FREQQQ*0.4
#define FREQQQ_ARR_MIN 72000000/FREQQQ*0.1

void TIM1_Mode_Config(void) 
{ 
			freq=1000000;
			freq_arr=72000000/freq-1;
      GPIO_InitTypeDef GPIO_InitStructure; 
      TIM_TimeBaseInitTypeDef TIM_BaseInitStructure; 
      TIM_OCInitTypeDef TIM_OCInitStructure; 

   //  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE); 

         RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_TIM1|RCC_APB2Periph_GPIOB, ENABLE); 

			 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; 
			 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
			 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			 GPIO_Init(GPIOB, &GPIO_InitStructure); 
			 
			 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 ; 
			 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
			 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
			 GPIO_Init(GPIOA, &GPIO_InitStructure); 

    //TIM1???????(??PWM??) 
    //??=TIM1_CLK/(ARR+1) 
    TIM_BaseInitStructure.TIM_Period = FREQQQ_ARR;//576-1;//   //125kHZ  (72M/125k)-1=575 
    TIM_BaseInitStructure.TIM_Prescaler = 0; 
    TIM_BaseInitStructure.TIM_ClockDivision = 0; 
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; 
    TIM_BaseInitStructure.TIM_RepetitionCounter = 0; 
    TIM_TimeBaseInit(TIM1, &TIM_BaseInitStructure); 
    //??ARR??????(?????????????) 
    TIM_ARRPreloadConfig(TIM1, ENABLE); 
		TIM1->BDTR=0x19;
		
    //TIM1_OC1????(??1?????) 
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable; 
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
    TIM_OCInitStructure.TIM_OCNPolarity =TIM_OCNPolarity_High; //TIM_OCNPolarity_Low; 
		TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;         //???????1 
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;   //?????????0  
    TIM_OCInitStructure.TIM_Pulse = 192;///288;  // 50%???= 576/2 
    TIM_OC1Init(TIM1, &TIM_OCInitStructure); 

    //??CCR1?????????(?????????????) 
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);    
    //TIM1?? 
    TIM_Cmd(TIM1, ENABLE); 
    //TIM1_OC????PWM(????) 
    TIM_CtrlPWMOutputs(TIM1, ENABLE); 

} 


void TIM3_PWMShiftInit(void)  
{  
		freq=100000;
		freq_arr=72000000/freq-1;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseInitStruct;  
    GPIO_InitTypeDef  GPIO_InitStruct;  
    TIM_OCInitTypeDef TIM_OCInitStruct;  

    /**********************TIM3 GPIO����*****************************/  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOA, ENABLE);  
      
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;  
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_13;  
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;  
  
    GPIO_Init(GPIOB,&GPIO_InitStruct);  
	
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8; 
    GPIO_Init(GPIOA,&GPIO_InitStruct);  
      
    /**********************��ʼ��TimBase�ṹ��*************************/  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE); //����TIMʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);	
      
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;   //��ʱ������Ƶ
    TIM_TimeBaseInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;  
    TIM_TimeBaseInitStruct.TIM_Period    = freq_arr;       //Ƶ�� = 72000000/PSC/(ARR+1) = 40KHz 
    TIM_TimeBaseInitStruct.TIM_Prescaler = 0;       
      
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);  
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStruct);  
    /**********************��ʼ��TIM3 OC�ṹ��*************************/  
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM2;  
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_Low;  
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;  
    TIM_OCInitStruct.TIM_Pulse = (TIM_TimeBaseInitStruct.TIM_Period+1)/2;                 //TIM3 CH3ռ�ձȣ�600/��ARR+1��=33.3%
		
    TIM_OC3Init(TIM3,&TIM_OCInitStruct);  
		TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3,&TIM_OCInitStruct);  


    //TIM1 CH1pwm�������                  
    TIM_OCInitStruct.TIM_Pulse=(TIM_TimeBaseInitStruct.TIM_Period+1)/2;                   //TIM1 CH1ռ�ձȣ�600/��ARR+1��=33.3%
   
    TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;        //�����������              
    TIM_OCInitStruct.TIM_OCNPolarity=TIM_OCNPolarity_High; 
		TIM_OCInitStruct.TIM_OutputNState=TIM_OutputNState_Enable;//��ֹ���������,ȱʧ������޷������������                         
    TIM_OC1Init(TIM1,&TIM_OCInitStruct);
		TIM1->CCR1=(TIM_TimeBaseInitStruct.TIM_Period+1)/2;

		TIM3->CCR4=(TIM_TimeBaseInitStruct.TIM_Period+1)/2+10;
		TIM3->CCR1=(TIM_TimeBaseInitStruct.TIM_Period+1)/2-10;
		TIM1->CCR2=(TIM_TimeBaseInitStruct.TIM_Period+1)/2*0.1;//��λƫ��
		TIM1->BDTR=20;
    /**************************�������ഥ��**************************/
  //  TIM_OCInitStruct.TIM_Pulse = (TIM_TimeBaseInitStruct.TIM_Period+1)/2*0.2;             //Լ��2����ʱ�����ڴ��������ࣺ360*600/(ARR+2)= 120��
    TIM_OC2Init(TIM1,&TIM_OCInitStruct);            //��ҪCH2�����ش�������޷��������������CH2�������
  

    /**************************��������ģʽ*************************/
    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC2Ref);  //TIM1 OC2�����Ӷ�ʱ��
    TIM_SelectMasterSlaveMode(TIM1, TIM_MasterSlaveMode_Enable); 
 
    TIM_SelectInputTrigger(TIM3, TIM_TS_ITR0);      //ITRO����
    TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Trigger);   
		TIM3->CCR4=(TIM_TimeBaseInitStruct.TIM_Period+1)/2+10;
		TIM3->CCR1=(TIM_TimeBaseInitStruct.TIM_Period+1)/2-10;
    TIM_Cmd(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);               //�߼���ʱ��pwm���ʹ�ܣ�һ��Ҫ�ǵô�
}  
