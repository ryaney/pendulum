/*
 * File						: pwmRotor4.c
 * Description		: This file is ...
 * Author					: lynx@sia  84693469@qq.com
 * Copyright			:
 *
 * History
 **--------------------
 * Rev						: 0.00
 * Date						: 10/19/2013
 *
 * create.
 *
 * Rev						: 0.00
 * Date						: 11/11/2013
 *
 * Exchanged the motor 3 and 4.
 **--------------------
 */

//----------------- Include files ------------------------//
#include "stm32f10x_gpio.h"
#include "pwmRotor4.h"
//----------------- Define -------------------------------//

//----------------- Function Prototype -------------------//

static int initialize(void);
static int frequency(int freq, float percentage);
static int setFreq(int freq);
static int setMotor1F(float percentage);
static int setMotor2F(float percentage);
static int setMotor3F(float percentage);
static int setMotor4F(float percentage);
static int setMotor1I(int percentage);
static int setMotor2I(int percentage);
static int setMotor3I(int percentage);
static int setMotor4I(int percentage);
//----------------- Variable -----------------------------//

static int PWM_Motor_Min = 0;
static int PWM_Motor_Max = 1050-1;   // Ĭ������ = 1ms, 21MHz��Ƶ��20kHz�����ﶨ����ͷ�ļ���PWM_Motor_Max�� ��һ�Ǿ�׼��ʱ����Ҫ ע�� �����ǳ�ʼֵ������θ�����Ҫ��������
//��ʹ�ø�Ƶ������£���׼Ƶ��21M�� �����õ����Ƶ�ʵ���340Hz

PWMROTOR4_T pwmRotor4 = {
    .initialize = initialize,
    .frequency = frequency,
    .setFreq = setFreq,
    .setMotor1F = setMotor1F,
    .setMotor2F = setMotor2F,
    .setMotor3F = setMotor3F,
    .setMotor4F = setMotor4F,
    .setMotor1I = setMotor1I,
    .setMotor2I = setMotor2I,
    .setMotor3I = setMotor3I,
    .setMotor4I = setMotor4I
};
//----------------- Function -----------------------------//

/*
 * Name						: initialize
 * Description		: This file is ...
 * Author					: lynx@sia  84693469@qq.com
 * Copyright			:
 *
 * History
 **--------------------
 * Rev						: 0.00
 * Date						: 10/19/2013
 *
 * create.
 **--------------------
 */
static int
initialize(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* TIM2 PWM1 PA0 */	/* TIM2 PWM2 PA1 */	/* TIM2 PWM3 PA2 */	/* TIM2 PWM4 PA3 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;		//����ΪPWM���
  	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	TIM_DeInit(TIM2);

/************************** PWM Output **************************************/
	/* �O�� TIM2 TIM3 TIM4 Time Base */
	TIM_TimeBaseStruct.TIM_Period = (u16)(PWM_Motor_Max);     // �L�� = 1ms, 20kHz�����ﶨ����ͷ�ļ���PWM_Motor_Max�� ������һ
	TIM_TimeBaseStruct.TIM_Prescaler = (u16)(4-1);             // ���l4 = 21MHz
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;		// �ϔ�
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

	/* �O�� TIM2 TIM3 TIM4 TIM8 OC */
	TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;							// ���Þ� PWM1 ģʽ
	TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;	// ���� OC
	TIM_OCInitStruct.TIM_Pulse = PWM_Motor_Min;									// �O����׃ֵ
	TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;			// ��Ӌ��ֵС� PWM_Motor_Min �r����ƽ
	TIM_OC1Init(TIM2, &TIM_OCInitStruct);												// ��ʼ�� TIM2 OC1
	TIM_OC2Init(TIM2, &TIM_OCInitStruct);												// ��ʼ�� TIM2 OC2
	TIM_OC3Init(TIM2, &TIM_OCInitStruct);												// ��ʼ�� TIM2 OC3
	TIM_OC4Init(TIM2, &TIM_OCInitStruct);												// ��ʼ�� TIM2 OC4
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);						// ���� TIM2 OC1 �A�b�d
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);						// ���� TIM2 OC2 �A�b�d
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);						// ���� TIM2 OC3 �A�b�d
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);						// ���� TIM2 OC4 �A�b�d
	
	//�õ��������
	PWM1 = PWM_Motor_Min;
	PWM2 = PWM_Motor_Min;
	PWM3 = PWM_Motor_Min;
	PWM4 = PWM_Motor_Min;

	/* ���� */
	TIM_ARRPreloadConfig(TIM2, ENABLE);													// ���� TIM2 ���d�Ĵ���ARR
	TIM_Cmd(TIM2, ENABLE);																			// ���� TIM2

	return 0;
}
/*
 * Name						: frequency
 * Description		: This file is ...
 * Author					: lynx@sia  84693469@qq.com
 * Copyright			:
 *
 * History 2012/11/3�ĳ�����float����ʽ�������û�����
 **--------------------
 * Rev						: 0.00
 * Date						: 10/19/2013
 *
 * create.
 **--------------------
 */
static int
frequency(int freq, float percentage)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	float Fpercentage;
	
	if (freq == 0 || percentage <= 0) {       //�����ǵ��Թص��õ�
		//�õ��������
		PWM1 = PWM_Motor_Min;
		PWM2 = PWM_Motor_Min;
		PWM3 = PWM_Motor_Min;
		PWM4 = PWM_Motor_Min;
		return 0;
	} else {	
		//�淶�������ֵ��Χ
		if(percentage > 100)
			percentage = 100;
	
		Fpercentage = percentage/100.0f;          //ת��ΪС��[0,1)
		
		PWM_Motor_Max = (21000000/freq-1); //�����Զ���װ�ص�ֵ
		/* �O�� TIM2 TIM3 TIM4 Time Base */
		TIM_TimeBaseStruct.TIM_Period = (u16)(PWM_Motor_Max);     // ������������
		TIM_TimeBaseStruct.TIM_Prescaler = (u16)(4-1);             // ���l4 = 21M
		TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;		// �ϔ�
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
		
		Fpercentage *= PWM_Motor_Max;
		
		PWM1 = Fpercentage;
		PWM2 = Fpercentage;
		PWM3 = Fpercentage;
		PWM4 = Fpercentage;
		
		/* ���� */
		TIM_ARRPreloadConfig(TIM2, ENABLE);													// ���� TIM2 ���d�Ĵ���ARR
		TIM_Cmd(TIM2, ENABLE);																			// ���� TIM2
		
	}

	return 1;
}

static int setFreq(int freq)
{	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	if (freq < 1) {       //�����ǵ��Թص��õ�
		//�õ��������
		PWM1 = PWM_Motor_Min;
		PWM2 = PWM_Motor_Min;
		PWM3 = PWM_Motor_Min;
		PWM4 = PWM_Motor_Min;
		return 0;
	} else {			
		PWM_Motor_Max = (21000000/freq-1); //�����Զ���װ�ص�ֵ
		/* �O�� TIM2 TIM3 TIM4 Time Base */
		TIM_TimeBaseStruct.TIM_Period = (u16)(PWM_Motor_Max);     // ������������
		TIM_TimeBaseStruct.TIM_Prescaler = (u16)(4-1);             // ���l4 = 21M 
		TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;		// �ϔ�
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);

		/* ���� */
		TIM_ARRPreloadConfig(TIM2, ENABLE);													// ���� TIM2 ���d�Ĵ���ARR
		TIM_Cmd(TIM2, ENABLE);																			// ���� TIM2
		
	}

	return 1;
}

static int setMotor1F(float percentage)
{
	float Fpercentage;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	Fpercentage = percentage/100.0f;
	Fpercentage *= PWM_Motor_Max;
		
	PWM1 = Fpercentage;  //�Ƚ�ֵ������İٷ������

	return 1;
}
static int setMotor2F(float percentage)
{
	float Fpercentage;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	Fpercentage = percentage/100.0f;
	Fpercentage *= PWM_Motor_Max;
		
	PWM2 = Fpercentage;  //�Ƚ�ֵ������İٷ������

	return 1;
}
static int setMotor3F(float percentage)
{
	float Fpercentage;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	Fpercentage = percentage/100.0f;
	Fpercentage *= PWM_Motor_Max;
		
	PWM3 = Fpercentage;  //�Ƚ�ֵ������İٷ������

	return 1;
}
static int setMotor4F(float percentage)
{
	float Fpercentage;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	Fpercentage = percentage/100.0f;
	Fpercentage *= PWM_Motor_Max;
		
	PWM4 = Fpercentage;  //�Ƚ�ֵ������İٷ������

	return 1;
}

static int setMotor1I(int percentage)
{
	int ITemp;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;
	
	ITemp = (PWM_Motor_Max*percentage)/100;
		
	PWM1 = ITemp;  //�Ƚ�ֵ������İٷ������

	return 1;
}
static int setMotor2I(int percentage)
{
	int ITemp;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	ITemp = (PWM_Motor_Max*percentage)/100;
		
	PWM2 = ITemp;  //�Ƚ�ֵ������İٷ������

	return 1;
}
static int setMotor3I(int percentage)
{
	int ITemp;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	ITemp = (PWM_Motor_Max*percentage)/100;
		
	PWM3 = ITemp;  //�Ƚ�ֵ������İٷ������

	return 1;
}
static int setMotor4I(int percentage)
{
	int ITemp;
	
	//�淶�������ֵ��Χ ʹ��ǰ���ȳ�ʼ���������趨һ��Ƶ��
	if(percentage < 0)
		percentage = 0;
	else if(percentage > 100)
		percentage = 100;

	ITemp = (PWM_Motor_Max*percentage)/100;
		
	PWM4 = ITemp;  //�Ƚ�ֵ������İٷ������
	
	return 1;
}
