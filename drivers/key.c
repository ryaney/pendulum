#include "key.h"

void Key_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /*开启按键端口（PA）的时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8); 
    //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 
    GPIO_Init(GPIOC, &GPIO_InitStructure);
}


uint8_t Key_Scan(GPIO_TypeDef* GPIOx,u16 GPIO_Pin)
{
    /*检测是否有按键按下 */
    if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) ==0 ) 
    {
        /*延时消抖*/
        // delay_ms(10);		
        if(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) ==0)  
        {
            /*等待按键释放 */
            while(GPIO_ReadInputDataBit(GPIOx,GPIO_Pin) == 0);   
            return  0;
        }
        return 1;
    }
    return 1;
}
