#include "bsp_ds18b20.h"
#include <rtthread.h>
#include <rthw.h>
#include "stm32f10x.h"
#include "myconfig.h"

void tim4_up_preocess(void);
static unsigned char Crc8(unsigned char* ptr, unsigned char len)   
{  
  unsigned char i;  
  unsigned char crc=0;  
  while(len--!=0)   
  {  
    for(i=1; i!=0; i*=2)  
    {  
      if((crc&1)!=0) 
      {
        crc>>=1; 
        crc^=0x8C;
      } 
      else 
        crc>>=1;  
      if((*ptr&i)!=0) 
        crc^=0x8C;      
    }   
    ptr++;  
  }  
  return(crc);  
} 


#define GPIOC_4LOW_RESET 0X000F0000
#define GPIOC_4LOW_SET   0X0000000F
#define STATE_18B20_RESET 0
#define STATE_18B20_SKIPROM 1
#define STATE_18B20_CONVERT 2
#define STATE_18B20_RESET2 3
#define STATE_18B20_READ_COMMAND 4
#define STATE_18B20_READ 5
#define STATE_18B20_WAIT 6
#define STATE_18B20_WAIT2 7  //һ�ζ�ȡ��ɺ�ȴ�ʱ��
#define STATE_18B20_SKIPROM2 8
#define STATE_18B20_RESET3 9
unsigned int Gpio_data[73];
int dam_send_buf=0X0000000a; //ֻʹ�õ�4λ
//#define dam_send_buf (GPIOC->BSRR) 

int state_18b20=0;
int process_time=0;
int read_count=0;
//18B20 RESET һ��ռ��500us
//500us�͵�ƽ��ʼ�ź�
//����Ӳ����������û��������,���õ�stm32�ĵ�Ƭ������
//��ȡ��ʱ����Ҫ�л�����������ģʽ
int dma_set_18b20_reset(int cur_time)
{
//	int ret=0;
	if(cur_time>5 && cur_time<55)  //500us�͵�ƽ
		dam_send_buf=GPIOC_4LOW_RESET;
	else  //500us����������,�������ź�
	{
		dam_send_buf=GPIOC_4LOW_SET;
		if(cur_time==56 && state_18b20==STATE_18B20_RESET)
		{
	//		dam_send_buf=GPIOC_4LOW_SET;
			DS18B201_Configuration_in2();
		}
		if(cur_time==61) //���Ͷ�ȡоƬ����ָ��
		{
			cur_time=cur_time;
			Gpio_data[0]=GPIOC->IDR&0xF;  //��ȡ4��io��״̬
//			if(state_18b20==STATE_18B20_RESET)
//			{
//				DMA_Cmd(DMA1_Channel6,DISABLE);
//				DMA_SetCurrDataCounter(DMA1_Channel6,1);
//				DMA_Cmd(DMA1_Channel6,ENABLE);
//			}
			
		}
	}
	return 0;
}
//��ʱ��
int dma_set_18b20_reset3(int cur_time)
{
		dam_send_buf=GPIOC_4LOW_SET;
		return 0;
}

//18B20 ��ROM һ��ռ��400us  0b11001100
//һ��λ10us�͵�ƽ��ʼ�ź�,40us����,10us���
//0XXXX1  ����һ��λ������.x�ɷ��͵����ݾ���
int dma_set_18b20_skiprom(int cur_time)
{
	int ret=cur_time%6;
	if(ret<1)
		dam_send_buf=GPIOC_4LOW_RESET;
	else if(ret>4)
		dam_send_buf=GPIOC_4LOW_SET;
	else 
	{

		if(cur_time<12 || (cur_time>24&& cur_time<36))
			dam_send_buf=GPIOC_4LOW_RESET;
		else
			dam_send_buf=GPIOC_4LOW_SET;
	}
	
	return 0;
}

//18B20 ����ת��һ��ռ��500us  0b0100 0100
//һ��λ10us�͵�ƽ��ʼ�ź�40us����10us���
//0XXXX1  ����һ��λ������.x�ɷ��͵����ݾ���
int dma_set_18b20_Convert(int cur_time)
{
	int ret=cur_time%6;
	if(ret<1)
		dam_send_buf=GPIOC_4LOW_RESET;
	else if(ret>4)
		dam_send_buf=GPIOC_4LOW_SET;
	else 
	{

		if((cur_time>12&&cur_time<18) || (cur_time>36&& cur_time<42))
			dam_send_buf=GPIOC_4LOW_SET;
		else
			dam_send_buf=GPIOC_4LOW_RESET;
	}
	
	return 0;
}

//18B20 ���Ͷ�romָ��һ��ռ��500us  0b1011 1110
//һ��λ10us�͵�ƽ��ʼ�ź�,40us����,10us���
//0XXXX1  ����һ��λ������.x�ɷ��͵����ݾ���
int dma_set_18b20_read_command(int cur_time)
{
	int ret=cur_time%6;
	if(ret<1)
		dam_send_buf=GPIOC_4LOW_RESET;
	else if(ret>4)
		dam_send_buf=GPIOC_4LOW_SET;
	else 
	{

		if((cur_time<6) || (cur_time>36 &&cur_time<=42))
			dam_send_buf=GPIOC_4LOW_RESET;
		else
			dam_send_buf=GPIOC_4LOW_SET;
	}
	return 0;
}

//18B20 ��9�ֽ����� 10us�͵�ƽ(��ʼ��)30us�ڶ�����
//ÿ��λռ��(40)  һ����Ҫ40*8*9=2160us.
//0111   �ڶ����ߵ�ƽ������
int dma_set_18b20_read(int cur_time)
{
	int ret=cur_time%4;
	if(ret==0)
		dam_send_buf=GPIOC_4LOW_RESET;
	else 
	{
		dam_send_buf=GPIOC_4LOW_SET;
		if(ret==2)
		{//���Ͷ�ȡ����ָ��
//			DMA_Cmd(DMA1_Channel6,DISABLE);
//			DMA_SetCurrDataCounter(DMA1_Channel6,4);
//			DMA_Cmd(DMA1_Channel6,ENABLE);
			//����18b20���ݶ˿�Ϊ����
			DS18B201_Configuration_in2();
			Gpio_data[read_count]=GPIOC->IDR&0xF;
			GPIO_18B20_init();
			read_count++;
		}
	}
	return 0;
}
char tim4_init(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	
	TIM_TimeBaseStructure.TIM_Prescaler = 72-1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 10-1;
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	//Ԥװ��ʹ��
	TIM_ARRPreloadConfig(TIM4, ENABLE);

#if 1
	//====================================�жϳ�ʼ��===========================
	//����NVIC���ȼ�����ΪGroup2��0-3��ռʽ���ȼ���0-3����Ӧʽ���ȼ�
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//�������жϱ�־λ
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	//��ʱ��3����жϹر�
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	//��ʱ��3����
	TIM_Cmd(TIM4, ENABLE);
#endif
	
	return TRUE;
}
void TIM4_IRQHandler(void)
{
	rt_interrupt_enter();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);		 //���жϱ��
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);  //�����ʱ��TIM2����жϱ�־λ
		tim4_up_preocess();
	}
	rt_interrupt_leave();
}
void gpio_dma_init_write()
{
	DMA_InitTypeDef 		 DMA_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
	DMA_DeInit(DMA1_Channel7); 
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(GPIOC->BSRR));	
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)(&dam_send_buf);
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;	
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Enable;  
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	DMA1_Channel7->CCR |= DMA_CCR7_EN;
}
int test_count=0;
int read_error_count[4]={0};
int check_error_count[4]={0};

void temp_analasis(void)
{
	int i=0;
//	int temp_state;
	int j=0;
	int temp=0;
	unsigned char temp_receive_byte[9]={0};
	for(i=0;i<4;i++)
	{
		if(Gpio_data[0]&(1<<i))
		{
			check_error_count[i]++;
			if(check_error_count[i]==100)
			{
				ro_data_yuanshi[CURTMP_YUANSHI+i]=0;
				ro_data[CURTMP+i]=200;
				warning.data|=(1<<i);
			}
			if(check_error_count[i]>100)
				check_error_count[i]=101;
			continue;
		}
		else //18b20����
		{
			warning.data&=~(1<<i);
			check_error_count[i]=0;
			for(j=71;j>=0;j--)
			{
				temp_receive_byte[j/8]|=((Gpio_data[j+1]&(1<<i))>>i);
				if(j&7)
					temp_receive_byte[j/8]=temp_receive_byte[j/8]<<1;
			}
		}
		if(Crc8(temp_receive_byte, 8)==(unsigned char)temp_receive_byte[8]) //crcУ��
		{
			temp=(short)((temp_receive_byte[1]<<8)+temp_receive_byte[0]);
			if(SYH_abs(temp-getro(CURTMP_YUANSHI+i))<100)
			{
				ro_data_yuanshi[CURTMP_YUANSHI+i]=temp;
				ro_data[CURTMP+i]=(float)temp*625/10000;
				read_error_count[i]=0;
			}
			else
			{
				read_error_count[i]++;
				if(read_error_count[i]>10)
				{
					read_error_count[i]=0;
					ro_data_yuanshi[CURTMP_YUANSHI+i]=temp;
					ro_data[CURTMP+i]=(float)temp*625/10000;
				}
			}
				
		}
		for(j=0;j<9;j++)
			temp_receive_byte[j]=0;
	}
}
void tim4_up_preocess()
{
	process_time++;
	switch(state_18b20)
	{
		case STATE_18B20_RESET:
			dma_set_18b20_reset(process_time);
			if(process_time>70)
			{	
				GPIO_18B20_init();
				process_time=0;
				dam_send_buf=GPIOC_4LOW_RESET;
				state_18b20=STATE_18B20_SKIPROM;
			}
			break;
		case STATE_18B20_RESET3:
			dma_set_18b20_reset3(process_time);
			if(process_time>46)
			{
				process_time=0;
				dam_send_buf=GPIOC_4LOW_RESET;
				state_18b20=STATE_18B20_RESET;
			}
			break;
		case STATE_18B20_SKIPROM:
			dma_set_18b20_skiprom(process_time);
			if(process_time>=48-1)
			{
				process_time=0;
				dam_send_buf=GPIOC_4LOW_RESET;
				state_18b20=STATE_18B20_CONVERT;
			}
			break;
			case STATE_18B20_CONVERT:
				dma_set_18b20_Convert(process_time);
				if(process_time>=48-1)
				{
					process_time=0;
					dam_send_buf=GPIOC_4LOW_SET;
					state_18b20=STATE_18B20_WAIT;
				}
				break;
			case STATE_18B20_WAIT:
				if(process_time>70)
				{
					process_time=0;
					dam_send_buf=GPIOC_4LOW_SET;
					state_18b20=STATE_18B20_RESET2;
				}
				break;
			case STATE_18B20_WAIT2:
				if(process_time>1000)
				{
					process_time=0;
					dam_send_buf=GPIOC_4LOW_SET;
					state_18b20=STATE_18B20_RESET;
				}
				break;
			case STATE_18B20_RESET2:
				dma_set_18b20_reset(process_time);
				if(process_time>70)
				{
					process_time=0;
					dam_send_buf=GPIOC_4LOW_RESET;
					state_18b20=STATE_18B20_SKIPROM2;
				}
				break;
			case STATE_18B20_SKIPROM2:
				dma_set_18b20_skiprom(process_time);
				if(process_time>46)
				{
					process_time=0;
					dam_send_buf=GPIOC_4LOW_RESET;
					state_18b20=STATE_18B20_READ_COMMAND;
				}
				break;
				case STATE_18B20_READ_COMMAND:
					dma_set_18b20_read_command(process_time);
					if(process_time>46)
					{
						process_time=0;
						dam_send_buf=GPIOC_4LOW_RESET;
						state_18b20=STATE_18B20_READ;
						read_count=1;
					}
					break;
			case STATE_18B20_READ:
				dma_set_18b20_read(process_time);
				if(process_time>288-1)
				{
					process_time=0;
					dam_send_buf=GPIOC_4LOW_SET;
					state_18b20=STATE_18B20_WAIT2;
				}
				break;
	}

}

//����18b20���ݶ˿�Ϊ����
void DS18B201_Configuration_in2() 
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; //2Mʱ���ٶ� 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
} 

void GPIO_18B20_init()
{
	GPIO_InitTypeDef GPIO_InitStructure; 

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_8|GPIO_Pin_1|GPIO_Pin_2
		|GPIO_Pin_3|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; //2Mʱ���ٶ� 
	GPIO_Init(GPIOC, &GPIO_InitStructure); 
	GPIO_SetBits(GPIOC,GPIO_Pin_8) ;
	GPIO_SetBits(GPIOC,GPIO_Pin_9) ;

}
static void dma_18b20_process(void* parameter)
{
	#define SCAN_TIME 100  //50msһ��
//	int temp=0;
	GPIO_18B20_init();
	tim4_init();
	gpio_dma_init_write();
	Gpio_data[0]=0xf;
	while(1)
	{
		temp_analasis();
		rt_thread_delay(SCAN_TIME);
	}



}


void dma_18b20_init()
{
	rt_thread_t thread;

    thread = rt_thread_create(
                   "dma1",
                   dma_18b20_process,
                   RT_NULL,
                   		512,
                   THREAD_CAL_18B20_PRIORITY,15);
    if (thread != RT_NULL)
        rt_thread_startup(thread);
}

