/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * uart_syslink.c - Uart syslink to nRF51 and raw access functions
 */
#include <stdint.h>
#include <string.h>

/*ST includes */
#include "stm32fxxx.h"

/*FreeRtos includes*/
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

#include "config.h"
#include "uart_syslink.h"
#include "crtp.h"
#include "cfassert.h"
#include "nvicconf.h"
#include "config.h"
#include "queuemonitor.h"
#include "debug.h"

#define UARTSLK_DATA_TIMEOUT_MS 1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((uint32_t)0x00000001)

static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone;
static xSemaphoreHandle waitUntilReceiveDone;
static xSemaphoreHandle uartTxBusy;
static xSemaphoreHandle uartRxBusy;
static xQueueHandle uartslkDataDelivery;

static uint8_t dmaBuffer[64];
static uint8_t *outDataIsr;
static uint8_t dataIndexIsr;
static uint8_t dataSizeIsr;
static bool    isUartDmaInitialized;
static uint32_t initialDMACount;
static uint32_t remainingDMACount;
static bool     dmaIsPaused;

#define RX_BUFFER_SIZE 256
static uint8_t RxBuffer[RX_BUFFER_SIZE];
static volatile uint32_t rxBufferContentsSize = 0;
static volatile uint32_t rxBufferContentsSizeAfterDisable = 0;

// for debug
volatile uint32_t dataRemainingAfterTransfer;

static void uartslkPauseDma();
static void uartslkResumeDma();

static bool isValidSyslinkPacket()
{
  uint8_t size;
  uint8_t chksum[2];

  // Must be at least 6 for start, type, len, and chksum.
  if(rxBufferContentsSize < 6)
  {
    return false;
  }

  // start byte 1
  if(RxBuffer[0] != 0xBC)
  {
    return false;
  }

  // start byte 2
  if(RxBuffer[1] != 0xCF)
  {
    return false;
  }

  chksum[0] = RxBuffer[2];
  chksum[1] = RxBuffer[2];

  size = RxBuffer[3];

  chksum[0] += RxBuffer[3];
  chksum[1] += chksum[0];

  // does the rxBufferContentSize match what the size param says it should?
  if((size + 6) < rxBufferContentsSize)
  {
    return false;
  }

  for(int8_t i = 0; i < size; i++)
  {
    chksum[0] += RxBuffer[4 + i];
    chksum[1] += chksum[0];
  }

  if(chksum[0] != RxBuffer[4 + size])
  {
    return false;
  }

  if(chksum[1] != RxBuffer[4 + size + 1])
  {
    return false;
  }

  return true;
}

/**
  * Configures the UART DMA. Mainly used for FreeRTOS trace
  * data transfer.
  */
void uartslkDmaInit(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

  // USART DMA Channel Config
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&UARTSLK_TYPE->DR;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_Memory0BaseAddr = 0; // Set seperately for Rx/Tx
  DMA_InitStructure.DMA_BufferSize = 0; // Set seperately for Rx/Tx

  // Configure TX DMA
  DMA_InitStructure.DMA_Channel = UARTSLK_TX_DMA_CH;
  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_Cmd(UARTSLK_TX_DMA_STREAM, DISABLE);
  DMA_Init(UARTSLK_TX_DMA_STREAM, &DMA_InitStructure);

  // Configure RX DMA
  DMA_InitStructure.DMA_Channel = UARTSLK_RX_DMA_CH;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)RxBuffer;
  DMA_InitStructure.DMA_BufferSize = RX_BUFFER_SIZE;
  DMA_Cmd(UARTSLK_RX_DMA_STREAM, DISABLE);
  DMA_Init(UARTSLK_RX_DMA_STREAM, &DMA_InitStructure);
  
  // Configure interrupts 
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_HIGH_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  
  NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_TX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);
  
  NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_RX_DMA_IRQ;
  NVIC_Init(&NVIC_InitStructure);
  
  
  // Enable Rx DMA
  DMA_ITConfig(UARTSLK_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
  /* Clear DMA flags */
  DMA_ClearFlag(UARTSLK_RX_DMA_STREAM, UARTSLK_RX_DMA_ALL_FLAGS);
  /* Enable USART DMA RX Requests */
  USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Rx, ENABLE);
  /* Clear transfer complete */
  //USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
  /* Enable DMA USART TX Stream */
  DMA_Cmd(UARTSLK_RX_DMA_STREAM, ENABLE);

  isUartDmaInitialized = true;
}

void uartslkInit(void)
{
  // initialize the FreeRTOS structures first, to prevent null pointers in interrupts
  waitUntilSendDone = xSemaphoreCreateBinary(); // initialized as blocking
  waitUntilReceiveDone = xSemaphoreCreateBinary(); // initialized as blocking
  uartTxBusy = xSemaphoreCreateBinary(); // initialized as blocking
  uartRxBusy = xSemaphoreCreateBinary(); // initialized as blocking
  xSemaphoreGive(uartTxBusy); // but we give it because the uart isn't busy at initialization
  xSemaphoreGive(uartRxBusy); // but we give it because the uart isn't busy at initialization

  uartslkDataDelivery = xQueueCreate(1024, sizeof(uint8_t));
  DEBUG_QUEUE_MONITOR_REGISTER(uartslkDataDelivery);

  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  EXTI_InitTypeDef extiInit;

  /* Enable GPIO and USART clock */
  RCC_AHB1PeriphClockCmd(UARTSLK_GPIO_PERIF, ENABLE);
  ENABLE_UARTSLK_RCC(UARTSLK_PERIF, ENABLE);

  /* Configure USART Rx as input floating */
  GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_RX_PIN;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

  /* Configure USART Tx as alternate function */
  GPIO_InitStructure.GPIO_Pin   = UARTSLK_GPIO_TX_PIN;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_Init(UARTSLK_GPIO_PORT, &GPIO_InitStructure);

  //Map uartslk to alternate functions
  GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_TX_PIN, UARTSLK_GPIO_AF_TX);
  GPIO_PinAFConfig(UARTSLK_GPIO_PORT, UARTSLK_GPIO_AF_RX_PIN, UARTSLK_GPIO_AF_RX);

#if defined(UARTSLK_OUTPUT_TRACE_DATA) || defined(ADC_OUTPUT_RAW_DATA) || defined(IMU_OUTPUT_RAW_DATA_ON_UART)
  USART_InitStructure.USART_BaudRate            = 2000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Tx;
#else
  USART_InitStructure.USART_BaudRate            = 1000000;
  USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
#endif
  USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits            = USART_StopBits_1;
  USART_InitStructure.USART_Parity              = USART_Parity_No ;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UARTSLK_TYPE, &USART_InitStructure);

  uartslkDmaInit();

  // Configure Rx buffer not empty interrupt
  NVIC_InitStructure.NVIC_IRQChannel = UARTSLK_IRQ;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_SYSLINK_PRI;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  USART_ITConfig(UARTSLK_TYPE, USART_IT_IDLE, ENABLE);

  //Setting up TXEN pin (NRF flow control)
  RCC_AHB1PeriphClockCmd(UARTSLK_TXEN_PERIF, ENABLE);

  bzero(&GPIO_InitStructure, sizeof(GPIO_InitStructure));
  GPIO_InitStructure.GPIO_Pin = UARTSLK_TXEN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(UARTSLK_TXEN_PORT, &GPIO_InitStructure);

  extiInit.EXTI_Line = UARTSLK_TXEN_EXTI;
  extiInit.EXTI_Mode = EXTI_Mode_Interrupt;
  extiInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  extiInit.EXTI_LineCmd = ENABLE;
  EXTI_Init(&extiInit);
  EXTI_ClearITPendingBit(UARTSLK_TXEN_EXTI);

  NVIC_EnableIRQ(EXTI4_IRQn);

  //Enable UART
  USART_Cmd(UARTSLK_TYPE, ENABLE);
  isInit = true;
}

bool uartslkTest(void)
{
  return isInit;
}

bool uartslkGetDataWithTimout(uint8_t *c)
{
  if (xQueueReceive(uartslkDataDelivery, c, UARTSLK_DATA_TIMEOUT_TICKS) == pdTRUE)
  {
    return true;
  }

  *c = 0;
  return false;
}

void uartslkSendData(uint32_t size, uint8_t* data)
{
  uint32_t i;

  if (!isInit)
    return;

  for(i = 0; i < size; i++)
  {
#ifdef UARTSLK_SPINLOOP_FLOWCTRL
    while(GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET);
#endif
    while (!(UARTSLK_TYPE->SR & USART_FLAG_TXE));
    UARTSLK_TYPE->DR = (data[i] & 0x00FF);
  }
}

void uartslkSendDataIsrBlocking(uint32_t size, uint8_t* data)
{
  xSemaphoreTake(uartTxBusy, portMAX_DELAY);
  outDataIsr = data;
  dataSizeIsr = size;
  dataIndexIsr = 1;
  uartslkSendData(1, &data[0]);
  USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, ENABLE);
  xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
  outDataIsr = 0;
  xSemaphoreGive(uartTxBusy);
}

int uartslkPutchar(int ch)
{
    uartslkSendData(1, (uint8_t *)&ch);
    
    return (unsigned char)ch;
}

void uartslkGetDataDmaBlocking(uint32_t size, uint8_t* data, uint32_t *pActualSize)
{
  if (isUartDmaInitialized)
  {
    // Enable the Transfer Complete interrupt
    xSemaphoreTake(waitUntilReceiveDone, portMAX_DELAY);

    uint32_t toCopy = rxBufferContentsSize;
    if (toCopy > size) 
    {
      toCopy = size;
    }

    memcpy(data, RxBuffer, toCopy);
    *pActualSize = toCopy;

  }
}

void uartslkSendDataDmaBlocking(uint32_t size, uint8_t* data)
{
  if (isUartDmaInitialized)
  {
    xSemaphoreTake(uartTxBusy, portMAX_DELAY);
    // Wait for DMA to be free
    while(DMA_GetCmdStatus(UARTSLK_TX_DMA_STREAM) != DISABLE);

    // Set the Tx buffer to the data
    UARTSLK_TX_DMA_STREAM->M0AR = (uint32_t)&data[0];
    UARTSLK_TX_DMA_STREAM->NDTR = size; 
    initialDMACount = size;

    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UARTSLK_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Clear DMA flags */
    DMA_ClearFlag(UARTSLK_TX_DMA_STREAM, UARTSLK_TX_DMA_ALL_FLAGS);
    /* Enable USART DMA TX Requests */
    USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UARTSLK_TX_DMA_STREAM, ENABLE);
    xSemaphoreTake(waitUntilSendDone, portMAX_DELAY);
    xSemaphoreGive(uartTxBusy);
  }
}

static void uartslkPauseDma()
{
  if (DMA_GetCmdStatus(UARTSLK_TX_DMA_STREAM) == ENABLE)
  {
    // Disable transfer complete interrupt
    DMA_ITConfig(UARTSLK_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
    // Disable stream to pause it
    DMA_Cmd(UARTSLK_TX_DMA_STREAM, DISABLE);
    // Wait for it to be disabled
    while(DMA_GetCmdStatus(UARTSLK_TX_DMA_STREAM) != DISABLE);
    // Disable transfer complete
    DMA_ClearITPendingBit(UARTSLK_TX_DMA_STREAM, UARTSLK_TX_DMA_FLAG_TCIF);
    // Read remaining data count
    remainingDMACount = DMA_GetCurrDataCounter(UARTSLK_TX_DMA_STREAM);
    dmaIsPaused = true;
  }
}

static void uartslkResumeDma()
{
  if (dmaIsPaused)
  {
    // Update DMA counter
    DMA_SetCurrDataCounter(UARTSLK_TX_DMA_STREAM, remainingDMACount);
    // Update memory read address
    UARTSLK_TX_DMA_STREAM->M0AR = (uint32_t)&dmaBuffer[initialDMACount - remainingDMACount];
    // Enable the Transfer Complete interrupt
    DMA_ITConfig(UARTSLK_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
    /* Clear transfer complete */
    USART_ClearFlag(UARTSLK_TYPE, USART_FLAG_TC);
    /* Enable DMA USART TX Stream */
    DMA_Cmd(UARTSLK_TX_DMA_STREAM, ENABLE);
    dmaIsPaused = false;
  }
}

void uartslkDmaTxIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Stop and cleanup DMA stream
  DMA_ITConfig(UARTSLK_TX_DMA_STREAM, DMA_IT_TC, DISABLE);
  DMA_ClearITPendingBit(UARTSLK_TX_DMA_STREAM, UARTSLK_TX_DMA_FLAG_TCIF);
  USART_DMACmd(UARTSLK_TYPE, USART_DMAReq_Tx, DISABLE);
  DMA_Cmd(UARTSLK_TX_DMA_STREAM, DISABLE);

  remainingDMACount = 0;
  xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
}

void uartslkDmaRxIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // Get the total number of bytes that were written
  if( rxBufferContentsSize != RX_BUFFER_SIZE - UARTSLK_RX_DMA_STREAM->NDTR)
  {
    rxBufferContentsSize = RX_BUFFER_SIZE - UARTSLK_RX_DMA_STREAM->NDTR;
  }

  if(rxBufferContentsSize > 32)
  {
    DEBUG_PRINT("LARGE PACKET?");
  }

  if(!isValidSyslinkPacket())
  {
    DEBUG_PRINT("RECEIVED INVALID SYSLINK PACKET");
  }

  // this breaks uplevel but will make it easier to find errors.
  memset(RxBuffer, 0, RX_BUFFER_SIZE);

  // Restart the DMA
  DMA_ClearFlag(UARTSLK_RX_DMA_STREAM, UARTSLK_RX_DMA_ALL_FLAGS);
  UARTSLK_RX_DMA_STREAM->NDTR = RX_BUFFER_SIZE;
  UARTSLK_RX_DMA_STREAM->M0AR = (uint32_t)RxBuffer;
  /* Enable DMA USART TX Stream */
  DMA_Cmd(UARTSLK_RX_DMA_STREAM, ENABLE);

  xSemaphoreGiveFromISR(waitUntilReceiveDone, &xHigherPriorityTaskWoken);
}

void uartslkIsr(void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  // the following if statement replaces:
  //   if (USART_GekktITStatus(UARTSLK_TYPE, USART_IT_RXNE) == SET)
  // we do this check as fast as possible to minimize the chance of an overrun,
  // which occasionally cause problems and cause packet loss at high CPU usage
  if ((UARTSLK_TYPE->SR & (1<<5)) != 0) // if the RXNE interrupt has occurred
  {
    uint8_t rxDataInterrupt = (uint8_t)(UARTSLK_TYPE->DR & 0xFF);
    xQueueSendFromISR(uartslkDataDelivery, &rxDataInterrupt, &xHigherPriorityTaskWoken);
  }
  else if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_IDLE) == SET)
  {
    // Disable the DMA
    DMA_Cmd(UARTSLK_RX_DMA_STREAM, DISABLE);
    
    // Block on the EN bit -- TODO - should add a timeout here
    while((UARTSLK_RX_DMA_STREAM->CR & CCR_ENABLE_SET) != 0);

    //rxBufferContentsSize = RX_BUFFER_SIZE - UARTSLK_RX_DMA_STREAM->NDTR;
  // Get the total number of bytes that were written
  rxBufferContentsSize = RX_BUFFER_SIZE - UARTSLK_RX_DMA_STREAM->NDTR;

  if(rxBufferContentsSize > 32)
  {
    DEBUG_PRINT("LARGE PACKET?");
  }
    
    // This step is necessary to clear the IDLE interrupt
    USART_ReceiveData(UARTSLK_TYPE);
  }
  else if (USART_GetITStatus(UARTSLK_TYPE, USART_IT_TXE) == SET)
  {
    if (outDataIsr && (dataIndexIsr < dataSizeIsr))
    {
      USART_SendData(UARTSLK_TYPE, outDataIsr[dataIndexIsr] & 0x00FF);
      dataIndexIsr++;
    }
    else
    {
      USART_ITConfig(UARTSLK_TYPE, USART_IT_TXE, DISABLE);
      xSemaphoreGiveFromISR(waitUntilSendDone, &xHigherPriorityTaskWoken);
    }
  }
  else
  {
    /** if we get here, the error is most likely caused by an overrun!
     * - PE (Parity error), FE (Framing error), NE (Noise error), ORE (OverRun error)
     * - and IDLE (Idle line detected) pending bits are cleared by software sequence:
     * - reading USART_SR register followed reading the USART_DR register.
     */
    asm volatile ("" : "=m" (UARTSLK_TYPE->SR) : "r" (UARTSLK_TYPE->SR)); // force non-optimizable reads
    asm volatile ("" : "=m" (UARTSLK_TYPE->DR) : "r" (UARTSLK_TYPE->DR)); // of these two registers
  }
}

void uartslkTxenFlowctrlIsr()
{
  EXTI_ClearFlag(UARTSLK_TXEN_EXTI);
  if (GPIO_ReadInputDataBit(UARTSLK_TXEN_PORT, UARTSLK_TXEN_PIN) == Bit_SET)
  {
    uartslkPauseDma();
    //ledSet(LED_GREEN_R, 1);
  }
  else
  {
    uartslkResumeDma();
    //ledSet(LED_GREEN_R, 0);
  }
}

void __attribute__((used)) EXTI4_Callback(void)
{
  uartslkTxenFlowctrlIsr();
}

void __attribute__((used)) USART6_IRQHandler(void)
{
  uartslkIsr();
}

void __attribute__((used)) DMA2_Stream7_IRQHandler(void)
{
  uartslkDmaTxIsr();
}

void __attribute__((used)) DMA2_Stream1_IRQHandler(void)
{
  uartslkDmaRxIsr();
}
