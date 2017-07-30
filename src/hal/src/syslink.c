/*
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2012 BitCraze AB
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
 * syslink.c: Communication between NRF51 and STM32
 */
#define DEBUG_MODULE "SL"

#include <stdbool.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "config.h"
#include "debug.h"
#include "syslink.h"
#include "radiolink.h"
#include "uart_syslink.h"
#include "configblock.h"
#include "pm.h"
#include "ow.h"

static bool isInit = false;
static uint8_t sendBuffer[64];

static void syslinkRouteIncommingPacket(SyslinkPacket *slp);

static xSemaphoreHandle syslinkAccess;

/* Syslink task, handles communication between nrf and stm and dispatch messages
 */
static void syslinkTask(void *param)
{
  SyslinkRxState rxState = waitForFirstStart;
  SyslinkPacket slp;
  uint8_t c;
  uint8_t dataIndex = 0;
  uint8_t cksum[2] = {0};
  uint8_t counter = 0;
  uint8_t done = 0;
  uint32_t bytesReceived;

  uint32_t lastWakeTime = xTaskGetTickCount(); 
  uint8_t rxBuffer[200];
  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(1000));
 
    // this shouldn't be necessary, but zero it for debug.
    memset(rxBuffer, 0, sizeof(rxBuffer));
//    uartslkGetDataDmaBlocking(sizeof(rxBuffer), rxBuffer, &actualSize);

    uartslkGetAvailableData(200, rxBuffer, &bytesReceived);

    counter = 0;
    done = 0;
    while(counter < bytesReceived && done == 0)
    {
      c = rxBuffer[counter];
      switch(rxState)
      {
        case waitForFirstStart:
          rxState = (c == SYSLINK_START_BYTE1) ? waitForSecondStart : waitForFirstStart;
          break;
        case waitForSecondStart:
          rxState = (c == SYSLINK_START_BYTE2) ? waitForType : waitForFirstStart;
          break;
        case waitForType:
          cksum[0] = c;
          cksum[1] = c;
          slp.type = c;
          rxState = waitForLengt;
          break;
        case waitForLengt:
          if (c <= SYSLINK_MTU)
          {
            slp.length = c;

            cksum[0] += c;
            cksum[1] += cksum[0];
            dataIndex = 0;
            rxState = (c > 0) ? waitForData : waitForChksum1;
          }
          else
          {
            rxState = waitForFirstStart;
          }
          break;
        case waitForData:
          slp.data[dataIndex] = c;
          cksum[0] += c;
          cksum[1] += cksum[0];
          dataIndex++;
          if (dataIndex == slp.length)
          {
            rxState = waitForChksum1;
          }
          break;
        case waitForChksum1:
          if (cksum[0] == c)
          {
            rxState = waitForChksum2;
          }
          else
          {
            rxState = waitForFirstStart; //Checksum error
            //IF_DEBUG_ASSERT(1);
          }
          break;
        case waitForChksum2:
          if (cksum[1] == c)
          {
            syslinkRouteIncommingPacket(&slp);
            done = 1;
          }
          else
          {
            rxState = waitForFirstStart; //Checksum error
            //IF_DEBUG_ASSERT(1);
          }
          rxState = waitForFirstStart;
          break;
        default:
          ASSERT(0);
          break;
      }
      counter++;
    }
  }
}

static void syslinkRouteIncommingPacket(SyslinkPacket *slp)
{
  uint8_t groupType;

  groupType = slp->type & SYSLINK_GROUP_MASK;

  switch (groupType)
  {
    case SYSLINK_RADIO_GROUP:
      radiolinkSyslinkDispatch(slp);
      break;
    case SYSLINK_PM_GROUP:
      pmSyslinkUpdate(slp);
      break;
    case SYSLINK_OW_GROUP:
      owSyslinkRecieve(slp);
      break;
    default:
      DEBUG_PRINT("Unknown packet:%X.\n", slp->type);
      break;
  }
}

/*
 * Public functions
 */

void syslinkInit()
{
  if(isInit)
    return;

  vSemaphoreCreateBinary(syslinkAccess);

  if (xTaskCreate(syslinkTask, SYSLINK_TASK_NAME,
                  SYSLINK_TASK_STACKSIZE, NULL, SYSLINK_TASK_PRI, NULL) == pdPASS)
  {
    isInit = true;
  }
}

bool syslinkTest()
{
  return isInit;
}

int syslinkSendPacket(SyslinkPacket *slp)
{
  int i = 0;
  int dataSize;
  uint8_t cksum[2] = {0};

  xSemaphoreTake(syslinkAccess, portMAX_DELAY);

  ASSERT(slp->length <= SYSLINK_MTU);

  sendBuffer[0] = SYSLINK_START_BYTE1;
  sendBuffer[1] = SYSLINK_START_BYTE2;
  sendBuffer[2] = slp->type;
  sendBuffer[3] = slp->length;

  memcpy(&sendBuffer[4], slp->data, slp->length);
  dataSize = slp->length + 6;
  // Calculate checksum delux
  for (i = 2; i < dataSize - 2; i++)
  {
    cksum[0] += sendBuffer[i];
    cksum[1] += cksum[0];
  }
  sendBuffer[dataSize-2] = cksum[0];
  sendBuffer[dataSize-1] = cksum[1];

  uartslkSendDataDmaBlocking(dataSize, sendBuffer);

  xSemaphoreGive(syslinkAccess);

  return 0;
}
