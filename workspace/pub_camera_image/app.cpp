/* mros2 example
 * Copyright (c) 2023 smorita_emb
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "mros2.h"
#include "mros2-platform.h"
#include "sensor_msgs/msg/image.hpp"

#define OV7670_WRITE (0x42)
#define OV7670_READ (0x43)
#define OV7670_I2CFREQ (200) // us half period

// The class OV7670 is imported from the repository below.
// https://os.mbed.com/users/sylvainkritter/code/ov7670s/
class OV7670
{
public:
  OV7670(
      PinName siod, // Camera I2C port data
      PinName sioc, // Camera I2C port clock
      PinName rst   // RESET
  );

  void WriteBy(int addr);             // write to camera
  void WriteReg2(int data);           // write to camera
  void WriteReg(int subad, int data); // write to camera
  int ReadReg(int subad);             // read from camera
  int ReadBy(void);
  void Reset(void); // reset reg camera
  void Start(void);
  void End(void);
  int Init();

private:
  DigitalInOut _siod;
  DigitalOut _sioc;
  DigitalOut _rst;
};

OV7670::OV7670(
    PinName siod,
    PinName sioc,
    PinName rst) : _siod(siod), _sioc(sioc), _rst(rst)
{
}

void OV7670::Start()
{
  _sioc = 1;
  _siod.output();
  _siod = 1;
  wait_us(OV7670_I2CFREQ);
  _siod = 0;
  wait_us(OV7670_I2CFREQ);
  _sioc = 0;
  wait_us(OV7670_I2CFREQ);
}

void OV7670::End()
{
  _sioc = 0;
  _siod.output();
  _siod = 0;
  wait_us(OV7670_I2CFREQ);
  _sioc = 1;
  wait_us(OV7670_I2CFREQ);
  _siod = 1;
  wait_us(OV7670_I2CFREQ);
  _siod.input();
  wait_us(OV7670_I2CFREQ);
}

void OV7670::Reset()
{
  _sioc = 1;
  _siod.input();
  _rst = 1;
  wait_us(1000);
  _rst = 0;
  wait_us(1000);
  _rst = 1;
  wait_us(1000);
  WriteReg(0x12, 0x80);
  wait_us(300000);
  WriteReg(0x12, 0x00);
  wait_us(300000);
}

void OV7670::WriteBy(int addr)
{
  int tempo = 0;
  _siod.output();
  wait_us(1);
  _sioc = 0;
  _siod = 1;
  for (int i = 0; i < 8; i++)
  {
    tempo = addr >> (7 - i);
    tempo = tempo & 0x01;
    _sioc = 0;
    _siod = tempo;
    ;
    wait_us(OV7670_I2CFREQ);
    _sioc = 1;
    wait_us(OV7670_I2CFREQ);
  }
  _sioc = 0;
  _siod = 1;
  _siod.input();
  wait_us(OV7670_I2CFREQ);
  _sioc = 1;
  wait_us(OV7670_I2CFREQ);
  _sioc = 0;
}

int OV7670::ReadBy()
{
  unsigned char data[8];
  int tt = 0;
  _siod.input();
  _sioc = 0;
  for (int i = 0; i < 8; i++)
  {
    _sioc = 0;
    wait_us(OV7670_I2CFREQ);
    _sioc = 1;
    data[7 - i] = _siod;
    wait_us(OV7670_I2CFREQ);
  }
  _sioc = 0;
  for (int i = 0; i < 8; i++)
  {
    tt = tt | data[7 - i];
    if (i < 7)
    {
      tt = tt << 1;
    }
  }
  _siod.output();
  _siod = 1;
  wait_us(OV7670_I2CFREQ);
  _sioc = 1;
  wait_us(OV7670_I2CFREQ);
  _sioc = 0;

  return tt;
}

void OV7670::WriteReg2(int data)
{
  Start();
  WriteBy(OV7670_WRITE);
  WriteBy(data);
  End();
}

void OV7670::WriteReg(int subad, int data)
{
  Start();
  WriteBy(OV7670_WRITE);
  WriteBy(subad);
  WriteBy(data);
  End();
}

int OV7670::ReadReg(int subad)
{
  int dr = 0;
  WriteReg2(subad);
  Start();
  WriteBy(OV7670_READ);
  dr = ReadBy();
  End();
  return dr;
}

int OV7670::Init()
{
  GPIO_InitTypeDef GPIO_InitStruct;
  // Configure GPIO pin : PA8 MCO1 for cam XCLK
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  __GPIOA_CLK_ENABLE();
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  Reset();

  int x = ReadReg(0x0a);
  if (x != 0x76)
  {
    MROS2_INFO("check id camera error! 0x%x", x);
    return 1;
  }
  else
  {
    MROS2_INFO("check id camera OK! 0x%x", x);
  }

  WriteReg(0x12, 0x08);

  WriteReg(0x8c, 0x00);
  WriteReg(0x04, 0x00);
  WriteReg(0x40, 0xc0);

  WriteReg(0x14, 0x48);
  WriteReg(0xaa, 0x94);
  WriteReg(0x3a, 0x0c);
  WriteReg(0x20, 0x04);
  WriteReg(0x13, 0xff);
  WriteReg(0x10, 0x0f);
  WriteReg(0x07, 0x00);

  WriteReg(0x01, 0x40);
  WriteReg(0x02, 0x60);
  WriteReg(0x03, 0x0a);
  WriteReg(0x0c, 0x00);
  WriteReg(0x0e, 0x61);
  WriteReg(0x0f, 0x4b);
  WriteReg(0x15, 0x00);
  WriteReg(0x16, 0x02);
  WriteReg(0x17, 0x18);
  WriteReg(0x18, 0x01);
  WriteReg(0x19, 0x02);
  WriteReg(0x1a, 0x7a);
  WriteReg(0x1e, 0x07);
  WriteReg(0x21, 0x02);
  WriteReg(0x22, 0x91);
  WriteReg(0x29, 0x07);
  WriteReg(0x32, 0xb6);
  WriteReg(0x33, 0x0b);
  WriteReg(0x34, 0x11);
  WriteReg(0x35, 0x0b);
  WriteReg(0x37, 0x1d);
  WriteReg(0x38, 0x71);
  WriteReg(0x39, 0x2a);
  WriteReg(0x3b, 0x92);
  WriteReg(0x3c, 0x78);
  WriteReg(0x3d, 0xc3);
  WriteReg(0x3e, 0x00);
  WriteReg(0x3f, 0x00);
  WriteReg(0x41, 0x08);
  WriteReg(0x41, 0x38);
  WriteReg(0x43, 0x0a);
  WriteReg(0x44, 0xf0);
  WriteReg(0x45, 0x34);
  WriteReg(0x46, 0x58);
  WriteReg(0x47, 0x28);
  WriteReg(0x48, 0x3a);
  WriteReg(0x4b, 0x09);
  WriteReg(0x4c, 0x00);
  WriteReg(0x4d, 0x40);
  WriteReg(0x4e, 0x20);
  WriteReg(0x4f, 0x80);
  WriteReg(0x50, 0x80);
  WriteReg(0x51, 0x00);
  WriteReg(0x52, 0x22);
  WriteReg(0x53, 0x5e);
  WriteReg(0x54, 0x80);
  WriteReg(0x56, 0x40);
  WriteReg(0x58, 0x9e);
  WriteReg(0x59, 0x88);
  WriteReg(0x5a, 0x88);
  WriteReg(0x5b, 0x44);
  WriteReg(0x5c, 0x67);
  WriteReg(0x5d, 0x49);
  WriteReg(0x5e, 0x0e);
  WriteReg(0x69, 0x00);
  WriteReg(0x6a, 0x40);
  WriteReg(0x6b, 0x0a);
  WriteReg(0x6c, 0x0a);
  WriteReg(0x6d, 0x55);
  WriteReg(0x6e, 0x11);
  WriteReg(0x6f, 0x9f);
  WriteReg(0x70, 0x3a);
  WriteReg(0x71, 0x35);
  WriteReg(0x72, 0x11);
  WriteReg(0x73, 0xf0);
  WriteReg(0x74, 0x10);
  WriteReg(0x75, 0x05);
  WriteReg(0x76, 0xe1);
  WriteReg(0x77, 0x01);
  WriteReg(0x78, 0x04);
  WriteReg(0x79, 0x01);
  WriteReg(0x8d, 0x4f);
  WriteReg(0x8e, 0x00);
  WriteReg(0x8f, 0x00);
  WriteReg(0x90, 0x00);
  WriteReg(0x91, 0x00);
  WriteReg(0x96, 0x00);
  WriteReg(0x96, 0x00);
  WriteReg(0x97, 0x30);
  WriteReg(0x98, 0x20);
  WriteReg(0x99, 0x30);
  WriteReg(0x9a, 0x00);
  WriteReg(0x9a, 0x84);
  WriteReg(0x9b, 0x29);
  WriteReg(0x9c, 0x03);
  WriteReg(0x9d, 0x4c);
  WriteReg(0x9e, 0x3f);
  WriteReg(0xa2, 0x02);
  WriteReg(0xa4, 0x88);
  WriteReg(0xb0, 0x84);
  WriteReg(0xb1, 0x0c);
  WriteReg(0xb2, 0x0e);
  WriteReg(0xb3, 0x82);
  WriteReg(0xb8, 0x0a);
  WriteReg(0xc8, 0xf0);
  WriteReg(0xc9, 0x60);
  WriteReg(0x6b, 0x00);

  return 0;
}

DMA_HandleTypeDef dcmi_Dma;
DCMI_HandleTypeDef dcmi;

void dcmi_Init(uint32_t buf_ptr,
               uint32_t buf_size)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  // GPIOA
  __GPIOA_CLK_ENABLE();
  //              HSYNC(PA4)   PIXCLK(PA6)
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // GPIOB
  __GPIOB_CLK_ENABLE();
  //              D5(PB6)      VSYNC(PB7)
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // GPIOC
  __GPIOC_CLK_ENABLE();
  //              D0(PC6)      D1(PC7)      D2(PC8)     D3(PC9)
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  // GPIOE
  __GPIOE_CLK_ENABLE();
  //              D4(PE4)      D6(PE5)      D7(PE6)
  GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  // DMA for DCMI
  __DMA2_CLK_ENABLE();
  dcmi_Dma.Instance = DMA2_Stream1;
  dcmi_Dma.Init.Channel = DMA_CHANNEL_1;
  dcmi_Dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
  dcmi_Dma.Init.PeriphInc = DMA_PINC_DISABLE;
  dcmi_Dma.Init.MemInc = DMA_MINC_ENABLE;
  dcmi_Dma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  dcmi_Dma.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  dcmi_Dma.Init.Mode = DMA_CIRCULAR;
  dcmi_Dma.Init.Priority = DMA_PRIORITY_HIGH;
  dcmi_Dma.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  dcmi_Dma.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  dcmi_Dma.Init.MemBurst = DMA_PBURST_SINGLE;
  dcmi_Dma.Init.PeriphBurst = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&dcmi_Dma);

  // DCMI
  __DCMI_CLK_ENABLE();
  dcmi.Instance = DCMI;
  dcmi.Init.ExtendedDataMode = DCMI_EXTEND_DATA_8B;
  dcmi.Init.CaptureRate = DCMI_CR_ALL_FRAME;
  dcmi.Init.VSPolarity = DCMI_VSPOLARITY_HIGH;
  dcmi.Init.HSPolarity = DCMI_HSPOLARITY_LOW;
  dcmi.Init.PCKPolarity = DCMI_PCKPOLARITY_RISING;
  dcmi.Init.SynchroMode = DCMI_SYNCHRO_HARDWARE;
  dcmi.DMA_Handle = &dcmi_Dma;
  HAL_DCMI_Init(&dcmi);
  __HAL_DCMI_ENABLE(&dcmi);
  HAL_DCMI_Start_DMA(&dcmi,
                     DCMI_MODE_CONTINUOUS,
                     buf_ptr,
                     buf_size / sizeof(uint32_t));
}

#define IP_ADDRESS ("192.168.11.2")      // IP address
#define SUBNET_MASK ("255.255.255.0")    // Subnet mask
#define DEFAULT_GATEWAY ("192.168.11.1") // Default gateway

OV7670 s(PB_9, PB_8, PA_0);

int main(int argc, char *argv[])
{
  /* connect to the network */
  if (mros2_platform::network_connect())
  {
    MROS2_ERROR("failed to connect and setup network! aborting,,,");
    return -1;
  }
  else
  {
    MROS2_INFO("successfully connect and setup network\r\n---");
  }

  MROS2_INFO("%s start!", MROS2_PLATFORM_NAME);
  MROS2_INFO("app name: pub_camera_image");

  auto msg = sensor_msgs::msg::Image();

  mros2::init(0, NULL);
  MROS2_DEBUG("mROS 2 initialization is completed");

  mros2::Node node = mros2::Node::create_node("mros2_node");
  mros2::Publisher pub = node.create_publisher<sensor_msgs::msg::Image>("to_linux", 10);

  osDelay(100);
  MROS2_INFO("ready to pub camera image\r\n---");
  s.Init();
  msg.sec = time(NULL);
  msg.nanosec = 0;
  msg.frame_id = "frame";
  msg.height = 145;
  msg.width = 150;
  msg.encoding = "yuv422";
  msg.is_bigendian = false;
  msg.step = msg.width * 2;
  size_t image_size = msg.step * msg.height;
  msg.data.resize(image_size);
  dcmi_Init((uint32_t)&msg.data[0], image_size);

  while (1)
  {
    MROS2_INFO("publishing image");
    pub.publish(msg);
    osDelay(100);
  }

  mros2::spin();
  return 0;
}
