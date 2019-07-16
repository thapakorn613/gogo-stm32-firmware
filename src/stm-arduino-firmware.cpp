#include <Arduino.h>

#include <libmaple/usart.h>
#include <libmaple/timer.h>

#include "stm-arduino-firmware.h"
// #include "stm32-firmware.h"
#include "stm32-uart.h"
#include "stm32-configuration.h"

#define REPORT_PACKET_SIZE 16


// * ////////////////////////////////////////////////////////////
// *	Global Serial Buffer
// * ////////////////////////////////////////////////////////////

uint8 inByte;

uint8 gbl1stCMDBuffer[SER_BUFFER_SIZE];
uint8 gbl2ndCMDBuffer[SER_BUFFER_SIZE];

uint8 gblSTMReport[REPORT_PACKET_SIZE] = {0};

uint8 headerReportPkt[4] = {0x54, 0xFE, 0x14, REPORT_PACKET_SIZE + 1};

bool reportPktMode = false; // default Polling
bool reportPkt_readReg = false;
bool reportPktReady = false;
uint8_t reportPkt_readRegVal = 0;
int inBytePktBegin = 4;

// * ////////////////////////////////////////////////////////////
// *	Global Variables
// * ////////////////////////////////////////////////////////////
bool gblTimeToSendReport = false;
int gblTimeToSendReportCounter = 0;

bool gblTimeToUpdateReport = false;
int gblTimeToUpdateReportCounter = 0;

int hbCounter = 0;
bool gblIsTimeToToggleLED = false;
bool gblToggleLED = false;

bool gblSerialPassthrough = false;
uint8 gblSerialTxLen = 0;
uint8 gblHIDCmdChecksum = 0;

int toggle = 0;

uint8_t reg_index[9] = {0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98, 0x99};
uint8_t gblRegStm[16];

// * ////////////////////////////////////////////////////////////
// *	EspSerial USART1 interrupt handler
// * ////////////////////////////////////////////////////////////

void usartSerialEvent(uint8 inByte)
{
    if (inByte == SERIAL_1ST_HEADER && gblSerialState == SER_WAITING_FOR_1ST_HEADER)
    {
        gblSerialState = SER_WAITING_FOR_2ND_HEADER;
    }
    else if (gblSerialState == SER_WAITING_FOR_2ND_HEADER)
    {
        if (inByte == SERIAL_2ND_HEADER)
        {
            gblSerialState = SER_WAITING_FOR_LENGTH;
        }
        else
        {
            gblSerialState = SER_WAITING_FOR_1ST_HEADER;
        }
    }
    else if (gblSerialState == SER_WAITING_FOR_LENGTH)
    {
        gblSerialCmdLength = inByte;
        gblSerialCmdCounter = 0;
        gblSerialCmdChecksum = 0;
        gblSerialState = SER_WAITING_FOR_CMD;
    }
    else if (gblSerialState == SER_WAITING_FOR_CMD)
    {
        // if at cmd end -> do check sum
        if (gblSerialCmdCounter == gblSerialCmdLength - 1)
        {
            if (gblSerialCmdChecksum == inByte)
            {
                if (!gblSerialPassthrough)
                {

                    gblNewCmdReady = true;
                }
            }
            gblSerialState = SER_WAITING_FOR_1ST_HEADER;
            gblUseFirstCmdBuffer = !gblUseFirstCmdBuffer;
            gblSerialPassthrough = false;
        }
        else // else store the cmd in the buffer
        {

            // if this is a serial-passthrough packet
            if ((gblSerialCmdCounter == 0) && (inByte == 17))
            {
                gblSerialPassthrough = true;
                // if passing through
            }
            else if (gblSerialPassthrough)
            {
                // gblSerialOutData[gblSerialCmdCounter++] = inByte; //* buffer wait for send out through HID
                gblSerialCmdChecksum += inByte;
                // if PIC command
            }
            else
            {
                if (gblUseFirstCmdBuffer)
                {
                    gbl1stCMDBuffer[gblSerialCmdCounter++] = inByte;
                }
                else
                {
                    gbl2ndCMDBuffer[gblSerialCmdCounter++] = inByte;
                }
                gblSerialCmdChecksum += inByte;
            }
        }
    }
    else
    {
        /* Handling RXNEIE and TXEIE interrupts. 
     	 * RXNE signifies availability of a byte in DR.
		 *
		 * See table 198(sec 27.4, p809)in STM document RM0008 rev 15.
		 * We enable RXNEIE. */
#ifdef USART_SAFE_INSERT
        /* If the buffer is full and the user defines USART_SAFE_INSERT,
         * ignore new bytes. */
        rb_safe_insert(USART1->rb, (uint8)USART1_BASE->DR);
#else
        /* By default, push bytes around in the ring buffer. */
        rb_push_insert(USART1->rb, (uint8)USART1_BASE->DR);
#endif
    }
}

int newCmdPacketReady()
{
    if (!gblNewCmdReady)
    {
        return 0;
    }
    else
    {
        return gblUseFirstCmdBuffer ? 2 : 1;
    }
}
// 	returns 0 if no cmd is ready
//  	   	1 if cmd is ready in the 1st buffer
// 			2 if cmd is ready in the 2nd buffer

void clearCmdReadyFlag()
{
    gblNewCmdReady = false;
}

void __irq_usart1()
{
    if ((USART1_BASE->CR1 & USART_CR1_RXNEIE) && (USART1_BASE->SR & USART_SR_RXNE))
    {
        //* received byte is ok to process
        usartSerialEvent((uint8)USART1_BASE->DR);
    }
    //* TXE signifies readiness to send a byte to DR.
    if ((USART1_BASE->CR1 & USART_CR1_TXEIE) && (USART1_BASE->SR & USART_SR_TXE))
    {
        if (!rb_is_empty(USART1->wb))
            USART1_BASE->DR = rb_remove(USART1->wb);
        else
            USART1_BASE->CR1 &= ~((uint32)USART_CR1_TXEIE); //* disable TXEIE
    }
}

// * ////////////////////////////////////////////////////////////
// *	Serial UART functions
// * ////////////////////////////////////////////////////////////

//! STM32 -> ESP32 (Report Pkt)
void sendReportPacket()
{
#ifdef REPORT_MODE_POLLING
    if (gblTimeToSendReport && reportPktReady) // Polling
    {
        //? report stuff goes here !
        uint8 gblReportChecksum = 0;
        DebugSerial.write(headerReportPkt, 4);

        for (int i = 0; i < REPORT_PACKET_SIZE; i++)
        {
            DebugSerial.write(gblSTMReport[i]);
            gblReportChecksum += gblSTMReport[i];
        }
        DebugSerial.write(gblReportChecksum);
        // clearCmdPktFlag();
        gblTimeToSendReport = false;
        reportPktReady = false;
    }
#endif
#ifndef REPORT_MODE_POLLING
    if (gblTimeToSendReport) // Streamming
    {
        //? report stuff goes here !
        uint8 gblReportChecksum = 0;

        DebugSerial.write(headerReportPkt, 4);

        for (int i = 0; i < REPORT_PACKET_SIZE; i++)
        {
            DebugSerial.write(gblSTMReport[i]);
            gblReportChecksum += gblSTMReport[i];
        }

        DebugSerial.write(gblReportChecksum);
        // clearCmdPktFlag();
        gblTimeToSendReport = false;
    }
#endif
}

void processCMD(uint8 *inBytePkt)
{
    if (*inBytePkt == CMD_PACKET)
    {
        switch (*(inBytePkt + 1))
        {
        case CMD_CATEGORY_ID:
            if (*(inBytePkt + 2) == CMD_READ_REG)
            {
                uint8_t temp = *(inBytePkt + 3) - 1;
                gblSTMReport[1] = 0x03;
                gblSTMReport[2] = 0x01;
                gblSTMReport[3] = gblRegStm[temp];
            }
            else if (*(inBytePkt + 2) == CMD_SEND_STM_RX)
            {
                int lenghtPkt = *(inBytePkt + 3);
                for (int i = 0; i < lenghtPkt; i++)
                {
                    gblRegStm[i] = *(inBytePkt + inBytePktBegin + i);
                    gblSTMReport[i+1] = *(inBytePkt + inBytePktBegin + i);  // For Debug
                }
            }
#ifdef REPORT_MODE_POLLING
            reportPktReady = true;
#endif
            break;

        case CMD_REBOOT:
            nvic_sys_reset();
            break;
        }
    }
}

//! ESP -> STM32
//! COM -> STM32
void processEspSerialCMD()
{
    // uint8 *inEspPkt; // point to CMD buffer
    if (newCmdPacketReady())
    {
        // DebugSerial.println("");
        // DebugSerial.println("-----------------------------------");
        // DebugSerial.print("Buffer: ");
        // DebugSerial.println(newCmdPacketReady());
        // DebugSerial.println("Found CMD packet");

        uint8 *inEspPkt = (newCmdPacketReady() == 1) ? gbl1stCMDBuffer : gbl2ndCMDBuffer;
        clearCmdReadyFlag();
        processCMD(inEspPkt);
    }
}

void processInputSerial()
{
    processEspSerialCMD();
}

void processOutputSerial()
{
    sendReportPacket();
}

// * ////////////////////////////////////////////////////////////
// *	Misc. functions
// * ////////////////////////////////////////////////////////////

void heartbeatLED()
{
    if (gblIsTimeToToggleLED)
    {
        toggle ^= 1;
        digitalWrite(STATUS_LED, toggle);
        gblIsTimeToToggleLED = false;
    }
}

// * ////////////////////////////////////////////////////////////
// *	Interrupt handler
// * ////////////////////////////////////////////////////////////

void interrupt_1ms()
{
    //* update peripherals state every 25ms
    if (gblTimeToUpdateReportCounter++ >= 25)
    {
        gblTimeToUpdateReport = true;
        gblTimeToUpdateReportCounter = 0;
    }

    //* send report packet every 30ms -> 33Hz
    // 30
    if (gblTimeToSendReportCounter++ >= 30)
    {
        gblTimeToSendReport = true;
        gblTimeToSendReportCounter = 0;
    }

    if (hbCounter++ >= 1000)
    {
        gblIsTimeToToggleLED = true;
        hbCounter = 0;
    }
}

void setup()
{
    // pinMode(led_pin, OUTPUT);
    // Serial.begin(115200);
    DebugSerial.begin(115200);
    // DebugSerial3.begin(115200);
    // gogoIO.begin();
    // pinMode(STATUS_LED, OUTPUT);
    //* initialize timer interrupt using timer 3 channel 1
    Timer3.setMode(TIMER_CH1, TIMER_OUTPUTCOMPARE);
    Timer3.setPeriod(1000);
    Timer3.setCompare(TIMER_CH1, 1);
    Timer3.attachInterrupt(TIMER_CH1, interrupt_1ms);
    
    // Serial.println("STM32 Started , wait for Packet. Serial");
    DebugSerial.println("STM32 Started , wait for Packet. ");
    // DebugSerial3.println("STM32 Started , wait for Packet. DebugSerial3");
    // put your setup code here, to run once:
    delay(100);
}

void loop()
{
    processInputSerial();
    processOutputSerial();

    // heartbeatLED();

    // put your main code here, to run repeatedly:
}