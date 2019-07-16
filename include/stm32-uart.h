#define SER_WAITING_FOR_1ST_HEADER  1
#define SER_WAITING_FOR_2ND_HEADER  2
#define SER_WAITING_FOR_LENGTH      3
#define SER_WAITING_FOR_CMD         4

#define SER_BUFFER_SIZE             32

#define SERIAL_1ST_HEADER           0x54
#define SERIAL_2ND_HEADER           0xFE

//* ===================================================
//* Serial operations
//* ===================================================
// #define EspSerial Serial2
#define DebugSerial Serial1
#define PASSTHROUGH_BYTE 17


int gblSerialState = SER_WAITING_FOR_1ST_HEADER;
volatile bool gblUseFirstCmdBuffer = true;
int gblSerialCmdCounter = 0;
int gblSerialCmdLength = 0;
uint8_t gblSerialCmdChecksum = 0;
bool gblNewCmdReady = false;
int gblSerialIdleCounter = 0;

extern "C"
{
    // static ring_buffer usart1_rb;
    // static ring_buffer usart1_wb;
    void __irq_usart1(void);
    // static usart_dev usart1;
}