#include <Arduino.h>
#include "CANFDInterface.h"
#include "CANInterface.h"
#include "ht_can.h"
#include <array>

FlexCAN_T4FD<CAN3> CAN_FD;

const uint32_t DELAY = 10;
const uint32_t CANFD_MSG_SIZE = sizeof(CANFD_message_t);
const unsigned long interval = 500; // for delay, in micros 
unsigned long previousMicros = 0; // also for delay
unsigned long previousMillis2 = 0; // also for delay

static uint32_t counter = 0; 
static uint32_t lastCount = 0;
static uint32_t lastCount2 = 0;
static uint32_t receiverCounter = 0;
static uint32_t lastReceiverCount = 0;
static uint32_t lastReceiverCount2 = 0;
static uint32_t lastSeen = 0;
static uint32_t totalLost = 0;     
static uint32_t lastTotalLost = 0; 


bool first_frame = true;

static uint32_t test_writes_success = 0;
static uint32_t test_writes_fail = 0;

uint32_t id1 = 0xDEADBEEF;
uint32_t id2 = 0xDEADBEAD;
uint32_t id3 = 0xDEADCAFE;

uint32_t deadObject; 

using CANFDRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, CANFD_MSG_SIZE>;
using CANFDTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, CANFD_MSG_SIZE>;
CANFDTXBuffer_t canfd_tx_buffer;
CANFDRXBuffer_t canfd_rx_buffer;

void check_function( uint32_t &interfaces, const CANFD_message_t &msg, unsigned long curr_millis, CANInterfaceType_e interface_type)
{
    if (msg.id == FD_TEST_CANID) 
    {
        FD_TEST_t test_msg; 
        if (msg.len == 64 && msg.edl == 1 && msg.brs == 1) 
        {
            lastReceiverCount = receiverCounter;
            receiverCounter++;
            Unpack_FD_TEST_ht_can(&test_msg, msg.buf, msg.len);
            
        
            if (id1 == test_msg.fd_test_byte8 && id2 == test_msg.fd_test_byte32 && id3 == test_msg.fd_test_byte60) 
            {

                if (!first_frame && test_msg.fd_test_counter != lastSeen + 1)
                {
                    if (test_msg.fd_test_counter > lastSeen)
                    {
                        uint32_t lost = test_msg.fd_test_counter - lastSeen - 1;
                        totalLost += lost;
                        Serial.println("Error: Counter did not increment correctly!");
                        Serial.println("Frames lost: " + String(lost));
                    }
                    else
                    {
                        Serial.println("Counter reset detected (sender restarted?)");
                    }
                }
                lastSeen = test_msg.fd_test_counter;
                first_frame = false;
            } 
            else 
            {
                Serial.println("Received CAN FD message with unexpected byte IDs");
            }
        }
        else 
        {
            Serial.println("Received CAN FD message with unexpected length or flags");
        }
    }
    else
    {
        Serial.println("Received CAN FD message with unexpected ID (!= 0x7f2U)");
    }
    
    
}
       
void on_telem_can_receive(const CANFD_message_t &msg)
{
    std::array<uint8_t, CANFD_MSG_SIZE> buf;
    memmove(buf.data(), &msg, sizeof(msg));
    canfd_rx_buffer.push_back(buf.data(), sizeof(CANFD_message_t));
    
}

void setup() 
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}
    CANFD_timings_t config;
    config.clock = CLK_80MHz;       
    config.baudrate = 1000000;       
    config.baudrateFD = 5000000;    
    config.propdelay = 190;       
    config.bus_length = 1;       
    config.sample = 75;     

    bool setup_worked = handle_CANFD_setup(CAN_FD, config, &on_telem_can_receive);

    if (!setup_worked)
    {
        Serial.println("Failed to setup CAN FD");
        while (true)
        {
            delay(1000);
        }
    }
    #ifdef LOOPBACK
    Serial.println("Enabling loopback mode for CAN FD");
    FLEXCAN_FD_EnterFreezeMode();
    FLEXCANb_MCR(CAN3) &= ~(1UL << 17);
    FLEXCANb_CTRL1(CAN3) |=  (1UL << 12);
    FLEXCANb_FDCTRL(CAN3) &= ~(1UL << 15);   // clear TDCEN (bit 15): disable transmitter delay compensation (TEST)
    FLEXCAN_FD_ExitFreezeMode();
    #endif
}

void loop() 
{
    unsigned long currentMicros = micros();

    #if defined(SENDER) || defined(LOOPBACK)
    
    if (currentMicros - previousMicros >= interval) {
        previousMicros = currentMicros;

        FD_TEST_t test_msg;
        CANFD_message_t frame;

        test_msg.fd_test_counter = counter;
        test_msg.fd_test_byte8 = id1;
        test_msg.fd_test_byte32 = id2;
        test_msg.fd_test_byte60 = id3;

        frame.id = Pack_FD_TEST_ht_can(&test_msg, frame.buf, &frame.len, (uint8_t*)&frame.flags.extended);
        frame.brs = 1;
        frame.edl = 1;
        
        int canfd_write = CAN_FD.write(frame);

        if (canfd_write == 1) {
            test_writes_success++;
            lastCount = counter;
            counter++; 
        }
        else
        {
            test_writes_fail++;
        }
        // **** This is the TX bufffer code ****
        // **** NOT NEEDED unless you want to test the TX buffer functionality ****
        // CAN_util::enqueue_fd_msg( &test_msg, &Pack_FD_TEST_ht_can , canfd_tx_buffer);

        // // CANFD_message_t recvd_msg;
        // // uint8_t buf[sizeof(CANFD_message_t)];
        // // while (canfd_tx_buffer.available() )
        // // {
        // //     canfd_tx_buffer.peek_front(buf, sizeof(CANFD_message_t));
        // //     memmove(&recvd_msg, buf, sizeof(recvd_msg));
        // //     int canfd_write = CAN_FD.write(recvd_msg);
        // //     if (canfd_write == 1) {
        // //         canfd_tx_buffer.pop_front(buf, sizeof(CANFD_message_t));
        // //         test_writes_success++;
        // //     }
        // //     else
        // //     {
        // //         test_writes_fail++;
        // //         break; //exit loop if write fails
        // //     }
        // // }
    } 
    #endif
    #if defined(RECEIVER) || defined(LOOPBACK)
    process_ring_buffer_FD
        (
        canfd_rx_buffer, 
        deadObject, 
        millis(), 
        etl::delegate<void( uint32_t &, const CANFD_message_t &, unsigned long, CANInterfaceType_e)>::create<&check_function>(), 
        CANInterfaceType_e::TELEM
        );
    

    #endif

    if (millis() - previousMillis2 >= 1000) 
    {
        #ifdef SENDER

        Serial.println(String((counter - lastCount2)) + " frames sent in last second");
        Serial.println("Test writes success: " + String(test_writes_success) + ", Test writes fail: " + String(test_writes_fail));
        lastCount2 = counter;
        previousMillis2 = millis();
        #else
        Serial.println(String(receiverCounter - lastReceiverCount2) + " frames received in last second");
        Serial.println(String(totalLost - lastTotalLost) + " frames lost in last second (" + String(totalLost) + " total)");
        lastReceiverCount2 = receiverCounter;
        lastTotalLost = totalLost;
        previousMillis2 = millis();
        // test
        #endif
    }
    
    
}
