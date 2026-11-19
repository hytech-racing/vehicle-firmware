#include <Arduino.h>
#include "CANFDInterface.h"
#include "ht_can.h"
#include <array>

FlexCAN_T4FD<CAN3> CAN_FD;

const uint32_t CANFD_MSG_SIZE = sizeof(CANFD_message_t);

static uint32_t receiverCounter = 0;
static uint32_t lastReceiverCount = 0;

static uint32_t interval = 1;
unsigned long previousMillis2 = 0;

static uint32_t lastSeen1 = 0;
static uint32_t lastSeen2 = 0;
static uint32_t lastSeen3 = 0;
static uint32_t lastSeen4 = 0;

static uint8_t counter1 = 0;
static uint32_t counter2 = 0;
static uint32_t counter3 = 0;
static uint32_t counter4 = 0;

static uint32_t totalLost = 0;

static uint32_t totalWritesSuccess = 0;
static uint32_t totalWritesFail = 0;

unsigned long previousMillis = 0;

bool first_frame1 = true;
bool first_frame2 = true;
bool first_frame3 = true;
bool first_frame4 = true;

using CANFDRXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)16, CANFD_MSG_SIZE>;
using CANFDTXBuffer_t = Circular_Buffer<uint8_t, (uint32_t)128, CANFD_MSG_SIZE>;
CANFDTXBuffer_t canfd_tx_buffer;
CANFDRXBuffer_t canfd_rx_buffer;

uint32_t deadObject; 

void check_function( uint32_t &interfaces, const CANFD_message_t &msg, unsigned long curr_millis, CANInterfaceType_e interface_type)
{
    if (msg.id == 0x123) // test_msg1
    {
        if (msg.len == 1 && msg.edl == 1 && msg.brs == 1) 
        {
  
            receiverCounter++;

            if (!first_frame1 && msg.buf[0] != uint8_t(lastSeen1 + 1))
                {
                    if (msg.buf[0] > lastSeen1)
                    {
                        uint8_t lost = uint8_t(msg.buf[0] - lastSeen1 - 1);
                        totalLost += lost;
                        Serial.println("Error: Counter did not increment correctly!");
                        Serial.println("Frames lost: " + String(lost));
                    }
                    else
                    {
                        Serial.println("Counter reset detected (sender restarted?)");
                    }
                }
            lastSeen1 = msg.buf[0];
            first_frame1 = false;
        }
        else
        {
            Serial.print("Received test_msg1 with unexpected length or flags: ");
            Serial.print("LEN: "); Serial.print(msg.len);
            Serial.print("  EDL: "); Serial.print(msg.edl);
            Serial.print("  BRS: "); Serial.println(msg.brs);

        }
    }
    else if (msg.id == 0x456) // test_msg2
    {
        if (msg.len == 12 && msg.edl == 1 && msg.brs == 1) 
        {
  
            receiverCounter++;

            if (msg.buf[4] == 0xBB && msg.buf[11] == 0xBB) {
                uint32_t got;
                memcpy(&got, &msg.buf[0], 4);   
                
                if (!first_frame2 && got != lastSeen2 + 1)
                {
                    if (got > lastSeen2)
                    {
                        uint32_t lost = got - lastSeen2 - 1;
                        totalLost += lost;
                        Serial.println("Error: Counter did not increment correctly!");
                        Serial.println("Frames lost: " + String(lost));
                    }
                    else
                    {
                        Serial.println("Counter reset detected (sender restarted?)");
                    }
                }
                lastSeen2 = got;
                first_frame2 = false;
            }
            else
            {
                Serial.println("test message 2 data is wrong; buf[4] and buf[11] should be 0xBB");
                Serial.println("buf[4]: " + String(msg.buf[4], HEX) + " buf[11]: " + String(msg.buf[11], HEX));
            }
        }
        else
        {
            Serial.print("Received test_msg2 with unexpected length or flags: ");
            Serial.print("LEN: "); Serial.print(msg.len);
            Serial.print("  EDL: "); Serial.print(msg.edl);
            Serial.print("  BRS: "); Serial.println(msg.brs);

        }
    }
    else if (msg.id == 0x789) // test_msg3
    {
        if (msg.len == 20 && msg.edl == 1 && msg.brs == 0) 
        {

            receiverCounter++;

            if (msg.buf[4] == 0xBB && msg.buf[19] == 0xBB) {
                uint32_t got;
                memcpy(&got, &msg.buf[0], 4);

                if (!first_frame3 && got != lastSeen3 + 1)
                {
                    if (got > lastSeen3)
                    {
                        uint32_t lost = got - lastSeen3 - 1;
                        totalLost += lost;
                        Serial.println("Error: Counter did not increment correctly!");
                        Serial.println("Frames lost: " + String(lost));
                    }
                    else
                    {
                        Serial.println("Counter reset detected (sender restarted?)");
                    }
                }
                lastSeen3 = got;
                first_frame3 = false;
            }
            else
            {
                Serial.println("test message 3 data is wrong; buf[4] and buf[19] should be 0xBB");
                Serial.println("buf[4]: " + String(msg.buf[4], HEX) + " buf[19]: " + String(msg.buf[19], HEX));
            }
        }
        else
        {
            Serial.print("Received test_msg3 with unexpected length or flags: ");
            Serial.print("LEN: "); Serial.print(msg.len);
            Serial.print("  EDL: "); Serial.print(msg.edl);
            Serial.print("  BRS: "); Serial.println(msg.brs);

        }
    }
    else if (msg.id == 0x7FF) // test_msg4
    {
        if (msg.len == 8 && msg.edl == 0 && msg.brs == 0) 
        {

            receiverCounter++;

            if (msg.buf[4] == 0xBB && msg.buf[7] == 0xBB) {
                uint32_t got;
                memcpy(&got, &msg.buf[0], 4);

                if (!first_frame4 && got != lastSeen4 + 1)
                {
                    if (got > lastSeen4)
                    {
                        uint32_t lost = got - lastSeen4 - 1;
                        totalLost += lost;
                        Serial.println("Error: Counter did not increment correctly!");
                        Serial.println("Frames lost: " + String(lost));
                    }
                    else
                    {
                        Serial.println("Counter reset detected (sender restarted?)");
                    }
                }
                lastSeen4 = got;
                first_frame4 = false;
            }
            else
            {
                Serial.println("test message 4 data is wrong; buf[4] and buf[7] should be 0xBB");
                Serial.println("buf[4]: " + String(msg.buf[4], HEX) + " buf[7]: " + String(msg.buf[7], HEX));
            }
        }
        else
        {
            Serial.print("Received test_msg4 with unexpected length or flags: ");
            Serial.print("LEN: "); Serial.println(msg.len);
            Serial.print("  EDL: "); Serial.println(msg.edl);
            Serial.print("  BRS: "); Serial.println(msg.brs);

        }
    }
    else // sumting wong; received id was never sent
    {
        Serial.print("Received unexpected CAN FD message with ID: "); Serial.println(msg.id, HEX); 
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
}

void loop()
{

    unsigned long currentMillis = millis();
    #if defined(SENDER)
    if (currentMillis - previousMillis2 >= interval) {

        previousMillis2 = currentMillis;
        
        CANFD_message_t test_msg1;
        test_msg1.id = 0x123;
        test_msg1.len = 1;
        test_msg1.buf[0] = counter1;
        test_msg1.brs = 1; 
        test_msg1.edl = 1; 

        CANFD_message_t test_msg2;
        test_msg2.id = 0x456;
        test_msg2.len = 12;
        memcpy(&test_msg2.buf[0], &counter2, 4); 
        memset(&test_msg2.buf[4], 0xBB, test_msg2.len - 4); 
        test_msg2.brs = 1;
        test_msg2.edl = 1;

        CANFD_message_t test_msg3;
        test_msg3.id = 0x789;
        test_msg3.len = 20;
        memcpy(&test_msg3.buf[0], &counter3, 4); 
        memset(&test_msg3.buf[4], 0xBB, test_msg3.len - 4); 
        test_msg3.brs = 0; // data phase baudrate will be nominal speed
        test_msg3.edl = 1;

        CANFD_message_t test_msg4;
        test_msg4.id = 0x7FF;
        test_msg4.len = 8; // to enforce classical CAN
        memcpy(&test_msg4.buf[0], &counter4, 4); 
        memset(&test_msg4.buf[4], 0xBB, test_msg4.len - 4); 
        test_msg4.brs = 0; // to enforce classical CAN
        test_msg4.edl = 0; // to enforce classical CAN

        uint8_t a = CAN_FD.write(test_msg1);
        uint8_t b = CAN_FD.write(test_msg2);
        uint8_t c = CAN_FD.write(test_msg3);
        uint8_t d = CAN_FD.write(test_msg4);

        if (a != 0) {totalWritesSuccess++; counter1++;} else {totalWritesFail++;}
        if (b != 0) {totalWritesSuccess++; counter2++;} else {totalWritesFail++;}
        if (c != 0) {totalWritesSuccess++; counter3++;} else {totalWritesFail++;}
        if (d != 0) {totalWritesSuccess++; counter4++;} else {totalWritesFail++;}

    }
    #endif
    #if defined(RECEIVER) 
    process_ring_buffer_FD
        (
        canfd_rx_buffer, 
        deadObject, 
        millis(), 
        etl::delegate<void( uint32_t &, const CANFD_message_t &, unsigned long, CANInterfaceType_e)>::create<&check_function>(), 
        CANInterfaceType_e::TELEM
        );

    #endif

    if (millis() - previousMillis >= 1000) 
    {
        #ifdef SENDER
        Serial.println("Total writes successful: " + String(totalWritesSuccess));
        Serial.println("Total writes failed: " + String(totalWritesFail));
        #endif 
        #ifdef RECEIVER
        Serial.println(String((receiverCounter - lastReceiverCount)) + " frames received in last second");
        Serial.println("Total lost frames: " + String(totalLost));
        previousMillis = millis();
        lastReceiverCount = receiverCounter;
        #endif 
        previousMillis = currentMillis;
    }
}