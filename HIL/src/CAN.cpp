#include <Arduino.h>
#include "CANInterface.h"

/**
 * @brief Unified CAN test harness. Replaces the three separate test files
 *        previously used on ACU, VCF, and VCR.
 *
 * Board Topology:
 *   ACU:                         CAN2 = CCU,   CAN3 = EM
 *   DASHBOARD:     TELEM?
 *   CCU:           CAN1 = ACU                  CAN3 = CHARGER
 *   VCF:           CAN1 = TELEM, CAN2 = FAUX
 *   VCR:           CAN1 = TELEM, CAN2 = RAUX,  CAN3 = INVERTER
 *
 * NOTE: TELEM (CAN1) is a SHARED bus between VCF and VCR. This harness can
 *       test having more than one transmitting node on the same physical
 *       wire. To make that possible, every outgoing frame is tagged with a
 *       board ID, and stats are tracked per-sender rather than as one
 *       aggregate per bus.
 *
 * NOTE: To use this file, set THIS_BOARD + the ENABLE_CANx / baudrate / id
 *       config below to match whichever board you're flashing, then open
 *       the serial monitor and type 'h' for the list of test commands.
 *
 * What is checked?
 *  - frame loss, per sending board    (gaps in an embedded sequence number)
 *  - reordering, per sending board    (sequence number goes backwards)
 *  - payload corruption               (each data byte is derived from the sequence number)
 *  - inter-arrival jitter, per sender (min/max gap between received frames)
 *  - variable DLC (3-8 bytes) and extended (29-bit) IDs
 *  - burst/stress behavior            (N frames back-to-back, no delay)
 *  - arbitration priority             (lower CAN ID should win contention -> lower latency)
*/


// ---------------------------------------------------------------------
//                        Board Identification
// ---------------------------------------------------------------------

/**
 * NOTE: BOARD_UNKNOWN must stay before NUM_BOARDS. It is a valid slot used
 *       for frames with a corrupted/foreign board-id byte, so it needs to
 *       be a real, in-bounds array index rather than sitting past the end
 *       of the from[] array.
 *
 * NOTE: Every value here is prefixed with BOARD_ specifically so it can't
 *       collide with CANInterfaceType_e's bare ACU/CCU/etc. names.
*/
enum BoardId : uint8_t
{
    BOARD_ACU,
    BOARD_DASHBOARD,
    BOARD_CCU,
    BOARD_VCF,
    BOARD_VCR,
    BOARD_UNKNOWN,
    NUM_BOARDS
};

#define THIS_BOARD BOARD_ACU // <-- change per board flashed: BOARD_ACU, BOARD_CCU, etc.

/**
 * @brief Translates a BoardId byte into a human-readable name for Serial output.
 *
 * NOTE: Using a const char* here since a string is just an array of char.
 *       Could use Arduino's String class.
*/
const char* board_name(uint8_t id)
{
    switch (id)
    {
        case BOARD_ACU:
        {
            return "ACU";
        }
        case BOARD_DASHBOARD:
        {
            return "DASHBOARD";
        }
        case BOARD_CCU:
        {
            return "CCU";
        }
        case BOARD_VCF:
        {
            return "VCF";
        }
        case BOARD_VCR:
        {
            return "VCR";
        }
        default:
        {
            return "UNKNOWN";
        }
    }
}



// ---------------------------------------------------------------------
//                       Bus Enable / Config
// ---------------------------------------------------------------------

/**
 * NOTE: Only enable the buses that this board actually uses / that you
 *       want to test. Double-check the baudrate before flashing!
 *
 * NOTE: #define here means the preprocessor replaces, e.g.,
 *       ENABLE_CAN1 with true/false everywhere it appears
*/
#define ENABLE_CAN1   false
#define ENABLE_CAN2   true
#define ENABLE_CAN3   true

static const uint32_t CAN1_BAUDRATE = 500000;
static const uint32_t CAN2_BAUDRATE = 1000000;
static const uint32_t CAN3_BAUDRATE = 500000;

// These are Arbitrary CAN identifiers for TEST HARNESS ONLY. These don't need
// to be unique or match real DBC message IDs unless you're deliberately
// testing arbitration/priority behavior between multiple senders.
static const uint32_t CAN1_TX_ID = 0x11;
static const uint32_t CAN2_TX_ID = 0x11;
static const uint32_t CAN3_TX_ID = 0x11;

static const uint32_t TX_PERIOD_MS     = 10;   // matches the original delay(10)
static const uint32_t REPORT_PERIOD_MS = 1000; // periodic stats line instead of per-frame spam

// FlexCAN bus objects - only instantiated for enabled buses.
#if ENABLE_CAN1
FlexCAN_T4<CAN1> CAN_bus_1;
#endif
#if ENABLE_CAN2
FlexCAN_T4<CAN2> CAN_bus_2;
#endif
#if ENABLE_CAN3
FlexCAN_T4<CAN3> CAN_bus_3;
#endif



// ---------------------------------------------------------------------
//                          Test-Frame Format
// ---------------------------------------------------------------------

/**
 * NOTE: A classic CAN message has a max payload of 8 bytes, so buf is a
 *       fixed-size array of 8 uint8_t.
 *
 * buf[0]    = Board id (who sent this frame)
 * buf[1..2] = 16-bit sequence number (little endian). Used to detect
 *             dropped/reordered frames - increments once per frame sent.
 * buf[3..7] = Corruption-checking pattern, derived from the sequence
 *             number: buf[i] = (uint8_t)(seq + i + 1). Works like a
 *             checksum since the receiver independently recomputes each
 *             byte's expected value from the sequence number
 *
 * NOTE: Minimum usable length is 3 bytes (board id + seq); shorter frames
 *       are still counted as received but skipped for loss/corruption
 *       tracking.
*/



// ---------------------------------------------------------------------
//                          Stats Structs
// ---------------------------------------------------------------------

/**
 * @brief Receive statistics per-sender. One instance exists for every
 *        possible BoardId, so a shared bus with multiple transmitting
 *        boards gets independent loss/corruption/timing tracking per
 *        sender instead of one number that mixes everyone's traffic
 *        together.
*/
struct RXStats_s
{
    uint32_t rx_count = 0;          // number of frames received from a sender
    uint32_t rx_dropped = 0;        // number of frames dropped; detected by gaps in sequence numbers
    uint32_t rx_corrupt = 0;        // number of frames where the payload didn't match expected pattern
    uint32_t rx_out_of_order = 0;   // number of frames that arrived out of order

    bool     has_last_sequence = false;
    uint16_t last_sequence_num = 0;

    uint32_t last_rx_ms = 0;
    uint32_t min_gap_ms = 0xFFFFFFFF;
    uint32_t max_gap_ms = 0;
};

/**
 * @brief Statistics for one CAN bus: this board's own transmit count, plus
 *        a per-sender lookup table (from[]) of everything received on this
 *        bus, indexed by BoardId.
*/
struct BusStats_s
{
    const char *name;      // bus label for Serial output, e.g. "CAN1", "CAN2", "CAN3"
    uint32_t tx_count = 0; // number of frames THIS_BOARD has transmitted on this bus

    // Toggle to print every received frame. Yet to be tested but this might just flood the serial monitor
    // and brick everything if left on during a burst test.
    bool verbose = false;

    RXStats_s from[NUM_BOARDS]; // receive stats, indexed by sending BoardId
};
BusStats_s CAN_1_stats{"CAN1"};
BusStats_s CAN_2_stats{"CAN2"};
BusStats_s CAN_3_stats{"CAN3"};

/**
 * @brief Per-CAN-ID timing stats used only by the arbitration test.
 *        Tracks inter-arrival gap in microseconds instead of ms.
*/
struct ArbIdStats_s
{
    uint32_t count = 0;
    uint32_t last_rx_us = 0;
    uint32_t min_gap_us = 0xFFFFFFFF;
    uint32_t max_gap_us = 0;
    uint64_t sum_gap_us = 0; // lets us report an average, not just min/max
};

/**
 * @brief Arbitration test state for one bus. Kept separate per-bus so a
 *        test running on CAN2 doesn't disturb CAN1's/CAN3's state.
*/
struct ArbBusState_s
{
    ArbIdStats_s high_priority_stats;  // stats for the high-priority (low numeric ID) test frame
    ArbIdStats_s low_priority_stats;   // stats for the low-priority (high numeric ID) test frame
    volatile bool is_arb_test_running = false;
    uint16_t frames_sent = 0;
};
ArbBusState_s CAN_1_arb;
ArbBusState_s CAN_2_arb;
ArbBusState_s CAN_3_arb;

// Global test-mode toggles (shared across buses), changed live via serial
// commands - see print_help().
bool     g_variable_len_mode = false; // cycle DLC 3..8 instead of fixed len = 3
bool     g_extended_id_mode  = false; // send frames with extended (29-bit) IDs instead of standard (11-bit)
uint16_t g_sequence_count    = 0;     // shared outgoing seq counter for this board, incremented per frame sent (regardless of which bus it goes out on)



// ---------------------------------------------------------------------
//                      Arbitration Test Config
// ---------------------------------------------------------------------

// Lower CAN ID = higher priority on a real bus. These are dedicated IDs
// used only during the arbitration test, kept separate from normal
// per-board traffic so they don't get mixed into the regular loss/
// corruption stats tracked by check_payload().
static const uint32_t ARB_HIGH_PRIORITY_ID = 0x001; // should win arbitration
static const uint32_t ARB_LOW_PRIORITY_ID  = 0x7FE; // should lose / back off under contention
static const uint32_t ARB_TRIGGER_ID       = 0x7FF; // "start the test now" sync frame
static const uint16_t ARB_BURST_COUNT      = 200;   // frame pairs sent per test run



// ---------------------------------------------------------------------
//                    Frame Building / Checking
// ---------------------------------------------------------------------

/**
 * @brief Builds one outgoing test frame: sets the CAN id/DLC/extended flag,
 *        and fills the payload per the buf[] layout documented above.
 *
 * @param msg          the CAN_message_t to fill in (FlexCAN's message struct)
 * @param can_id       arbitration ID to send this frame with (e.g. CAN1_TX_ID)
 * @param sequence_num this frame's sequence number; caller increments it
*/
void fill_test_frame(CAN_message_t &msg, uint32_t can_id, uint16_t sequence_num)
{
    msg.id = can_id;
    msg.flags.extended = g_extended_id_mode ? 1 : 0;
    msg.len = g_variable_len_mode ? (uint8_t)(3 + (sequence_num % 6)) : 3; // 3..8 when variable length mode is on

    msg.buf[0] = THIS_BOARD;
    msg.buf[1] = sequence_num & 0xFF;        // sequence number, low byte
    msg.buf[2] = (sequence_num >> 8) & 0xFF; // sequence number, high byte

    // Sequence-derived checksum pattern - see buf[3..7] note above.
    for (uint8_t i = 3; i < 8; i++)
    {
        msg.buf[i] = (uint8_t)(sequence_num + i + 1);
    }
}

/**
 * @brief Post-processes a received frame: figures out which board sent it,
 *        updates that sender's stats (received count, corruption check,
 *        loss/reorder detection, jitter timing).
*/
void check_payload(const CAN_message_t &msg, BusStats_s &bus_stats)
{
    if (msg.len < 3)
    {
        return; // Can't recover board id + sequence number from anything shorter than 3 bytes.
    }

    uint8_t sender = msg.buf[0];
    if (sender >= NUM_BOARDS)
    {
        sender = BOARD_UNKNOWN;
    }

    // Reference (not a copy!) to this sender's stats slot, so every update
    // below modifies the real entry inside bus_stats.from[sender].
    RXStats_s &sender_stats = bus_stats.from[sender];

    uint16_t sequence_num = msg.buf[1] | (msg.buf[2] << 8); // rebuild the full 16-bit sequence

    bool is_data_corrupt = false;
    for (uint8_t i = 3; i < msg.len; i++)
    {
        if (msg.buf[i] != (uint8_t)(sequence_num + i + 1))
        {
            is_data_corrupt = true;
            break;
        }
    }

    if (is_data_corrupt)
    {
        sender_stats.rx_corrupt++;
    }

    if (sender_stats.has_last_sequence)
    {
        uint16_t expected = (uint16_t)(sender_stats.last_sequence_num + 1);

        if (sequence_num != sender_stats.last_sequence_num && sequence_num != expected)
        {
            // A small forward_gap means frames were skipped (dropped)
            // A huge forward_gap (>= 0x8000) means the sequence number actually went backwards (wraparound),
            // treated as reordering rather than loss. Not sure if this is even possible, just added the check.
            uint16_t forward_gap = (uint16_t)(sequence_num - expected);

            if (forward_gap < 0x8000)
            {
                sender_stats.rx_dropped += forward_gap;
            }
            else
            {
                sender_stats.rx_out_of_order++;
            }
        }
    }
    sender_stats.last_sequence_num = sequence_num;
    sender_stats.has_last_sequence = true;

    // Jitter Tracking: how long since we last heard from this sender.
    uint32_t now = millis();
    if (sender_stats.last_rx_ms != 0)
    {
        uint32_t elapsed_time = now - sender_stats.last_rx_ms;

        if (elapsed_time < sender_stats.min_gap_ms)
        {
            sender_stats.min_gap_ms = elapsed_time;
        }
        if (elapsed_time > sender_stats.max_gap_ms)
        {
            sender_stats.max_gap_ms = elapsed_time;
        }

        sender_stats.last_rx_ms = now;
        sender_stats.rx_count++;
    }
}

/**
 * @brief Handles arbitration-test frames on one bus: a trigger frame
 *        (re)starts the test and resets stats, while HIGH/LOW priority
 *        frames update their respective inter-arrival timing stats.
 *
 * NOTE: This is intentionally separate from check_payload()!
 *       Arbitration test frames aren't board-tagged/sequence-numbered
 *       the same way and they have different stats structs.
*/
void check_arb_frame(const CAN_message_t &msg, ArbBusState_s &arb)
{
    if (msg.id == ARB_TRIGGER_ID)
    {
        // Any board hearing the trigger joins the test
        arb.is_arb_test_running = true;
        arb.frames_sent = 0;
        arb.high_priority_stats = ArbIdStats_s{};
        arb.low_priority_stats = ArbIdStats_s{};
        return;
    }

    ArbIdStats_s *stats = nullptr;
    if (msg.id == ARB_HIGH_PRIORITY_ID)
    {
        stats = &arb.high_priority_stats;
    }
    else if (msg.id == ARB_LOW_PRIORITY_ID)
    {
        stats = &arb.low_priority_stats;
    }
    else
    {
        return; // not an arbitration-test frame
    }

    uint32_t now = micros();
    /// NOTE: For those new to pointer notaton, '->' combines these two steps: (*ptr).member
    if (stats->last_rx_us != 0)
    {
        uint32_t gap = now - stats->last_rx_us;
        stats->sum_gap_us += gap;

        if (gap < stats->min_gap_us)
        {
            stats->min_gap_us = gap;
        }
        if (gap > stats->max_gap_us)
        {
            stats->max_gap_us = gap;
        }

        stats->last_rx_us = now;
        stats->count++;
    }
}



// ---------------------------------------------------------------------
//                      Debugging and Prints
// ---------------------------------------------------------------------

/**
 * @brief Prints one received frame's raw contents to Serial. Only called
 *        when a bus's verbose flag is on.
*/
void print_frame(const char *bus_name, const CAN_message_t &msg)
{
    Serial.print(bus_name); Serial.print(" receiving  ");
    Serial.print("MB:"); Serial.print(msg.mb);
    Serial.print(" ID:0x"); Serial.print(msg.id, HEX);
    Serial.print(" EXTENDED:"); Serial.print(msg.flags.extended);
    Serial.print(" LEN:"); Serial.print(msg.len);
    Serial.print(" DATA:");
    for (uint8_t i = 0; i < msg.len; i++)
    {
        Serial.print(msg.buf[i], HEX); Serial.print(' ');
    }
    Serial.print(" TIMESTAMP:"); Serial.println(msg.timestamp);
}

/**
 * @brief Prints a summary line for a bus's own tx_count, followed by one
 *        line per sender actually heard from on that bus (senders with no
 *        traffic yet are skipped).
*/
void print_report(BusStats_s &bus_stats)
{
    Serial.print("[report] "); Serial.print(bus_stats.name);
    Serial.print("  tx="); Serial.println(bus_stats.tx_count);

    for (uint8_t board_id = 0; board_id < NUM_BOARDS; board_id++)
    {
        RXStats_s &sender_stats = bus_stats.from[board_id];
        if (sender_stats.rx_count == 0 && !sender_stats.has_last_sequence)
        {
            continue; // nothing heard from this sender yet
        }
        Serial.print("    from "); Serial.print(board_name(board_id));
        Serial.print(": rx count ="); Serial.print(sender_stats.rx_count);
        Serial.print(" dropped =");   Serial.print(sender_stats.rx_dropped);
        Serial.print(" corrupt =");   Serial.print(sender_stats.rx_corrupt);
        Serial.print(" reordered ="); Serial.print(sender_stats.rx_out_of_order);
        Serial.print(" gap(min/max ms)="); Serial.print(sender_stats.min_gap_ms == 0xFFFFFFFF ? 0 : sender_stats.min_gap_ms);
        Serial.print('/'); Serial.println(sender_stats.max_gap_ms);
    }
}

/**
 * @brief Prints HIGH vs. LOW priority timing stats for one bus's
 *        arbitration test.
 * NOTE: A widening gap between the two (LOW's avg/max
 *       growing worse than HIGH's) is the sign of arbitration
 *       actually deciding who gets the bus under contention.
*/
void print_arb_report(const char *bus_name, ArbBusState_s &arb)
{
    Serial.print("[Arbitration Report] "); Serial.println(bus_name);

    Serial.print("  HIGH (0x1):   count="); Serial.print(arb.high_priority_stats.count);
    Serial.print(" min/avg/max us=");
    Serial.print(arb.high_priority_stats.min_gap_us == 0xFFFFFFFF ? 0 : arb.high_priority_stats.min_gap_us);
    Serial.print('/'); Serial.print(arb.high_priority_stats.count ? arb.high_priority_stats.sum_gap_us / arb.high_priority_stats.count : 0);
    Serial.print('/'); Serial.println(arb.high_priority_stats.max_gap_us);

    Serial.print("  LOW  (0x7FE): count="); Serial.print(arb.low_priority_stats.count);
    Serial.print(" min/avg/max us=");
    Serial.print(arb.low_priority_stats.min_gap_us == 0xFFFFFFFF ? 0 : arb.low_priority_stats.min_gap_us);
    Serial.print('/'); Serial.print(arb.low_priority_stats.count ? arb.low_priority_stats.sum_gap_us / arb.low_priority_stats.count : 0);
    Serial.print('/'); Serial.println(arb.low_priority_stats.max_gap_us);
}



// ---------------------------------------------------------------------
//                           Helper Functions
// ---------------------------------------------------------------------

/**
 * @brief Clears a bus's tx_count and every sender's receive stats back to
 *        their initial values.
*/
void reset_stats(BusStats_s &bus_stats)
{
    bus_stats.tx_count = 0;
    for (uint8_t board_id = 0; board_id < NUM_BOARDS; board_id++)
    {
        bus_stats.from[board_id] = RXStats_s{};
    }
}



// ---------------------------------------------------------------------
//            Receive callbacks (registered per-bus in setup)
// ---------------------------------------------------------------------

// Each callback routes the incoming frame to both check_payload() (normal
// loss/corruption/jitter stats) and check_arb_frame() (arbitration test
// timing), then optionally prints the raw frame if verbose mode is on.
#if ENABLE_CAN1
void on_recv_CAN_1(const CAN_message_t &msg)
{
    check_payload(msg, CAN_1_stats);
    check_arb_frame(msg, CAN_1_arb);
    if (CAN_1_stats.verbose)
    {
        print_frame("CAN1", msg);
    }
}
#endif
#if ENABLE_CAN2
void on_recv_CAN_2(const CAN_message_t &msg)
{
    check_payload(msg, CAN_2_stats);
    check_arb_frame(msg, CAN_2_arb);
    if (CAN_2_stats.verbose)
    {
        print_frame("CAN2", msg);
    }
}
#endif
#if ENABLE_CAN3
void on_recv_CAN_3(const CAN_message_t &msg)
{
    check_payload(msg, CAN_3_stats);
    check_arb_frame(msg, CAN_3_arb);
    if (CAN_3_stats.verbose)
    {
        print_frame("CAN3", msg);
    }
}
#endif



// ---------------------------------------------------------------------
//                          Burst / Stress test
// ---------------------------------------------------------------------

/**
 * @brief Sends `count` frames back-to-back on one bus with no delay
 *        between them, to stress-test mailbox/buffer handling and bus
 *        contention. Templated on bus type since FlexCAN_T4<CAN1>,
 *        <CAN2>, and <CAN3> are distinct C++ types despite sharing an
 *        interface.
*/
template <typename Bus_t>
void burst_test(Bus_t &bus, uint32_t can_id, BusStats_s &bus_stats, uint16_t count)
{
    Serial.print(" Starting Burst Test For: "); Serial.println(bus_stats.name);
    Serial.print(" Sending "); Serial.print(count); Serial.println(" frames back-to-back...");

    uint32_t start = micros();
    for (uint16_t i = 0; i < count; i++)
    {
        CAN_message_t msg;
        fill_test_frame(msg, can_id, g_sequence_count++);
        bus.write(msg);
        bus_stats.tx_count++;
    }
    uint32_t elapsed = micros() - start;

    Serial.print(" Burst test done in "); Serial.print(elapsed); Serial.println(" us - check the rx/dropped counters on the receiving end");
}



// ---------------------------------------------------------------------
//                          Arbitration test
// ---------------------------------------------------------------------

/**
 * @brief Sends the trigger frame that (re)starts the arbitration test on a
 *        given bus, and arms this board's own test loop immediately (it
 *        doesn't have to wait to hear its own trigger echoed back).
*/
template <typename Bus_t>
void arb_test_trigger(Bus_t &bus, ArbBusState_s &arb, const char *bus_name)
{
    CAN_message_t trigger{};
    trigger.id  = ARB_TRIGGER_ID;
    trigger.len = 0;
    bus.write(trigger);

    arb.is_arb_test_running = true;
    arb.frames_sent = 0;
    arb.high_priority_stats = ArbIdStats_s{};
    arb.low_priority_stats = ArbIdStats_s{};

    Serial.print("[arbitration test started on "); Serial.print(bus_name); Serial.println("]");
}

/**
 * @brief Called every loop() iteration, per enabled bus. While a test is
 *        running, sends one HIGH + one LOW priority frame back-to-back
 *        (no delay) each iteration, forcing contention, until
 *        ARB_BURST_COUNT pairs have been sent, then stops itself.
*/
template <typename Bus_t>
void arb_test_service(Bus_t &bus, ArbBusState_s &arb)
{
    if (!arb.is_arb_test_running || arb.frames_sent >= ARB_BURST_COUNT)
    {
        return;
    }

    CAN_message_t high_priority_msg{};
    high_priority_msg.id = ARB_HIGH_PRIORITY_ID;
    high_priority_msg.len = 1;
    high_priority_msg.buf[0] = (uint8_t)arb.frames_sent;

    CAN_message_t low_priority_msg{};
    low_priority_msg.id = ARB_LOW_PRIORITY_ID;
    low_priority_msg.len = 1;
    low_priority_msg.buf[0] = (uint8_t)arb.frames_sent;

    bus.write(high_priority_msg);
    bus.write(low_priority_msg); // sent immediately after high priority message. This is what forces contention
    arb.frames_sent++;

    if (arb.frames_sent >= ARB_BURST_COUNT)
    {
        arb.is_arb_test_running = false;
    }
}



// ---------------------------------------------------------------------
//                      Serial Command Interface
// ---------------------------------------------------------------------

/**
 * @brief Prints which board this harness identifies as, plus the list of
 *        available serial commands.
*/
void print_help()
{
    Serial.print("This board is: "); Serial.println(board_name(THIS_BOARD));
    Serial.println(F(
        "Commands:\n"
        "  s     - print stats for all enabled buses now\n"
        "  r     - reset all stats\n"
        "  v     - toggle variable-length payload mode (cycles DLC 3-8)\n"
        "  x     - toggle extended (29-bit) ID mode\n"
        "  1/2/3 - fire a 100-frame burst test on CAN1/CAN2/CAN3 (if enabled)\n"
        "  a/b/c - fire an arbitration test on CAN1/CAN2/CAN3 (if enabled)\n"
        "  p     - toggle verbose per-frame printing\n"
        "  h     - show this help\n"
    ));
}

/**
 * @brief Reads and dispatches a single character command from Serial, if
 *        one is available. Non-blocking: does nothing if no input waiting.
*/
void handle_serial_commands()
{
    if (!Serial.available())
    {
        return;
    }

    char c = Serial.read();
    switch (c)
    {
        case 's': // print current stats for every enabled bus
        {
            #if ENABLE_CAN1
                print_report(CAN_1_stats);
                print_arb_report("CAN1", CAN_1_arb);
            #endif
            #if ENABLE_CAN2
                print_report(CAN_2_stats);
                print_arb_report("CAN2", CAN_2_arb);
            #endif
            #if ENABLE_CAN3
                print_report(CAN_3_stats);
                print_arb_report("CAN3", CAN_3_arb);
            #endif
            break;
        }
        case 'r': // reset stats for every enabled bus
        {
            #if ENABLE_CAN1
                reset_stats(CAN_1_stats);
            #endif
            #if ENABLE_CAN2
                reset_stats(CAN_2_stats);
            #endif
            #if ENABLE_CAN3
                reset_stats(CAN_3_stats);
            #endif
            Serial.println("[stats reset]");
            break;
        }
        case 'v': // toggle cycling DLC 3-8 vs. always sending a fixed 3-byte frame
        {
            g_variable_len_mode = !g_variable_len_mode;
            Serial.print("[variable length mode] ");
            Serial.println(g_variable_len_mode ? "ON" : "OFF");
            break;
        }
        case 'x': // toggle sending extended (29-bit) vs. standard (11-bit) CAN IDs
        {
            g_extended_id_mode = !g_extended_id_mode;
            Serial.print("[extended id mode] ");
            Serial.println(g_extended_id_mode ? "ON" : "OFF");
            break;
        }
        case 'p': // toggle printing every received frame (per enabled bus)
        {
            #if ENABLE_CAN1
                CAN_1_stats.verbose = !CAN_1_stats.verbose;
            #endif
            #if ENABLE_CAN2
                CAN_2_stats.verbose = !CAN_2_stats.verbose;
            #endif
            #if ENABLE_CAN3
                CAN_3_stats.verbose = !CAN_3_stats.verbose;
            #endif
            Serial.println("[verbose toggled]");
            break;
        }

        // Fire a 100-frame burst test on whichever bus is enabled and requested.
        #if ENABLE_CAN1
            case '1':
            {
                burst_test(CAN_bus_1, CAN1_TX_ID, CAN_1_stats, 100);
                break;
            }
        #endif
        #if ENABLE_CAN2
            case '2':
            {
                burst_test(CAN_bus_2, CAN2_TX_ID, CAN_2_stats, 100);
                break;
            }
        #endif
        #if ENABLE_CAN3
            case '3':
            {
                burst_test(CAN_bus_3, CAN3_TX_ID, CAN_3_stats, 100);
                break;
            }
        #endif

        // Fire an arbitration test on whichever bus is enabled and requested.
        #if ENABLE_CAN1
            case 'a':
            {
                arb_test_trigger(CAN_bus_1, CAN_1_arb, "CAN1");
                break;
            }
        #endif
        #if ENABLE_CAN2
            case 'b':
            {
                arb_test_trigger(CAN_bus_2, CAN_2_arb, "CAN2");
                break;
            }
        #endif
        #if ENABLE_CAN3
            case 'c':
            {
                arb_test_trigger(CAN_bus_3, CAN_3_arb, "CAN3");
                break;
            }
        #endif

        case 'h':
        default:
        {
            print_help();
            break;
        }
    }
}



// ---------------------------------------------------------------------
//                          Setup / Loop
// ---------------------------------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(400);

    // Initialize each enabled bus: sets baud rate and registers this bus's receive callback.
    #if ENABLE_CAN1
        handle_CAN_setup(CAN_bus_1, CAN1_BAUDRATE, &on_recv_CAN_1);
    #endif
    #if ENABLE_CAN2
        handle_CAN_setup(CAN_bus_2, CAN2_BAUDRATE, &on_recv_CAN_2);
    #endif
    #if ENABLE_CAN3
        handle_CAN_setup(CAN_bus_3, CAN3_BAUDRATE, &on_recv_CAN_3);
    #endif

    print_help();
}

void loop()
{
    handle_serial_commands();

    #if ENABLE_CAN1
        CAN_bus_1.events();
    #endif
    #if ENABLE_CAN2
        CAN_bus_2.events();
    #endif
    #if ENABLE_CAN3
        CAN_bus_3.events();
    #endif

    static uint32_t last_tx     = 0;
    static uint32_t last_report = 0;
    uint32_t now = millis();

    // Periodic send: one frame per enabled bus, every TX_PERIOD_MS, all
    // sharing the same sequence number so multi-bus outgoing traffic
    // stays together.
    if (now - last_tx >= TX_PERIOD_MS)
    {
        last_tx = now;
        uint16_t seq = g_sequence_count++;

        #if ENABLE_CAN1
            { CAN_message_t m; fill_test_frame(m, CAN1_TX_ID, seq); CAN_bus_1.write(m); CAN_1_stats.tx_count++; }
        #endif
        #if ENABLE_CAN2
            { CAN_message_t m; fill_test_frame(m, CAN2_TX_ID, seq); CAN_bus_2.write(m); CAN_2_stats.tx_count++; }
        #endif
        #if ENABLE_CAN3
            { CAN_message_t m; fill_test_frame(m, CAN3_TX_ID, seq); CAN_bus_3.write(m); CAN_3_stats.tx_count++; }
        #endif
    }

    #if ENABLE_CAN1
        arb_test_service(CAN_bus_1, CAN_1_arb);
    #endif
    #if ENABLE_CAN2
        arb_test_service(CAN_bus_2, CAN_2_arb);
    #endif
    #if ENABLE_CAN3
        arb_test_service(CAN_bus_3, CAN_3_arb);
    #endif

    // Periodic summary report, instead of printing every single frame.
    if (now - last_report >= REPORT_PERIOD_MS)
    {
        last_report = now;

        #if ENABLE_CAN1
            print_report(CAN_1_stats);
        #endif
        #if ENABLE_CAN2
            print_report(CAN_2_stats);
        #endif
        #if ENABLE_CAN3
            print_report(CAN_3_stats);
        #endif
    }
}