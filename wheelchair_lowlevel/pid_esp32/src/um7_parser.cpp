#include "um7_parser.h"
#include <string.h>
#include <math.h>

// UART
static HardwareSerial IMU(1); // UART1

// UM7 TX -> ESP32 RX, UM7 RX -> ESP32 TX
static const int RX_PIN = 38;
static const int TX_PIN = 39;

// Scale
static const float QUAT_SCALE = 29789.09091f;

// Packet Structure
struct Um7Packet {
  uint8_t address = 0;
  uint8_t packet = 0;
  uint8_t data_len = 0;
  uint8_t data[256] = {0};
  uint16_t checksum = 0;
};

// RX buffer
static uint8_t rx_buffer[512];
static size_t rx_len = 0;

// Latest raw states
static bool quat_valid = false;
static int16_t quat_a_raw = 0;
static int16_t quat_b_raw = 0;
static int16_t quat_c_raw = 0;
static int16_t quat_d_raw = 0;

static bool gyro_valid = false;
static float gyro_x_dps = 0.0f;
static float gyro_y_dps = 0.0f;
static float gyro_z_dps = 0.0f;

static bool accel_valid = false;
static float accel_x_mps2 = 0.0f;
static float accel_y_mps2 = 0.0f;
static float accel_z_mps2 = 0.0f;


// Output Sample
static ImuSample latest_sample = {0};
static bool sample_ready = false;


// Helpers
static inline bool packet_has_data(uint8_t packet) {
  return (packet >> 7) & 0x01;
}

static inline bool packet_is_batch(uint8_t packet) {
  return (packet >> 6) & 0x01;
}

static inline uint8_t packet_batch_len(uint8_t packet) {
  return (packet >> 2) & 0x0F;
}

static inline int16_t be_to_int16(uint8_t msb, uint8_t lsb) {
  return (int16_t)((msb << 8) | lsb);
}

static float be_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
  uint32_t u =
      ((uint32_t)b0 << 24) |
      ((uint32_t)b1 << 16) |
      ((uint32_t)b2 << 8)  |
      ((uint32_t)b3);

  float f;
  memcpy(&f, &u, sizeof(float));
  return f;
}

static void update_latest_sample(){
    if(!quat_valid || !gyro_valid || !accel_valid) return;

    // UM7 quaternion: a = w, b = x, c = y, d = z
    latest_sample.qw = quat_a_raw / QUAT_SCALE;
    latest_sample.qx = quat_b_raw / QUAT_SCALE;
    latest_sample.qy = quat_c_raw / QUAT_SCALE;
    latest_sample.qz = quat_d_raw / QUAT_SCALE;

    // Gyro (deg/s -> rad/s)
    latest_sample.wx = gyro_x_dps * PI / 180.0f;
    latest_sample.wy = gyro_y_dps * PI / 180.0f;
    latest_sample.wz = gyro_z_dps * PI / 180.0f;

    // Accel: already m/s^2
    latest_sample.ax = accel_x_mps2;
    latest_sample.ay = accel_y_mps2;
    latest_sample.az = accel_z_mps2;

    latest_sample.valid = true;
    sample_ready = true;
}


// Parser
static bool try_parse_one_packet(Um7Packet& out, size_t& consumed){
    consumed = 0;

    if(rx_len < 7) return false; // Minimum packet size

    // Look for header ('s', 'n', 'p')
    size_t start = 0;
    bool found = false;

    for(; start +2 < rx_len; ++start){
        if(rx_buffer[start] == 's' && rx_buffer[start + 1] == 'n' && rx_buffer[start + 2] == 'p'){
            found = true;
            break;
        }
    }

    if(!found){
        if(rx_len > 2){
            memmove(rx_buffer, rx_buffer + rx_len -2, 2);
            rx_len = 2;
        }
        return false;
    }


    // Remove garbage before header
    if(start > 0){
        memmove(rx_buffer, rx_buffer + start, rx_len - start);
        rx_len -= start;
    }

    if(rx_len < 7) return false;

    uint8_t packet = rx_buffer[3];
    bool has_data = packet_has_data(packet);
    bool is_batch = packet_is_batch(packet);
    uint8_t batch_len = packet_batch_len(packet);

    size_t data_len = 0;
    if(has_data)    data_len = is_batch ? (4 * batch_len) : 4; 

    const size_t total_len = 3 + 1 + 1 + data_len + 2; // header + packet + addr + data + checksum
    if(rx_len < total_len) return false;


    // Checksum
    uint16_t calc = 0;
    for(size_t i = 0; i < total_len - 2; ++i){
        calc += rx_buffer[i];
    }

    uint16_t recv = ((uint16_t)rx_buffer[total_len -2] << 8) | rx_buffer[total_len -1];

    if(calc != recv){
        memmove(rx_buffer, rx_buffer + 1, rx_len -1);
        rx_len -= 1;
        return false;
    }


    out.packet = packet;
    out.address = rx_buffer[4];
    out.data_len = data_len;
    out.checksum = recv;

    if(data_len > 0)    memcpy(out.data, rx_buffer + 5, data_len);

    consumed = total_len;
    return true;
}


// Packet Handling
static void handle_packet(const Um7Packet& packet){
    const bool has_data = packet_has_data(packet.packet);
    const bool is_batch = packet_is_batch(packet.packet);
    const uint8_t batch_len = packet_batch_len(packet.packet);

    if(!has_data || !is_batch) return;

    // Quaternion batch: 0x6D, 0x6E, 0x6F
    if(packet.address == 0x6D && batch_len >= 2 && packet.data_len >= 8){
        quat_a_raw = be_to_int16(packet.data[0], packet.data[1]);
        quat_b_raw = be_to_int16(packet.data[2], packet.data[3]);
        quat_c_raw = be_to_int16(packet.data[4], packet.data[5]);
        quat_d_raw = be_to_int16(packet.data[6], packet.data[7]);
        quat_valid = true;
        update_latest_sample();
        return;
    }

    // Processed gyro batch: 0x61, 0x62, 0x63, 0x64
    if(packet.address == 0x61 && batch_len >= 4 && packet.data_len >= 16){
        gyro_x_dps = be_to_float(packet.data[0], packet.data[1], packet.data[2], packet.data[3]);
        gyro_y_dps = be_to_float(packet.data[4], packet.data[5], packet.data[6], packet.data[7]);
        gyro_z_dps = be_to_float(packet.data[8], packet.data[9], packet.data[10], packet.data[11]);
        gyro_valid = true;
        update_latest_sample();
        return;
    }

    // Processed accel batch: 0x65, 0x66, 0x67, 0x68
    if(packet.address == 0x65 && batch_len >= 4 && packet.data_len >= 16){
        accel_x_mps2 = be_to_float(packet.data[0], packet.data[1], packet.data[2], packet.data[3]);
        accel_y_mps2 = be_to_float(packet.data[4], packet.data[5], packet.data[6], packet.data[7]);
        accel_z_mps2 = be_to_float(packet.data[8], packet.data[9], packet.data[10], packet.data[11]);
        accel_valid = true;
        update_latest_sample();
        return;
    }
}


// Public API
void um7_begin(){
    rx_len = 0;
    quat_valid = false;
    gyro_valid = false;
    accel_valid = false;
    sample_ready = false;
    latest_sample = {0};

    IMU.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
}


void um7_update(){
    // Read available data
    while(IMU.available()){
        if(rx_len < sizeof(rx_buffer)){
            rx_buffer[rx_len++] = (uint8_t)IMU.read();
        }
        else{
            rx_len = 0; // overflow recovery
        }
    }

    // Parse all complete packets
    while(true){
        Um7Packet packet;
        size_t consumed = 0;

        if(!try_parse_one_packet(packet, consumed)) break;

        handle_packet(packet);

        if(consumed > 0 && consumed <= rx_len){
            memmove(rx_buffer, rx_buffer + consumed, rx_len - consumed);
            rx_len -= consumed;
        }
        else{
            rx_len = 0;
            break;
        }
    }
}


bool um7_get_sample(ImuSample& out){
    if(!sample_ready || !latest_sample.valid) return false;

    out = latest_sample;
    sample_ready = false;
    return true;
}