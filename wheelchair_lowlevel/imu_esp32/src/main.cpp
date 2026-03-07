#include <Arduino.h>

HardwareSerial IMU(1); // UART1

// GPIO
static const int RX_PIN = 38;   // UM7 TX -> ESP32 RX
static const int TX_PIN = 39;   // UM7 RX -> ESP32 TX

// Scale
static const float QUAT_SCALE = 29789.09091f;

struct Um7Packet{
    uint8_t address = 0;
    uint8_t packet = 0;
    uint8_t data_len = 0;
    uint8_t data[256] = {0};
    uint16_t checksum = 0;
};

// Receive buffer for incoming UM7 packets
static uint8_t rx_buffer[512];
static size_t rx_len = 0;

// Quaternion
static bool quat_valid = false;
static int16_t quat_a_raw = 0, quat_b_raw = 0, quat_c_raw = 0, quat_d_raw = 0;

// Gyro
static bool gyro_valid = false;
static float gyro_x_dps = 0.0f, gyro_y_dps = 0.0f, gyro_z_dps = 0.0f;


// Quaternion Printing
static void printQuaternion(){
  float a = quat_a_raw / QUAT_SCALE;
  float b = quat_b_raw / QUAT_SCALE;
  float c = quat_c_raw / QUAT_SCALE;
  float d = quat_d_raw / QUAT_SCALE;

  float norm = sqrt(a*a + b*b + c*c + d*d);

  Serial.print("Quaternion: ");
  Serial.print(b, 6);
  Serial.print(", ");
  Serial.print(c, 6);
  Serial.print(", ");
  Serial.print(d, 6);
  Serial.print(", ");
  Serial.println(a, 6);
  Serial.print(" | norm = ");
  Serial.println(norm, 6);
}

// Gyro Printing
static void printGyro(){
  Serial.print("Gyro [deg/s]: ");
  Serial.print(gyro_x_dps, 6); Serial.print(", ");
  Serial.print(gyro_y_dps, 6); Serial.print(", ");
  Serial.println(gyro_z_dps, 6);
}

// Print IMU state
static void printImuState(){
  if(!quat_valid || !gyro_valid) return;

  // Quaternion
  float qw = quat_a_raw / QUAT_SCALE;   // Scaler
  float qx = quat_b_raw / QUAT_SCALE;   // x
  float qy = quat_c_raw / QUAT_SCALE;   // y
  float qz = quat_d_raw / QUAT_SCALE;   // z

  // Gyro (deg/s -> rad/s)
  float wx = gyro_x_dps * PI / 180.0f;  // Convert to radians/s
  float wy = gyro_y_dps * PI / 180.0f; 
  float wz = gyro_z_dps * PI / 180.0f;  

  Serial.print("orientation: ");
  Serial.print(qx, 6); Serial.print(" ");
  Serial.print(qy, 6); Serial.print(" ");
  Serial.print(qz, 6); Serial.print(" ");
  Serial.print(qw, 6);

  Serial.print(" | angular_velocity: ");
  Serial.print(wx, 6); Serial.print(" ");
  Serial.print(wy, 6); Serial.print(" ");
  Serial.println(wz, 6);
}


// Helper
static inline bool packet_has_data(uint8_t packet){
    return (packet >> 7) & 0x01;
}

static inline bool packet_is_batch(uint8_t packet){
    return (packet >> 6)  & 0x01;
}

static inline uint8_t packet_batch_len(uint8_t packet){
    return (packet >> 2) & 0x0F;
}

static inline int16_t be_to_int16(uint8_t msb, uint8_t lsb){
    return (int16_t)(msb << 8 | lsb);
}

static float be_to_float(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3){
    uint32_t u = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) | ((uint32_t)b2 << 8) | b3;
    float f;
    memcpy(&f, &u, sizeof(float));
    return f;
}



// Parser
static bool try_parse_one_packet(Um7Packet& out, size_t& consumed){
  consumed = 0;

  if(rx_len < 7)  return false;

  // Look for header  ('s', 'n', 'p')
  size_t start = 0;
  bool found = false;

  for(; start+2 < rx_len; ++start){
    if(rx_buffer[start] == 's' && rx_buffer[start+1] == 'n' && rx_buffer[start+2] == 'p'){
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

  // Discard garbage before header
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

  if(has_data)  data_len = is_batch ? (4 * batch_len) : 4;

  size_t total_len = 3 + 1 + 1 + data_len + 2;  // header (s,n,p) + packet + address + data_len + checksum
  if(rx_len < total_len) return false;


  // Checksum
  uint16_t calc = 0;

  for(size_t i = 0; i < total_len - 2; ++i){
    calc += rx_buffer[i];
  }

  uint16_t recv = ((uint16_t)rx_buffer[total_len - 2] << 8 | rx_buffer[total_len - 1]);

  if(calc != recv){
    memmove(rx_buffer, rx_buffer + 1, rx_len - 1);
    rx_len -= 1;
    return false;
  }


  out.packet = packet;
  out.address = rx_buffer[4];
  out.data_len = data_len;
  out.checksum = recv;

  if(data_len > 0)  memcpy(out.data, rx_buffer + 5, data_len);

  consumed = total_len;
  return true;
}


static void handle_packet(const Um7Packet& packet){
  bool has_data = packet_has_data(packet.packet);
  bool is_batch = packet_is_batch(packet.packet);
  uint8_t batch_len = packet_batch_len(packet.packet);

  // Quaternion
  if(has_data && is_batch && packet.address == 0x6D && batch_len >= 2 && packet.data_len >= 8){
    quat_a_raw = be_to_int16(packet.data[0], packet.data[1]);
    quat_b_raw = be_to_int16(packet.data[2], packet.data[3]);
    quat_c_raw = be_to_int16(packet.data[4], packet.data[5]);
    quat_d_raw = be_to_int16(packet.data[6], packet.data[7]);

    quat_valid = true;
    if(gyro_valid){
      printImuState();
      quat_valid = false;
      gyro_valid = false;
    }
    return;
  }

  // Gyro
  if(has_data && is_batch && packet.address == 0x61 && batch_len >= 4 && packet.data_len >= 16){
    gyro_x_dps = be_to_float(packet.data[0], packet.data[1], packet.data[2], packet.data[3]);
    gyro_y_dps = be_to_float(packet.data[4], packet.data[5], packet.data[6], packet.data[7]);
    gyro_z_dps = be_to_float(packet.data[8], packet.data[9], packet.data[10], packet.data[11]);

    gyro_valid = true;
    if(quat_valid){
      printImuState();
      quat_valid = false;
      gyro_valid = false;
    }
    return;
  }
}


void setup(){
  Serial.begin(115200);
  delay(500);

  IMU.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("UM7 start");
}

void loop(){
  while(IMU.available()){
    if(rx_len < sizeof(rx_buffer)){
      rx_buffer[rx_len++] = (uint8_t)IMU.read();
    }
    else{
      rx_len = 0;
    }
  }

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