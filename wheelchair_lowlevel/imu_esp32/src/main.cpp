#include <Arduino.h>

static const int IMU_RX_PIN = 38;   
static const int IMU_TX_PIN = 39;

// UART 1
HardwareSerial IMU(1);

struct Um7Packet {
  uint8_t addr;
  uint8_t packet;
  uint8_t data_len;
  uint8_t data[64];  
};

static uint8_t rx_buffer[512];
static size_t  rx_len = 0;

static bool try_parse_one_packet(uint8_t* buf, size_t len, Um7Packet& out, size_t& bytes_to_discard) {
  bytes_to_discard = 0;
  if (len < 7) return false;

  // Search for header 's' 'n' 'p' (0x73 0x6E 0x70)
  size_t i = 0;
  for (; i + 2 < len; i++) 
    if (buf[i] == 's' && buf[i+1] == 'n' && buf[i+2] == 'p') break;

  // No header found -> Discard all
  if (i + 2 >= len) { bytes_to_discard = len; return false; }

  // Header found but not enough bytes
  if (len - i < 7) { bytes_to_discard = i; return false; }

  uint8_t packet = buf[i+3];
  uint8_t addr = buf[i+4];

  bool has_data = (packet >> 7) & 0x01;
  bool is_batch = (packet >> 6) & 0x01;

  // batch length (register count)
  uint8_t batch_len = (packet >> 2) & 0x0F;

  size_t data_len = 0;
  if (has_data) data_len = is_batch ? (4 * (size_t)batch_len) : 4;

  // 'snp' (3 bytes) + packet(1 byte) + addr(1 byte) + data + checksum (2 bytes)
  size_t packet_len = 7 + data_len;

  // Full packet not yet received
  if (len - i < packet_len) { bytes_to_discard = i; return false; }



  // Verify checksum
  uint16_t sum = (uint16_t)('s' + 'n' + 'p' + packet + addr);

  for (size_t k = 0; k < data_len; k++) 
    sum += buf[i + 5 + k];

  uint16_t rx_ck = ((uint16_t)buf[i + 5 + data_len] << 8) | buf[i + 6 + data_len];

  // Checksum mismatch -> shift by one byte and try
  if (sum != rx_ck) {
    bytes_to_discard = i + 1;
    return false;
  }

  // Fill output packet structure
  out.packet = packet;
  out.addr = addr;
  out.data_len = (uint8_t)data_len;

  for (size_t k = 0; k < data_len && k < sizeof(out.data); k++) 
    out.data[k] = buf[i + 5 + k];

  // Confirmed processed bytes
  bytes_to_discard = i + packet_len;
  return true;
}

static float roll = 0, pitch = 0, yaw = 0;
static uint32_t last_print_ms = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("USB SERIAL OK");

  IMU.begin(115200, SERIAL_8N1, IMU_RX_PIN, IMU_TX_PIN);
  Serial.println("IMU UART started");
}

void loop() {
  
  // Fill RX buffer (prevent overflow)
  while (IMU.available() && rx_len < sizeof(rx_buffer)) {
    rx_buffer[rx_len++] = (uint8_t)IMU.read();
  }

  // Reset buffer if full without valid packet
  if (rx_len == sizeof(rx_buffer))
    rx_len = 0;


  // Parse as many packets as possible
  while (true) {
    Um7Packet p;
    size_t bytes_to_discard = 0;
    bool ok = try_parse_one_packet(rx_buffer, rx_len, p, bytes_to_discard);

    // Remove processed invalid bytes and shift buffer
    if (bytes_to_discard > 0) {
      memmove(rx_buffer, rx_buffer + bytes_to_discard, rx_len - bytes_to_discard);
      rx_len -= bytes_to_discard;
    }

    if (!ok) break;



    // 0x70: Roll & Pitch (16-bits signed)
    if (p.addr == 0x70 && p.data_len >= 4) {
      int16_t roll_raw  = (int16_t)((p.data[0] << 8) | p.data[1]);
      int16_t pitch_raw = (int16_t)((p.data[2] << 8) | p.data[3]);

      // Scale factor (datasheet)
      roll  = (float)roll_raw  / 91.02222f;
      pitch = (float)pitch_raw / 91.02222f;
    }

    // 0x71: Yaw (16-bits signed)
    if (p.addr == 0x71 && p.data_len >= 2) {
      int16_t yaw_raw = (int16_t)((p.data[0] << 8) | p.data[1]);
      yaw = (float)yaw_raw / 91.02222f;
    }
  }

  // Print at 50Hz (every 20ms)
  uint32_t now = millis();
  if (now - last_print_ms >= 20) {
    last_print_ms = now;
    Serial.printf("RPY: %.2f, %.2f, %.2f\n", roll, pitch, yaw);
  }
}