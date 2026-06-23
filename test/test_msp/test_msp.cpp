#include <unity.h>
#include <ArduinoFake.h>
#include <Hal/Gpio.h>
#include <helper_3dmath.h>
#include <Kalman.h>
#include <EscDriver.h>
#include <printf.h>
#include "Connect/Msp.hpp"
#include "Connect/MspParser.hpp"
#include "Connect/MspProcessor.hpp"
#include "msp/msp_protocol.h"
#include <Gps.hpp>
#include <vector>

using namespace fakeit;
using namespace Espfc;
using namespace Espfc::Connect;

/*void setUp(void)
{
  ArduinoFakeReset();
}*/

// void tearDown(void) {
// // clean stuff up here
// }


#define MSP_V2_FLAG 0

class DummySerial: public Device::SerialDevice {
public:
  void begin(const SerialDeviceConfig&) override {}
  void updateBaudRate(int) override {}
  int available() override { return 0; }
  int read() override { return -1; }
  size_t readMany(uint8_t *, size_t) override { return 0; }
  int peek() override { return -1; }
  void flush() override {}
  size_t write(uint8_t c) override {
    tx.push_back(c);
    return 1;
  }
  size_t write(const uint8_t * c, size_t l) override {
    if(!c) return 0;
    tx.insert(tx.end(), c, c + l);
    return l;
  }
  int availableForWrite() override { return 4096; }
  bool isTxFifoEmpty() override { return true; }
  bool isSoft() const override { return false; }
  operator bool() const override { return true; }

  std::vector<uint8_t> tx;
};

static MspMessage makeCmd(uint16_t cmd, const uint8_t *payload, size_t len)
{
  MspMessage msg;
  msg.state = MSP_STATE_RECEIVED;
  msg.dir = MSP_TYPE_CMD;
  msg.version = MSP_V1;
  msg.cmd = cmd;
  msg.received = len;
  msg.read = 0;
  if(payload && len > 0)
  {
    std::copy(payload, payload + len, msg.buffer);
  }
  return msg;
}

void test_msp_v1_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<' };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V1, msg.version);
}

void test_msp_v1_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<', 0, MSP_API_VERSION, 1 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(1, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v1_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'M' , '<', 2, MSP_API_VERSION, 1, 2, 0 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(0, msg.checksum);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_header()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'X' , '<' };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL(MSP_V2, msg.version);
}

void test_msp_v2_parse_no_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'X' , '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 0, 0, 69 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(0, msg.received);
  TEST_ASSERT_EQUAL_INT(0, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(69, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_v2_parse_payload()
{
  MspMessage msg;
  MspParser parser;
  const uint8_t data[] = { '$', 'X' , '<', MSP_V2_FLAG, MSP_API_VERSION, 0, 2, 0, 1, 2, 102 };
  for(size_t i = 0; i < sizeof(data); i++)
  {
    parser.parse(data[i], msg);
  }
  TEST_ASSERT_EQUAL_INT(MSP_TYPE_CMD, msg.dir);
  TEST_ASSERT_EQUAL_INT(MSP_API_VERSION, msg.cmd);
  TEST_ASSERT_EQUAL_UINT16(2, msg.received);
  TEST_ASSERT_EQUAL_INT(2, msg.remain());
  TEST_ASSERT_EQUAL_UINT8(102, msg.checksum2);
  TEST_ASSERT_EQUAL_UINT8(MSP_STATE_RECEIVED, msg.state);
}

void test_msp_osd_config_roundtrip()
{
  Model model;
  MspProcessor proc(model);
  DummySerial serial;

  const uint8_t setGeneral[] = {
    0xFF, // addr = -1, general config
    2,    // video system
    1,    // units
    33,   // rssi alarm
    0x34, 0x12, // cap alarm
    0x00, 0x00, // legacy timer placeholder
    0xCD, 0xAB, // alt alarm
    0x22, 0x11, // warnings low16
    0x44, 0x33, 0x22, 0x11, // warnings u32
    2,    // profile
    1,    // overlay mode
    24, 13, // camera frame
    0x55, 0x00, // link quality alarm
    0x66, 0x00  // rssi dbm alarm
  };

  MspResponse rsp;
  proc.processCommand(makeCmd(MSP_SET_OSD_CONFIG, setGeneral, sizeof(setGeneral)), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);

  const uint8_t setItem[] = {3, 0x77, 0x06, 1}; // addr=3, value=0x0677, screen=1
  proc.processCommand(makeCmd(MSP_SET_OSD_CONFIG, setItem, sizeof(setItem)), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);

  const uint8_t setTimer[] = {0xFE, 1, 0x21, 0x03}; // addr=-2, timer idx=1, value=0x0321
  proc.processCommand(makeCmd(MSP_SET_OSD_CONFIG, setTimer, sizeof(setTimer)), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);

  proc.processCommand(makeCmd(MSP_OSD_CONFIG, nullptr, 0), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);

  TEST_ASSERT_GREATER_THAN_UINT16(100, rsp.len);
  TEST_ASSERT_EQUAL_UINT8(2, rsp.data[1]); // video system
  TEST_ASSERT_EQUAL_UINT8(1, rsp.data[2]); // units
  TEST_ASSERT_EQUAL_UINT8(33, rsp.data[3]); // rssi alarm
  TEST_ASSERT_EQUAL_UINT8(0x34, rsp.data[4]);
  TEST_ASSERT_EQUAL_UINT8(0x12, rsp.data[5]);

  const size_t itemBase = 11;
  const size_t itemOffset = itemBase + 3u * 2u;
  TEST_ASSERT_EQUAL_UINT8(0x77, rsp.data[itemOffset + 0]);
  TEST_ASSERT_EQUAL_UINT8(0x06, rsp.data[itemOffset + 1]);

  const size_t timerBase = itemBase + (ESPFC_OSD_ITEM_COUNT * 2) + 1 + ESPFC_OSD_STAT_COUNT + 1;
  TEST_ASSERT_EQUAL_UINT8(0x21, rsp.data[timerBase + 2]);
  TEST_ASSERT_EQUAL_UINT8(0x03, rsp.data[timerBase + 3]);

  const size_t profileCountOffset = timerBase + (ESPFC_OSD_TIMER_COUNT * 2) + 2 + 1 + 4;
  TEST_ASSERT_EQUAL_UINT8(model.config.osd.profileCount, rsp.data[profileCountOffset]);
  TEST_ASSERT_EQUAL_UINT8(2, rsp.data[profileCountOffset + 1]);
}

void test_msp_board_info_payload_shape()
{
  Model model;
  MspProcessor proc(model);
  DummySerial serial;
  MspResponse rsp;

  proc.processCommand(makeCmd(MSP_BOARD_INFO, nullptr, 0), rsp, serial);

  TEST_ASSERT_EQUAL_INT(1, rsp.result);
  TEST_ASSERT_EQUAL_UINT16(51, rsp.len);

  // 4-byte FC identifier, then board version/type/capabilities.
  TEST_ASSERT_NOT_EQUAL(0u, rsp.data[0] | rsp.data[1] | rsp.data[2] | rsp.data[3]);
  TEST_ASSERT_EQUAL_UINT8(0, rsp.data[8]);  // target name length
  TEST_ASSERT_EQUAL_UINT8(0, rsp.data[9]);  // board name length
  TEST_ASSERT_EQUAL_UINT8(0, rsp.data[10]); // manufacturer id length

  for(size_t i = 11; i < 43; i++)
  {
    TEST_ASSERT_EQUAL_UINT8(0, rsp.data[i]); // signature bytes
  }

  TEST_ASSERT_EQUAL_UINT8(0, rsp.data[43]); // mcu type id
  TEST_ASSERT_EQUAL_UINT8(0, rsp.data[44]); // configuration state
}

void test_msp_compass_declination_roundtrip()
{
  Model model;
  MspProcessor proc(model);
  DummySerial serial;
  MspResponse rsp;

  const uint8_t setDeclination[] = {0x66, 0x00}; // 10.2 deg in 0.1-deg units
  proc.processCommand(makeCmd(MSP_SET_COMPASS_CONFIG, setDeclination, sizeof(setDeclination)), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);

  proc.processCommand(makeCmd(MSP_COMPASS_CONFIG, nullptr, 0), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);
  TEST_ASSERT_EQUAL_UINT16(2, rsp.len);
  TEST_ASSERT_EQUAL_UINT8(0x66, rsp.data[0]);
  TEST_ASSERT_EQUAL_UINT8(0x00, rsp.data[1]);
}

void test_msp_sensor_config_payload_shape()
{
  Model model;
  MspProcessor proc(model);
  DummySerial serial;
  MspResponse rsp;

  proc.processCommand(makeCmd(MSP_SENSOR_CONFIG, nullptr, 0), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);
  TEST_ASSERT_EQUAL_UINT16(5, rsp.len);

  const uint8_t setCfg[] = {
    2, // accel
    3, // baro
    4, // mag
    2, // rangefinder (mapped to internal default, reported back as enabled)
    1  // optical flow
  };
  proc.processCommand(makeCmd(MSP_SET_SENSOR_CONFIG, setCfg, sizeof(setCfg)), rsp, serial);
  TEST_ASSERT_EQUAL_INT(1, rsp.result);

  proc.processCommand(makeCmd(MSP_SENSOR_CONFIG, nullptr, 0), rsp, serial);
  TEST_ASSERT_EQUAL_UINT16(5, rsp.len);
  TEST_ASSERT_EQUAL_UINT8(2, rsp.data[0]);
  TEST_ASSERT_EQUAL_UINT8(3, rsp.data[1]);
  TEST_ASSERT_EQUAL_UINT8(4, rsp.data[2]);
  TEST_ASSERT_EQUAL_UINT8(1, rsp.data[3]);
  TEST_ASSERT_EQUAL_UINT8(1, rsp.data[4]);
}

void test_msp2_gyro_sensor_active_auto_maps_to_bf_default()
{
  Model model;
  MspProcessor proc(model);
  DummySerial serial;
  MspResponse rsp;

  model.config.gyro.dev = GYRO_AUTO;
  proc.processCommand(makeCmd(MSP2_GYRO_SENSOR_ACTIVE, nullptr, 0), rsp, serial);

  TEST_ASSERT_EQUAL_INT(1, rsp.result);
  TEST_ASSERT_EQUAL_UINT16(2, rsp.len);
  TEST_ASSERT_EQUAL_UINT8(1, rsp.data[0]); // one gyro slot
  TEST_ASSERT_EQUAL_UINT8(1, rsp.data[1]); // BF GYRO_DEFAULT (not GYRO_NONE)
}

int main(int argc, char **argv)
{
  UNITY_BEGIN();
  RUN_TEST(test_msp_v1_parse_header);
  RUN_TEST(test_msp_v1_parse_no_payload);
  RUN_TEST(test_msp_v1_parse_payload);
  RUN_TEST(test_msp_v2_parse_header);
  RUN_TEST(test_msp_v2_parse_no_payload);
  RUN_TEST(test_msp_v2_parse_payload);
  RUN_TEST(test_msp_osd_config_roundtrip);
  RUN_TEST(test_msp_board_info_payload_shape);
  RUN_TEST(test_msp_compass_declination_roundtrip);
  RUN_TEST(test_msp_sensor_config_payload_shape);
  RUN_TEST(test_msp2_gyro_sensor_active_auto_maps_to_bf_default);

  return UNITY_END();
}