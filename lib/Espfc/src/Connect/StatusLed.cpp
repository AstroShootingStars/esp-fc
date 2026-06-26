#include "StatusLed.hpp"
#include "Target/Target.h"
#include "Model.h"
#include <Arduino.h>
#include <algorithm>

#ifdef ESPFC_LED_WS2812
#include "driver/i2s.h"

// https://docs.espressif.com/projects/esp-idf/en/v4.4.4/esp32/api-reference/peripherals/i2s.html
// https://github.com/vunam/esp32-i2s-ws2812/blob/master/ws2812.c

static constexpr size_t LED_NUMBER = Espfc::LED_STRIP_MAX_LENGTH;
static constexpr size_t PIXEL_SIZE = 12; // each colour takes 4 bytes in buffer
static constexpr size_t ZERO_BUFFER = 32;
static constexpr size_t SIZE_BUFFER = LED_NUMBER * PIXEL_SIZE + ZERO_BUFFER;
static constexpr uint32_t SAMPLE_RATE = 93750;
static constexpr i2s_port_t I2S_NUM = I2S_NUM_0;

typedef struct {
  uint8_t g;
  uint8_t r;
  uint8_t b;
} ws2812_pixel_t;

static i2s_config_t i2s_config = {
  .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
  .sample_rate = SAMPLE_RATE,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
  .communication_format = I2S_COMM_FORMAT_STAND_MSB,
  .intr_alloc_flags = 0,
  .dma_buf_count = 2,
  .dma_buf_len = SIZE_BUFFER / 2,
  .use_apll = false,
  .tx_desc_auto_clear = false,
  .fixed_mclk = 0,
  .mclk_multiple = I2S_MCLK_MULTIPLE_DEFAULT,
  .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
};

static i2s_pin_config_t pin_config = {
  .bck_io_num = -1,
  .ws_io_num = -1,
  .data_out_num = -1,
  .data_in_num = -1
};

static uint8_t out_buffer[SIZE_BUFFER] = {0};

static const uint16_t bitpatterns[4] = {0x88, 0x8e, 0xe8, 0xee};

static void ws2812_init(int8_t pin)
{
  pin_config.data_out_num = pin;
  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM);
  std::fill_n(out_buffer, SIZE_BUFFER, 0);
}

static void ws2812_write_pixel(uint8_t * buffer, const ws2812_pixel_t& pixel)
{
#if defined(ESPFC_LED_WS2812_ORDER_RGB) && (ESPFC_LED_WS2812_ORDER_RGB)
  *buffer++ = bitpatterns[pixel.r >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.g >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.b >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 0 & 0x03];
#else
  *buffer++ = bitpatterns[pixel.g >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.g >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.r >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.r >> 0 & 0x03];

  *buffer++ = bitpatterns[pixel.b >> 6 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 4 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 2 & 0x03];
  *buffer++ = bitpatterns[pixel.b >> 0 & 0x03];
#endif
}

static void ws2812_update(const ws2812_pixel_t * pixels, size_t count)
{
  size_t bytes_written = 0;
  const ws2812_pixel_t pixelOff = {0, 0, 0};
  const size_t ledCount = std::min(count, LED_NUMBER);
  for (size_t i = 0; i < ledCount; i++)
  {
    size_t loc = i * PIXEL_SIZE;
    ws2812_write_pixel(out_buffer + loc, pixels[i]);
  }

  for(size_t i = ledCount; i < LED_NUMBER; i++)
  {
    size_t loc = i * PIXEL_SIZE;
    ws2812_write_pixel(out_buffer + loc, pixelOff);
  }

  i2s_zero_dma_buffer(I2S_NUM);
  i2s_write(I2S_NUM, out_buffer, SIZE_BUFFER, &bytes_written, portMAX_DELAY);
}

static const ws2812_pixel_t PIXEL_BOOT[] = {{0x00, 0x80, 0x00}};   // red
static const ws2812_pixel_t PIXEL_OK[] = {{0x80, 0x00, 0x00}};     // green
// Use strong red + medium green so ERROR appears clearly orange/yellow even if
// board/channel ordering differs from expected GRB mapping.
static const ws2812_pixel_t PIXEL_ERROR[] = {{0x60, 0xFF, 0x00}};
static const ws2812_pixel_t PIXEL_OFF[] = {{0, 0, 0}};

#endif

namespace Espfc::Connect
{

namespace {

constexpr uint8_t LED_BASEFUNCTION_COUNT = 10;
constexpr uint8_t LED_OVERLAY_BLINK = 3;
constexpr uint8_t LED_OVERLAY_RAINBOW = 1;

constexpr uint8_t LED_FUNCTION_COLOR = 0;
constexpr uint8_t LED_FUNCTION_FLIGHT_MODE = 1;
constexpr uint8_t LED_FUNCTION_ARM_STATE = 2;
constexpr uint8_t LED_FUNCTION_BATTERY = 3;
constexpr uint8_t LED_FUNCTION_RSSI = 4;
constexpr uint8_t LED_FUNCTION_GPS = 5;
constexpr uint8_t LED_FUNCTION_THRUST_RING = 6;

constexpr uint8_t LED_DIRECTION_COUNT_LOCAL = 6;
constexpr uint8_t LED_MODE_ORIENTATION = 0;
constexpr uint8_t LED_MODE_HEADFREE = 1;
constexpr uint8_t LED_MODE_HORIZON = 2;
constexpr uint8_t LED_MODE_ANGLE = 3;
constexpr uint8_t LED_MODE_MAG = 4;

constexpr uint8_t LED_SCOLOR_DISARMED = 0;
constexpr uint8_t LED_SCOLOR_ARMED = 1;
constexpr uint8_t LED_SCOLOR_ANIMATION = 2;
constexpr uint8_t LED_SCOLOR_BACKGROUND = 3;
constexpr uint8_t LED_SCOLOR_BLINKBACKGROUND = 4;
constexpr uint8_t LED_SCOLOR_GPSNOSATS = 5;
constexpr uint8_t LED_SCOLOR_GPSNOLOCK = 6;
constexpr uint8_t LED_SCOLOR_GPSLOCKED = 7;

constexpr uint8_t LED_FUNCTION_OFFSET = 8;
constexpr uint8_t LED_OVERLAY_OFFSET = 12;
constexpr uint8_t LED_COLOR_OFFSET = 22;
constexpr uint8_t LED_DIRECTION_OFFSET = 26;

constexpr uint8_t LED_FUNCTION_MASK = 0x0f;
constexpr uint16_t LED_OVERLAY_MASK = 0x03ff;
constexpr uint8_t LED_COLOR_MASK = 0x0f;
constexpr uint8_t LED_DIRECTION_MASK = 0x3f;

inline uint8_t getFunction(uint32_t config)
{
  return (config >> LED_FUNCTION_OFFSET) & LED_FUNCTION_MASK;
}

inline uint16_t getOverlay(uint32_t config)
{
  return (config >> LED_OVERLAY_OFFSET) & LED_OVERLAY_MASK;
}

inline uint8_t getColor(uint32_t config)
{
  return (config >> LED_COLOR_OFFSET) & LED_COLOR_MASK;
}

inline uint8_t getDirection(uint32_t config)
{
  return (config >> LED_DIRECTION_OFFSET) & LED_DIRECTION_MASK;
}

inline bool getOverlayBit(uint32_t config, uint8_t id)
{
  return (getOverlay(config) >> id) & 0x01;
}

ws2812_pixel_t hsvToRgb(const LedHsvConfig& hsv, uint8_t brightness)
{
  const float h = (float)(hsv.h % 360) / 60.0f;
  const float sat = (255.0f - (float)hsv.s) / 255.0f; // Betaflight stores saturation inverted.
  const float val = (float)hsv.v / 255.0f;

  const int sector = (int)h;
  const float f = h - (float)sector;
  const float p = val * (1.0f - sat);
  const float q = val * (1.0f - sat * f);
  const float t = val * (1.0f - sat * (1.0f - f));

  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;

  switch(sector)
  {
    case 0: r = val; g = t; b = p; break;
    case 1: r = q; g = val; b = p; break;
    case 2: r = p; g = val; b = t; break;
    case 3: r = p; g = q; b = val; break;
    case 4: r = t; g = p; b = val; break;
    default: r = val; g = p; b = q; break;
  }

  const uint8_t scale = std::min<uint8_t>(brightness, 100);
  const float bs = (float)scale / 100.0f;

  ws2812_pixel_t rgb;
  rgb.r = (uint8_t)std::clamp((int)lrintf(r * 255.0f * bs), 0, 255);
  rgb.g = (uint8_t)std::clamp((int)lrintf(g * 255.0f * bs), 0, 255);
  rgb.b = (uint8_t)std::clamp((int)lrintf(b * 255.0f * bs), 0, 255);
  return rgb;
}

const LedHsvConfig& getSpecialColor(const Model& model, uint8_t specialColorIdx)
{
  const uint8_t specialIdx = std::min<uint8_t>(specialColorIdx, LED_SPECIAL_COLOR_COUNT - 1);
  const uint8_t colorIdx = std::min<uint8_t>(model.config.ledStrip.specialColors[specialIdx], LED_CONFIGURABLE_COLOR_COUNT - 1);
  return model.config.ledStrip.colors[colorIdx];
}

const LedHsvConfig& getDirectionalColor(const Model& model, uint8_t mode, uint8_t directionMask)
{
  const uint8_t modeIdx = std::min<uint8_t>(mode, LED_MODE_COUNT - 1);
  for(uint8_t i = 0; i < LED_DIRECTION_COUNT_LOCAL; i++)
  {
    if((directionMask >> i) & 0x01)
    {
      const uint8_t colorIdx = std::min<uint8_t>(model.config.ledStrip.modeColors[modeIdx][i], LED_CONFIGURABLE_COLOR_COUNT - 1);
      return model.config.ledStrip.colors[colorIdx];
    }
  }

  return getSpecialColor(model, LED_SCOLOR_DISARMED);
}

uint8_t getActiveFlightModeColorMode(const Model& model)
{
  if(model.isModeActive(MODE_HEADFREE)) return LED_MODE_HEADFREE;
  if(model.isModeActive(MODE_MAG)) return LED_MODE_MAG;
  if(model.isModeActive(MODE_HORIZON)) return LED_MODE_HORIZON;
  if(model.isModeActive(MODE_ANGLE)) return LED_MODE_ANGLE;
  return LED_MODE_ORIENTATION;
}

}

static int LED_OFF_PATTERN[] = {0};
static int LED_BOOT_PATTERN[] = {80, 80, 80, 300, 0};
static int LED_OK_PATTERN[] = {100, 900, 0};
static int LED_ERROR_PATTERN[] = {100, 100, 100, 100, 100, 1500, 0};
static int LED_ON_PATTERN[] = {100, 0};

StatusLed::StatusLed() : _pin(-1), _invert(0), _status(LED_OFF), _next(0), _state(LOW), _step(0), _pattern(LED_OFF_PATTERN) {}

void StatusLed::begin(int8_t pin, uint8_t type, uint8_t invert)
{
  if(pin == -1) return;

  _pin = pin;
  _type = type;
  _invert = invert;

#ifdef ESPFC_LED_WS2812
  if(_type == LED_STRIP) ws2812_init(_pin);
  if(_type == LED_SIMPLE) pinMode(_pin, OUTPUT);
#else
  pinMode(_pin, OUTPUT);
#endif
  setStatus(LED_ON, true);
}

void StatusLed::setStatus(LedStatus newStatus, bool force)
{
  if(_pin == -1) return;
  if(!force && newStatus == _status) return;

  _status = newStatus;
  _state = LOW;
  _step = 0;
  _next = millis();

  switch (_status)
  {
    case LED_BOOT:
      _pattern = LED_BOOT_PATTERN;
      _state = HIGH;
      break;
    case LED_OK:
      _pattern = LED_OK_PATTERN;
      break;
    case LED_ERROR:
      _pattern = LED_ERROR_PATTERN;
      break;
    case LED_ON:
      _pattern = LED_ON_PATTERN;
      _state = HIGH;
      break;
    case LED_OFF:
    default:
      _pattern = LED_OFF_PATTERN;
      break;
  }
  _write(_state);
}

void StatusLed::update()
{
  if(_pin == -1 || !_pattern) return;
  
  uint32_t now = millis();
  
  if(now < _next) return;

  if (!_pattern[_step])
  {
    _step = 0;
    _next = now + 20;
    return;
  }

  _state = !(_step & 1);
  _write(_state);

  _next = now + _pattern[_step];
  _step++;
}

void StatusLed::update(const Espfc::Model& model)
{
#ifdef ESPFC_LED_WS2812
  if(_type != LED_STRIP || !model.isFeatureActive(FEATURE_LED_STRIP))
  {
    update();
    return;
  }

  if(_pin == -1) return;

  const uint32_t now = millis();
  if(now < _next) return;
  _next = now + 16; // ~60 Hz

  size_t ledCount = 0;
  for(size_t i = 0; i < LED_STRIP_MAX_LENGTH; i++)
  {
    if(model.config.ledStrip.ledConfig[i] != 0)
    {
      ledCount = i + 1;
    }
  }

  // Fall back to single-pixel status behavior until a strip layout is configured.
  if(ledCount == 0)
  {
    _write(_state);
    return;
  }

  static ws2812_pixel_t pixels[LED_STRIP_MAX_LENGTH];

  const bool blinkActive = ((now / 125) & 0x07) < 2;
  const uint8_t activeMode = getActiveFlightModeColorMode(model);
  const bool armed = model.isModeActive(MODE_ARMED);

  for(size_t i = 0; i < ledCount; i++)
  {
    const uint32_t cfg = model.config.ledStrip.ledConfig[i];
    const uint8_t fn = getFunction(cfg);
    const uint8_t colorIdx = std::min<uint8_t>(getColor(cfg), LED_CONFIGURABLE_COLOR_COUNT - 1);
    const uint8_t dirMask = getDirection(cfg);

    const LedHsvConfig* selected = &getSpecialColor(model, LED_SCOLOR_BACKGROUND);

    switch(fn)
    {
      case LED_FUNCTION_COLOR:
      case LED_FUNCTION_THRUST_RING:
        selected = &model.config.ledStrip.colors[colorIdx];
        break;
      case LED_FUNCTION_FLIGHT_MODE:
        selected = &getDirectionalColor(model, activeMode, dirMask);
        break;
      case LED_FUNCTION_ARM_STATE:
        selected = &getSpecialColor(model, armed ? LED_SCOLOR_ARMED : LED_SCOLOR_DISARMED);
        break;
      case LED_FUNCTION_GPS:
        if(!model.state.gps.present || model.state.gps.numSats == 0) selected = &getSpecialColor(model, LED_SCOLOR_GPSNOSATS);
        else if(!model.state.gps.fix) selected = &getSpecialColor(model, LED_SCOLOR_GPSNOLOCK);
        else selected = &getSpecialColor(model, LED_SCOLOR_GPSLOCKED);
        break;
      case LED_FUNCTION_BATTERY:
      case LED_FUNCTION_RSSI:
      default:
        if(fn >= LED_BASEFUNCTION_COUNT)
        {
          selected = &getSpecialColor(model, LED_SCOLOR_BACKGROUND);
        }
        else
        {
          selected = &getSpecialColor(model, armed ? LED_SCOLOR_ARMED : LED_SCOLOR_DISARMED);
        }
        break;
    }

    LedHsvConfig effective = *selected;

    if(getOverlayBit(cfg, LED_OVERLAY_RAINBOW))
    {
      const uint16_t delta = model.config.ledStrip.rainbowDelta;
      const uint16_t freq = std::max<uint16_t>(model.config.ledStrip.rainbowFreq, 1);
      const uint16_t phase = (uint16_t)((now / freq) % 360u);
      effective.h = (effective.h + phase + (uint16_t)(delta * i)) % 360u;
    }

    if(getOverlayBit(cfg, LED_OVERLAY_BLINK) && !blinkActive)
    {
      effective = getSpecialColor(model, LED_SCOLOR_BLINKBACKGROUND);
    }

    pixels[i] = hsvToRgb(effective, model.config.ledStrip.brightness);
  }

  ws2812_update(pixels, ledCount);
#else
  update();
#endif
}

void StatusLed::_write(uint8_t val)
{
#ifdef ESPFC_LED_WS2812
  if(_type == LED_STRIP)
  {
    if(!val)
    {
      ws2812_update(PIXEL_OFF, 1);
      return;
    }

    switch(_status)
    {
      case LED_BOOT:
        ws2812_update(PIXEL_BOOT, 1);
        break;
      case LED_ERROR:
        ws2812_update(PIXEL_ERROR, 1);
        break;
      case LED_ON:
      case LED_OK:
      default:
        ws2812_update(PIXEL_OK, 1);
        break;
    }
    return;
  }
  if(_type == LED_SIMPLE) digitalWrite(_pin, val ^ _invert);
#else
  digitalWrite(_pin, val ^ _invert);
#endif
}

}
