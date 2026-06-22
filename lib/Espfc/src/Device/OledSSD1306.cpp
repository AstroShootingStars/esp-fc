#include "Device/OledSSD1306.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace Espfc::Device {

namespace {
static constexpr uint32_t OLED_REFRESH_US = 250000; // 4 Hz refresh
}

int OledSSD1306::begin(BusDevice* bus)
{
  if (!bus) return 0;

  if (begin(bus, ADDRESS_0)) return 1;
  if (begin(bus, ADDRESS_1)) return 1;

  return 0;
}

int OledSSD1306::begin(BusDevice* bus, uint8_t addr)
{
  if (!bus) return 0;

  setBus(bus, addr);
  if (!testConnection()) return 0;

  initDisplay();
  clearDisplay();
  _lastRenderUs = 0;
  _lastPageSwitchMs = millis();
  _currentPage = 0;
  return 1;
}

void OledSSD1306::setHeight(uint8_t h)
{
  if(h == 32 || h == 64)
  {
    _height = h;
  }
  else
  {
    _height = 64;
  }
  _pages = _height / 8;
}

void OledSSD1306::setPageInterval(int32_t intervalMs)
{
  _pageIntervalMs = (uint16_t)constrain(intervalMs, 500l, 30000l);
}

void OledSSD1306::update(const Model& model)
{
  if(!_bus) return;

  const uint32_t nowUs = micros();
  if(_lastRenderUs && (nowUs - _lastRenderUs) < OLED_REFRESH_US) return;
  _lastRenderUs = nowUs;

  char lines[OLED_MAX_LINES][OLED_MAX_CHARS] = {{0}};
  uint8_t lineCount = 0;
  composeLines(model, lines, lineCount);

  const uint8_t visibleLines = _pages;
  const uint8_t totalPages = std::max<uint8_t>(1u, (lineCount + visibleLines - 1u) / visibleLines);

  const uint32_t nowMs = millis();
  if(totalPages > 1 && (nowMs - _lastPageSwitchMs) >= _pageIntervalMs)
  {
    _currentPage = (_currentPage + 1u) % totalPages;
    _lastPageSwitchMs = nowMs;
  }
  if(_currentPage >= totalPages) _currentPage = 0;

  renderPage(lines, lineCount, _currentPage);
}

bool OledSSD1306::testConnection()
{
  if (!_bus) return false;

  // SSD1306 is often write-only over I2C; command-write ACK is used as probe.
  return writeCommand(0xAE);
}

bool OledSSD1306::writeCommand(uint8_t cmd)
{
  if (!_bus) return false;
  return _bus->write(_addr, 0x00, 1, &cmd);
}

bool OledSSD1306::writeData(const uint8_t* data, uint8_t len)
{
  if(!_bus || !data || len == 0) return false;
  return _bus->write(_addr, 0x40, len, data);
}

void OledSSD1306::setCursor(uint8_t page, uint8_t col)
{
  writeCommand(0xB0 | (page & 0x07));
  writeCommand(0x00 | (col & 0x0F));
  writeCommand(0x10 | ((col >> 4) & 0x0F));
}

void OledSSD1306::clearDisplay()
{
  uint8_t blank[16] = {0};
  for(uint8_t p = 0; p < _pages; p++)
  {
    setCursor(p, 0);
    for(uint8_t i = 0; i < (OLED_WIDTH / sizeof(blank)); i++)
    {
      writeData(blank, sizeof(blank));
    }
  }
}

void OledSSD1306::drawTextLine(uint8_t page, const char* text)
{
  if(page >= _pages) return;

  uint8_t row[OLED_WIDTH] = {0};
  const uint8_t charsPerLine = OLED_WIDTH / 6;

  for(uint8_t i = 0; i < charsPerLine; i++)
  {
    const char c = (text && text[i]) ? text[i] : ' ';
    uint8_t glyph[5] = {0};
    getGlyph(c, glyph);
    const uint8_t offset = i * 6;
    row[offset] = glyph[0];
    row[offset + 1] = glyph[1];
    row[offset + 2] = glyph[2];
    row[offset + 3] = glyph[3];
    row[offset + 4] = glyph[4];
    row[offset + 5] = 0x00;
  }

  setCursor(page, 0);
  for(uint8_t i = 0; i < (OLED_WIDTH / 16); i++)
  {
    writeData(&row[i * 16], 16);
  }
}

void OledSSD1306::composeLines(const Model& model, char lines[OLED_MAX_LINES][OLED_MAX_CHARS], uint8_t& lineCount) const
{
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "ESP-FC");
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "ARM:%s", model.isModeActive(MODE_ARMED) ? "ON" : "OFF");
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "VBAT:%0.2fV %d%%", model.state.battery.voltage, (int)lrintf(model.state.battery.percentage));
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "ALT:%0.2fm", model.state.altitude.height);
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "RNG-B:%umm", (unsigned)model.state.rangefinder[RANGEFINDER_BOTTOM].distance);
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "RNG-F:%umm", (unsigned)model.state.rangefinder[RANGEFINDER_FRONT].distance);
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "GPS:%s SAT:%u", model.state.gps.fix ? "FIX" : "NO", (unsigned)model.state.gps.numSats);
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "THR:%0.0fus", model.state.input.us[AXIS_THRUST]);
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "LOOP:%ldHz", (long)model.state.loopRate);
  if(lineCount < OLED_MAX_LINES) std::snprintf(lines[lineCount++], OLED_MAX_CHARS, "I2CERR:%d", (int)model.state.i2cErrorCount);

  for(uint8_t i = 0; i < lineCount && i < OLED_MAX_LINES; i++)
  {
    lines[i][OLED_MAX_CHARS - 1] = '\0';
  }
}

void OledSSD1306::renderPage(const char lines[OLED_MAX_LINES][OLED_MAX_CHARS], uint8_t lineCount, uint8_t pageIndex)
{
  const uint8_t visibleLines = _pages;
  const uint8_t firstLine = pageIndex * visibleLines;

  for(uint8_t p = 0; p < visibleLines; p++)
  {
    const uint8_t lineIndex = firstLine + p;
    if(lineIndex < lineCount)
    {
      drawTextLine(p, lines[lineIndex]);
    }
    else
    {
      drawTextLine(p, "");
    }
  }
}

void OledSSD1306::getGlyph(char c, uint8_t glyph[5]) const
{
  const char uc = (char)std::toupper((unsigned char)c);
  switch(uc)
  {
    case 'A': glyph[0]=0x7E; glyph[1]=0x11; glyph[2]=0x11; glyph[3]=0x11; glyph[4]=0x7E; break;
    case 'B': glyph[0]=0x7F; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x36; break;
    case 'C': glyph[0]=0x3E; glyph[1]=0x41; glyph[2]=0x41; glyph[3]=0x41; glyph[4]=0x22; break;
    case 'D': glyph[0]=0x7F; glyph[1]=0x41; glyph[2]=0x41; glyph[3]=0x22; glyph[4]=0x1C; break;
    case 'E': glyph[0]=0x7F; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x41; break;
    case 'F': glyph[0]=0x7F; glyph[1]=0x09; glyph[2]=0x09; glyph[3]=0x09; glyph[4]=0x01; break;
    case 'G': glyph[0]=0x3E; glyph[1]=0x41; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x7A; break;
    case 'H': glyph[0]=0x7F; glyph[1]=0x08; glyph[2]=0x08; glyph[3]=0x08; glyph[4]=0x7F; break;
    case 'I': glyph[0]=0x00; glyph[1]=0x41; glyph[2]=0x7F; glyph[3]=0x41; glyph[4]=0x00; break;
    case 'J': glyph[0]=0x20; glyph[1]=0x40; glyph[2]=0x41; glyph[3]=0x3F; glyph[4]=0x01; break;
    case 'K': glyph[0]=0x7F; glyph[1]=0x08; glyph[2]=0x14; glyph[3]=0x22; glyph[4]=0x41; break;
    case 'L': glyph[0]=0x7F; glyph[1]=0x40; glyph[2]=0x40; glyph[3]=0x40; glyph[4]=0x40; break;
    case 'M': glyph[0]=0x7F; glyph[1]=0x02; glyph[2]=0x0C; glyph[3]=0x02; glyph[4]=0x7F; break;
    case 'N': glyph[0]=0x7F; glyph[1]=0x04; glyph[2]=0x08; glyph[3]=0x10; glyph[4]=0x7F; break;
    case 'O': glyph[0]=0x3E; glyph[1]=0x41; glyph[2]=0x41; glyph[3]=0x41; glyph[4]=0x3E; break;
    case 'P': glyph[0]=0x7F; glyph[1]=0x09; glyph[2]=0x09; glyph[3]=0x09; glyph[4]=0x06; break;
    case 'Q': glyph[0]=0x3E; glyph[1]=0x41; glyph[2]=0x51; glyph[3]=0x21; glyph[4]=0x5E; break;
    case 'R': glyph[0]=0x7F; glyph[1]=0x09; glyph[2]=0x19; glyph[3]=0x29; glyph[4]=0x46; break;
    case 'S': glyph[0]=0x46; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x31; break;
    case 'T': glyph[0]=0x01; glyph[1]=0x01; glyph[2]=0x7F; glyph[3]=0x01; glyph[4]=0x01; break;
    case 'U': glyph[0]=0x3F; glyph[1]=0x40; glyph[2]=0x40; glyph[3]=0x40; glyph[4]=0x3F; break;
    case 'V': glyph[0]=0x1F; glyph[1]=0x20; glyph[2]=0x40; glyph[3]=0x20; glyph[4]=0x1F; break;
    case 'W': glyph[0]=0x7F; glyph[1]=0x20; glyph[2]=0x18; glyph[3]=0x20; glyph[4]=0x7F; break;
    case 'X': glyph[0]=0x63; glyph[1]=0x14; glyph[2]=0x08; glyph[3]=0x14; glyph[4]=0x63; break;
    case 'Y': glyph[0]=0x07; glyph[1]=0x08; glyph[2]=0x70; glyph[3]=0x08; glyph[4]=0x07; break;
    case 'Z': glyph[0]=0x61; glyph[1]=0x51; glyph[2]=0x49; glyph[3]=0x45; glyph[4]=0x43; break;
    case '0': glyph[0]=0x3E; glyph[1]=0x51; glyph[2]=0x49; glyph[3]=0x45; glyph[4]=0x3E; break;
    case '1': glyph[0]=0x00; glyph[1]=0x42; glyph[2]=0x7F; glyph[3]=0x40; glyph[4]=0x00; break;
    case '2': glyph[0]=0x62; glyph[1]=0x51; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x46; break;
    case '3': glyph[0]=0x22; glyph[1]=0x41; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x36; break;
    case '4': glyph[0]=0x18; glyph[1]=0x14; glyph[2]=0x12; glyph[3]=0x7F; glyph[4]=0x10; break;
    case '5': glyph[0]=0x2F; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x31; break;
    case '6': glyph[0]=0x3E; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x32; break;
    case '7': glyph[0]=0x01; glyph[1]=0x71; glyph[2]=0x09; glyph[3]=0x05; glyph[4]=0x03; break;
    case '8': glyph[0]=0x36; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x36; break;
    case '9': glyph[0]=0x26; glyph[1]=0x49; glyph[2]=0x49; glyph[3]=0x49; glyph[4]=0x3E; break;
    case ':': glyph[0]=0x00; glyph[1]=0x36; glyph[2]=0x36; glyph[3]=0x00; glyph[4]=0x00; break;
    case '.': glyph[0]=0x00; glyph[1]=0x60; glyph[2]=0x60; glyph[3]=0x00; glyph[4]=0x00; break;
    case '-': glyph[0]=0x08; glyph[1]=0x08; glyph[2]=0x08; glyph[3]=0x08; glyph[4]=0x08; break;
    case '/': glyph[0]=0x20; glyph[1]=0x10; glyph[2]=0x08; glyph[3]=0x04; glyph[4]=0x02; break;
    case '%': glyph[0]=0x62; glyph[1]=0x64; glyph[2]=0x08; glyph[3]=0x13; glyph[4]=0x23; break;
    default:  glyph[0]=0x00; glyph[1]=0x00; glyph[2]=0x00; glyph[3]=0x00; glyph[4]=0x00; break;
  }
}

void OledSSD1306::initDisplay()
{
  const uint8_t mux = (_height == 32) ? 0x1F : 0x3F;
  const uint8_t compins = (_height == 32) ? 0x02 : 0x12;

  static const uint8_t initSeq[] = {
    0xAE, 0x20, 0x02, 0x40, 0xA1, 0xC8, 0x81, 0x7F,
    0xA6, 0xD3, 0x00, 0xD5, 0x80,
  };

  for (size_t i = 0; i < sizeof(initSeq); i++)
  {
    writeCommand(initSeq[i]);
  }

  writeCommand(0xA8);
  writeCommand(mux);
  writeCommand(0xD9);
  writeCommand(0xF1);
  writeCommand(0xDA);
  writeCommand(compins);
  writeCommand(0xDB);
  writeCommand(0x40);
  writeCommand(0x8D);
  writeCommand(0x14);
  writeCommand(0xAF);
}

} // namespace Espfc::Device
