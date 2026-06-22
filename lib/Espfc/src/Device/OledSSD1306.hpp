#pragma once

#include "Device/OledDevice.hpp"
#include "Model.h"

namespace Espfc::Device {

class OledSSD1306 : public OledDevice
{
public:
  int begin(BusDevice* bus) final;
  int begin(BusDevice* bus, uint8_t addr) final;
  void setHeight(uint8_t h);
  void setPageInterval(int32_t intervalMs);
  void setStartupDuration(int32_t durationMs);
  void update(const Model& model);

  DeviceType getType() const final { return OLED_SSD1306; }
  bool testConnection() final;

private:
  static constexpr uint8_t OLED_WIDTH = 128;
  static constexpr uint8_t OLED_MAX_LINES = 16;
  static constexpr uint8_t OLED_MAX_CHARS = 22;

  static constexpr uint8_t ADDRESS_0 = 0x3C;
  static constexpr uint8_t ADDRESS_1 = 0x3D;

  bool writeCommand(uint8_t cmd);
  bool writeData(const uint8_t* data, uint8_t len);
  void setCursor(uint8_t page, uint8_t col);
  void clearDisplay();
  void drawTextLine(uint8_t page, const char* text);
  void composeLines(const Model& model, char lines[OLED_MAX_LINES][OLED_MAX_CHARS], uint8_t& lineCount) const;
  void renderStartupPage(uint8_t totalPages);
  void renderPage(const char lines[OLED_MAX_LINES][OLED_MAX_CHARS], uint8_t lineCount, uint8_t pageIndex);
  void getGlyph(char c, uint8_t glyph[5]) const;
  void initDisplay();

  uint8_t _height = 64;
  uint8_t _pages = 8;
  uint16_t _pageIntervalMs = 3000;
  uint16_t _startupDurationMs = 1500;
  uint32_t _lastRenderUs = 0;
  uint32_t _lastPageSwitchMs = 0;
  uint32_t _startupUntilMs = 0;
  uint8_t _currentPage = 0;
};

} // namespace Espfc::Device
