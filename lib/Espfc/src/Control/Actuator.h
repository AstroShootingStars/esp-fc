#pragma once

#include "Model.h"

namespace Espfc::Control {

class Actuator
{
  public:
    Actuator(Model& model);

    int begin();
    int update();

  #ifndef UNIT_TEST
  private:
  #endif

    void updateScaler();
    void updateAdjustments();
    void updateArmingDisabled();
    void updateModeMask();
    bool canActivateMode(FlightMode mode);
    void updateArmed();
    void updateAirMode();
    void updateBuzzer();
    void updateDynLpf();
    void updateRescueConfig();
    void updateLed();
    bool applyAdjustmentStep(uint8_t function, int8_t direction);
    bool applyAdjustmentSelection(uint8_t function, int8_t position);
    bool applyAdjustmentAbsolute(uint8_t stateIndex, uint8_t function, float input, int16_t center, int16_t scale);

    struct SliderMasterSnapshot
    {
      bool valid = false;
      uint8_t rate[3] = {0, 0, 0};
      uint8_t superRate[3] = {0, 0, 0};
      uint8_t pidP[3] = {0, 0, 0};
      uint8_t pidI[3] = {0, 0, 0};
      uint8_t pidD[3] = {0, 0, 0};
      int16_t pidF[3] = {0, 0, 0};
      uint8_t throttleExpo = 0;
      uint8_t feedForwardTransition = 0;
      uint8_t horizonStrength = 100;
    };

    Model& _model;
    int8_t _adjustmentPosition[ADJUSTMENT_RANGES_COUNT] = {0};
    uint32_t _adjustmentRepeatMs[ADJUSTMENT_RANGES_COUNT] = {0};
    SliderMasterSnapshot _sliderMasterSnapshot[4];
};

}
