#pragma once

// RP2350B reuses RP2350 pinout and peripheral defaults from the RP target.
#include "TargetRP2040.h"

#define ESPFC_TARGET_RP2350B

#if defined(ESPFC_RP2350_RISCV)
  // Hazard3 mode is selected by PlatformIO board_build.mcu = rp2350-riscv.
  #define ESPFC_TARGET_RP2350_HAZARD3
#endif

#if defined(ESPFC_RP2350_OVERCLOCK)
  // Optional profile marker for overclocked RP2350B builds.
  #define ESPFC_TARGET_RP2350_OVERCLOCK
#endif

#if defined(ESPFC_RP2350_PIO_OFFLOAD)
  // Optional profile marker for RP2350B PIO-offload experiments.
  #define ESPFC_TARGET_RP2350_PIO_OFFLOAD
#endif
