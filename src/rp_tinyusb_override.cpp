#if defined(ARCH_RP2040) && defined(USE_TINYUSB)

extern "C" void TinyUSB_Port_EnterDFU(void)
{
  // Disable touch-1200 DFU auto-reset for RP2040 runtime stability with Configurator.
}

#endif
