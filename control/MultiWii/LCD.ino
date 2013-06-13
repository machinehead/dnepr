// ************************************************************************************************************
// LCD & display & monitoring
// ************************************************************************************************************
// in any of the following cases an LCD is required and
// the primitives for exactly one of the available types are setup
#if  defined(HAS_LCD)

/* ------------------------------------------------------------------ */
void LCDprint(uint8_t i) {
    SerialWrite(LCD_SERIAL_PORT, i );
}

void LCDprintChar(const char *s) {
  while (*s) {LCDprint(*s++);}
}

void LCDcrlf() {
  LCDprintChar("\r\n");
}
void LCDclear() {
    LCDcrlf();
}

void LCDsetLine(byte line) { // Line = 1 or 2 - vt100 has lines 1-99
    LCDcrlf();
}

#define LCD_FLUSH {/*UartSendData();*/ delayMicroseconds(20000); }

void initLCD() {
  blinkLED(20,30,1);
  #if defined(BUZZER)
    alarmArray[7] = 1;
  #endif
}
#endif //Support functions for LCD_CONF and LCD_TELEMETRY

// -------------------- telemetry output to LCD over serial/i2c ----------------------------------

