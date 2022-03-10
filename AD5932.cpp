#include "Arduino.h"
#include "AD5932.h"
#include <SPI.h>

AD5932::AD5932(byte fsync, byte standby, byte interrupt, byte ctrl, byte syncout, byte msbout) : p_fsync{fsync}, p_standby{standby}, p_interrupt{interrupt}, p_ctrl{ctrl}, p_syncout{syncout}, p_msbout{msbout} {
  pinMode(p_fsync, OUTPUT);
  pinMode(p_standby, OUTPUT);
  pinMode(p_interrupt, OUTPUT);
  pinMode(p_ctrl, OUTPUT);
  pinMode(p_syncout, OUTPUT);
  pinMode(p_msbout, OUTPUT);
  SPI.begin();
  digitalWrite(p_fsync, HIGH);
  data.control_reg = CONTROL;
  data.number_of_inc_reg = NINC;
  data.lower_delta_reg = LF;
  data.upper_delta_reg = HF;
  data.inc_int_reg = INCINTERVAL;
  data.lower_start_reg = LSF;
  data.upper_start_reg = HSF;
}

/*
  AD5932::AD5932(byte standby, byte interrupt, byte ctrl, byte syncout, byte msbout){
    #if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
        AD5932(10, standby, interrupt, ctrl, syncout, msbout);
    #elif defined(__AVR_MEGA2560__) || defined(__AVR_MEGA1280__) || (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        AD5932(53, standby, interrupt, ctrl, syncout, msbout);
    #elif defined(__Arduino_Nano_RP2040_Connect__) || defined(__NANO_RP2040_CONNECT__)
        #error "You need to provide the FSYNC (Chip Select) PIN of the SPI bus"
    #else
      #warning "Your Board Type is not specified in this library contact maintainer"
    #endif
  }
*/

void AD5932::reset() {
  digitalWrite(p_interrupt, HIGH);
  delay(10);
  digitalWrite(p_interrupt, LOW);
}

void AD5932::trigger() {
  digitalWrite(p_ctrl, HIGH);
  delay(10);
  digitalWrite(p_ctrl, LOW);
}

void AD5932::powerdown() {
  reset();
  digitalWrite(p_standby, HIGH);
}

void AD5932::wakeup() {
  digitalWrite(p_standby, LOW);
  update_settings();
}

void AD5932::set_start_frequency(unsigned long frequency) {
  data.lower_start_reg |= (0x0FFF & frequency);
  data.upper_start_reg |= ((0x0FFF000 & frequency) >> 12);
  transfer_register(LSF);
  transfer_register(HSF);
}

void AD5932::set_delta_frequency(long delta_frequency) {
  data.lower_delta_reg |= (0x0FFF & (delta_frequency < 0 ? (-delta_frequency) : delta_frequency));
  if (delta_frequency < 0) {
    data.upper_delta_reg |= 0x0800;
    data.upper_delta_reg |= ((0x07FF000 & (-delta_frequency)) >> 12);
  } else {
    data.upper_delta_reg |= ((0x07FF000 & delta_frequency) >> 12);
  }
  transfer_register(LF);
  transfer_register(HF);
}

void AD5932::set_multiplier(MULTIPLIER multiplier) {
  data.inc_int_reg |= ((multiplier & 0x03) << 11);
  transfer_register(INCINTERVAL);
}

void AD5932::set_trigger_mode(TRIGGER_MODE mode) {
  data.control_reg |= (mode << 5);
  transfer_register(CONTROL);
}

void AD5932::set_waveform(WAVEFORM wave) {
  data.control_reg |= (wave << 9);
  transfer_register(CONTROL);
}

void AD5932::set_increment_interval_mode(INCREMENT_BY mode) {
  data.inc_int_reg |= (mode << 13);
  transfer_register(INCINTERVAL);
}

void AD5932::set_increment_interval(unsigned int inc) {
  data.inc_int_reg |= (0x03FF & inc);
  transfer_register(INCINTERVAL);
}

void AD5932::set_number_of_increments(unsigned int n) {
  if (n < 2) {
    n = 2;
  }
  data.number_of_inc_reg |= (n & 0x0FFF);
  transfer_register(NINC);
}

void AD5932::set_msbout(bool enable) {
  data.control_reg |= (enable << 8);
  transfer_register(CONTROL);
}

void AD5932::set_syncselect(SYNC_MODE mode) {
  set_syncout(true);
  data.control_reg |= (mode << 3);
  transfer_register(CONTROL);
}

void AD5932::set_syncout(bool enable) {
  data.control_reg |= (enable << 2);
  transfer_register(CONTROL);
}

unsigned long AD5932::get_start_frequency() {
  unsigned long start_freq;
  start_freq &= 0x00000000;
  start_freq = ((data.upper_start_reg << 12) & 0x0FFF000);
  start_freq |= (data.lower_start_reg & 0x0FFF);
  return start_freq;
}

long AD5932::get_delta_frequency() {
  long delta_freq = 0;
  delta_freq = (((((long)data.upper_delta_reg) & 0x7FF) << 12) | (((long) data.lower_delta_reg) & 0xFFF));
  delta_freq *= ((data.upper_delta_reg & 0x0800) == 0x0800) ? -1 : 1;
  return delta_freq;
}

int AD5932::get_multiplier() {
  switch (((data.inc_int_reg >> 11) & 0x03)) {
    case x1:
      return 1;
    case x5:
      return 5;
    case x100:
      return 100;
    case x500:
      return 500;
    default:
      return 0;
  }
  return 0;
}

unsigned int AD5932::get_number_of_increments() {
  return (data.number_of_inc_reg & 0x0FFF);
}

void AD5932::transfer_settings_updates_on_change(bool b) {
  update_on_change = b;
}

void AD5932::update_settings() {
  bool h = update_on_change;
  update_on_change = true;
  for (int i = 0; i < 7; i++) {
    transfer_register(all_addresses[i]);
  }
  update_on_change = h;
}

void AD5932::print_config() {
  Serial.println("\nControl\n");
  print_register(CONTROL);
  Serial.println("\nNumber of Increments\n");
  print_register(NINC);
  Serial.println("\nLower 12 bits of delta frequency\n");
  print_register(LF);
  Serial.println("\nHigher 12 bits of delta frequency\n");
  print_register(HF);
  Serial.println("\nIncrement Interval\n");
  print_register(INCINTERVAL);
  Serial.println("\nLower 12 bits of start frequency\n");
  print_register(LSF);
  Serial.println("\nHigher 12 bits of start frequency\n");
  print_register(HSF);
  Serial.println();
}

AD5932_DATA_STRUCT* AD5932::getDataStructPointer() {
  return &data;
}

int AD5932::pack_register(AD5932_ADDRESS address) {
  int reg = address;
  switch (address) {
    case CONTROL:
      reg |= data.control_reg;
      //reg |= (data.B24 << 11);
      //reg |= (data.DACEN << 10);
      //reg |= (data.SINE << 9);
      //reg |= (data.MSBOUT << 8);
      //reg |= (data.INCR << 5);
      //reg |= (data.SYNCSEL << 3);
      //reg |= (data.SYNCOUTEN << 2);
      return reg;
    case NINC:
      reg |= data.number_of_inc_reg;
      //reg |= data.number_of_increments;
      return reg;
    case LF:
      reg |= data.lower_delta_reg;
      //reg |= (data.frequency_inc & 0x00000FFF);
      return reg;
    case HF:
      reg |= data.upper_delta_reg;
      //reg |= ((data.frequency_inc & 0x00FFF000) >> 12);
      return reg;
    case INCINTERVAL:
      reg |= data.inc_int_reg;
      //reg |= (data.inc_on_clock_periods << 13);
      //reg |= ((data.multiplier & 0x03) << 11);
      //reg |= (data.increment_interval & 0x7FF);
      return reg;
    case LSF:
      reg |= data.lower_start_reg;
      //reg |= (data.start_frequency & 0x00000FFF);
      return reg;
    case HSF:
      reg |= data.upper_start_reg;
      //reg |= ((data.start_frequency & 0x00FFF000) >> 12);
      return reg;
    default:
      reg |= 0xFFFF;
      return reg;
  }
}

void AD5932::transfer_register(AD5932_ADDRESS address) {
  if (update_on_change) {
    write_to_chip(pack_register(address));
  }
}

void AD5932::print_register(AD5932_ADDRESS address) {
  for (int j = 0; j < 2; j++) {
    int d_register = pack_register(address);
    for (int i = 15; i >= 0; i--) {
      if (j == 0) {
        Serial.print("D");
        Serial.print(i, DEC);
      } else {
        bool bit = ((d_register & (1 << i)) >> max(i - 1, 0));
        Serial.print(bit, DEC);
      }
      Serial.print("\t");
    }
    Serial.println("");
  }
}

void AD5932::write_to_chip(int data) {
  SPISettings settings(14000000, MSBFIRST, SPI_MODE1);
  SPI.beginTransaction(settings);
  digitalWrite(p_fsync, LOW);
  SPI.transfer16(data);
  digitalWrite(p_fsync, HIGH);
  SPI.endTransaction();
}
