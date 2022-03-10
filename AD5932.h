
#ifndef AD5932_h
#define AD5932_h
#include "Arduino.h"



typedef byte MULTIPLIER;
typedef int AD5932_ADDRESS;
typedef bool TRIGGER_MODE;
typedef bool WAVEFORM;
typedef bool INCREMENT_BY;
typedef bool SYNC_MODE;

const inline char* UUID_WAVEFORM = "cb68774e-5438-4c5a-8937-814be0a249bb";
const inline char* UUID_MULTIPLIER = "cd98f4b7-df60-4d9c-9c87-6a64e19a7c4f";
const inline char* UUID_INCREMENT_BY = "f2207ee2-1b29-419f-8f7d-b079db007255";
const inline char* UUID_SYNC_MODE = "3aaf6709-7f59-4e9c-8041-ef31f8b20791";
const inline char* UUID_TRIGGER_MODE = "1c54116d-d6aa-4d1f-96a4-5827842c3e4f";
const inline char* UUID_START_FREQ = "cbd047db-ed1e-4523-bf97-f29f5b4f4c8d";
const inline char* UUID_INC_FREQ = "e146d40a-6fdf-4887-86e1-0c4875b349e6";
const inline char* UUID_INCN = "6ba8446b-5711-43f6-98e6-2bd314fa56a3";
const inline char* UUID_MSBOUT = "072585df-390b-434e-ba25-116533274d3e";
const inline char* UUID_SYNCOUT = "6a2676fa-20e0-48a7-9cd6-560a48013bfe";
const inline char* UUID_POWERSAVING = "9988258d-5663-4e1f-a4bc-a8e211f98e39";


const INCREMENT_BY BY_OUTPUT_FREQUENCY = 0;
const INCREMENT_BY BY_MCLK_PERIODS = 1;

const WAVEFORM SINE = 1;
const WAVEFORM TRIANGULAR = 0;

const MULTIPLIER x1 = 0x00;
const MULTIPLIER x5 = 0x01;
const MULTIPLIER x100 = 0x02;
const MULTIPLIER x500 = 0x03;

const SYNC_MODE SYNC_AT_FRQ_INC = 0;
const SYNC_MODE SYNC_AT_EOS = 1;

const TRIGGER_MODE AUTO_INCREMENT = 0;
const TRIGGER_MODE EXTERNAL_INCREMENT = 1;

const AD5932_ADDRESS CONTROL = 0x00D3;      //0000 0000   Control Bits
const AD5932_ADDRESS NINC = 0x1000;         //0000 0001   Number of Increments
const AD5932_ADDRESS LF = 0x2000;           //0000 0010   Lower 12 Bits of delta Frequency
const AD5932_ADDRESS HF = 0x3000;           //0000 0011   HIGHER 12 Bits of delta Frequency
const AD5932_ADDRESS INCINTERVAL = 0x4000;  //0000 0100   Increment Interval
const AD5932_ADDRESS LSF = 0xC000;          //0000 1100   Lower 12 Bits of start Frequency
const AD5932_ADDRESS HSF = 0xD000;          //0000 1101   Higher 12 Bits of start Frequency

struct __attribute__((__packed__)) AD5932_DATA_STRUCT {
  short control_reg;
  short number_of_inc_reg;
  short lower_delta_reg;
  short upper_delta_reg;
  short inc_int_reg;
  short lower_start_reg;
  short upper_start_reg;
};

// the #include statment and code go here...
class AD5932
{
  public:
    AD5932(byte fsync, byte standby, byte interrupt, byte ctrl, byte syncout, byte msbout);
    //AD5932(byte standby, byte interrupt, byte ctrl, byte syncout, byte msbout);
    void reset();
    void trigger();
    void powerdown();
    void wakeup();
    void set_start_frequency(unsigned long frequency);
    void set_delta_frequency(long delta_frequency);
    void set_multiplier(MULTIPLIER multiplier);
    void set_trigger_mode(TRIGGER_MODE mode);
    void set_waveform(WAVEFORM wave);
    void set_increment_interval_mode(INCREMENT_BY mode);
    void set_increment_interval(unsigned int inc);
    void set_number_of_increments(unsigned int n);
    void set_msbout(bool enable);
    void set_syncselect(SYNC_MODE mode);
    void set_syncout(bool enable);
    unsigned long get_start_frequency();
    long get_delta_frequency();
    int get_multiplier();
    unsigned int get_number_of_increments();
    void transfer_settings_updates_on_change(bool b);
    void update_settings();
    void print_config();
    AD5932_DATA_STRUCT* getDataStructPointer();
  private:
    struct AD5932_DATA_STRUCT data;
    byte p_fsync, p_standby, p_interrupt, p_ctrl, p_syncout, p_msbout;
    bool update_on_change;
    AD5932_ADDRESS all_addresses[7] = {CONTROL, NINC, LF, HF, INCINTERVAL, LSF, HSF};
    int pack_register(AD5932_ADDRESS address);
    void transfer_register(AD5932_ADDRESS address);
    void print_register(AD5932_ADDRESS address);
    void write_to_chip(int data);
};
#endif
