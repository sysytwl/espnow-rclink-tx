#include <Arduino.h>
#include "EspNowRcLink/Transmitter.h"
#include "ppm.h"
#include <Preferences.h>
#include "Shell.h"



#define send_gap 20 * 1000 //micro scond

// uncomment to activate simultor on channel 3
//#define SIM_TX_INTERVAL_MS (20 * 1000) // 20ms => 50Hz

// uncomment to use ppm on pin 13
//#define PPM_PIN 13

// uncomment to print some details to console
//#define PRINT_INFO

Preferences preferences;

EspNowRcLink::Transmitter tx;
#ifdef PPM_PIN
PPM ppm;
#endif

int shell_reader(char * data){
  // Wrapper for Serial.read() method
  if (Serial.available()) {
    *data = Serial.read();
    return 1;
  }
  return 0;
}

void shell_writer(char data){
  // Wrapper for Serial.write() method
  Serial.write(data);
}

int mac(int argc, char** argv){
  if(argc < 7){
    shell_print_error(E_SHELL_ERR_ARGCOUNT, 0);
    shell_println("");
    return SHELL_RET_FAILURE;
  }

  for(int i=1; i<7; i++)
    tx._peer[i-1] = strtol(argv[i], NULL, 16);

  preferences.begin("pair_data", false);
  preferences.putBytes("RxMacAddr", tx._peer, 6);
  preferences.end();

  ESP.restart();
  return SHELL_RET_SUCCESS;
}

void setup()
{
  Serial.begin(115200);

  preferences.begin("pair_data", false);
  preferences.getBytes("RxMacAddr", tx._peer, 6);
  preferences.end();

  Serial.printf("Send to: %x:%x:%x:%x:%x:%x\n",tx._peer[0], tx._peer[1], tx._peer[2], tx._peer[3], tx._peer[4], tx._peer[5]); //dc:54:75:ed:70:a8

  shell_init(shell_reader, shell_writer, "controller,type <mac FF FF FF FF FF FF>");
  shell_register(mac, "mac");

#ifdef PPM_PIN
  ppm.begin(PPM_PIN, FALLING);
#endif

  tx.begin(true);
}



long next_send = 0;
void loop()
{
  shell_task();

  uint32_t now = micros();
  static int v = 0;
  static uint32_t delta = 0;

  if(next_send < micros()){
    tx.setChannel(0, map(analogRead(32), 0, 4096, 880, 2120));
    tx.setChannel(1, map(analogRead(33), 0, 4096, 880, 2120));
    tx.setChannel(2, map(analogRead(35), 0, 4096, 880, 2120));
    tx.setChannel(3, map(analogRead(34), 0, 4096, 880, 2120));
    tx.commit();
    next_send = micros() + send_gap;
  }

#ifdef PPM_PIN
  static uint32_t lastSent = 0;
  uint32_t sentPeriod = (now - lastSent);
  if(ppm.available() || sentPeriod >= 50000ul)
  {
    lastSent = now;
    delta = sentPeriod;
    for(size_t c = 0; c < 8; c++)
    {
      const int16_t val = ppm.get(c);
      tx.setChannel(c, val);
      if(c == 2) v = val;
    }
    tx.commit();
    //Serial.println(v);
  }
#endif

#ifdef SIM_TX_INTERVAL_MS
  static int sim_val = 0;
  static bool sim_dir = true;
  const int sim_rate = 4;

  static uint32_t sendNext = now + SIM_TX_INTERVAL_MS;

  // send rc channels
  if(now >= sendNext)
  {
    v = 1000 + sim_val;
    tx.setChannel(2, v);
    tx.commit();
    sendNext = now + TX_INTERVAL_MS;

    if(sim_dir)
    {
      sim_val += sim_rate;
      if(sim_val >= 1000) sim_dir = false;
    }
    else
    {
      sim_val -= sim_rate;
      if(sim_val <= 0) sim_dir = true;
    }
  }
#endif

  tx.update();

#ifdef PRINT_INFO
  static uint32_t printNext = now + 500000;
  if(now >= printNext)
  {
    Serial.printf("V: %d, P: %d, D: %d, C: %d\n", v, ppm.get(2), delta / 100, WiFi.channel());
    printNext = now + 500000;
  }
#else
  (void)v;
  (void)delta;
#endif
}