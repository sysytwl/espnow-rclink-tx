#include <Arduino.h>
#include "EspNowRcLink/Transmitter.h"
#include "ppm.h"
#include <Preferences.h>
#include "Shell.h"
//https://docs.espressif.com/projects/esp-faq/en/latest/application-solution/esp-now.html


#define send_gap 20 * 1000 //micro scond

// uncomment to use ppm on pin 13
#define PPM_PIN 36

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

void setup(){
  Serial.begin(115200);

  preferences.begin("pair_data", false);
  preferences.getBytes("RxMacAddr", tx._peer, 6);
  preferences.end();

  Serial.printf("Send to: %x:%x:%x:%x:%x:%x\n",tx._peer[0], tx._peer[1], tx._peer[2], tx._peer[3], tx._peer[4], tx._peer[5]); //dc:54:75:ed:70:a8
  Serial.println(sizeof(tx._peer));
  shell_init(shell_reader, shell_writer, "controller, type <mac aa aa aa aa aa aa>");
  shell_register(mac, "mac");

#ifdef PPM_PIN
  ppm.begin(PPM_PIN, FALLING);
#endif

  WiFi.mode(WIFI_STA);
  WiFi.channel(3);
  tx.begin(false);
}

long next_send = 0;
void loop(){
  shell_task();
  if(next_send < micros()){
#ifdef PPM_PIN
    if(ppm.available()){
      for(size_t c = 0; c < 8; c++){
        const int16_t val = ppm.get(c);
        tx.setChannel(c, val);
      }
      tx.commit();
    }
#else
    tx.setChannel(0, map(analogRead(32), 0, 4096, 880, 2120));
    tx.setChannel(1, map(analogRead(33), 0, 4096, 880, 2120));
    tx.setChannel(2, map(analogRead(35), 0, 4096, 880, 2120));
    tx.setChannel(3, map(analogRead(34), 0, 4096, 880, 2120));
    tx.commit();
#endif
    next_send = micros() + send_gap;
  }
  tx.update();
}