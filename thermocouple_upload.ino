/*
   sketch.ino

   Author:   Hiromasa Ihara (taisyo)
   Created:  2016-02-18
*/

#include <Ethernet.h>
#include <EthernetUdp.h>

#include <PFFIAPUploadAgent.h>
#include <TimeLib.h>
#include <LocalTimeLib.h>
#include <SerialCLI.h>
#include <ADT74x0.h>
#include "SHT2x.h"
#include <Adafruit_AM2315.h>
#include <NTP.h>
#include <Wire.h>

#define GLED 22
#define RLED 23
#define PMOS_PWM 5
#define NMOS_PWM 6

//cli
SerialCLI      commandline(Serial);
//  ethernet
MacEntry       mac("MAC", "B0:12:66:01:02:8A", "mac address");
//  ip
BoolEntry      dhcp("DHCP", "true", "DHCP enable/disable");
IPAddressEntry ip("IP", "192.168.0.2", "IP address");
IPAddressEntry gw("GW", "192.168.0.1", "default gateway IP address");
IPAddressEntry sm("SM", "255.255.255.0", "subnet mask");
IPAddressEntry dns_server("DNS", "8.8.8.8", "dns server");
//  ntp
StringEntry    ntp("NTP", "ntp.nict.jp", "ntp server");
//  fiap
StringEntry    host("HOST", "202.15.110.21", "host of ieee1888 server end point");
IntegerEntry   port("PORT", "80", "port of ieee1888 server end point");
StringEntry    path("PATH", "/axis2/services/FIAPStorage", "path of ieee1888 server end point");
StringEntry    prefix("PREFIX", "http://iwata.com/thermocouple/", "prefix of point id");
//  debug
int debug = 0;

//ntp
NTPClient ntpclient;
TimeZone localtimezone = { 9 * 60 * 60, 0, "+09:00" };

//fiap
FIAPUploadAgent fiap_upload_agent;
char sht_str[16];
char humi_str[16];
char adt_str[16];
char amt_str[16];
char amh_str[16];
char pow_str[16];
char vol_str[16];
struct fiap_element fiap_elements [] = {
  //  { "SHT21_Heating_Temperature", sht_str, 0, &localtimezone, },
  //  { "SHT21_Heating_Humidity", humi_str, 0, &localtimezone, },
  { "ADT7410_Air_Temperature", adt_str, 0, &localtimezone, },
  { "AM2315_Temperature", amt_str, 0, &localtimezone, },
  { "AM2315_Humidity", amh_str, 0, &localtimezone, },
  { "3WSolarPowerProduction", pow_str, 0, &localtimezone, },
  { "3WSolarSupplyVoltage", vol_str, 0, &localtimezone, },
};

//sensor
ADT74x0 adt_sensor;
SHT2x sht_sensor;
Adafruit_AM2315 am2315;

void enable_debug()
{
  debug = 1;
}

void disable_debug()
{
  debug = 0;
}

void setup()
{
  int ret;
  commandline.add_entry(&mac);

  commandline.add_entry(&dhcp);
  commandline.add_entry(&ip);
  commandline.add_entry(&gw);
  commandline.add_entry(&sm);
  commandline.add_entry(&dns_server);

  commandline.add_entry(&ntp);

  commandline.add_entry(&host);
  commandline.add_entry(&port);
  commandline.add_entry(&path);
  commandline.add_entry(&prefix);

  commandline.add_command("debug", enable_debug);
  commandline.add_command("nodebug", disable_debug);

  commandline.begin(9600, "ADT74x0 Gateway");

  // ethernet & ip connection
  if (dhcp.get_val() == 1) {
    ret = Ethernet.begin(mac.get_val());
    if (ret == 0) {
      restart("Failed to configure Ethernet using DHCP", 10);
    }
  } else {
    Ethernet.begin(mac.get_val(), ip.get_val(), dns_server.get_val(), gw.get_val(), sm.get_val());
  }

  // fetch time
  uint32_t unix_time;
  ntpclient.begin();
  ret = ntpclient.getTime(ntp.get_val(), &unix_time);
  if (ret < 0) {
    restart("Failed to configure time using NTP", 10);
  }
  setTime(unix_time);

  // fiap
  fiap_upload_agent.begin(host.get_val(), path.get_val(), port.get_val(), prefix.get_val());

  // sensor
  Wire.begin();
  adt_sensor.begin(0x48);
  sht_sensor.begin();
  am2315.begin();

  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  digitalWrite(RLED, HIGH); // LED on when pin is High
  //  analogWrite(NMOS_PWM, 64); // PWM:1/4
  analogWrite(NMOS_PWM, 128); // PWM:1/2
}

void loop()
{
  static unsigned long old_epoch = 0, epoch;

  commandline.process();
  epoch = now();
  if (dhcp.get_val() == 1) {
    Ethernet.maintain();
  }

  if (epoch != old_epoch) {
    char buf[48];
    char i_buf[8];
    char v_buf[8];
    char p_buf[8];
    int A0value = analogRead(A0); // Solar cuurent through PMOS (1ohm) 100 times amplified
    sprintf(buf, "A0=%4d, I=", A0value);
    double solar_current = double(A0value * 5) / 102300.0; // I = 100 x 5 / 1023 / 100 = 0.005[A]
    dtostrf(solar_current * 1000, -1, 1, i_buf);
    strcat(buf, i_buf);
    strcat(buf, "[mA], V=");
    int A1value = analogRead(A1); // Solar voltage 10V -> 5V 5V->1023
    double solar_voltage = double(A1value * 10) / 1023.0; // actual voltage
    dtostrf(solar_voltage, -1, 2, v_buf);
    strcat(buf, v_buf);
    strcat(buf, "[V], P=");
    double solar_power   = solar_current * solar_voltage; // P = I x V
    dtostrf(solar_power, -1, 4, p_buf);
    strcat(buf, p_buf);
    strcat(buf, "[W].");
    debug_msg(buf);

    if (epoch % 60 == 0) {
      debug_msg("uploading...");
      //      sht_sensor.heaterOn();
      dtostrf(solar_power, -1, 4, pow_str);
      debug_msg(pow_str);
      dtostrf(solar_voltage, -1, 2, vol_str);
      debug_msg(vol_str);
      dtostrf(adt_sensor.readTemperature(), -1, 2, adt_str);
      debug_msg("ADT7410 temp");
      debug_msg(adt_str);
      //      dtostrf(sht_sensor.readTemperature(), -1, 2, sht_str);
      //      debug_msg("SHT21 temp");
      //      debug_msg(sht_str);
      //      dtostrf(sht_sensor.readHumidity(), -1, 2, humi_str);
      //      debug_msg("SHT21 humi");
      //      debug_msg(humi_str);
      float am_temp, am_humi;
      if ( am2315.readTemperatureAndHumidity( am_temp, am_humi ) ) {
        dtostrf(am_temp, -1, 1, amt_str); // Accuracy is pm 0.1
        debug_msg("AM2315 temp");
        debug_msg(amt_str);
        dtostrf(am_humi, -1, 1, amh_str);
        debug_msg("AM2315 humi");
        debug_msg(amh_str);
      } else {
        debug_msg("[Error] Cannot get temperature and humidity by AM2315!");
      }

      for (int i = 0; i < sizeof(fiap_elements) / sizeof(fiap_elements[0]); i++) {
        fiap_elements[i].time = epoch;
      }
      int ret = fiap_upload_agent.post(fiap_elements, sizeof(fiap_elements) / sizeof(fiap_elements[0]));
      if (ret == 0) {
        debug_msg("done");
      } else {
        debug_msg("failed");
        Serial.println(ret);
      }
    }
  }

  old_epoch = epoch;
}

void debug_msg(String msg)
{
  if (debug == 1) {
    Serial.print("[");
    print_time();
    Serial.print("]");
    Serial.println(msg);
  }
}

void print_time()
{
  char print_time_buf[32];
  TimeElements* tm;
  tm = localtime();
  sprintf(print_time_buf, "%04d/%02d/%02d %02d:%02d:%02d",
          tm->Year + 1970, tm->Month, tm->Day, tm->Hour, tm->Minute, tm->Second);
  Serial.print(print_time_buf);
}

void restart(String msg, int restart_minutes)
{
  Serial.println(msg);
  Serial.print("This system will restart after ");
  Serial.print(restart_minutes);
  Serial.print("minutes.");

  unsigned int start_ms = millis();
  while (1) {
    commandline.process();
    if (millis() - start_ms > restart_minutes * 60UL * 1000UL) {
      commandline.reboot();
    }
  }
}
