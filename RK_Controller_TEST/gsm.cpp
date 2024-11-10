#include "rc.h"

#define GSM_RX              13
#define GSM_TX              22
#define GSM_SMS_TIMEOUT  30000
#define GSM_READ_TIMEOUT    40
#define GSM_SMS_ATTEMPTS    10
#define GSM_ATTEMPT_FREQ  4000

extern String phone_number;

HardwareSerial gsm_uart(2);

bool gsm_status;
bool gsm_phone_call_detected = false;
SemaphoreHandle_t gsm_listener_lock = NULL;
long gsm_call_detector_cooldown = 10000;
long gsm_last_call_dected = -3600000;

String get_gsm_responce() {
  String s = "";
  while(gsm_uart.available()) {
    s += (char)gsm_uart.read();
    if(!gsm_uart.available()) delay(GSM_READ_TIMEOUT);
  }
  while(s.charAt(s.length() - 1) == '\n' or s.charAt(s.length() - 1) == '\r')
    s = s.substring(0, s.length() - 1);
  return s;
}

void SIM800L_listener(void*) {
  xSemaphoreTake(gsm_listener_lock, portMAX_DELAY);
  for(;;) {
    String s = get_gsm_responce();
    if(s == "2") {
      gsm_uart.println("AT+CLCC");
      long t = millis();
      while(!gsm_uart.available()) {
        if(millis() - t > 500) {
          log("SIM800L listener: Got \"2\" (should be a phone call), but \"AT+CLCC\" did not return anythyng (timeout arter 500 ms)");
          break;
        }
      }
      if(!gsm_uart.available()) continue;
      String r1 = get_gsm_responce();
      if(r1.startsWith("+CLCC:") && (phone_number != "" && r1.indexOf("\"" + phone_number + "\"") >= 0)) {
        gsm_uart.println("AT+HVOIC");
        while(!gsm_uart.available()) {
          if(millis() - t > 800) {
            log("SIM800L listener: Got a phone call, but \"AT+HVOIC\" did not return anythyng (timeout arter 800 ms)");
            break;
          }
          delay(10);
        }
        if(!gsm_uart.available()) continue;
        String r2 = get_gsm_responce();
        if(r2 == "0") log("SIM800L listener: Got a phone call");
        else log("SIM800L listener: Got a phone call, but \"AT+HVOIC\" returned \"" + r2 + "\"");
        long t = millis();
        if(t - gsm_last_call_dected >= gsm_call_detector_cooldown) gsm_phone_call_detected = true;
        else log("Cooldown has not been reached, ignoring phone call");
        gsm_last_call_dected = t;
      } else if(r1.startsWith("+CLCC:")) {
        log("SIM800L listener: Call from a wrong number: \"" + r1 + "\"");
        gsm_uart.println("AT+HVOIC");
        while(!gsm_uart.available()) {
          if(millis() - t > 800) {
            log("SIM800L listener: Got a phone call (from a wrong number), but \"AT+HVOIC\" did not return anythyng (timeout arter 800 ms)");
            break;
          }
          delay(10);
        }
        if(!gsm_uart.available()) continue;
        String r2 = get_gsm_responce();
        if(r2 == "0") log("SIM800L listener: Got a phone call (from a wrong number)");
        else log("SIM800L listener: Got a phone call (from a wrong number), but \"AT+HVOIC\" returned \"" + r2 + "\"");
      } else {
        log("SIM800L listener: Got \"2\" (should be a phone call), but \"AT+CLCC\" returned \"" + r1 + "\"");
      }
    } else if(s.length()) {
      log("SIM800L listener: Got unknown line: \"" + s + "\"");
    }
    xSemaphoreGive(gsm_listener_lock);
    delay(40);
    xSemaphoreTake(gsm_listener_lock, portMAX_DELAY);
  }
}
//extern float x;
bool __send_sms(String message) {
//  Serial.println(x);
  xSemaphoreTake(gsm_listener_lock, portMAX_DELAY);
  delay(120);
  long t = millis();
  if(phone_number == "0" || !phone_number.length()) {
    log("SMS fail reason: phone number is empty");
    xSemaphoreGive(gsm_listener_lock); return false;
  }
  while(gsm_uart.available()) gsm_uart.read();
  String cmd = "AT+CMGS=\"" + phone_number + "\"\n";
  gsm_uart.write(cmd.c_str(), cmd.length());
  while(!gsm_uart.available())
    if(millis() - t > GSM_SMS_TIMEOUT) {
      log("SMS fail reason: timeout while waiting for responce to the AT+CMGS command");
      xSemaphoreGive(gsm_listener_lock); return false;
    }
  String prompt = get_gsm_responce();
  prompt.trim();
  if(prompt != ">") {
    log("SMS fail reason: did not get prompt \">\" (got \"" + prompt + "\")");
    xSemaphoreGive(gsm_listener_lock); return false;
  }
  gsm_uart.write(message.c_str(), message.length());
  //  gsm_uart.write('\n');
  delayMicroseconds(500);
  gsm_uart.write((char)26);
//  gsm_listener_lock = false;
//  Serial.println(x);
//  return true;
  while(!gsm_uart.available()) {
    if(millis() - t > GSM_SMS_TIMEOUT) {
      log("SMS fail reason: timeout while waiting for responce after sending the message");
      xSemaphoreGive(gsm_listener_lock); return false;
    }
  }
  String invalid_responces = "";
  for(;;) {
    String s = get_gsm_responce();
    if(s.startsWith("+CMGS:") and s.substring(s.indexOf('\n') + 1).startsWith("0")) {
      xSemaphoreGive(gsm_listener_lock); return true;
    }
    if(s.indexOf("ERROR") >= 0) {
      log("SMS fail reason: got error code: \"" + s + "\"");
      xSemaphoreGive(gsm_listener_lock); return false;
    }
    if(s == "4") {
      log("SMS fail reason: got responce: \"" + s + "\" (problem with processing command)");
      xSemaphoreGive(gsm_listener_lock); return false;
    } else if(s == "2") {
      log("SMS fail reason: got responce: \"" + s + "\" (got an incoming signal (phone call))");
      xSemaphoreGive(gsm_listener_lock); return false;
    } else if(s == "1" || s == "3" || s == "6" || s == "7" || s == "8" || s == "9") {
      log("SMS fail reason: got responce: \"" + s + "\" (an error)");
      xSemaphoreGive(gsm_listener_lock); return false;
    }
    if(invalid_responces.length()) invalid_responces += ", ";
    invalid_responces += "\"" + s + "\"";
    while(!gsm_uart.available())
      if(millis() - t > GSM_SMS_TIMEOUT) {
        log("SMS fail reason: timeout while waiting for valid responce after sending the message (invalid responces: " + invalid_responces + ")");
        xSemaphoreGive(gsm_listener_lock); return false;
      }
  }
  xSemaphoreGive(gsm_listener_lock);
}

bool _send_sms(String message) {
  log((String)"Sending SMS with text \"" + message + "\"");
  for(int i = 1; i <= GSM_SMS_ATTEMPTS; i ++) {
    log((String)"Attempt " + i + " to send SMS with text \"" + message + "\"");
    if(__send_sms(message)) {
      log((String)"Attempt " + i + " to send SMS with text \"" + message + "\" succeeded");
      return true;
    }
    log((String)"Attempt " + i + " to send SMS with text \"" + message + "\" failed");
    delay(GSM_ATTEMPT_FREQ);
  }
  log((String)"SMS with text \"" + message + "\" failed");
  return false;
}

String sms_queue = "";
String failed_sms_queue = "";
//bool sms_queue_lock = false;
SemaphoreHandle_t sms_queue_lock = NULL;
SemaphoreHandle_t sms_lock = NULL;

extern String run_name;

void sms_sender(void *_) {
  for(;;) {
    bool waiting = true;
    String message;
    while(waiting) {
      xSemaphoreTake(sms_lock, portMAX_DELAY);
      xSemaphoreTake(sms_queue_lock, portMAX_DELAY);
      if(sms_queue.length()) {
        int i = sms_queue.indexOf((char)26);
        if(i >= 0) {
          message = sms_queue.substring(0, i);
          sms_queue = sms_queue.substring(i + 1);
        } else {
          message = sms_queue;
          sms_queue = "";
        }
        waiting = false;
      }
      xSemaphoreGive(sms_queue_lock);
    }
    if(!_send_sms(run_name + ": " + message)) {
      if(failed_sms_queue.length()) failed_sms_queue += (char)26;
      failed_sms_queue += message;
    }
    xSemaphoreGive(sms_lock);
  }
}

void send_sms(String message) {
  //return;
  xSemaphoreTake(sms_queue_lock, portMAX_DELAY);
  if(sms_queue.length()) sms_queue += (char)26;
  sms_queue += message;
  xSemaphoreGive(sms_queue_lock);
  xSemaphoreGive(sms_lock);
}

void resend_sms() {
  xSemaphoreTake(sms_queue_lock, portMAX_DELAY);
  if(sms_queue.length() and failed_sms_queue.length()) sms_queue += (char)26;
  sms_queue += failed_sms_queue;
  failed_sms_queue = "";
  xSemaphoreGive(sms_queue_lock);
  xSemaphoreGive(sms_lock);
}

void init_gsm() {
  sms_queue_lock = xSemaphoreCreateMutex();
  sms_lock = xSemaphoreCreateBinary();
  gsm_listener_lock = xSemaphoreCreateMutex();
  
  gsm_uart.begin(115200, SERIAL_8N1, GSM_TX, GSM_RX);
  delay(1500);
  //gsm_uart.write("AT&F;E0;V0;+CMEE=1;+CMGF=1;+IPR=115200;&W\n");
  gsm_uart.write("ATE0;V0;+CMEE=1;+CMGF=1;+IPR=115200;&W\n");
  delay(100);
  while(gsm_uart.available()) gsm_uart.read();
  gsm_uart.write("AT\n");
  long t = millis();
  while(!gsm_uart.available() and millis() - t < 400);
  String ok = "";
  while(gsm_uart.available()) {
    char c = gsm_uart.read();
    if(c != '\n' and c != '\r') ok += c;
    if(!gsm_uart.available()) delay(GSM_READ_TIMEOUT);
  }
  gsm_status = ok == "0";
  if(gsm_status) log("SIM800L connected");
  else log("SIM800L not connected: \"" + ok + "\"");

  xTaskCreatePinnedToCore(sms_sender, "sms", DEFALT_STACK, NULL, 0, NULL, 1);
  xTaskCreatePinnedToCore(SIM800L_listener, "gsm_listener", DEFALT_STACK, NULL, 0, NULL, 1);
}
