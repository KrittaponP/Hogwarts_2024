#include <CanCommunication.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 20 chars and 4 line display

#define encoder1 GPIO_NUM_39
#define encoder2 GPIO_NUM_36

#define drive_selecter_f 25
#define drive_selecter_r 34

#define enable_pin GPIO_NUM_33
#define auto_stop_switch GPIO_NUM_35
#define trigger_pin GPIO_NUM_13
#define horn_pin GPIO_NUM_14

#define slide_bar GPIO_NUM_15

#define emergency_switch GPIO_NUM_32
#define safty_pin GPIO_NUM_27

#define short_motor_pin GPIO_NUM_12
#define fault_led GPIO_NUM_2

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

uint16_t last_pulse_d = 0;
uint16_t last_pulse_f = 0;
uint16_t last_pulse_f_cal = 0;


bool driveMode;
bool slipMode = 0;
bool auto_stop = 0;
TaskHandle_t Communication_task;
TaskHandle_t Driving_task;

uint16_t pulse_d = 0;
uint16_t pulse_f = 0;

float train_speed;
void onTimer_cal_speed() {
  train_speed = (pulse_f - last_pulse_f_cal) * 1.4137 / 0.01;
  last_pulse_f_cal = pulse_f;
}

void read_driveMode() {
  if (digitalRead(drive_selecter_f) == 1 && digitalRead(drive_selecter_r) == 1) {
    driveMode = 0;  // parking mode
  } else {
    driveMode = 1;  // driving mode
  }
}

void pulse_d_counter() {
  pulse_d++;
}

void pulse_f_counter() {
  pulse_f++;
}

void read_auto_stop() {  // read auto stop emergency button
  if (digitalRead(auto_stop_switch) == 0) {
    auto_stop = 1;  // driving mode
  } else {
    auto_stop = 0;  // parking mode
  }
}

void short_motor() {
  // digitalWrite(enable_pin, LOW);
  // send can request to read voltage only once
  // receive response and save to variable
  // while(voltage>60){
  //  send can request to read voltage only once
  // receive response and update to variable
  // }
  // digital write pin to short motor
}

bool trig = 0;
bool brake = 0;
void auto_stop_trig() {  // trig signal
  if (trig == 0) {       //position B
    trig = 1;
  } else {  //position C
    brake = 1;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(encoder1, INPUT);
  pinMode(encoder2, INPUT);
  pinMode(auto_stop_switch, INPUT_PULLUP);
  pinMode(trigger_pin, INPUT_PULLUP);
  pinMode(drive_selecter_f, INPUT);
  pinMode(drive_selecter_r, INPUT);

  pinMode(enable_pin, OUTPUT);
  pinMode(fault_led, OUTPUT);
  pinMode(horn_pin, OUTPUT);

  attachInterrupt(encoder1, pulse_d_counter, FALLING);
  attachInterrupt(encoder2, pulse_f_counter, FALLING);

  attachInterrupt(auto_stop_switch, read_auto_stop, RISING);
  attachInterrupt(trigger_pin, auto_stop_trig, FALLING);

  attachInterrupt(emergency_switch, short_motor, RISING);

  attachInterrupt(drive_selecter_f, read_driveMode, CHANGE);
  attachInterrupt(drive_selecter_r, read_driveMode, CHANGE);

  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer_cal_speed, true);
  timerAlarmWrite(timer, 10000, true);  // 10ms to generate event
  timerAlarmEnable(timer);

  while (digitalRead(safty_pin) != 0) {  // wait safty ready signal
    vTaskDelay(50);
  }

  while (digitalRead(drive_selecter_f) != 1 || digitalRead(drive_selecter_r) != 1) {  // wait until park mode before enable
    Serial.println("wait");
    vTaskDelay(50);
  }

  if (digitalRead(drive_selecter_f) == 1 && digitalRead(drive_selecter_r) == 1) {  // trig enable and horn 3 times
    driveMode = 1;                                                                 // driving mode
    digitalWrite(enable_pin, HIGH);
  }

  xTaskCreatePinnedToCore(
    Communication,       /* Task function. */
    "Task1",             /* name of task. */
    10000,               /* Stack size of task */
    NULL,                /* parameter of the task */
    1,                   /* priority of the task */
    &Communication_task, /* Task handle to keep track of created task */
    1);                  /* pin task to core 1 */
  vTaskDelay(500);

  xTaskCreatePinnedToCore(
    Driving,       /* Task function. */
    "Task2",       /* name of task. */
    10000,         /* Stack size of task */
    NULL,          /* parameter of the task */
    1,             /* priority of the task */
    &Driving_task, /* Task handle to keep track of created task */
    0);            /* pin task to core 0 */
  vTaskDelay(500);

  lcd.init();  // initialize the lcd
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Speed :");
  lcd.setCursor(0, 1);
  lcd.print("N :");
  lcd.setCursor(0, 2);
  lcd.print("Vdc :");
  lcd.setCursor(0, 3);
  lcd.print("Temp  :");
}

int16_t N_actual;
int16_t N_cmd;
float Iq;
float I_cmd;
float temperature;
float Vdc;

long actual_position;
unsigned long last_send_speed = 0;
CanCommunication can(GPIO_NUM_4, GPIO_NUM_5, 500000);

void Communication(void *pvParameters) {  // to do: change print debugger to fault lamp
  if (!can.begin()) {
    Serial.println("Failed to initialize CAN communication");
    return;
  }

  if (!can.start()) {
    Serial.println("Failed to start CAN communication");
    return;
  }

  uint8_t mode_data[3] = { 0x36, 0x00, 0x00 };
  if (can.transmit(0x201, mode_data, 3)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }

  uint8_t actual_speed_data[3] = { 0x3D, 0x30, 0x0A };
  if (can.transmit(0x201, actual_speed_data, 3)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }

  uint8_t actual_Iq_data[3] = { 0x3D, 0x27, 0xFF };
  if (can.transmit(0x201, actual_Iq_data, 3)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }

  uint8_t actual_I_cmd_data[3] = { 0x3D, 0x26, 0xFF };
  if (can.transmit(0x201, actual_I_cmd_data, 3)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }

  uint8_t temperature_data[3] = { 0x3D, 0x49, 0x0A };
  if (can.transmit(0x201, temperature_data, 3)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }

  uint8_t Vdc_data[3] = { 0x3D, 0xEB, 0x0A };
  if (can.transmit(0x201, Vdc_data, 3)) {
    Serial.println("Message queued for transmission");
  } else {
    Serial.println("Failed to queue message for transmission");
  }
  unsigned long last_print = 0;

  while (true) {
    can_message_t message;
    if (can.receive(&message)) {
      int value;
      Serial.printf("ID 0x%02x: ", message.identifier);
      for (int i = message.data_length_code - 1; i >= 0; i--) {

        Serial.print(message.data[i], HEX);
        Serial.print(" ");
        value = value << 8 | message.data[i];
      }
      int reg = value & 0xFF;
      switch (reg) {
        case 0x30:  // N_actual
          N_actual = (value & 0xFFFF00) >> 8;
          break;
        case 0x27:  // Iq
          Iq = (value & 0xFFFF00) >> 8;
          break;
        case 0x26:  // I command
          I_cmd = (value & 0xFFFF00) >> 8;
          break;
        case 0x49:  // temperature
          temperature = (value & 0xFFFF00) >> 8;
          break;
        case 0xEB:  // Vdc
          Vdc = (value & 0xFFFF00) >> 8;
          break;
      }
      //Send speed value
      if (millis() - last_send_speed > 20) {
        N_d = (N_actual / 32767) * 1300 * (16 / 39);
        N_f = (((pulse_f - last_pulse_f) / 40) / 0.02) * 60;
        last_pulse_f = pulse_f;
        float pulse_ratio = N_d / (N_d + N_f);
        Serial.println(pulse_ratio);
        byte speed0to7 = train_speed & 0xFF;
        byte speed8to15 = (train_speed >> 8) & 0xFF;
        byte speed16to23 = (train_speed >> 16) & 0xFF;
        byte speed24to31 = (train_speed >> 24) & 0xFF;
        byte train_speed_data[4] = { speed0to7, speed8to15, speed16to23, speed24to31 };
        can.transmit(0x10, train_speed_data, 4);
        last_send_speed = millis();
      }
      //LCD
      if (millis() - last_print > 1500) {
        // Display random values on LCD
        lcd.setCursor(8, 0);
        lcd.print(train_speed);
        lcd.setCursor(16, 0);
        lcd.print("km/h");

        lcd.setCursor(8, 1);
        lcd.print(N_actual * 1300 / 32767, 2);
        lcd.setCursor(16, 1);
        lcd.print("RPM");

        lcd.setCursor(8, 2);
        lcd.print(Vdc);
        lcd.setCursor(16, 2);
        lcd.print("Volt");

        lcd.setCursor(8, 3);
        lcd.print(temperature);
        lcd.setCursor(16, 3);
        lcd.print("C");
        last_print = millis();
      }
      //
    }
  }
}

bool state = 1;  // 1 represent a+ and 0 represent a-
int K_pos = 5;
int K_neg = 12;
bool a;

int Tref, Tm, Tmem;
int a_com = 0;

int last_command_speed = 0;
int speed_command = 0;

uint64_t t_minus1 = 0;
unsigned long prev_time = 0;

double error = 0;
long pulse_dif;
bool isSendDistance = false;

bool finished = 0;
void Driving(void *pvParameters) {
  int delay_time = 50;
  while (true) {
    if (driveMode == 1) {  // Forward to ฺBackward
      float N_d;
      float N_f;
      if (auto_stop && trig) {
        if (finished == 1) {
          set_speed(0);
          return;
        }
        if (!isSendDistance) {
          uint8_t actual_position_data[3] = { 0x3D, 0x6D, 0x00 };
          can.transmit(0x201, actual_position_data, 3);
          isSendDistance = true;
        }
        float target_distance = 200;  // meter unit

        set_position(actual_position + ((target_distance * 100) / 23.19945) * 65536 + (error / 23.19945) * 65536);

        if (millis() - prev_time > 100) {  //error correction
          pulse_dif = (pulse_d - last_pulse_d) - (pulse_f - last_pulse_f);
          if (pulse_dif > 5) {
            error += pulse_dif * 1.413716694;
          }
          last_pulse_d = pulse_d;
          last_pulse_f = pulse_f;
          prev_time = millis();
        }

        if (brake == 1) {
          set_speed(0);
          finished = 1;
          return;
        }

        // float distance = (pulse_f - last_pulse_f) * 1.413716694 * 0.01;
        // last_pulse_d = pulse_d;
        // last_pulse_f = pulse_f;
        // float target_distance = 200; // adjust
        // while (distance < target_distance)
        // {
        //   if (speed_command > 7)
        //   {
        //     set_speed(speed_command--);
        //   }
        //   distance = pulse_f * 1.413716694 * 0.01;
        //   vTaskDelay(100); // adjust
        // }
        // while (speed_command > 0)
        // {
        //   set_speed(speed_command--);
        //   vTaskDelay(20); // adjust
        // }
      } else {
        if ((N_actual > 0 || pulse_f - last_pulse_f > 0) && (millis() - prev_time > 20)) {  // check slip
                                                                                            // if ((pulse_f-last_pulse_f > 0 || pulse_d - last_pulse_d > 0) && (millis() - prev_time > 5)){
          // float pulse_ratio = (pulse_d - last_pulse_d)/((pulse_d-last_pulse_d)+(pulse_f-last_pulse_f));
          N_d = (N_actual / 32767) * 1300 * (16 / 39);
          N_f = (((pulse_f - last_pulse_f) / 40) / 0.02) * 60;
          last_pulse_f = pulse_f;
          // last_pulse_d = pulse_d;
          float pulse_ratio = N_d / (N_d + N_f);
          Serial.println(pulse_ratio);
          if (pulse_ratio >= 0.6) {  // change the slip ratio threshold
            slipMode = 1;
          } else {
            slipMode = 0;
          }
        }
        if (slipMode == 1) {
          set_speed(last_command_speed++);
          if () {
            return;
          }
          vTaskDelay(delay_time);
          delay_time += 10;

          // vTaskDelay(10);
        } else {  // no anti slip
          delay_time = 50;
          int raw_value = analogRead(slide_bar);
          speed_command = map(raw_value, 0, 4095, 0, 100);
          while (last_command_speed > speed_command) {  //slow decrease speed
            set_speed(last_command_speed--);
            vTaskDelay(80);
          }
          while (last_command_speed < speed_command) {  //slow increase speed
            set_speed(last_command_speed++);
            if () {
              return;
            }
            vTaskDelay(100);
          }
          set_speed(speed_command);
          last_command_speed = speed_command;
          vTaskDelay(5);
        }
      }
    } else {  // Parking mode, decrease speed to zero
      while (speed_command > 0) {
        set_speed(speed_command--);
        vTaskDelay(40);  // adjust delay after
      }
      set_speed(0);
    }
  }
}

void set_speed(int cmd) {
  cmd = map(cmd, 0, 100, 0, 32767);
  uint8_t speed_lsb = cmd & 0xFF;
  uint8_t speed_msb = (cmd >> 8) & 0xFF;
  uint8_t actual_speed_data[3] = { 0x31, speed_lsb, speed_msb };
  can.transmit(0x201, actual_speed_data, 3);
}

void set_position(long cmd) {
  uint8_t data7to0 = cmd & 0xFF;
  uint8_t data15to8 = (cmd >> 8) & 0xFF;
  uint8_t data23to16 = (cmd >> 16) & 0xFF;
  uint8_t data31to24 = (cmd >> 24) & 0xFF;

  uint8_t position_data[5] = { 0x6E, data7to0, data15to8, data23to16, data31to24 };
  can.transmit(0x201, position_data, 5);
}

void loop() {
}