#include <driver/i2s.h>
#include <LiquidCrystal.h>
#include <ESP32Time.h>

#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2

#define I2S_PORT I2S_NUM_0
#define bufferLen 64

#define LEDPIN_BLUE 4
#define PIEZO 27

#define FREQ_FS4 370
#define FREQ_A7  3520

#define BUTTON_YELLOW_START 14
#define BUTTON_GREEN_RESET 12

#define LCD_RS 16
#define LCD_EN 17
#define LCD_D4 5
#define LCD_D5 18
#define LCD_D6 19
#define LCD_D7 21

//ESP32Time real time clock;
ESP32Time rtc(0);  // offset unset

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

struct epochExhale {
  bool is_set = false;  // locks reading of all values
  bool exhale_start_lock = false;
  bool exhale_end_lock = false;
  int highest_read = 0;
  unsigned long epoch_begin_exhale = 0;
  unsigned long millis_begin_exhale = 0;
  unsigned long epoch_end_exhale = 0;
  unsigned long millis_end_exhale = 0;
};

// Controls the state of the whole program
enum controlState{
  IDLE, // Not polling
  CALIBRATING, // Waiting 40-50 seconds to elapse to fill high_sample_memory
  ACTIVE_READING, // Actively taking reads of breaths
  AWAIT_RESET // State that will force state back into IDLE
} control_state = IDLE;

// Controls the state of whether a breath is detected
enum breathState{
  BREATH_UNSET, // No state set
  BREATH_EXHALE, // Detected sound above threshold on mic
  BREATH_INHALE, // Sound level below threshold
} breath_state = BREATH_UNSET;

// Controls the states of the alarms
enum alarmState{
  ALARM_DISABLED, // Not alarming
  ALARM_NOT_TRIGGERED, // No alarms triggered
  ALARM_NO_BREATHING, // No breathing detected
  ALARM_SLOW_BREATHING, // Slow breathing detected
  ALARM_FAST_BREATHING // Fast breathing detected
} alarm_state = ALARM_DISABLED;

// For reading from the MEMs mic
int16_t sBuffer[bufferLen];
size_t bytesIn = 0;

// LENGTH_OSM and one_second_memory must be the same
// These are for calibration
size_t LENGTH_OSM = 25;
int one_second_memory[25];

size_t LENGTH_HPVDM = 5;
int hpvd_memory[5];

size_t LENGTH_HSM = 3000;
int high_sample_memory[3000];
bool config_dynamic_threshold = false; // set to True to dynamically change exhale detection threshold


// hsb_pointer and osb_pointer both always point to epochExhale that is to be written on.
size_t LENGTH_OSB = 5;
epochExhale one_second_breath[5];  // used to read the last 4 breaths (1 always open for writing)
size_t osb_pointer = 0;

size_t LENGTH_HSB = 1000;
epochExhale high_sample_breath[1000]; // used for reading all breath data.
size_t hsb_pointer = 0;


// For reading and keeping track of exhale peak to peak
size_t epoch = 0;
esp_err_t result; // Used as the main readout
int peak_to_peak_read = 0;
bool read_success = false;
int osm_sum = 0;
int hpvdm_sum = 0;
int hsm_sum = 0;

String top_line;
String bot_line;

bool write_lock = false;
unsigned long last_write_time = 0; // last breath written

unsigned long last_lcd_clear_update = 0;

void i2s_install(){
  const i2s_config_t i2s_config = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = bufferLen,
    .use_apll = false
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

void i2s_setpin(){
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

// Serial print out in order to input to Arduino plotter
void demo_plotter_print(){
    Serial.print("ZERO:");
    Serial.print(0);
    Serial.print(",");
    Serial.print("current_peak_to_peak:");
    Serial.print(peak_to_peak_read);
    Serial.print(",");
    Serial.print("hsm_sum_ptp:");
    Serial.print(hsm_sum);
    Serial.print(",");
    Serial.print("osm_sum_ptp:");
    Serial.print(osm_sum);
    Serial.print(",");
    Serial.print("hpvdm_sum_ptp:");
    Serial.print(hpvdm_sum);
    Serial.print(",");
    Serial.print("hsm_avg_ptp:");
    Serial.print(hsm_sum/LENGTH_HSM);
    Serial.print(",");
    Serial.print("osm_avg_ptp:");
    Serial.print(osm_sum/LENGTH_OSM);
    Serial.print(",");
    Serial.print("hpvdm_avg_ptp:");
    Serial.print(hpvdm_sum/LENGTH_HPVDM);
    Serial.print(",");
    Serial.print("Epoch:");
    Serial.print(epoch);
    Serial.println();
}

// Return true if sample read and peak to peak is updated.
bool read_and_update_mic_values(){
  int samples_read = bytesIn / 8;
  int min_read = 100000;
  int max_read = -100000;
  if (samples_read > 0) {
    float mean = 0;
    for (int i = 0; i < samples_read; ++i) {
      mean += (sBuffer[i]);
      if (sBuffer[i] < min_read)
      {
        min_read = sBuffer[i];
      }
      if (sBuffer[i] > max_read)
      {
        max_read = sBuffer[i];
      }
    }
    mean /= samples_read;
    peak_to_peak_read = max_read - min_read;
    return true;
  }
  return false;
}


void unlock_next_breath_slot(epochExhale all_breaths[], size_t current_pointer){
  all_breaths[current_pointer].is_set = false;
  all_breaths[current_pointer].exhale_start_lock = false;
  all_breaths[current_pointer].exhale_end_lock = false;
  all_breaths[current_pointer].highest_read = 0;
  all_breaths[current_pointer].epoch_begin_exhale = 0;
  all_breaths[current_pointer].epoch_end_exhale = 0;
  all_breaths[current_pointer].millis_begin_exhale = 0;
  all_breaths[current_pointer].millis_end_exhale = 0;
}

void update_highest_read(epochExhale all_breaths[], size_t current_pointer, int current_read){
  if (all_breaths[current_pointer].is_set == true){
    return; // this value is already set, ignore the update
  }
  int prev_read = all_breaths[current_pointer].highest_read;
  if (current_read > prev_read){  //update previous read with higher value
    all_breaths[current_pointer].highest_read = current_read;
  }
}

void update_exhale_begin(epochExhale all_breaths[], size_t current_pointer){
  if (all_breaths[current_pointer].exhale_start_lock == false){  // don't change the first time stamp
    all_breaths[current_pointer].epoch_begin_exhale = rtc.getEpoch();
    all_breaths[current_pointer].millis_begin_exhale = millis();
    all_breaths[current_pointer].exhale_start_lock = true;
  }
}

void update_exhale_end(epochExhale all_breaths[], size_t current_pointer){
  if (all_breaths[current_pointer].exhale_end_lock == false){ 
    all_breaths[current_pointer].epoch_end_exhale = rtc.getEpoch();
    all_breaths[current_pointer].millis_end_exhale = millis();
    all_breaths[current_pointer].exhale_end_lock = true;
    all_breaths[current_pointer].is_set = true;
  }
}

int read_exhale_to_exhale_breath_rate(const epochExhale all_breaths[], size_t& length_all_breaths, int oldest_pointer, int newest_pointer){
  // Respiratry rate is not ready to be read
  if (all_breaths[oldest_pointer].is_set == false){
    return -1;
  }

  unsigned long oldest_time = all_breaths[oldest_pointer].epoch_begin_exhale;
  unsigned long oldest_millis = all_breaths[oldest_pointer].millis_begin_exhale;

  unsigned long recent_time = all_breaths[newest_pointer].epoch_begin_exhale;
  unsigned long recent_millis = all_breaths[newest_pointer].millis_begin_exhale;
  long duration = recent_time - oldest_time;
  long duration_millis = (recent_millis - oldest_millis);

  if (duration == 0 ){
    return -1; // division by zero
  }

  float raw_rate = (((length_all_breaths - 1) * 60) * 1000) / duration_millis;  //breaths per minute
  return raw_rate;
}

int read_last_sixty_seconds_of_breaths(const epochExhale all_breaths[], size_t& length_all_breaths, int read_backwards_pointer){
  // read first slot, and see if we even have 60 seconds of data. If we do not return
  unsigned long now = millis();
  if ((is_exhale_within_last_sixty(all_breaths, 0, now) == true )){
    return -1;
  }

  int total_breaths_in_last_sixty_seconds = 0;
  while (is_exhale_within_last_sixty(all_breaths, read_backwards_pointer, now)){
    total_breaths_in_last_sixty_seconds += 1;
    read_backwards_pointer -= 1;
  }
  return total_breaths_in_last_sixty_seconds;
}

bool is_exhale_within_last_sixty(const epochExhale all_breaths[], int current_pointer, unsigned long now){
  if (current_pointer < 0){
    return false;
  }
  unsigned long breath_start_time = all_breaths[current_pointer].millis_begin_exhale;
  if ((breath_start_time + 60000) > now){
    return true;
  }
  return false;
}

void silence_detected_reset_one_second_memory(){
  for (size_t i = 0; i < LENGTH_OSM; ++i ){
    one_second_memory[i] = 0;
  }
  osm_sum = 0;
}

// Updates sum using passed in circular queue
void update_readings_over_time_sum(int& current_sum, int& current_value, size_t& current_epoch, int arr[], size_t& size){
  current_sum += current_value;
  size_t next_index = current_epoch % size;
  int last_out = arr[next_index];
  current_sum -= last_out;
  arr[next_index] = current_value;
}


void alarm_check_and_sound(){
  if ((alarm_state == ALARM_NO_BREATHING) || (alarm_state == ALARM_FAST_BREATHING)){
  tone(PIEZO, FREQ_A7, 10);
  }
}

void handle_mems_read(){
  // Read from MEMS mic
  result = i2s_read(I2S_PORT, &sBuffer, bufferLen, &bytesIn, portMAX_DELAY);
  read_success = read_and_update_mic_values();
  // Update the one second memory
  if (result == ESP_OK && read_and_update_mic_values())
  {
    // Short burst, one second memory for detecting exhale
    update_readings_over_time_sum(osm_sum, peak_to_peak_read, epoch, one_second_memory, LENGTH_OSM);
    // For hyperventilation detection
    update_readings_over_time_sum(hpvdm_sum, peak_to_peak_read, epoch, hpvd_memory, LENGTH_HPVDM);
  }
}

void tone_start_up(){
  tone(PIEZO, FREQ_A7, 100);
  tone(PIEZO, FREQ_A7 + 1000, 100);
  tone(PIEZO, FREQ_A7 + 2000, 100);
  tone(PIEZO, FREQ_A7 + 3000, 100);
}

void tone_reset(){
  tone(PIEZO, FREQ_A7 + 2000, 100);
  tone(PIEZO, FREQ_A7 + 1000, 100);
  tone(PIEZO, FREQ_A7, 100);
}

void tone_begin_monitoring(){
  tone(PIEZO, FREQ_A7 + 3000, 100);
  tone(PIEZO, FREQ_A7 + 2000, 100);
  tone(PIEZO, FREQ_A7 + 1000, 100);
  tone_start_up();
}

void tone_button_press(){
  tone(PIEZO, FREQ_FS4,250);
}

// Interrupt for resetting the device
void IRAM_ATTR alarm_isr(){
  if (control_state != AWAIT_RESET){
    tone_button_press();
    tone_reset();
    detachInterrupt(BUTTON_GREEN_RESET);
    control_state = AWAIT_RESET;
  }
}

// Interrupt for calibrating the device
void IRAM_ATTR calibrate_isr(){
  if (control_state == IDLE){
    tone_button_press();
    control_state = CALIBRATING;
    // Flush screen contents; reset epoch
    lcd.clear();
    // Must set epoch to zero to begin filling circular queues
    epoch = 0;
    detachInterrupt(BUTTON_YELLOW_START);
  }
}


// Resetting all variables and arrays to zero before start of any execution
void initialize_state(){
   epoch = 0;
   bytesIn = 0;
  // Initialize arrays
  for (size_t i = 0; i < LENGTH_OSM; ++i ){
    one_second_memory[i] = 0;
  }

  for (size_t i = 0; i < LENGTH_HPVDM; ++i ){
    hpvd_memory[i] = 0;
  }

  for (size_t i = 0; i < LENGTH_HSM; ++i ){
    high_sample_memory[i] = 0;
  }

  for (size_t i = 0; i < LENGTH_HSM; ++i ){
    high_sample_memory[i] = 0;
  }

  for (size_t i = 0; i < LENGTH_OSB; ++i ){
    unlock_next_breath_slot(one_second_breath, i);
  }

  for (size_t i = 0; i < LENGTH_HSB; ++i ){
    unlock_next_breath_slot(high_sample_breath, i);
  }

  hsb_pointer = 0;
  osb_pointer = 0;

  attachInterrupt(BUTTON_GREEN_RESET, alarm_isr, FALLING);
  delay(50);
  attachInterrupt(BUTTON_YELLOW_START, calibrate_isr, FALLING);
  alarm_state = ALARM_DISABLED;
  control_state = IDLE;
  alarm_state = ALARM_NOT_TRIGGERED;
  breath_state = BREATH_UNSET;
  last_write_time = 0;
  peak_to_peak_read = 0;
  read_success = false;
  osm_sum = 0;
  hpvdm_sum = 0;
  hsm_sum = 0;

  // Set up real time clock on ESP32
  rtc.setTime(0, 0, 0, 1, 12, 2023);  // 1st Dec 2023 00:00:00

  tone_start_up();
}

void setup() {
  Serial.begin(115200);
  Serial.println("Setup I2S ...");
  lcd.print("Setting up");
  delay(100);
  i2s_install();
  i2s_setpin();
  i2s_start(I2S_PORT);
  delay(500);

  lcd.begin(16, 2);
  pinMode(LEDPIN_BLUE, OUTPUT);
  pinMode(BUTTON_YELLOW_START, INPUT_PULLUP);
  pinMode(BUTTON_GREEN_RESET, INPUT_PULLUP);
  delay(500);

  initialize_state(); // Zero out all variables
}

void loop() {
  handle_mems_read(); // Read from microphone

  if (control_state == AWAIT_RESET)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Resetting device");
    delay(1000);
    initialize_state();  // Zero out all variables
  }
  else if (control_state == IDLE)
  {
    lcd.clear();
    lcd.print("Press button to");
    lcd.setCursor(0 , 1);
    lcd.print("start");
    delay(1000);
    return;
  }
  else if (control_state == CALIBRATING )
  {
    lcd.setCursor(0, 0);
    lcd.print("Calibrating");
    lcd.setCursor(0, 1);
    lcd.print(rtc.getTime("%T"));
    if (result == ESP_OK && read_success)
    {
      // Update the high sample memory
      update_readings_over_time_sum(hsm_sum, peak_to_peak_read, epoch, high_sample_memory, LENGTH_HSM);
    }
    // Once we have filled the high sample memory with calibration values
    if (epoch > LENGTH_HSM){
      tone_begin_monitoring();
      control_state = ACTIVE_READING;
    }
  }
  else if (write_lock == true){  //refuse to write after first exhale detected to prevent bouncing
    unsigned long now = millis();
    if (now > (last_write_time + 1))
    {
      write_lock = false;
      }
    demo_plotter_print(); 
    return;
  }
  else if ( (control_state == ACTIVE_READING) && (write_lock == false) ){
    // If dynamic thresholding is true, update the high sample memory over time
    if (config_dynamic_threshold){
      update_readings_over_time_sum(hsm_sum, peak_to_peak_read, epoch, high_sample_memory, LENGTH_HSM);
    }
    unsigned long current_epoch = rtc.getEpoch();
    if (current_epoch != last_lcd_clear_update){
      lcd.clear();
      last_lcd_clear_update = current_epoch;
    }
    lcd.setCursor(0, 0);
    lcd.print("RR: ");
    breathState  prev_breath_state = breath_state;
    update_highest_read(high_sample_breath, hsb_pointer, peak_to_peak_read);
    update_highest_read(one_second_breath, osb_pointer, peak_to_peak_read);


    if ((hsm_sum/LENGTH_HSM) < (osm_sum/LENGTH_OSM) ){
      if (alarm_state == ALARM_NOT_TRIGGERED){
        lcd.print("EXHALING        ");
      }
      digitalWrite(LEDPIN_BLUE, HIGH);
      breath_state = BREATH_EXHALE;
      update_exhale_begin(high_sample_breath, hsb_pointer);
      update_exhale_begin(one_second_breath, osb_pointer);
      // write_lock = true;  //disabled due to reducing performance of reads
      last_write_time = millis();
    }
    else{
      if (alarm_state == ALARM_NOT_TRIGGERED){
        lcd.print("--        ");
      }
      digitalWrite(LEDPIN_BLUE, LOW);
      breath_state = BREATH_INHALE;
      if (prev_breath_state == BREATH_EXHALE){
        update_exhale_end(high_sample_breath, hsb_pointer);
        update_exhale_end(one_second_breath, osb_pointer);
        hsb_pointer += 1;
        hsb_pointer = hsb_pointer % LENGTH_HSB;
        osb_pointer += 1;
        osb_pointer = osb_pointer % LENGTH_OSB;
        unlock_next_breath_slot(high_sample_breath, hsb_pointer);
        unlock_next_breath_slot(one_second_breath, osb_pointer);
      }
    }
    if( ( (hpvdm_sum * 2/LENGTH_HPVDM) < (hsm_sum/LENGTH_HSM)) && breath_state == BREATH_EXHALE)  // breath is at exhale, yet hypvdm has detected drop 1/2 below set threshold.
    {
      silence_detected_reset_one_second_memory();
    }
    size_t recent_pointer = 0;
    if (osb_pointer == 0)  //edge case of rolling back index from 0 on a size_t
    {
      recent_pointer = LENGTH_OSB - 1;
    }
    else{
      recent_pointer = (osb_pointer - 1) % LENGTH_OSB;
    }
    size_t oldest_pointer = (osb_pointer + 1) % LENGTH_OSB;
    int rate = read_exhale_to_exhale_breath_rate(one_second_breath, LENGTH_OSB, oldest_pointer, recent_pointer);

    int last_sixty = read_last_sixty_seconds_of_breaths(high_sample_breath, LENGTH_HSB, hsb_pointer - 1);
    lcd.setCursor(0, 1);
    lcd.print("Bpm:");
    lcd.setCursor(8, 1);
    lcd.print("Act:");
    lcd.setCursor(13,1);
    if (last_sixty == -1){
      lcd.print("--");
    }
    else {
      lcd.print(last_sixty);
    }
    unsigned long now = millis();
    bool no_breath_detected_last_ten = (last_write_time + 10000) < now; // ten second have passed without writing a new exhale

    // TODO Refactor this into separate check; when this logic is true, we want to skip the
    // rest of the block, which is why we return. This forces a call to increment epoch, check for alarm, and demo etc.
    if ((last_write_time > 0) && (no_breath_detected_last_ten == true)){
      alarm_state = ALARM_NO_BREATHING;
      lcd.setCursor(4,0);
      lcd.print("NO BREATHING");
      lcd.setCursor(4,1);
      lcd.print("--");
      epoch +=1;
      alarm_check_and_sound();
      demo_plotter_print();  // Used for Arduino plotter demo purposes
      return;
    }

    if (rate == -1){
      alarm_state = ALARM_NOT_TRIGGERED;
      lcd.print("--");  // display "--" if invalid results are returned
    }
    else if (rate > 60){
      alarm_state = ALARM_FAST_BREATHING;
      lcd.setCursor(4,0);
      lcd.print("FAST BREATHING");
      lcd.setCursor(4,1);
      lcd.print(rate);
    }
    else if (rate < 5) {
      alarm_state = ALARM_SLOW_BREATHING;
      lcd.setCursor(4,0);
      lcd.print("SLOW BREATHING");
      lcd.setCursor(4,1);
      lcd.print(rate);
    }
    else{
      alarm_state = ALARM_NOT_TRIGGERED;
      lcd.setCursor(4,1);
      lcd.print(rate);
    }
    alarm_check_and_sound();
    }

  demo_plotter_print();  // Used for Arduino plotter demo purposes
  epoch+=1;
}
