#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define ONE_WIRE_BUS 4
#define COOL_MODE_PIN 5
#define HEAT_MODE_PIN 6
#define OVERRIDE_BUTTON_PIN 7
#define FAN_PIN 10
// How many degrees should the temperature have to rise/fall
// before we turn the fan off?
#define FAN_OFF_DELTA 5.0

// TODO: Store floats in memory as uint16_t, and only convert them to floats when we want to display them?

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

enum SYSTEM_MODES: byte {
    OFF,
    COOL,
    HEAT,
    ERR_DUAL
};

static const char * SYSTEM_Mode_Strings[] = {
    "Off    ",
    "Cooling",
    "Heating",
    "**ERR**" };

SYSTEM_MODES current_system_mode;
SYSTEM_MODES prev_system_mode;

float sys_off_temperature = 0.0;
float fan_off_temperature = 0.0;
volatile boolean override_fan = false;
void setup_temp_probe() {
    sensors.begin();
    sensors.setResolution(10);
}

void print_lcd_labels() {
    lcd.setCursor(0,0);
    lcd.print("Ambient:");
    lcd.setCursor(0,1);
    lcd.print("Target:");
    lcd.setCursor(0,2);
    lcd.print("System:");
    lcd.setCursor(0,3);
    lcd.print("Fan:");
}

void setup_lcd() {
    lcd.begin(20,4);
    lcd.backlight();
    lcd.clear();
    print_lcd_labels();
}

const char* getSystemStateTextForEnum(int enumVal)
{
    return SYSTEM_Mode_Strings[enumVal];
}

SYSTEM_MODES get_system_mode() {
    if (digitalRead(COOL_MODE_PIN) == LOW && digitalRead(HEAT_MODE_PIN) == LOW) {
        return ERR_DUAL;
    } else if (digitalRead(COOL_MODE_PIN) == LOW) {
        return COOL;
    } else if (digitalRead(HEAT_MODE_PIN) == LOW) {
        return HEAT;
    } else {
        return OFF;
    }
}

void clear_target_temperature() {
    fan_off_temperature = 0.0;
}

void set_fan_state(bool on_off) {
    if (on_off) {
        digitalWrite(FAN_PIN, HIGH);
    } else {
        digitalWrite(FAN_PIN, LOW);
    }
}

void system_mode_changed(SYSTEM_MODES prev_system_mode, float temperature) {

    if ((current_system_mode == COOL) ^ (current_system_mode == HEAT)) {
        set_fan_state(true);
        clear_target_temperature();
    } else if (digitalRead(FAN_PIN) == HIGH){
        // Record the temperature when the system turned off
        sys_off_temperature = temperature;

        // Determine a fan off temperature threshold
        if (prev_system_mode == COOL) {
            // Expect the temperature to RISE because we've stopped cooling
            fan_off_temperature = sys_off_temperature + FAN_OFF_DELTA;
        } else if ( prev_system_mode == HEAT ) {
            fan_off_temperature = sys_off_temperature - FAN_OFF_DELTA;
        }
    }
}

float get_temperature() {
    float sensor_value;
    digitalWrite(LED_BUILTIN, HIGH);
    sensors.requestTemperatures(); // Send the command to get temperature readings
    sensor_value = sensors.getTempFByIndex(0);
    digitalWrite(LED_BUILTIN, LOW);
    return sensor_value;
}

void update_display(float temperature, bool sys_change) {
    lcd.setCursor(9,0);
    lcd.print(temperature);
    if ( sys_change ) {
        lcd.setCursor(9,1);
        if ( fan_off_temperature > 0.0 ) {
            lcd.print(fan_off_temperature);
        } else {
            lcd.print("-    ");
        }
        lcd.setCursor(8,2);
        lcd.print(getSystemStateTextForEnum(current_system_mode));
        lcd.setCursor(8, 3);
        lcd.print(digitalRead(FAN_PIN) == HIGH ? "On " : "Off");
    }
}

void serial_write(float current_temperature) {
    /*
     * First four bits: header
     * 0x00 temperature
     * 0x01 system state
     * Second four bits: unused
     * Temperature
     * next 12 bits: current temperature
     * next 12 bits: target temperature
     * 2 bits: system state
     * 1 bit: fan state
     */
    uint32_t tp = 0;
    uint32_t sp = 0;
    tp |= (uint16_t)(current_temperature * 100);
    tp<<=12;
    tp |= (uint16_t)(fan_off_temperature * 100);

    sp |= 1; // Header
    sp<<=27;
    sp |= current_system_mode;
    sp<<=1;
    sp |= digitalRead(FAN_PIN) == HIGH;
    Serial.println(tp);
    Serial.println(sp);

}

void override_button() {
    set_fan_state(false); // turn the fan off immediately
    override_fan = true; // set a notification var for loop()
}

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(COOL_MODE_PIN, INPUT_PULLUP);
    pinMode(HEAT_MODE_PIN, INPUT_PULLUP);
    pinMode(OVERRIDE_BUTTON_PIN, INPUT_PULLUP);
    pinMode(FAN_PIN, OUTPUT);
    digitalWrite(FAN_PIN, LOW);
    setup_temp_probe();
    setup_lcd();
    current_system_mode = get_system_mode(); // Get the current mode as we start up
    update_display(get_temperature(), true); //
    attachInterrupt(digitalPinToInterrupt(OVERRIDE_BUTTON_PIN), override_button, FALLING);
}

void loop() {
    boolean sys_change = false;
    float current_temperature = get_temperature();
    current_system_mode = get_system_mode();
    if (prev_system_mode != current_system_mode) {
        system_mode_changed(prev_system_mode, current_temperature);
        prev_system_mode = current_system_mode;
        sys_change = true;
    }

    if (override_fan) {
        override_fan = false;
        sys_change = true;
        clear_target_temperature();
    }
    else if ( fan_off_temperature > sys_off_temperature && current_temperature >= fan_off_temperature ) { // Cooling
        set_fan_state(false);
        sys_change = true;
    } else if ( fan_off_temperature < sys_off_temperature && current_temperature <= fan_off_temperature ) { // Heat
        set_fan_state(false);
        sys_change = true;
    }

    update_display(current_temperature, sys_change);

    delay(2000);
}
