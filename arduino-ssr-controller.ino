// Controller for an SSR with a potentiometer controlling how much power is sent through
// An attached 16x2 LCD shows the current setting

// The SSR can either be "chunked" meaning that the time it is ON is a continous amount of time.
// If the SSR is "dispersed" the time-slots it is on are spread out.
// The chunked mode is good for something like a heater element, 
//  where the dispersed mode is good for something like a pump that needs to run continously
//  and not in a stop-go manner

// The only dependency is the LiquidCrystal library which is included with the Arduino IDE

#include <LiquidCrystal.h>

// Pin setup, change to match your board wiring
const uint16_t sensorPin = A0;   // select the input pin for the potentiometer
const uint16_t ssrPin = 13;      // select the pin for toggling the SSR
const uint16_t togglePin = 9;    // select the pin for toggling chunked/dispersed mode

// LCD pin mapping, change to match your board wiring
LiquidCrystal lcd(2, 3, 4, 5, 6, 7);

// LCD layout (16x2)
const uint16_t LCD_COLUMNS = 16;
const uint16_t LCD_ROWS = 2;

// Handy constants
#define MAX_UINT16 (0xFFFFF)
#define MAX_UINT32 (0xFFFFFFFF)

// Debug output, uncomment for serial debug output
//#define SERIAL_OUT

// Helper method for computing the number of milliseconds elapsed,
// with handling of wrap-over after ~50 days
uint32_t compute_elapsed(uint32_t current, uint32_t last) {
  return last > current
    ? (MAX_UINT32 - last) + current
    : (current - last);
}

// Abstract base class for a process, i.e. "something that runs"
// All instantiated sub-classes are automatically registered
// in a linked-list
class RunProcess {
  public:
    RunProcess() 
      : next(NULL)
    {
      if (head == NULL) {
        head = this;
      } else {
        RunProcess* n = head;
        while(n->next != NULL)
          n = n->next;

        n->next = this;
      }      
    }

    // The root entry in the linked list
    static RunProcess* head;
    // The next element
    RunProcess* next;

    // These virtual functions must be implemented by all processes
    virtual void setup() = 0;
    virtual void run() = 0;

    // Helper method to iterate the list and invoke `setup()` on all processes
    static void AllSetup() {
      RunProcess* cur = head;
      while(cur != NULL) {
        cur->setup();
        cur = cur->next;
      }      
    }

    // Helper method to iterate the list and invoke `run()` on all processes
    static void AllRun() {
      RunProcess* cur = head;
      while(cur != NULL) {
        cur->run();
        cur = cur->next;
      }    
    }
};

// Allocate the root entry
RunProcess* RunProcess::head;

// Helper for reading analog input
class AnalogReader : public RunProcess {
  public:

  uint16_t sensor; // The adjusted sensor value
  uint16_t sensor_raw; // The unadjusted sensor value
  uint32_t last_update; // The time the value was last read

  const uint16_t analog_pin; // Pin reading from
  const uint16_t low_adc_threshold; // Low ADC value (ignores lower)
  const uint16_t high_adc_threshold; // High ADC value (ignores higher)
  const uint16_t max_adc; // The maximum raw ADC input
  const uint16_t adc_range; // The computed number of ADC values possible
  
  AnalogReader(uint16_t pin, uint16_t low_threshold = 20, uint16_t high_threshold = 1003, uint16_t maxadc = 1023) 
    : analog_pin(pin), low_adc_threshold(low_threshold), 
      high_adc_threshold(high_threshold), 
      max_adc(maxadc), 
      adc_range(maxadc - low_threshold - (maxadc - high_threshold)),
      sensor(0),
      sensor_raw(0),
      last_update(0)
  {
  }

  // Compute the input value as an integer percent, in range [0-100]
  uint16_t get_percent() const {
    return (uint16_t)((sensor * ((uint32_t)100)) / (uint32_t)adc_range);    
  }

  void setup() { }

  void run() {
    // Reading is fast, so we read every time
    last_update = millis();
    sensor_raw = analogRead(analog_pin);

    // Clamp values in the desired range
    if (sensor_raw < low_adc_threshold) {
      sensor = 0;
    } else if (sensor_raw > high_adc_threshold) {
      sensor = adc_range;
    } else {
      sensor = sensor_raw - low_adc_threshold;
    }    
  }
};

// Helper for reading a pin, to avoid calling `digitalRead()`
//  in multiple places, which could lead to race conditions
class PinReader : public RunProcess {
  public:

  const uint16_t pin; // The digital pin to read from
  uint32_t last_update; // The time the value was last read
  bool state; // The pin state (true means HIGH)
  
  PinReader(uint16_t pin_) 
    : pin(pin_), last_update(0), state(false)
  {
  }

  void setup() {
    pinMode(pin, INPUT);
  }
  
  void run() {
    last_update = millis();
    state = digitalRead(pin) != LOW;    
  }
};

// The supported output modes for the SSR
enum SSROutputMode { Chunked, Dispersed };

// SSR controller support
class SSRController : public RunProcess {
  public:

  const uint16_t pin; // The digital pin to write to

  SSROutputMode mode; // The output mode 
  
  uint16_t window_size; // The number of ticks to handle
  uint32_t tickDuration; // The number of milliseconds pr. tick  

  uint16_t value; // The current value (the amount of cycles to keep high)

  uint16_t window_index; // The current window index
  uint32_t last_update; // The time the SSR was last toggled
  bool last_state; // The state of the output during the last update
  bool updated; // Flag indicating if the output was updated in this cycle

  // Set up the SSR controller, each "round" is (win_size * duration) milliseconds
  SSRController(uint16_t outpin, uint16_t win_size = 100, uint32_t duration = 1000, SSROutputMode ssrmode = Chunked)
    : pin(outpin),
      window_index(0), 
      window_size(win_size), 
      tickDuration(duration), 
      mode(ssrmode),
      value(0),
      last_update(0),
      last_state(false),
      updated(false)
  { }

  void setup() { 
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  void run() {
    uint32_t current = millis();
    uint32_t elapsed = compute_elapsed(current, last_update);

    // Figure out if we should update the output
    if (elapsed >= tickDuration) {
      updated = true;
      last_update = current;

      if (mode == Chunked) {
        last_state = chunkedRating(value, window_size, window_index, window_size);
      } else {
        last_state = dispersedRating(value, window_size, window_index, window_size);
      }

      // Move the index forward, and wrap around
      window_index = (window_index + 1) % window_size;  

      // Drive the output pin
      digitalWrite(pin, last_state ? HIGH : LOW);      
    } else {
      updated = false;
    }
  }

  // Sets the chunked output mode,
  //  with a long pulse, and fewest pulse changes
  void setChunked() {
    window_size = 100;
    tickDuration = 1000;
    mode = Chunked;
  }

  // Sets the dispersed output mode,
  //  with a short pulse, and most pulse changes
  void setDispersed() {
    window_size = 100;
    tickDuration = 10;
    mode = Dispersed;
  }

  private:
  /*
   value is the measured value
   max_value is the maximum value
   index is the timestep
   window_length is the maximum index to accept
  
   This method returns a boolean that indicates if a control signal should be set.
   The `index` value should be a timestep, wrapping at `window_length`.
   The `value` is used to control how many timesteps the output is `true`.
   This method groups the `true` and `false` periods, such that there is only.
   a single change within each `window_length` period.
  */
  bool chunkedRating(uint16_t value, uint16_t max_value, uint16_t index, uint16_t window_length)
  {
      uint32_t const MULTIPLIER = 10000;
  
      uint16_t cap = (uint16_t)((((window_length * MULTIPLIER) / max_value) * value) / MULTIPLIER);
      return index < cap;
  }
  
  /*
   value is the measured value
   max_value is the maximum value
   index is the timestep
   window_length is the maximum index to accept
  
   This method returns a boolean that indicates if a control signal should be set.
   The `index` value should be a timestep, wrapping at `window_length`.
   The `value` is used to control how many timesteps the output is `true`.
   This method spreads the `true`/`false` changes to have as many switches
   as possible within each `window_length`.
  */
  bool dispersedRating(uint16_t value, uint16_t max_value, uint16_t index, uint16_t window_length)
  {
      if (value >= max_value)
          return true;
  
      // To avoid using floats, we use a kind of fixed-precision numbers
      uint32_t const MULTIPLIER = 10000;
  
      // Scale output variations to fit the window length
      uint16_t deltax = window_length;
      uint16_t deltay = (uint16_t)((((window_length * MULTIPLIER) / max_value) * value) / MULTIPLIER);
      if (deltay == 0)
          return false;
  
      // First half is counting `true` values,
      // second half is counting `false` values
      bool counting_down = deltay < window_length / 2;
      if (!counting_down)
          deltay = window_length - deltay;
  
      uint32_t frac = (deltay * MULTIPLIER) / deltax;
      uint32_t pos_prev = ((index - 1) * frac) % MULTIPLIER;
      uint32_t pos = pos_prev + frac;
  
      if(pos_prev < (MULTIPLIER/2) && pos >= (MULTIPLIER/2))
          return counting_down;
      return !counting_down;
  }

};

// Custom logic for connecting the potentiometer with the SSR
class LogicHandler : public RunProcess {
  public:
    const PinReader& pin;
    const AnalogReader& pot;
    SSRController& ssr;
  
    LogicHandler(PinReader& pin_, AnalogReader& pot_, SSRController& ssr_)
      : pin(pin_), pot(pot_), ssr(ssr_)
    {
        
    }

    void setup() { }
    
    void run() {
      // Since we assume 50 Hz, we can change at most 100 Hz
      // so using the percent is easiest
      ssr.value = pot.get_percent();

      // Change mode, if the pin has changed
      if (pin.state && ssr.mode != Chunked) {        
        ssr.setChunked();
      } else if (!pin.state && ssr.mode != Dispersed) {
        ssr.setDispersed();
      }
    }
};

// Specialized writer to update the LCD display
class LCDUpdater : public RunProcess {
  public:

  LiquidCrystal& disp;
  AnalogReader& pot;
  SSRController& ssr;

  uint32_t last_print;
  uint32_t last_ssr_update;
  uint32_t ssr_elapsed;
  uint16_t updatePeriod;

  LCDUpdater(LiquidCrystal& disp_, AnalogReader& pot_, SSRController& ssr_, uint16_t updatePeriod_ = 500) 
    : disp(disp_),
      pot(pot_),
      ssr(ssr_),
      last_print(0),
      last_ssr_update(0),
      ssr_elapsed(0),
      updatePeriod(updatePeriod_)
  {
  }

  void setup() {
    // set up the LCD's number of columns and rows:
    lcd.begin(LCD_COLUMNS, LCD_ROWS);
    lcd.noBlink();

    // Prepare the display fixed parts
    lcd.setCursor(0, 0);
    lcd.print("Power: OFF  000%");
    lcd.setCursor(0, 1);
    lcd.print("Mode : F   00000");
  }

  void run() {
    // Keep track of the update duration by recording last change
    if (last_ssr_update != ssr.last_update) {
      ssr_elapsed = compute_elapsed(ssr.last_update, last_ssr_update);
      last_ssr_update = ssr.last_update;
    }

    // Check if we should update the display
    uint32_t current = millis();
    if (compute_elapsed(current, last_print) < updatePeriod)
      return;

    last_print = current;

    char linebuffer[16];

    lcd.setCursor(7, 0);
    if (ssr.value == 0)
      lcd.print("OFF");
    else if (ssr.value >= ssr.window_size)
      lcd.print("MAX");
    else
      lcd.print("ON ");
  
    lcd.setCursor(12, 0);
    sprintf(linebuffer, "%3d", pot.get_percent());
    lcd.print(linebuffer);
  
    lcd.setCursor(7, 1);
    lcd.print(ssr.mode == Chunked ? 'C' : 'D');
  
    lcd.setCursor(11, 1);
    sprintf(linebuffer, "%5d", ssr_elapsed);
    lcd.print(linebuffer);    
  }
};

#ifdef SERIAL_OUT
// Serial printer for debug
class DebugPrinter : public RunProcess {
  public:

  AnalogReader& pot;
  SSRController& ssr;

  uint32_t last_print;
  uint16_t updatePeriod;

  DebugPrinter(AnalogReader& pot_, SSRController& ssr_, uint16_t updatePeriod_ = 500) 
    : pot(pot_),
      ssr(ssr_),
      last_print(0),
      updatePeriod(updatePeriod_)
  {
  }
  
  void setup() {
    Serial.begin(9600);
  }

  void run() {
    uint32_t current = millis();
    if (compute_elapsed(current, last_print) < updatePeriod)
      return;

    // Update to avoid repeated prints
    last_print = current;
  
    Serial.print(pot.low_adc_threshold);
    Serial.print(", ");
    Serial.print(pot.high_adc_threshold);
    Serial.print(", ");
    Serial.print(pot.adc_range);
    Serial.print(", ");  
    Serial.print(pot.sensor_raw);
    Serial.print(", ");  
    Serial.println(pot.sensor);
  }
};
#endif

AnalogReader pot0(sensorPin);
PinReader modePin(togglePin);
SSRController ssr0(ssrPin);
LCDUpdater lcd0(lcd, pot0, ssr0);
LogicHandler lh(modePin, pot0, ssr0);

#ifdef SERIAL_OUT
DebugPrinter dbg(pot0, ssr0);
#endif

void setup() {
  RunProcess::AllSetup();
}

void loop() {
  RunProcess::AllRun();
}
