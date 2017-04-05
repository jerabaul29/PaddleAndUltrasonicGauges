// Debug mode: better looking serial output
#define DebugMode true

// all gauges to read
#define Gauge_A A0
#define Gauge_B A1
#define Gauge_C A2
#define Gauge_D A3
#define Gauge_E A4
#define Gauge_F A5

// value of the different gauges readings
int value_A;
int value_B;
int value_C;
int value_D;
int value_E;
int value_F;

// time between readings in ms, ie 20 ms is 50 Hz, ie 10ms is 100Hz
#define time_between_readings 10000UL
// variables for reading intervals
unsigned long last_reading = 0;
unsigned long current_time = 0;


void setup(){

  // open serial port
  Serial.begin(57600);

  // use the analog pins as inputs
  pinMode(Gauge_A,INPUT);  
  pinMode(Gauge_B,INPUT);  
  pinMode(Gauge_C,INPUT);  
  pinMode(Gauge_D,INPUT);  
  pinMode(Gauge_E,INPUT);
  pinMode(Gauge_F,INPUT);

  // prepare for the first reading
  last_reading = micros();
  
}

void loop(){

  // current time
  current_time = micros();

  // check if time to do a new reading
  if (current_time-last_reading >= time_between_readings){

    // update current time
    last_reading = current_time;

    // do the readings
    value_A = analogRead(Gauge_A);
    value_B = analogRead(Gauge_B);
    value_C = analogRead(Gauge_C);
    value_D = analogRead(Gauge_D);
    value_E = analogRead(Gauge_E);
    value_F = analogRead(Gauge_F);

    // post the values
    post_gauge(value_A,'A');
    post_gauge(value_B,'B');
    post_gauge(value_C,'C');
    post_gauge(value_D,'D');
    post_gauge(value_E,'E');
    post_gauge(value_F,'F');

    // jump one line if degug for clarity
    #if DebugMode
      Serial.println();
    #endif
}

  
}

void post_gauge(int value, char prefix){
  // helper function: output a key for each probe and the current reading

  Serial.print(prefix);
  Serial.print(value);
  
  #if DebugMode
    Serial.println();
  #endif

}

