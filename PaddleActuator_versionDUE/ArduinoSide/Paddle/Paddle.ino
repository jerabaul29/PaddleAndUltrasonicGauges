#include <Arduino.h>
#include "PID_vmicroseconds.h"

// Protocol properties ---------------------------------------------------------
// the most significant bit of a byte
#define NBIT 7
// the default value to output if bug; take 2^(NBR_BITS_DATA_INT-1) if zero position paddle
// if 12 bits resolution: 2^(11) = 2048
#define DEFAULT_VALUE 2048
// the number of bits of the int sent by serial
// not used for simple protocol (only one data sent, no more for example to distinguish several sensors
// or modes). If used must agree with the Python definition
//#define NBR_BITS_DATA_INT 10

// debug mode ------------------------------------------------------------------
#define DEBUG_MODE false

// Frequency parameters --------------------------------------------------------
// number of microseconds to wait between readings of set values
// this value must agree with the Python program /input signal ('scan rate')
// 500 Hz is each 2 milli second is each 2000 micro second
#define NMICROS_READ_SETPOINT 2000UL
// number of microseconds to wait before performing again the PID control
// this value works fine but may be too fast
//#define NMICROS_PID_LOOP 100UL
// other value
#define NMICROS_PID_LOOP 500UL
// number of micro seconds to wait before allowed to ask for a new buffer
// to avoid asking several time in a row for the same buffer
#define NMICROS_REQUEST_BUFFER 100000UL
// how often sends feedback
#define NMICROS_FEEDBACK 50000UL

//Defines so the device can do a self reset ------------------------------------
// NOTE: this part is Arduino Due specific
#define SYSRESETREQ    (1<<2)
#define VECTKEY        (0x05fa0000UL)
#define VECTKEY_MASK   (0x0000ffffUL)
#define AIRCR          (*(uint32_t*)0xe000ed0cUL) // fixed arch-defined address
#define REQUEST_EXTERNAL_RESET (AIRCR=(AIRCR&VECTKEY_MASK)|VECTKEY|SYSRESETREQ);

// PID parameters --------------------------------------------------------------
// coefficients with dummy initializations
float PID_P = 1.;
float PID_I = 1.;
float PID_D = 1.;
int PID_S = 1;
// working variables with initialization
double Setpoint = DEFAULT_VALUE;
double Input = DEFAULT_VALUE;
double Output = 0;
// if some update of the output has been performed
bool performed_update_output = false;
// initialize the PID instance
PID myPID(&Input, &Output, &Setpoint, PID_P, PID_I, PID_D, PID_S, NMICROS_PID_LOOP);

// half size of the input buffer -----------------------------------------------
// should agree with the Python code; half size of the input buffer should be the size (in bytes!!) of the python transmitted buffer
// i.e. if use two bytes per set point, double NUMBER_OF_POINTS_PER_BUFFER in the Python code
// it should also be no more than half the total size of the Arduino Due custon RX buffer as defined in the modified core.
// in practise, take a little bit less to avoid overflow problems
#define HALF_INPUT_BUFFER 4096
//#define HALF_INPUT_BUFFER 2048

// variable for timing ---------------------------------------------------------
// timing of the set point update
unsigned long last_update_set_point;
// timiing for buffer request
unsigned long last_request_buffer;
// timing for sending feedback
unsigned long last_time_sent_feedback;
// time actuation
unsigned long actuation_beginning;
unsigned long actuation_end;

// properties of the PWM -------------------------------------------------------
// max value output (8 bits is default is 255)
#define PWM_RESOLUTION 255
// pins of the PWM (two pins)
#define PWM_PIN_A 11
#define PWM_PIN_B 3
#define ENABLE_PIN 13

// properties of the ADC -------------------------------------------------------
#define ADC_RESOLUTION 12
#define ADC_PIN A0

// variables for the filters ---------------------------------------------------
// position reading
int pos_1_before = DEFAULT_VALUE;
int pos_2_before = DEFAULT_VALUE;
int pos_3_before = DEFAULT_VALUE;
int current_reading;

// control
int control_1_before = 0;
int control_2_before = 0;
int control_3_before = 0;
int smoothed_control = 0;

// additional statistics -------------------------------------------------------
unsigned long number_of_update_set_point = 0;
unsigned long number_of_PID_loop = 0;
unsigned long number_loop_called = 0;
unsigned long number_of_feedback_send = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

     // open serial port on native USB; baud rate must be the same as in Python
     Serial.begin(115200);

    #if DEBUG_MODE
      Serial.println(F("M booted"));
    #endif

    // TAKE CARE OF ANALOG_READ RESOLUTION -------------------------------------
    // change the resolution to 12 bits and read A0 (12 bits ADC on Arduino Due)
    analogReadResolution(ADC_RESOLUTION);
    // prepare for reading
    pinMode(ADC_PIN,INPUT);

    // TAKE CARE OF PWM FREQUENCY AND RESOLUTION -------------------------------
    // no point increasing the PWM resolution, 8 bits is enough
    // increase the PWM frequency for avoiding the annoying PWM noise and allowing
    // faster control. This is done directly in variants.h

    // TAKE CARE OF MOTO SHIELD INIT -------------------------------------------
    // set pins function
    pinMode(PWM_PIN_A,OUTPUT);
    pinMode(PWM_PIN_B,OUTPUT);
    pinMode(ENABLE_PIN,OUTPUT);
    digitalWrite(ENABLE_PIN,LOW);

    // wait for ready signal ---------------------------------------------------
    wait_for_character('R');

    // get the PID parameters --------------------------------------------------
    get_PID_parameters();

    // prepare the PID object for performing control
    myPID.SetTunings(PID_P,PID_I,PID_D);
    myPID.SetControllerDirection(PID_S);
    myPID.SetOutputLimits(-PWM_RESOLUTION, PWM_RESOLUTION);

    // Now we are ready to do the actuation! -----------------------------------
    wait_for_character('X');

    // wait a few instants to receive the two first buffers and start ----------
    delay(1000);
    last_update_set_point = micros();
    last_request_buffer = micros();
    last_time_sent_feedback = micros();

    // start PID
    myPID.SetMode(AUTOMATIC);

    // beginning actuation
    actuation_beginning = millis();

    // start the MotoShield
    digitalWrite(ENABLE_PIN,HIGH);

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

    // send feedback -----------------------------------------------------------
    send_feedback();

    // check if time to update set point ---------------------------------------
    set_point_control();

    // read raw input and update input -----------------------------------------
    update_input();

    // call PID function -------------------------------------------------------
    performed_update_output = myPID.Compute();

    // write to PWM ------------------------------------------------------------
    if (performed_update_output){
      // no smoothing
      //write_to_PWM();

      // smoothing
      smooth_control();
      write_to_PWM_smoothed();
    }

    number_loop_called ++;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// diverse small helper functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void wait_for_character(char char_to_get){
    // wait for char_to_get
    // if it is well the next char received, continue
    // if it is not the next char received, reboot
    while(true){

        if (Serial.available()){

            char char_crrt = Serial.read();

            // if it is the right char acknowledge and continue
            if (char_crrt==char_to_get){
                Serial.print(char_crrt);
                break;
            }

            // if not the right char, reboot
            else{
                Serial.println(char_crrt);
                Serial.print("E that was not right, expected: ");
                Serial.println(char_to_get);
                delay(50);
                REQUEST_EXTERNAL_RESET
            }
        }
    }
}

void smooth_control(){
  // smooth the control value to reduce high frequency actuation mistake

  // compute smooth control
  smoothed_control = (Output*4 + control_1_before*3 + control_2_before*2 + control_3_before)/10;

  // prepare values for next actuation
  control_3_before = control_2_before;
  control_2_before = control_1_before;
  control_1_before = Output;

}

void write_to_PWM(){
    // write to PWM using two pins for having two directions control (H bridge)
    if (Output>0){
        analogWrite(PWM_PIN_A, 0);
        analogWrite(PWM_PIN_B, Output);
    }
    else{
        analogWrite(PWM_PIN_A, -Output);
        analogWrite(PWM_PIN_B, 0);
    }
}

void write_to_PWM_smoothed(){
  // write to PWM using two pins for having two directions control (H bridge)
  if (smoothed_control>0){
    analogWrite(PWM_PIN_A, 0);
    analogWrite(PWM_PIN_B, smoothed_control);
  }
  else{
    analogWrite(PWM_PIN_A, -smoothed_control);
    analogWrite(PWM_PIN_B, 0);
  }
  }

void update_input(){
    // update input; do some filtering to smooth the signal

    // NOTE: this is a very simple filter; maybe something more sophisticated would be better
    pos_3_before = pos_2_before;
    pos_2_before = pos_1_before;
    pos_1_before = Input;
    current_reading = analogRead(ADC_PIN);

    Input = (current_reading*4 + pos_1_before*3 + pos_2_before*2 + pos_3_before)/10;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// functions to get and compute the PID parameters
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float get_one_PID_parameter(char param_Key){
    // get one PID parameter and assemble it
    // in case of a problem, reboot

    // check that it is the right parameter coming
    wait_for_character(param_Key);

    // get the mantissa
    while (Serial.available()==0);
    byte mantissa = Serial.read();
    // get the exponent
    while (Serial.available()==0);
    byte exponent = Serial.read();
    // compute the PID parameter
    float PID_parameter = float(mantissa)*pow(10.0,float(exponent)-128.0);

    #if DEBUG_MODE
      Serial.println(PID_parameter*1000);
    #endif

    return(PID_parameter);
}

int get_actuation_sign(){
    // get the actuation sign
    // in case of a problem, reboot

    // check that it is the right parameter coming
    wait_for_character('S');

    while (Serial.available()==0);
    byte sign_byte = Serial.read();

    int sign_actuation;
    sign_actuation = int(sign_byte);

    //#if DEBUG_MODE
    Serial.println(sign_actuation);
    //#endif

    return(sign_actuation);
}

void get_PID_parameters(){
    // receive all PID parameters needed for actuation
    // in case of a problem, reboot

    PID_P = get_one_PID_parameter('P');
    PID_I = get_one_PID_parameter('I');
    PID_D = get_one_PID_parameter('D');
    PID_S = get_actuation_sign();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function to take care of the reading of the set point from serial buffer
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void set_point_control(){
    // the set point control function to be called at each loop iteration

    // check if set point should be updated
    if (micros() - last_update_set_point >= NMICROS_READ_SETPOINT){

        int number_bits_available = Serial.available();

        // if buffer empty, it is the end of actuation, reboot
        if (number_bits_available == 0){

            actuation_end = millis();
            Serial.println('Z');
            digitalWrite(ENABLE_PIN,LOW);

            send_post_actuation_data();

            delay(100);
            REQUEST_EXTERNAL_RESET
        }

        // otherwise, buffer is not empty, update buffer and last read time
        // do not use micros but increment on last read time, to avoid slow drift
        // unsigned long take care of wrapping
        Setpoint = assemble_bytes_protocol();
        last_update_set_point += NMICROS_READ_SETPOINT;

        // if less than half a buffer left, and longer than a typical wait time, ask Python for a new buffer
        if ((number_bits_available < HALF_INPUT_BUFFER)&&(micros() - last_request_buffer >= NMICROS_REQUEST_BUFFER)){
            Serial.print('T');
            last_request_buffer = micros();
        }

        number_of_update_set_point ++;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// function for decoding protocol
// tries to read on Serial port each time called
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int assemble_bytes_protocol(){
  // assemble byte1 and byte2 according to the protocol to
  // compute an int

  // the bytes that will be read
  byte byte1;
  byte byte2;

  // initialize the assembled value to return -----------------------------------------------------------------
  int value_assembled = 0;

  // check that enough bytes available and that it is well a packet -------------------------------------------
  // read if 2 bytes or more available
  if(Serial.available()>1){

    // read first byte; Wait to read second, so that if error and not start of a packet
    // next time assemble_bytes_protocol is called it will be the beginning of a packet
    byte1 = Serial.read();

    // if not the start of a packet, stop here
    // next call will hopefully start at the beginning of a packet!
    if (bitRead(byte1,NBIT)==0){
      Serial.println(F("E error: not start of a packet!"));

      //Serial.print(F("in ascii, this was: "));
      //Serial.println(byte1);

      // try to read next
      Serial.println(F("try to read next"));

      int result_next = assemble_bytes_protocol();
      return(result_next);
    }

    // if it was the start of a packet, read byte 2
    byte2 = Serial.read();
    // and check it is the continuation of a packet; otherwise, break
    if (bitRead(byte2,NBIT)==1){
      Serial.println(F("E error: not continuation of a packet!"));

      //Serial.print(F("in ascii, this was: "));
      //Serial.println(byte2);

      // try to read next
      Serial.println(F("try to read next"));

      int result_next = assemble_bytes_protocol();
      return(result_next);
    }

    // now we know that everything is OK! ---------------------------------------------------------------------
    // do the recombination -----------------------------------------------------------------------------------

    // second byte is low weight bits
    value_assembled = int(byte2);

    // first part is high weight bits
    // note: the following approach only works for simple protocol with only start stop and messsage
    int high_value = int(byte1) - 128;
    value_assembled += high_value*128;

    // for debugging
    #if DEBUG_MODE
      Serial.print(F("M protocol deciphered: "));
      Serial.println(value_assembled);
    #endif

    // final result
    return(value_assembled);

  }

  // if no 2 bytes or more to read, say it and return DEFAULT_VALUE!
  else{
    // comment when debugging
    //SerialUSB.println(F("Error: empty SerialUSB buffer!"));
    return(DEFAULT_VALUE);
  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION TO SEND FEEDBACK
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void send_feedback(){

  if (micros() - last_time_sent_feedback >= NMICROS_FEEDBACK){

    // update timer
    last_time_sent_feedback = micros();

    // send current set point, key A
    Serial.println('A');
    Serial.println(Setpoint);

    // send current position, key B
    Serial.println('B');
    Serial.println(Input);

    // send current control, key C
    Serial.println('C');
    Serial.println(Output);

    // send current time micro seconds, key D
    Serial.println('D');
    Serial.println(micros());

    number_of_feedback_send ++;

  }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTION TO SEND POST ACTUATION DATA
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void send_post_actuation_data(){

  // send total actuation time
  unsigned long actuation_total_time;
  actuation_total_time = actuation_end - actuation_beginning;
  Serial.println('T');
  Serial.println(actuation_total_time);

  // send number of set point updates
  Serial.println('U');
  Serial.println(number_of_update_set_point);

  // send number of times loop was run
  Serial.println('V');
  Serial.println(number_loop_called);

  // send number of feedback that were sent
  Serial.println('W');
  Serial.println(number_of_feedback_send);


}
