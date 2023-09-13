#include <Arduino.h>
#include <Servo.h>

// constants for scaling servo stuff properly
const float PAN_MIN = -90;
const float PAN_MAX = 90;
const float PAN_SCL = -7.7777;
const long PAN_CENTER = 1500; //in us

const float TILT_MIN = -70.0;
const float TILT_MAX = 70.0;
const float TILT_SCL = 16.0;
const long TILT_CENTER = 1500; //in us

// constants for firing locations (in us)
const long PULL_FWD = 2320; // pull servo in forward position
const long PULL_BACK = 850; // pull servo in backward but not yet released position
const long RELEASE = 700;   // pull servo in released (shot) position

// firing timing/delays (in ms)
// must be unsigned long since millis() returns unsigned long
// and we need to work with it
const unsigned long PULL_DELAY = 900;    // how long it takes to pull from forward to back hold pos
const unsigned long RELEASE_DELAY = 200; // how long it takes to pull from back hold to release (this fires)
const unsigned long RELOAD_DELAY = 700;  // how long it takes to go back to forward pos

// other constants
const long CMD_BUF_LEN = 25;   // max length of command queue
const long AIM_SPEED = 60.0; // in ms/deg


Servo pull;
Servo pan;
Servo tilt;

void setup() {
  pull.attach(9);
  pan.attach(10);
  tilt.attach(11);
  Serial.begin(115200);
  delay(1000);
  Serial.write("started");
}

// lets us reset the board from software
// aka provides a way to do emergency off
void(* resetFunc) (void) = 0;

// utils to enforce bounds
int bound(long in, long min, long max) {
  if (in > max) { return max; } 
  else if (in < min) { return min; }
  else { return in; }
}
float bound(float in, float min, float max) {
  if (in > max) { return max; }
  else if (in < min) { return min; }
  else { return in; }
}

// tracker for G90/G91
bool relative = false;

// delay variables
// these are what we'll use to do async timing
unsigned long delay_time_ms = millis();
unsigned long pull_delay_ms = millis();

// funny bit to hold coordinates
struct coord {
    float pan;
    float tilt;
    coord(float pan_val, float tilt_val) {
        pan = pan_val;
        tilt = tilt_val;
    }
    long panUs() { return (long)(bound(pan, PAN_MIN, PAN_MAX)*PAN_SCL) + PAN_CENTER; }
    long tiltUs() { return (long)(bound(tilt, TILT_MIN, TILT_MAX)*TILT_SCL) + TILT_CENTER; }
    void update(float p, float t) {
        if (relative) {
            pan += p;
            tilt += t;
            delay_time_ms = (unsigned long)(max(p, t)*AIM_SPEED) + millis();
        } else {
            pan = p;
            tilt = t;
            float dp = abs(pan - p);
            float dt = abs(tilt - t);
            delay_time_ms = (unsigned long)(max(dp, dt)*AIM_SPEED) + millis();
        }
    }
    

};

// the possible states for the 
// firing sequence
enum class pullstate: int {
  rest = PULL_FWD+1,
  pull = PULL_BACK+1,
  hold = PULL_BACK,
  release = RELEASE,
  reload = PULL_FWD,
};

// our current state with respect to the
// pull, hold, release, reload, rest, repeat cycle
// we'll use this as the state tracker
// for our state machine
enum pullstate state = pullstate::rest;

// queue stuff
String cmdbuf[CMD_BUF_LEN];
long start = 0;
long end = 0;

// overall position tracker
coord curpos = coord(0L, 0L);

// shot counter: so we know to start 
// pulling back again once we get done shooting
// if we have more shots remaining
long shot_counter = 0L;

// add a command to the buffer
// deals with incrementing the shot counter if needed
// and moving the end pointer
void add_command(String cmd) {
  Serial.println("added command: " + cmd);
  if (cmd.indexOf(String("M3")) != -1) {
    Serial.println("here!");
    shot_counter++;
  }
  cmdbuf[end] = cmd;
  if (end == (CMD_BUF_LEN - 1)) {
      end = 0;
  } else {
      end += 1;
  }
}

String readString;
void loop() {
    // read new commands from serial if available
    if (Serial.available() > 0){
        readString = String("");
        while (true) {
            delay(3);  //delay to allow buffer to fill 
            char c = Serial.read();  //gets one byte from serial buffer
            if (c == '\n') { break; }
            readString += c; //makes the string readString
        }
        // add command to buffer
        add_command(readString);
    }

    // state transition logic for firing sequence FSM
    if (state == pullstate::rest) {
        // if we still have shots left to fire,
        // go from rest to pull, and begin pulling
        // the bow back
        if (shot_counter > 0L) {
            shot_counter--;
            state = pullstate::pull;
            Serial.println("rest -> pull");
            // set the delay so we continue after we're done pulling
            pull_delay_ms = PULL_DELAY + millis();
        }
    } else if (state == pullstate::pull) {
        if (pull_delay_ms < millis()) {
            // if we're done pulling, set the state to "hold"
            // since we're holding and waiting for the fire signal
            Serial.println("pull -> hold");
            state = pullstate::hold;
        }
    } else if (state == pullstate::release) {
        if (pull_delay_ms < millis()) {
            // if we're done releasing, set the state to "reload",
            // and set the delay to the reload delay
            Serial.println("release -> reload");
            state = pullstate::reload;
            pull_delay_ms = RELOAD_DELAY + millis();
        }
    } else if (state == pullstate::reload) {
        if (pull_delay_ms < millis()) {
            // once we're done reloading, we're back to the "rest" state
            Serial.println("reload -> rest");
            state = pullstate::rest;
        }
    }
    // update all the servos
    pan.writeMicroseconds(curpos.panUs());
    tilt.writeMicroseconds(curpos.tiltUs());

    // int(state) is always the constant we tuned at the start
    // for each location. So this sets the pull location
    pull.writeMicroseconds(int(state));

    // the rest of the code deals with new instructions

    // if start = end, there's nothing in the queue
    // so we just exit the loop (return = continue since
    // this isn't a real loop; just a function called over
    // and over again)
    if (start == end) { return; } // nothing to do, no commands in queue

    // if delay_time is still in the future, we need to wait longer
    // until it's passed, so we shouldn't do anything
    // delay time is the delay set by each action
    if (delay_time_ms > millis()) { return; } //nothing to do, waiting for thing to happen

    // string to store our command
    String todo = cmdbuf[start];
    // now that we looked up our command, increment the start pointer
    // so it points to the next command in the queue
    // we will deal with wrapping later
    start += 1;

    // dealing with the commands!
    
    // M999: emergency reset. Just resets the board.
    if (todo.startsWith("M999")) { resetFunc(); }
    // G90: set absolute positioning mode
    else if (todo.startsWith("G90")) { relative = false; }
    // G91: set relative positioning mode
    else if (todo.startsWith("G91")) { relative = true; }
    // G0: move to/by coordinates G0 P[pan angle] T[tilt angle]
    else if (todo.startsWith("G0")) {
        // string parsing!!! (fun!!!!!)
        // ex. todo = "G0 P40 T-10"
        // indexOf("P")+1 = 4 so we get todo[4:] = "40 T-10"
        // String.toFloat() just goes from start until non-digit/period-character
        long p = todo.substring(todo.indexOf(String("P"))+1).toFloat();
        long t = todo.substring(todo.indexOf(String("T"))+1).toFloat();

        Serial.println(p);
        Serial.println(t);

        // update global state
        // this also updates delay_time_ms
        curpos.update(p, t);

        Serial.println("made it here");
    }
    else if (todo.startsWith("G4")) {
        delay_time_ms = todo.substring(todo.indexOf("P")+1).toFloat() + millis();
    }
    else if (todo.startsWith("M3")) {
        if (pull_delay_ms > millis() || state != pullstate::hold) {
            // this means we are either a) still moving
            // or b) done moving but not at the hold state yet
            // in this case we should undo the pointer increment
            // so that next time around it'll still say we're on this command
            start--;
        } else {
            Serial.println("setting to release mode");
            // this means we are cleared to fire!
            // step 1: set state machine to release state, resuming the cycle
            state = pullstate::release;
            // step 2: set timers
            pull_delay_ms = RELEASE_DELAY + millis(); // for when we are done releasing and can start moving back to reload
            delay_time_ms = RELEASE_DELAY + millis(); // other actions should wait until we've fired bc we shouldn't move while firing
        }
    }
    // now deal with moving the start pointer if it 
    // looped. do this at the end since we want the
    // "start--;" in M3 to work properly
    if (start == CMD_BUF_LEN) {
        start = 0;
    }
}