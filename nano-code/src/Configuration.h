#ifndef CONFIG
#define CONFIG

//* misc settings
// enable/disable debug printing
// #define DEBUG
// serial comms baudrate
#define BAUDRATE 115200
//command buffer size
#define CMD_BUF_LEN 8L

//* motion limits and scale factors and stuff

// how fast the servos move while aiming
// (determines delay times)
#define AIM_SPEED 60.0 // in ms/deg

// pan axis
#define PAN_MAX 90.0    // degrees
#define PAN_MIN -90.0     // degrees
#define PAN_SCL 7.7777   // scale factor, us/deg
#define PAN_CENTER 1460L //in us

// tilt axis
#define TILT_MAX 40.0    // degrees
#define TILT_MIN -35.0     // degrees
#define TILT_SCL 7.39     // scale factor, us/deg
#define TILT_CENTER 1520L //in us

//* firing sequence constants
// positions
#define PULL_FWD 2320L // us
#define PULL_BACK 850L // us
#define RELEASE 700L   // us

// delays
#define PULL_DELAY 900UL    // ms
#define RELEASE_DELAY 300UL //200UL // ms
#define RELOAD_DELAY 900UL  // ms


#endif
