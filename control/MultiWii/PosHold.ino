typedef struct PID_PARAM_ {
  float kP;
  float kI;
  float kD;
  float Imax;
} PID_PARAM;

PID_PARAM posholdPID_PARAM;
PID_PARAM poshold_ratePID_PARAM;
PID_PARAM navPID_PARAM;

typedef struct PID_ {
  float   integrator; // integrator value
  int32_t last_input; // last input for derivative
  float   lastderivative; // last derivative for low-pass filter
  float   output;
  float   derivative;
} PID;
PID posholdPID[2];
PID poshold_ratePID[2];
PID navPID[2];

int32_t get_P(int32_t error, struct PID_PARAM_* pid) {
  return (float)error * pid->kP;
}

int32_t get_I(int32_t error, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) {
  pid->integrator += ((float)error * pid_param->kI) * *dt;
  pid->integrator = constrain(pid->integrator,-pid_param->Imax,pid_param->Imax);
  return pid->integrator;
}
  
int32_t get_D(int32_t input, float* dt, struct PID_* pid, struct PID_PARAM_* pid_param) { // dt in milliseconds
  pid->derivative = (input - pid->last_input) / *dt;

  /// Low pass filter cut frequency for derivative calculation.
  float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
  // Examples for _filter:
  // f_cut = 10 Hz -> _filter = 15.9155e-3
  // f_cut = 15 Hz -> _filter = 10.6103e-3
  // f_cut = 20 Hz -> _filter =  7.9577e-3
  // f_cut = 25 Hz -> _filter =  6.3662e-3
  // f_cut = 30 Hz -> _filter =  5.3052e-3

  // discrete low pass filter, cuts out the
  // high frequency noise that can drive the controller crazy
  pid->derivative = pid->lastderivative + (*dt / ( filter + *dt)) * (pid->derivative - pid->lastderivative);
  // update state
  pid->last_input = input;
  pid->lastderivative    = pid->derivative;
  // add in derivative component
  return pid_param->kD * pid->derivative;
}

void reset_PID(struct PID_* pid) {
  pid->integrator = 0;
  pid->last_input = 0;
  pid->lastderivative = 0;
}

#define _X 1
#define _Y 0

#define NAV_SPEED_MAX              20    // cm/sec
#define NAV_BANK_MAX               500  // 5deg max banking when navigating (just for security and testing)

static float  dTnav;            // Delta Time in milliseconds for navigation computations, updated with every good GPS read
static int16_t actual_speed[2] = {0,0};

// The difference between the desired rate of travel and the actual rate of travel
// updated after GPS read - 5-10hz
static int16_t rate_error[2];
static int32_t error[2];

void GPS_NewData() {
  static int32_t sonarTimestamp[2] = {0,0};
  if(srf08_ctx.readAt[1] <= sonarTimestamp[0] || srf08_ctx.readAt[2] <= sonarTimestamp[1]) return; // sonars haven't been updated
  sonarTimestamp[0] = srf08_ctx.readAt[1];
  sonarTimestamp[1] = srf08_ctx.readAt[2];
  
  uint8_t axis;

  static uint8_t GPS_pids_initialized;
  if (!GPS_pids_initialized) {
    GPS_set_pids();
    GPS_pids_initialized = 1;
  } 

  #if !defined(DONT_RESET_HOME_AT_ARM)
    if (!f.ARMED) {f.GPS_FIX_HOME = 0;}                                          //Clear home position if disarmed
  #endif
  if (!f.GPS_FIX_HOME && f.ARMED) {        //if home is not set set home position to WP#0 and activate it
    GPS_reset_home_position();
  }
  if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
     GPS_distanceToHome = 0;
     GPS_directionToHome = 0;
  }

  //dTnav calculation
  //Time for calculating x,y speed and navigation pids
  static uint32_t nav_loopTimer;
  dTnav = (float)(millis() - nav_loopTimer)/ 1000.0;
  nav_loopTimer = millis();

  //calculate distance and bearings for gui and other stuff continously - From home to copter
  uint32_t dist;
  int32_t  dir;
  GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_home[LAT],&GPS_home[LON],&dist,&dir);
  GPS_distanceToHome = dist;
  GPS_directionToHome = dir/100;
  
  GPS_coord[LON] = srf08_ctx.range[1]; // X, forward
  GPS_coord[LAT] = srf08_ctx.range[2]; // Y, left/right

  if (!f.GPS_FIX_HOME) {     //If we don't have home set, do not display anything
     GPS_distanceToHome = 0;
     GPS_directionToHome = 0;
  }
  
  //calculate the current velocity based on gps coordinates continously to get a valid speed at the moment when we start navigating
  GPS_calc_velocity();        
  
  if (f.GPS_HOLD_MODE) {    //ok we are navigating 
    //do gps nav calculations here, these are common for nav and poshold  
    // inekhay: already calculated above
    //GPS_distance_cm_bearing(&GPS_coord[LAT],&GPS_coord[LON],&GPS_WP[LAT],&GPS_WP[LON],&wp_distance,&target_bearing);
    GPS_calc_location_error(&GPS_home[LAT],&GPS_home[LON],&GPS_coord[LAT],&GPS_coord[LON]);

    //Desired output is in nav_lat and nav_lon where 1deg inclination is 100 
    GPS_calc_poshold();
  } //end of gps calcs  
}

void GPS_reset_home_position() {
    GPS_home[LAT] = GPS_coord[LAT];
    GPS_home[LON] = GPS_coord[LON];
    //Set ground altitude
    f.GPS_FIX_HOME = 1;
}

//reset navigation (stop the navigation processor, and clear nav)
void GPS_reset_nav() {
  uint8_t i;
  
  for(i=0;i<2;i++) {
    GPS_angle[i] = 0;
    nav_rated[i] = 0;
    nav[i] = 0;
    reset_PID(&posholdPID[i]);
    reset_PID(&poshold_ratePID[i]);
    reset_PID(&navPID[i]);
  }
}

//Get the relevant P I D values and set the PID controllers 
void GPS_set_pids() {
  posholdPID_PARAM.kP   = (float)conf.P8[PIDPOS]/100.0;
  posholdPID_PARAM.kI   = (float)conf.I8[PIDPOS]/100.0;
  posholdPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
  
  poshold_ratePID_PARAM.kP   = (float)conf.P8[PIDPOSR]/10.0;
  poshold_ratePID_PARAM.kI   = (float)conf.I8[PIDPOSR]/100.0;
  poshold_ratePID_PARAM.kD   = (float)conf.D8[PIDPOSR]/1000.0;
  poshold_ratePID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
  
  navPID_PARAM.kP   = (float)conf.P8[PIDNAVR]/10.0;
  navPID_PARAM.kI   = (float)conf.I8[PIDNAVR]/100.0;
  navPID_PARAM.kD   = (float)conf.D8[PIDNAVR]/1000.0;
  navPID_PARAM.Imax = POSHOLD_RATE_IMAX * 100;
}

//It was mobed here since even i2cgps code needs it
int32_t wrap_18000(int32_t ang) {
  if (ang > 18000)  ang -= 36000;
  if (ang < -18000) ang += 36000;
  return ang;
}

////////////////////////////////////////////////////////////////////////////////////
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t* lat1, int32_t* lon1, int32_t* lat2, int32_t* lon2,uint32_t* dist, int32_t* bearing) {
  float dLat = *lat2 - *lat1;                                    // difference of latitude in 1/10 000 000 degrees
  float dLon = -(*lon2 - *lon1);
  *dist = sqrt(sq(dLat) + sq(dLon));
  
  *bearing = 9000.0f + atan2(-dLat, dLon) * 5729.57795f;      //Convert the output redians to 100xdeg
  if (*bearing < 0) *bearing += 36000;
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate our current speed vector from gps position data
//
static void GPS_calc_velocity(){
  static int16_t speed_old[2] = {0,0};
  static int32_t last[2] = {0,0};
  static uint8_t init = 0;
  // y_GPS_speed positve = Up
  // x_GPS_speed positve = Right

  if (init) {
    float tmp = 1.0/dTnav;
    actual_speed[_X] = -(float)(GPS_coord[LON] - last[LON]) * tmp;
    actual_speed[_Y] = (float)(GPS_coord[LAT]  - last[LAT])  * tmp;
  
    actual_speed[_X] = (actual_speed[_X] + speed_old[_X]) / 2;
    actual_speed[_Y] = (actual_speed[_Y] + speed_old[_Y]) / 2;
  
    speed_old[_X] = actual_speed[_X];
    speed_old[_Y] = actual_speed[_Y];
  }
  GPS_speed = sqrt(sq(actual_speed[_X]) + sq(actual_speed[_Y]));
  init=1;

  last[LON] = GPS_coord[LON];
  last[LAT] = GPS_coord[LAT];
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate a location error between two gps coordinates
// Because we are using lat and lon to do our distance errors here's a quick chart:
//   100  = 1m
//  1000  = 11m    = 36 feet
//  1800  = 19.80m = 60 feet
//  3000  = 33m
// 10000  = 111m
//
static void GPS_calc_location_error( int32_t* target_lat, int32_t* target_lng, int32_t* gps_lat, int32_t* gps_lng ) 
{
  error[LON] = -(*target_lng - *gps_lng);  // X Error
  error[LAT] = *target_lat - *gps_lat; // Y Error
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate nav_lat and nav_lon from the x and y error and the speed
//
static void GPS_calc_poshold() {
  int32_t d;
  int32_t target_speed;
  uint8_t axis;
  
  for (axis=0;axis<2;axis++) {
    target_speed = get_P(error[axis], &posholdPID_PARAM); // calculate desired speed from lat/lon error
    target_speed = constrain(target_speed,-NAV_SPEED_MAX,NAV_SPEED_MAX);
    rate_error[axis] = target_speed - actual_speed[axis]; // calc the speed error

    int32_t p = get_P(rate_error[axis],                                               &poshold_ratePID_PARAM);
    int32_t i = get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
    d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);

    if(axis == _X) {
      debug[0] = p;
      debug[1] = d;
    }
    else if(axis == _Y) {
      debug[2] = p;
      debug[3] = d;
    }
    
    /*
    nav[axis]      =
        get_P(rate_error[axis],                                               &poshold_ratePID_PARAM)
       +get_I(rate_error[axis] + error[axis], &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
    d = get_D(error[axis],                    &dTnav, &poshold_ratePID[axis], &poshold_ratePID_PARAM);
    */

    nav[axis] = p + i;
    d = constrain(d, -2000, 2000);
    // get rid of noise
/*    
    #if defined(GPS_LOW_SPEED_D_FILTER)
      if(abs(actual_speed[axis]) < 50) d = 0;
    #endif
*/

    nav[axis] += d;
    nav[axis]  = constrain(nav[axis], -NAV_BANK_MAX, NAV_BANK_MAX);
    navPID[axis].integrator = poshold_ratePID[axis].integrator;
  }
}


