/************************************
,        .       .           .      , 
|        |       |           |     '| 
|    ,-: |-. ,-. |-. ,-. ,-. |-     | 
|    | | | | `-. | | |-' |-' |      | 
`--' `-` `-' `-' ' ' `-' `-' `-'    ' 
*************************************/

// this #ifndef stops this file
// from being included more than
// once by the compiler.
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

// Max requested power (mapped to 0–255 for analogWrite)
#define MAX_PWM 100
#define PWM_CAP 180

// Class to operate the motors.
class Motors_c {

  public:

    Motors_c() {
      // Use initialise() before driving the motors.
    }

    // Configure pins and put motors in a safe state
    void initialise() {
      pinMode(L_PWM, OUTPUT);
      pinMode(L_DIR, OUTPUT);
      pinMode(R_PWM, OUTPUT);
      pinMode(R_DIR, OUTPUT);

      analogWrite(L_PWM, 0);
      analogWrite(R_PWM, 0);
      digitalWrite(L_DIR, LOW);
      digitalWrite(R_DIR, LOW);
    } // End of initialise()


    // Drive motors with requested power (float), left/right.
    // Sign sets direction via DIR; magnitude is mapped to 0–255 PWM.
    void setPWM( float left_pwr, float right_pwr ) {

      // Direction from sign
      if ( left_pwr < 0 ) {
        digitalWrite(L_DIR, HIGH); 
      } else {
        digitalWrite(L_DIR, LOW);  
      }

      if ( right_pwr < 0 ) {
        digitalWrite(R_DIR, HIGH);  
      } else {
        digitalWrite(R_DIR, LOW);  
      }

      // Magnitude: abs -> clamp to [0, MAX_PWM] -> map to [0,255]
      float left_mag  = left_pwr  < 0 ? -left_pwr  : left_pwr;
      float right_mag = right_pwr < 0 ? -right_pwr : right_pwr;

      if (left_mag  > MAX_PWM) left_mag  = MAX_PWM;
      if (right_mag > MAX_PWM) right_mag = MAX_PWM;
      if (left_mag  < 0) left_mag  = 0;
      if (right_mag < 0) right_mag = 0;

      int left_cmd  = (int)((left_mag  / MAX_PWM) * PWM_CAP + 0.5f);   
      int right_cmd = (int)((right_mag / MAX_PWM) * PWM_CAP + 0.5f);   

      if (left_cmd  > 255) left_cmd  = 255;
      if (right_cmd > 255) right_cmd = 255;
      if (left_cmd  < 0)   left_cmd  = 0;
      if (right_cmd < 0)   right_cmd = 0;

      // Output PWM
      analogWrite( L_PWM, left_cmd );
      analogWrite( R_PWM, right_cmd );

      return;
    } // End of setPWM()

}; // End of Motors_c class definition.

#endif
