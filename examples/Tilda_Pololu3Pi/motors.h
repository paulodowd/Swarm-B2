// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H

// Pin definitions.
#define L_PWM 10
#define L_DIR 16
#define R_PWM 9
#define R_DIR 15

// To make code more readable.
#define FWD LOW
#define BCK HIGH

#define MAX_PWM 180.0

// Class to operate the motor(s).
class Motors_c {
  public:
  
    // Constructor, must exist.
    Motors_c() {
      // leaving this empty.
      // ensure initialise() is called
      // instead.
    } 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    void initialise() {
      pinMode( L_PWM , OUTPUT );
      pinMode( L_DIR , OUTPUT );
      pinMode( R_PWM , OUTPUT );
      pinMode( R_DIR , OUTPUT );
    
      digitalWrite( L_DIR , HIGH );
      digitalWrite( R_DIR , HIGH );

      // Ensure we init stopped.
      setMotorsPWM( 0, 0 );
    }

    void setMotorsPWM( float left, float right ) {
      
      // Set directions based on sign
      if( left < 0 ) {
        digitalWrite( L_DIR, BCK );
      } else {
        digitalWrite( L_DIR, FWD );
      }
      if( right < 0 ) {
        digitalWrite( R_DIR, BCK );
      } else {
        digitalWrite( R_DIR, FWD );
      }

      // Sign used, now convert to abs 
      // magnitude.
      left = abs( left );
      right = abs( right );

      // Constrain values.
      // Could be done with an if statement
      left = constrain( left, 0, MAX_PWM );
      right = constrain( right, 0, MAX_PWM );

      // set power.
      analogWrite( L_PWM, left );
      analogWrite( R_PWM, right );

      // Done!
      return;
    }
    
};



#endif
