//Contains the declaration of the state variables for the control loop  

#ifndef __STATE_H__
#define __STATE_H__

#define LoggerSize 1000



//interrupt vars

extern volatile int U;  //control effort (abs)
extern volatile float r;  //setpoint
extern volatile float y;  // measured angle
extern volatile float v;  // estimated velocity (velocity loop)
extern volatile float yw;
extern volatile float yw_1;
extern volatile float e;  // e = r-y (error)
extern volatile float p;  // proportional effort
extern volatile float i;  // integral effort

extern volatile float u;  //real control effort (not abs)
extern volatile float u_1;
extern volatile float e_1;
extern volatile float u_2;
extern volatile float e_2;
extern volatile float u_3;
extern volatile float e_3;
extern volatile long counter;

extern volatile long wrap_count;
extern volatile float y_1;

extern volatile long step_count;  //For step/dir interrupt
extern int stepNumber; // step index for cal routine


extern volatile float ITerm;
extern volatile float DTerm;
//extern volatile char mode;
extern int dir;

extern bool print_yw;     //for step response, under development...
extern float StepAmplitude; //for step response, under development...
extern bool log_position;      //for step response, under development...
extern bool log_velocity;      //for step response, under development...
extern bool log_torque;      //for step response, under development...
extern bool log_error;      //for step response, under development...
extern int StepStart;      //for step response, under development...
extern int StepEnd;      //for step response, under development...
extern float log_yw[LoggerSize];      //for step response, under development...
extern float log_v[LoggerSize];      //for step response, under development...
extern float log_u[LoggerSize];      //for step response, under development...
extern float log_r[LoggerSize];      //for step response, under development...
extern bool ResetLogger;

extern volatile int StartStep;
extern volatile int StopStep;

#endif








