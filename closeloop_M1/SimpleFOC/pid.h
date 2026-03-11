#ifndef PID_H
#define PID_H

/******************************************************************************/
void PID_init(void);
float PID_velocity(float error);
float PID_angle(float error);
/******************************************************************************/

#endif

