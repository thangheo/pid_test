#include <stdio.h>
#include <stdlib.h>
 #include <time.h>
#include <math.h>

#include "PID.h"

#define PID_TAU 0.02f
#define PID_BETA 0.21f
#define PID_ALPHA 0.1f
/* Controller parameters */
#define PID_KP  2.0f
#define PID_KI  0.5f
#define PID_KD  0.25f


// (1+Kd)s^2 + (Kp-a)s +ki=0
// 1.25s2 + (2-a)s + 0.5 =0
//
// #define PID_KP  -(PID_ALPHA*PID_BETA)/((1-PID_BETA)*(1-PID_BETA))
// #define PID_KI  PID_ALPHA/(1-PID_BETA)
// #define PID_KD  (PID_ALPHA*PID_BETA)/((1-PID_BETA)*(1-PID_BETA)*(1-PID_BETA))




#define PID_LIM_MIN -10.0f
#define PID_LIM_MAX  10.0f

#define PID_LIM_MIN_INT -5.0f
#define PID_LIM_MAX_INT  5.0f

#define SAMPLE_TIME_S 120.0f

/* Maximum run-time of simulation */
#define SIMULATION_TIME_MAX 72000.0f


float setpoint = 28.5f;

/* Simulated dynamical system (first order) */
float TestSystem_Update(float inp);
float get_random(void);
// SD of a population
float calculateSD(float data[],int len);

float calculateSD(float data[],int len) {
    double sum = 0.0, mean, SD = 0.0;
    int i;
    for (i = 0; i < len; ++i) {
        sum += data[i];
    }
    mean = sum / len;
    for (i = 0; i < len; ++i) {
        SD += pow(data[i] - setpoint, 2);
    }
    return sqrt(SD / len);
}

float error_mean(float data[],int len) {
    double ret = 0.0;
    int i;
    // for (i = 0; i < len; ++i) {
    //     sum += data[i];
    // }
    for (i = 0; i < len; ++i) {
        ret += abs(data[i] - setpoint);
    }
    return ret / len;
}
int main()
{
    /* Initialise PID controller */
    PIDController pid = { PID_KP, PID_KI, PID_KD,
                          PID_TAU,PID_BETA,
                          PID_LIM_MIN, PID_LIM_MAX,
			  PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                          SAMPLE_TIME_S };
    srand(time(NULL));

    PIDController_Init(&pid);

    /* Simulate response using test system */
    float data[(int)(SIMULATION_TIME_MAX/SAMPLE_TIME_S)];
    printf("Time (s)\tSystem Output\tControllerOutput\r\n");
    int i=0;
    for (float t = 0.0f; t <= SIMULATION_TIME_MAX; t += SAMPLE_TIME_S) {

        /* Get measurement from system */
        float measurement = TestSystem_Update(pid.out);
        data[i++] = measurement;
        int target =setpoint + 5*sin(2*3.14*t);
    
        /* Compute new control signal */
        PIDController_Update(&pid, setpoint, measurement);

        printf("%f\t%f\t%f\r\n", t, measurement, pid.out);

    }
    printf("\error mean = %.6f\n", error_mean(data,(int)(SIMULATION_TIME_MAX/SAMPLE_TIME_S)));
    printf("\nStandard Deviation = %.6f\n", calculateSD(data,(int)(SIMULATION_TIME_MAX/SAMPLE_TIME_S)));

    return 0;
}
float get_random(void){
    // srand(time(NULL));   // Initialization, should only be called once.
    float r =(rand()%10)/10.0-0.5;
    // printf("%1.2f\n",r);
    return r;
}
float TestSystem_Update(float inp) {

    static float temp = 45.0f;
    static const float alpha = 0.2f;
    float tmp_incr_coeff= 0.53215;
    // output = (SAMPLE_TIME_S * inp + output) / (1.0f + alpha * SAMPLE_TIME_S);
    get_random();
    temp = ( 1*temp + tmp_incr_coeff * inp + 3.1*get_random()) ;/// (1.0f + alpha * SAMPLE_TIME_S);
    //yn= a*y[n-1] + c*x
    // y(z)= a*y[z]z^-1 +cx[z]
    // x(z) = (y(z) -a*y(z)z^-1 )/c
    //G=y(z)/x(z) = c*(y[z]z^-1 +cx[z])/(y(z) -y(z)z^-1 )
    // poles : (y(z) -a*y(z)z^-1 ) =0
    // 1-a/z=0
    // z=a
    return temp;
}
