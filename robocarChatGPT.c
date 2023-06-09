/*
 *   ROBOSAMPLE.C -- A sample/template for RoboKar program with uCOS-II
 *   Written by: Rosbi Mamat 6/5/2014
 *   Updated : 1/5/2023 Modified to show proximity & light sensor usage
 */

 /*
 Line Sensor Values:

 000 = 0 = off track
 001 = 1 = right sensor on track
 010 = 2 = exactly on track
 011 = 3 = middle and right on track
 100 = 4 = left sensor on track
 101 = 5 = left & right sensor on track, but not middle sensor (Not Possible)
 110 = 6 = right and middle sensor on track
 111 = 7 = all sensors on track
 
 */

#include "..\inc\kernel.h"                  /* Always include these to use uCOS-II      */
#include "..\inc\hal_robo.h"                /*   and RoboKar HAL                        */

#define TASK_STK_SZ            128          /* Size of each task's stacks (# of bytes)  */
#define TASK_START_PRIO          1          /* Highest priority                         */
#define TASK_CHKCOLLIDE_PRIO     2
#define TASK_CTRLMOTOR_PRIO      3
#define TASK_NAVIG_PRIO          4          /* Lowest priority                          */

OS_STK TaskStartStk[TASK_STK_SZ];           /* TaskStartTask stack                      */
OS_STK ChkCollideStk[TASK_STK_SZ];          /* Task StopOnCollide stack                 */
OS_STK CtrlmotorStk[TASK_STK_SZ];           /* Task CtrlMotors stack                    */
OS_STK NavigStk[TASK_STK_SZ];               /* Task NavigRobot stack                    */

/* ------ Global shared variable -------*/
/* Ideally, this should be protected by a semaphore etc */
struct robostate
{
    int rspeed;                             /* right motor speed  (-100 -- +100)        */
    int lspeed;                             /* leftt motor speed  (-100 -- +100)        */
    char obstacle;                          /* obstacle? 1 = yes, 0 = no                */

    // PID Properties
    double KP;                              /* Adjustment value of P value of PID  */
    double KD;                              /* Adjustment value of D value of PID */
    double KI;                           /* Adjustment value of I of PID */
    double lastError;                       /* last error from Goal */
    double integral;                     /* integral value of PID */
    int goal;                               /* goal is to be on the center of the line (010 or 2)*/
    int prevLeftSpeed;
    int prevRightSpeed;
    int inRoundabout;
    // int prevSense;
    // int prev2Sense;
} myrobot;

int constrain(int value, int lowerBound, int upperBound) {
    if (value < lowerBound) {
        return lowerBound;
    }
    else if (value > upperBound) {
        return upperBound;
    }
    else {
        return value;
    }
}


/*------High pririority task----------*/
void CheckCollision (void *data)
{
    for(;;)
    {
        // myrobot.timer++;
        if ( (robo_proxSensor() == 1) )             /* obstacle?                         */
            myrobot.obstacle = 1;                   /* signal obstacle present           */
        else
            myrobot.obstacle = 0;                   /* signal no obstacle                */

		OSTimeDlyHMSM(0, 0, 0, 100);                /* Task period ~ 100 ms              */
    }
}

/* Control robot Motors TASK */
void CntrlMotors (void *data)
{
    int speed_r, speed_l;

    for(;;)
    {
        speed_r = myrobot.rspeed;
        speed_l = myrobot.lspeed;
        robo_motorSpeed(speed_l, speed_r);
        OSTimeDlyHMSM(0, 0, 0, 10);                /* Task period ~ 250 ms              */
    }
}

/* --- Task for navigating robot ----
 * Write you own navigation task here
 */

void Navig (void *data)
{
    for (;;)
    {
        // if (myrobot.obstacle == 1)                  /* If blocked then reverse              */
        // {
        //     myrobot.rspeed   = -LOW_SPEED;          /* REVERSE */
        //     myrobot.lspeed   = -LOW_SPEED;
        // }
        // else                                        /* obstacle is far away & no collision   */
        // {
        //     myrobot.rspeed   = MEDIUM_SPEED;        /* move forward with medium speed        */
        //     myrobot.lspeed   = MEDIUM_SPEED;
        // }

		// if (robo_lightSensor() > 80)                /* it is too bright, I'm photophobia     */
		// {
		// 	myrobot.rspeed   = -LOW_SPEED;          /* turn right to avoid                   */
        //     myrobot.lspeed   =  LOW_SPEED;
		// }


        // int position = robo_lineSensor();

        // if(myrobot.timer == 250) {
        //     robo_Honk();
        //     myrobot.KP = 0.06;
        // }

        // if(myrobot.timer == 350) {
        //     robo_Honk();
        //     myrobot.KP = 0.03;
        // }

        if (myrobot.obstacle == 1) {
            myrobot.lspeed = STOP_SPEED;
            myrobot.rspeed = STOP_SPEED;
        } else {


            int sense = myrobot.goal;

            int sensorValue = robo_lineSensor();

            if (myrobot.inRoundabout == 1) {
                // myrobot.lspeed = LOW_SPEED;
                // myrobot.rspeed = LOW_SPEED;
                myrobot.KP = 1.2;
            } else {
                myrobot.KP = 0.03;
            }

            if(sensorValue == 0) {
                // myrobot.lspeed = - 50 - (myrobot.integral * myrobot.KI * 20);
                // myrobot.rspeed = - 50 + (myrobot.integral * myrobot.KI * 20);
                // if(myrobot.prev2Sense == 2 && myrobot.prevSense == 7) {
                //     robo_Honk();
                //     robo_LED_toggle();
                //     myrobot.inRoundabout = 1;
                // }

                myrobot.lspeed = - (myrobot.prevRightSpeed * 1.1);
                myrobot.rspeed = - (myrobot.prevLeftSpeed * 1.1);


                myrobot.KI = 0.0008; // 0.0008
            } else {
                // myrobot.KP = 0.045; 

                switch (sensorValue) {
                    case 2: sense = 1000;
                            break;
                    
                    case 1: sense = 2000;
                            break;

                    case 3: sense = 1500;
                            break;

                    case 4: sense = 0;
                            break;

                    case 5: sense = 1000;
                            break;

                    case 6: sense = 500;
                            break;
                    
                    case 7: sense = 1000;
                            break;

                    default: sense = 1000;
                            break;
                }


                // Calculate error

                int error = myrobot.goal - sense;

                myrobot.integral = myrobot.integral + error;


                // Calculate motor adjustments
                int adjustment = (myrobot.KP * error) + (myrobot.KI * (myrobot.integral)) + (myrobot.KD * (error - myrobot.lastError));
                // int adjustment = (myrobot.KP * error) + (myrobot.KD * (error - myrobot.lastError));

                // Store error for the next iteration
                myrobot.lastError = error;

                // Adjust motors
                myrobot.lspeed = constrain((60 - adjustment), -100 , 100);
                myrobot.rspeed = constrain((60 + adjustment), -100 , 100);
                

                myrobot.prevLeftSpeed = myrobot.lspeed;
                myrobot.prevRightSpeed = myrobot.rspeed;
                // myrobot.prev2Sense = myrobot.prevSense;
                // myrobot.prevSense = sensorValue;
            }

            OSTimeDlyHMSM(0, 0, 0, 10);                /* Task period ~ 500 ms                  */
        }

        
    }
}


/*------Highest pririority task----------*/
/* Create all other tasks here           */
void TaskStart( void *data )
{
    OS_ticks_init();                                        /* enable RTOS timer tick        */

    OSTaskCreate(CheckCollision,                            /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&ChkCollideStk[TASK_STK_SZ - 1],    /* stack allocated to task       */
                TASK_CHKCOLLIDE_PRIO);                      /* priority of task              */

    OSTaskCreate(CntrlMotors,                               /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&CtrlmotorStk[TASK_STK_SZ - 1],     /* stack allocated to task       */
                TASK_CTRLMOTOR_PRIO);                       /* priority of task              */

    OSTaskCreate(Navig,                                     /* Task function                 */
                (void *)0,                                  /* nothing passed to task        */
                (void *)&NavigStk[TASK_STK_SZ - 1],         /* stack allocated to task       */
                TASK_NAVIG_PRIO);                           /* priority of task              */

    while(1)
    {
        OSTimeDlyHMSM(0, 0, 5, 0);                          /* Task period ~ 5 secs          */
       // robo_LED_toggle();                                  /* Show that we are alive        */
    }

}

int main( void )
{
    robo_Setup();                                          /* initialize HAL for RoboKar     */
    OSInit();                                              /* initialize UCOS-II kernel      */

    robo_motorSpeed(STOP_SPEED, STOP_SPEED);               /* Stop the robot                 */
    myrobot.rspeed   = STOP_SPEED;                         /* Initialize myrobot states      */
    myrobot.lspeed   = STOP_SPEED;
    myrobot.obstacle = 0;                                  /*  No collisioin                 */
    // myrobot.flag = false;
    myrobot.KP = 0.03; //0,03
    myrobot.KD = 0.4; //0.07
    myrobot.KI = 0.0005; //0.0004
    myrobot.lastError = 0.0;
    myrobot.integral = 0.0;
    // myrobot.timer = 0;
    myrobot.prevLeftSpeed = 0;
    myrobot.prevRightSpeed = 0;
    myrobot.inRoundabout = 0;
    // myrobot.prevSense = 0;
    // myrobot.prev2Sense = 0;
    myrobot.goal = 1000;                                      /* goal is to follow on the middle sensor */

    OSTaskCreate(TaskStart,                                /* create TaskStart Task          */
                (void *)0,
                (void *)&TaskStartStk[TASK_STK_SZ - 1],
                TASK_START_PRIO);
	robo_Honk(); robo_wait4goPress();                      /* Wait for to GO                 */
    OSStart();                                             /* Start multitasking             */
    while (1);                                             /* die here                       */
}
