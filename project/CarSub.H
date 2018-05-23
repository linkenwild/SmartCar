/*
**==============================================================================
** CarSub.H:             -- by Dr. ZhuoQing, 2011-12-3
**
**  Description:
**
**    The demo program for smart car competition. In the elec-magnetic group
**  in which car should be driver in:
**  (1) The car body must be erectly move. Only the two rears wheels be on the 
**      gound.
**  (2) Using the magnetic field to guild the car to running along the road.
**
**==============================================================================
*/
#ifndef __CarSub__
#define __CarSub__
//------------------------------------------------------------------------------
#ifdef CarSub_GLOBALS
    #define CARSUB_EXT
#else
    #define CARSUB_EXT extern
#endif // CarSub_GLOBALS
//------------------------------------------------------------------------------
//==============================================================================
//					SMART CAR DRIVER SUBROUTINE
//------------------------------------------------------------------------------
void CarInit(void);

//------------------------------------------------------------------------------
//					MOTOR SUBROUTINE
//------------------------------------------------------------------------------
void MotorInit(void);

//------------------------------------------------------------------------------
//  CAR DIRECTION DEFINITION:
//			Dir = 1 <--		|       --> Dir = 0
//			(front)			|		(back)
//				           *|
//                         *O
//
#define MOTOR_DIR				0				// 0: Move direction face the back
                                                // 1: Move direciton face the front
//void SetMotorVoltage(int nLeftVol, int nRightVol);    
                                                // Voltage: > 0 : Move face according to the MOTOR_DIR
                                                //          < 0 : Move face back according to the MOTOR DIR

#define MOTOR_STOP				SetMotorVoltage(0, 0)
#define MOTOR_SET(left, right)	SetMotorVoltage(left, right)

#define MOTOR_SET0(x)		PWMC1_SetRatio15(0, x)
#define MOTOR_SET1(x)		PWMC1_SetRatio15(1, x)
#define MOTOR_SET2(x)		PWMC1_SetRatio15(2, x)
#define MOTOR_SET3(x)		PWMC1_SetRatio15(3, x)
#define MOTOR_SETLOAD		PWMC1_Load();//???


//------------------------------------------------------------------------------
CARSUB_EXT int g_nLeftMotorOut, g_nRightMotorOut;
#define MOTOR_OUT_MAX			(int)0x7ff
#define MOTOR_OUT_MIN			(int)(-0x7ff)

#define MOTOR_OUT_DEAD_VAL		0xe0
void MotorSpeedOut(void);                       // Output the g_nLeftMotorOut, nRightMotorOut
                                                // : As the MOTOR_OUT_MAX, MIN 
                                                //   right shift bits to 15bits

//------------------------------------------------------------------------------
CARSUB_EXT int g_nLeftSpeedSet, g_nRightSpeedSet; // >= 0: Forward, < 0 : Backward
CARSUB_EXT int g_nMotorSpeedSet;
void SetMotorSpeed(int nLeft, int nRight);
#define MOTOR_SPEED_SET_MAX		1000
#define MOTOR_SPEED_SET_MIN		-1000


//------------------------------------------------------------------------------
#define CAR_MOTION_PERIOD		20			 	// unit : CAR_SPEED_PERIOD
#define MOTOR_SPEED_P			0.3          		// unit 1
#define MOTOR_SPEED_I			0.3				// unit 1
#define MOTOR_SPEED_P_INT		(int)(MOTOR_SPEED_P * 0x7fff)  	// unit 1/CAR_MOTION_PERIOD
#define MOTOR_SPEED_I_INT		(int)(MOTOR_SPEED_I * 0x7fff) 	// unit 1/CAR_MOTION_PERIOD 

#define MOTOR_LEFT_SPEED_POSITIVE		(g_nLeftMotorOut > 0)
#define MOTOR_RIGHT_SPEED_POSITIVE		(g_nRightMotorOut > 0)

void MotorSpeedAdjustCal(void);                 // Adjust Control Calculating subroutine
void MotorSpeedAdjust(void);
CARSUB_EXT int g_nLeftMotorOutKeep, g_nRightMotorOutKeep;
CARSUB_EXT int g_nLeftMotorOutSpeed, g_nRightMotorOutSpeed;
CARSUB_EXT int g_nLeftMotorSpeedCount, g_nRightMotorSpeedCount;
CARSUB_EXT int g_nLeftMotorSpeed, g_nRightMotorSpeed;
CARSUB_EXT int g_nMotorOutSpeedOld, g_nMotorOutSpeedNew;
CARSUB_EXT int g_nMotorOutSpeedKeep;

CARSUB_EXT int g_nMotorLeftRightDiff;
#define MOTOR_LEFTRIGHT_DIFF_MAX			40 //80
#define MOTOR_LEFTRIGHT_DIFF_MAX_16			(MOTOR_LEFTRIGHT_DIFF_MAX >> 4)

//------------------------------------------------------------------------------
//	CALCUATE MOTOR OUT SPEED:
//  Input : g_nMotorOutSpeedNew, g_nMotorOutSpeedOld
//        : g_nCarMotorCount
//  Output: g_nLeftMotorOutSpeed, g_nRightMotorOutSpeed;
//  Algrithm:
//          Left(Right)MotorOutSpeed = (SpeedNew - SpeedOld) * (Count + 1) /
//											CAR_MOTOION_PERIOD + SpeedOld
void CalculateMotorOutSpeed(void);

//==============================================================================
//				CAR ERECT ANGLE CALCULATING
//------------------------------------------------------------------------------
CARSUB_EXT unsigned int g_nCarVoltage[4];       // Y,Z,Gyro,Set
#define CV_MAGNET			0
#define CV_ACCE_Z			1
#define CV_GYRO				2
#define CV_SET				3
#define CV_MAGNET_DEF		0x7fff
#define CV_ACCE_Z_DEF		0x7fff
#define CV_GYRO_DEF			0x7fff
#define CV_SET_DEF			0x7fff

void CarVoltageGet(void);

#define CV_ACCE_VAL			(g_nCarVoltage[CV_ACCE_Z])
#define CV_ACCE_OFFSET_VAL	(g_nCarVoltage[CV_SET])
#define CV_GYRO_VAL			(g_nCarVoltage[CV_GYRO])
#define CV_MAGNET_VAL		g_nCarVoltage[CV_MAGNET]
#define CV_MAGNETLEFT_VAL	(g_nCarVoltage[3])
#define CV_MAGNETRIGHT_VAL  (g_nCarVoltage[0])

//------------------------------------------------------------------------------
//  The following macro definition is determined by reading the real ADC value
//  of corresponding channels.
//
#define CV_GYRO_ZERO		37920               // By Read the voltage when 
                                                // gyroscope is still
#define CV_ACCE_Z_MAX		47500               //
#define CV_ACCE_Z_ZERO		32768               // Read by experiment
#define CV_ACCE_ANGLE_MAX	1000
#define CV_ACCE_ANGLE_RATIO	(int)(CV_ACCE_ANGLE_MAX * 0x7fffL / 			\
                             (long)(CV_ACCE_Z_MAX - CV_ACCE_Z_ZERO))
#define CV_ACCE_OFFSET		38000

CARSUB_EXT int g_nCarAcceVal;                   // Normalize from g_nCarVoltage[1] to -90 ~+90 degree.
CARSUB_EXT int g_nCarGyroVal;                   // Shift of the CV_GYRO_ZERO form CV_GYRO


#define CAR_GYRO_RATIO		0.85
#define CAR_GYRO_RATIO_INT	(int)(CAR_GYRO_RATIO * 0x7fff)
CARSUB_EXT long g_lnCarAngleSigma;				
CARSUB_EXT int g_nCarAngle;
#define CAR_ACCE_RATIO		2

//==============================================================================
//						CAR ERECT ADJUST
//------------------------------------------------------------------------------
CARSUB_EXT int g_nCarAngleSet;					// -1000~1000

#define CAR_ANGLE_SET_MAX		1000
#define CAR_ANGLE_SET_MIN		-1000

void CarAngleAdjust(void);
#define CAR_AA_P				0.75				// (error) * P
#define CAR_AA_D				0.5				// (GyroVal >> 8) * D

#define CAR_AA_P_INT		(int)(CAR_AA_P * 0x7fff)
#define CAR_AA_D_INT		(int)(CAR_AA_D * 0x7fff / 4)

//==============================================================================
//						CAR RUNNING CONTROL
//------------------------------------------------------------------------------
CARSUB_EXT int g_nCarStopFlag;
#define CAR_STOP			g_nCarStopFlag = 1
#define CAR_START			g_nCarStopFlag = 0
#define CAR_STOP_FLAG		        g_nCarStopFlag
void TestCarErect(void);

//==============================================================================
//						CAR SPEED ADJUST
// Input : The car speed set:
// Feedback: Car speed = (Car Left Wheel Speed + Right Wheel Speed) / 2;
// Output: g_nCarAngleSet;
//
// Adjust Period: CAR_MOTION_PERIOND * CAR_SPEED_PERIOD
//                default : 20ms
//------------------------------------------------------------------------------

//	CAR MOTOR SPEED CONTROL
#define CAR_SPEED_PERIOD		5				// unit: 1ms

CARSUB_EXT unsigned int g_nCarSpeedCount;

void ClearMotorSpeed(void);
void GetMotorSpeed(unsigned int * pnLeft, unsigned int *pnRight);
//CARSUB_EXT unsigned int g_nLeftMotorSpeed, g_nRightMotorSpeed;
CARSUB_EXT unsigned int g_nLeftMotorDir, g_nRightMotorDir;
                                                // Speed : Wheel pulse number
                                                // every CAR_MOTION_PERIOD * CAR_SPEED_PERIOD

CARSUB_EXT int g_nCarMotorSpeedDif;             // Motor Left += Dif, Motor Right -= Dif

//------------------------------------------------------------------------------
void CarSpeedAdjust(void);
#define CSA_P				0.5
#define CSA_I				0.1
#define CSA_P_INT			(int)(CSA_P * 0x7fff / 4)
#define CSA_I_INT			(int)(CSA_I * 0x7fff)

                                                //         used for adjust car speed
CARSUB_EXT int g_nCarMotionCount;

CARSUB_EXT int g_nCarSpeedSet, g_nCarAngleSetKeep;
												// Keep : Unit:1/64

#define CAR_ANGLE_MAX		30000
#define CAR_ANGLE_MIN		(-CAR_ANGLE_MAX)

//------------------------------------------------------------------------------
#define CPA_P				0.05
#define CPA_I				0.1
#define CPA_P_INT			(int)(CPA_P * 0x7fff)
#define CPA_I_INT			(int)(CPA_I * 0x7fff)

CARSUB_EXT long g_lnCarLeftPosition, g_lnCarRightPosition;
void CarPositionAdjust(void);
CARSUB_EXT long g_lnCarPositionSet;

CARSUB_EXT int g_nCarPositionAdjustPeriod;
#define CPAP_PERIOD			20					// unit : CAR_SPEED_PERIOD * CAR_MOTION_PERIOD
                                                //  5ms

//------------------------------------------------------------------------------
//				CAR DIRECTION ADJUST
//
// INPUT	: Left Right Position Diff;
// FEEDBACK	: LEFT Position and Right Position;
//       
// OUTPUT	: g_nMotorLeftRightDiff
//         	  Output limited by MOTOR_LEFTRIGHT_DIFF_MAX
//------------------------------------------------------------------------------
void CarDirectionAdjust(void);
CARSUB_EXT long g_lnLeftRightPositionDiff;
CARSUB_EXT long g_lnLeftRightPositionDiffOld;
#define CDA_P				0.02                 	// unit: 1
#define CDA_I				0.02					// unit: 1/16
#define CDA_D				0.02
#define CDA_P_INT			(int)(CDA_P * 0x7fff)
#define CDA_I_INT			(int)(CDA_I * 0x7fff)
#define CDA_D_INT			(int)(CDA_D * 0x7fff)
CARSUB_EXT int g_nMotorLeftRightDiffKeep;
CARSUB_EXT int g_nMotorLeftRightDiffNew, g_nMotorLeftRightDiffOld;

void CalculateMotorLeftRightDiff(void);

//------------------------------------------------------------------------------

//==============================================================================
//				CAR MAGNETIC ADJUST
// INPUT	:  CV_MAGNET
//             CMA_OFFSET_SET : 0x3fff
// OUTPUT   : g_lnLeftRightPositionDiff
////            g_nMotorLeftRightDiff
//         	  Output limited by MOTOR_LEFTRIGHT_DIFF_MAX
//
//------------------------------------------------------------------------------
void CarMagneticAdjust(void);
#define CMA_OFFSET_SET		(32400 / 2)
#define CMA_P				0.02			////0.15					// unit 1/256
#define CMA_I				0.01			////0.00				// unit 1/256
#define CMA_D				//0.02
#define CMA_P_INT			(int)(CMA_P * 0x7fff)
#define CMA_I_INT			(int)(CMA_I * 0x7fff)
#define CMA_D_INT			(int)(CMA_D * 0x7fff)
CARSUB_EXT long g_lnCarMagneticLeftCount, g_lnCarMagneticRightCount;
CARSUB_EXT long g_lnLeftRightPositionDiffKeep;
CARSUB_EXT int g_nCarMagneticLeftAverage, g_nCarMagneticRightAverage;
#define CM_RIGHT_OFFSET		0
#define CM_LEFT_OFFSET		200

#define CMA_P_MAX			6                  // unit : 1
#define CMA_I_MAX			0				    // unit : 1/16

CARSUB_EXT int g_nCarMagneticAdjustFlag;
#define CMA_START			g_nCarMagneticAdjustFlag = 1
#define CMA_STOP			g_nCarMagneticAdjustFlag = 0
#define IF_CMA_START		(g_nCarMagneticAdjustFlag == 1)
#define IF_CMA_STOP			(g_nCarMagneticAdjustFlag == 0)



//==============================================================================
//             END OF THE FILE : CarSub.H
//------------------------------------------------------------------------------
#endif // __CarSub__
