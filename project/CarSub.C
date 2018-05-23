/*
**==============================================================================
** CarSub.C:             -- by Dr. ZhuoQing, 2011-12-3
**
**==============================================================================
*/

#define CarSub_GLOBALS        1                       // Define the global variables
#include "CarSub.H"
//------------------------------------------------------------------------------
#include "led.h"
#include "key.h"
#include "delay.h"
#include "pit.h"
#include "pdb.h"
#include "adc0.h"
#include "adc1.h"
#include "usart.h"
#include "pwm.h"
//#include "i2c_ee.h"
#include "oled.h"
#include "mpu6050.h"
#include "angle.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h" 
//#include "speed.h"
#include "speed_quad.h"
#include "ccd0.h"
#include "ccd1.h"
#include "nrf24l01.h"
#include "ov7725.h"
#include "hc_sr04.h"
//#include "ov7725.h"
#include "imu.h"


//==============================================================================
//				CAR DRIVER SUBROUTINE
//------------------------------------------------------------------------------
void CarInit(void) {
	MotorInit();
	
	g_nCarSpeedCount = 0;
	ClearMotorSpeed();                          // Clear the motor speed counter1, 2
	g_nLeftMotorSpeed = 0;                      // The value is get at 1ms interrupt.
	g_nRightMotorSpeed = 0;                     // The value is get at 1ms interrupt.
	g_nLeftMotorDir = 0;
	g_nRightMotorDir = 0;
	
	g_nLeftSpeedSet = 0;
	g_nRightSpeedSet = 0;
	g_nMotorSpeedSet = 0;
	g_nLeftMotorOutKeep = 0;
	g_nRightMotorOutKeep = 0;
	g_nLeftMotorOutSpeed = 0;
	g_nRightMotorOutSpeed = 0;
	g_nLeftMotorSpeedCount = 0;
	g_nRightMotorSpeedCount = 0;
	g_nMotorOutSpeedOld = 0;
	g_nMotorOutSpeedNew = 0;
	g_nMotorOutSpeedKeep = 0;
	
	g_nCarVoltage[CV_MAGNET] = CV_MAGNET_DEF;   // Initialize all the car voltage
	g_nCarVoltage[CV_ACCE_Z] = CV_ACCE_Z_DEF;
	g_nCarVoltage[CV_GYRO] = CV_GYRO_DEF;
	g_nCarVoltage[CV_SET] = CV_SET_DEF;
	
	g_nCarAcceVal = 0;
	g_nCarGyroVal = 0;
	g_lnCarAngleSigma = 0;

	g_nCarAngleSet = 0;                         // Car angle set
	g_nCarMotorSpeedDif = 0;	
	
	CAR_STOP;
	g_nCarSpeedSet = 0;                         // Car speed adjust setting
	g_nCarAngleSetKeep = 0;
	g_nCarMotionCount = 0;
	
	g_lnCarLeftPosition = 0;
	g_lnCarRightPosition = 0;
	g_lnCarPositionSet = 0;
	g_nCarPositionAdjustPeriod = 0;
	
	//--------------------------------------------------------------------------
	g_nMotorLeftRightDiff = 0;
	g_lnLeftRightPositionDiff = 0;
	g_nMotorLeftRightDiffKeep = 0;

	g_nMotorLeftRightDiffNew = 0;
	g_nMotorLeftRightDiffOld = 0;
	g_lnLeftRightPositionDiffOld = 0;
	
	//--------------------------------------------------------------------------
	g_lnCarMagneticLeftCount = 0;
	g_lnCarMagneticRightCount = 0;
	g_nCarMagneticLeftAverage = 0;
	g_nCarMagneticRightAverage = 0;
	g_lnLeftRightPositionDiffKeep = 0;
	CMA_STOP;
}

//==============================================================================
//				MOTOR DRIVER SUBROUTINE
//------------------------------------------------------------------------------
void MotorInit(void) {
	MOTOR_STOP;
}
	
//------------------------------------------------------------------------------
//  CAR DIRECTION DEFINITION:
//			Dir = 1 <--		|       --> Dir = 0
//			(front)			|		(back)
//				           *|
//                         *O
//
//void SetMotorVoltage(int nLeftVol, int nRightVol) {
//                                                // Voltage: > 0 : Move face according to the MOTOR_DIR
//                                                //          < 0 : Move face back according to the MOTOR DIR
//
//	short nPeriod;
//	nPeriod = (short)getReg(PWM_PWMCM);
//#if MOTOR_DIR == 1
//	if(nLeftVol > 0) {
//      	setReg(PWM_PWMVAL0,	0);        
//      	nLeftVol = mult(nLeftVol, nPeriod);
//      	setReg(PWM_PWMVAL1, nLeftVol);
//	} else {
//		nLeftVol = -nLeftVol;      
//      	setReg(PWM_PWMVAL1,	0);        
//      	nLeftVol = mult(nLeftVol, nPeriod);
//      	setReg(PWM_PWMVAL0, nLeftVol);
//    }
//    if(nRightVol > 0) {
//      	setReg(PWM_PWMVAL3,	0);        
//      	nRightVol = mult(nRightVol, nPeriod);
//      	setReg(PWM_PWMVAL2, nRightVol);
//    } else {
//      	setReg(PWM_PWMVAL2,	0);        
//      	nRightVol = -nRightVol;
//      	nRightVol = mult(nRightVol, nPeriod);
//      	setReg(PWM_PWMVAL3, nRightVol);
//    }
//#else 
//	if(nLeftVol > 0) {
//      	setReg(PWM_PWMVAL2,	0);        
//      	nLeftVol = mult(nLeftVol, nPeriod);
//      	setReg(PWM_PWMVAL3, nLeftVol);
//	} else {
//		nLeftVol = -nLeftVol;      
//      	setReg(PWM_PWMVAL3,	0);        
//      	nLeftVol = mult(nLeftVol, nPeriod);
//      	setReg(PWM_PWMVAL2, nLeftVol);
//    }
//    if(nRightVol > 0) {
//      	setReg(PWM_PWMVAL1,	0);        
//      	nRightVol = mult(nRightVol, nPeriod);
//      	setReg(PWM_PWMVAL0, nRightVol);
//    } else {
//      	setReg(PWM_PWMVAL0,	0);        
//      	nRightVol = -nRightVol;
//      	nRightVol = mult(nRightVol, nPeriod);
//      	setReg(PWM_PWMVAL1, nRightVol);
//    }
//#endif // MOTOR_DIR == 0 ? 1
//		
///*
//#if MOTOR_DIR == 1
//	if(nLeftVol > 0) {                          // Attension:
//		MOTOR_SET0(0);                          // Left and right motor are mirror installation. So their
//		MOTOR_SET1(nLeftVol);                   // driver voltage should be inverted !
//	} else {
//		nLeftVol = -nLeftVol;
//		MOTOR_SET1(0);
//		MOTOR_SET0(nLeftVol);
//	}
//	if(nRightVol > 0) {
//		MOTOR_SET3(0);
//		MOTOR_SET2(nRightVol);
//	} else {
//		nRightVol = -nRightVol;
//		MOTOR_SET2(0);
//		MOTOR_SET3(nRightVol);
//	}
//#else 	// MOTOR_DIR == 0
//	if(nLeftVol > 0) {
//		MOTOR_SET2(0);
//		MOTOR_SET3(nLeftVol);
//	} else {
//		nLeftVol = -nLeftVol;
//		MOTOR_SET3(0);
//		MOTOR_SET2(nLeftVol);
//	}
//	if(nRightVol > 0) {
//		MOTOR_SET1(0);
//		MOTOR_SET0(nRightVol);
//	} else {
//		nRightVol = -nRightVol;
//		MOTOR_SET0(0);
//		MOTOR_SET1(nRightVol);
//	}
//#endif 	// MOTOR_DIR
//*/
//		MOTOR_SETLOAD;
//}

//------------------------------------------------------------------------------
//	CAR MOTOR SPEED CONTROL
void ClearMotorSpeed(void) {
	COUNTER1_Reset();
	COUNTER2_Reset();
}
void GetMotorSpeed(unsigned int * pnLeft, unsigned int *pnRight) {
	COUNTER1_GetNumEvents(pnLeft);
	COUNTER2_GetNumEvents(pnRight);
}

//------------------------------------------------------------------------------
void SetMotorSpeed(int nLeft, int nRight) {
	g_nLeftSpeedSet = nLeft;
	g_nRightSpeedSet = nRight;
}
	
//------------------------------------------------------------------------------
//	CAR MOTOR OUTPUT
//------------------------------------------------------------------------------
void MotorSpeedOut(void) {
	int nLeftVal, nRightVal;
	
	//--------------------------------------------------------------------------
	nLeftVal = g_nLeftMotorOut;
	nRightVal = g_nRightMotorOut;
	if(nLeftVal > 0) 		nLeftVal += MOTOR_OUT_DEAD_VAL;
	else if(nLeftVal < 0) 	nLeftVal -= MOTOR_OUT_DEAD_VAL;
	if(nRightVal > 0) 		nRightVal += MOTOR_OUT_DEAD_VAL;
	else if(nRightVal < 0) 	nRightVal -= MOTOR_OUT_DEAD_VAL;
		
	if(nLeftVal > MOTOR_OUT_MAX)	nLeftVal = MOTOR_OUT_MAX;
	if(nLeftVal < MOTOR_OUT_MIN)	nLeftVal = MOTOR_OUT_MIN;
	if(nRightVal > MOTOR_OUT_MAX)	nRightVal = MOTOR_OUT_MAX;
	if(nRightVal < MOTOR_OUT_MIN)	nRightVal = MOTOR_OUT_MIN;

	nLeftVal = nLeftVal << 4;
	nRightVal = nRightVal << 4;
		
/*	nLeftVal = g_nLeftMotorOut << 4;
	nRightVal = g_nRightMotorOut << 4;
	
	if(nLeftVal > 0) {
		if(nLeftVal < MOTOR_OUT_DEAD_VAL) 
			nLeftVal = MOTOR_OUT_DEAD_VAL;
	} else if(nLeftVal < 0) {
		if(nLeftVal > - MOTOR_OUT_DEAD_VAL)
			nLeftVal = -MOTOR_OUT_DEAD_VAL;
	}

	if(nRightVal > 0) {
		if(nRightVal < MOTOR_OUT_DEAD_VAL) 
			nRightVal = MOTOR_OUT_DEAD_VAL;
	} else if(nRightVal < 0) {
		if(nRightVal > - MOTOR_OUT_DEAD_VAL)
			nRightVal = -MOTOR_OUT_DEAD_VAL;
	}
*/
	//--------------------------------------------------------------------------
	MOTOR_SET(nLeftVal, nRightVal);
//	MOTOR_SET(0x1000, 0x1000);
}

//------------------------------------------------------------------------------
//	MOTOR SPEED ADJUST SUBROUINT
//  Function is called in the events:1 millisecond / 
void MotorSpeedAdjust(void) {
	MotorSpeedAdjustCal();		                // 4.2us
//	MotorSpeedOut();
}

//------------------------------------------------------------------------------
//  PID Adjust Motor Wheel Speed:
//  
void MotorSpeedAdjustCal(void) {
	int nLeftSpeed, nRightSpeed;
	int nDeltaValue, nP, nI;
	int nSpeed;
	
	nLeftSpeed = (int)g_nLeftMotorSpeedCount;
	nRightSpeed = (int)g_nRightMotorSpeedCount;
	nSpeed = (nLeftSpeed + nRightSpeed) / 2;
	
	nDeltaValue = g_nMotorSpeedSet - nSpeed;
	nP = mult_r(nDeltaValue, MOTOR_SPEED_P_INT);
	nI = mult_r(nDeltaValue, MOTOR_SPEED_I_INT);
//	nP = nDeltaValue * MOTOR_SPEED_P;
//	nI = nDeltaValue * MOTOR_SPEED_I;

	g_nMotorOutSpeedOld = g_nMotorOutSpeedNew;
	
	g_nMotorOutSpeedKeep -= nI;
	g_nMotorOutSpeedNew = (g_nMotorOutSpeedKeep >> 3) - nP;
	if(g_nMotorOutSpeedKeep > MOTOR_OUT_MAX) g_nMotorOutSpeedKeep = MOTOR_OUT_MAX;
	if(g_nMotorOutSpeedKeep < MOTOR_OUT_MIN) g_nMotorOutSpeedKeep = MOTOR_OUT_MIN;		
	
	g_nLeftMotorOutKeep = g_nRightMotorOutKeep = g_nMotorOutSpeedKeep;

	//--------------------------------------------------------------------------
/*	nDeltaValue = g_nLeftSpeedSet - nLeftSpeed;
//	nDeltaValue <<= 2;
//	nP = mult_r(nDeltaValue, MOTOR_SPEED_P_INT);
//	nI = mult_r(nDeltaValue, MOTOR_SPEED_I_INT);
	nP = nDeltaValue * MOTOR_SPEED_P;
	nI = nDeltaValue * MOTOR_SPEED_I;
	
	g_nLeftMotorOutKeep -= nI;
	g_nLeftMotorOutSpeed = (g_nLeftMotorOutKeep >> 3) - nP;
	if(g_nLeftMotorOut > MOTOR_OUT_MAX)	g_nLeftMotorOut = MOTOR_OUT_MAX;
	if(g_nLeftMotorOut < MOTOR_OUT_MIN) g_nLeftMotorOut = MOTOR_OUT_MIN;
	if(g_nLeftMotorOutKeep > MOTOR_OUT_MAX)	g_nLeftMotorOutKeep = MOTOR_OUT_MAX;
	if(g_nLeftMotorOutKeep < MOTOR_OUT_MIN) g_nLeftMotorOutKeep = MOTOR_OUT_MIN;
		
	nDeltaValue = g_nRightSpeedSet - nRightSpeed;
//	nDeltaValue <<= 2;
//	nP = mult_r(nDeltaValue, MOTOR_SPEED_P_INT);
//	nI = mult_r(nDeltaValue, MOTOR_SPEED_I_INT);
	nP = nDeltaValue * MOTOR_SPEED_P;
	nI = nDeltaValue * MOTOR_SPEED_I;
	
	g_nRightMotorOutKeep -= nI;
	g_nRightMotorOutSpeed = (g_nRightMotorOutKeep >> 3) - nP;
	if(g_nRightMotorOut > MOTOR_OUT_MAX) g_nRightMotorOut = MOTOR_OUT_MAX;
	if(g_nRightMotorOut < MOTOR_OUT_MIN) g_nRightMotorOut = MOTOR_OUT_MIN;		
	if(g_nRightMotorOutKeep > MOTOR_OUT_MAX) g_nRightMotorOutKeep = MOTOR_OUT_MAX;
	if(g_nRightMotorOutKeep < MOTOR_OUT_MIN) g_nRightMotorOutKeep = MOTOR_OUT_MIN;		
*/	
//	g_nRightMotorOutSpeed = g_nLeftMotorOutSpeed;
}


//==============================================================================
//		CAR POSTURE ANGLE ESTIMATE 
//------------------------------------------------------------------------------
void CarVoltageGet(void) {
	long lnDeltaValue;
	//--------------------------------------------------------------------------
	ADC_GetValue16(g_nCarVoltage);
	
	lnDeltaValue = (int)CV_ACCE_VAL;
	lnDeltaValue = lnDeltaValue - (int)CV_ACCE_OFFSET;
	g_nCarAcceVal = (int)lnDeltaValue;
//	g_nCarAcceVal = (int)CV_ACCE_VAL;
//	g_nCarAcceVal = g_nCarAcceVal - (int)CV_ACCE_OFFSET;//(int)CV_ACCE_OFFSET_VAL;
	
	
	g_nCarAcceVal = mult_r(g_nCarAcceVal, CV_ACCE_ANGLE_RATIO);	
	
	g_nCarGyroVal = (int)CV_GYRO_VAL;
	g_nCarGyroVal = (int)(g_nCarGyroVal - CV_GYRO_ZERO);
	g_nCarGyroVal = mult_r(g_nCarGyroVal, CAR_GYRO_RATIO_INT);

	//--------------------------------------------------------------------------
	g_nCarAngle = (int)(g_lnCarAngleSigma >> 10); //..... 
	lnDeltaValue = g_nCarAcceVal - g_nCarAngle;
	lnDeltaValue = lnDeltaValue * CAR_ACCE_RATIO;
	
	g_lnCarAngleSigma += (g_nCarGyroVal + lnDeltaValue);	
}

//==============================================================================
//				CAR ANGLE ADJUST
//------------------------------------------------------------------------------
#define MAX_ANGLE					500
void CarAngleAdjust(void) {
	int nLeft, nRight;
	int nSpeed;
	int nP, nD;

	nP = g_nCarAngleSet - g_nCarAngle;
	
	//--------------------------------------------------------------------------
	if(nP > MAX_ANGLE || nP < -MAX_ANGLE || g_nCarStopFlag) {
		SetMotorSpeed(0, 0);
		g_nLeftMotorOut = 0;
		g_nRightMotorOut = 0;
		g_nLeftMotorOutKeep = 0;
		g_nRightMotorOutKeep = 0;
		MotorSpeedOut();
		g_nCarStopFlag = 1;
		return;
	}
	
	//--------------------------------------------------------------------------
	nP = (int)mult_r(nP, CAR_AA_P_INT);
	nD = g_nCarGyroVal >> 5;
	nD = (int)mult_r(nD, CAR_AA_D_INT);

	nSpeed = nD - nP;                           //  ***** (nP)

	if(nSpeed > MOTOR_SPEED_SET_MAX) 		nSpeed = MOTOR_SPEED_SET_MAX;
	else if(nSpeed < MOTOR_SPEED_SET_MIN)	nSpeed = MOTOR_SPEED_SET_MIN;
	
	nLeft = nSpeed + g_nLeftMotorOutSpeed - g_nMotorLeftRightDiff;
	nRight = nSpeed + g_nRightMotorOutSpeed + g_nMotorLeftRightDiff;

	g_nLeftMotorOut = nLeft << 6;
	g_nRightMotorOut = nRight << 6;
	if(g_nLeftMotorOut > MOTOR_OUT_MAX)	g_nLeftMotorOut = MOTOR_OUT_MAX;
	if(g_nLeftMotorOut < MOTOR_OUT_MIN) g_nLeftMotorOut = MOTOR_OUT_MIN;
	if(g_nRightMotorOut > MOTOR_OUT_MAX) g_nRightMotorOut = MOTOR_OUT_MAX;
	if(g_nRightMotorOut < MOTOR_OUT_MIN) g_nRightMotorOut = MOTOR_OUT_MIN;		
	MotorSpeedOut();

}

//------------------------------------------------------------------------------
//			TEST CAR ERECT
#define ERECT_TIME			4
void TestCarErect(void) {
	int nCount = 0;
	int nLEDFlag = 0;
	int nAccAverage = 0;
	unsigned int nADCValue[6];
	unsigned int nValue;
	
	for(;;) {
		nLEDFlag = nLEDFlag == 0 ? 1 : 0;
		if(nLEDFlag) LED_ON;
		else LED_OFF;
			
		//----------------------------------------------------------------------
		if(g_nCarAcceVal >= -100 &&
		   g_nCarAcceVal <= 100) {
		   	nCount ++;
		   	nAccAverage += g_nCarAcceVal;
		   	WaitTime(200);
		}
		else {
			nCount = 0;
			nAccAverage = 0;
			WaitTime(20);
		}
			
		if(nCount >= ERECT_TIME) {
			break;
		}
		nADCValue[0] = (unsigned int)(g_nCarAcceVal + 20000);
		nADCValue[1] = (unsigned int)(g_nCarGyroVal + 30000);
		nADCValue[2] = (unsigned int)(g_nCarAngle);
		nADCValue[3] = CV_MAGNETLEFT_VAL;//(unsigned int)(g_nCarMagneticLeftAverage);
		nADCValue[4] = CV_MAGNETRIGHT_VAL;//(unsigned int)(g_nCarMagneticRightAverage);
		nValue = 0;
		if(RC1_GetVal()) nValue |= 0x1;
		if(RC2_GetVal()) nValue |= 0x2;
		if(RC3_GetVal()) nValue |= 0x4;
		if(RC4_GetVal()) nValue |= 0x8;
		nADCValue[5] = (unsigned int)nValue;
		
  		//----------------------------------------------------------------------
  		SendChar(0x55);
  		SendWord(nADCValue[0]);
  		SendWord(nADCValue[1]);
  		SendWord(nADCValue[2]);
  		SendWord(nADCValue[3]);
  		SendWord(nADCValue[4]);
  		SendWord(nADCValue[5]);
	}
	//--------------------------------------------------------------------------
//	g_lnCarAngleSigma = nAccAverage / ERECT_TIME;
//	g_lnCarAngleSigma = g_lnCarAngleSigma << 10;
	g_lnCarAngleSigma = 0;

	//--------------------------------------------------------------------------
	return;
}

//==============================================================================
//					CAR SPEED ADJUST
//------------------------------------------------------------------------------
void CarSpeedAdjust(void) {
/*	int nSpeed, nLeftSpeed, nRightSpeed;
	int nDelta;
	int nP, nI;
	
	if(CAR_STOP_FLAG == 1) {
		g_nCarAngleSet = 0;
		g_nCarAngleSetKeep = 0;
		return;
  	}

	nLeftSpeed = (int)g_nLeftMotorSpeed;
	nRightSpeed = (int)g_nRightMotorSpeed;
	if(!MOTOR_LEFT_SPEED_POSITIVE)		
		nLeftSpeed = -nLeftSpeed;
	if(!MOTOR_RIGHT_SPEED_POSITIVE)
		nRightSpeed = -nRightSpeed;

	nSpeed = (nLeftSpeed + nRightSpeed) / 2;
	nDelta = g_nCarSpeedSet - nSpeed;
	nP = mult_r((nDelta << 2), CSA_P_INT);
	nI = mult_r(nDelta, CSA_I_INT);
	g_nCarAngleSetKeep += nI;
	g_nCarAngleSet = (g_nCarAngleSetKeep >> 6) + nP;
	
	if(g_nCarAngleSetKeep > CAR_ANGLE_MAX) g_nCarAngleSetKeep = CAR_ANGLE_MAX;
	if(g_nCarAngleSetKeep < CAR_ANGLE_MIN) g_nCarAngleSetKeep = CAR_ANGLE_MIN;
	if(g_nCarAngleSet > CAR_ANGLE_MAX) g_nCarAngleSet = CAR_ANGLE_MAX;
	if(g_nCarAngleSet < CAR_ANGLE_MIN) g_nCarAngleSet = CAR_ANGLE_MIN;
*/
}

//------------------------------------------------------------------------------
void CarPositionAdjust(void) {
/*
	int nLeftSpeed, nRightSpeed;
	int nP, nI, nDelta;
	long lnPosition, lnDelta;
	
	if(CAR_STOP_FLAG == 1) {
		g_nCarAngleSet = 0;
		g_nCarAngleSetKeep = 0;
		g_lnCarLeftPosition = 0;
		g_lnCarRightPosition = 0;
		g_lnCarPositionSet = 0;
		g_nCarPositionAdjustPeriod = 0;
		return;
  	}

	nLeftSpeed = (int)g_nLeftMotorSpeed;
	nRightSpeed = (int)g_nRightMotorSpeed;
	if(!MOTOR_LEFT_SPEED_POSITIVE)		
		nLeftSpeed = -nLeftSpeed;
	if(!MOTOR_RIGHT_SPEED_POSITIVE)
		nRightSpeed = -nRightSpeed;
	
	g_lnCarLeftPosition += nLeftSpeed;
	g_lnCarRightPosition += nRightSpeed;
	lnPosition = (g_lnCarLeftPosition + g_lnCarRightPosition) / 2;
	lnDelta = g_lnCarPositionSet - lnPosition;

	nDelta = (int)lnDelta;
	
	nP = mult_r(nDelta, CPA_P_INT);
	nI = mult_r(nDelta, CPA_I_INT);
	
	g_nCarPositionAdjustPeriod ++;
	if(g_nCarPositionAdjustPeriod >= CPAP_PERIOD) {
		g_nCarPositionAdjustPeriod = 0;
		g_nCarAngleSetKeep += nI;
		g_nCarAngleSet = (g_nCarAngleSetKeep >> 6) + nP;
	}
	
	if(g_nCarAngleSetKeep > CAR_ANGLE_MAX) g_nCarAngleSetKeep = CAR_ANGLE_MAX;
	if(g_nCarAngleSetKeep < CAR_ANGLE_MIN) g_nCarAngleSetKeep = CAR_ANGLE_MIN;
	if(g_nCarAngleSet > CAR_ANGLE_MAX) g_nCarAngleSet = CAR_ANGLE_MAX;
	if(g_nCarAngleSet < CAR_ANGLE_MIN) g_nCarAngleSet = CAR_ANGLE_MIN;
*/
}

//------------------------------------------------------------------------------
//	CALCUATE MOTOR OUT SPEED:
//  Input : g_nMotorOutSpeedNew, g_nMotorOutSpeedOld
//        : g_nCarMotorCount
//  Output: g_nLeftMotorOutSpeed, g_nRightMotorOutSpeed;
//  Algrithm:
//          Left(Right)MotorOutSpeed = (SpeedNew - SpeedOld) * (Count + 1) /
//								   	   (CAR_MOTOION_PERIOD - 1)+ SpeedOld
//                                      
void CalculateMotorOutSpeed(void) {
	int nValue;
	nValue = g_nMotorOutSpeedNew - g_nMotorOutSpeedOld;
	nValue = nValue * (g_nCarMotionCount + 1) / (CAR_MOTION_PERIOD - 1) + g_nMotorOutSpeedOld;
	g_nLeftMotorOutSpeed = g_nRightMotorOutSpeed = nValue;
}

//------------------------------------------------------------------------------
// 		CAR DIRECTION ADJUST
//
void CarDirectionAdjust(void) {
/*	long lnValue;
	int nValue, nP, nI, nD;
	
	lnValue = g_lnCarRightPosition - g_lnCarLeftPosition;
	lnValue = g_lnLeftRightPositionDiff - lnValue;
	
//	if(lnValue > 0x7fff) lnValue = 0x7fff;
//	else if(lnValue < -0x8000) lnValue = -0x8000;
	
	nValue = (int)lnValue;
	nP = mult_r(nValue, CDA_P_INT);
	nI = mult_r(nValue, CDA_I_INT);
	g_nMotorLeftRightDiffKeep += nI;
	
	nValue = (int)(lnValue - g_lnLeftRightPositionDiffOld);
	g_lnLeftRightPositionDiffOld = lnValue;
	nD = mult_r(nValue, CDA_D_INT);
	
	if(g_nMotorLeftRightDiffKeep >= MOTOR_LEFTRIGHT_DIFF_MAX_16) 
		g_nMotorLeftRightDiffKeep = MOTOR_LEFTRIGHT_DIFF_MAX_16;
	else if(g_nMotorLeftRightDiffKeep <= -MOTOR_LEFTRIGHT_DIFF_MAX_16)
		g_nMotorLeftRightDiffKeep = -MOTOR_LEFTRIGHT_DIFF_MAX_16;
	
	g_nMotorLeftRightDiffOld = g_nMotorLeftRightDiffNew;
	g_nMotorLeftRightDiffNew = (g_nMotorLeftRightDiffKeep >> 4) + nP + nD;

	if(g_nMotorLeftRightDiffNew >= MOTOR_LEFTRIGHT_DIFF_MAX) 
		g_nMotorLeftRightDiffNew = MOTOR_LEFTRIGHT_DIFF_MAX;
	else if(g_nMotorLeftRightDiffNew <= -MOTOR_LEFTRIGHT_DIFF_MAX)
		g_nMotorLeftRightDiffNew = -MOTOR_LEFTRIGHT_DIFF_MAX;
*/
}

//------------------------------------------------------------------------------
//	CALCUATE MOTOR OUT DIFF
//  Input : g_nMotorLeftRightDiffNew, g_nMotorLeftRightDiffOld;
//  Output: g_nMotorLeftRightOutDiff
//  Algrithm:
//          g_nMotorLeeftRightOutDiff = (DiffNew - DiffOld) * (Count + 1) /
//								   	   (CAR_MOTOION_PERIOD - 1)+ DiffOld
//                                      
void CalculateMotorLeftRightDiff(void) {
	int nValue;
	if(CAR_STOP_FLAG == 1 || IF_CMA_STOP) {
//		return;
	}
	
	nValue = g_nMotorLeftRightDiffNew - g_nMotorLeftRightDiffOld;
	nValue = nValue * (g_nCarMotionCount + 1) / (CAR_MOTION_PERIOD - 1) + g_nMotorLeftRightDiffOld;
	g_nMotorLeftRightDiff = nValue;
}

//==============================================================================
//				CALCULATE MAGNETIC 
//------------------------------------------------------------------------------
void CarMagneticAdjust(void) {
/*	int nDelta, nP, nI;

	if(CAR_STOP_FLAG == 1 || IF_CMA_STOP) {
//		g_lnLeftRightPositionDiffKeep = 0;
//		g_lnLeftRightPositionDiff = 0;
		return;
  	}

	nDelta = g_nCarMagneticLeftAverage - g_nCarMagneticRightAverage;
	nDelta = -nDelta;
	
	nP = mult_r(nDelta, CMA_P_INT);
	nI = mult_r(nDelta, CMA_I_INT);
	g_lnLeftRightPositionDiffKeep += nI;
	g_lnLeftRightPositionDiff = g_lnLeftRightPositionDiffKeep + nP;
*/

/*	int nDelta, nP, nI, nDeltaD, nD;

	if(CAR_STOP_FLAG == 1 || IF_CMA_STOP) {
		
//		g_lnLeftRightPositionDiffKeep = 0;
//		g_lnLeftRightPositionDiff = 0;
		return;
  	}


	nDelta = (int)(g_nCarMagneticAverage >> 1);
	if(g_nCarMagneticAverageOld == 0)
		g_nCarMagneticAverageOld = nDelta;
	nDeltaD = nDelta - g_nCarMagneticAverageOld;
	g_nCarMagneticAverageOld = nDelta;
	nDeltaD >>= 6;
	
	nDelta -= CMA_OFFSET_SET;
	nDelta >>= 8;
	nDelta = -nDelta;
	
	nP = mult_r(nDelta, CMA_P_INT);
	nI = mult_r(nDelta, CMA_I_INT);
	nD = mult_r(nDeltaD, CMA_D_INT);
	g_nMotorLeftRightDiffKeep += nI;
		
	if(g_nMotorLeftRightDiffKeep >= MOTOR_LEFTRIGHT_DIFF_MAX_16) 
		g_nMotorLeftRightDiffKeep = MOTOR_LEFTRIGHT_DIFF_MAX_16;
	else if(g_nMotorLeftRightDiffKeep <= -MOTOR_LEFTRIGHT_DIFF_MAX_16)
		g_nMotorLeftRightDiffKeep = -MOTOR_LEFTRIGHT_DIFF_MAX_16;
		
	g_nMotorLeftRightDiffOld = g_nMotorLeftRightDiffNew;
	g_nMotorLeftRightDiffNew = (g_nMotorLeftRightDiffKeep / 16) + nP + nD;

	if(g_nMotorLeftRightDiffNew >= MOTOR_LEFTRIGHT_DIFF_MAX) 
		g_nMotorLeftRightDiffNew = MOTOR_LEFTRIGHT_DIFF_MAX;
	else if(g_nMotorLeftRightDiffNew <= -MOTOR_LEFTRIGHT_DIFF_MAX)
		g_nMotorLeftRightDiffNew = -MOTOR_LEFTRIGHT_DIFF_MAX;
	
*/
	int nP, nI;
	long lnDelta;
	int nSigma;

	if(CAR_STOP_FLAG == 1 || IF_CMA_STOP) {
		return;
  	}

	lnDelta = g_nCarMagneticRightAverage - g_nCarMagneticLeftAverage;
	nSigma = (g_nCarMagneticLeftAverage / 2) + (g_nCarMagneticRightAverage / 2);
	if(nSigma == 0) return;
	
	nP = (int)(lnDelta * CMA_P_MAX / nSigma) / 2;
	nI = (int)(lnDelta * CMA_I_MAX / nSigma);
	
	g_nMotorLeftRightDiffKeep += nI;
		
	if(g_nMotorLeftRightDiffKeep >= MOTOR_LEFTRIGHT_DIFF_MAX_16) 
		g_nMotorLeftRightDiffKeep = MOTOR_LEFTRIGHT_DIFF_MAX_16;
	else if(g_nMotorLeftRightDiffKeep <= -MOTOR_LEFTRIGHT_DIFF_MAX_16)
		g_nMotorLeftRightDiffKeep = -MOTOR_LEFTRIGHT_DIFF_MAX_16;
		
	g_nMotorLeftRightDiffOld = g_nMotorLeftRightDiffNew;
	g_nMotorLeftRightDiffNew = (g_nMotorLeftRightDiffKeep / 16) + nP;

	if(g_nMotorLeftRightDiffNew >= MOTOR_LEFTRIGHT_DIFF_MAX) 
		g_nMotorLeftRightDiffNew = MOTOR_LEFTRIGHT_DIFF_MAX;
	else if(g_nMotorLeftRightDiffNew <= -MOTOR_LEFTRIGHT_DIFF_MAX)
		g_nMotorLeftRightDiffNew = -MOTOR_LEFTRIGHT_DIFF_MAX;
	

	//--------------------------------------------------------------------------
/*
#define DIFF_MAX			3
#define DIFF_SPEED_DEC		10
	if(g_nMotorLeftRightDiff > DIFF_MAX ||
	   g_nMotorLeftRightDiff < -DIFF_MAX) {
			if(g_nMotorSpeedSet > DIFF_SPEED_DEC)
				g_nMotorSpeedSet -= DIFF_SPEED_DEC;
			else if(g_nMotorSpeedSet < -DIFF_SPEED_DEC)
				g_nMotorSpeedSet += DIFF_SPEED_DEC;
	}
*/
	//--------------------------------------------------------------------------
		
}



//==============================================================================
//                END OF THE FILE : CarSub.C
//------------------------------------------------------------------------------