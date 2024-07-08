/* 
    GPL - 
    Gianluca Canzolino      -       Master Student of Computer Engineering - UNISA
    Giuseppe Gambardella    -       Master Student of Computer Engineering - UNISA 
    Alberto Provenza        -       Master Student of Computer Engineering - UNISA

/******************************** Header Start ***********************************/

#ifndef __SOC_STM_ENCODER_H__
#define __SOC_STM_ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif
	/************************DriverFunction********************************/
	/************************InitFunctions********************************/
	void initEncoder(unsigned short selection);

	/************************StepFunctions********************************/
	unsigned short int getEncoderCount(unsigned short selection);
	double getSpeed(unsigned short selection);

	/************************ReleaseFunctions********************************/
	void releaseEncoder(unsigned short selection);

	/*******************************************************************/
#ifdef __cplusplus
}
#endif

#endif /* __SOC_STM_ENCODER_H__ */