#ifndef __CCD0_H
#define __CCD0_H

void ccd0_init(void);
float ImageCapture(uint8_t * ImageData) ;
void SendImageData(uint8_t * ImageData) ;
//���½��ع���ɼ��ֿ�
void CCD0_exposure(void);
float CCD0_ImageCapture_Data(uint8_t * ImageData) ;
#endif