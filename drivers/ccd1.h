#ifndef __CCD1_H
#define __CCD1_H

void ccd1_init(void);
float ccd1_ImageCapture(uint8_t * ImageData) ;
//���½��ع���ɼ��ֿ�
void CCD1_exposure(void);
float CCD1_ImageCapture_Data(uint8_t * ImageData) ;
#endif