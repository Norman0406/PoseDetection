//
//        3D Image Sensor by PEW 
//

extern "C" {

//A value of type CBPROC is a pointer to an application-defined callback.
typedef int (__stdcall *CBPROC)(int Trigger);

// DLL function decralation
int		__stdcall InitImageDriver	( void );
int		__stdcall FreeImageDriver	( void );
int		__stdcall GetImageKN		( unsigned short *kdat, unsigned short *ndat );
int 	__stdcall GetImage3DSht		(short *xdat, short *ydat, short *zdat, unsigned short *ndat);
int 	__stdcall GetImage3DFlt		(float *xdat, float *ydat, float *zdat, unsigned short *ndat);
int 	__stdcall GetImage3DDbl		(double *xdat, double *ydat, double *zdat, unsigned short *ndat);

char*	__stdcall GetDLLversion		( void );

int __stdcall ChangeFreq	( int freq_pat );
int __stdcall Freqmode		( void );
int __stdcall ChangeSleep	( int sleep_pat );
int __stdcall Sleepmode		( void );
int __stdcall InitPara		( void );
int __stdcall ChangeSpeed   ( int speed_pat );
int __stdcall Speedmode     ( void );

int __stdcall ChangeGrayscalePara	( int, int );
void __stdcall GetGrayscalePara		( int*, int* );

int __stdcall SetCallback	( CBPROC );
int __stdcall GetHardversion (void);

}

