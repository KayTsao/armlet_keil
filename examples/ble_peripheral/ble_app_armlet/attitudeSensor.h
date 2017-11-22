#ifndef __ATTITUDESENSOR_H
#define __ATTITUDESENSOR_H
#include "math.h"
//#include "quaternion.h"

#define MaxGoodMagSamplesCount (50) 
#define MAX_MatrixDimension (50) 

static const int16_t MagMeasuredRadius = 250 ;

#ifndef bool
    #define bool int
    #define false ((bool)0)
    #define true  ((bool)1)
#endif

#define SAMPLE_TIME_GAP 25 
		
#define ACC_SAMPLES_RATE 500  // ACC采样率
#define ACC_LIMIT_FREQ  10    // ACC截止频率
 
#define _2PI        (6.283185307179f)
#define _3PI_over2  (4.712388980384f)
#define PI          (3.141592653589f)
#define PI_over2    (1.570796326794f)

#define _2_over_PI  (0.636619772367f)
#define _1_over_PI  (0.318309886183f)
#define _1_over_2PI (0.159154943091f)
		
		
#define SLIDE_BUF_LEN 50
#define AccVarWindowSize 25
#define GyrVarWindowSize 25
#define AccVar_StaticThreshold 0.05;
#define GyrVar_StaticThreshold 0.0000025;
		
static int16_t cur_time;
static const float gyroRatio = 0.07f * (PI/180.0f) ;   // ITG3205陀螺仪手册上给的典型值是14.375(LSB/(°/s))
static const float accRatio = ( 16.0f/32768.0f ) ;    // 加速度计的分度值是4mg，即0.004倍的重力加速度。
static const float magRatio_9918 = ( 0.0015f * 0.01f ) ;    // compass 9918
static const float magRatio_9911 = (  0.006f * 0.01f ) ;    // compass 9911

 typedef struct Quaternion_t
{
  
  float w ;
  float x ;
  float y ;
  float z ;
  
} Quaternion ;


typedef struct EulerAngle_t
{
  
  float yaw ;     // ?????????,???Z?
  float pitch ;   // ?????????,???Y?
  float roll ;    // ?????????,???X?
                  // ????????????????????,???????????????????????????
  
} EulerAngle ;


typedef struct Vector3D_t
{
  
  float x ;
  float y ;
  float z ;
  
} Vector3D ;

		
static bool UpdateReady;
static int countNo;


typedef struct {
	int judgeVal[AccVarWindowSize];
	int Idx;
	bool FullArray;
}JudgeWindow_V1;  // info about acc covariance


typedef struct {
	int judgeVal[AccVarWindowSize][3];
	int Idx;
	bool FullArray;
}JudgeWindow_V3;  // info about acc covariance



typedef struct {
	int curPos ;
	int32_t x[SLIDE_BUF_LEN] ;    
	int32_t y[SLIDE_BUF_LEN] ;
	int32_t z[SLIDE_BUF_LEN] ;
	 
	int32_t aveX ;  
	int32_t aveY ;
	int32_t aveZ ;  

	int32_t followPredictX ;
	int32_t followPredictY ;
	int32_t followPredictZ ;

	int32_t sumX ;    
	int32_t sumY ;
	int32_t sumZ ;

} SampleSlideBuf ;



typedef struct AttitudeSensor_t
{ 
	float testOutPut[4];
	//KK add code
	float ax_raw, ay_raw, az_raw;
	float gx_raw, gy_raw, gz_raw;
	float mx_raw, my_raw, mz_raw;

	SampleSlideBuf accSlideBuf;  
	float Acc_SlideAverage[3];
	float Acc_SlideStable[3];
	float Acc_SlidePre[3];  
	float Acc_SlideVariance[3];  
	JudgeWindow_V1 accJudgeWindow;

	SampleSlideBuf gyrSlideBuf; 
	float Gyr_SlideAverage[3];
	float Gyr_SlideStable[3];
	float Gyr_SlidePre[3];  
	float Gyr_SlideVariance[3];  
	JudgeWindow_V3 gyrJudgeWindow;
  
	
	uint32_t ts_cur_ms;
	uint32_t ts_prev_ms;
	float SamplePeriod;
		
	float AccWeight;
	bool GyrStable;
	
	Vector3D GyroBias ; 
		
	Quaternion Ori; 
	
	
	int accSamplesCount;
	int MagSampleSpan;    //(mag 100Hz IMU 500HZ --> 1:5)

	
	// 磁场计低通滤波
	float Mag_Filtered[3]; 
	float Mag_Scale[3];
	float Mag_Bias[3];
	

float MagFilterRatio;
	
	//  Float G_Variance; 


 
	// 磁场水平分量滤波值
	float  filtered_Hmx ;
	float  filtered_Hmy ;
	float  filtered_Hmz ;

	int16_t GyroZero[3] ;

	int16_t AccMax[3] ;
	int16_t AccMin[3] ;

	int16_t MagMax[3] ;
	int16_t MagMin[3] ;

	bool AccMaxReady[3] ;
	bool AccMinReady[3] ;
	bool MagMaxReady[3] ;
	bool MagMinReady[3] ;
	bool MagReady ;
	bool AllReady ;

	int16_t AccOffset[3] ;
	int16_t MagOffset[3] ;
	float MagRatio[3] ; 
	void(*sample)() ; 
	//int accSamplesCount ;                // 记录陀螺和加速度计的采样次数 
	int M_sample_Period ;         // 磁场的采样周期要慢。这个值代表没采样多少次陀螺仪，才采样一次磁场计。   

	uint32_t cur_t_us; 

	
} AttitudeSensor ;


void initAlgoParam(AttitudeSensor * sensor);

void processData(AttitudeSensor * sensor);	 
int processRawData(AttitudeSensor * sensor);
int processRawMag(AttitudeSensor * sensor);
int processRawAcc(AttitudeSensor * sensor);
int processRawGyr(AttitudeSensor * sensor);
int processRawMag(AttitudeSensor * sensor);

void MahonyAHRS(AttitudeSensor * sensor);




typedef struct calibrator_t
{ 
	float MagSamples [MaxGoodMagSamplesCount][3] ;
	float MagOriginEquationFactors[MaxGoodMagSamplesCount][6] ;  
	int idx; 
	float mag_bias_x;
	float mag_bias_y;
	float mag_bias_z;
	
	float mag_scale_x;
	float mag_scale_y;
	float mag_scale_z;
	 
} MagSensorCalibrator ;

void initCalibrator(MagSensorCalibrator * calibrator);
void calcMagParam(MagSensorCalibrator * c);

int GaussElimination ( float**x, int row, int col, int backEnable,float*y, int* sign, float **follow );
int SolveLinearEquations ( float*x, float*y, int n, int xStep, float* result );
//int GaussElimination ( float**x, int row, int col, bool backEnable,float*y, int* sign, float **follow );
void ApplyMagParam2Sensor(MagSensorCalibrator * c, AttitudeSensor * sensor);	

#endif 