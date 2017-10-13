#ifndef __ATTITUDESENSOR_H
#define __ATTITUDESENSOR_H
 
#ifndef bool
    #define bool int
    #define false ((bool)0)
    #define true  ((bool)1)
#endif

#define SAMPLE_TIME_GAP 25 
		
#define ACC_SAMPLES_RATE 400  // ACC������
#define ACC_LIMIT_FREQ  20    // ACC��ֹƵ��
 
#define _2PI        (6.283185307179f)
#define _3PI_over2  (4.712388980384f)
#define PI          (3.141592653589f)
#define PI_over2    (1.570796326794f)

#define _2_over_PI  (0.636619772367f)
#define _1_over_PI  (0.318309886183f)
#define _1_over_2PI (0.159154943091f)
		
		
#define SLIDE_BUF_LEN 20
		
static int16_t cur_time;
static const float gyroRatio = 0.07f * (PI/180.0f) ;   // ITG3205�������ֲ��ϸ��ĵ���ֵ��14.375(LSB/(��/s))
static const float accRatio = ( 16.0f/32768.0f ) ;    // ���ٶȼƵķֶ�ֵ��4mg����0.004�����������ٶȡ�
static const float magRatio_9918 = ( 0.0015f * 0.01f ) ;    // compass 9918
static const float magRatio_9911 = (  0.006f * 0.01f ) ;    // compass 9911

 
		
static bool UpdateReady;

typedef struct AttitudeSensor_t
{
  // ���ٶȼ�
  int16_t ax ;
  int16_t ay ;
  int16_t az ;
  int16_t a_align ;   // �����壬ֻΪʹ4�ֽڶ��롣
	
	float ax_raw, ay_raw, az_raw;
	float gx_raw, gy_raw, gz_raw;
	float mx_raw, my_raw, mz_raw;

  // �ų���
  int16_t mx ;
  int16_t my ;
  int16_t mz ;
  int16_t m_align ;   // �����壬ֻΪʹ4�ֽڶ��롣
  // ������
  int16_t gx ;
  int16_t gy ;
  int16_t gz ;
  int16_t g_align ;   // �����壬ֻΪʹ4�ֽڶ��롣
  

  // �ų��Ƶ�ͨ�˲�
  int16_t filtered_mx ;
  int16_t filtered_my ;
  int16_t filtered_mz ;
  
  // �ų�ˮƽ�����˲�ֵ
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
  void* accSlideBuf ; 
  void(*sample)() ; 
  int accSamplesCount ;                // ��¼���ݺͼ��ٶȼƵĲ������� 
  int M_sample_Period ;         // �ų��Ĳ�������Ҫ�������ֵ����û�������ٴ������ǣ��Ų���һ�δų��ơ�   
	
	uint32_t cur_t_us;
	
} AttitudeSensor ;
 
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
#endif //__TIMER_H


static inline uint32_t getCurTime(uint32_t* t)
{
  *t = get_time_us();
  return *t ;
}