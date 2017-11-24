#include "stdint.h" 
#include "attitudeSensor.h"  

#include "nrf_log.h"
#include "float.h"

#define FLOAT_MIN (5*FLT_MIN) 


static const float _1_ = 0.99999999 ;
static const float _0_ = 0.00000001 ;

static float getLength(float* V3)
{ 
	return sqrt(V3[0]*V3[0] + V3[1]*V3[1] + V3[2]*V3[2]);
}
static float getLength_v3(Vector3D* V3)
{ 
	Vector3D _v = *V3 ; 
	Vector3D* v = &_v ;
	return sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
}
static float getLength_q4(Quaternion* V4)
{ 
	Quaternion _v = *V4 ; 
	Quaternion * v = &_v ;
	return sqrt(v->w * v->w + v->x * v->x + v->y * v->y + v->z * v->z);
}

static Vector3D getNormalized_v3(Vector3D* V3)
{ 
	float len;
	Vector3D rst;
	Vector3D _v = *V3 ; 
	Vector3D* v = &_v ; 
	len = getLength_v3(v);
	rst.x = v->x / len;
	rst.y = v->y / len;
	rst.z = v->z / len; 
	return rst;
}
static Quaternion getNormalized_q4(Quaternion* q4)
{ 
	float len;
	Quaternion  rst;
	Quaternion  _q = *q4 ; 
	Quaternion * q = &_q ; 
	len = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);//getLength_q4(q);
	rst.w = q->w / len;
	rst.x = q->x / len;
	rst.y = q->y / len;
	rst.z = q->z / len; 
	return rst;
}

static Vector3D Cross(Vector3D* __v1, Vector3D* __v2){
	Vector3D _v1 = *__v1 ; 
	Vector3D* v1 = &_v1 ;
	Vector3D _v2 = *__v2 ; 
	Vector3D* v2 = &_v2 ;

	Vector3D rst;
	rst.x = (v1->y * v2->z - v1->z * v2->y);
	rst.y = (v1->z * v2->x - v1->x * v2->z);
	rst.z = (v1->x * v2->y - v1->y * v2->x);
	return rst;
}

static float Dot(Vector3D* __v1, Vector3D* __v2){
	Vector3D _v1 = *__v1 ; 
	Vector3D* v1 = &_v1 ;
	
	Vector3D _v2 = *__v2 ; 
	Vector3D* v2 = &_v2 ;

	return (v1->x * v2->x + v1->y * v2->y + v1->z * v2->z);
}

// quaternion mult
static Quaternion quaternionMult(Quaternion* __left, Quaternion* __right)
{   
	Quaternion result;
	Quaternion _left = *__left ;
    Quaternion _right = *__right ;
    Quaternion* left = &_left ;
    Quaternion* right = &_right ;
	float d1, d2, d3, d4; 

	d1 =  left->w * right->w; 
	d2 = -left->x * right->x; 
	d3 = -left->y * right->y; 
	d4 = -left->z * right->z; 
	result.w = d1+ d2+ d3+ d4; 

	d1 =  left->w * right->x; 
	d2 =  left->x * right->w; 
	d3 =  left->y * right->z; 
	d4 = -left->z * right->y; 
	result.x =  d1+ d2+ d3+ d4; 

	d1 =  left->w * right->y; 
	d2 =  left->y * right->w; 
	d3 =  left->z * right->x; 
	d4 = -left->x * right->z; 
	result.y =  d1+ d2+ d3+ d4; 

	d1 =  left->w * right->z; 
	d2 =  left->z * right->w; 
	d3 =  left->x * right->y; 
	d4 = -left->y * right->x; 
	result.z =  d1+ d2+ d3+ d4; 
    return result;          
} 

static Quaternion getConj(Quaternion* q) 
{ 
	Quaternion rst;
	rst.w = q->w;
	rst.x = -q->x;
	rst.y = -q->y;
	rst.z = -q->z;
	return rst;
}
 
static Vector3D rotVbyQ(Vector3D* __v, Quaternion* __q)
{
	Vector3D result;
	Quaternion _q = *__q ;
    Vector3D _v = *__v ;
    Quaternion* q = &_q ;
    Vector3D* v = &_v ;
	 
	Quaternion vv;
	vv.w = 0;
	vv.x = v->x;
	vv.y = v->y;
	vv.z = v->z;
	
	Quaternion tmp1, tmp2, q_conj;
	q_conj = getConj(q);
	tmp1 = quaternionMult(q, &vv); 
	tmp2 = quaternionMult(&tmp1, &q_conj);                   //	q*V*q.conj();
    
	result.x = tmp2.x;
	result.y = tmp2.y; 
	result.z = tmp2.z;
	
	return result; 
}

static Quaternion v2q(Vector3D* __v1, Vector3D* __v2)
{
	Quaternion rst;
	Vector3D _v1 = *__v1 ;
    Vector3D _v2 = *__v2 ;
    Vector3D* v1 = &_v1 ;
    Vector3D* v2 = &_v2 ;
	
	Vector3D V_1,V_2,V_3;
	V_1 = getNormalized_v3(v1);
	V_2 = getNormalized_v3(v2);
	  
	V_3.x = V_1.x + V_2.x ;
	V_3.y = V_1.y + V_2.y ;
	V_3.z = V_1.z + V_2.z ;
    
	if ( getLength_v3(&V_3) > _0_ )
    {
		Vector3D v3 = getNormalized_v3(&V_3); 
        Vector3D CrossVal = Cross(&V_1, &v3);
        float w = Dot(&V_1, &v3);
        Quaternion tmp;
		tmp.w = w;
		tmp.x = CrossVal.x;
		tmp.y = CrossVal.y;
		tmp.z = CrossVal.z;
		rst = getNormalized_q4(&tmp);
    }
    else
    {
        Vector3D Any1, Any2;
		Any1.x = 1.0f; Any1.y = 0.0f; Any1.z = 0.0f;
        Any2.x = 0.0f; Any2.y = 1.0f; Any2.z = 0.0f;
        Vector3D Cross1 = Cross(&V_1, &Any1);
        Vector3D Cross2 = Cross(&V_1, &Any2);
        Vector3D CrossVal = getLength_v3(&Cross1) > getLength_v3(&Cross2) ? Cross1 : Cross2 ;
        Quaternion tmp;
		tmp.w = 0.0f;
		tmp.x = CrossVal.x;
		tmp.y = CrossVal.y;
		tmp.z = CrossVal.z; 
		rst = getNormalized_q4(&tmp);  
    }
	return rst;
}

void initAlgoParam(AttitudeSensor *sensor)
{
	UpdateReady = false;
	countNo = 0;
	sensor->ax_raw = sensor->ay_raw = sensor->az_raw = 0.0f;
	sensor->gx_raw = sensor->gy_raw = sensor->gz_raw = 0.0f;
	sensor->mx_raw = sensor->my_raw = sensor->mz_raw = 0.0f;
	
	sensor->accSlideBuf.curPos = 0;
	sensor->accSlideBuf.aveX = sensor->accSlideBuf.aveY  = sensor->accSlideBuf.aveZ = 0.0f; 
	sensor->accSlideBuf.sumX = sensor->accSlideBuf.sumY = sensor->accSlideBuf.sumZ = 0.0f;
	sensor->accSlideBuf.followPredictX = sensor->accSlideBuf.followPredictY = sensor->accSlideBuf.followPredictZ =0.0f;
	
	
	sensor->gyrSlideBuf.curPos = 0;
	sensor->gyrSlideBuf.aveX = sensor->gyrSlideBuf.aveY = sensor->gyrSlideBuf.aveZ = 0.0f; 
	sensor->gyrSlideBuf.sumX = sensor->gyrSlideBuf.sumY = sensor->gyrSlideBuf.sumZ = 0.0f;
	sensor->gyrSlideBuf.followPredictX = sensor->gyrSlideBuf.followPredictY = sensor->gyrSlideBuf.followPredictZ =0.0f;
	
	int i = 0;
	for(i=0; i < SLIDE_BUF_LEN; i++)
	{
		sensor->accSlideBuf.x[i] = sensor->accSlideBuf.y[i] = sensor->accSlideBuf.z[i] = 0.0f; 
		sensor->gyrSlideBuf.x[i] = sensor->gyrSlideBuf.y[i] = sensor->gyrSlideBuf.z[i] = 0.0f; 
	} 
	sensor->Acc_SlideAverage[0] = sensor->Acc_SlideAverage[1] = sensor->Acc_SlideAverage[2] = 0.0f;
	sensor->Acc_SlideStable[0] = sensor->Acc_SlideStable[1] = sensor->Acc_SlideStable[2] = 0.0f ;
	sensor->Acc_SlidePre[0] = sensor->Acc_SlidePre[1] = sensor->Acc_SlidePre[2] = 0.0f;  
	sensor->Acc_SlideVariance[0] = sensor->Acc_SlideVariance[1] = sensor->Acc_SlideVariance[2] = 1.0f; 
	
	sensor->Gyr_SlideAverage[0] = sensor->Gyr_SlideAverage[1] = sensor->Gyr_SlideAverage[2] = 0.0f;
	sensor->Gyr_SlideStable[0] = sensor->Gyr_SlideStable[1] = sensor->Gyr_SlideStable[2] = 0.0f ;
	sensor->Gyr_SlidePre[0] = sensor->Gyr_SlidePre[1] = sensor->Gyr_SlidePre[2] = 0.0f;  
	sensor->Gyr_SlideVariance[0] = sensor->Gyr_SlideVariance[1] = sensor->Gyr_SlideVariance[2] = 1.0f; 
	
	sensor->accJudgeWindow.FullArray = false;
	sensor->accJudgeWindow.Idx = 0;
	
	sensor->gyrJudgeWindow.FullArray = false;
	sensor->gyrJudgeWindow.Idx = 0;
	for(i = 0; i < AccVarWindowSize; i++)
	{
		sensor->accJudgeWindow.judgeVal[i] = 1;
		
		sensor->gyrJudgeWindow.judgeVal[i][0] = 1;
		sensor->gyrJudgeWindow.judgeVal[i][1] = 1;
		sensor->gyrJudgeWindow.judgeVal[i][2] = 1;
	}
	
	sensor->GyroBias.x = sensor->GyroBias.y = sensor->GyroBias.z =0.0f ; 
	
	sensor->Ori.w = 1.0f; 
	sensor->Ori.x = 0.0f; 
	sensor->Ori.y = 0.0f; 
	sensor->Ori.z = 0.0f;
	
	sensor->AccWeight = 1.0f;
	sensor->SamplePeriod = 0.0f;
	sensor->GyrStable = false;
	
	sensor->ts_cur_ms = 0;
	sensor->ts_prev_ms = 0;
	  
	sensor->accSamplesCount = 0; 
    sensor->MagSampleSpan = 5;    //(mag 100Hz IMU 500HZ --> 1:5)

    sensor->Mag_Filtered[0] = sensor->Mag_Filtered[1] = sensor->Mag_Filtered[2] = 0.0f;
	sensor->MagFilterRatio = 0.02f ; 
	
	sensor->Mag_Scale[0] = 1.062f;
	sensor->Mag_Scale[1] = 1.0367f;
	sensor->Mag_Scale[2] = 1.0f; 
	sensor->Mag_Bias[0] = -0.034f; 
	sensor->Mag_Bias[1] = 0.0595f; 
	sensor->Mag_Bias[2] = 0.0f; 
/*	
0.548188 -0.089588 0.0
0.523096 0.265903 0.0
0.509748 -0.008840 0.0
*/
	return;
}

int processRawMag(AttitudeSensor *s)
{ 
	float Mag[3]; 
	Mag[0] = s->mx_raw * 256.0f;//64.0f; 
	Mag[1] = s->my_raw * 256.0f;//64.0f;
	Mag[2] = s->mz_raw * 256.0f;//64.0f; 

	Mag[0] = s->Mag_Scale[0] * (Mag[0] - s->Mag_Bias[0]); 
	Mag[1] = s->Mag_Scale[1] * (Mag[1] - s->Mag_Bias[1]); 
	Mag[2] = s->Mag_Scale[2] * (Mag[2] - s->Mag_Bias[2]); 

	float Ax, Ay, Az ;
	float Mx, My, Mz ;
	float  x,  y,  z ;
	float  ratio = 0.02f;  //MagFilterRatio = 0.02f ;
	x = s->Mag_Filtered[0] - Mag[0] ;
	y = s->Mag_Filtered[1] - Mag[1];
	z = s->Mag_Filtered[2] - Mag[2] ;

	float d = sqrt(x*x+y*y+z*z);

	if ( d < 0.05f * 0.5 )
		ratio = 0.01f ;
	else if ( d < 0.1f * 0.5 )
		ratio = 0.02f ;
	else if ( d < 0.25f * 0.5 )
		ratio = 0.1f ;
	else
		ratio = 0.2f ;
	s->Mag_Filtered[0] = s->Mag_Filtered[0] * (1.0f - ratio) + Mag[0] * ratio ;
	s->Mag_Filtered[1] = s->Mag_Filtered[1] * (1.0f - ratio) + Mag[1] * ratio ;
	s->Mag_Filtered[2] = s->Mag_Filtered[2] * (1.0f - ratio) + Mag[2] * ratio ; 	

	return 0;
}

int processRawAcc(AttitudeSensor *sensor)
{
	SampleSlideBuf *accBuf;
	accBuf = &sensor->accSlideBuf; 
	//gyrBuf = &sensor->gyrSlideBuf; 

	float rawAcc[3];
	rawAcc[0] = sensor->ax_raw; rawAcc[1] = sensor->ay_raw; rawAcc[2] = sensor->az_raw; 
	
	int curPos = accBuf->curPos ;
	int nextPos = curPos + 1 ;
	if (nextPos==SLIDE_BUF_LEN)
		nextPos = 0 ;

	//Last Value in Buffer -- to eliminate
	int oldX = accBuf->x[nextPos] ;
	int oldY = accBuf->y[nextPos] ;
	int oldZ = accBuf->z[nextPos] ;

	int ratio = 50;            // LOW_PASS_RATIO = sampleRate/limit_freq (500/10);  //??????????????,
	int ratio2 = ratio - 1 ;
  //LPF
	accBuf->x[nextPos] = ( accBuf->x[curPos]*ratio2 + 1000*rawAcc[0] ) / ratio ;
	accBuf->y[nextPos] = ( accBuf->y[curPos]*ratio2 + 1000*rawAcc[1] ) / ratio ;
	accBuf->z[nextPos] = ( accBuf->z[curPos]*ratio2 + 1000*rawAcc[2] ) / ratio ;
	accBuf->curPos = nextPos ;
	//Update sum, moving average and follow update

	int32_t deltaX = accBuf->x[nextPos] - oldX ;
	int32_t deltaY = accBuf->y[nextPos] - oldY ;
	int32_t deltaZ = accBuf->z[nextPos] - oldZ ;

	accBuf->sumX += deltaX ;
	accBuf->sumY += deltaY ;
	accBuf->sumZ += deltaZ ;

	accBuf->aveX = (accBuf->sumX) / (SLIDE_BUF_LEN) ;
	accBuf->aveY = (accBuf->sumY) / (SLIDE_BUF_LEN) ;
	accBuf->aveZ = (accBuf->sumZ) / (SLIDE_BUF_LEN) ;
	
	accBuf->followPredictX = accBuf->aveX + deltaX ;
	accBuf->followPredictY = accBuf->aveY + deltaY ;
	accBuf->followPredictZ = accBuf->aveZ + deltaZ ;
	
	// after LPF
	sensor->Acc_SlideStable[0] = (accBuf->x[accBuf->curPos]) / 1000.0f;
	sensor->Acc_SlideStable[1] = (accBuf->y[accBuf->curPos]) / 1000.0f;
	sensor->Acc_SlideStable[2] = (accBuf->z[accBuf->curPos]) / 1000.0f;
	
	
	sensor->Acc_SlidePre[0] = (accBuf->x[(accBuf->curPos + SLIDE_BUF_LEN -1)%SLIDE_BUF_LEN]) / 1000.0f;
	sensor->Acc_SlidePre[1] = (accBuf->y[(accBuf->curPos + SLIDE_BUF_LEN -1)%SLIDE_BUF_LEN]) / 1000.0f;
	sensor->Acc_SlidePre[2] = (accBuf->z[(accBuf->curPos + SLIDE_BUF_LEN -1)%SLIDE_BUF_LEN]) / 1000.0f;
	
	sensor->Acc_SlideAverage[0] = (accBuf->aveX) / 1000.0f;
	sensor->Acc_SlideAverage[1] = (accBuf->aveY) / 1000.0f;
	sensor->Acc_SlideAverage[2] = (accBuf->aveZ) / 1000.0f;	 
	
	int i ;
	float tmp_Var[3] = {0.0f, 0.0f, 0.0f};
  
	for (i=0; i<SLIDE_BUF_LEN; i++)
	{
		float tmp[3] = {(accBuf->x[i])/1000.0f - sensor->Acc_SlideAverage[0], 
										(accBuf->y[i])/1000.0f - sensor->Acc_SlideAverage[1], 
										(accBuf->z[i])/1000.0f - sensor->Acc_SlideAverage[2]};
		tmp_Var[0] += tmp[0] * tmp[0];
		tmp_Var[1] += tmp[1] * tmp[1];
		tmp_Var[2] += tmp[2] * tmp[2];
	}
	sensor->Acc_SlideVariance[0] = tmp_Var[0] / SLIDE_BUF_LEN;
	sensor->Acc_SlideVariance[1] = tmp_Var[1] / SLIDE_BUF_LEN;
	sensor->Acc_SlideVariance[2] = tmp_Var[2] / SLIDE_BUF_LEN;
	
//Fill in AccJudgeWindow	 
	{
		if( getLength(sensor->Acc_SlideVariance) > 0.005) //AccVar_StaticThreshold)
		{
			sensor->accJudgeWindow.judgeVal[sensor->accJudgeWindow.Idx] = 1; 
		}
		else
		{
			sensor->accJudgeWindow.judgeVal[sensor->accJudgeWindow.Idx] = 0;
		} 
		sensor->accJudgeWindow.Idx++;
		if(sensor->accJudgeWindow.Idx == AccVarWindowSize)
		{
			if(!sensor->accJudgeWindow.FullArray)
			{
				sensor->accJudgeWindow.FullArray = true;
			}
			sensor->accJudgeWindow.Idx = 0;
		}
	} 
//Make judgement basing on values in AccJudgeWindow	
	{
		if(sensor->accJudgeWindow.FullArray)
		{
			int judge;
			judge = 0;
			int i = 0;
			for(int i = 0; i < AccVarWindowSize; i++)
			{
				judge += sensor->accJudgeWindow.judgeVal[i];
			}		
			if(judge == 0)
			{
				 sensor->AccWeight = 1;
			}
			else
			{
				sensor->AccWeight = 0;	
			}
        } 
	}

	sensor->accSamplesCount++;
	return 0;
}	


int processRawGyr(AttitudeSensor *sensor)
{
	SampleSlideBuf *gyrBuf;  
	gyrBuf = &sensor->gyrSlideBuf; 

	float Gyr[3];
	Gyr[0] = sensor->gx_raw - sensor->GyroBias.x; 
	Gyr[1] = sensor->gy_raw - sensor->GyroBias.y;
	Gyr[2] = sensor->gz_raw - sensor->GyroBias.z;


	int curPos = gyrBuf->curPos ;
	int nextPos = curPos + 1 ;
	if (nextPos == SLIDE_BUF_LEN)
		nextPos = 0 ;

	//Last Value in Buffer -- to eliminate
	int oldX = gyrBuf->x[nextPos] ;
	int oldY = gyrBuf->y[nextPos] ;
	int oldZ = gyrBuf->z[nextPos] ;

	int ratio = 2;            //20;//LOW_PASS_RATIO ;  //??????????????,
	int ratio2 = ratio - 1 ;
  //LPF
	gyrBuf->x[nextPos] = ( gyrBuf->x[curPos]*ratio2 + 1000* Gyr[0] ) / ratio ;
	gyrBuf->y[nextPos] = ( gyrBuf->y[curPos]*ratio2 + 1000* Gyr[1] ) / ratio ;
	gyrBuf->z[nextPos] = ( gyrBuf->z[curPos]*ratio2 + 1000* Gyr[2] ) / ratio ;
	gyrBuf->curPos = nextPos ;
	//Update sum, moving average and follow update

	int32_t deltaX = gyrBuf->x[nextPos] - oldX ;
	int32_t deltaY = gyrBuf->y[nextPos] - oldY ;
	int32_t deltaZ = gyrBuf->z[nextPos] - oldZ ;

	gyrBuf->sumX += deltaX ;
	gyrBuf->sumY += deltaY ;
	gyrBuf->sumZ += deltaZ ;

	gyrBuf->aveX = (gyrBuf->sumX) / (SLIDE_BUF_LEN) ;
	gyrBuf->aveY = (gyrBuf->sumY) / (SLIDE_BUF_LEN) ;
	gyrBuf->aveZ = (gyrBuf->sumZ) / (SLIDE_BUF_LEN) ;
	
	gyrBuf->followPredictX = gyrBuf->aveX + deltaX ;
	gyrBuf->followPredictY = gyrBuf->aveY + deltaY ;
	gyrBuf->followPredictZ = gyrBuf->aveZ + deltaZ ;
	
	// after LPF
	sensor->Gyr_SlideStable[0] = (gyrBuf->x[gyrBuf->curPos]) / 1000.0f;
	sensor->Gyr_SlideStable[1] = (gyrBuf->y[gyrBuf->curPos]) / 1000.0f;
	sensor->Gyr_SlideStable[2] = (gyrBuf->z[gyrBuf->curPos]) / 1000.0f;
	
	sensor->Gyr_SlidePre[0] = (gyrBuf->x[(gyrBuf->curPos + SLIDE_BUF_LEN -1)%SLIDE_BUF_LEN]) / 1000.0f;
	sensor->Gyr_SlidePre[1] = (gyrBuf->y[(gyrBuf->curPos + SLIDE_BUF_LEN -1)%SLIDE_BUF_LEN]) / 1000.0f;
	sensor->Gyr_SlidePre[2] = (gyrBuf->z[(gyrBuf->curPos + SLIDE_BUF_LEN -1)%SLIDE_BUF_LEN]) / 1000.0f;
	
	sensor->Gyr_SlideAverage[0] = (gyrBuf->aveX) / 1000.0f;
	sensor->Gyr_SlideAverage[1] = (gyrBuf->aveY) / 1000.0f;
	sensor->Gyr_SlideAverage[2] = (gyrBuf->aveZ) / 1000.0f;	 
	
	int i ;
	float tmp_Var[3] = {0.0f, 0.0f, 0.0f};
  
	for (i=0; i<SLIDE_BUF_LEN; i++)
	{
		float tmp[3] = {(gyrBuf->x[i])/1000.0f - sensor->Gyr_SlideAverage[0], 
										(gyrBuf->y[i])/1000.0f - sensor->Gyr_SlideAverage[1], 
										(gyrBuf->z[i])/1000.0f - sensor->Gyr_SlideAverage[2]};
		tmp_Var[0] += tmp[0] * tmp[0];
		tmp_Var[1] += tmp[1] * tmp[1];
		tmp_Var[2] += tmp[2] * tmp[2];
	}
	sensor->Gyr_SlideVariance[0] = tmp_Var[0] / SLIDE_BUF_LEN;
	sensor->Gyr_SlideVariance[1] = tmp_Var[1] / SLIDE_BUF_LEN;
	sensor->Gyr_SlideVariance[2] = tmp_Var[2] / SLIDE_BUF_LEN;
	
//Fill in GyrJudgeWindow	 
	{
		////X Axis
		if( (sensor->Gyr_SlideVariance[0]) > 0.0000025)       // GyrVar_StaticThreshold )
		{
			sensor->gyrJudgeWindow.judgeVal[sensor->gyrJudgeWindow.Idx][0] = 1; 
		}
		else
		{
			sensor->gyrJudgeWindow.judgeVal[sensor->gyrJudgeWindow.Idx][0] = 0;
		}
		////Y Axis
		if( (sensor->Gyr_SlideVariance[1]) >  0.0000025)       // GyrVar_StaticThreshold)
		{
			sensor->gyrJudgeWindow.judgeVal[sensor->gyrJudgeWindow.Idx][1] = 1; 
		}
		else
		{
			sensor->gyrJudgeWindow.judgeVal[sensor->gyrJudgeWindow.Idx][1] = 0;
		}
		////Z Axis
		if( (sensor->Gyr_SlideVariance[2]) >  0.0000025)       // GyrVar_StaticThreshold )
		{
			sensor->gyrJudgeWindow.judgeVal[sensor->gyrJudgeWindow.Idx][2] = 1; 
		}
		else
		{
			sensor->gyrJudgeWindow.judgeVal[sensor->gyrJudgeWindow.Idx][2] = 0;
		} 
		sensor->gyrJudgeWindow.Idx++;
		if(sensor->gyrJudgeWindow.Idx == GyrVarWindowSize)
		{
			if(!sensor->gyrJudgeWindow.FullArray)
			{
				sensor->gyrJudgeWindow.FullArray = true;
			}
			sensor->gyrJudgeWindow.Idx = 0;
		}
	} 
//Make judgement basing on values in AccJudgeWindow	
	{
		if(sensor->gyrJudgeWindow.FullArray)
		{
			int judge[3] = {0,0,0} ; 
			int i = 0;
			for(int i = 0; i < GyrVarWindowSize; i++)
			{
				judge[0] += sensor->gyrJudgeWindow.judgeVal[i][0];
				judge[1] += sensor->gyrJudgeWindow.judgeVal[i][1];
				judge[2] += sensor->gyrJudgeWindow.judgeVal[i][2];
			}		
			if(judge[0] == 0)
			{
				sensor->GyroBias.x += sensor->Gyr_SlideAverage[0];
				sensor->Gyr_SlideStable[0] = 0;
			}
			if(judge[1] == 0)
			{
				sensor->GyroBias.y += sensor->Gyr_SlideAverage[1];
					sensor->Gyr_SlideStable[1] = 0;
			//sensor->gy = 0;	
			}
			if(judge[2] == 0)
			{
				sensor->GyroBias.z += sensor->Gyr_SlideAverage[2];
					sensor->Gyr_SlideStable[2] = 0;
			//sensor->gz = 0;	
			} 
        } 
	}	
	return 0;
}	

int processRawData(AttitudeSensor *sensor){

	processRawAcc(sensor);
	processRawGyr(sensor);
	if(sensor->accSamplesCount % sensor->MagSampleSpan == 0)
	{
		processRawMag(sensor);
	}		
	return 0;
} 




void MahonyAHRS(AttitudeSensor * sensor)
{ 
	Vector3D raw_Acc, raw_Mag, raw_Gyr;
	raw_Acc.x = sensor->Acc_SlideStable[0];//ax; 
	raw_Acc.y = sensor->Acc_SlideStable[1];//ay; 
	raw_Acc.z = sensor->Acc_SlideStable[2];//az;
	raw_Mag.x = sensor->Mag_Filtered[0];
	raw_Mag.y = sensor->Mag_Filtered[1];
	raw_Mag.z = sensor->Mag_Filtered[2];
	raw_Gyr.x = sensor->Gyr_SlideStable[0] ;//- sensor->GyroBias.x;  //gx ;//- sensor->GyroBias.x;
	raw_Gyr.y = sensor->Gyr_SlideStable[1] ;//- sensor->GyroBias.y; //gy ; //- sensor->GyroBias.y;
	raw_Gyr.z = sensor->Gyr_SlideStable[2] ;//- sensor->GyroBias.z; //gz ; //- sensor->GyroBias.z;
	
	if(getLength_v3(&raw_Acc) == 0.0f || getLength_v3(&raw_Mag) == 0.0f)
		return;
	
	Quaternion q_prev;
	
	q_prev.w = sensor->Ori.w;
	q_prev.x = sensor->Ori.x;
	q_prev.y = sensor->Ori.y;
	q_prev.z = sensor->Ori.z; 

	Quaternion q, deltaQ, qgyr, qtmp, rstQ;
	Vector3D accErr, magErr, err;
///////Acc Part
	Vector3D EstimateG;
    EstimateG.x = 2 * (q_prev.x * q_prev.z - q_prev.w * q_prev.y);
    EstimateG.y = 2 * (q_prev.w * q_prev.x + q_prev.y * q_prev.z);
    EstimateG.z = q_prev.w * q_prev.w - q_prev.x * q_prev.x - q_prev.y * q_prev.y + q_prev.z * q_prev.z; 
	Vector3D acc_norm = getNormalized_v3(&raw_Acc); 
	accErr = Cross(&acc_norm, &EstimateG);  
///////Mag Part 
	
	
	Vector3D mag_norm = getNormalized_v3(&raw_Mag);
	// Reference direction of Earth's magnetic feild 
    Vector3D h = rotVbyQ(&mag_norm, &q_prev);
	float b2 = sqrtf(h.x * h.x + h.y * h.y);
	float b4 = h.z; 
	Vector3D EstimateM;
	// Estimated direction of magnetic field
    EstimateM.x = 2 * b2 * (0.5 - q_prev.y * q_prev.y - q_prev.z *q_prev.z) + 2 * b4 * (q_prev.x * q_prev.z - q_prev.w * q_prev.y);
    EstimateM.y = 2 * b2 * (q_prev.x * q_prev.y - q_prev.w * q_prev.z) + 2 * b4 * (q_prev.w * q_prev.x + q_prev.y * q_prev.z);
    EstimateM.z = 2 * b2 * (q_prev.w * q_prev.y + q_prev.x * q_prev.z) + 2 * b4 * (0.5 - q_prev.x * q_prev.x - q_prev.y * q_prev.y);
	magErr = Cross(&mag_norm, &EstimateM); 
	
	float sinAlpha = getLength_v3(&magErr);
    float rad = asinf(sinAlpha);
    float deg = rad * 57.3;
	
	
	float Kp = 1.0f;

    if((deg < 5.0 && deg > -5.0))
    {
        magErr.x = magErr.y = magErr.z = 0.0;
    }
    else
    {
		//if(deg > 10 || deg < -10)
		//{
			Kp = 10.0f;
		//}
			
    }
	//Error is sum of cross product between estimated direction and measured direction of fields
	err.x = sensor->AccWeight * (accErr.x + magErr.x);
	err.y = sensor->AccWeight * (accErr.y + magErr.y);
	err.z = sensor->AccWeight * (accErr.z + magErr.z);
 


	// Apply feedback terms
	Vector3D revised_gyr;
	revised_gyr.x = raw_Gyr.x + Kp * err.x;
	revised_gyr.y = raw_Gyr.y + Kp * err.y;
	revised_gyr.z = raw_Gyr.z + Kp * err.z;

	
	qgyr.w = 0;
	qgyr.x = revised_gyr.x;
	qgyr.y = revised_gyr.y;
	qgyr.z = revised_gyr.z;
	
	qtmp.w = 0.5 * q_prev.w;
	qtmp.x = 0.5 * q_prev.x;
	qtmp.y = 0.5 * q_prev.y;
	qtmp.z = 0.5 * q_prev.z; 
	 
	
	deltaQ = quaternionMult(&qtmp, &qgyr);
	float dt = sensor->SamplePeriod;	
	deltaQ.w = deltaQ.w * dt;
	deltaQ.x = deltaQ.x * dt;
	deltaQ.y = deltaQ.y * dt;
	deltaQ.z = deltaQ.z * dt;
 
	q.w = q_prev.w + deltaQ.w;
	q.x = q_prev.x + deltaQ.x;
	q.y = q_prev.y + deltaQ.y;
	q.z = q_prev.z + deltaQ.z;
	sensor->Ori =  getNormalized_q4(&q);

//NRF_LOG_PRINTF("RAW_GYR: %5.4f %5.4f %5.4f \n\n\n",raw_Acc.x,raw_Acc.y,raw_Acc.z);

return;
   
}

void processData(AttitudeSensor * sensor)
{
	int process_Err_code;
	process_Err_code = processRawData(sensor); 
	if(process_Err_code == 0)
		MahonyAHRS(sensor);  
	
	return; 
}


//=================================Mag_Calibration_Part================================

void ApplyMagParam2Sensor(MagSensorCalibrator * c, AttitudeSensor * sensor)
{
	sensor->Mag_Scale[0] = c->mag_scale_x;
	sensor->Mag_Scale[1] = c->mag_scale_y;
	sensor->Mag_Scale[2] = c->mag_scale_z; 
	sensor->Mag_Bias[0] = c->mag_bias_x; 
	sensor->Mag_Bias[1] = c->mag_bias_y; 
	sensor->Mag_Bias[2] = c->mag_bias_z; 
	return;

}




void initCalibrator(MagSensorCalibrator * c)
{
	c->idx = 0;  
	c->mag_bias_x = 0.0f;
	c->mag_bias_y = 0.0f;
	c->mag_bias_z = 0.0f;
	
	c->mag_scale_x = 1.0f;
	c->mag_scale_y = 1.0f;
	c->mag_scale_z = 1.0f;
	return;
}

int AddMagSample(MagSensorCalibrator* c, AttitudeSensor * s)
{
	int idx = c->idx; 
	float mx,my,mz;
	float dx,dy,dz;
	float minRatio = 0.3f;
	int i ;
	float length_diff, length_self;
			
	mx = s->mx_raw * 256;
	my = s->my_raw * 256;
	mz = s->mz_raw * 256;
	if(mx == 0 && my == 0 && mz ==0 )
		return 0;
	 
	for(i = 0 ; i < idx; i++)
	{
		dx =  c->MagSamples[i][0] - mx;
		dy =  c->MagSamples[i][1] - my;
		dz =  c->MagSamples[i][2] - mz;
		length_diff = dx*dx+dy*dy+dz*dz ;
		length_self = (minRatio*minRatio) * (mx*mx + my*my + mz*mz);//应该会有问题 还有待确定。。。。
		if(length_diff < length_self)
			return 0; 
	}
	if(i == idx)
	{
		//NRF_LOG_PRINTF("diff Len:%f  self_Len%f\n",length_diff, length_self);
		
		c->MagSamples[i][0] = mx;
		c->MagSamples[i][1] = my;
		c->MagSamples[i][2] = mz;
		
		c->MagOriginEquationFactors[i][0] = mx * mx;
		c->MagOriginEquationFactors[i][1] = -2 * mx;
		c->MagOriginEquationFactors[i][2] = my * my;
		c->MagOriginEquationFactors[i][3] = -2 * my;
		c->MagOriginEquationFactors[i][4] = mz * mz;
		c->MagOriginEquationFactors[i][5] = -2 * mz; 
		c->idx++;
	}
	return 1;
}

void calcMagParam(MagSensorCalibrator * c)
{
	//NRF_LOG_PRINTF("\n insideFunc\n"); //NRF_LOG_PRINTF("insideFunc addr:%x %x %x \n", c, c->MagSamples, &(c->MagSamples[0][0]));
	float(*p)[6] = c->MagOriginEquationFactors;//MagOriginEquationFactors;  // [50][6]     //float(*s)[3] = c->MagSamples;
	int i;
	float x[6][6];
	float y[6]; 	
	for(i = 0; i < 6; i++)
	{
		int j,k;
		//计算x[i][:]
		for(j = 0; j < 6; j++)
		{
			x[i][j] = 0;
			for(k = 0; k < MaxGoodMagSamplesCount; k++)
			{
				x[i][j] += p[k][j] * p[k][i];
			}
		}
		//计算y[i]
		y[i] = 0;
		for(k = 0; k < MaxGoodMagSamplesCount; k++)
		{
			float weight = 1;
			if(k < 6)
			{
				weight *= 1.0f;
			}
			y[i] += p[k][i] * weight ;
		}
		y[i] *= MagMeasuredRadius * MagMeasuredRadius;
	}  
	//求解拟合的方程 
	if(SolveLinearEquations( (float*)x, y, 6, 6, y))
	{  
		//计算校正参数
		float ratioX, ratioY, ratioZ ;
        float offsetX, offsetY, offsetZ ;

        offsetX = y[1]/y[0] ;
        offsetY = y[3]/y[2] ;
        offsetZ = y[5]/y[4] ;

        ratioX = sqrt(y[0]/y[4]) ;
        ratioY = sqrt(y[2]/y[4]) ;
        ratioZ = 1 ;   //比例都以z轴为参考
		
		c->mag_bias_x = offsetX;
		c->mag_bias_y = offsetY;
		c->mag_bias_z = offsetZ;
		c->mag_scale_x = ratioX;
		c->mag_scale_y = ratioY;
		c->mag_scale_z = ratioZ;
		
		//NRF_LOG_PRINTF("Infunc offset: %f\t%f\t%f\nScale: %f\t%f\t%f\n",offsetX,offsetY,offsetZ,ratioX,ratioY,ratioZ);
	}
}


//=================================Matrix_Part==============================================

/*高斯消元法
 *
 * 参数 : 
 * x:  待消元的矩阵，二级指针 且输入的内容会被改变 不使能得到上三角矩阵
 * row： 矩阵行数 
 * col:矩阵列数 
 * backEnable：是否向回消 使能后得到对角阵
 * y: 用于求解方程组 方程组等号右边的列
 * sign：用于求解行列式 表示矩阵在变换前后行列式的正负号发生过变化 返回true代表没有发生变化 
 * follow：用于求逆 本身是矩阵 一般 输入一个与x一样大的单位矩阵作为follow，它将跟随x发生变化
 * 返回值：
 * true: 行列式非0
*/
int GaussElimination ( float**x, int row, int col, int backEnable,float*y, int* sign, float **follow )
{
	int ret  = 1;
	int i, j ;
  
	if ( sign )
		*sign = 1 ;
  
	for ( i=0; i<row; i++ )
	{
		if ( i >= col )      
		break ;

		for ( j=i; j<row; j++ )
		{
		  if ( x[j][i] >= FLOAT_MIN || x[j][i] <= - FLOAT_MIN  )
			break ;
		  else
			x[j][i] = 0.0f ;
		}

		if ( j == row )
		{
		  ret = 0; //false ;
		  continue ; 
		}
		else if ( j > i )
		{
		  // swap row i and row j 
		  float *temp = x[i] ;
		  x[i] = x[j] ;
		  x[j] = temp ;

		  if (follow)
		  {
			float *temp = follow[i] ;
			follow[i] = follow[j] ;
			follow[j] = temp ;
		  }
		  
		  if (y)
		  {
			float temp = y[i] ;
			y[i] = y[j] ;
			y[j] = temp ;
		  }
		  
		  if (sign)
		  {
			*sign  *=  -1 ;   
		  }
		}
		int start ;
		if ( backEnable == 1)
		start = 0 ;
		else
		start = i+1 ;
		for ( j=start; j<row; j++ )
		{
			if (j==i)
				continue ;
			float ratio = -x[j][i] / x[i][i] ;
			//  x[j] = x[i]*ratio + x[j] ;
			int k ;
			for ( k=i; k<col; k++ )
			{
				x[j][k] += ratio*x[i][k] ;
				if ( k==i )
				  x[j][k] = 0.0f ;      
				if (follow)
				  follow[j][k] += ratio*follow[i][k] ;
			} 
			if (y)
			y[j] += ratio*y[i] ;
		}
	}	  
	return ret ;
}


static float* EquationsXp  [MAX_MatrixDimension] ;
static float* MatrixFollowp [MAX_MatrixDimension] ;
static float  EquationsY  [MAX_MatrixDimension] ;

/**
解线性方程组
为了节省单片机的栈 函数内部使用了静态数组EquationsXp,因此本函数不可重入
*/
int SolveLinearEquations ( float*x, float*y, int n, int xStep, float* result )
{
    if ( n > MAX_MatrixDimension )
      return 0;    // ????
  
    int i ;
    float ** px = EquationsXp ;
    float * py = EquationsY ;
    for (i=0; i<n;i++)
    {
      px[i] = x + xStep*i ;
      py[i] = y[i] ;
    }
    
    //if ( GaussElimination ( px, n, n, true, py, 0, 0 ) )
    if ( GaussElimination ( px, n, n, 1, py, 0, 0 ) ) 
	{
        for (i=0; i<n; i++)
          result [ i ] = py[i] / px[i][i] ;
        return 1 ;
    }
    else
    {
      return 0 ;
    }
}

 