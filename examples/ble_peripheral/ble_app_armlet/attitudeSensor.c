#include "stdint.h" 
#include "attitudeSensor.h"  


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
	len = getLength_q4(q);
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
	int i = 0;
	for(i=0; i < SLIDE_BUF_LEN; i++)
	{
		sensor->accSlideBuf.x[i] = sensor->accSlideBuf.y[i] = sensor->accSlideBuf.z[i] = 0.0f; 
	} 
	sensor->Acc_SlideAverage[0] = sensor->Acc_SlideAverage[1] = sensor->Acc_SlideAverage[2] = 0.0f;
	sensor->Acc_SlideStable[0] = sensor->Acc_SlideStable[1] = sensor->Acc_SlideStable[2] = 0.0f ;
	sensor->Acc_SlidePre[0] = sensor->Acc_SlidePre[1] = sensor->Acc_SlidePre[2] = 0.0f;  
	sensor->Acc_SlideVariance[0] = sensor->Acc_SlideVariance[1] = sensor->Acc_SlideVariance[2] = 0.0f; 
	
	sensor->accJudgeWindow.FullArray = false;
	sensor->accJudgeWindow.Idx = 0;
	for(i = 0; i < AccVarWindowSize; i++)
	{
		sensor->accJudgeWindow.judgeVal[i] = 1;
	} 
	sensor->GyroBias[0] = sensor->GyroBias[1] = sensor->GyroBias[2] =0.0f ; 
	sensor->Ori.w = 1.0f; sensor->Ori.x = 0.0f; sensor->Ori.y = 0.0f; sensor->Ori.z = 0.0f;
	sensor->AccWeight = 0.0f;
	sensor->SamplePeriod = 0.0f;
	
	sensor->ts_cur_ns = 0;
	sensor->ts_prev_ns = 0;
	
	
	return;
}
  

int processRawData(AttitudeSensor *sensor){
	
	
	if(sensor->ts_prev_ns == sensor->ts_cur_ns)
    { 
        return 1; 
    }
	uint32_t dt ;
    if(sensor->ts_prev_ns > sensor->ts_cur_ns) //prev is larger than current
    {
		dt = 4294967296.0 - sensor->ts_prev_ns + sensor->ts_cur_ns; 
    }
	else
	{	
		dt = sensor->ts_cur_ns - sensor->ts_prev_ns;
	}
	 
    sensor->SamplePeriod = 0.000001f * dt;
	
	
	SampleSlideBuf *accBuf;
	SampleSlideBuf *gyrBuf;

	accBuf = &sensor->accSlideBuf; 
	//gyrBuf = &sensor->gyrSlideBuf; 

	float rawAcc[3], rawGyr[3];
	rawAcc[0] = sensor->ax_raw; rawAcc[1] = sensor->ay_raw; rawAcc[2] = sensor->az_raw; 
	rawGyr[0] = sensor->gx_raw; rawGyr[1] = sensor->gy_raw;	rawGyr[2] = sensor->gz_raw;


	int curPos = accBuf->curPos ;
	int nextPos = curPos + 1 ;
	if (nextPos==SLIDE_BUF_LEN)
		nextPos = 0 ;

	//Last Value in Buffer -- to eliminate
	int oldX = accBuf->x[nextPos] ;
	int oldY = accBuf->y[nextPos] ;
	int oldZ = accBuf->z[nextPos] ;

	int ratio = 2;            //20;//LOW_PASS_RATIO ;  //??????????????,
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
		if( getLength(sensor->Acc_SlideVariance) > AccVar_StaticThreshold)
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
		
	}
	return 0;
} 



void MahonyAHRS(AttitudeSensor * sensor)
{
	if(sensor->ax_raw == sensor->ay_raw == sensor->az_raw == 0.0f)
		return;
	Quaternion q;
	q.w = sensor->Ori.w;
	q.x = sensor->Ori.x;
	q.y = sensor->Ori.y;
	q.z = sensor->Ori.z; 
	 
	Vector3D EstimateG;
	
    EstimateG.x = 2 * (q.x * q.z - q.w * q.y);
    EstimateG.y = 2 * (q.w * q.x + q.y * q.z);
    EstimateG.z = q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z;
	
	
	Vector3D raw_Acc, raw_Mag, raw_Gyr;
	raw_Acc.x = sensor->ax_raw; 
	raw_Acc.y = sensor->ay_raw; 
	raw_Acc.z = sensor->az_raw;
	raw_Mag.x = sensor->mx_raw;
	raw_Mag.y = sensor->my_raw;
	raw_Mag.z = sensor->mz_raw;
	raw_Gyr.x = sensor->gx_raw;
	raw_Gyr.y = sensor->gy_raw;
	raw_Gyr.z = sensor->gz_raw;
	
	Vector3D acc_norm = getNormalized_v3(&raw_Acc);
	Vector3D mag_norm = getNormalized_v3(&raw_Mag);
    
	float DotVal = Dot(&EstimateG, &acc_norm);

// Reference direction of Earth's magnetic feild
    Vector3D h = rotVbyQ(&raw_Mag, &q);
    float b2 = sqrtf(h.x *h.x  + h.y *h.y );
    float b4 = h.z ;
 // Estimated direction of gravity and magnetic field
    Vector3D v,w;
	float x,y,z;
    x = 2 * (q.x * q.z - q.w * q.y);
    y = 2 * (q.w * q.x + q.y * q.z);
    z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;
    
	v.x = x;
	v.y = y;
	v.z = z;
    x = 2 * b2 * (0.5f - q.y*q.y - q.z*q.z) + 2 * b4 *(q.x * q.z - q.w * q.y);
    y = 2 * b2 * (q.x * q.y - q.w * q.z) + 2 * b4 * (q.w * q.x + q.y * q.z);
    z = 2 * b2 * (q.w * q.y + q.x * q.z) + 2 * b4 * (0.5f - q.x*q.x - q.y * q.y);
    w.x = x;
	w.y = y;
	w.z = z;
//Error is sum of cross product between estimated direction and measured direction of fields
	Vector3D accErr, magErr, err;
	accErr = Cross(&acc_norm, &v);
	magErr = Cross(&mag_norm, &w);
	err.x = sensor->AccWeight * accErr.x + magErr.x;
	err.y = sensor->AccWeight * accErr.y + magErr.y;
	err.z = sensor->AccWeight * accErr.z + magErr.z;

	float Kp = 2.0f;
	
	// Apply feedback terms
	Vector3D revised_gyr;
	revised_gyr.x = raw_Gyr.x + Kp * err.x;
	revised_gyr.y = raw_Gyr.y + Kp * err.y;
	revised_gyr.z = raw_Gyr.z + Kp * err.z;
	 
	//Compute rate of change of quaternion
    Quaternion qDot, tmp1,tmp2,q_gyr;
	q_gyr.w = 0.0f;
	q_gyr.x = revised_gyr.x;
	q_gyr.y = revised_gyr.y;
	q_gyr.z = revised_gyr.z;
	
	tmp1.w = 0.5f * q.w;
	tmp1.x = 0.5f * q.x;
	tmp1.y = 0.5f * q.y;
	tmp1.z = 0.5f * q.z;
	qDot = quaternionMult(&tmp1, &q_gyr); 
// Integrate to yield quaternion	
	tmp2.w = q.w + qDot.w * sensor->SamplePeriod;
	tmp2.x = q.x + qDot.x * sensor->SamplePeriod;
	tmp2.y = q.y + qDot.y * sensor->SamplePeriod;
	tmp2.z = q.z + qDot.z * sensor->SamplePeriod;
 
    q = getNormalized_q4(&tmp2);
	
	sensor->Ori.w = q.w;
	sensor->Ori.x = q.x;
	sensor->Ori.y = q.y;
	sensor->Ori.z = q.z;
	
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
 