#include "msm_sensor.h"
#include "oppo_feature.h"

enum{
    THR_CASE = 0,
    STABLE_CASE
};
#define m9mo_caf_debug(fmt, arg...) //printk(fmt, ##arg);

static u_int32_t mPreFv;
static u_int32_t uPreWB;
static u_int32_t uPreBright;

static bool is_moving = false;

static struct oppo_interface* caf_i;

static u_int8_t getOrder(u_int32_t wd_value)
{
    u_int8_t order = 0;
	
    while(wd_value != 0){
       wd_value = wd_value/10;
       order++;
    }
    return order; 
}
static bool m9mo_check_if_do_caf(struct frame_info_t *frame_info, u_int32_t mode)
{
	bool result = false;

    u_int32_t curFv = 0;
    int32_t delta  = 0;  
    int32_t doCAFthr = 0;
    int32_t doCAFthrLow = 0;
    int32_t uCurBrightness = 0;
    int32_t uCurWB = 0;
    int32_t uWhiteBalance = 0;
    int32_t uBrightness = 0;
	int32_t uGravityX = 0;
	int32_t uGravityY = 0;
	int32_t uGravityZ = 0;
	
	static int gravity_data_pre[3];

	curFv = frame_info->wave_detect;
	uCurWB = frame_info->wb;
	uCurBrightness = frame_info->brightness;
	
    if(getOrder(curFv) < 5 || getOrder(curFv) > 6)
	{
		m9mo_caf_debug("WaveDetect Value is not valid, not use it \r\n");
		return false;
    }

    delta = abs(mPreFv - curFv);
	uWhiteBalance = abs(uCurWB - uPreWB);
	uBrightness = abs(uCurBrightness - uPreBright);

	if (frame_info->gravity_valid)
	{
		uGravityX = abs(frame_info->gravity_data[0]-gravity_data_pre[0]);
		uGravityY = abs(frame_info->gravity_data[1]-gravity_data_pre[1]);
		uGravityZ = abs(frame_info->gravity_data[2]-gravity_data_pre[2]);
	}
	
    switch(mode)
	{
		case THR_CASE:
		{
			m9mo_caf_debug("THR_CASE-----start \r\n");
			if (getOrder(mPreFv) == 5)
			{
				doCAFthr = mPreFv/20;
			}
			else if (getOrder(mPreFv) == 6)
			{
				doCAFthr = mPreFv/30;
			}

			m9mo_caf_debug("delta = %d , doCAFthr = %d \r\n", delta, doCAFthr);
			
			if(delta > doCAFthr)
			{
				m9mo_caf_debug("THR_CASE  true \n");
				result = true;
			}
			else 
			{
				result = false;
			}
			
			//avoid focus continous at some scenes
			if (result == true)
			{
				m9mo_caf_debug("check the brightness and wb------thread \r\n");
				//m9mo_caf_debug("uBrightness---[%d], uCurBrightness----[%d] \r\n", uBrightness, uCurBrightness);
				//m9mo_caf_debug("uWhiteBalance---[%d], uCurWB----[%d] \r\n", uWhiteBalance, uCurWB);
				if ((uBrightness <10 && uCurBrightness > 0)
					&& (uWhiteBalance < 10 && uCurWB > 0))
				{
					m9mo_caf_debug("the brightness vare not strongly, do not focus \r\n");
					result = false;
				}
				if (uGravityX<5 && uGravityY<5 && uGravityZ<5 && frame_info->gravity_valid
					&& (frame_info->gravity_data[0]<90 || frame_info->brightness == 0))
				{
					m9mo_caf_debug("The phone is not moving \r\n");
					result = false;
				}
			}
			else
			{
				if ((uGravityX>30 || uGravityY>30 || uGravityZ>30) 
					&& frame_info->gravity_valid)
				{
					m9mo_caf_debug("The gravity data vare strongly, do focus \r\n");
					result = true;
				}
			}
			break;
        }
        case STABLE_CASE:
		{
			m9mo_caf_debug("STABLE_CASE-----start \r\n");
			
			//if gyro not work, detect by WaveDetect Value
			if (getOrder(mPreFv) == 5)
			{
				doCAFthrLow = mPreFv/80;
			}
			else if (getOrder(mPreFv) == 6)
			{
				doCAFthrLow = mPreFv/400;
			}

			if(WDV_CAF_LOW_THR <= mPreFv/400) 
			{
				doCAFthrLow = mPreFv/600;
			}
			else
			{
				doCAFthrLow = WDV_CAF_LOW_THR;
			}
			
			m9mo_caf_debug("delta = %d , doCAFthrLow = %d, mPreFv = %d \r\n", delta, doCAFthrLow, mPreFv);
			if(delta > doCAFthrLow)
			{
				result = false;
			} 
			else 
			{
				m9mo_caf_debug("STABLE_CASE true ^^^^^^^^^^^^^^^\n");
				result = true;
			}

			//avoid focus when moving sometimes
			if (result == false)
			{
				m9mo_caf_debug("check the brightness and wb-----stable\r\n");
				//m9mo_caf_debug("[stable] uBrightness---[%d], uCurBrightness----[%d] \r\n", uBrightness, uCurBrightness);
				//m9mo_caf_debug("[stable] uWhiteBalance---[%d], uCurWB----[%d] \r\n", uWhiteBalance, uCurWB);
				
				if ((uBrightness < 10 && uCurBrightness > 0) && 
					(uWhiteBalance < 10 && uCurWB > 0))
				{
					m9mo_caf_debug("the brightness and WB is stable, do focus \r\n");
					result = true;
				}
			}
			if (result)
			{
				//check the gravity data if change
				if (uGravityX<3 && uGravityY<3 && uGravityZ<3 && frame_info->gravity_valid)
				{
					m9mo_caf_debug("The phone is not moving \r\n");
					is_moving = false;
					result = false;
				}
				else
				{
					is_moving = true;
				}

				//at low light scene, check wb value and gravity value
				if ((frame_info->brightness == 0) &&
					((uGravityX<5 && uGravityY<5 && uGravityZ<5) || 
					(uWhiteBalance < 30)))
				{
					result = false;		
				}

				//check current ae if stable
				if (frame_info->ae_stable == false)
					result = false;
			}
			break;
		}
        default:
            break;
    }
 
	mPreFv = curFv;
	uPreWB = uCurWB;
	uPreBright = uCurBrightness;

	if (frame_info->gravity_valid)
	{
		gravity_data_pre[0] = frame_info->gravity_data[0];
		gravity_data_pre[1] = frame_info->gravity_data[1];
		gravity_data_pre[2] = frame_info->gravity_data[2];
	}

    return result;
}
static bool m9mo_detect_caf_for_first_focus(struct frame_info_t *frame_info)
{
	bool bTriggered = false;
	u_int32_t u4CurBrightness = 0;
	u_int32_t u4CurWB = 0;
	u_int32_t u4DiffBrightness = 0;
	u_int32_t u4DiffWB = 0;
	static u_int32_t u4PreBrightness = 0;
	static u_int32_t u4PreWB = 0;

	u4CurBrightness = frame_info->brightness;
	u4CurWB = frame_info->wb;

	u4DiffBrightness = abs(u4CurBrightness - u4PreBrightness);
	u4DiffWB = abs(u4CurWB - u4PreWB);
	m9mo_caf_debug("u4DiffBrightness [%d], u4PreBrightness [%d] \r\n", u4DiffBrightness,u4PreBrightness);
	m9mo_caf_debug("u4DiffWB [%d], u4PreWB [%d] \r\n", u4DiffWB, u4PreWB);
	if ((u4DiffBrightness * 100 > u4PreBrightness * 10)
		|| ((u4DiffWB * 100 > u4PreWB * 3) && (u4DiffWB > 50))
		)
	{
		m9mo_caf_debug("[%s]------------trigger \r\n", __func__);
		bTriggered = true;
	}
	else
	{
		m9mo_caf_debug("[%s]------------not trigger \r\n", __func__);
		bTriggered = false;
	}

	u4PreBrightness = u4CurBrightness;
	u4PreWB = u4CurWB;

	return bTriggered;
	
}

static void m9mo_do_caf(struct msm_sensor_ctrl_t *s_ctrl,
	struct frame_info_t *frame_info)
{
	static bool mCAFflag = false;
	static bool mmIsCAFin = false;
	static bool bWaveDetectValid = false;
	static bool bNeedDelay = false;
	static int32_t mdelayDetectCnt = 0;
	static int32_t mdelayAFCnt = 0;
	static int32_t mdelayFrameCnt = 0;

	//when start monitor, must do one time focus,the WD value can valid
	if (frame_info->wd_valid == false)
		bWaveDetectValid = false;
	else
		bWaveDetectValid = true;
	if (bWaveDetectValid == false)
	{
		m9mo_caf_debug("Detect by WB and Bright \r\n");
		if (m9mo_detect_caf_for_first_focus(frame_info))
		{
			m9mo_caf_debug("WaveDetect------trigger \r\n");
			caf_i->notify(s_ctrl, is_moving);
			bWaveDetectValid = true;
			mdelayFrameCnt = 20;
		}
		else
		{
			m9mo_caf_debug("WaveDetect------not trigger \r\n");
		}
		return;	
	}
	
	if (frame_info->capture_start)
	{
		m9mo_caf_debug("Now is at capture, not do CAF \r\n");
		bNeedDelay = true;
		mdelayFrameCnt = 20;
		return;
	}
	
	//delay some frames not to detect after capture
	if (bNeedDelay)
	{
		m9mo_caf_debug("After capture, need delay some frames \r\n");
		
		if (mdelayFrameCnt > 0)
		{
			mdelayFrameCnt--;

			//if not stable and WD value vare out of thread, do focus
			if (m9mo_check_if_do_caf(frame_info, THR_CASE))
			{
				caf_i->notify(s_ctrl, is_moving);
				bNeedDelay = false;
				bWaveDetectValid = true;
				mdelayFrameCnt = 0;
			}
		}
		else
		{
			bNeedDelay = false;
		}
		return;
	}
	
	//after check bright and WB trigger focus, can delay some frames
	if (bWaveDetectValid == true && mdelayFrameCnt > 0)
	{
		mPreFv = frame_info->wave_detect;
		uPreBright = frame_info->brightness;
		uPreWB = frame_info->wb;
		mdelayFrameCnt--;
		return;
	}
	
	if(!mmIsCAFin)
	{
		mCAFflag = m9mo_check_if_do_caf(frame_info, THR_CASE);
		if(mCAFflag)
		{
			mdelayAFCnt = DO_AF_DELAY;
			mmIsCAFin = true;
		}
	}
	
	if(mCAFflag)
	{
		m9mo_caf_debug("mdelayAFCnt = %d \r\n", mdelayAFCnt);
		if((mdelayAFCnt < 1) && m9mo_check_if_do_caf(frame_info, STABLE_CASE)) 
		{
			caf_i->notify(s_ctrl, is_moving);
			mCAFflag = false;
			mdelayDetectCnt = ANTI_WDV_SHAKE_DELAY;
		}
		
		mdelayAFCnt--;
		
	}
	
	if(mdelayDetectCnt > 0)
	{
		mdelayDetectCnt--;
	} 
	else if(mdelayDetectCnt == 0)
	{
		mPreFv = frame_info->wave_detect;
		uPreBright = frame_info->brightness;
		uPreWB = frame_info->wb;
		mmIsCAFin = false;
		mdelayDetectCnt--;
	}	
}

int32_t oppo_caf_init(struct oppo_interface *interface)
{
    caf_i =  interface;
    //reg process
	if(caf_i)
	  caf_i->process = m9mo_do_caf;

    return 0; 
}