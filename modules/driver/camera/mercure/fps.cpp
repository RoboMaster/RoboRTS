//--------------------------------------------------------------------------------
/**
\file     Fps.h
\brief     CFps   Class declaration file
			
\version  v1.1.1301.9071
\date     2013-01-07
\author   Li  Yanjie

<p>Copyright (c) 2012-2013  China Daheng Group, Inc. Beijing Image
Vision Technology Branch and all right reserved.</p>
*/
//----------------------------------------------------------------------------------

#include <stdio.h>
#include "fps.h"

//----------------------------------------------------------------------------------
/**
\brief  Constructor
*/
//----------------------------------------------------------------------------------
CLock::CLock()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&m_mtxObject, &attr);
    pthread_mutexattr_destroy(&attr);
}

//----------------------------------------------------------------------------------
/**
\brief  Destructor
*/
//----------------------------------------------------------------------------------
CLock::~CLock()
{
    pthread_mutex_destroy(&m_mtxObject);
}

//----------------------------------------------------------------------------------
/**
\brief  Try to lock
*/
//----------------------------------------------------------------------------------
bool CLock::TryLock()
{
    const int err = pthread_mutex_trylock(&m_mtxObject);
    if (err == 0)
        return true;
    else
        return false;
}

//----------------------------------------------------------------------------------
/**
\brief  Lock
*/
//----------------------------------------------------------------------------------
void CLock::Lock()
{
    const int err = pthread_mutex_lock(&m_mtxObject);
    if (err != 0)
        printf("pthread_mutex_lock error!");
}

//----------------------------------------------------------------------------------
/**
\brief  Unlocked
*/
//----------------------------------------------------------------------------------
void CLock::Unlock()
{
    const int err = pthread_mutex_unlock(&m_mtxObject);
    if (err != 0)
        printf("pthread_mutex_unlock error!");
}

#define BASE_TIMES 1000.0
#define BASE_TIMEUS 1000000

//--------------------------------------------------------------------------------
/**
\brief    Constructor
*/
//----------------------------------------------------------------------------------

CDispTime::CDispTime(void)
{
}

//--------------------------------------------------------------------------------
/**
\brief    Destructor
*/
//----------------------------------------------------------------------------------

CDispTime::~CDispTime(void)
{
}


//--------------------------------------------------------------------------------
/**
\brief    Start the timer
*/
//----------------------------------------------------------------------------------

void CDispTime::Start(void)
{
    clock_gettime(CLOCK_MONOTONIC,&tvBegin );
}

//--------------------------------------------------------------------------------
/**
\brief    Start timing again
*/
//----------------------------------------------------------------------------------

void CDispTime::Restart(void)
{
    this->Start();
}

//--------------------------------------------------------------------------------
/**
\brief  Gets the time from Start or Restart
\param  The time in milliseconds
*/
//----------------------------------------------------------------------------------
double CDispTime::Elapsed(void)
{
    clock_gettime(CLOCK_MONOTONIC,&tvEnd );
    return ((tvEnd.tv_sec * BASE_TIMES + tvEnd.tv_nsec /BASE_TIMEUS ) - (tvBegin.tv_sec * BASE_TIMES + tvBegin.tv_nsec /BASE_TIMEUS)) ;
}

//----------------------------------------------------------------------------------
/**
\brief  Constructor
*/
//----------------------------------------------------------------------------------

CFps::CFps(void)
{
    //Reset all parameters
    this->Reset();
}

//----------------------------------------------------------------------------------
/**
\brief  Destructor
*/
//----------------------------------------------------------------------------------

CFps::~CFps(void)
{
}

//----------------------------------------------------------------------------------
/**
\brief  Get the last frame rate
\param  pFrame  Current frame image
*/
//----------------------------------------------------------------------------------
double CFps::GetFps(void) 
{
     CAutoLock objAutoLock(m_objLock);

    //Returns the current frame rate
    return m_dCurrentFps;
} 


//----------------------------------------------------------------------------------
/**
\brief  Increase the number of frames
*/
//----------------------------------------------------------------------------------

void CFps::IncreaseFrameNum(void)
{
     CAutoLock objAutoLock(m_objLock);

    //Cumulative number of frames
    m_nTotalFrameCount++;

    //Increase the number of frames
    m_nFrameCount++;

    //Update the time interval
    m_dEndTime = m_objTime.Elapsed();
}


//----------------------------------------------------------------------------------
/**
\brief  Update frame rate
         If the function is called more than the frame frequency, the frame rate will be reduced to zero
\return Updated frame rate
*/
//----------------------------------------------------------------------------------
void CFps::UpdateFps(void)
{
    CAutoLock objAutoLock(m_objLock);
   
    //Calculate the time interval
    double dInterval = m_dEndTime - m_dBeginTime;
    //modified  by gomo 2016-04-12
    //Time interval is greater than zero (with new frame)
    if(dInterval > 0.000001)
    {
        m_dFps = 1000.0 * m_nFrameCount / dInterval;
        m_nFrameCount   = 0;              //Clear The totol number of frames
        m_dBeginTime   = m_dEndTime;      //Update the start time
        
        m_dCurrentFps = m_dFps;
    }
    else if(dInterval <= 0.000001) //The time interval is equal to zero (no new frame)
    {
        //If the last frame rate is nonzero, update the frame rate
        if(m_dCurrentFps > 0.000001)
        {
            //The time from previous frame to current frame(milliseconds)
            double dCurrentInterval     = m_objTime.Elapsed() - m_dBeginTime;  

            //Update the frame rate based on the current frame rate
            double dPeriod              = 1000.0 / m_dCurrentFps;   //Last frame period (ms)
            const double RATIO          = 1.5;                      //The rate ratio which extra the frame period time will determin the frame rate to update
            double dThresh              = RATIO * dPeriod;          //The time determine the frame rate to update
            
            //If there is no frame for more than 2 seconds, the frame rate will drop to zero.
            const double ZERO_FPS_INTERVAL   = 2000.0;
            if(dCurrentInterval > ZERO_FPS_INTERVAL)
            {
                m_dCurrentFps = 0.0;
            }
            //If the frames does not grab within 2 seconds which the frame period extra 1.5 ratio, the frame rate will be reduced
            else if(dCurrentInterval > dThresh) 
            {
                m_dCurrentFps = 1000.0 / dCurrentInterval;
            }
            else{}
        }
        else{}
    }
    else{}

}
