//--------------------------------------------------------------------------------
/**
\file     Fps.h
\brief    CFps   Class declaration file
			
\version  v1.1.1301.9071
\date     2013-01-07
\author   Li  Yanjie

<p>Copyright (c) 2012-2013  China Daheng Group, Inc. Beijing Image
Vision Technology Branch and all right reserved.</p>
*/
//----------------------------------------------------------------------------------

#ifndef FPS_H
#define FPS_H

#include <pthread.h>
#include <unistd.h>
#include <semaphore.h>
#include <memory.h>

//-----------------------------------------------------------------
// CLock
//-----------------------------------------------------------------
/**
\brief A lock class
*/
//-----------------------------------------------------------------
class CLock
{
public:
    //! Constructor
    CLock();

    //! Destructor
    ~CLock();

    //! Try to enter the critical section, if success,returns true
    bool TryLock();

    //! Enter the critical section (may block)
    void Lock();

    //! Leave the critical section
    void Unlock();

private:
    //! no copy constructor
    CLock(const CLock&);

    //! no assignment operator
    CLock& operator=(const CLock&);

protected:
    pthread_mutex_t m_mtxObject;
};


//-----------------------------------------------------------------
// AutoLock
//-----------------------------------------------------------------
class CAutoLock
{
    CLock& m_Lock;
public:
    CAutoLock(CLock& lock)
        : m_Lock(lock)
    {
        m_Lock.Lock();
    }

    ~CAutoLock()
    {
        m_Lock.Unlock();
    }

private:
    CAutoLock& operator=(const CAutoLock&);
    CAutoLock(const CAutoLock&);
};

//--------------------------------------------------------------------------------
/**
\brief     CDispTime implements cross-platform timing.

           CDispTime have the similar high timing accuracy with CPU clock in the windows ,
           it's about 10ms in Linux.

\author
*/
//----------------------------------------------------------------------------------

class CDispTime
{
public:
    ///Constructor
    CDispTime(void);

    ///Destructor
    ~CDispTime(void);

    ///Start the timer
    void Start(void);

    ///Restart timing again
    void Restart(void);

    ///Gets the time from Start or Restart
    double Elapsed(void) ;

private:
    struct timespec  tvBegin;
    struct timespec  tvEnd;
};

//--------------------------------------------------------------------------------
/**
\file     The CFps class is used to calculate the frame rate.
			
                  CFps are thread safe.

\author
*/
//----------------------------------------------------------------------------------

class CFps
{
public:
        ///Constructor
	CFps(void);

        ///Destructor
	~CFps(void);	

    ///Get the last frame rate
    double GetFps(void);

    ///Gets the total frame count
    inline size_t GetTotalFrameCount(void); 

        ///Increase the number of frames
	void IncreaseFrameNum(void);

    ///Update frame rate
    void UpdateFps(void);

    ///Reset the timer to its original state
    inline void Reset(void);

    ///Reset frame rate
    inline void ResetFPS(void);

    ///Reset the total number of frames
    inline void ResetTotalFrameCount(void);

private:
        size_t    m_nFrameCount;        ///< The number of frames accumulated from the last calculation

    double    m_dBeginTime;         ///< The time of a frame before the first frame (initially 0)
    double    m_dEndTime;           ///< The last frame of time

        double    m_dFps;               ///< The frame rate (frames per second) from the ratio of the number of frames to the time interval,
    double    m_dCurrentFps;        ///< The current frame rate may be predicted (frame / s)
        size_t    m_nTotalFrameCount;   ///< The number of frames accumulated


        CDispTime m_objTime;            ///< Timer

        CLock m_objLock;       ///< Protect the data lock
};

//----------------------------------------------------------------------------------
/**
\brief  Gets the total frame count
\param  pFrame  Current frame image
*/
//----------------------------------------------------------------------------------
inline size_t CFps::GetTotalFrameCount(void) 
{
    CAutoLock objAutoLock(m_objLock);
    return m_nTotalFrameCount;
} 

//----------------------------------------------------------------------------------
/**
\brief  Reset frame rate
\return void
*/
//----------------------------------------------------------------------------------
inline void CFps::ResetFPS(void)
{
    CAutoLock objAutoLock(m_objLock); //modified  by gomo 2016-04-12
    m_nFrameCount       = 0;
    m_dBeginTime        = 0.0;
    m_dEndTime          = 0.0;
    m_dFps              = 0.0;
    m_dCurrentFps       = 0.0;
    m_objTime.Start();          //Restart timer
}

//----------------------------------------------------------------------------------
/**
\brief  Reset the total number of frames
\return void
*/
//----------------------------------------------------------------------------------
inline void CFps::ResetTotalFrameCount(void)
{
    CAutoLock objAutoLock(m_objLock);
    m_nTotalFrameCount  = 0;
}

//----------------------------------------------------------------------------------
/**
\brief  Reset the timer to its original state
*/
//----------------------------------------------------------------------------------

inline void CFps::Reset(void)
{
    ResetFPS();
    ResetTotalFrameCount();
}

#endif //FPS_H
