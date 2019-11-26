//
//  NexDomeV3.h
//
//  Created by Rodolphe Pineau on 8/11/2019.
//  NexDome X2 plugin for V3 firmware

#ifndef __NEXDOME__
#define __NEXDOME__

// standard C includes
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <memory.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <ctype.h>
#ifdef SB_MAC_BUILD
#include <unistd.h>
#endif
// C++ includes
#include <string>
#include <vector>
#include <sstream>
#include <iostream>

// SB includes
#include "../../licensedinterfaces/sberrorx.h"
#include "../../licensedinterfaces/serxinterface.h"
#include "../../licensedinterfaces/sleeperinterface.h"
#include "../../licensedinterfaces/loggerinterface.h"

#include "StopWatch.h"

#define DRIVER_VERSION      1.07

#define SERIAL_BUFFER_SIZE 256
#define MAX_TIMEOUT 1000
#define PLUGIN_LOG_BUFFER_SIZE 256

#define CMD_WAIT_INTERVAL	50
#define CMD_RESP_READ_TIMEOUTS  5

// #define PLUGIN_DEBUG 2

// error codes
// Error code
enum NexDomeErrors {PLUGIN_OK=0, CMD_PROC_DONE, NOT_CONNECTED, PLUGIN_CANT_CONNECT, PLUGIN_BAD_CMD_RESPONSE, COMMAND_FAILED};
enum NexDomeShutterState {OPEN = 0, CLOSED, OPENING, CLOSING, IDLE, SHUTTER_ERROR };
enum HomeStatuses {NEVER_HOMED = 0, HOMED, ATHOME};
// RG-11
enum RainSensorStates {RAINING= 0, NOT_RAINING};

class CNexDomeV3
{
public:
    CNexDomeV3();
    ~CNexDomeV3();

    int         Connect(const char *pszPort);
    void        Disconnect(void);
    const bool  IsConnected(void) { return m_bIsConnected; }

    void        setSerxPointer(SerXInterface *p) { m_pSerx = p; }
    void        setSleeprPinter(SleeperInterface *p) {m_pSleeper = p; }

    // Dome commands
    int syncDome(double dAz, double dEl);
    int parkDome(void);
    int unparkDome(void);
    int gotoAzimuth(double dNewAz);
    int openShutter();
    int closeShutter();
    int getFirmwareVersion(char *szVersion, int nStrMaxLen);
    int getFirmwareVersion(double &fVersion);
    int goHome();

    // command complete functions
    int isGoToComplete(bool &bComplete);
    int isOpenComplete(bool &bComplete);
    int isCloseComplete(bool &bComplete);
    int isParkComplete(bool &bComplete);
    int isUnparkComplete(bool &bComplete);
    int isFindHomeComplete(bool &bComplete);

    int abortCurrentCommand();

    // getter/setter
    int getNbTicksPerRev();
    int setNbTicksPerRev(int nSteps);

    void setShutterPresent(bool bPresent);
	int getShutterStepsRange();
	int setShutterStepsRange(int nSteps);

    
    int getBatteryLevel();

    double getHomeAz();
    int setHomeAz(double dAz);

    double getParkAz();
    int setParkAz(double dAz);

    double getCurrentAz();
    double getCurrentEl();

    int getCurrentShutterState();
    int getShutterVolts(double &dShutterVolts);
    
    int getRainSensorStatus(int &nStatus);

    int getRotationSpeed(int &nSpeed);
    int setRotationSpeed(int nSpeed);

    int getRotationAcceleration(int &nAcceleration);
    int setRotationAcceleration(int nAcceleration);

    int getShutterSpeed(int &nSpeed);
    int setShutterSpeed(int nSpeed);

    int getShutterAcceleration(int &nAcceleration);
    int setShutterAcceleration(int nAcceleration);
    
    int getRotatorStepPos(int &nPos);

    int getRotatorDeadZone(int &nDeadZoneSteps);
    int setRotatorDeadZone(int &nDeadZoneSteps);

    int saveParamToEEProm();
    int loadParamFromEEProm();
    int resetToFactoryDefault();
    
    void setHomeOnPark(const bool bEnabled);
    void setHomeOnUnpark(const bool bEnabled);

protected:
    
	int             domeCommand(const char *cmd, char *result, int resultMaxLen);
    int             readResponse(char *respBuffer, int nBufferLen, int nTimeout = MAX_TIMEOUT);
	int				processResponse(char *szResp, char *pszResult, int nResultMaxLen);
    int             processAsyncResponses();
    
    int             getDomeAz(double &dDomeAz);
    int             getDomeEl(double &dDomeEl);
    int             getDomeHomeAz(double &dAz);
    int             getDomeParkAz(double &dAz);
    int             getShutterState(int &nState);
    int             getDomeStepPerRev(int &nStepPerRev);
    int             setDomeStepPerRev(int nStepPerRev);

    int             getShutterSteps(int &nStepPerRev);
    int             setShutterSteps(int &nStepPerRev);

    bool            isDomeMoving();
    bool            isDomeAtHome();
    
    
    int             parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator);

    std::string&    trim(std::string &str, const std::string &filter );
    std::string&    ltrim(std::string &str, const std::string &filter);
    std::string&    rtrim(std::string &str, const std::string &filter);
    std::string     findField(std::vector<std::string> &svFields, const std::string& token);

    
    SerXInterface   *m_pSerx;
    SleeperInterface *m_pSleeper;

    bool            m_bIsConnected;
    bool            m_bHomed;
    bool            m_bParked;
    bool            m_bShutterOpened;
	bool			m_bDomeIsMoving;

    int             m_nNbStepPerRev;
    double          m_dShutterBatteryVolts;
    double          m_dHomeAz;
    double          m_dParkAz;
    
    bool            m_bShutterPresent;
    int             m_nShutterSteps;
	int				m_nCurrentShutterCmd;

    double          m_dCurrentAzPosition;
    int             m_nCurrentRotatorPos;
    
    double          m_dCurrentElPosition;
    int             m_nCurrentShutterPos;

    double          m_dGotoAz;

    double          m_fVersion;

    char            m_szFirmwareVersion[SERIAL_BUFFER_SIZE];
    int             m_nShutterState;
    bool            m_bShutterOnly; // roll off roof so the arduino is running the shutter firmware only.
    char            m_szLogBuffer[PLUGIN_LOG_BUFFER_SIZE];

    int             m_nIsRaining;
    bool            m_bParking;
    bool            m_bUnParking;
    
    bool            m_bHomeOnPark;
    bool            m_bHomeOnUnpark;

    double          m_dShutterVolts;

	int				m_nRotationDeadZone;
	
	CStopWatch		m_cmdDelayCheckTimer;
#ifdef PLUGIN_DEBUG
    std::string m_sLogfilePath;
    // timestamp for logs
    char *timestamp;
    time_t ltime;
    FILE *Logfile;	  // LogFile
#endif

};

#endif
