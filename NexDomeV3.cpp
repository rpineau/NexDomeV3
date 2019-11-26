//
//  NexDomeV3.cpp
//
//  Created by Rodolphe Pineau on 8/11/2019.
//  NexDome X2 plugin for V3 firmware


#include "NexDomeV3.h"

CNexDomeV3::CNexDomeV3()
{
    // set some sane values
    m_pSerx = NULL;
    m_bIsConnected = false;
    m_bShutterPresent = false;
    
    m_nNbStepPerRev = 0;
    m_nShutterSteps = 0;
    m_dShutterBatteryVolts = 0.0;
    
    m_dHomeAz = 0;

	m_nCurrentRotatorPos = 0;
	
    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

	m_bDomeIsMoving = false;

    m_bShutterOpened = false;
	m_nCurrentShutterCmd = IDLE;

    m_bParked = true;
    m_bHomed = false;

    m_fVersion = 0.0;

    m_nIsRaining = NOT_RAINING;
    m_bParking = false;
    m_bUnParking = false;
    
    m_dShutterVolts = -1.0;
    
    m_bHomeOnPark = false;
    m_bHomeOnUnpark = false;

	m_nRotationDeadZone = 0;

    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,PLUGIN_LOG_BUFFER_SIZE);

	m_cmdDelayCheckTimer.Reset();

#ifdef PLUGIN_DEBUG
#if defined(SB_WIN_BUILD)
    m_sLogfilePath = getenv("HOMEDRIVE");
    m_sLogfilePath += getenv("HOMEPATH");
    m_sLogfilePath += "\\NexDomeV3Log.txt";
#elif defined(SB_LINUX_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/NexDomeV3Log.txt";
#elif defined(SB_MAC_BUILD)
    m_sLogfilePath = getenv("HOME");
    m_sLogfilePath += "/NexDomeV3Log.txt";
#endif
    Logfile = fopen(m_sLogfilePath.c_str(), "w");
#endif

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::CNexDomeV3] Version %3.2f build 2019_11_25_1940.\n", timestamp, DRIVER_VERSION);
    fprintf(Logfile, "[%s] [CNexDomeV3] Constructor Called.\n", timestamp);
    fflush(Logfile);
#endif

}

CNexDomeV3::~CNexDomeV3()
{
#ifdef	PLUGIN_DEBUG
    // Close LogFile
    if (Logfile) fclose(Logfile);
#endif

}

int CNexDomeV3::Connect(const char *pszPort)
{
    int nErr;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDomeV3::Connect Called %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // 115200 8N1
    nErr = m_pSerx->open(pszPort, 115200, SerXInterface::B_NOPARITY, "-DTR_CONTROL 1");
    if(nErr) {
        m_bIsConnected = false;
        return nErr;
    }
    m_bIsConnected = true;
	m_bDomeIsMoving = false;
    m_bHomed = false;
    m_bParking = false;
    m_bUnParking = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDomeV3::Connect connected to %s\n", timestamp, pszPort);
    fflush(Logfile);
#endif

    // the arduino take over a second to start as it need to init the XBee
    if(m_pSleeper)
        m_pSleeper->sleep(2000);
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDomeV3::Connect Getting Firmware\n", timestamp);
    fflush(Logfile);
#endif

    // if this fails we're not properly connected.
    nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::Connect] Error Getting Firmware.\n", timestamp);
        fflush(Logfile);
#endif
        m_bIsConnected = false;
        m_pSerx->close();
        return FIRMWARE_NOT_SUPPORTED;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDomeV3::Connect Got Firmware %s ( %f )\n", timestamp, m_szFirmwareVersion, m_fVersion);
    fflush(Logfile);
#endif
    if(m_fVersion < 3.0f) {
        return FIRMWARE_NOT_SUPPORTED;
    }

    nErr = getDomeStepPerRev(m_nNbStepPerRev);
    if(m_bShutterPresent)
        nErr = getShutterSteps(m_nShutterSteps);
    
    nErr = getDomeHomeAz(m_dHomeAz);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CNexDomeV3::Connect getDomeHomeAz nErr : %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    nErr = getShutterState(m_nShutterState);
    switch(m_nShutterState) {
        case OPEN :
            m_bShutterOpened = true;
            m_dCurrentElPosition = 90.0;
            break;
        case CLOSED :
            m_bShutterOpened = false;
            m_dCurrentElPosition = 0.0;
            break;
        default :
            m_bShutterOpened = false;
            m_dCurrentElPosition = 0.0;
            break;
    }

	getRotatorDeadZone(m_nRotationDeadZone);

    return SB_OK;
}


void CNexDomeV3::Disconnect()
{
    if(m_bIsConnected) {
        abortCurrentCommand();
        m_pSerx->purgeTxRx();
        m_pSerx->close();
    }
    m_bIsConnected = false;
	m_bDomeIsMoving = false;
    m_bHomed = false;
    m_bParking = false;
    m_bUnParking = false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::Disconnect] m_bIsConnected = %d\n", timestamp, m_bIsConnected);
    fflush(Logfile);
#endif
}


int CNexDomeV3::domeCommand(const char *pszCmd, char *pszResult, int nResultMaxLen)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    unsigned long  ulBytesWrite;
    int nb_timeout;
	 int dDelayMs;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

	// do we need to wait ?
	if(m_cmdDelayCheckTimer.GetElapsedSeconds()<CMD_WAIT_INTERVAL) {
		dDelayMs = CMD_WAIT_INTERVAL - int(m_cmdDelayCheckTimer.GetElapsedSeconds() *1000);
		if(dDelayMs>0)
			m_pSleeper->sleep(dDelayMs);
	}
	
    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
	m_cmdDelayCheckTimer.Reset();
    if(nErr)
        return nErr;

    nb_timeout = 0;
    
    while(true) {
        if(nb_timeout>5) { // durring a movement we get a lot of extra stuff in there
            nErr = ERR_RXTIMEOUT;
            return nErr;
        }
        // read response
        nErr = readResponse(szResp, SERIAL_BUFFER_SIZE);
        if(nErr &&  nErr != ERR_DATAOUT) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] ***** ERROR READING RESPONSE **** error = %d , response : '%s'\n", timestamp, nErr, szResp);
            fflush(Logfile);
#endif
            return nErr;
        }
        if(nErr == ERR_DATAOUT) {
            m_pSleeper->sleep(50);
            nb_timeout++;
            continue;
        }
		nErr = processResponse(szResp, pszResult, nResultMaxLen);
		if(nErr && nErr != CMD_PROC_DONE)
			return nErr;

		if(nErr == CMD_PROC_DONE)
			return PLUGIN_OK;
        continue;
    }

    return nErr;
}


int CNexDomeV3::readResponse(char *szRespBuffer, int nBufferLen, int nTimeout )
{
    int nErr = PLUGIN_OK;
    unsigned long ulBytesRead = 0;
    unsigned long ulTotalBytesRead = 0;
    char *pszBufPtr;

    memset(szRespBuffer, 0, (size_t) nBufferLen);
    pszBufPtr = szRespBuffer;

    do {
        nErr = m_pSerx->readFile(pszBufPtr, 1, ulBytesRead, nTimeout);
        if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::readResponse] readFile error\n", timestamp);
            fflush(Logfile);
#endif
            return nErr;
        }

        if (ulBytesRead !=1) {// timeout
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] CNexDomeV3::readResponse Timeout while waiting for response from controller\n", timestamp);
            fflush(Logfile);
#endif
            nErr = ERR_DATAOUT;
            break;
        }
        ulTotalBytesRead += ulBytesRead;
    } while (*pszBufPtr++ != 0x0a && ulTotalBytesRead < nBufferLen );

    if(ulTotalBytesRead)
        *(pszBufPtr-1) = 0; //remove the \n

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::readResponse] response = %s\n", timestamp, szRespBuffer);
                fflush(Logfile);
    #endif

    return nErr;
}

int CNexDomeV3::processResponse(char *szResp, char *pszResult, int nResultMaxLen)
{
	int nErr = PLUGIN_OK;
    char szTmp[SERIAL_BUFFER_SIZE];
    std::string sResp;
    std::string sTmp;

	#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
		ltime = time(NULL);
		timestamp = asctime(localtime(&ltime));
		timestamp[strlen(timestamp) - 1] = 0;
		fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] ==== START ====\n", timestamp);
		fflush(Logfile);
	#endif

	//cleanup the string
	sTmp.assign(szResp);
	sResp = trim(sTmp," \n\r#");

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] sResp = %s\n", timestamp, sResp.c_str());
        fflush(Logfile);
    #endif

    
	strncpy(szResp, sResp.c_str(), SERIAL_BUFFER_SIZE);

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] szResp[0] = %c\n", timestamp, szResp[0]);
        fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] szResp = %s\n", timestamp, szResp);
        fflush(Logfile);
    #endif
	// we got some event notification., read next response
	switch(szResp[0]) {
		case 'C' :
			if(sResp.find("CLS") != -1) {
				strncpy(pszResult, sResp.c_str(), nResultMaxLen);
				nErr = CMD_PROC_DONE;
			}
			break;
        case 'O' :
            if(sResp.find("OPS") != -1) {
                strncpy(pszResult, sResp.c_str(), nResultMaxLen);
                nErr = CMD_PROC_DONE;
            }
            break;
		case 'P' :
            if(sResp.find("PR") != -1) {
                strncpy(pszResult, sResp.c_str(), nResultMaxLen);
                nErr = CMD_PROC_DONE;
            }
            else if(sResp.find("PW") != -1) {
                strncpy(pszResult, sResp.c_str(), nResultMaxLen);
                nErr = CMD_PROC_DONE;
            }
            else {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] case 'P' rotator position update (cur pos = %d) : '%s'\n", timestamp, m_nCurrentRotatorPos, szResp);
                fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] case 'P' rotator position update (cur az = %3.2f) : '%s'\n", timestamp, m_dCurrentAzPosition, szResp);
                fflush(Logfile);
#endif
                m_nCurrentRotatorPos = atoi(szResp+1); // Pxxxxx
                // convert steps to deg
                m_dCurrentAzPosition = (double(m_nCurrentRotatorPos)/m_nNbStepPerRev) * 360.0;
				while(m_dCurrentAzPosition >= 360)
					m_dCurrentAzPosition = m_dCurrentAzPosition - 360;
				while(m_dCurrentAzPosition < 0)
					m_dCurrentAzPosition = m_dCurrentAzPosition + 360;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] case 'P' rotator position update (new pos = %d) : '%s'\n", timestamp, m_nCurrentRotatorPos, szResp);
                fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] case 'P' rotator position update (new az = %3.2f) : '%s'\n", timestamp, m_dCurrentAzPosition, szResp);
                fflush(Logfile);
#endif
                break;
            }

		case 'S' :
            if(sResp.find("SES") != -1) {
                strncpy(pszResult, sResp.c_str(), nResultMaxLen);
				nErr = CMD_PROC_DONE;
			}
            else {
                if(isdigit(szResp[1])) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] case 'S' shutter state update : '%s'\n", timestamp, szResp);
                fflush(Logfile);
#endif
                m_nCurrentShutterPos = atoi(szResp+1); // Sxxxxx
                // convert steps to deg
                if(m_nShutterSteps)
                    m_dCurrentElPosition = (double(m_nCurrentShutterPos)/m_nShutterSteps) * 360.0;
                }
            }
			break;

		case 'X' :
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] XBee status : '%s'\n", timestamp, szResp);
			fflush(Logfile);
#endif
			if(strstr(szResp, "Online")) {
				m_bShutterPresent = true;
			}
			else {
				m_bShutterPresent = false;
			}
			break;

		case ':' :
			// we got some response, parse
            if(sResp.find(":BV") != -1) {
				memcpy(szTmp, szResp+3, SERIAL_BUFFER_SIZE);
				m_dShutterVolts = float(atoi(szTmp)) * 3.0 * (5.0 / 1023.0);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
				ltime = time(NULL);
				timestamp = asctime(localtime(&ltime));
				timestamp[strlen(timestamp) - 1] = 0;
				fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] m_dShutterVolts : %3.2f\n", timestamp, m_dShutterVolts);
				fflush(Logfile);
#endif
			}
            else if(sResp.find(":RainStopped") != -1) {
				m_nIsRaining = NOT_RAINING;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
				ltime = time(NULL);
				timestamp = asctime(localtime(&ltime));
				timestamp[strlen(timestamp) - 1] = 0;
				fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] NOT_RAINING\n", timestamp);
				fflush(Logfile);
#endif
			}
            else if(sResp.find(":Rain") != -1) {
				m_nIsRaining = RAINING;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
				ltime = time(NULL);
				timestamp = asctime(localtime(&ltime));
				timestamp[strlen(timestamp) - 1] = 0;
				fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] RAINING\n", timestamp);
				fflush(Logfile);
#endif
			}
            else if(sResp.find(":left") != -1) {
                strncpy(pszResult, szResp+1, nResultMaxLen);
                nErr = CMD_PROC_DONE;
			}
            else if(sResp.find(":right") != -1) {
                strncpy(pszResult, szResp+1, nResultMaxLen);
                nErr = CMD_PROC_DONE;
			}
            else if(sResp.find(":open") != -1) {
                strncpy(pszResult, szResp+1, nResultMaxLen);
                nErr = CMD_PROC_DONE;
			}
            else if(sResp.find(":close") != -1) {
                strncpy(pszResult, szResp+1, nResultMaxLen);
                nErr = CMD_PROC_DONE;
			}
            else {
                strncpy(pszResult, szResp+1, nResultMaxLen);
				nErr = CMD_PROC_DONE;
			}
			break;

        case 'o': //onMotorStopped
            break;
            
		default :
			strncpy(pszResult, sResp.c_str(), nResultMaxLen);
			nErr = CMD_PROC_DONE;
			break;
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] nErr = '%d'\n", timestamp, nErr);
    fprintf(Logfile, "[%s] [CNexDomeV3::processResponse] ==== END ====\n", timestamp);
    fflush(Logfile);
#endif

	return nErr;
}

int CNexDomeV3::processAsyncResponses()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nbBytesWaiting = 0;
    char szTmp[SERIAL_BUFFER_SIZE];
    std::string sResp;
    std::string sTmp;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    if(m_bDomeIsMoving)
        return nErr;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::processAsyncResponses]\n", timestamp);
    fflush(Logfile);
#endif
    do {
        m_pSerx->bytesWaitingRx(nbBytesWaiting);
        if(nbBytesWaiting) {
            nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, 250);
            if(nErr && nErr != ERR_DATAOUT)
                return nErr;

            if(strlen(szResp)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::processAsyncResponses] szResp = '%s'\n", timestamp, szResp);
                fflush(Logfile);
#endif

				nErr = processResponse(szResp, szTmp, SERIAL_BUFFER_SIZE);
				if(nErr && nErr != CMD_PROC_DONE)
					return nErr;
            }
        }
    } while(nbBytesWaiting);
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::processAsyncResponses] Done\n", timestamp);
    fprintf(Logfile, "[%s] [CNexDomeV3::processAsyncResponses] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif
    
    return nErr;
}

int CNexDomeV3::getDomeAz(double &dDomeAz)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz]\n", timestamp);
    fflush(Logfile);
#endif

    
    if(m_bDomeIsMoving) {
		dDomeAz = m_dCurrentAzPosition;
		return nErr;
	}
    
    nErr = domeCommand("@PRR\r\n", szResp,  SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "PRR") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        dDomeAz = m_dCurrentAzPosition;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        dDomeAz = m_dCurrentAzPosition;
        return PLUGIN_OK;
    }

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] szResp = %s\n", timestamp, szResp);
        fflush(Logfile);
    #endif

    m_nCurrentRotatorPos = atoi(szResp+3); // PRRxxx

    // convert steps to deg
    dDomeAz = (double(m_nCurrentRotatorPos)/m_nNbStepPerRev) * 360.0;
    
    while(dDomeAz >= 360)
        dDomeAz = dDomeAz - 360;

    m_dCurrentAzPosition = dDomeAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] m_nNbStepPerRev = %d\n", timestamp, m_nNbStepPerRev);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] m_nCurrentRotatorPos = %d\n", timestamp, m_nCurrentRotatorPos);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] dDomeAz = %3.2f\n", timestamp, dDomeAz);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::getDomeEl(double &dDomeEl)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bDomeIsMoving) {
		dDomeEl = m_dCurrentElPosition;
		return nErr;
	}

    if(!m_bShutterPresent) {
        dDomeEl = m_dCurrentElPosition;
        return nErr;
    }
    
    dDomeEl = m_dCurrentElPosition;
    return nErr;
    
    /// we might use this when firmware timeouts are fixed
    nErr = domeCommand("@PRS\r\n", szResp,  SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "PRS") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        dDomeEl = m_dCurrentElPosition;
        return PLUGIN_OK;
    }

    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        dDomeEl = m_dCurrentElPosition;
        return PLUGIN_OK;
    }

    // convert steps to deg
    m_nCurrentShutterPos = atoi(szResp+3); // PRSxxx
    
    dDomeEl = (double(m_nCurrentShutterPos)/m_nNbStepPerRev) * 360.0;
    m_dCurrentElPosition = dDomeEl;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeEl] m_nCurrentShutterPos = %d\n", timestamp, m_nCurrentShutterPos);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeEl] dDomeEl = %3.2f\n", timestamp, dDomeEl);
    fflush(Logfile);
#endif
    return nErr;
}


int CNexDomeV3::getDomeHomeAz(double &dAz)
{
    
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    int nStepPos;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bDomeIsMoving) {
        dAz = m_dHomeAz;
        return nErr;
    }
    
    nErr = domeCommand("@HRR\r\n", szResp,  SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "HRR") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        dAz = m_dHomeAz;
        return PLUGIN_OK;
    }

    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        dAz = m_dHomeAz;
        return nErr;
    }

    // convert Az string to double
    nStepPos = atoi(szResp+3); // HRRxxx
    dAz = (double(nStepPos)/m_nNbStepPerRev) * 360.0;
    m_dHomeAz = dAz;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeHomeAz] nStepPos = %d\n", timestamp, nStepPos);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeHomeAz] dAz = %3.2f\n", timestamp, dAz);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeHomeAz] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::getShutterState(int &nState)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    std::vector<std::string> shutterStateFields;
    int nOpen, nClosed;
    int nb_timeout;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState]\n", timestamp);
    fflush(Logfile);
#endif

    
	if(m_bDomeIsMoving) {
		nState = m_nShutterState;
		return nErr;
	}

    if(!m_bShutterPresent) {
        nState = IDLE;
        return nErr;
    }

    nErr = domeCommand("@SRS\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "SES") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState] response = '%s'\n", timestamp, szResp);
        fflush(Logfile);
    #endif

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    if(nErr) {
        nState = m_nShutterState;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS ){
        nState = m_nShutterState;
        return PLUGIN_OK;
    }

    // need to parse :SES,-125,46000,0,0#
    nErr = parseFields(szResp, shutterStateFields, ',');
    if(nErr)
        return nErr;
    if(shutterStateFields.size()<4)
        return ERR_CMDFAILED;

    nOpen = std::stoi(shutterStateFields[3]);
    nClosed = std::stoi(shutterStateFields[4]);
    
	if(!nOpen && !nClosed && m_nCurrentShutterCmd != IDLE) {
		nState = m_nCurrentShutterCmd;
	}
	else if(!nOpen && !nClosed)
		nState = SHUTTER_ERROR;
    else if(nClosed) {
        nState = CLOSED;
    }
    else if(nOpen) {
        nState = OPEN;
    }

    m_nShutterState = nState;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState] nState = '%d'\n", timestamp, nState);
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}


int CNexDomeV3::getDomeStepPerRev(int &nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    nErr = domeCommand("@RRR\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "RRR") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    if(nErr) {
        nStepPerRev = m_nNbStepPerRev;
        return PLUGIN_OK;
    }

    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nStepPerRev = m_nNbStepPerRev;
        return PLUGIN_OK;
    }
    // RRR99498
    nStepPerRev = atoi(szResp+3);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeStepPerRev] nStepPerRev = %d\n", timestamp, nStepPerRev);
    fflush(Logfile);
#endif
    m_nNbStepPerRev = nStepPerRev;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeStepPerRev] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::setDomeStepPerRev(int nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    m_nNbStepPerRev = nStepPerRev;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@RWR,%d\r\n", nStepPerRev);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::setDomeStepPerRev] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}


int CNexDomeV3::getShutterSteps(int &nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nStepPerRev = 0;
        return nErr;
    }

    nErr = domeCommand("@RRS\r\n", szResp, SERIAL_BUFFER_SIZE);
    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "RRS") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }
    
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    if(nErr) {
        nStepPerRev = m_nShutterSteps;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nStepPerRev = m_nShutterSteps;
        return PLUGIN_OK;
    }

    nStepPerRev = atoi(szResp+3);
    m_nShutterSteps = nStepPerRev;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterSteps] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::setShutterSteps(int &nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    
    m_nShutterSteps = nStepPerRev;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@RWS,%d\r\n", nStepPerRev);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::setShutterSteps] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::getShutterVolts(double &dShutterVolts)
{
    int nErr = PLUGIN_OK;

    if(!m_bShutterPresent) {
        dShutterVolts = 0;
        return nErr;
    }

    if(m_bIsConnected) {
        nErr = processAsyncResponses();
    }
    dShutterVolts = m_dShutterVolts;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterVolts] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::getRotatorDeadZone(int &nDeadZoneSteps)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@DRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "DRR") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }
    
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    if(nErr) {
        nDeadZoneSteps = 0;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nDeadZoneSteps = 0;
        return PLUGIN_OK;
    }

    nDeadZoneSteps = atoi(szResp+3);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getRotatorDeadZone] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::setRotatorDeadZone(int &nDeadZoneSteps)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@DWR,%d\r\n", nDeadZoneSteps);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::setRotatorDeadZone] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}


bool CNexDomeV3::isDomeMoving()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	int nbBytesWaiting = 0;
    int nbRespRead = 0;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isDomeMoving] In : m_bDomeIsMoving = %s\n", timestamp, m_bDomeIsMoving?"Yes":"No");
        fflush(Logfile);
    #endif
    if(!m_bDomeIsMoving) {
        return false;
    }

	do {
		m_pSerx->bytesWaitingRx(nbBytesWaiting);
		if(nbBytesWaiting ) {
			nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, 250);
			if(nErr && nErr != ERR_DATAOUT)
				return m_bDomeIsMoving;
            nbRespRead++;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::isDomeMoving] nbRespRead = %d\n", timestamp, nbRespRead);
            fflush(Logfile);
#endif
			if(strlen(szResp)) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::isDomeMoving] szResp = %s\n", timestamp, szResp);
                fflush(Logfile);
#endif
				switch(szResp[0]) {
					case 'P' :
                        if(isdigit(szResp[1])) {
                            m_nCurrentRotatorPos = atoi(szResp+1); // Pxxxxx
                            // convert steps to deg
                            m_dCurrentAzPosition = (double(m_nCurrentRotatorPos)/m_nNbStepPerRev) * 360.0;
                            while(m_dCurrentAzPosition >= 360)
                                m_dCurrentAzPosition = m_dCurrentAzPosition - 360;
                            while(m_dCurrentAzPosition < 0)
                                m_dCurrentAzPosition = m_dCurrentAzPosition + 360;
                        } else if (szResp[1]==':') {
                            if(strstr(szResp+2,"SER")) {
                                m_bDomeIsMoving = false;
                            }
                            else if(strstr(szResp+2,"SES")) {
                                m_bDomeIsMoving = false;
                            }
                            break;
                        }
						break;
                    case 'S' :
                        if(isdigit(szResp[1])) {
                            m_nCurrentShutterPos = atoi(szResp+1);
                            if(m_nShutterSteps)
                                m_dCurrentElPosition = (double(m_nCurrentShutterPos)/m_nShutterSteps) * 360.0;
                        }
                        break;
					case ':' :
                        // :SER or:SES is sent at the end of the move-> parse :SER,0,0,55080,0,300#
						if(strstr(szResp,"SER")) {
							m_bDomeIsMoving = false;
						}
                        else if(strstr(szResp,"SES")) {
                            m_bDomeIsMoving = false;
                        }
						break;
					default:
						break;
				}
			}
		}
	} while(nbBytesWaiting);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CNexDomeV3::isDomeMoving] Out: m_bDomeIsMoving = %s\n", timestamp, m_bDomeIsMoving?"Yes":"No");
	fflush(Logfile);
#endif
    return m_bDomeIsMoving;
}

bool CNexDomeV3::isDomeAtHome()
{
    bool bAtHome;
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    std::vector<std::string> rotatorStateFields;
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@SRR\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "SER") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }
    
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    if(nErr) {
        return false;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        return false;
    }

     // need to parse :SER,0,0,99498,0,300#
    nErr = parseFields(szResp, rotatorStateFields, ',');
    if(nErr)
        return false;
    if(rotatorStateFields.size()<3)
        return false;
    
    bAtHome = rotatorStateFields[2] == "1";
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDomeV3::isDomeAtHome bAthome : %s\n", timestamp, bAtHome?"True":"False");
    fflush(Logfile);
#endif

    return bAtHome;
  
}

int CNexDomeV3::syncDome(double dAz, double dEl)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nTmp;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_dCurrentAzPosition = dAz;
    // convert Az to steps;
    if(!m_nNbStepPerRev) {
        getDomeStepPerRev(nTmp);
    }
    nTmp = int((dAz/360.0)*m_nNbStepPerRev);
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@PWR,%d\r\n", nTmp);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::syncDome] ERROR = %d\n", timestamp, nErr);
        fprintf(Logfile, "[%s] [CNexDomeV3::syncDome] nErr = '%d'\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    // TODO : Also set Elevation when supported by the firmware.
    // m_dCurrentElPosition = dEl;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::syncDome] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::parkDome()
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bHomeOnPark) {
        m_bParking = true;
        nErr = goHome();
    } else
        nErr = gotoAzimuth(m_dParkAz);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::parkDome] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;

}

int CNexDomeV3::unparkDome()
{
    int nErr = PLUGIN_OK;
    if(m_bHomeOnUnpark) {
        m_bUnParking = true;
        nErr = goHome();
    }
    else {
        nErr = syncDome(m_dParkAz, m_dCurrentElPosition);
        m_bParked = false;
        m_bUnParking = false;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::unparkDome] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::gotoAzimuth(double dNewAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
	int nTmp;
	int nNewStepPos;
	std::vector<std::string> svFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_nNbStepPerRev) {
        getDomeStepPerRev(nTmp);
    }

    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;

	#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
			ltime = time(NULL);
			timestamp = asctime(localtime(&ltime));
			timestamp[strlen(timestamp) - 1] = 0;
			fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] m_dCurrentAzPosition = %3.2f\n", timestamp, m_dCurrentAzPosition);
			fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] dNewAz               = %3.2f\n", timestamp, dNewAz);
            fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] m_dCurrentAzPosition = %d\n", timestamp, int(round(m_dCurrentAzPosition)));
            fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] dNewAz               = %d\n", timestamp, int(round(dNewAz)));
            fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] m_nRotationDeadZone = %d\n", timestamp, m_nRotationDeadZone);
            fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] m_nCurrentRotatorPos = %d\n", timestamp, m_nCurrentRotatorPos);
			fflush(Logfile);
	#endif

	if(int(round(dNewAz)) == int(round(m_dCurrentAzPosition))) {
        m_dGotoAz = dNewAz;
		m_bDomeIsMoving = false;
		return nErr;
	}

	snprintf(szBuf, SERIAL_BUFFER_SIZE, "@GAR,%d\r\n", int(round(dNewAz)));

	// check if we're moving inside the dead zone.
	nNewStepPos = int((dNewAz/360.0) * m_nNbStepPerRev);

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] nNewStepPos = %d\n", timestamp, nNewStepPos);
            fflush(Logfile);
    #endif

    if (nNewStepPos <= (m_nCurrentRotatorPos + m_nRotationDeadZone) && nNewStepPos >= (m_nCurrentRotatorPos - m_nRotationDeadZone) ) {
        m_dGotoAz = dNewAz;
		m_bDomeIsMoving = false;
		// send the command anyway to update the controller internal counters
        #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] move is in dead zone\n", timestamp);
                fflush(Logfile);
        #endif
		nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
		return nErr;
	}

    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] szResp = %s\n", timestamp, szResp);
            fflush(Logfile);
    #endif
    m_bDomeIsMoving = true;
	memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    m_dGotoAz = dNewAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::gotoAzimuth] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::openShutter()
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nState;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bDomeIsMoving) {
        return SB_OK;
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] Opening shutter\n", timestamp);
    fflush(Logfile);
#endif
    nErr = getShutterState(nState);
    if(nState == OPEN)
        return nErr;
        
    nErr = domeCommand("@OPS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] ERROR = %d\n", timestamp, nErr);
        fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] nErr = '%d'\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }

    m_bDomeIsMoving = true;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    m_nCurrentShutterCmd = OPENING;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::closeShutter()
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bDomeIsMoving) {
        return SB_OK;
	}
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::closeShutter] Closing shutter\n", timestamp);
    fflush(Logfile);
#endif

    nErr = getShutterState(nState);
    if(nState == CLOSED)
        return nErr;

    nErr = domeCommand("@CLS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::closeShutter] ERROR = %d\n", timestamp, nErr);
        fprintf(Logfile, "[%s] [CNexDomeV3::closeShutter] nErr = '%d'\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    m_bDomeIsMoving = true;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;


    m_nCurrentShutterCmd = CLOSING;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::closeShutter] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::getFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = PLUGIN_OK;
    int i;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bDomeIsMoving) {
        return SB_OK;
	}

    nErr = domeCommand("@FRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "FR") && nb_timeout < (CMD_RESP_READ_TIMEOUTS*3)) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

    if(nErr) {
        strncpy(szVersion, "Unknown", SERIAL_BUFFER_SIZE);
        return PLUGIN_OK;
    }

    if(nb_timeout >= (CMD_RESP_READ_TIMEOUTS*3)) {
        strncpy(szVersion, "Unknown", SERIAL_BUFFER_SIZE);
        return PLUGIN_OK;
    }
    strncpy(szTmp, szResp+2, SERIAL_BUFFER_SIZE);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] szResp = %s\n", timestamp, szResp);
    fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] szTmp = %s\n", timestamp, szTmp);
    fflush(Logfile);
#endif

    nErr = parseFields(szTmp,versionFields, '.');
    if(versionFields.size()>1) {
        strVersion=versionFields[0]+".";
        for(i=1; i<versionFields.size(); i++) {
            strVersion+=versionFields[i];
        }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] strVersion = %s\n", timestamp, strVersion.c_str());
        fflush(Logfile);
#endif
        strncpy(szVersion, szTmp, nStrMaxLen);
        m_fVersion = atof(strVersion.c_str());
    }
    else {
        strncpy(szVersion, szTmp, nStrMaxLen);
        m_fVersion = atof(szResp);
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] szVersion = %s\n", timestamp, szVersion);
    fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] m_fVersion = %3.3f\n", timestamp, m_fVersion);
    fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::getFirmwareVersion(double &fVersion)
{
    int nErr = PLUGIN_OK;

    if(m_fVersion == 0.0f) {
        nErr = getFirmwareVersion(m_szFirmwareVersion, SERIAL_BUFFER_SIZE);
        if(nErr)
            return nErr;
    }

    fVersion = m_fVersion;

    return nErr;
}

int CNexDomeV3::goHome()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bDomeIsMoving) {
        return SB_OK;
    }
    else if(isDomeAtHome()){
            m_bHomed = true;
            syncDome(m_dHomeAz,m_dCurrentElPosition);
        #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] CNexDomeV3::goHome syncing to : %3.2f\n", timestamp,m_dHomeAz);
            fflush(Logfile);
        #endif
        return PLUGIN_OK;
    }
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::goHome]\n", timestamp);
    fflush(Logfile);
#endif

    // m_nHomingTries = 0;
    nErr = domeCommand("@GHR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::goHome] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }

    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    m_bDomeIsMoving = true;
    processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::goHome] nErr = '%d'\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isGoToComplete]\n", timestamp);
        fflush(Logfile);
    #endif

    if(isDomeMoving()) {
        bComplete = false;
        #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::isGoToComplete] Dome is still moving\n", timestamp);
            fflush(Logfile);
        #endif
        return nErr;
    }

	getDomeAz(dDomeAz);
    if(dDomeAz >0 && dDomeAz<1)
        dDomeAz = 0;
    
    while(ceil(m_dGotoAz) >= 360)
          m_dGotoAz = ceil(m_dGotoAz) - 360;

    while(ceil(dDomeAz) >= 360)
        dDomeAz = ceil(dDomeAz) - 360;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isGoToComplete] DomeAz    = %3.2f\n", timestamp, dDomeAz);
    fprintf(Logfile, "[%s] [CNexDomeV3::isGoToComplete] m_dGotoAz = %3.2f\n", timestamp, m_dGotoAz);
    fflush(Logfile);
#endif

    // we need to test "large" depending on the heading error , this is new in firmware 1.10 and up
    if ((ceil(m_dGotoAz) <= ceil(dDomeAz)+3) && (ceil(m_dGotoAz) >= ceil(dDomeAz)-3)) {
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isGoToComplete] ***** ERROR **** domeAz = %3.2f, m_dGotoAz = %3.2f\n", timestamp, dDomeAz, m_dGotoAz);
        fflush(Logfile);
#endif
        nErr = ERR_CMDFAILED;
        }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isGoToComplete] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::isOpenComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isOpenComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(!m_bShutterPresent) {
        bComplete = true;
        return nErr;
    }

    if(isDomeMoving()) {
        if(m_nCurrentShutterCmd == IDLE) {
            if(m_nShutterState == OPEN)
                bComplete = true;
            else
                bComplete = false;
        }
        return nErr;
    }

    nErr = getShutterState(nState);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isOpenComplete] ERROR nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }
    
    if(nState == OPEN){
        m_bShutterOpened = true;
        bComplete = true;
        m_dCurrentElPosition = 90.0;
    }
    else {
        m_bShutterOpened = false;
        bComplete = false;
        m_dCurrentElPosition = 0.0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isOpenComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isOpenComplete] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::isCloseComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isCloseComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(!m_bShutterPresent) {
        bComplete = true;
        return nErr;
    }

    if(isDomeMoving()) {
        if(m_nCurrentShutterCmd == CLOSED) {
            if(m_nShutterState == OPEN)
                bComplete = true;
            else
                bComplete = false;
        }
        return nErr;
    }

    nErr = getShutterState(nState);
    if(nErr) {
        #if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::isCloseComplete] ERROR nErr = %d\n", timestamp, nErr);
                fflush(Logfile);
        #endif
        return ERR_CMDFAILED;
    }
    if(nState == CLOSED){
        m_bShutterOpened = false;
        bComplete = true;
        m_dCurrentElPosition = 0.0;
    }
    else {
        m_bShutterOpened = true;
        bComplete = false;
        m_dCurrentElPosition = 90.0;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isCloseComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isCloseComplete] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}


int CNexDomeV3::isParkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz=0;
    bool bFoundHome;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] m_bParking = %s\n", timestamp, m_bParking?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif
    
    if(isDomeMoving()) {
        getDomeAz(dDomeAz);
        bComplete = false;
        return nErr;
    }
    
    if(m_bParking) {
        bComplete = false;
        nErr = isFindHomeComplete(bFoundHome);
        if(bFoundHome) { // we're home, now park
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
            ltime = time(NULL);
            timestamp = asctime(localtime(&ltime));
            timestamp[strlen(timestamp) - 1] = 0;
            fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] found home, now parking\n", timestamp);
            fflush(Logfile);
#endif
            m_bParking = false;
            nErr = gotoAzimuth(m_dParkAz);
        }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
        fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    nErr = getDomeAz(dDomeAz);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
        fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // we need to test "large" depending on the heading error
    if ((ceil(m_dParkAz) <= ceil(dDomeAz)+3) && (ceil(m_dParkAz) >= ceil(dDomeAz)-3)) {
        m_bParked = true;
        bComplete = true;
    }
    else {
        // we're not moving and we're not at the final destination !!!
        bComplete = false;
        m_bHomed = false;
        m_bParked = false;
        nErr = ERR_CMDFAILED;
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] m_bParked = %s\n", timestamp, m_bParked?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isParkComplete] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::isUnparkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    
    bComplete = false;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    if(!m_bParked) {
        bComplete = true;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isUnparkComplete] UNPARKED \n", timestamp);
        fflush(Logfile);
#endif
    }
    else if (m_bUnParking) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isUnparkComplete] unparking.. checking if we're home \n", timestamp);
        fflush(Logfile);
#endif
        nErr = isFindHomeComplete(bComplete);
        if(nErr)
            return nErr;
        if(bComplete) {
            m_bParked = false;
        }
        else {
            m_bParked = true;
        }
    }
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isUnparkComplete] m_bParked = %s\n", timestamp, m_bParked?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isUnparkComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isUnparkComplete] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::isFindHomeComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete]\n", timestamp);
    fflush(Logfile);
#endif

    if(isDomeMoving()) {
        m_bHomed = false;
        bComplete = false;
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete] still moving\n", timestamp);
        fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete] nErr = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;

    }

	if(isDomeAtHome()){
        m_bHomed = true;
        if(m_bUnParking)
            m_bParked = false;
        bComplete = true;
        // m_nHomingTries = 0;
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete] At Home\n", timestamp);
        fflush(Logfile);
#endif
    }
    else {
        // we're not moving and we're not at the home position !!!
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete] Not moving and not at home !!!\n", timestamp);
        fflush(Logfile);
#endif
        bComplete = false;
        m_bHomed = false;
        nErr = ERR_CMDFAILED;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete] bComplete = %s\n", timestamp, bComplete?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::isFindHomeComplete] nErr = %d\n", timestamp, nErr);
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::abortCurrentCommand()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    m_bHomed = false;
    m_bParked = false;
	m_bDomeIsMoving = false;
    m_bParking = false;
    m_bUnParking = false;

    nErr = domeCommand("@SWR\r\n", szResp, SERIAL_BUFFER_SIZE);
    nErr = domeCommand("@SWS\r\n", szResp, SERIAL_BUFFER_SIZE);

    getDomeAz(m_dGotoAz);

    return nErr;
}


#pragma mark - Getter / Setter

int CNexDomeV3::getNbTicksPerRev()
{
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getNbTicksPerRev] m_bIsConnected = %s\n", timestamp, m_bIsConnected?"True":"False");
    fflush(Logfile);
#endif

    if(m_bIsConnected)
        getDomeStepPerRev(m_nNbStepPerRev);

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getNbTicksPerRev] m_nNbStepPerRev = %d\n", timestamp, m_nNbStepPerRev);
    fflush(Logfile);
#endif

    return m_nNbStepPerRev;
}

int CNexDomeV3::setNbTicksPerRev(int nSteps)
{
    int nErr = PLUGIN_OK;

    if(m_bIsConnected)
        nErr = setDomeStepPerRev(nSteps);
    return nErr;
}

double CNexDomeV3::getHomeAz()
{
    if(m_bIsConnected)
        getDomeHomeAz(m_dHomeAz);
    return m_dHomeAz;
}

int CNexDomeV3::setHomeAz(double dAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nTmp;
    
    m_dHomeAz = dAz;

    #ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::setHomeAz] m_dHomeAz = %3.2f\n", timestamp, m_dHomeAz);
        fflush(Logfile);
    #endif

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    nTmp = int((dAz/360.0)*m_nNbStepPerRev);
    #ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::setHomeAz] nTmp = %d\n", timestamp, nTmp);
        fflush(Logfile);
    #endif

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@HWR,%d\r\n", nTmp);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

double CNexDomeV3::getParkAz()
{
    return m_dParkAz;
}

int CNexDomeV3::setParkAz(double dAz)
{
    m_dParkAz = dAz;
    return PLUGIN_OK;
}

void CNexDomeV3::setShutterPresent(bool bPresent)
{
    
    m_bShutterPresent = bPresent;
    
}

int CNexDomeV3::getShutterStepsRange()
{
	if(m_bIsConnected)
		getShutterSteps(m_nShutterSteps);
	return m_nShutterSteps;
}

int CNexDomeV3::setShutterStepsRange(int nSteps)
{
	int nErr = PLUGIN_OK;

	if(m_bIsConnected)
		nErr = setShutterSteps(nSteps);
	return nErr;
}

double CNexDomeV3::getCurrentAz()
{

    if(m_bIsConnected) {
        getDomeAz(m_dCurrentAzPosition);
   }
    return m_dCurrentAzPosition;
}

double CNexDomeV3::getCurrentEl()
{
    if(m_bIsConnected)
        getDomeEl(m_dCurrentElPosition);
    
    return m_dCurrentElPosition;
}

int CNexDomeV3::getCurrentShutterState()
{
    if(m_bIsConnected)
        getShutterState(m_nShutterState);

    return m_nShutterState;
}


int CNexDomeV3::getRainSensorStatus(int &nStatus)
{
    int nErr = PLUGIN_OK;

    if(m_bIsConnected) {
        nErr = processAsyncResponses();
    }
    nStatus = m_nIsRaining;

    return nErr;
}

int CNexDomeV3::getRotationSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@VRR\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "VRR") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        nSpeed = 0;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nSpeed = 0;
        return PLUGIN_OK;
    }

    // need to parse
    nSpeed = atoi(szResp+3);
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getRotationSpeed] nSpeed =  %d\n", timestamp, nSpeed);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::setRotationSpeed(int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@VWR,%d\r\n", nSpeed);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}


int CNexDomeV3::getRotationAcceleration(int &nAcceleration)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@ARR\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "ARR") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        nAcceleration = 0;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nAcceleration = 0;
        return PLUGIN_OK;
    }

    nAcceleration = atoi(szResp+3);
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getRotationAcceleration] nAcceleration =  %d\n", timestamp, nAcceleration);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::setRotationAcceleration(int nAcceleration)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@AWR,%d\r\n", nAcceleration);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CNexDomeV3::getShutterSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nSpeed = 0;
        return nErr;
    }

    nErr = domeCommand("@VRS\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "VRS") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        nSpeed = 0;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nSpeed = 0;
        return PLUGIN_OK;
    }

    nSpeed = atoi(szResp+3);
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterSpeed] nSpeed =  %d\n", timestamp, nSpeed);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::setShutterSpeed(int nSpeed)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bShutterPresent) {
        return nErr;
    }

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@VWS,%d\r\n", nSpeed);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CNexDomeV3::getShutterAcceleration(int &nAcceleration)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    char szBuf[SERIAL_BUFFER_SIZE];
    int nb_timeout;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_bShutterPresent) {
        nAcceleration = 0;
        return nErr;
    }

    nErr = domeCommand("@ARS\r\n", szResp, SERIAL_BUFFER_SIZE);

    nb_timeout = 0;
    memcpy(szBuf, szResp, SERIAL_BUFFER_SIZE);
    while(!strstr(szBuf, "ARS") && nb_timeout < CMD_RESP_READ_TIMEOUTS) {
        readResponse(szBuf, SERIAL_BUFFER_SIZE);
        nErr = processResponse(szBuf, szResp, SERIAL_BUFFER_SIZE);
        nb_timeout++;
    }

    if(nErr == CMD_PROC_DONE)
        nErr = PLUGIN_OK;
    
    if(nErr) {
        nAcceleration = 0;
        return PLUGIN_OK;
    }
    
    if(nb_timeout >= CMD_RESP_READ_TIMEOUTS) {
        nAcceleration = 0;
        return PLUGIN_OK;
    }

    nAcceleration = atoi(szResp+3);
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterAcceleration] nAcceleration =  %d\n", timestamp, nAcceleration);
    fflush(Logfile);
#endif
    return nErr;
}


int CNexDomeV3::setShutterAcceleration(int nAcceleration)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@AWS,%d\r\n", nAcceleration);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CNexDomeV3::getRotatorStepPos(int &nPos)
{
    nPos = m_nCurrentRotatorPos;
    return PLUGIN_OK;
}

int CNexDomeV3::loadParamFromEEProm()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("@ZRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(m_bShutterPresent)
        nErr = domeCommand("@ZRS\r\n", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
    
}

int CNexDomeV3::resetToFactoryDefault()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("@ZDR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(m_bShutterPresent)
        nErr = domeCommand("@ZDS\r\n", szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CNexDomeV3::saveParamToEEProm()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("@ZWR\r\n", szResp, SERIAL_BUFFER_SIZE);
    m_pSleeper->sleep(500);

    if(m_bShutterPresent) {
        nErr = domeCommand("@ZWS\r\n", szResp, SERIAL_BUFFER_SIZE);
        m_pSleeper->sleep(500);

    }
    return nErr;

}

void CNexDomeV3::setHomeOnPark(const bool bEnabled)
{
    m_bHomeOnPark = bEnabled;
}

void CNexDomeV3::setHomeOnUnpark(const bool bEnabled)
{
    m_bHomeOnUnpark = bEnabled;
}


int CNexDomeV3::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    if(!pszResp) {
        return ERR_CMDFAILED;
    }

    if(!strlen(pszResp)) {
        return ERR_CMDFAILED;
    }

    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
    }

    if(svFields.size()==0) {
        nErr = ERR_CMDFAILED;
    }
    return nErr;
}


std::string& CNexDomeV3::trim(std::string &str, const std::string& filter )
{
    return ltrim(rtrim(str, filter), filter);
}

std::string& CNexDomeV3::ltrim(std::string& str, const std::string& filter)
{
    str.erase(0, str.find_first_not_of(filter));
    return str;
}

std::string& CNexDomeV3::rtrim(std::string& str, const std::string& filter)
{
    str.erase(str.find_last_not_of(filter) + 1);
    return str;
}

std::string CNexDomeV3::findField(std::vector<std::string> &svFields, const std::string& token)
{
    for(int i=0; i<svFields.size(); i++){
        if(svFields[i].find(token)!= -1) {
            return svFields[i];
        }
    }
    return std::string();
}

