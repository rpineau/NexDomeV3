//
//  nexdome.cpp
//  NexDome X2 plugin
//
//  Created by Rodolphe Pineau on 6/11/2016.


#include "NexDomeV3.h"

CNexDomeV3::CNexDomeV3()
{
    // set some sane values
    m_bDebugLog = true;
    
    m_pSerx = NULL;
    m_bIsConnected = false;

    m_nNbStepPerRev = 0;
    m_dShutterBatteryVolts = 0.0;
    
    m_dHomeAz = 0;
    
    m_dCurrentAzPosition = 0.0;
    m_dCurrentElPosition = 0.0;

	m_bDomeIsMoving = false;
    m_bCalibrating = false;

    m_bShutterOpened = false;
	m_nCurrentShutterCmd = IDLE;

    m_bParked = true;
    m_bHomed = false;

    m_fVersion = 0.0;
    m_nHomingTries = 0;
    m_nGotoTries = 0;

    m_nIsRaining = NOT_RAINING;

    m_dShutterVolts = -1.0;
    
    memset(m_szFirmwareVersion,0,SERIAL_BUFFER_SIZE);
    memset(m_szLogBuffer,0,PLUGIN_LOG_BUFFER_SIZE);

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
    fprintf(Logfile, "[%s] CNexDomeV3 Constructor Called.\n", timestamp);
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
    m_bCalibrating = false;
	m_bDomeIsMoving = false;
    m_bHomed = false;

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
    m_bCalibrating = false;
	m_bDomeIsMoving = false;
    m_bHomed = false;

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
    char szTmp[SERIAL_BUFFER_SIZE];
    int nb_timeout;
    std::string sResp;
    std::string sTmp;

    // m_pSerx->purgeTxRx();

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] sending : %s\n", timestamp, pszCmd);
    fflush(Logfile);
#endif

    nErr = m_pSerx->writeFile((void *)pszCmd, strlen(pszCmd), ulBytesWrite);
    m_pSerx->flushTx();
    if(nErr)
        return nErr;

    nb_timeout = 0;
    
    while(true) {
        if(nb_timeout>10) {
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
            m_pSleeper->sleep(250);
            nb_timeout++;
            continue;
        }
        //cleanup the string
        sTmp.assign(szResp);
        sResp = trim(sTmp," \n\r#");
        strncpy(szResp, sResp.c_str(), SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] response : '%s'\n", timestamp, szResp);
        fflush(Logfile);
#endif
        // we got some event notification., read next response
        switch(sResp[0]) {
            case 'C' :
                nb_timeout++;
                break;
            case 'P' :
                if(!strstr(pszCmd,"PRS")) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] rotator position update : '%s'\n", timestamp, szResp);
                    fflush(Logfile);
#endif
                }
                else {
                    if(pszResult)
                        strncpy(pszResult, szResp, nResultMaxLen);
                    return nErr;
                }
                break;

            case 'S' :
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] shutter position update : '%s'\n", timestamp, szResp);
                fflush(Logfile);
#endif
                break;

            case 'X' :
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                ltime = time(NULL);
                timestamp = asctime(localtime(&ltime));
                timestamp[strlen(timestamp) - 1] = 0;
                fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] XBee status : '%s'\n", timestamp, szResp);
                fflush(Logfile);
#endif
                nb_timeout++;
                break;

            case 'o' :
                nb_timeout++;
                break;

            case ':' :
                // we got some response, parse and read next response if needed
                if(strstr(szResp,":BV")) {
                    memcpy(szTmp, szResp+3, SERIAL_BUFFER_SIZE);
                    m_dShutterVolts = float(atoi(szTmp)) * 3.0 * (5.0 / 1023.0);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] m_dShutterVolts : %3.2f\n", timestamp, m_dShutterVolts);
                    fflush(Logfile);
#endif
                    nb_timeout++;
                }
                else if(strstr(szResp,":RainStopped")) {
                    m_nIsRaining = NOT_RAINING;
                    nb_timeout++;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] NOT_RAINING\n", timestamp);
                    fflush(Logfile);
#endif
                }
                else if(strstr(szResp,":Rain")) {
                    m_nIsRaining = RAINING;
                    nb_timeout++;
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
                    ltime = time(NULL);
                    timestamp = asctime(localtime(&ltime));
                    timestamp[strlen(timestamp) - 1] = 0;
                    fprintf(Logfile, "[%s] [CNexDomeV3::domeCommand] RAINING\n", timestamp);
                    fflush(Logfile);
#endif
                }
                else if(strstr(szResp,":left")) {
                    nb_timeout++;
                }
                else if(strstr(szResp,":right")) {
                    nb_timeout++;
                }
                else if(strstr(szResp,":open")) {
                    nb_timeout++;
                }
                else if(strstr(szResp,":close")) {
                    nb_timeout++;
                }
                else {
                    if(pszResult)
                        strncpy(pszResult, &szResp[1], nResultMaxLen);
                    return nErr;
                }
                break;
            default :
                if(pszResult)
                    strncpy(pszResult, szResp, nResultMaxLen);
                return nErr;
                break;
        }
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
			if(!ulTotalBytesRead)
				nErr = PLUGIN_BAD_CMD_RESPONSE;
			else
				nErr = ERR_DATAOUT;
			break;
		}
		ulTotalBytesRead += ulBytesRead;
	} while (*pszBufPtr++ != 0x0a && ulTotalBytesRead < nBufferLen );

	if(ulTotalBytesRead)
		*(pszBufPtr-1) = 0; //remove the \n

	return nErr;
}

int CNexDomeV3::getDomeAz(double &dDomeAz)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nStepPos;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(m_bCalibrating || m_bDomeIsMoving) {
		dDomeAz = m_dCurrentAzPosition;
		return nErr;
	}

    nErr = domeCommand("@PRR\r\n", szResp,  SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeAz] szResp = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    
    nStepPos = atoi(szResp+3); // PRRxxx

    // convert steps to deg
    dDomeAz = (double(nStepPos)/m_nNbStepPerRev) * 360.0;
    m_dCurrentAzPosition = dDomeAz;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeStepPerRev] nStepPos = %d\n", timestamp, nStepPos);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeStepPerRev] dDomeAz = %3.2f\n", timestamp, dDomeAz);
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::getDomeEl(double &dDomeEl)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nStepPos;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
		dDomeEl = m_dCurrentElPosition;
		return nErr;
	}

    if(!m_bShutterOpened)
    {
        dDomeEl = 0.0;
        return nErr;
    }
    else {
        dDomeEl = 90.0;
        return nErr;
    }

    nErr = domeCommand("@PRS\r\n", szResp, SERIAL_BUFFER_SIZE); // might implement latter
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getDomeEl] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    
    // convert steps to deg
    nStepPos = atoi(szResp+3); // PRRxxx
    
    dDomeEl = (double(nStepPos)/m_nNbStepPerRev) * 360.0;
    m_dCurrentElPosition = dDomeEl;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeEl] nStepPos = %d\n", timestamp, nStepPos);
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeEl] dDomeAz = %3.2f\n", timestamp, dDomeEl);
    fflush(Logfile);
#endif
    return nErr;
}


int CNexDomeV3::getDomeHomeAz(double &dAz)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    int nStepPos;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
		dAz = m_dHomeAz;
		return nErr;
	}

    nErr = domeCommand("@HRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getDomeHomeAz] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
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
    fflush(Logfile);
#endif
    return nErr;
}

int CNexDomeV3::getShutterState(int &nState)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> shutterStateFields;
    int nOpen, nClosed;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
		nState = m_nShutterState;
		return nErr;
	}

    nErr = domeCommand("@SRS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState] response = '%s'\n", timestamp, szResp);
    fflush(Logfile);
#endif
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
    else if(nClosed)
        nState = CLOSED;
    else if(nOpen)
        nState = OPEN;

    m_nShutterState = nState;
    
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getShutterState] nState = '%d'\n", timestamp, nState);
    fflush(Logfile);
#endif

    return nErr;
}


int CNexDomeV3::getDomeStepPerRev(int &nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@RRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getDomeStepPerRev] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    // need parsing RRR99498
    
    nStepPerRev = atoi(szResp+3);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDomeStepPerRev] nStepPerRev = %d\n", timestamp, nStepPerRev);
    fflush(Logfile);
#endif
    m_nNbStepPerRev = nStepPerRev;
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
    return nErr;
}

int CNexDomeV3::getShutterSteps(int &nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("@RRS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getShutterSteps] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    // need parsing
    nStepPerRev = atoi(szResp+3);
    m_nShutterSteps = nStepPerRev;
    return nErr;
}

int CNexDomeV3::setShutterSteps(int &nStepPerRev)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    
    m_nNbStepPerRev = nStepPerRev;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@RWS,%d\r\n", nStepPerRev);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}

int CNexDomeV3::getdShutterVolts(double &dShutterVolts)
{
    int nErr = PLUGIN_OK;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    dShutterVolts = m_dShutterVolts;
    return nErr;
}


bool CNexDomeV3::isDomeMoving()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];
	int nStepPos;
	int nbByteWaiting = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(!m_bDomeIsMoving)
		return false;

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
	ltime = time(NULL);
	timestamp = asctime(localtime(&ltime));
	timestamp[strlen(timestamp) - 1] = 0;
	fprintf(Logfile, "[%s] [CNexDomeV3::isDomeMoving] In : m_bDomeIsMoving = %s\n", timestamp, m_bDomeIsMoving?"Yes":"No");
	fflush(Logfile);
#endif
	do {
		m_pSerx->bytesWaitingRx(nbByteWaiting);
		if(nbByteWaiting) {
			nErr = readResponse(szResp, SERIAL_BUFFER_SIZE, 250);
			if(nErr && nErr != ERR_DATAOUT)
				return m_bDomeIsMoving;

			if(strlen(szResp)) {
				switch(szResp[0]) {
					case 'P' :
						nStepPos = atoi(szResp+1); // Pxxxxx
						// convert steps to deg
						m_dCurrentAzPosition = (double(nStepPos)/m_nNbStepPerRev) * 360.0;
						break;
					case ':' :
						// :SER is sent at the end of the move-> parse :SER,0,0,55080,0,300#
						if(strstr(szResp,"SER")) {
							m_bDomeIsMoving = false;
						}
						break;
					default:
						break;
				}
			}
		}
	} while(nbByteWaiting);

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
    std::vector<std::string> rotatorStateFields;

    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = domeCommand("@SRR\r\n", szResp, SERIAL_BUFFER_SIZE);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::isDomeAtHome] z# response = %s\n", timestamp, szResp);
    fflush(Logfile);
#endif
    if(nErr) {
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
    nTmp = int((dAz/360)*m_nNbStepPerRev);
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@PWR,%d\r\n", nTmp);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::syncDome] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    // TODO : Also set Elevation when supported by the firmware.
    // m_dCurrentElPosition = dEl;
    return nErr;
}

int CNexDomeV3::parkDome()
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = goHome();
    return nErr;

}

int CNexDomeV3::unparkDome()
{
    int nErr = PLUGIN_OK;
    nErr = goHome();
    return nErr;
}

int CNexDomeV3::gotoAzimuth(double dNewAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nTmp;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(!m_nNbStepPerRev) {
        getDomeStepPerRev(nTmp);
    }

    while(dNewAz >= 360)
        dNewAz = dNewAz - 360;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@GAR,%d\r\n", int(dNewAz));
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

    m_dGotoAz = dNewAz;
    m_nGotoTries = 0;
	m_bDomeIsMoving = true;
    return nErr;
}

int CNexDomeV3::openShutter()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
        return SB_OK;
	}

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] Opening shutter\n", timestamp);
    fflush(Logfile);
#endif

    if (m_bDebugLog) {
        snprintf(m_szLogBuffer,PLUGIN_LOG_BUFFER_SIZE,"[CNexDomeV3::openShutter] Opening shutter");
        m_pLogger->out(m_szLogBuffer);
    }


    nErr = domeCommand("@OPS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        snprintf(m_szLogBuffer,PLUGIN_LOG_BUFFER_SIZE,"[CNexDomeV3::openShutter] ERROR gotoAzimuth");
        m_pLogger->out(m_szLogBuffer);
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }

	m_nCurrentShutterCmd = OPENING;

    return nErr;
}

int CNexDomeV3::closeShutter()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
        return SB_OK;
	}
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::closeShutter] Closing shutter\n", timestamp);
    fflush(Logfile);
#endif

    nErr = domeCommand("@CLS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::openShutter] closeShutter = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
    }

	m_nCurrentShutterCmd = CLOSING;
    return nErr;
}

int CNexDomeV3::getFirmwareVersion(char *szVersion, int nStrMaxLen)
{
    int nErr = PLUGIN_OK;
    int i;
    char szResp[SERIAL_BUFFER_SIZE];
    char szTmp[SERIAL_BUFFER_SIZE];
    std::vector<std::string> firmwareFields;
    std::vector<std::string> versionFields;
    std::string strVersion;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
        return SB_OK;
	}
    nErr = domeCommand("@FRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::getFirmwareVersion] ERROR = %s\n", timestamp, szResp);
        fflush(Logfile);
#endif
        return nErr;
    }
    strncpy(szTmp, szResp+2, SERIAL_BUFFER_SIZE);

#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
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
    fflush(Logfile);
#endif

    return nErr;
}

int CNexDomeV3::getFirmwareVersion(float &fVersion)
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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

	if(m_bCalibrating || m_bDomeIsMoving) {
        return SB_OK;
    }
    else if(isDomeAtHome()){
            m_bHomed = true;
            return PLUGIN_OK;
    }
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] CNexDomeV3::goHome \n", timestamp);
    fflush(Logfile);
#endif

    m_nHomingTries = 0;
    nErr = domeCommand("@GHR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CNexDomeV3::goHome ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
	m_bDomeIsMoving = true;
    return nErr;
}

int CNexDomeV3::calibrate()
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;


    // nErr = domeCommand("c#", szResp, 'c', SERIAL_BUFFER_SIZE);
    if(nErr) {
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::calibrate] ERROR = %d\n", timestamp, nErr);
        fflush(Logfile);
#endif
        return nErr;
    }
    m_bCalibrating = true;
	m_bDomeIsMoving = true;

    return nErr;
}

int CNexDomeV3::isGoToComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        bComplete = false;
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
    fprintf(Logfile, "[%s] CNexDomeV3::isGoToComplete DomeAz = %3.2f\n", timestamp, dDomeAz);
    fflush(Logfile);
#endif

    // we need to test "large" depending on the heading error , this is new in firmware 1.10 and up
    if ((ceil(m_dGotoAz) <= ceil(dDomeAz)+3) && (ceil(m_dGotoAz) >= ceil(dDomeAz)-3)) {
        bComplete = true;
        m_nGotoTries = 0;
    }
    else {
        // we're not moving and we're not at the final destination !!!
#if defined PLUGIN_DEBUG && PLUGIN_DEBUG >= 2
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CNexDomeV3::isGoToComplete ***** ERROR **** domeAz = %3.2f, m_dGotoAz = %3.2f\n", timestamp, dDomeAz, m_dGotoAz);
        fflush(Logfile);
#endif
        if(m_nGotoTries == 0) {
            bComplete = false;
            m_nGotoTries = 1;
            gotoAzimuth(m_dGotoAz);
        }
        else {
            m_nGotoTries = 0;
            nErr = ERR_CMDFAILED;
        }
    }

    return nErr;
}

int CNexDomeV3::isOpenComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    int nState;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
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
    fprintf(Logfile, "[%s] CNexDomeV3::isOpenComplete bComplete = %s\n", timestamp, bComplete?"True":"False");
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

    nErr = getShutterState(nState);
    if(nErr)
        return ERR_CMDFAILED;
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
    fprintf(Logfile, "[%s] CNexDomeV3::isCloseComplete bComplete = %s\n", timestamp, bComplete?"True":"False");
    fflush(Logfile);
#endif

    return nErr;
}


int CNexDomeV3::isParkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = isFindHomeComplete(bComplete);
    if(bComplete)
        m_bParked = true;
    return nErr;
}

int CNexDomeV3::isUnparkComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    
    nErr = isFindHomeComplete(bComplete);
    if(bComplete)
        m_bParked = false;

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
    fprintf(Logfile, "[%s] CNexDomeV3::isFindHomeComplete\n", timestamp);
    fflush(Logfile);
#endif

    if(isDomeMoving()) {
        m_bHomed = false;
        bComplete = false;
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CNexDomeV3::isFindHomeComplete still moving\n", timestamp);
        fflush(Logfile);
#endif
        return nErr;

    }

	if(isDomeAtHome()){
        m_bHomed = true;
        bComplete = true;
        syncDome(m_dHomeAz, m_dCurrentElPosition);
        m_nHomingTries = 0;
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] CNexDomeV3::isFindHomeComplete At Home\n", timestamp);
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
        // sometimes we pass the home sensor and the dome doesn't rotate back enough to detect it.
        // this is mostly the case with firmware 1.10 with the new error correction ... 
        // so give it another try
        if(m_nHomingTries == 0) {
            m_nHomingTries = 1;
            nErr = goHome();
        }
		else
        	nErr = ERR_CMDFAILED;
    }

    return nErr;
}


int CNexDomeV3::isCalibratingComplete(bool &bComplete)
{
    int nErr = PLUGIN_OK;
    double dDomeAz = 0;

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    if(isDomeMoving()) {
        // getDomeAz(dDomeAz);
        m_bHomed = false;
        bComplete = false;
        return nErr;
    }

    nErr = getDomeAz(dDomeAz);

    if (ceil(m_dHomeAz) != ceil(dDomeAz)) {
        // We need to resync the current position to the home position.
        m_dCurrentAzPosition = m_dHomeAz;
        syncDome(m_dCurrentAzPosition,m_dCurrentElPosition);
        m_bHomed = true;
        bComplete = true;
    }

    nErr = getDomeStepPerRev(m_nNbStepPerRev);
    m_bHomed = true;
    bComplete = true;
    m_bCalibrating = false;
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getNbTicksPerRev] final m_nNbStepPerRev = %d\n", timestamp, m_nNbStepPerRev);
    fprintf(Logfile, "[%s] [CNexDomeV3::getNbTicksPerRev] final m_bHomed = %s\n", timestamp, m_bHomed?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::getNbTicksPerRev] final m_bCalibrating = %s\n", timestamp, m_bCalibrating?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::getNbTicksPerRev] final bComplete = %s\n", timestamp, bComplete?"True":"False");
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
    m_bCalibrating = false;
	m_bDomeIsMoving = false;
    m_nGotoTries = 1;   // prevents the goto retry
    m_nHomingTries = 1; // prevents the find home retry
    
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


int CNexDomeV3::setHomeAz(double dAz)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];
    int nTmp;
    
    m_dHomeAz = dAz;
    
    if(!m_bIsConnected)
        return NOT_CONNECTED;
    nTmp = (dAz/360)*m_nNbStepPerRev;
    
    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@HWR,%d\r\n", nTmp);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
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


int CNexDomeV3::getDefaultDir(bool &bNormal)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    bNormal = true;
    nErr = domeCommand("y#", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }

    bNormal = atoi(szResp) ? false:true;
#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::getDefaultDir] bNormal =  %s\n", timestamp, bNormal?"True":"False");
    fflush(Logfile);
#endif


    return nErr;
}

int CNexDomeV3::setDefaultDir(bool bNormal)
{
    int nErr = PLUGIN_OK;
    char szBuf[SERIAL_BUFFER_SIZE];
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "y %1d#", bNormal?0:1);

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::setDefaultDir] bNormal =  %s\n", timestamp, bNormal?"True":"False");
    fprintf(Logfile, "[%s] [CNexDomeV3::setDefaultDir] szBuf =  %s\n", timestamp, szBuf);
    fflush(Logfile);
#endif

    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    return nErr;

}

int CNexDomeV3::getRainSensorStatus(int &nStatus)
{
    int nErr = PLUGIN_OK;

    nStatus = m_nIsRaining;

    return nErr;
}

int CNexDomeV3::getRotationSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@VRR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@ARR\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
    // need parsing
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

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@AWR.%d#", nAcceleration);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);

    return nErr;
}

int CNexDomeV3::getShutterSpeed(int &nSpeed)
{
    int nErr = PLUGIN_OK;
    char szResp[SERIAL_BUFFER_SIZE];

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@VRS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
    // need parsing
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

    if(!m_bIsConnected)
        return NOT_CONNECTED;

    nErr = domeCommand("@ARS\r\n", szResp, SERIAL_BUFFER_SIZE);
    if(nErr) {
        return nErr;
    }
    // need parsing
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

    snprintf(szBuf, SERIAL_BUFFER_SIZE, "@AWS.%d#", nAcceleration);
    nErr = domeCommand(szBuf, szResp, SERIAL_BUFFER_SIZE);
    return nErr;
}


int CNexDomeV3::parseFields(const char *pszResp, std::vector<std::string> &svFields, char cSeparator)
{
    int nErr = PLUGIN_OK;
    std::string sSegment;
    if(!pszResp) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::parseFields] pszResp is NULL\n", timestamp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

    if(!strlen(pszResp)) {
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::parseFields] pszResp is enpty\n", timestamp);
        fflush(Logfile);
#endif
        return ERR_CMDFAILED;
    }

#ifdef PLUGIN_DEBUG
    ltime = time(NULL);
    timestamp = asctime(localtime(&ltime));
    timestamp[strlen(timestamp) - 1] = 0;
    fprintf(Logfile, "[%s] [CNexDomeV3::parseFields] pszResp =%s\n", timestamp, pszResp);
    fflush(Logfile);
#endif

    std::stringstream ssTmp(pszResp);

    svFields.clear();
    // split the string into vector elements
    while(std::getline(ssTmp, sSegment, cSeparator))
    {
        svFields.push_back(sSegment);
#ifdef PLUGIN_DEBUG
        ltime = time(NULL);
        timestamp = asctime(localtime(&ltime));
        timestamp[strlen(timestamp) - 1] = 0;
        fprintf(Logfile, "[%s] [CNexDomeV3::parseFields] sSegment = %s\n", timestamp, sSegment.c_str());
        fflush(Logfile);
#endif
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

