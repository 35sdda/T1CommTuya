#include "T1Serial.hpp"
#include <thread>
#include <iostream>

Serial::Serial() {
    b_OpenSign = false;
    nSerialID = 0;
}

Serial::~Serial() {
    Close();
}

int Serial::OpenSerial(std::string SerialID, E_BaudRate Bps, E_DataSize DataSize, E_Parity Parity, E_StopBit StopBit)
{
	Close();
    nSerialID = open( SerialID.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if ( -1 == nSerialID ) {
        std::string str = SerialID + " open fail !!!";
        perror(str.c_str());
        return -1;
    } 
    struct termios Opt;
    tcgetattr(nSerialID, &ProtoOpt);  //get the setting of the device
    
    Opt = ProtoOpt;

    /* set the Baud rate */
    switch (Bps) {
        case E_BaudRate::_2400:
            cfsetispeed(&Opt,B2400);
            cfsetospeed(&Opt,B2400);
            break;
        case E_BaudRate::_4800:
            cfsetispeed(&Opt,B4800);
            cfsetospeed(&Opt,B4800);
            break;	
        case E_BaudRate::_9600:
            cfsetispeed(&Opt,B9600);
            cfsetospeed(&Opt,B9600);
            break;	
        case E_BaudRate::_19200:
            cfsetispeed(&Opt,B19200);
            cfsetospeed(&Opt,B19200);
            break;
        case E_BaudRate::_38400:
            cfsetispeed(&Opt,B38400);
            cfsetospeed(&Opt,B38400);
            break;	
        case E_BaudRate::_57600:
            cfsetispeed(&Opt,B57600);
            cfsetospeed(&Opt,B57600);
            break;														
        case E_BaudRate::_115200:
            cfsetispeed(&Opt,B115200);
            cfsetospeed(&Opt,B115200);
            break;	
        case E_BaudRate::_460800:
            cfsetispeed(&Opt,B460800);
            cfsetospeed(&Opt,B460800);
            break;						
        default		:
            printf("T1Serial: Don't exist baudrate %d !\n",Bps);
            return (-1);																	
    }

    /* set the data size */
    Opt.c_cflag &= (~CSIZE);
    switch (DataSize) {
		case E_DataSize::_5:
            Opt.c_cflag |= CS5;
            break;
        case E_DataSize::_6:
            Opt.c_cflag |= CS6;
        case E_DataSize::_7:
            Opt.c_cflag |= CS7;
            break;
        case E_DataSize::_8:
            Opt.c_cflag |= CS8;
            break;
        default:
            /*perror("Don't exist iDataSize !");*/
            printf("T1Serial: Don't exist DataSize %d !\n",DataSize);
            return (-1);								
    }

	/* set the parity */
	switch (Parity) {
        case E_Parity::NoneParity:		
            Opt.c_cflag &= (~PARENB);
            break;
        case E_Parity::OddParity:	
            Opt.c_cflag |= PARENB;
            Opt.c_cflag |= PARODD;
            Opt.c_iflag |= (INPCK | ISTRIP);
            break;
        case E_Parity::EvenParity:
            Opt.c_cflag |= PARENB;
            Opt.c_cflag &= (~PARODD);
            Opt.c_iflag |= (INPCK | ISTRIP);
            break;				
        default:
            /*perror("Don't exist cParity  !");*/
            printf("T1Serial: Don't exist Parity %c !\n",Parity);
            return (-1);								
	}

	/* set the stop bit */
	switch (StopBit) {
        case E_StopBit::_1:
            Opt.c_cflag &= (~CSTOPB);
            break;
        case E_StopBit::_2:
            Opt.c_cflag |= CSTOPB;
            break;
        default:
            printf("T1Serial: Don't exist iStopBit %d !\n",StopBit);
            return (-1);								
	}

    // use the3 Raw Mode
    Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);   
    Opt.c_oflag &= ~OPOST;
    Opt.c_iflag &= ~(ICRNL | IGNCR | INLCR);  // Do NOT convert \r(0x0D) to \n(0x0A)

	tcflush(nSerialID,TCIOFLUSH);		/* refresh the IO*/

    Opt.c_cc[VTIME] = 0;	/* set the waiting time */
	Opt.c_cc[VMIN] = 0;	 /* set the minimum value */

    int Result = tcsetattr(nSerialID,TCSANOW,&Opt);  // activity all setting
	if (Result) {
		perror("T1Serial: Set new terminal description error !");
		return (-1);
	}	

    b_OpenSign = true;
    RunRecv();
    return 0;
}

int Serial::Send(unsigned char *Buff, int length) {
	int iLen = 0;
	if ( length <= 0 )
	{
		printf("T1Serial: Send byte number error !\n");
		return -1;
	}
	//printf("T1Serial(%d), Send[%s], bytes:%d\n", nSerialID, Buff, length);
	//length = 2;
	iLen = write(nSerialID,Buff,length);
	//printf("T1Serial(%d): Send bytes:%d\n", nSerialID, iLen);
	
	return iLen;
}

int Serial::Recv(unsigned char *Buff, int length) {
    int res =  RefreshBuffer(Buff, length, true);

    return res;
}

int Serial::Close() {
    if ( nSerialID > 0 ) {
        tcsetattr (nSerialID, TCSADRAIN, &ProtoOpt);  // reset to default setting
    }
    close(nSerialID);
    b_OpenSign = false;
    
    return 0;
}


void Serial::RunRecv() {
    std::thread ThRecv	= std::thread
    {
        [&]()
		{
            unsigned char RecvBuf[4096] = {0};
            while (b_OpenSign)
            {
                usleep(10*1000);
                if((nSerialID < 0))
		        {
			        continue;
		        }

                memset(RecvBuf, 0, 4096);

                int res = read(nSerialID, RecvBuf, sizeof(RecvBuf));
                //std::cout << "res = " << res << std::endl;
                if(res > 0)
                {
                    RefreshBuffer(RecvBuf, res, false);
                }
            }
        }
    };

    ThRecv.detach();
}

int Serial::RefreshBuffer(unsigned char *pBuf, int Len, bool RecvTypet) {
    static unsigned char  Buffer[RecvBufferLen + 1] = {0};
    static int nSum=0;				    //	data bytes in buffer
	signed int nStop=0;

    int ren = 0;

    if ( false == RecvTypet ) {
        //************************ copy received data to buffer ************************/
        //std::cout<<"recv = "<< Len <<std::endl;
        
        if ( (Len + nSum) <= RecvBufferLen ) {		//	must less than 1K
            memcpy(&Buffer[nSum], pBuf, Len);
            nSum = Len + nSum;
        } else {
            if ( Len <= RecvBufferLen )	{
                memcpy(Buffer, pBuf, Len);
                nSum = Len;
            } else {						//	more than 1K, will recover write some buffer
                memcpy(Buffer, pBuf + (Len - RecvBufferLen), RecvBufferLen);
                nSum = RecvBufferLen;
            }
        }
        //std::cout<<"----> nSum = "<< nSum <<std::endl;
        ren = 0;
    } else {
        if ( Len <= 0 ) {
            return -1;
        }
        if ( nSum <= 0 ) {
            return 0;
        }
        
        if ( Len <= nSum ) {
            memcpy(pBuf, Buffer, Len);
            nStop =  Len;
            ren = Len;
        } else {
            memcpy(pBuf, Buffer, nSum);
            nStop = nSum;
            ren = nSum;
        }

        //************ shift data ***************/
        if ( nStop==0 ) {
            return 0;
        } else if ( nSum > nStop )	{  // shift data to buffer[0]
            for ( int i=0; i<(nSum-nStop); i++ ) {
                Buffer[i] = Buffer[nStop + i];
            }
            nSum = nSum - nStop;
        } else if ( nSum == nStop ) {
            nSum = 0;
        }
    }

    return ren;
}