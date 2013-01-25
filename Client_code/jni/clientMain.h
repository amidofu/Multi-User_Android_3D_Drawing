#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <android/log.h>
#include <fcntl.h>
#include "AuxFile.h"

#define PORT 8888
#define bufferSize 1024
#define MatrixSize 16*4
#define NumMatrices 2
#define MatrixNumber 1//1 for client

class Client
{
public:
    int client_skt;  /* client socket */
    int rc;
    struct sockaddr_in local_addr, serv_addr;
    char * serverIP;
    int serverIPlength;
    int numbytes;
    char buf[bufferSize];
    ScamInfo m_acceInfo;
    ScamInfo other_acceInfo;


    Client();
    void startClient();
    void connectServer();
    void sendMSG();
    void recvMSG();
    void sendMSG(char* buf,size_t bufSize);
	void sendMSG(int &numBytes);
	void recvMSG(int MaxNumBytes);
    bool sendINT(int in);
    bool recvINT(int & out);
    bool sendLongBuffer(char* & buf, int arraylength);
    bool recvLongBuffer(char* & buf, int arraylength);
    void recvMSG2();
    template <class T> inline void storeSimpleStructIntoBuf(char* & buf, T & t){
    	size_t s=sizeof(T);
    	buf=new char[s];
    	memcpy(buf,&t,s);
    }
    template <class T> inline void loadSimpleStructFromBuf(char*  buf, T & t){
    	size_t s=sizeof(T);
    	memcpy(&t,buf,s);
    }

};
