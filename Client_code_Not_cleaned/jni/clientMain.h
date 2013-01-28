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
//#define PORT 8999
//#define TRACKPORT 44444
//#define TRACKADDR "137.110.118.26"//sessions
//#define TRACKADDR "137.110.119.176"//rubble
#define bufferSize 1024
#define MatrixSize 16*4
#define NumMatrices 2
#define MatrixNumber 1//1 for client
//#define MatricesReadSize MatrixSize*NumMatrices

class Client
{
public:
    int client_skt;  /* client socket */
    //int track_skt;
    int rc;
    struct sockaddr_in local_addr, serv_addr;
    //struct sockaddr_in track_addr;
    char * serverIP;
    int serverIPlength;
    int numbytes;
    char buf[bufferSize];
    //char trackBuf[bufferSize];
    ScamInfo m_acceInfo;
    ScamInfo other_acceInfo;

    //float TTmatrix[16];//Transform matrix from Tracking system
    //char trackRecvBuf[MatricesReadSize];
    //float translation[3];
    //float position[3];
    //bool firstRead;

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
    //void updateTrack();
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
