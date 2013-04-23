extern "C"{
#include <stdio.h>
}
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <android/log.h>

#include <arpa/inet.h>
#include <netdb.h>
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
#define MatrixNumber 0//0 for server
//#define MatricesReadSize MatrixSize*NumMatrices

class Server
{
public:

	int server_skt, client_skt, numbytes;
	//int track_skt;
	struct sockaddr_in server_addr;
	struct sockaddr_in clients_addr;
	//struct sockaddr_in track_addr;
	int sin_size;
	char buf[bufferSize];
	//char trackBuf[bufferSize];
    ScamInfo m_acceInfo;
    ScamInfo other_acceInfo;

    Server();
	void startServer();
	void sendMSG();
	void recvMSG();
	void sendMSG(int &numBytes);
	void recvMSG(int MaxNumBytes);
    void sendMSG(char* buf,size_t bufSize);
    bool sendINT(int in);
    bool recvINT(int & out);
    bool sendFloat(float in);
    bool recvFloat(float & out);
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
