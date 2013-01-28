#include "clientMain.h"
Client::Client()
{
}
void Client::startClient()
{
	/*
	//connect to tracking system

	memset(&track_addr, 0, sizeof(track_addr));

    track_addr.sin_family = AF_INET;
    track_addr.sin_addr.s_addr=inet_addr(TRACKADDR);
    track_addr.sin_port=htons(TRACKPORT);

    if((track_skt=socket(AF_INET,SOCK_DGRAM ,0))==-1){
    	__android_log_print(ANDROID_LOG_ERROR,"jni client Track","socket creation error");
    	return;
    }
    else
    {
    	fcntl(track_skt,F_SETFL, O_NONBLOCK);
    	__android_log_print(ANDROID_LOG_INFO,"jni client Track","socket creation OK");
    }

	//END connect to tracking system
	*/
	//__android_log_print(ANDROID_LOG_INFO,"jni client","client start 1, server IP length:%d",serverIPlength);
	//__android_log_print(ANDROID_LOG_INFO,"jni client","client start 1, server IP:%s",serverIP);
    /* now fill in sockaddr_in for remote address */
    serv_addr.sin_family = AF_INET;
    /* save the server IP (input from Java */
    serv_addr.sin_addr.s_addr = inet_addr(serverIP);
    /* set port */
    serv_addr.sin_port = htons(PORT);
    memset(serv_addr.sin_zero, 0, 8);

    //__android_log_print(ANDROID_LOG_INFO,"jni client","client start 2, server IP length:%d",serverIPlength);
    //__android_log_print(ANDROID_LOG_INFO,"jni client","client start 2, server IP:%s",serverIP);
    /* create local stream socket */
    client_skt = socket(PF_INET, SOCK_STREAM, 0);
    if (client_skt < 0) {
    	__android_log_print(ANDROID_LOG_ERROR,"jni client","socket creation error");
        exit(-1);
    }

    /* bind local socket to any port number */
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    //local_addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    local_addr.sin_port = htons(PORT);
    memset(local_addr.sin_zero, 0, 8);

    rc = bind(client_skt, (struct sockaddr *) &local_addr, sizeof(local_addr));

    if (rc < 0)
    {
    	__android_log_print(ANDROID_LOG_ERROR,"jni client","socket bind error");
        exit(1);
    }
    __android_log_print(ANDROID_LOG_INFO,"jni client","client start finished");
}
void Client::connectServer()
{
	__android_log_print(ANDROID_LOG_INFO,"jni client","connect to server starts");
    /* connect to server */
    rc = connect(client_skt, (struct sockaddr *) &serv_addr, sizeof(serv_addr));
    if (rc < 0)
    {
    	__android_log_print(ANDROID_LOG_ERROR,"jni client","socket connect to server error");
    	__android_log_print(ANDROID_LOG_ERROR,"jni client",strerror(errno));
        exit(1);
    }
    __android_log_print(ANDROID_LOG_ERROR,"jni client","server connected");
}
void Client::sendMSG()
{
	if ( (numbytes = send(client_skt, "hello!", strlen("hello!"),0) )== -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni client","send error");
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni client","send %d bytes",numbytes);

}
void Client::recvMSG()
{
	//Receive from server
	if ( (numbytes = recv(client_skt, buf, bufferSize,0) ) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni client","recv error");
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni client","read %d bytes",numbytes);
	buf[numbytes] = '\0';
	//__android_log_print(ANDROID_LOG_INFO,"jni client","result %s",buf);

	//close(sockfd);


}
void Client::sendMSG(char* buf,size_t bufSize)
{
	if ( (numbytes = send(client_skt, buf, bufSize,0) )== -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni client","max size limited send error");
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni client","buf size: %d",strlen(buf));
	//__android_log_print(ANDROID_LOG_INFO,"jni client","sendMSG2 send %d bytes",numbytes);
}
void Client::recvMSG2()
{
	if ( (numbytes = recv(client_skt, buf, bufferSize,0) ) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni client","recv error");
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni client","recvMSG2 read %d bytes",numbytes);
}

void Client::sendMSG(int &numBytes)
{
	if ( (numbytes = send(client_skt, buf, bufferSize,0) )== -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni client","get numbytes send error");
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni client","sendMSG2 send %d bytes",numbytes);
	numBytes=numbytes;
}
void Client::recvMSG(int MaxNumBytes)
{
	//__android_log_print(ANDROID_LOG_INFO,"jni client","recvMSG2 read 1");
	if ( (numbytes = recv(client_skt, buf, MaxNumBytes,0) ) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni client","max limited recv error");
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni client","recvMSG2 read 2");
	//__android_log_print(ANDROID_LOG_INFO,"jni client","recvMSG2 read %d bytes",numbytes);
}
bool Client::sendLongBuffer(char* & buf, int arraylength)
{
	int total=0;
	int bytesleft=arraylength;
	int n;
	int i=1;
	while(total<arraylength)
	{
		__android_log_print(ANDROID_LOG_INFO,"jni client","run %d times, bytes left: %d",i,bytesleft);
		n=send(client_skt,buf+total,bytesleft,0);
		if(n==-1)
			break;
		total+=n;
		bytesleft-=n;
		i++;
	}
	if(n==-1)
		return false;
	if(total==arraylength)
		return true;
	else
		return false;

}
bool Client::recvLongBuffer(char* & buffer, int arraylength)
{
	buffer=new char[arraylength];
	int size=4096;
	char b[size];
	int total=0;
	int bytesleft=arraylength;
	int n;
	__android_log_print(ANDROID_LOG_INFO,"jni client","in recvLongBuffer, arraylendght:%d",arraylength);
	int numBytesToRead;
	while(total<arraylength)
	{
		if(size<bytesleft)
			numBytesToRead=size;
		else
			numBytesToRead=bytesleft;
		n=recv(client_skt, b, numBytesToRead,0);
		//__android_log_print(ANDROID_LOG_INFO,"jni client","run %d times, bytes left: %d",i,bytesleft);
		if(n==-1)
		{
			__android_log_print(ANDROID_LOG_ERROR,"jni client","recv long buffer error");
			break;
		}
		else
			memcpy(buffer+total,b,n);
		total+=n;
		bytesleft-=n;
		//i++;
	}
	if(n==-1)
		return false;
	if(total==arraylength)
		return true;
	else
		return false;

}

bool Client::sendINT(int in)
{
	char b[4];
	memcpy(b,&in,4);
	if(send(client_skt, b, 4,0)==-1 )
	{
		__android_log_print(ANDROID_LOG_ERROR,"jni server","send INT error");
		return false;
	}
	else
		return true;

}
bool Client::recvINT(int & out)
{
	char b[4];
	if(recv(client_skt, b,4,0) ==-1  )
	{
		__android_log_print(ANDROID_LOG_ERROR,"jni server","recv INT error");
		return false;
	}
	else
	{
		memcpy(&out,b,4);
		return true;
	}
}

