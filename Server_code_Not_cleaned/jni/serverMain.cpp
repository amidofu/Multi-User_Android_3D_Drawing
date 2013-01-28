#include "serverMain.h"
Server::Server()
{
	firstRead=true;
	translation[0]=translation[1]=translation[2]=0.0f;
}
void Server::startServer()
{
    memset(&track_addr, 0, sizeof(track_addr));
    //hints.ai_family = AF_UNSPEC;
    //hints.ai_socktype = SOCK_DGRAM;

    track_addr.sin_family = AF_INET;
    track_addr.sin_addr.s_addr=inet_addr(TRACKADDR);
    track_addr.sin_port=htons(TRACKPORT);

    if((track_skt=socket(AF_INET,SOCK_DGRAM ,0))==-1){
    	__android_log_print(ANDROID_LOG_ERROR,"jni server Track","socket creation error");
    	return;
    }
    else
    {
    	fcntl(track_skt,F_SETFL, O_NONBLOCK);
    	__android_log_print(ANDROID_LOG_INFO,"jni server Track","socket creation OK");
    }


	//TCP socket
    /* now create the server socket
       make it an IPV4 socket (PF_INET) and stream socket (TCP)
       and 0 to select default protocol type */
	if ( (server_skt = socket(AF_INET, SOCK_STREAM, 0)) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","socket creation failed");
		exit(1);
	}

	//Initail, bind to port
    /* now fill in values of the server sockaddr_in struct
       s_addr and sin_port are in Network Byte Order (Big Endian)
       Since Intel CPUs use Host Byte Order (Little Endian), conversion
       is necessary (e.g. htons(), and htonl() */
	server_addr.sin_family = AF_INET;//IPv4
	server_addr.sin_port = htons(PORT);
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	memset(server_addr.sin_zero, 0, 8);

	//binding

	 /* now bind server port
	       associate socket (server) with IP address:port (server_addr) */

	if ( bind(server_skt, (struct sockaddr*)&server_addr, sizeof(struct sockaddr)) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","socket bind failed");
		exit(1);
	}

	int queueSize=1;
	//Start listening, wait for connection from client with a pending queue of size
	if ( listen(server_skt, queueSize) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","socket listen failed");
		exit(1);
	}

	//Wait for connect!
	bool gotClient=false;
	while(!gotClient){
		sin_size = sizeof(struct sockaddr_in);
		__android_log_print(ANDROID_LOG_INFO,"jni server","server starts");
		if ( (client_skt = accept(server_skt, (struct sockaddr*)&clients_addr, &sin_size)) == -1 ){
			perror("accept");
			__android_log_print(ANDROID_LOG_ERROR,"jni server",strerror(errno));
			exit(1);
		}
		else
		{
			gotClient=true;
			__android_log_print(ANDROID_LOG_INFO,"got client from IP address","%s",(char *) &clients_addr.sin_addr.s_addr);
		}

		/*//for multiple clients
		if ( !fork() ){
			//Receive
			if ( (numbytes = read(client_fd, buf, sizeof(buf))) == -1 ){
			  	perror("recv");
				exit(1);
			}

			//Send back
			if ( (numbytes = write(client_fd, buf, strlen(buf))) == -1){
				perror("send");
				exit(0);
			}

			close(client_fd);
		}
	}
	close(sockfd);
	*/
	}
}

void Server::sendMSG()
{
	//if ( (numbytes = write(client_fd, buf, strlen(buf))) == -1){
	if ( (numbytes = send(client_skt, buf, strlen(buf),0)) == -1){
	//if ( (numbytes = send(sockfd, buf, strlen(buf),0)) == -1){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","normal send error");
		//exit(0);
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni server","send %d bytes",numbytes);
	//__android_log_print(ANDROID_LOG_INFO,"jni server","send result: %s ",buf);
}
void Server::recvMSG()
{
	//if ( (numbytes = read(client_fd, buf, sizeof(buf))) == -1 ){
	if ( (numbytes = recv(client_skt, buf, bufferSize,0)) == -1 ){
	//if ( (numbytes = recv(sockfd, buf, bufferSize,0)) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","normal receive error");
		//exit(1);
	}
	//__android_log_print(ANDROID_LOG_INFO,"jni server","receive %d bytes",numbytes);
	//__android_log_print(ANDROID_LOG_INFO,"jni server","receive result: %s ",buf);
}

void Server::sendMSG(char* buf,size_t bufSize)
{
	//if ( (numbytes = send(client_skt, buf, strlen(buf),0) )== -1 ){
	if ( (numbytes = send(client_skt, buf, bufSize,0) )== -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","max size limited send error");
		//exit(1);
	}
	////__android_log_print(ANDROID_LOG_INFO,"jni server","sendMSG2 send %d bytes",numbytes);
}
void Server::recvMSG2()
{
	if ( (numbytes = recv(client_skt, buf, bufferSize,0) ) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","recv error");
		//exit(1);
	}
	////__android_log_print(ANDROID_LOG_INFO,"jni server","recvMSG2 read %d bytes",numbytes);
	//buf[numbytes] = '\0';
}
void Server::sendMSG(int &numBytes)
{
	//if ( (numbytes = send(client_skt, buf, strlen(buf),0) )== -1 ){
	if ( (numbytes = send(client_skt, buf, bufferSize,0) )== -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","get numbytes send error");
		//exit(1);
	}
	////__android_log_print(ANDROID_LOG_INFO,"jni server","sendMSG2 send %d bytes",numbytes);
	numBytes=numbytes;
}
void Server::recvMSG(int MaxNumBytes)
{
	if ( (numbytes = recv(client_skt, buf, MaxNumBytes,0) ) == -1 ){
		__android_log_print(ANDROID_LOG_ERROR,"jni server","max limited recv error");
		//exit(1);
	}
	////__android_log_print(ANDROID_LOG_INFO,"jni server","recvMSG2 read %d bytes",numbytes);
}
bool Server::sendLongBuffer(char* & buf, int arraylength)
{
	int total=0;
	int bytesleft=arraylength;
	int n;
	int i=1;
	__android_log_print(ANDROID_LOG_INFO,"jni server","array length: %d",arraylength);
	while(total<arraylength)
	{
		__android_log_print(ANDROID_LOG_INFO,"jni server","run %d times, bytes left: %d",i,bytesleft);
		n=send(client_skt,buf+total,bytesleft,0);
		if(n==-1)
		{
			__android_log_print(ANDROID_LOG_ERROR,"jni server","send long buffer error");
			break;
		}
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
bool Server::recvLongBuffer(char* & buffer, int arraylength)
{
	//free(buf);
	buffer=new char[arraylength];
	int size=4096;
	char b[size];
	int total=0;
	int bytesleft=arraylength;
	int n;
	__android_log_print(ANDROID_LOG_INFO,"jni server","in recvLongBuffer, arraylendght:%d",arraylength);
	//int i=1;
	int numBytesToRead;
	while(total<arraylength)
	{
		if(size<bytesleft)
			numBytesToRead=size;
		else
			numBytesToRead=bytesleft;
		n=recv(client_skt, b, numBytesToRead,0);

		if(n==-1)
		{
			__android_log_print(ANDROID_LOG_ERROR,"jni server","recv long buffer error");
			break;
		}
		else
			memcpy(buffer+total,b,n);
		total+=n;
		bytesleft-=n;
	}
	if(n==-1)
		return false;
	if(total==arraylength)
		return true;
	else
		return false;

}
void Server::updateTrack()
{
    if ((numbytes = sendto(track_skt, trackBuf, strlen(trackBuf), 0,(struct sockaddr *)&track_addr, sizeof(track_addr))) == -1) {
    	__android_log_print(ANDROID_LOG_ERROR,"jni server Track","send error");
        exit(1);
    }
    //else
    //	__android_log_print(ANDROID_LOG_INFO,"jni server Track","send %d bytes",numbytes);

    int len=sizeof(track_addr);

    //if((numbytes=recvfrom(track_skt,trackBuf,bufferSize,0,(struct sockaddr *)&track_addr,&len))==-1){
    if((numbytes=recvfrom(track_skt,trackRecvBuf,MatricesReadSize,0,(struct sockaddr *)&track_addr,&len))==-1){
    	__android_log_print(ANDROID_LOG_ERROR,"jni server Track","recv error");
    	//exit(1);
    }
    else
    {
    	////__android_log_print(ANDROID_LOG_INFO,"jni server Track","read %d bytes",numbytes);
    	if(!firstRead)
    	 {
    		memcpy(TTmatrix,trackRecvBuf+MatrixNumber*MatrixSize,MatrixSize);
    		//for(int i=0;i<4;i++)
    		//__android_log_print(ANDROID_LOG_DEBUG,"jni client Track","recv matrix: %f, %f, %f, %f", TTmatrix[i*4],TTmatrix[i*4+1],TTmatrix[i*4+2],TTmatrix[i*4+3]);
    		translation[0]=TTmatrix[3]-position[0];
    		position[0]=TTmatrix[3];
    		translation[1]=TTmatrix[7]-position[1];
    		position[1]=TTmatrix[7];
    		translation[2]=TTmatrix[11]-position[2];
    		position[2]=TTmatrix[11];
    	 }
    	else
    	{
    		firstRead=false;
    		memcpy(TTmatrix,trackRecvBuf,MatrixSize);
    		position[0]=TTmatrix[3];
    		position[1]=TTmatrix[7];
    		position[2]=TTmatrix[11];
    		translation[0]=translation[1]=translation[2]=0.0f;
    	}
    }
}
bool Server::sendINT(int in)
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
bool Server::recvINT(int & out)
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

bool Server::sendFloat(float in)
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
bool Server::recvFloat(float & out)
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
