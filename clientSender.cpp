#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

#include "clientSender.hpp"
using namespace cv;
using namespace std;

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

int clientSender()
{
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    //char buffer[256];
    /*if (argc < 2) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }*/
    portno = 50000;
    //portno = atoi(argv[2]);
    //printf("%d\n", portno);
    //portno = 10000;    
    /*sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname("localhost");
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);*/



    //namedWindow( "Client", CV_WINDOW_AUTOSIZE );// Create a window for display.
    
    Mat image;
    int tag = 2;
    char jpg_dumpname[]="laps00000000.jpg";
    while(tag<=8)
    {
    	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	    if (sockfd < 0) 
	        error("ERROR opening socket");
	    server = gethostbyname("localhost");
	    if (server == NULL) {
	        fprintf(stderr,"ERROR, no such host\n");
	        exit(0);
	    }
	    bzero((char *) &serv_addr, sizeof(serv_addr));
	    serv_addr.sin_family = AF_INET;
	    bcopy((char *)server->h_addr, 
	         (char *)&serv_addr.sin_addr.s_addr,
	         server->h_length);
	    serv_addr.sin_port = htons(portno);
    	//printf("start\n");
    	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
    	{
    	 	//printf("rey mama\n");
        	error("ERROR connecting");
    	}
    	snprintf(&jpg_dumpname[4], 9, "%08d", tag);
        strncat(&jpg_dumpname[12], ".jpg", 5);
        tag++;
        //printf("%d\n", tag);
	    image = imread(jpg_dumpname, CV_LOAD_IMAGE_COLOR);
	    if(! image.data )                              // Check for invalid input
	    {
	        cout <<  "Could not open or find the image" << std::endl ;
	        return -1;
	    }
	    
	    //imshow( "Client", image ); 
		
		image = (image.reshape(0,1)); // to make it continuous
		int  imgSize = image.total()*image.elemSize();
		n = send(sockfd, image.data, imgSize, 0);
	    if (n < 0) 
	         error("ERROR writing to socket");
	    close(sockfd);
	    //sockfd = -1;
	    //printf("end\n");
	}

    return 0;
}
