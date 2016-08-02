/*
 *
 *  Adapted by Sam Siewert for use with UVC web cameras and Bt878 frame
 *  grabber NTSC cameras to acquire digital video from a source,
 *  time-stamp each frame acquired, save to a PGM or PPM file.
 *
 *  The original code adapted was open source from V4L2 API and had the
 *  following use and incorporation policy:
 * 
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */
// Hello rahul
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

#include <linux/videodev2.h>
#include <sys/utsname.h>

#include <time.h>

//opencv library
 #include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/photo/photo.hpp>

#include <semaphore.h>
#include <pthread.h>

#define NUM_THREADS 2
/*POSIX*/ 
pthread_t threads[NUM_THREADS];
pthread_attr_t rt_sched_attr[NUM_THREADS];
int rt_max_prio, rt_min_prio;
struct sched_param rt_param[NUM_THREADS];
struct sched_param nrt_param;

#define startService       0
#define serviceCompression 1
#define serviceSocketSend  2

sem_t sendSocket;
sem_t compression;


#define CLEAR(x) memset(&(x), 0, sizeof(x))
#define COLOR_CONVERT
#define HRES 640
#define VRES 480
#define HRES_STR "640"
#define VRES_STR "480"

//To run system at higher rate 
//#define highRate

using namespace cv;
using namespace std;

// Format is used by a number of functions, so made as a file global
static struct v4l2_format fmt;

uint8_t *buffertocv;


//Capturing frame rate 
struct timespec start_time;
struct timespec stop_time;


//for uname
struct utsname unameData;

enum io_method 
{
        IO_METHOD_READ,
        IO_METHOD_MMAP,
        IO_METHOD_USERPTR,
};

struct buffer 
{
        void   *start;
        size_t  length;
};

static char            *dev_name;
//static enum io_method   io = IO_METHOD_USERPTR;
//static enum io_method   io = IO_METHOD_READ;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format=1;
static int              frame_count = 200;


static void errno_exit(const char *s)
{
        fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
        exit(EXIT_FAILURE);
}

//int ioctl(int fd, unsigned long request, ...);
//fd - file descriptor
//request - request code depending on device 
//arg - untyoed pointer to memory 
static int xioctl(int fh, int request, void *arg)
{
        int r;

        do 
        {
            r = ioctl(fh, request, arg);

        } while (-1 == r && EINTR == errno);

        return r;
}


long getEpochTimeShift(){
        struct timeval epochtime;
        struct timespec  vsTime;

        gettimeofday(&epochtime, NULL);
        clock_gettime(CLOCK_MONOTONIC, &vsTime);

        long uptime_ms = vsTime.tv_sec* 1000 + (long)  round( vsTime.tv_nsec/ 1000000.0);
        long epoch_ms =  epochtime.tv_sec * 1000  + (long) round( epochtime.tv_usec/1000.0);
        return epoch_ms - uptime_ms;
    }



tm * timeofFrameCpature( v4l2_buffer buf)
{
    //capturing the time of at which frame is DQ
            //without epoch set 
            printf("\n image captured(before correction) at %ld s, %ld ms\n",buf.timestamp.tv_sec, buf.timestamp.tv_usec/1000);


            //with epoch changes 
           time_t toEpochOffset_ms ;
           toEpochOffset_ms = getEpochTimeShift();


            time_t time_s;
            long int temp_ms ;
            long int epochTimeStamp_ms;
            long int temp_s;
            long int epochTimeStamp_s;


            time(&time_s);
            temp_ms = 1000 * buf.timestamp.tv_sec + (long int) round(  buf.timestamp.tv_usec / 1000.0);            
            epochTimeStamp_ms= temp_ms + toEpochOffset_ms ;

            temp_s = buf.timestamp.tv_sec + (long int) (round(  buf.timestamp.tv_usec / 1000.0)/1000.0);            
            epochTimeStamp_s= temp_s + (long int )(round(toEpochOffset_ms/1000)) ;

            
            printf( "\nFrame time epoch: %ld s\n",epochTimeStamp_s);
            printf( "\nFrame time epoch: %ld ms\n",epochTimeStamp_ms);
            printf( "\nTime in linux : %ld s\n",time_s);
            
            
            //struct timeval tv;
            //gettimeofday(&tv, 0);
            //printf("current time %ld sec, %ld msec\n", tv.tv_sec, tv.tv_usec/1000);
            
            struct tm * timeinfo;
            char timebuffer[50];



            timeinfo = localtime(&time_s);            
            //In format HH:MM PM
            strftime(timebuffer,50,"%I:%M:%S %p", timeinfo);
            printf("System Time  %s\n",timebuffer );
            //time(&rawtime);
            timeinfo = localtime(&epochTimeStamp_s);            
            //In format HH:MM PM
            strftime(timebuffer,50,"%I:%M:%S %p", timeinfo);
            printf("Frame Buffer capture time %s\n",timebuffer );
            
            
            return timeinfo ;

}

void error(const char *msg)
{
    perror(msg);
    exit(0);
}
int tagFrmdump = 0;

/***************************************************************************************************
//Basic reference - http://theembeddedsystems.blogspot.com/2011/05/background-subtraction-using-opencv.html#!/tcmbck
//upgraded from old format and added integrated multiple changes according to algorithm 

***************************************************************************************************/
Mat image,frameTime1,frameTime2,frameForeground,img1,img2;
int BckGnd(Mat imageIn)
{
    Mat frame;
    int erosion_elem = 0;
    int erosion_size = 2;
    int dilation_elem = 0;
    int dilation_size = 2;
    int dilation_type;
    static int countabs =0;   
    
    if (imageIn.empty()) 
    {
        printf("No Frame\n");
        return -1;
    }

    imageIn.copyTo(frame);

    if (frame.empty()) 
    {
        printf("No Frame\n");
        return -1;
    }

    if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
    else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
    else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }


    Mat element = getStructuringElement( dilation_type,Size( 2*dilation_size + 1, 2*dilation_size+1 ),Point( dilation_size, dilation_size ) );

    
    frame.copyTo(image);

    if (image.empty()) 
    {
        printf("no image\n");
        return -1;
    }

    cvtColor(image,img1,CV_BGR2GRAY);
    img1.copyTo(frameTime1);//frame in grayscale


    if (frameTime1.empty()) 
    {
        printf("No Frame Time 1\n");
        return -1;
    }

    if (countabs++ >2)
    {

        absdiff(frameTime1,frameTime2,frameForeground);
        imwrite("frameTime1.png",frameTime1);
        imwrite("frameTime2.png",frameTime2);
        threshold(frameForeground,frameForeground,10,255,CV_THRESH_BINARY);
        erode(frameForeground,frameForeground,element);

        dilate(frameForeground,frameForeground,element);
        imwrite("dilate.png",frameForeground);
        dilate(frameForeground,frameForeground,element);    
        erode(frameForeground,frameForeground,element);
        imwrite("erode.png",frameForeground);
    }   

    //copy previous frame  to current
    frameTime1.copyTo(frameTime2);
    //store this 
    imwrite("image.png",image);
    imwrite("frameForeground.png",frameForeground);


    return 0;
}

void *clientSender(void *input)
{
    static int abortTestSocketSend = 1; 
    while(1)
    {
        abortTestSocketSend++;
        printf("%d abortTestSocketSend %d frame_count\n",abortTestSocketSend, frame_count );
        if (abortTestSocketSend == frame_count)
        {
            printf("breaking loop\n");
            break;
        }
        //printf("%s\n", );
        printf("Entering clientSender\n");
        sem_wait(&sendSocket);
        printf(" mama tag raa %d    \n", tagFrmdump);
        int sockfd, portno, n;
        struct sockaddr_in serv_addr;
        struct hostent *server;
        portno = 50000;
        
        Mat image;
        //int tag = 3;
        char jpg_dumpname[]="laps00000000.jpg";
        //while(tag<=frame_count)
        //{
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
            snprintf(&jpg_dumpname[4], 9, "%08d", tagFrmdump);
            strncat(&jpg_dumpname[12], ".jpg", 5);
            //tag++;
            //printf("%d\n", tag);
            image = imread(jpg_dumpname, CV_LOAD_IMAGE_COLOR);
            if(! image.data )                              // Check for invalid input
            {
                cout <<  "Could not open or find the image" << std::endl ;
                //return -1;
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
        //}
    }

    printf("Exiting sendSocket \n");


}



/*int convertPpmToJpeg(char *ppm_dumpname, char *jpg_dumpname)
{

    Mat imagePpm;
    int retBck=0;
    static int count =0;


    imagePpm = imread(ppm_dumpname,1);
    //changes 
    if (imagePpm.empty())
    {
        printf("empty ppm\n");
    }
    if(count++>2)
    {
        retBck=BckGnd(imagePpm);
        //printf("count value %d\n",count );
    }
    //printf("value return for BckGnd %d\n",retBck);
    //end changes     
    //just for analysis
    //imwrite( jpg_dumpname, imagePpm);
    //

    return 0;
}*/

void *convertPpmToJpeg(void * input)
{
    char jpg_dumpname[]="laps00000000.jpg";
    char ppm_dumpname[]="laps00000000.jpg";
    static int abortTestJpeg = 1;
    while(1)
    {
        abortTestJpeg++;
        if (abortTestJpeg == frame_count)
            break;
        printf("abortTestJpeg %d\n", abortTestJpeg);
        printf("Entering compression\n");
        sem_wait(&compression);
        Mat imagePpm;
        snprintf(&ppm_dumpname[4], 9, "%08d", tagFrmdump);
        strncat(&ppm_dumpname[12], ".ppm", 5);
        snprintf(&jpg_dumpname[4], 9, "%08d", tagFrmdump);
        strncat(&jpg_dumpname[12], ".jpg", 5);
        imagePpm = imread(ppm_dumpname,1);
        imwrite( jpg_dumpname, imagePpm);
        sem_post(&sendSocket); // starting send of data
    }

}


int checkFrameCount = 0;

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *timestamp,v4l2_buffer buf)
{
    int written, i, total, dumpfd;
    char ppm_dumpname[]="laps00000000.ppm";
    char jpg_dumpname[]="laps00000000.jpg";
    char ppm_user_name[98];
    char ppm_header[]="#Frame00000 HH:MM:SS PM";
    char ppm_resolution[16]="\n"HRES_STR" "VRES_STR"";
    int logfd;
    char logs[]="logger.txt";


    tagFrmdump = tag; // Take care of this
    //Appending time to each frame 
    time_t rawtime;
    struct tm * timeinfo;
    char timebuffer[50];
    time(&rawtime);
    //timeinfo = localtime ( &rawtime );
    
    timeinfo = timeofFrameCpature(buf);
    //In format HH:MM PM
    strftime(timebuffer,50,"%I:%M:%S %p", timeinfo);

    //printf("Formatted date & time frame %c%d:%s\n",'F',tag,timebuffer );
 
    //creating name of file in accordance to frame 
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    snprintf(&jpg_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    strncat(&jpg_dumpname[12], ".jpg", 5);



    //open the file 
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);  
    logfd = open(logs, O_WRONLY | O_NONBLOCK|O_APPEND|O_CREAT, 00666);  
    
    //uname -a o/p embedded to header 
    //debug print 
    
    strncat(ppm_user_name,"P6",sizeof("P6")); 

    struct utsname my_uname;
    if(uname(&my_uname) == -1)
        printf("uname call failed!");
    else
    {
        //printf("System name: %s\nNodename:%s\nRelease:%s\nVersion:%s\nMachine:%s\n",my_uname.sysname, my_uname.nodename, my_uname.release,my_uname.version,my_uname.machine);
        strncat(ppm_user_name,"\n#",sizeof("\n#")); 
        strncat(ppm_user_name,my_uname.sysname,sizeof(my_uname.sysname)); 
        strncat(ppm_user_name,my_uname.nodename,sizeof(my_uname.nodename)); 
        strncat(ppm_user_name,my_uname.release,sizeof(my_uname.release)); 
        strncat(ppm_user_name,my_uname.version,sizeof(my_uname.version)); 
        //strncat(ppm_user_name,my_uname.machine,sizeof(my_uname.machine)); 
        //strncat(ppm_user_name,"\n",sizeof("\n")); 

    }
    written=write(dumpfd, ppm_user_name, sizeof(ppm_user_name));

    memset(ppm_user_name,0,sizeof(ppm_user_name));   

    //appending the time and time stamps   
    snprintf(&ppm_header[8],5,"%04d",tag);
    strncat(ppm_header,timebuffer,sizeof(timebuffer));

    written=write(dumpfd, ppm_header, sizeof(ppm_header));
    //debugging
    written=write(logfd, ppm_header, sizeof(ppm_header));
    written=write(logfd, "\n", sizeof("\n"));

    //if ()
    //
    strncat(ppm_resolution,"\n255\n",sizeof("\n255\n"));    
    written=write(dumpfd,ppm_resolution,sizeof(ppm_resolution));

    total=0;
    do
    {
        written=write(dumpfd, p, size);
        total+=written;
        
    } while(total < size);

    //printf("wrote %d bytes\n", total);
    close(dumpfd);
    close(logfd); 
    //convertPpmToJpeg(ppm_dumpname,jpg_dumpname);
    checkFrameCount++;
    //convertPpmToJpeg(ppm_dumpname,jpg_dumpname);   
    //clientSender(tag);
   /* if (checkFrameCount == frame_count)
    {
        printf("Entering client Sender loop\n");
        clientSender();
    }*/
    if(tag>2)
    {
        //clock_gettime(CLOCK_REALTIME, &start_time);
        //clientSender(tag);
        //clock_gettime(CLOCK_REALTIME, &stop_time);
        //printf("Time elapsed in transfer of files for %d image is %ld sec %ld nsec\n", frame_count, (stop_time.tv_sec)-(start_time.tv_sec), ((stop_time.tv_nsec)-(start_time.tv_nsec)));
        printf("Entering for loop\n");
        sem_post(&compression);
    }    

}


/*
char ppm_header[]="P6\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char ppm_dumpname[]="test00000000.ppm";

static void dump_ppm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&ppm_dumpname[4], 9, "%08d", tag);
    strncat(&ppm_dumpname[12], ".ppm", 5);
    dumpfd = open(ppm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&ppm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&ppm_header[14], " sec ", 5);
    snprintf(&ppm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&ppm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    written=write(dumpfd, ppm_header, sizeof(ppm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}
*/



char pgm_header[]="P5\n#9999999999 sec 9999999999 msec \n"HRES_STR" "VRES_STR"\n255\n";
char pgm_dumpname[]="test00000000.pgm";

static void dump_pgm(const void *p, int size, unsigned int tag, struct timespec *time)
{
    int written, i, total, dumpfd;
   
    snprintf(&pgm_dumpname[4], 9, "%08d", tag);
    strncat(&pgm_dumpname[12], ".pgm", 5);
    dumpfd = open(pgm_dumpname, O_WRONLY | O_NONBLOCK | O_CREAT, 00666);

    snprintf(&pgm_header[4], 11, "%010d", (int)time->tv_sec);
    strncat(&pgm_header[14], " sec ", 5);
    snprintf(&pgm_header[19], 11, "%010d", (int)((time->tv_nsec)/1000000));
    strncat(&pgm_header[29], " msec \n"HRES_STR" "VRES_STR"\n255\n", 19);
    written=write(dumpfd, pgm_header, sizeof(pgm_header));

    total=0;

    do
    {
        written=write(dumpfd, p, size);
        total+=written;
    } while(total < size);

    //printf("wrote %d bytes\n", total);

    close(dumpfd);
    
}


void yuv2rgb_float(float y, float u, float v, 
                   unsigned char *r, unsigned char *g, unsigned char *b)
{
    float r_temp, g_temp, b_temp;

    // R = 1.164(Y-16) + 1.1596(V-128)
    r_temp = 1.164*(y-16.0) + 1.1596*(v-128.0);  
    *r = r_temp > 255.0 ? 255 : (r_temp < 0.0 ? 0 : (unsigned char)r_temp);

    // G = 1.164(Y-16) - 0.813*(V-128) - 0.391*(U-128)
    g_temp = 1.164*(y-16.0) - 0.813*(v-128.0) - 0.391*(u-128.0);
    *g = g_temp > 255.0 ? 255 : (g_temp < 0.0 ? 0 : (unsigned char)g_temp);

    // B = 1.164*(Y-16) + 2.018*(U-128)
    b_temp = 1.164*(y-16.0) + 2.018*(u-128.0);
    *b = b_temp > 255.0 ? 255 : (b_temp < 0.0 ? 0 : (unsigned char)b_temp);
}


// This is probably the most acceptable conversion from camera YUYV to RGB
//
// Wikipedia has a good discussion on the details of various conversions and cites good references:
// http://en.wikipedia.org/wiki/YUV
//
// Also http://www.fourcc.org/yuv.php
//
// What's not clear without knowing more about the camera in question is how often U & V are sampled compared
// to Y.
//
// E.g. YUV444, which is equivalent to RGB, where both require 3 bytes for each pixel
//      YUV422, which we assume here, where there are 2 bytes for each pixel, with two Y samples for one U & V,
//              or as the name implies, 4Y and 2 UV pairs
//      YUV420, where for every 4 Ys, there is a single UV pair, 1.5 bytes for each pixel or 36 bytes for 24 pixels

void yuv2rgb(int y, int u, int v, unsigned char *r, unsigned char *g, unsigned char *b)
{
   int r1, g1, b1;

   // replaces floating point coefficients
   int c = y-16, d = u - 128, e = v - 128;       

   // Conversion that avoids floating point
   r1 = (298 * c           + 409 * e + 128) >> 8;
   g1 = (298 * c - 100 * d - 208 * e + 128) >> 8;
   b1 = (298 * c + 516 * d           + 128) >> 8;

   // Computed values may need clipping.
   if (r1 > 255) r1 = 255;
   if (g1 > 255) g1 = 255;
   if (b1 > 255) b1 = 255;

   if (r1 < 0) r1 = 0;
   if (g1 < 0) g1 = 0;
   if (b1 < 0) b1 = 0;

   *r = r1 ;
   *g = g1 ;
   *b = b1 ;
}


unsigned int framecnt=0;
unsigned char bigbuffer[(1280*960)];

static void process_image(const void *p, int size,v4l2_buffer buf)

{
    int i, newi, newsize=0;
    struct timespec frame_time;

    int y_temp, y2_temp, u_temp, v_temp;
    unsigned char *pptr = (unsigned char *)p;

    // record when process was called
    clock_gettime(CLOCK_REALTIME, &frame_time);    

    framecnt++;
    //printf("frame %d: ", framecnt);

    // This just dumps the frame to a file now, but you could replace with whatever image
    // processing you wish.
    //

    if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_GREY)
    {
        printf("Dump graymap as-is size %d\n", size);
        dump_pgm(p, size, framecnt, &frame_time);
    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV)
    {

    #if defined(COLOR_CONVERT)
        //printf("Dump YUYV converted to RGB size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want RGB, so RGBRGB which is 6 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+6)
        {
            y_temp=(int)pptr[i]; u_temp=(int)pptr[i+1]; y2_temp=(int)pptr[i+2]; v_temp=(int)pptr[i+3];
            yuv2rgb(y_temp, u_temp, v_temp, &bigbuffer[newi], &bigbuffer[newi+1], &bigbuffer[newi+2]);
            yuv2rgb(y2_temp, u_temp, v_temp, &bigbuffer[newi+3], &bigbuffer[newi+4], &bigbuffer[newi+5]);
        }

        dump_ppm(bigbuffer, ((size*6)/4), framecnt, &frame_time,buf);
    #else
        printf("Dump YUYV converted to YY size %d\n", size);
       
        // Pixels are YU and YV alternating, so YUYV which is 4 bytes
        // We want Y, so YY which is 2 bytes
        //
        for(i=0, newi=0; i<size; i=i+4, newi=newi+2)
        {
            // Y1=first byte and Y2=third byte
            bigbuffer[newi]=pptr[i];
            bigbuffer[newi+1]=pptr[i+2];
        }

        dump_pgm(bigbuffer, (size/2), framecnt, &frame_time);
    #endif

    }

    else if(fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_RGB24)
    {
        printf("Dump RGB as-is size %d\n", size);
        dump_ppm(p, size, framecnt, &frame_time,buf);
    }
    else
    {
        printf("ERROR - unknown dump format\n");
    }

    fflush(stderr);
    //fprintf(stderr, ".");
    fflush(stdout);
}


//DQueuing
//
static int read_frame(void)
{
    struct v4l2_buffer buf;
    unsigned int i;
    switch (io)
    {

        case IO_METHOD_READ:
            if (-1 == read(fd, buffers[0].start, buffers[0].length))
            {
                switch (errno)
                {

                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("read");
                }
            }

            process_image(buffers[0].start, buffers[0].length,buf);
            break;

        case IO_METHOD_MMAP:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            //buf.flags = V4L2_BUF_FLAG_TIMECODE;
            buf.flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
            //buf.flags = V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
            //Applications call the VIDIOC_QBUF ioctl to enqueue an empty (capturing) or filled (output) buffer in the driver's incoming queue
            //DQUEUE the buffer and capture which is furthre processed 

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, but drivers should only set for serious errors, although some set for
                           non-fatal errors too.
                         */
                        return 0;


                    default:
                        printf("mmap failure\n");
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            assert(buf.index < n_buffers);

            
            process_image(buffers[buf.index].start, buf.bytesused,buf);


            
             
            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;

        case IO_METHOD_USERPTR:
            CLEAR(buf);

            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;

            if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf))
            {
                switch (errno)
                {
                    case EAGAIN:
                        return 0;

                    case EIO:
                        /* Could ignore EIO, see spec. */

                        /* fall through */

                    default:
                        errno_exit("VIDIOC_DQBUF");
                }
            }

            for (i = 0; i < n_buffers; ++i)
                    if (buf.m.userptr == (unsigned long)buffers[i].start
                        && buf.length == buffers[i].length)
                            break;

            assert(i < n_buffers);

            process_image((void *)buf.m.userptr, buf.bytesused,buf);

            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                    errno_exit("VIDIOC_QBUF");
            break;
    }

    //printf("R");
    return 1;
}


static void mainloop(void)
{
    unsigned int count;
    int rsys;
    struct timespec read_delay;
    struct timespec time_error;
    time_error.tv_sec=0;
    time_error.tv_nsec=0;
    int fps=30; //choose the fps for video conversion using ffmpeg
    Mat high_rate_frame;

    struct timespec frame_time;
    {
        /* data */
    };

    read_delay.tv_sec=1;
    read_delay.tv_nsec=0;

    count = frame_count;

    while (count > 0)
    {
        for (;;)
        {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            //https://linuxtv.org/downloads/v4l-dvb-apis/func-select.html
            //suspend execution until the driver has captured data or is ready to accept data for output
            r = select(fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r)
            {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r)
            {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }
            
            //read time for start of frame 
            clock_gettime(CLOCK_REALTIME, &start_time);    
            #ifndef highRate
                if (read_frame())
                {

                    
                        int j = nanosleep(&read_delay, &time_error);
                        if( j!= 0)
                            perror("nanosleep");
                        else
                            count--;
                        break;
                    
                        //cvCapture();

                    
                    clock_gettime(CLOCK_REALTIME, &stop_time);  
                    //printf("%d\n", j);
                    
                }
            #else




            #endif     

            /* EAGAIN - continue select loop unless count done. */
            if(count <= 0) break;
        }

        if(count <= 0) break;
    }


    //Convert the frames to timelapse video 
    printf("converting files to video\n");
    
    //string command ="sh image2Video.sh";
    //strncat(command,string(fps));
    //rsys =system(command);
    rsys =system("sh image2Video.sh 30");
    
    if(rsys<0)
    {
        //printf("Errors in shell execution\n");

        perror("video conversion shell");
        exit(1);
    }
    else if (rsys == 0)
    {

        perror("shell not available");
        printf("Success in creation of video \n");     

    }
    else if (rsys == 127)
    {
        
        perror("child process issue");
        exit(1);
    }
    else
    {
        perror("shell not available");
        exit(1);
        
    }
}

static void stop_capturing(void)
{
        enum v4l2_buf_type type;

        switch (io) {
        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
                        errno_exit("VIDIOC_STREAMOFF");
                break;
        }
}

//Queuing
static void start_capturing(void)
{
        unsigned int i;
        enum v4l2_buf_type type;

        switch (io) 
        {

        case IO_METHOD_READ:
                /* Nothing to do. */
                break;

        case IO_METHOD_MMAP:
                //creating multiple buffers
                //Queuing the frames from stream into buffer 
                for (i = 0; i < n_buffers; ++i) 
                {
                        printf("allocated buffer %d\n", i);
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_MMAP;//The buffer is used for memory mapping I/O
                        buf.index = i;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }

                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                // Start streaming I/O
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i) {
                        struct v4l2_buffer buf;

                        CLEAR(buf);
                        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                        buf.memory = V4L2_MEMORY_USERPTR;
                        buf.index = i;
                        buf.m.userptr = (unsigned long)buffers[i].start;
                        buf.length = buffers[i].length;

                        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
                                errno_exit("VIDIOC_QBUF");
                }
                type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
                        errno_exit("VIDIOC_STREAMON");
                break;
        }
}

static void uninit_device(void)
{
        unsigned int i;

        switch (io) {
        case IO_METHOD_READ:
                free(buffers[0].start);
                break;

        case IO_METHOD_MMAP:
                for (i = 0; i < n_buffers; ++i)
                        if (-1 == munmap(buffers[i].start, buffers[i].length))
                                errno_exit("munmap");
                break;

        case IO_METHOD_USERPTR:
                for (i = 0; i < n_buffers; ++i)
                        free(buffers[i].start);
                break;
        }

        free(buffers);
}

static void init_read(unsigned int buffer_size)
{
        buffers = (buffer *) calloc(1, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        buffers[0].length = buffer_size;
        buffers[0].start = malloc(buffer_size);

        if (!buffers[0].start) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }
}

static void init_mmap(void)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count = 6;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) 
        {
                if (EINVAL == errno) 
                {
                        fprintf(stderr, "%s does not support "
                                 "memory mapping\n", dev_name);
                        exit(EXIT_FAILURE);
                } else 
                {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        if (req.count < 2) 
        {
                fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
                exit(EXIT_FAILURE);
        }

        buffers = (buffer *)calloc(req.count, sizeof(*buffers));

        if (!buffers) 
        {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
                struct v4l2_buffer buf;

                CLEAR(buf);

                buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                buf.memory      = V4L2_MEMORY_MMAP;
                buf.index       = n_buffers;

                if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
                        errno_exit("VIDIOC_QUERYBUF");

                buffers[n_buffers].length = buf.length;
                buffers[n_buffers].start =
                        mmap(NULL /* start anywhere */,
                              buf.length,
                              PROT_READ | PROT_WRITE /* required */,
                              MAP_SHARED /* recommended */,
                              fd, buf.m.offset);
                 //NEW       
                //buffers.length = buf.length;        
                buffertocv = (uint8_t *)mmap (NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd,buf.m.offset);        
                //
                printf("done buffertocv\n");
                if (MAP_FAILED == buffers[n_buffers].start)
                        errno_exit("mmap");
        }
}

static void init_userp(unsigned int buffer_size)
{
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = 4;
        req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
                if (EINVAL == errno) {
                        fprintf(stderr, "%s does not support "
                                 "user pointer i/o\n", dev_name);
                        exit(EXIT_FAILURE);
                } else {
                        errno_exit("VIDIOC_REQBUFS");
                }
        }

        buffers = (buffer *)calloc(4, sizeof(*buffers));

        if (!buffers) {
                fprintf(stderr, "Out of memory\n");
                exit(EXIT_FAILURE);
        }

        for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
                buffers[n_buffers].length = buffer_size;
                buffers[n_buffers].start = malloc(buffer_size);

                if (!buffers[n_buffers].start) {
                        fprintf(stderr, "Out of memory\n");
                        exit(EXIT_FAILURE);
                }
        }
}

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    unsigned int min;

    //to find Query device capabilities and store it

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) 
    {
        if (EINVAL == errno) {
            fprintf(stderr, "%s is no V4L2 device\n",
                     dev_name);
            exit(EXIT_FAILURE);
        }
        else
        {
                errno_exit("VIDIOC_QUERYCAP");
        }
    }

    //to check the cap supports video 
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        fprintf(stderr, "%s is no video capture device\n",
                 dev_name);
        exit(EXIT_FAILURE);
    }



    //
    switch (io)
    {
        case IO_METHOD_READ:
            if (!(cap.capabilities & V4L2_CAP_READWRITE))
            {
                fprintf(stderr, "%s does not support read i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;

        // Check video capabilities     
        case IO_METHOD_MMAP:
        case IO_METHOD_USERPTR:
            if (!(cap.capabilities & V4L2_CAP_STREAMING))
            {
                fprintf(stderr, "%s does not support streaming i/o\n",
                         dev_name);
                exit(EXIT_FAILURE);
            }
            break;
    }


    /* Select video input, video standard and tune here. */
    CLEAR(cropcap);


    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    //https://linuxtv.org/downloads/v4l-dvb-apis/devices.html#capture
    //set the  query the current image format applications set the type field
    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect; /* reset to default */
        
        //Get or set the current cropping rectangle
        if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop))
        {
            switch (errno)
            {
                case EINVAL:
                    /* Cropping not supported. */
                    break;
                default:
                    /* Errors ignored. */
                        break;
            }
        }

    }
    else
    {
        /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (force_format)
    {
        printf("FORCING FORMAT\n");
        fmt.fmt.pix.width       = HRES;
        fmt.fmt.pix.height      = VRES;

        // Specify the Pixel Coding Formate here

        // This one work for Logitech C200
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;

        // Would be nice if camera supported
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_GREY;
        //fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;

        //fmt.fmt.pix.field       = V4L2_FIELD_INTERLACED;
        fmt.fmt.pix.field       = V4L2_FIELD_NONE;

        //set video format 
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
                errno_exit("VIDIOC_S_FMT");

        /* Note VIDIOC_S_FMT may change width and height. */
    }
    else
    {
        printf("ASSUMING FORMAT\n");
        /* Preserve original settings as set by v4l2-ctl for example */
        if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
                    errno_exit("VIDIOC_G_FMT");
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
            fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
            fmt.fmt.pix.sizeimage = min;

       
    
    switch (io)
    {
        case IO_METHOD_READ:
            init_read(fmt.fmt.pix.sizeimage);
            break;

        case IO_METHOD_MMAP:
            init_mmap();
            break;

        case IO_METHOD_USERPTR:
            init_userp(fmt.fmt.pix.sizeimage);
            break;
    }
}


static void close_device(void)
{
        if (-1 == close(fd))
                errno_exit("close");

        fd = -1;
}

static void open_device(void)
{
        struct stat st;
        //Check status after opening the file 
        if (-1 == stat(dev_name, &st)) {
                fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
        //Check if device is present or not 
        if (!S_ISCHR(st.st_mode)) {
                fprintf(stderr, "%s is no device\n", dev_name);
                exit(EXIT_FAILURE);
        }


        //Open the capture video /dev/video0 or /dev/video1 according to dev_name
        //https://linuxtv.org/downloads/v4l-dvb-apis/func-open.html
        //O_RDWR - open with read and write 
        //O_NONBLOCK - 
        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0); 
        

        if (-1 == fd) {
                fprintf(stderr, "Cannot open '%s': %d, %s\n",
                         dev_name, errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}

static void usage(FILE *fp, int argc, char **argv)
{
        fprintf(fp,
                 "Usage: %s [options]\n\n"
                 "Version 1.3\n"
                 "Options:\n"
                 "-d | --device name   Video device name [%s]\n"
                 "-h | --help          Print this message\n"
                 "-m | --mmap          Use memory mapped buffers [default]\n"
                 "-r | --read          Use read() calls\n"
                 "-u | --userp         Use application allocated buffers\n"
                 "-o | --output        Outputs stream to stdout\n"
                 "-f | --format        Force format to 640x480 GREY\n"
                 "-c | --count         Number of frames to grab [%i]\n"
                 "",
                 argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
        { "device", required_argument, NULL, 'd' },
        { "help",   no_argument,       NULL, 'h' },
        { "mmap",   no_argument,       NULL, 'm' },
        { "read",   no_argument,       NULL, 'r' },
        { "userp",  no_argument,       NULL, 'u' },
        { "output", no_argument,       NULL, 'o' },
        { "format", no_argument,       NULL, 'f' },
        { "count",  required_argument, NULL, 'c' },
        { 0, 0, 0, 0 }
};

void *Sequencer(void *input)
{
    printf("Entering Sequencer\n");
   sem_init(&sendSocket,0,0);
   sem_init(&compression,0,0);

  // Creating threads 

   int rc;
   //printf("Starting Sequencer\n");
   //sem_init(&semF10,0,1);
   //sem_init(&semF20,0,1);
   
   // Creating attributes for Service fib10
   pthread_attr_init(&rt_sched_attr[serviceCompression]);
   pthread_attr_setinheritsched(&rt_sched_attr[serviceCompression], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[serviceCompression], SCHED_FIFO);
    
   // Creating attributes for Service fib20
   pthread_attr_init(&rt_sched_attr[serviceSocketSend]);
   pthread_attr_setinheritsched(&rt_sched_attr[serviceSocketSend], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[serviceSocketSend], SCHED_FIFO);
   
   rt_max_prio = sched_get_priority_max(SCHED_FIFO);
   rt_min_prio = sched_get_priority_min(SCHED_FIFO);
   
   printf("min prio = %d, max prio = %d\n", rt_min_prio, rt_max_prio);
   
   // Setting priority for Service fib10
   rt_param[serviceCompression].sched_priority = rt_max_prio - 10;
   pthread_attr_setschedparam(&rt_sched_attr[serviceCompression], &rt_param[serviceCompression]);

   printf("Creating thread serviceCompression %d\n", serviceCompression);
   rc = pthread_create(&threads[serviceCompression], &rt_sched_attr[serviceCompression], convertPpmToJpeg, (void *)serviceCompression);
   
   if (rc)
   {
       printf("ERROR; pthread_create for serviceCompression rc is %d\n", rc);
       perror(NULL);
       exit(-1);
   }
   
   // Setting priority for Service fib20
   rt_param[serviceSocketSend].sched_priority = rt_max_prio - 20;
   pthread_attr_setschedparam(&rt_sched_attr[serviceSocketSend], &rt_param[serviceSocketSend]);
   /*
   printf("Creating thread serviceSocketSend %d\n", serviceSocketSend);
   rc = pthread_create(&threads[serviceSocketSend], &rt_sched_attr[serviceSocketSend], clientSender, (void *)serviceSocketSend);


   if (rc)
   {
       printf("ERROR; pthread_create for serviceF20 rc is %d\n", rc);
       perror(NULL);
       exit(-1);
   }
   */


    open_device();
    printf("hello mama\n");
    init_device();
    start_capturing();
    mainloop();
    stop_capturing();
    uninit_device();
    close_device();
    fprintf(stderr, "\n");

    printf("mama ekkadiki ostunna\n");
    //abortTest = 0;
    //printf("abortTest %d\n", abortTest);

        if(pthread_join(threads[serviceSocketSend ], NULL) == 0)
        printf("serviceSocketSend done\n");
    else
        perror("serviceSocketSend");
    if(pthread_join(threads[serviceCompression ], NULL) == 0)
        printf("serviceCompression done\n");
    else
        perror("serviceCompression");




    printf("mama ekkadiki ostunna\n");
    //abortTest = 0;
    printf("serviceSocketSend Completed\n");
    printf("serviceCompression Completed\n");
}

int main(int argc, char **argv)
{
    if(argc > 1)
        dev_name = argv[1];
    else
        dev_name = "/dev/video0";

    for (;;)
    {
        int idx;
        int c;

        c = getopt_long(argc, argv,
                    short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c)
        {
            case 0: /* getopt_long() flag */
                break;

            case 'd':
                dev_name = optarg;
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                io = IO_METHOD_MMAP;
                break;

            case 'r':
                io = IO_METHOD_READ;
                break;

            case 'u':
                io = IO_METHOD_USERPTR;
                break;

            case 'o':
                out_buf++;
                break;

            case 'f':
                force_format++;
                break;

            case 'c':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                        errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }


    int rc;
   
   // Creating attributes for Service startService
   pthread_attr_init(&rt_sched_attr[startService]);
   pthread_attr_setinheritsched(&rt_sched_attr[startService], PTHREAD_EXPLICIT_SCHED);
   pthread_attr_setschedpolicy(&rt_sched_attr[startService], SCHED_FIFO);
   
   rt_max_prio = sched_get_priority_max(SCHED_FIFO);
   rt_min_prio = sched_get_priority_min(SCHED_FIFO);
   
   printf("min prio = %d, max prio = %d\n", rt_min_prio, rt_max_prio);
   
   //Setting priority for Service startService
   rt_param[startService].sched_priority = rt_max_prio;
   pthread_attr_setschedparam(&rt_sched_attr[startService], &rt_param[startService]);

   printf("Creating thread %d\n", startService);
   rc = pthread_create(&threads[startService], &rt_sched_attr[startService], Sequencer, (void *)startService);
   
   if (rc)
   {
       printf("ERROR; pthread_create for startService rc is %d\n", rc);
       perror(NULL);
       exit(-1);
   }
   if(pthread_join(threads[startService], NULL) == 0)
    printf("startService done\n");
   else
        perror("startService");

    printf("Sequencer Completed\n");

}
