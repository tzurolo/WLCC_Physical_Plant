/*

protocols:
    images:
        <identifier:imagedata>
        identifier is the IMG element ID

    JSON:
        { ... }

    TODO:

*/
//#define _POSIX_C_SOURCE 200809L

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <vector>
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <opencv2/opencv.hpp>
extern "C" {
#include <b64/cencode.h>
}

#define CONTINUOUS_STREAM 1

typedef std::map<std::string, cv::Scalar> ThresholdData;
static ThresholdData thresholdData;
static char b64code[500000];

struct ROIDescriptor {
    std::string name;
    int topLeftX;   // percent
    int topLeftY;   // percent
    int width;
    int height;
};
typedef std::vector<ROIDescriptor> ROIList;

namespace {

void readColorThresholdData (
    const std::string &thresholdsFilename,
    ThresholdData &thresholds)
{
    std::ifstream thresholdsFile(thresholdsFilename.c_str());
    if (thresholdsFile.is_open()) {
        std::string line;
        while ( getline (thresholdsFile ,line) ) {
            // std::cout << line << '\n'
            std::istringstream iss (line);
            std::string name;
            double y, cr, cb;
            iss >> name >> y >> cr >> cb;
            const cv::Scalar t(y, cr, cb);
            thresholds[name] = t;
        }
        thresholdsFile.close();

#if 0
        // display data we read above
        for (ThresholdData::const_iterator i = thresholds.begin();
                i != thresholds.end(); ++i) {
            std::cout << "  '" << i->first << "': " << 
            i->second[0] << "," << i->second[1] << "," << i->second[2] << std::endl;
        }
#endif
    } else {
        std::cerr << "Unable to open file " << thresholdsFilename << std::endl;
    }
}

void error(const char *msg)
{
    perror(msg);
    exit(0);
}

void getCurrentTimeStrings (
    std::string &forImageAnnotation,    // 2015-12-27 01:54:18.86 (somewhat human-readable)
    std::string &forFilespec,           // 20151227T01541886 (like iso but without the punctuation)
    std::string &forServerDatabase)     // 2015-12-27T01:54:18.86 (sortable, iso-like string)
{
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
    struct tm localnow = *localtime(&spec.tv_sec);
    struct tm gmtnow = *gmtime(&spec.tv_sec);
    const int hs = floor(spec.tv_nsec / 1.0e7); // Convert nanoseconds to hundredths of seconds
    char dateTimeString[40];

    // for image annotation
    sprintf(dateTimeString, "%04d-%02d-%02d %02d:%02d:%02d.%02d",
        localnow.tm_year + 1900,
        localnow.tm_mon + 1,
        localnow.tm_mday,
        localnow.tm_hour,
        localnow.tm_min,
        localnow.tm_sec,
        hs);
    forImageAnnotation = dateTimeString;

    // for filespec
    sprintf(dateTimeString, "%04d%02d%02dT%02d%02d%02d%02d",
        gmtnow.tm_year + 1900,
        gmtnow.tm_mon + 1,
        gmtnow.tm_mday,
        gmtnow.tm_hour,
        gmtnow.tm_min,
        gmtnow.tm_sec,
        hs);
    forFilespec = dateTimeString;

    // for server database
    sprintf(dateTimeString, "%04d-%02d-%02dT%02d:%02d:%02d.%02dZ",
        gmtnow.tm_year + 1900,
        gmtnow.tm_mon + 1,
        gmtnow.tm_mday,
        gmtnow.tm_hour,
        gmtnow.tm_min,
        gmtnow.tm_sec,
        hs);
    forServerDatabase = dateTimeString;
}

void readCamImage (
    const int sockfd,
    const ROIList &ROIs,
    cv::VideoCapture &cap,
    int &tankLevel, // in units of percent
    std::string &serverJSON,
    std::string &serverImage)
{
    tankLevel = 0;

    cv::Mat edges;
    cv::Mat tedges;
    cv::Mat ycrcb;
    cv::Mat floatMask;
    cv::Mat baseMask;

    // get image from camera
    cv::Mat frame;
    for (int f = 0; f < 1; ++f) {   // this loop is a hack to flush out old frames from the buffer
                                    // TODO: use a thread to continuously grab
        cap >> frame; // get a new frame from camera
    }

    // get timestamp strings
    std::string forImageAnnotation;
    std::string forFilespec;
    std::string forServerDatabase;
    getCurrentTimeStrings(forImageAnnotation, forFilespec, forServerDatabase);
    char annotationBuf[80];
    sprintf(annotationBuf, "%s %d%%", forImageAnnotation.c_str(), tankLevel);
#if 0
    cv::putText(frame, annotationBuf, cv::Point(5, frame.size().height - 10), 
        cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 0), 2);
#endif
    for (ROIList::const_iterator rIter = ROIs.begin(); rIter != ROIs.end(); ++rIter) {
        const ROIDescriptor& roiDesc = *rIter;
#if 1
     cv::Rect roi((frame.size().width * roiDesc.topLeftX) / 100,
                  (frame.size().height * roiDesc.topLeftY) / 100,
                  150, 150);
#if 0
    // whole frame
    cv::Mat dst = frame;
#else
    // ROI
    cv::Mat dst = cv::Mat(frame, roi);
#endif

    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(80); // 1-100
    std::vector<uint8_t> jpeg_buffer;
    cv::imencode(".jpg", dst, jpeg_buffer, compression_params);

    base64_encodestate b64State;
    base64_init_encodestate(&b64State);
    const int b64Len = base64_encode_block((const char*)&jpeg_buffer[0], (int)jpeg_buffer.size(),
        b64code, &b64State);
    const int b64Len2 = base64_encode_blockend(&b64code[b64Len], &b64State);
    //std::cerr << "len1: " << b64Len << ", len2: " << b64Len2 << std::endl;

    serverImage = std::string("<" + roiDesc.name + ":data:image/jpeg;base64,");
#if 0
    serverImage += "iVBORw0KGgoAA"
        "AANSUhEUgAAABAAAAAQAQMAAAAlPW0iAAAABlBMVEUAAAD///+l2Z/dAAAAM0l"
        "EQVR4nGP4/5/h/1+G/58ZDrAz3D/McH8yw83NDDeNGe4Ug9C9zwz3gVLMDA/A6"
        "P9/AFGGFyjOXZtQAAAAAElFTkSuQmCC";
#else
    // the encoder puts in cr/lf every 72 chars
    const int len = (b64Len + b64Len2);
    for (int i = 0; i < len; ++i) {
        const char ch = b64code[i];
        if ((ch != 10) && (ch != 13)) {
            serverImage += ch;
        }
    }
    serverImage += '>';
#endif

#else
    const std::string imageFilespec = std::string("camImages/img") + forFilespec + ".png";
    cv::imwrite(imageFilespec, frame);
#endif
    // create JSON to return to the server
    char tankLevelStr[20];
    sprintf(tankLevelStr, "%d", tankLevel);
    serverJSON = std::string("{ \"level\": \"") + tankLevelStr +
//        "\", \"imageFile\": \"" + imageFilespec +
        "\", \"timestamp\": \"" + forServerDatabase +
        "\"}";

    // write image to server
    const int n = write(sockfd,serverImage.c_str(), serverImage.size());
    if (n < 0) {
            error("ERROR writing to socket");
    } else {
        //std::cout << "wrote " << n << " of " << serverImage.size() << " bytes." << std::endl;
    }
    }   // ROI loop
}

}   // namespace

int main (const int argc, const char* argv[])
{
    // thought this might be why this program seems to die after a number of loops
    // but it had no effect. The real reason is that node.js' child_process does
    // not allow more than 200K bytes to be sent through stdout. It kills the
    // child process when that limit is reached.
    //static char buf[50000]; /* buf must survive until stdout is closed */
    //setvbuf ( stdout , buf , _IOFBF , sizeof(buf) );

    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()) {  // check if we succeeded
        std::cout << "failed to open camera" << std::endl;
        return -1;
    }
#if 1
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1920);
#else
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
#endif
    cap.set(cv::CAP_PROP_BRIGHTNESS, 0.45);

    cap.set(cv::CAP_PROP_FPS, 15);

    // set up output data socket
    int sockfd, portno, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    if (argc < 2) {
       fprintf(stderr,"usage %s port\n", argv[0]);
       exit(0);
    }
    portno = atoi(argv[1]);
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
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    else
        std::cout << "connected to host" << std::endl;

    ROIList ROIs;
    ROIDescriptor roi;
    roi.name = "pressureGaugeImage";
    roi.topLeftX = 65;
    roi.topLeftY = 5;
    ROIs.push_back(roi);
    roi.name = "controlPanelImage";
    roi.topLeftX = 27;
    roi.topLeftY = 81;
    ROIs.push_back(roi);

    char buffer[50000];
    std::string cinStr;
    do {
#if CONTINUOUS_STREAM == 0
        std::cerr << "waiting for cmd" << std::endl;
        std::getline(std::cin, cinStr);

        if (cinStr == "readCam") {
#endif
            int tankLevel;
            std::string serverJSON;
            std::string serverImage;
            readCamImage(sockfd, ROIs, cap, tankLevel, serverJSON, serverImage);

#if CONTINUOUS_STREAM == 0
            }
#endif

#if 0
    bzero(buffer,256);
    n = read(sockfd,buffer,255);
    if (n < 0) 
         error("ERROR reading from socket");
    printf("%s\n",buffer);
#endif

    } while (cinStr != "exit");
    std::cout << "exiting" << std::endl;

    close(sockfd);
    return 0;
}
