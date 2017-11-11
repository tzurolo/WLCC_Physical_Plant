/*

protocols:
    images:
        <identifier:imagedata>
        identifier is the IMG element ID

    JSON:
        { ... }

    TODO:
    1. change this over to use a socket to transmit data to node.js server (especially
        image data). We can't use std::cout because node.js's execFile has a 200K data
        limit on stdout.

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
    cv::VideoCapture &cap,
    int &tankLevel, // in units of percent
    std::string &serverJSON,
    std::string &serverImage)
{
    tankLevel = 0;

    // std::cerr << "read cam image" << std::endl;

    cv::Mat edges;
    cv::Mat tedges;
    cv::Mat ycrcb;
    cv::Mat floatMask;
    cv::Mat baseMask;

    // get image from camera
    cv::Mat frame;
    for (int f = 0; f < 3; ++f) {   // this loop is a hack to flush out old frames from the buffer
                                    // TODO: use a thread to continuously grab
        cap >> frame; // get a new frame from camera
    }

    // get timestamp strings
    std::string forImageAnnotation;
    std::string forFilespec;
    std::string forServerDatabase;
    getCurrentTimeStrings(forImageAnnotation, forFilespec, forServerDatabase);
#if 0
    // analyze image to find and read gauge
    // find float and base
    cv::cvtColor(frame, ycrcb, CV_BGR2YCrCb);
    cv::GaussianBlur(ycrcb, ycrcb, cv::Size(7,7), 1.5, 1.5);
    cv::inRange(ycrcb, thresholdData["floatMin"], thresholdData["floatMax"], floatMask);

    // get the contours of the float mask
    std::vector<std::vector<cv::Point> > contours;
    /// Find contours
    cv::findContours(floatMask, contours,
        cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    std::cerr << contours.size() << " float contours found." << std::endl;
#if 0
    // pick the contour with biggest bbox
    double floatMaxBboxSize = 0;
    cv::Rect floatBbox;
    for (int c = 0; c < contours.size(); ++c) {
      cv::Rect contourRec(boundingRect(contours[c]));
      const double bboxSize = contourRec.area();
      if (bboxSize > floatMaxBboxSize) {
	floatBbox = contourRec;
	floatMaxBboxSize = bboxSize;
      }
    }
#else
    // pick the contour with lowest top coord, but more
    // than minimum size
    const double minFloatContourArea = 12.0;
    const int maxFloatBboxHeight = 15;
    int minTop = 1000;
    cv::Rect floatBbox;
    for (int c = 0; c < contours.size(); ++c) {
      cv::Rect contourRec(boundingRect(contours[c]));
      int contourRecTop = contourRec.y;
      if ((contourArea(contours[c]) > minFloatContourArea) &&
          (contourRecTop < minTop)) {
	floatBbox = contourRec;
	minTop = contourRecTop;
      }
    }
    if (floatBbox.height > maxFloatBboxHeight) {
        floatBbox.height = maxFloatBboxHeight;
    }
#endif
#if 0
    // find bounding box around all the float contours
    std::vector<cv::Point> bboxPts;
    for (int c = 0; c < contours.size(); ++c) {
      cv::Rect contourRec(boundingRect(contours[c]));
      bboxPts.push_back(contourRec.tl());
      bboxPts.push_back(contourRec.br());
    }
    cv::Rect floatBbox(boundingRect(bboxPts));
#endif

    // find the base
    cv::inRange(ycrcb, thresholdData["baseMin"], thresholdData["baseMax"], baseMask);
    cv::findContours(baseMask, contours,
        cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));
    std::cerr << contours.size() << " base contours found." << std::endl;
    // pick the contour with biggest bbox
    double maxBboxSize = 0;
    cv::Rect baseBbox;
    for (int c = 0; c < contours.size(); ++c) {
      cv::Rect contourRec(boundingRect(contours[c]));
      const double bboxSize = contourRec.area();
      if (bboxSize > maxBboxSize) {
	baseBbox = contourRec;
	maxBboxSize = bboxSize;
      }
    }

    // find the scale
    // find float and base
    cv::Mat scaleMask;
    cv::inRange(ycrcb, thresholdData["scaleMin"], thresholdData["scaleMax"], scaleMask);
    // get the contours of the scale mask
    /// Find contours
    cv::findContours(scaleMask, contours,
        cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    // pick the contour with lowest top coord, but more
    // than minimum size
    const double minScaleContourArea = 40.0;
    minTop = 1000;
    cv::Rect scaleBbox;
    for (int c = 0; c < contours.size(); ++c) {
      cv::Rect contourRec(boundingRect(contours[c]));
      int contourRecTop = contourRec.y;
      if ((contourArea(contours[c]) > minScaleContourArea) &&
          (contourRecTop < minTop)) {
	scaleBbox = contourRec;
	minTop = contourRecTop;
      }
    }
    cv::Rect caseBbox(floatBbox.x - 20, scaleBbox.y,
        50, baseBbox.y - scaleBbox.y);

    const int level = caseBbox.br().y - floatBbox.y;
    tankLevel = (level * 100) / caseBbox.height;

    // annotate and write image
    cv::rectangle(frame, floatBbox, cv::Scalar(0, 255, 0), 2);
    cv::rectangle(frame, caseBbox, cv::Scalar(255, 255, 255), 1);
#endif
    char annotationBuf[80];
    sprintf(annotationBuf, "%s %d%%", forImageAnnotation.c_str(), tankLevel);
#if 1
    cv::putText(frame, annotationBuf, cv::Point(5, frame.size().height - 10), 
        cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 0), 2);
#endif
#if 1
    cv::Rect roi(0, 0, 200, 200);
#if 1
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

    serverImage = std::string("<boilerImage:data:image/jpeg;base64,");
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
}

void error(const char *msg)
{
    perror(msg);
    exit(0);
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
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
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

    // read color threshold data
    const char* thresholdsFilename = "HomeThresholds.txt"; 
    readColorThresholdData(thresholdsFilename, thresholdData);

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
            std::string serverInage;
            readCamImage(cap, tankLevel, serverJSON, serverInage);
            // std::cout << serverJSON << std::endl;

            n = write(sockfd,serverInage.c_str(), serverInage.size());
            if (n < 0) 
                 error("ERROR writing to socket");
            else {
                //std::cout << "wrote " << n << " of " << serverInage.size() << " bytes." << std::endl;
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

        }
    } while (cinStr != "exit");
    std::cout << "exiting" << std::endl;

    close(sockfd);
    return 0;
}
