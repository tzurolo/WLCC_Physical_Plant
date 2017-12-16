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
#include <fcntl.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h> 
#include <opencv2/opencv.hpp>
#include <raspicam/raspicam_cv.h>
extern "C" {
#include <b64/cencode.h>
}

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
    sprintf(dateTimeString, "%02d:%02d:%02d.%02d",
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

void sendImageToHost (
    const std::string &annotation,
    const std::string &imageID,
    const int sockfd,
    cv::Mat &image)
{
    // put in annotation, if any
    if (!annotation.empty()) {
        char annotationBuf[80];
        sprintf(annotationBuf, "%s", annotation.c_str());
        cv::putText(image, annotationBuf, cv::Point(10, image.size().height - 15), 
            cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 0));
    }

    // convert the image to JPEG
    std::vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    compression_params.push_back(95); // 1-100
    std::vector<uint8_t> jpeg_buffer;
    cv::imencode(".jpg", image, jpeg_buffer, compression_params);

    // encode the JPEG image as base64
    base64_encodestate b64State;
    base64_init_encodestate(&b64State);
    const int b64Len1 = base64_encode_block((const char*)&jpeg_buffer[0], (int)jpeg_buffer.size(),
        b64code, &b64State);
    const int b64Len2 = base64_encode_blockend(&b64code[b64Len1], &b64State);
    const int encodedLen = (b64Len1 + b64Len2);
    std::string imageStr;
    imageStr.reserve(encodedLen);
    imageStr = std::string("<") + imageID + ":data:image/jpeg;base64,";
    // the encoder puts in cr/lf every 72 chars
    for (int i = 0; i < encodedLen; ++i) {
        const char ch = b64code[i];
        if ((ch != 10) && (ch != 13)) {
            imageStr += ch;
        }
    }
    imageStr += '>';

    const int n = write(sockfd, imageStr.c_str(), imageStr.size());
    if (n < 0) {
        fprintf(stderr,"ERROR writing to socket");
        exit(0);
    } else {
        //std::cout << "wrote " << n << " of " << serverImage.size() << " bytes." << std::endl;
    }
}

void showHistogram (
    const std::string &imageID,
    const int sockfd,
    cv::Mat &image)
{
  std::vector<cv::Mat> bgr_planes;
  cv::split( image, bgr_planes );

  /// Establish the number of bins
  int histSize = 256;

  /// Set the ranges ( for B,G,R) )
  float range[] = { 0, 256 } ;
  const float* histRange = { range };

  bool uniform = true; bool accumulate = false;

  cv::Mat b_hist, g_hist, r_hist;

  /// Compute the histograms:
  cv::calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
  cv::calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate );
  cv::calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate );

  // Draw the histograms for B, G and R
  int hist_w = 512; int hist_h = 400;
  int bin_w = cvRound( (double) hist_w/histSize );

  cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

  /// Normalize the result to [ 0, histImage.rows ]
  cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
  cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

  /// Draw for each channel
  for (int i = 1; i < histSize; ++i) {
      line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
                       cv::Scalar( 255, 0, 0), 2, 8, 0  );
      line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(g_hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(g_hist.at<float>(i)) ),
                       cv::Scalar( 0, 255, 0), 2, 8, 0  );
      line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(r_hist.at<float>(i-1)) ) ,
                       cv::Point( bin_w*(i), hist_h - cvRound(r_hist.at<float>(i)) ),
                       cv::Scalar( 0, 0, 255), 2, 8, 0  );
  }

  sendImageToHost(std::string(), imageID, sockfd, histImage);
}

void readCamImage (
    raspicam::RaspiCam_Cv &cam,
//    cv::VideoCapture &cap,
    int sockfd)
{
    //std::cerr << "read cam image" << std::endl;

    cv::Mat edges;
    cv::Mat tedges;
    cv::Mat ycrcb;
    cv::Mat floatMask;
    cv::Mat baseMask;

    // get image from camera
    cv::Mat frame;
#if 1
    // RaspiCam
    cam.grab();
    cam.retrieve(frame);
#else
    // OpenCV
    for (int f = 0; f < 6; ++f) {   // this loop is a hack to flush out old frames from the buffer
                                    // TODO: use a thread to continuously grab
        cap >> frame; // get a new frame from camera
    }
#endif
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
#if 1
    cv::Rect roi(125, 190, 150, 350);
    cv::Mat dst = cv::Mat(frame, roi);
#else
    cv::Mat dst = frame;
#endif
#if 0
    cv::Mat cvImage;
    cv::cvtColor(dst, cvImage, cv::COLOR_BGR2GRAY);
#endif
#if 0
    GaussianBlur(cvImage, cvImage, cv::Size(3,3), 2, 2);
#endif
#if 0
    // canny edge detection
    cv::Canny(cvImage, dst, 75, 200, 3);
#endif

#if 0
    // histograms
    cv::Rect baseroi(160, 475, 65, 50);
    cv::Mat baseimage = cv::Mat(frame, baseroi);
    cv::Rect floatroi(180, 350, 40, 17);
    cv::Mat floatimage = cv::Mat(frame, floatroi);
    showHistogram("histogram", sockfd, floatimage);
    cv::rectangle(dst, floatroi, cv::Scalar(255, 255, 255), 2);
#endif
    sendImageToHost(forImageAnnotation, "tankGaugeImage", sockfd, dst);
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
#if 1
    raspicam::RaspiCam_Cv Camera;
    const double width = Camera.get( CV_CAP_PROP_FRAME_WIDTH);
    const double height = Camera.get( CV_CAP_PROP_FRAME_HEIGHT);
    const double brightness = Camera.get( CV_CAP_PROP_BRIGHTNESS);
    const double constrast = Camera.get( CV_CAP_PROP_CONTRAST);
    const double gain = Camera.get( CV_CAP_PROP_GAIN);
    const double saturation = Camera.get( CV_CAP_PROP_SATURATION);
    const double exposure = Camera.get( CV_CAP_PROP_EXPOSURE);
    const double wb_red_v = Camera.get( CV_CAP_PROP_WHITE_BALANCE_RED_V);
    const double wb_blue_u = Camera.get( CV_CAP_PROP_WHITE_BALANCE_BLUE_U);
    const double mode = Camera.get( CV_CAP_PROP_MODE);
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;
    std::cout << "brightness: " << brightness << std::endl;
    std::cout << "constrast: " << constrast << std::endl;
    std::cout << "gain: " << gain << std::endl;
    std::cout << "saturation: " << saturation << std::endl;
    std::cout << "exposure: " << exposure << std::endl;
    std::cout << "wb_red_v: " << wb_red_v << std::endl;
    std::cout << "wb_blue_u: " << wb_blue_u << std::endl;
    std::cout << "mode: " << mode << std::endl;

    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, 480);
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, 640);
    Camera.set( CV_CAP_PROP_BRIGHTNESS, 80);
    Camera.set( CV_CAP_PROP_CONTRAST, 100);
    Camera.set( CV_CAP_PROP_SATURATION, 65);
    //Camera.set( CV_CAP_PROP_GAIN, 85);
    //Camera.set( CV_CAP_PROP_EXPOSURE, 85);
    //Camera.set( CV_CAP_PROP_MODE, 2);
    if (!Camera.open()) {std::cerr<<"Error opening the camera"<<std::endl;return -1;}

#else
    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()) {  // check if we succeeded
        std::cout << "failed to open camera" << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_BRIGHTNESS, 0.65);

    cap.set(cv::CAP_PROP_FPS, 15);
#endif

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
    if (sockfd < 0) {
        fprintf(stderr,"ERROR opening socket");
        exit(0);
    }
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
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        fprintf(stderr,"ERROR connecting");
        exit(0);
    } else {
        std::cout << "connected to host" << std::endl;
        //fcntl(sockfd, F_SETFL, O_NONBLOCK);
    }
    struct timeval tv;
    fd_set readfds;


/*
    if (argc < 2) {
       fprintf(stderr,"usage %s port\n", argv[0]);
       exit(0);
    }
    portno = atoi(argv[1]);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        fprintf(stderr,"ERROR opening socket");
    }
    bzero(&serv_addr,sizeof(serv_addr));
    serv_addr.sin_family=AF_INET;
    serv_addr.sin_port=htons(portno);
 
    const int ret = inet_pton(AF_INET,"192.168.1.71",&(serv_addr.sin_addr));
    if (ret != 1) {
        fprintf(stderr,"ERROR from inet_pton");
        exit(0);
    }
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
        fprintf(stderr,"ERROR connecting");
        exit(0);
    } else {
        std::cout << "connected to host" << std::endl;
    }
*/
    // read color threshold data
    const char* thresholdsFilename = "HomeThresholds.txt"; 
    readColorThresholdData(thresholdsFilename, thresholdData);

    std::string cinStr;
    do {
#if 1
        FD_ZERO(&readfds);
        FD_SET(sockfd, &readfds);
        tv.tv_sec = 0;
        tv.tv_usec = 500000;
        select(sockfd+1, &readfds, NULL, NULL, &tv);

        if (FD_ISSET(sockfd, &readfds)) {
            char readBuf[512];
            int readCount;
            readCount = recv(sockfd, readBuf, sizeof(readBuf), 0);
            if (readCount == -1) {
                // check errono
            } else if (readCount == 0) {
                // socket closed by host
            } else {
                // got data
                const std::string dataStr(&readBuf[0], readCount);
                //std::cout << "got command: '" << dataStr << "'" << std::endl;
                const std::string cmdArg = dataStr.substr(2);
                const double cmdArgVal = atoi(cmdArg.c_str());
                switch (dataStr[0]) {
                    case 'B' : Camera.set( CV_CAP_PROP_BRIGHTNESS, cmdArgVal); break;
                    case 'C' : Camera.set( CV_CAP_PROP_CONTRAST, cmdArgVal); break;
                    case 'G' : Camera.set( CV_CAP_PROP_GAIN, cmdArgVal); break;
                    case 'E' : Camera.set( CV_CAP_PROP_EXPOSURE, cmdArgVal); break;
                    case 'S' : Camera.set( CV_CAP_PROP_SATURATION, cmdArgVal); break;
                    default:    break;
                }
            }
        } else {
            // time to read and process a frame
            readCamImage(Camera, sockfd);
        }

#endif
#if 0
        std::cerr << "waiting for cmd" << std::endl;
        std::getline(std::cin, cinStr);

        if (cinStr == "readCam") {
#endif

#if 0
            std::cout << serverJSON << std::endl;
        }
#endif
    } while (cinStr != "exit");
    std::cout << "exiting" << std::endl;
    return 0;
}
