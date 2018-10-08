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
#include <cstdlib>
#include <algorithm>
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
#include "json.hpp"
extern "C" {
#include <b64/cencode.h>
}

using json = nlohmann::json;

namespace {

struct CameraSettings {
    double frameWidth;          // CV_CAP_PROP_FRAME_WIDTH
    double frameHeight;         // CV_CAP_PROP_FRAME_HEIGHT
    double format;              // CV_CAP_PROP_FORMAT: CV_8UC1 or CV_8UC3
    double brightness;          // CV_CAP_PROP_BRIGHTNESS: [0,100]
    double contrast;            // CV_CAP_PROP_CONTRAST: [0,100]
    double saturation;          // CV_CAP_PROP_SATURATION: [0,100]
    double gain;                // CV_CAP_PROP_GAIN: (iso): [0,100]
    double exposure;            // CV_CAP_PROP_EXPOSURE: -1 auto. [1,100] shutter speed from 0 to 33ms
    double whiteBalanceRedV;    // CV_CAP_PROP_WHITE_BALANCE_RED_V : [1,100] -1 auto whitebalance
    double whiteBalanceBlueU;   // CV_CAP_PROP_WHITE_BALANCE_BLUE_U : [1,100] -1 auto whitebalance
    double fps;                 // CV_CAP_PROP_FPS
    CameraSettings () :
        frameWidth(320),
        frameHeight(480),
        format(CV_8UC3),
        brightness(80),
        contrast(100),
        saturation(50),
        gain(50),
        exposure(-1),
        whiteBalanceRedV(-1),
        whiteBalanceBlueU(-1),
        fps(10)
        {}
    CameraSettings (
        const json &j) :
        frameWidth(j["frameWidth"]),
        frameHeight(j["frameHeight"]),
        format(j["format"]),
        brightness(j["brightness"]),
        contrast(j["contrast"]),
        saturation(j["saturation"]),
        gain(j["gain"]),
        exposure(j["exposure"]),
        whiteBalanceRedV(j["whiteBalanceRedV"]),
        whiteBalanceBlueU(j["whiteBalanceBlueU"]),
        fps(j["fps"])
        {}
    json toJson() const
    {
        json j = {
            {"frameWidth", frameWidth},
            {"frameHeight", frameHeight},
            {"format", format},
            {"brightness", brightness},
            {"contrast", contrast},
            {"saturation", saturation},
            {"gain", gain},
            {"exposure", exposure},
            {"whiteBalanceRedV", whiteBalanceRedV},
            {"whiteBalanceBlueU", whiteBalanceBlueU},
            {"fps", fps}
        };
        return j;
    }
};

json cvScalarToJson (
    const cv::Scalar s)
{
    json j = {s[0], s[1], s[2]};
    return j;
}
cv::Scalar cvScalarFromJson (
    const json &j)
{
    cv::Scalar s;

    for (int i = 0; i < 3; ++i)
        s[i] = j[i];

    return s;
}

struct ImageThresholds {
    cv::Scalar lower;
    cv::Scalar upper;
    ImageThresholds() :
        lower(255, 255, 255),
        upper(0, 0, 0)
        {}
    ImageThresholds(
        const json &j) :
        lower(cvScalarFromJson(j["lower"])),
        upper(cvScalarFromJson(j["upper"]))
        {}
    bool isValid () const
        { return (lower[0] <= upper[0]) && (lower[1] <= upper[1]) && (lower[2] <= upper[2]); }
    json toJson() const
    {
        json j = { {"lower", cvScalarToJson(lower)},
                   {"upper", cvScalarToJson(upper)} };
        return j;
    }
};

struct ObjectDetectionParameters {
    // color thresholds of the desired object
    ImageThresholds imageThresholds;
    // of the contours returned by thresholding, the contour that has a
    // bounding box with approximate size (width and height) will be selected
    int expectedContourBboxWidth;
    int expectedContourBboxHeight;
    ObjectDetectionParameters () :
        imageThresholds(),
        expectedContourBboxWidth(0),
        expectedContourBboxHeight(0)
        {}
    ObjectDetectionParameters (
        const json &j) :
        imageThresholds(j["thresholds"]),
        expectedContourBboxWidth(j["expectedBboxWidth"]),
        expectedContourBboxHeight(j["expectedBboxHeight"])
        {}
    bool isValid () const
        { return imageThresholds.isValid() &&
                 (expectedContourBboxWidth > 0) &&
                 (expectedContourBboxHeight > 0);
        }
    json toJson() const
    {
        json j = { {"thresholds", imageThresholds.toJson()},
                   {"expectedBboxWidth", expectedContourBboxWidth},
                   {"expectedBboxHeight", expectedContourBboxHeight}
                 };
        return j;
    }
};

struct Parameters {
    CameraSettings camSettings;
    ObjectDetectionParameters floatParameters;
    ObjectDetectionParameters baseParameters;
    int scaleBaseDy;
    int scaleHeight;
    Parameters() :
        camSettings(),
        floatParameters(),
        baseParameters(),
        scaleBaseDy(0),
        scaleHeight(0)
        {}
    Parameters (
        const json &j) :
        camSettings(j["camSettings"]),
        floatParameters(j["floatParameters"]),
        baseParameters(j["baseParameters"]),
        scaleBaseDy(j["scaleBaseDy"]),
        scaleHeight(j["scaleHeight"])
        {}
    json toJson () const
    {
        json j = {
            {"version", "V1.0"},
            {"camSettings", camSettings.toJson()},
            {"floatParameters", floatParameters.toJson()},
            {"baseParameters", baseParameters.toJson()},
            {"scaleBaseDy", scaleBaseDy},
            {"scaleHeight", scaleHeight}
        };
        return j;
    }
};
Parameters params;

// computed values
cv::Rect floatBBox;
cv::Rect baseBBox;
int gaugeLevelPct = 0;
cv::Rect viewport;

char b64code[500000];
bool fullFrame = false;
bool genHistogram = false;
cv::Rect histogramRoi(0, 0, 0, 0);
ObjectDetectionParameters trainingParameters;

int cannyThreshold = 90;
int numFrames = 0;
int numDetections = 0;

std::string commandLine;

    bool isValidRoi (
        const cv::Rect &roi) {
        return ((roi.width > 5) && (roi.height > 5));
    }

    bool rectIsInsideRect (
        const cv::Rect &candidate,
        const cv::Rect &containingRect)
    {
        const int candidateLeft = candidate.tl().x;
        const int candidateRight = candidate.br().x;
        const int candidateTop = candidate.tl().y;
        const int candidateBottom = candidate.br().y;
        const int containingRectLeft = containingRect.tl().x;
        const int containingRectRight = containingRect.br().x;
        const int containingRectTop = containingRect.tl().y;
        const int containingRectBottom = containingRect.br().y;
        return (candidateLeft > containingRectLeft) &&
               (candidateRight < containingRectRight) &&
               (candidateTop > containingRectTop) &&
               (candidateBottom < containingRectBottom);
    }

    bool rectIsInsideImage (
        const cv::Rect &candidate,
        const cv::Mat &containingImage)
    {
		cv::Rect imageRect(0, 0, containingImage.cols, containingImage.rows);
		return rectIsInsideRect(candidate, imageRect);
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

void updateThresholdsWithHist (
    const cv::Mat &histData,
    const int histSize,
    const float histLevel,
    const int thresholdChannel,
    ImageThresholds &thresholds)
{
    // search for lower and upper edge of histogram curve at the given percentage
    int lowerEdge = 0;
    while ((histData.at<float>(lowerEdge) < histLevel) &&
           (lowerEdge < histSize)) {
        ++lowerEdge;
    }
    int upperEdge = histSize - 1;
    while ((histData.at<float>(upperEdge) < histLevel) &&
           (upperEdge > 0)) {
        --upperEdge;
    }
    //std::cout << lowerEdge << ".." << upperEdge << std::endl;
    if (lowerEdge < thresholds.lower[thresholdChannel]) {
        thresholds.lower[thresholdChannel] = lowerEdge;
    }
    if (upperEdge > thresholds.upper[thresholdChannel]) {
        thresholds.upper[thresholdChannel] = upperEdge;
    }
}

void findImageContoursByThresholds (
    const cv::Mat &image,
    const cv::Point &contourOffset,
    const ImageThresholds &thresholds,
    std::vector<std::vector<cv::Point> > &contours)
{
    // get a mask based on the thresholds
    cv::Mat contourMask;
    cv::inRange(image, thresholds.lower, thresholds.upper, contourMask);

    // morphologically "close" the image (dilate then erode)
    const int elementSizeForClose = 3;
    cv::Mat elementForClose = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                    cv::Size( 2*elementSizeForClose + 1, 2*elementSizeForClose+1 ),
                    cv::Point( elementSizeForClose, elementSizeForClose ) );
    cv::dilate(contourMask, contourMask, elementForClose );
    cv::erode(contourMask, contourMask, elementForClose );

    // get the contours of the mask
    cv::findContours(contourMask, contours,
        cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, contourOffset );
}

void findContourBoundingBox(
    const cv::Mat &image,
    const cv::Rect &roi,
    const ObjectDetectionParameters &parameters,
    cv::Rect &contourBoundingBox)
{
    contourBoundingBox.x = 0;
    contourBoundingBox.y = 0;
    contourBoundingBox.width = 0;
    contourBoundingBox.height = 0;

    const cv::Mat imageRoi =
		(isValidRoi(roi) && rectIsInsideImage(roi, image))
		? cv::Mat(image, roi)
		: image;
    if (parameters.isValid()) {
        std::vector<std::vector<cv::Point> > contours;
        findImageContoursByThresholds(imageRoi, roi.tl(), parameters.imageThresholds, contours);
        // pick the contour with bbox near the expected size
        int expectedWidth = ((roi.width > 0) && (roi.width < parameters.expectedContourBboxWidth))
            ? roi.width
            : parameters.expectedContourBboxWidth;
        int expectedHeight = ((roi.height > 0) && (roi.height < parameters.expectedContourBboxHeight))
            ? roi.height
            : parameters.expectedContourBboxHeight;
        const double sizeTolerance = 0.33;
        const int widthTolerance = cvRound(expectedWidth * sizeTolerance);
        const int heightTolerance = cvRound(expectedHeight * sizeTolerance);
        for (int c = 0; c < contours.size(); ++c) {
            const cv::Rect contourRec(cv::boundingRect(contours[c]));
            if ((abs(contourRec.width - expectedWidth) <= widthTolerance) &&
                (abs(contourRec.height - expectedHeight) <= heightTolerance)) {
                contourBoundingBox = contourRec;
                break;
            }
        }
    }
}

bool contourMatchesParameters(
    const int widthTolerance,
    const int heightTolerance,
    const cv::Rect &contourBoundingBox,
    const ObjectDetectionParameters &parameters)
{
    return //parameters.isValid() &&
        (parameters.expectedContourBboxWidth > 0) &&
        (parameters.expectedContourBboxHeight > 0) &&
        (abs(contourBoundingBox.width - parameters.expectedContourBboxWidth) <= widthTolerance) &&
        (abs(contourBoundingBox.height - parameters.expectedContourBboxHeight) <= heightTolerance);
}

void updateBoundingBox (
    const int deadband,
    const cv::Rect &newBBox,
    cv::Rect &updatedBBox)
{
    if (newBBox.width != 0) {
        // we have a valid new box
        if (updatedBBox.width == 0) {
            // first time through - no exsiting bbox - take new one
            updatedBBox = newBBox;
        } else {
            // latch biggest width and height
            if (newBBox.width > updatedBBox.width) {
                updatedBBox.width = newBBox.width;
            }
            if (newBBox.height > updatedBBox.height) {
                updatedBBox.height = newBBox.height;
            }

            // hysteresis for position
            if (newBBox.x > (updatedBBox.x + (deadband / 2))) {
                updatedBBox.x = newBBox.x - (deadband / 2);
            } else if (newBBox.x < (updatedBBox.x - (deadband / 2))) {
                updatedBBox.x = newBBox.x + (deadband / 2);
            }
            if (newBBox.y > (updatedBBox.y + (deadband / 2))) {
                updatedBBox.y = newBBox.y - (deadband / 2);
            } else if (newBBox.y < (updatedBBox.y - (deadband / 2))) {
                updatedBBox.y = newBBox.y + (deadband / 2);
            }
        }
    } else {
        updatedBBox = cv::Rect();
        // std::cout << "no new bbox" << std::endl;
    }
}

void showHistogram (
    const std::string &imageID,
    const int sockfd,
    ObjectDetectionParameters &parameters,
    cv::Mat &histogramSourceImage,
    cv::Mat &cameraImage)
{
    const bool haveValidRoi = isValidRoi(histogramRoi);
    std::vector<cv::Mat> bgr_planes;
    if (haveValidRoi) {
        cv::Mat srcImage = cv::Mat(histogramSourceImage, histogramRoi);
        cv::split( srcImage, bgr_planes );
    } else {
        cv::split( histogramSourceImage, bgr_planes );
    }

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

    if (haveValidRoi) {
        // update roi thresholds
        const float histLevel = histImage.rows * 0.15;
        if (haveValidRoi) {
            updateThresholdsWithHist(b_hist, histSize, histLevel, 0, parameters.imageThresholds);
            updateThresholdsWithHist(g_hist, histSize, histLevel, 1, parameters.imageThresholds);
            updateThresholdsWithHist(r_hist, histSize, histLevel, 2, parameters.imageThresholds);
        }

        const int lineY = hist_h - cvRound(histLevel);
        line(histImage, cv::Point(bin_w * parameters.imageThresholds.lower[0], lineY),
                        cv::Point(bin_w * parameters.imageThresholds.upper[0], lineY),
                        cv::Scalar( 255, 0, 0), 2, 8, 0  );
        line(histImage, cv::Point(bin_w * parameters.imageThresholds.lower[1], lineY + 4),
                        cv::Point(bin_w * parameters.imageThresholds.upper[1], lineY + 4),
                        cv::Scalar( 0, 255, 0), 2, 8, 0  );
        line(histImage, cv::Point(bin_w * parameters.imageThresholds.lower[2], lineY + 8),
                        cv::Point(bin_w * parameters.imageThresholds.upper[2], lineY + 8),
                        cv::Scalar( 0, 0, 255), 2, 8, 0  );

        std::vector<std::vector<cv::Point> > contours;
        findImageContoursByThresholds(histogramSourceImage, cv::Point(0, 0),
            parameters.imageThresholds, contours);
        if (!contours.empty()) {
            // find the contour with biggest bbox
            int maxContourArea = 0;
            cv::Rect largestContourBbox;
            int largestContourIndex = -1;
            for (int c = 0; c < contours.size(); ++c) {
                const cv::Rect contourBBox = cv::boundingRect(contours[c]);
                const int contourArea = contourBBox.area();
                if (contourArea > maxContourArea) {
                    largestContourIndex = c;
                    largestContourBbox = contourBBox;
                    maxContourArea = contourArea;
                }
            }
            if (largestContourIndex >= 0) {
                // update contour bounding box parameters
                if (largestContourBbox.width > parameters.expectedContourBboxWidth) {
                    parameters.expectedContourBboxWidth = largestContourBbox.width;
                }
                if (largestContourBbox.height > parameters.expectedContourBboxHeight) {
                    parameters.expectedContourBboxHeight = largestContourBbox.height;
                }

                // draw largest contour on image
                cv::drawContours(cameraImage, contours, largestContourIndex, cv::Scalar(255, 0, 0), 2);
                char msg[100];
                sprintf(msg, "top: %d, width: %d, num contours: %d",
                    largestContourBbox.y, largestContourBbox.width, contours.size());
                std::cout << msg << std::endl;
            }
        }
    }

  sendImageToHost(std::string(), imageID, sockfd, histImage);
}

void drawBracket (
    const int x,
    const int y,
    const int height,
    const int width,
    cv::Mat &frame)
{
    const cv::Scalar color(0, 255, 255);
    const int lineWidth = 2;
    cv::line(frame, cv::Point(x, y),
                    cv::Point(x + width, y),
                    color, lineWidth);
    cv::line(frame, cv::Point(x, y),
                    cv::Point(x, y + height),
                    color, lineWidth);
    cv::line(frame, cv::Point(x, y + height),
                    cv::Point(x + width, y + height),
                    color, lineWidth);
}

void drawFloatIndicator (
    const cv::Rect &floatBBox,
    const ObjectDetectionParameters &floatParameters,
    cv::Mat &frame)
{
    const int width = 5;
    drawBracket(floatBBox.x - floatParameters.expectedContourBboxWidth / 2,
        floatBBox.y, floatParameters.expectedContourBboxHeight, width, frame);
    drawBracket(floatBBox.x + floatParameters.expectedContourBboxWidth +
        floatParameters.expectedContourBboxWidth / 2,
        floatBBox.y, floatParameters.expectedContourBboxHeight, -width, frame);
}

void readCamImage (
    raspicam::RaspiCam_Cv &cam,
//    cv::VideoCapture &cap,
    int sockfd)
{
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

    cv::Mat histogramSourceImage;
#if 0
    histogramSourceImage = frame;
#else
    cv::cvtColor(frame, histogramSourceImage, cv::COLOR_BGR2HSV);
#endif

    bool haveValidLevelPct = false;

    // histograms
    //showHistogram("histogram", sockfd, trainingParameters, histogramSourceImage, frame);
    const bool haveValidRoi = isValidRoi(histogramRoi);

    ++numFrames;
    cv::Mat edgeImage;
    cv::cvtColor(frame, edgeImage, cv::COLOR_BGR2GRAY);
    GaussianBlur(edgeImage, edgeImage, cv::Size(3,3), 2, 2);
    cv::Canny(edgeImage, edgeImage, cannyThreshold, cannyThreshold*3, 3);

    // close the edge image
    const int elementSize = 4;
    cv::Mat elementForClose = cv::getStructuringElement(cv::MORPH_RECT,
                cv::Size(elementSize, elementSize),
                cv::Point(1, 1));
    cv::dilate(edgeImage, edgeImage, elementForClose );
    cv::erode(edgeImage, edgeImage, elementForClose );

    if (genHistogram) {
        // create overlay of the closed edge image onto the camera image
        cv::Mat edgeImageBGR;
        cv::cvtColor(edgeImage, edgeImageBGR, cv::COLOR_GRAY2BGR);
        cv::bitwise_or(edgeImageBGR, frame, frame);
    }

    // NOTE: get the histogram of the level 2 contour inside the selected ROI.
    // get the polygon for the level 2 contour and fill it to make a mask
    // for calcHist, but we must remove the area from any nested contours

    bool detectedFloat = false;
    bool detectedBase = false;

    // get the level two contours
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edgeImage, contours, hierarchy,
        cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1);
    if (!contours.empty()) {
        // find the contour with biggest bbox
        int maxContourArea = 0;
        cv::Rect selectedContourBbox;
        int selectedContourIndex = -1;
        const int widthTolerance = 5;
        const int heightTolerance = 8;
        for (int c = 0; c < contours.size(); ++c) {
            const int parent = hierarchy[c][3];
            if ((parent >= 0) && (hierarchy[parent][3] < 0)) {
                // level 2 contour
                if (genHistogram) {
                    cv::drawContours(frame, contours, c, cv::Scalar(255, 0, 255), 1);
                }
                const cv::Rect contourBBox = cv::boundingRect(contours[c]);
                if (contourMatchesParameters(3, 5, contourBBox, params.floatParameters)) {
                    updateBoundingBox(4, contourBBox, floatBBox);
                    detectedFloat = true;
                } else if (contourMatchesParameters(1, 1, contourBBox, params.baseParameters)) {
                    updateBoundingBox(2, contourBBox, baseBBox);
                    detectedBase = true;
                }
#if 1
                if (haveValidRoi) {
                    // detect contour that is inside ROI
                    if (rectIsInsideRect(contourBBox, histogramRoi)) {
                        selectedContourIndex = c;
                        selectedContourBbox = contourBBox;
                    }
                }
#else
                if ((contourBBox.width >= 15) && (contourBBox.width <= 42) &&
                    (abs(contourBBox.height - 30) <= heightTolerance)) {
                    const int contourArea = contourBBox.area();
                    if (contourArea > maxContourArea) {
                        largestContourIndex = c;
                        largestContourBbox = contourBBox;
                        maxContourArea = contourArea;
                    }
                }
#endif
            }
        }
#if 1
        if (selectedContourIndex >= 0) {
            // update contour bounding box parameters
            if (selectedContourBbox.width > trainingParameters.expectedContourBboxWidth) {
                trainingParameters.expectedContourBboxWidth = selectedContourBbox.width;
            }
            if (selectedContourBbox.height > trainingParameters.expectedContourBboxHeight) {
                trainingParameters.expectedContourBboxHeight = selectedContourBbox.height;
            }
            ++numDetections;
            // draw selected contour on image
            cv::drawContours(frame, contours, selectedContourIndex, cv::Scalar(255, 0, 0), 1);
            char msg[200];
            sprintf(msg, "top: %d, w: %d, h: %d, #contours: %d, detection: %d%%",
                selectedContourBbox.y, selectedContourBbox.width, selectedContourBbox.height, contours.size(),
                (numDetections * 100) / numFrames);
            std::cout << msg << std::endl;
        } else {
            // overlay edge image if we did not detect
            //cv::bitwise_or(edgeImageBGR, frame, frame);
        }
#endif
        if ((floatBBox.width > 0) && (baseBBox.width > 0) &&    // detected float and base
            (floatBBox.br().y < baseBBox.y) &&  // float is above base
            (abs((baseBBox.x + baseBBox.width / 2) - (floatBBox.x + floatBBox.width / 2)) <= 4)) { // float is roughly centered over base
            // we have a valid reading
            // estimate oil level
            int gaugeDy = baseBBox.y - floatBBox.y;
            if ((params.scaleBaseDy > 0) && (params.scaleHeight> 0)) {
                gaugeLevelPct = ((gaugeDy - params.scaleBaseDy) * 100) / params.scaleHeight;
                haveValidLevelPct = true;

                // update viewport
                cv::Rect newViewport(
                    floatBBox.x - (floatBBox.width + (floatBBox.width / 2)),
                    baseBBox.y - (params.scaleBaseDy + (params.scaleHeight / 2) + params.scaleHeight),
                    floatBBox.width * 4,
                    params.scaleHeight * 2);
                updateBoundingBox(20, newViewport, viewport);
            } else {
                std::cout << "---> gauge level: " << gaugeDy << " pixels" << std::endl;
            }
        }
    }
    //cv::bitwise_or(frame, edgeImage, frame);

    if (genHistogram && (viewport.width > 0)) {
        cv::rectangle(frame, viewport, cv::Scalar(255, 255, 255), 1);
    }

    if (haveValidLevelPct) {
        if (floatBBox.width > 0) {
            drawFloatIndicator(floatBBox, params.floatParameters, frame);
        }
        if (baseBBox.width > 0) {
            cv::rectangle(frame, baseBBox, cv::Scalar(0, 0, 255), 2);
            if (genHistogram && (params.scaleBaseDy > 0) && (params.scaleHeight> 0)) {
                // show scale empty and full levels
                const int scaleX1 = baseBBox.x;
                const int scaleX2 = baseBBox.x + baseBBox.width;
                const int scaleEmptyY = baseBBox.y - params.scaleBaseDy;
                const int scaleFullY = baseBBox.y - (params.scaleBaseDy + params.scaleHeight);
                cv::line(frame, cv::Point(scaleX1, scaleEmptyY),
                                cv::Point(scaleX2, scaleEmptyY),
                                cv::Scalar(255, 0, 0), 2);
                cv::line(frame, cv::Point(scaleX1, scaleFullY),
                                cv::Point(scaleX2, scaleFullY),
                                cv::Scalar(255, 0, 0), 2);
            }
        }
    }
    cv::Mat dst;
    if (isValidRoi(viewport) && rectIsInsideImage(viewport, frame) && !fullFrame) {
        //cv::Rect roi(80, 135, 115, 250);
        dst = cv::Mat(frame, viewport);
    } else {
        dst = frame;
    }
#if 1
    if (haveValidLevelPct) {
        char buf[50];
        sprintf(buf, "%d%% %c%c", gaugeLevelPct,
            detectedBase ? '^' : '_',
            detectedFloat ? '^' : '_');
        cv::putText(dst, buf, cv::Point(10, dst.size().height - 30), 
            cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 0));
    }
#endif
    sendImageToHost(forImageAnnotation, "tankGaugeImage", sockfd, dst);
}

void executeCommand (
    const std::string &command,
    const std::string &settingsFilename,
    bool &running,
    raspicam::RaspiCam_Cv &cam)
{
    const std::string cmdArgs = (command.length() > 2) ? command.substr(2) : std::string();
    switch (command[0]) {
        case 'A' :  // accept training parameters
            switch (cmdArgs[0]) {
                case 'F' :
                    params.floatParameters = trainingParameters;
                    std::cout << "float parameters accepted" << std::endl;
                    break;
                case 'B' :
                    params.baseParameters = trainingParameters;
                    std::cout << "base parameters accepted" << std::endl;
                    break;
                case 'S' :
                    if (isValidRoi(histogramRoi) && (baseBBox.width > 0)) {
                        params.scaleBaseDy = baseBBox.y - (histogramRoi.y + histogramRoi.height);
                        params.scaleHeight = histogramRoi.height;
                        std::cout << "scale bounds accepted: base Dy=" << params.scaleBaseDy <<
                            ", height=" << params.scaleHeight << std::endl;
                    }
                    break;
                default:
                    break;
            }
            break;
        case 'C' : {    // camera setting
            const char* camArgs = (cmdArgs.length() > 2)
                ? cmdArgs.substr(2).c_str()
                : nullptr;
            switch (cmdArgs[0]) {
                case 'B' : cam.set( CV_CAP_PROP_BRIGHTNESS, atoi(camArgs)); break;
                case 'C' : cam.set( CV_CAP_PROP_CONTRAST, atoi(camArgs)); break;
                case 'G' : cam.set( CV_CAP_PROP_GAIN, atoi(camArgs)); break;
                case 'E' : cam.set( CV_CAP_PROP_EXPOSURE, atoi(camArgs)); break;
                case 'S' : 
                    cannyThreshold = atoi(camArgs);
                    // cam.set( CV_CAP_PROP_SATURATION, atoi(camArgs));
                    break;
                default: break;
            }
            }
            break;
        case 'F' : fullFrame = (atoi(cmdArgs.c_str()) == 1);  break;
        case 'H' : genHistogram = (atoi(cmdArgs.c_str()) == 1);  break;
        case 'R' :    
            // set histogram rectangle
            sscanf(cmdArgs.c_str(), "%d %d %d %d",
                    &histogramRoi.x,
                    &histogramRoi.y,
                    &histogramRoi.width,
                    &histogramRoi.height);
            trainingParameters = ObjectDetectionParameters();
            break;
        case 'S' : {
            const json j = params.toJson();
            std::ofstream os1(settingsFilename);
            os1 << std::setw(4) << j << std::endl;
            }
            break;
        case 'X' :
            running = false;
            break;
        case 'Z' :
            params = Parameters();
            floatBBox = cv::Rect();
            baseBBox = cv::Rect();
            gaugeLevelPct = 0;
            viewport = cv::Rect();
            break;
        default:
            break;
    }
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
    std::cout << "C++ version: " << __cplusplus << std::endl;
    const std::string settingsFilename = std::string(argv[0]) + "Settings.json";
    std::cout << "reading settings from " << settingsFilename << std::endl;
    json j;
    std::ifstream is1(settingsFilename);
    is1 >> j;
    params = Parameters(j);
#if 1
    raspicam::RaspiCam_Cv Camera;
    Camera.set( CV_CAP_PROP_FORMAT, params.camSettings.format);
    Camera.set( CV_CAP_PROP_FRAME_WIDTH, params.camSettings.frameWidth);
    Camera.set( CV_CAP_PROP_FRAME_HEIGHT, params.camSettings.frameHeight);
    Camera.set( CV_CAP_PROP_BRIGHTNESS, params.camSettings.brightness);
    Camera.set( CV_CAP_PROP_CONTRAST, params.camSettings.contrast);
    Camera.set( CV_CAP_PROP_SATURATION, params.camSettings.saturation);
    Camera.set( CV_CAP_PROP_GAIN, params.camSettings.gain);
    Camera.set( CV_CAP_PROP_EXPOSURE, params.camSettings.exposure);
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

    bool running = true;
    while (running) {
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
                for (int c = 0; c < readCount; ++c) {
                    const char ch = dataStr[c];
                    switch (ch) {
                        case '\r' :
                            // we have a complete command
                            executeCommand(commandLine, settingsFilename, running, Camera);
                            commandLine.clear();
                            break;
                        case '\n' :
                            // ignore newlines
                            break;
                        default:
                            commandLine += ch;
                            break;
                    }
                }
            }
        } else {
            // time to read and process a frame
            readCamImage(Camera, sockfd);
        }
    }
    std::cout << "exiting" << std::endl;
    return 0;
}

#if 0
    // analyze image to find and read gauge
    // find float
    cv::Rect newFloatBBox;
    findContourBoundingBox(histogramSourceImage, cv::Rect(), params.floatParameters, newFloatBBox);
    updateBoundingBox(4, newFloatBBox, floatBBox);
    cv::Rect baseROI;
    if (floatBBox.width > 0) {
        // valid float bounding box
        // find base
        const int baseROIy = floatBBox.y + floatBBox.height;
        const cv::Rect newBaseROI(floatBBox.x, baseROIy,
                         floatBBox.width, frame.size().height - baseROIy);
        cv::Rect newBaseBBox;
        findContourBoundingBox(histogramSourceImage, newBaseROI, params.baseParameters, newBaseBBox);
        updateBoundingBox(8, newBaseBBox, baseBBox);
        baseROI = newBaseROI;
    } else {
        baseBBox = cv::Rect();
    }

    bool haveValidLevelPct = false;
    if ((floatBBox.width > 0) && (baseBBox.width > 0)) {
        // we have a valid reading
        // estimate oil level
        int gaugeDy = baseBBox.y - floatBBox.y;
        if ((params.scaleBaseDy > 0) && (params.scaleHeight> 0)) {
            gaugeLevelPct = ((gaugeDy - params.scaleBaseDy) * 100) / params.scaleHeight;
            haveValidLevelPct = true;

            // update viewport
            cv::Rect newViewport(
                floatBBox.x - (floatBBox.width * 2),
                baseBBox.y - (params.scaleBaseDy + (params.scaleHeight / 2) + params.scaleHeight),
                floatBBox.width * 5,
                params.scaleHeight * 2);
            updateBoundingBox(20, newViewport, viewport);
        } else {
            std::cout << "---> gauge level: " << gaugeDy << " pixels" << std::endl;
        }
    }
#endif
