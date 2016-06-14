#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "xbee.h"

using namespace cv;
using namespace std;

int main()
{


    VideoCapture cap(0); //capture the video from webcam

    if ( !cap.isOpened() )  // if not success, exit program
    {
        cout << "Cannot open the web cam" << endl;
        return -1;
    }



    int iLowH = 0;
    int iHighH = 14;

    int iLowS = 110;
    int iHighS = 255;

    int iLowV = 163;
    int iHighV = 255;


    Mat imgTmp;

    cap.read(imgTmp);

    //Create a black image with the size as the camera output
    Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;

    while(1)
    {
        Mat imgOriginal;

        bool bSuccess = cap.read(imgOriginal); // read a new frame from video



        if (!bSuccess) //if not success, break loop
        {
            cout << "Cannot read a frame from video stream" << endl;
            break;
        }

        Mat imgHSV;



        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

        Mat imgThresholded;

        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        //morphological opening (removes small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        //morphological closing (removes small holes from the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

        Mat circle;

        vector<Vec3f> circles;

        // Convert it to gray
        //  cvtColor(imgOriginal, circle, CV_BGR2GRAY);

        // Reduce the noise so we avoid false circle detection
        GaussianBlur(imgThresholded, circle, Size(9, 9), 2, 2);

        // Apply the Hough Transform to find the circles
        HoughCircles(circle, circles, CV_HOUGH_GRADIENT, 1, circle.rows/16, 150, 30, 0, 0);

        // Draw the circles detected
        for(size_t i = 0; i < circles.size(); i++ ) {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            cout <<   cvRound(circles[i][0]) << "," <<  cvRound(circles[i][1]) << endl;
            cv::circle(imgOriginal, center, radius, Scalar(255,0, 68), 3, 8, 0);


        }

        //Calculate the moments of the thresholded image
        Moments oMoments = moments(imgThresholded);

        double dM01 = oMoments.m01;
        double dM10 = oMoments.m10;
        double dArea = oMoments.m00;

        // if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
        if (dArea > 1000000)
        {

            //calculate the position of the ball
            int posX = dM10 / dArea;
            int posY = dM01 / dArea;
            if(dArea < 4000000){
                send('f');
                cout << "go forward" ;
            }
            else if(dArea > 10000000){
                send('b');
                cout << "go back" ;
            }
            else if(posX < 250){
                send('a');
                cout << "turn left" ;
            }
            else if(posX > 950){
                send('d');
                cout << "turn right" ;
            }
            else if(dArea < 6000000){
                send('w');
                cout << "go forward" ;
            }
            else if(dArea > 8000000){
                send('s');
                cout << "go back" ;
            }

            else{
                send('n');
            }
            cout << "(" << posX << "," << posY << ")" << endl ;



        }
        else{

            cout << "lose target" << endl ;
        }
        imshow("video2", imgThresholded);
        imshow("Video", imgOriginal); //show the original image

        waitKey(30);


    }


    return 0;
}

void clean_buffer () {
    fflush(stdin);
    // while (getchar() != '\n');
}
int send (char com) {
    int serial_d;
    speed_t speed;
    struct termios soptions, soptions_org;
    char command;
    unsigned char send_buf[BUFLEN];
    unsigned char recv_buf[BUFLEN];
    int sent_c, recv_c;

    serial_d = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);

    if (serial_d == -1) cout<<"error";

    // ----- Begin of setup serial ports
    tcgetattr(serial_d, &soptions_org);
    tcgetattr(serial_d, &soptions);

    speed = B9600; // Speed options: B19200, B38400, B57600, B115200
    cfsetispeed(&soptions, speed);
    cfsetospeed(&soptions, speed);

    // Enable the reciver and set local mode...
    soptions.c_cflag |= ( CLOCAL | CREAD );
    // Setting Parity Checking (8N1)
    soptions.c_cflag &= ~PARENB;
    soptions.c_cflag &= ~CSTOPB;
    soptions.c_cflag &= ~CSIZE;
    soptions.c_cflag |= CS8;

    // Local setting
    //soptions.c_lflag = (ICANON | ECHO | ECHOE); //canonical
    soptions.c_lflag =  ~(ICANON | ECHO | ECHOE | ISIG); //noncanonical

    // Input setting
    //soptions.c_iflag |= (IXON | IXOFF | IXANY); //software flow control
    soptions.c_iflag |= (INPCK | ISTRIP);
    soptions.c_iflag = IGNPAR;

    // Output setting
    soptions.c_oflag = 0;
    soptions.c_oflag &= ~OPOST;

    // Read options
    soptions.c_cc[VTIME] = 0;
    soptions.c_cc[VMIN] = 1; //transfer at least 1 character or block

    // Apply setting
    tcsetattr(serial_d, TCSANOW, &soptions);
    // ----- End of setup serial ports

    command = com;
    clean_buffer();
    printf("Sending command '%c'...\n", command);
    sent_c = write(serial_d, &command, 1); // Send command
    tcdrain(serial_d);
    //   usleep(100000); // Wait for response
    /*            memset(recv_buf, '\0', BUFLEN);
            recv_c = read(serial_d, recv_buf, BUFLEN); // Get response message
            tcdrain(serial_d);
            printf("%s\n", recv_buf);*/
    //     }
    // restore setting and close
    //tcsetattr(serial_d, TCSANOW, &soptions_org);
    close(serial_d);
    return 0;
}
