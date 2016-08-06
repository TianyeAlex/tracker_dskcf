#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <OpenNI.h>

#include "kcftracker.hpp"


boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev);

unsigned char startByte = 0xff;
unsigned char endByte = 0xfe;
std::string Base_Port = "/dev/ttyUSB0";

using namespace std;
using namespace openni;
using namespace cv;

static const std::string RGB_WINDOW = "RGB Image window";
static const std::string DEPTH_WINDOW = "DEPTH Image window";

#define Max_linear_speed 0.6
#define Min_linear_speed 0.4
#define Min_distance 1.5
#define Max_distance 5.0
#define Max_rotation_speed 0.75

float linear_speed = 0;
float rotation_speed = 0;

float k_linear_speed = (Max_linear_speed - Min_linear_speed) / (Max_distance - Min_distance);
float h_linear_speed = Min_linear_speed - k_linear_speed * Min_distance;

float k_rotation_speed = 0.004;
float h_rotation_speed_left = 1.2;
float h_rotation_speed_right = 1.36;
 
int ERROR_OFFSET_X_left1 = 100;
int ERROR_OFFSET_X_left2 = 300;
int ERROR_OFFSET_X_right1 = 340;
int ERROR_OFFSET_X_right2 = 540;

cv::Mat rgbimage;
cv::Mat depthimage;
cv::Rect selectRect;
cv::Point origin;
cv::Rect result;

bool select_flag = false;
bool bRenewROI = false;  // the flag to enable the implementation of KCF algorithm for the new chosen ROI
bool bBeginKCF = false;
bool enable_get_depth = false;

bool HOG = true;
bool FIXEDWINDOW = false;
bool MULTISCALE = true;
bool SILENT = true;
bool LAB = false;

// Create KCFTracker object
KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB);

float dist_val[5] ;

union Max_Value {
    unsigned char buf[8];
    struct _FLOAT_ {
        float _double_vT;
        float _double_vR;
    } Double_RAM;
} Send_Data;


void openPort()
{
    sp.open(Base_Port);
    sp.set_option(boost::asio::serial_port::baud_rate(9600));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));
}

/**
 * send the velocity to the lower machine
 * @param vTranslation [the translation velocity]    max <= 0.8
 * @param vRotation    [the rotation velocity]       max <= 1.0
 */
void sendVelocity(float vTranslation, float vRotation)
{
    Send_Data.Double_RAM._double_vT = vTranslation;
    Send_Data.Double_RAM._double_vR = vRotation;
    write(sp, boost::asio::buffer(&startByte, 1));
    write(sp, boost::asio::buffer(Send_Data.buf, 8));
    write(sp, boost::asio::buffer(&endByte, 1));
}

void onMouse(int event, int x, int y, int, void*)
{
    if (select_flag)
    {
        selectRect.x = MIN(origin.x, x);        
        selectRect.y = MIN(origin.y, y);
        selectRect.width = abs(x - origin.x);   
        selectRect.height = abs(y - origin.y);
        selectRect &= cv::Rect(0, 0, rgbimage.cols, rgbimage.rows);
    }
    if (event == CV_EVENT_LBUTTONDOWN)
    {
        bBeginKCF = false;  
        select_flag = true; 
        origin = cv::Point(x, y);       
        selectRect = cv::Rect(x, y, 0, 0);  
    }
    else if (event == CV_EVENT_LBUTTONUP)
    {
        select_flag = false;
        bRenewROI = true;
    }
}


void showdevice(){
    // 获取设备信息
    Array<DeviceInfo> aDeviceList;
    OpenNI::enumerateDevices(&aDeviceList);

    cout << "电脑上连接着 " << aDeviceList.getSize() << " 个体感设备." << endl;

    for (int i = 0; i < aDeviceList.getSize(); ++i)
    {
        cout << "设备 " << i << endl;
        const DeviceInfo& rDevInfo = aDeviceList[i];
        cout << "设备名： " << rDevInfo.getName() << endl;
        cout << "设备Id： " << rDevInfo.getUsbProductId() << endl;
        cout << "供应商名： " << rDevInfo.getVendor() << endl;
        cout << "供应商Id: " << rDevInfo.getUsbVendorId() << endl;
        cout << "设备URI: " << rDevInfo.getUri() << endl;
    }
}

void hMirrorTrans(const Mat &src, Mat &dst)
{
    dst.create(src.rows, src.cols, src.type());

    int rows = src.rows;
    int cols = src.cols;

    switch (src.channels())
    {
    case 1:   //1通道比如深度图像
        const uchar *origal;
        uchar *p;
        for (int i = 0; i < rows; i++){
            origal = src.ptr<uchar>(i);
            p = dst.ptr<uchar>(i);
            for (int j = 0; j < cols; j++){
                p[j] = origal[cols - 1 - j];
            }
        }
        break;
    case 3:   //3通道比如彩色图像
        const Vec3b *origal3;
        Vec3b *p3;
        for (int i = 0; i < rows; i++) {
            origal3 = src.ptr<Vec3b>(i);
            p3 = dst.ptr<Vec3b>(i);
            for (int j = 0; j < cols; j++){
                p3[j] = origal3[cols - 1 - j];
            }
        }
        break;
    default:
        break;
    }

}

int main()
{
    Status rc = STATUS_OK;

    // 初始化OpenNI环境
    OpenNI::initialize();

    showdevice();

    // 声明并打开Device设备。
    Device xtion;
    const char * deviceURL = openni::ANY_DEVICE;  //设备名
    rc = xtion.open(deviceURL);

    // 创建深度数据流
    VideoStream streamDepth;
    rc = streamDepth.create(xtion, SENSOR_DEPTH);
    if (rc == STATUS_OK)
    {
        // 设置深度图像视频模式
        VideoMode mModeDepth;
        // 分辨率大小
        mModeDepth.setResolution(640, 480);
        // 每秒30帧
        mModeDepth.setFps(30);
        // 像素格式
        mModeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

        streamDepth.setVideoMode(mModeDepth);

        // 打开深度数据流
        rc = streamDepth.start();
        if (rc != STATUS_OK)
        {
            cerr << "无法打开深度数据流：" << OpenNI::getExtendedError() << endl;
            streamDepth.destroy();
        }
    }
    else
    {
        cerr << "无法创建深度数据流：" << OpenNI::getExtendedError() << endl;
    }

    // 创建彩色图像数据流
    VideoStream streamColor;
    rc = streamColor.create(xtion, SENSOR_COLOR);
    if (rc == STATUS_OK)
    {
        // 同样的设置彩色图像视频模式
        VideoMode mModeColor;
        mModeColor.setResolution(320, 240);
        mModeColor.setFps(30);
        mModeColor.setPixelFormat(PIXEL_FORMAT_RGB888);

        streamColor.setVideoMode(mModeColor);

        // 打开彩色图像数据流
        rc = streamColor.start();
        if (rc != STATUS_OK)
        {
            cerr << "无法打开彩色图像数据流：" << OpenNI::getExtendedError() << endl;
            streamColor.destroy();
        }
    }
    else
    {
        cerr << "无法创建彩色图像数据流：" << OpenNI::getExtendedError() << endl;
    }

    if (!streamColor.isValid() || !streamDepth.isValid())
    {
        cerr << "彩色或深度数据流不合法" << endl;
        OpenNI::shutdown();
        return 1;
    }

    // 图像模式注册,彩色图与深度图对齐
    if (xtion.isImageRegistrationModeSupported(
        IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
        xtion.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }


    // 创建OpenCV图像窗口
    namedWindow(RGB_WINDOW, CV_WINDOW_AUTOSIZE);
    //namedWindow(DEPTH_WINDOW, CV_WINDOW_AUTOSIZE);

    // 获得最大深度值
    int iMaxDepth = streamDepth.getMaxPixelValue();

    // 循环读取数据流信息并保存在VideoFrameRef中
    VideoFrameRef  frameDepth;
    VideoFrameRef  frameColor;

    //openPort();

    while (true)
    {
        // 读取数据流
        rc = streamDepth.readFrame(&frameDepth);
        if (rc == STATUS_OK)
        {
            // 将深度数据转换成OpenCV格式
            const Mat mImageDepth(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
            // 为了让深度图像显示的更加明显一些，将CV_16UC1 ==> CV_8U格式
            //Mat mScaledDepth;//, hScaledDepth;
            //mImageDepth.convertTo(mScaledDepth, CV_8U, 255.0 / iMaxDepth);
            // cout << " 1 " << mImageDepth.ptr<ushort>(320)[240] << endl;
            // cout << " 2 " << mImageDepth.at<float>(320,240) << endl;
            //水平镜像深度图
            //hMirrorTrans(mScaledDepth, depthimage);
            //hMirrorTrans(mImageDepth, depthimage);
            mImageDepth.copyTo(depthimage);
        }

        rc = streamColor.readFrame(&frameColor);
        if (rc == STATUS_OK)
        {
            // 同样的将彩色图像数据转化成OpenCV格式
            const Mat mImageRGB(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
            // 首先将RGB格式转换为BGR格式
            // Mat cImageBGR,bImageBGR;//,hImageBGR;
            cvtColor(mImageRGB, rgbimage, CV_RGB2BGR);

            //水平镜像深度图
            //hMirrorTrans(cImageBGR, rgbimage);
            resize(rgbimage, rgbimage, Size(640, 480));     
        }

        setMouseCallback(RGB_WINDOW, onMouse, 0);

        if(bRenewROI)
        {
            if (selectRect.width <= 0 || selectRect.height <= 0)
            {
                bRenewROI = false;
                continue;
            }
            tracker.init(selectRect, rgbimage, depthimage);
            bBeginKCF = true;
            bRenewROI = false;
            enable_get_depth = false;
        }

        if(bBeginKCF)
        {
            result = tracker.update(rgbimage, depthimage);
            cv::rectangle(rgbimage, result, cv::Scalar( 0, 255, 255 ), 2, 8 );
            enable_get_depth = true;
        }
        else
            cv::rectangle(rgbimage, selectRect, cv::Scalar(255, 0, 0), 2, 8, 0);

        // 然后显示彩色图像
        imshow(RGB_WINDOW, rgbimage);

        if(enable_get_depth)
        {
            dist_val[0] = depthimage.ptr<ushort>(result.y+result.height/3)[result.x+result.width/3];
            dist_val[1] = depthimage.ptr<ushort>(result.y+result.height/3)[result.x+2*result.width/3];
            dist_val[2] = depthimage.ptr<ushort>(result.y+2*result.height/3)[result.x+result.width/3] ;
            dist_val[3] = depthimage.ptr<ushort>(result.y+2*result.height/3)[result.x+2*result.width/3] ;
            dist_val[4] = depthimage.ptr<ushort>(result.y+result.height/2)[result.x+result.width/2] ;

            for(int i = 0; i < 5; i++)
                dist_val[i] = dist_val[i] / 1000.0;

            float distance = 0;
            int num_depth_points = 5;
            for(int i = 0; i < 5; i++)
            {
                if(dist_val[i] > 0.4)
                    distance += dist_val[i];
                else
                    num_depth_points--;
            }
            distance /= num_depth_points;

            //calculate linear speed
            if(distance > Min_distance)
                linear_speed = distance * k_linear_speed + h_linear_speed;
            else
                linear_speed = 0;

            if(linear_speed > Max_linear_speed)
                linear_speed = Max_linear_speed;

            //calculate rotation speed
            int center_x = result.x + result.width/2;
            if(center_x < ERROR_OFFSET_X_left1) 
                rotation_speed =  Max_rotation_speed;
            else if(center_x > ERROR_OFFSET_X_left1 && center_x < ERROR_OFFSET_X_left2)
                rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_left;
            else if(center_x > ERROR_OFFSET_X_right1 && center_x < ERROR_OFFSET_X_right2)
                rotation_speed = -k_rotation_speed * center_x + h_rotation_speed_right;
            else if(center_x > ERROR_OFFSET_X_right2)
                rotation_speed = -Max_rotation_speed;
            else 
                rotation_speed = 0;

            // std::cout <<  "-----------------------------------------------------------------------"  << std::endl;
            // std::cout <<  "linear_speed = " << linear_speed << "  rotation_speed = " << -rotation_speed << std::endl;
            // std::cout <<  dist_val[0]  << " / " <<  dist_val[1] << " / " << dist_val[2] << " / " << dist_val[3] <<  " / " << dist_val[4] << std::endl;
            // std::cout <<  "distance = " << distance << std::endl;
        }
        // 显示出深度图像
        //imshow(DEPTH_WINDOW, depthimage);

        //sendVelocity(linear_speed, -rotation_speed);

        // 终止快捷键
        if (waitKey(30) >= 0)
        {
            //sendVelocity(0, 0);
            break;
        }
    }
    destroyWindow(RGB_WINDOW);
    // 关闭数据流
    streamDepth.destroy();
    streamColor.destroy();
    // 关闭设备
    xtion.close();
    // 最后关闭OpenNI
    OpenNI::shutdown();

    return 0;
}
