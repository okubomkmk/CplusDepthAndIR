#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <atlbase.h>
#include "C:\Users\mkuser\Documents\K4W2-Book-master\K4W2-Book-master\C++(Native)\02_Depth\KinectV2-Depth-01\KinectV2\ForEnglish.h"

using namespace std;
// 次のように使います
// ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
// 書籍での解説のためにマクロにしています。実際には展開した形で使うことを検討してください。
#define ERROR_CHECK( ret )  \
    if ( (ret) != S_OK ) {    \
        std::stringstream ss;	\
        ss << "failed " #ret " " << std::hex << ret << std::endl;			\
        throw std::runtime_error( ss.str().c_str() );			\
    }

class KinectApp
{
private:

    // Kinect SDK
    CComPtr<IKinectSensor> kinect = nullptr;

    CComPtr<IInfraredFrameReader> infraredFrameReader = nullptr;

    int infraredWidth;
    int infraredHeight;

    // 表示用
    std::vector<UINT16> infraredBuffer;
	CComPtr<IDepthFrameReader> depthFrameReader = nullptr;

	// 表示部分
	int depthWidth;
	int depthHeight;

	vector<UINT16> depthBuffer;
	int x1 = depthWidth / 2;
	int y1 = depthHeight / 2;
	int x2 = 300;
	int y2 = 300;

	const char* DepthWindowName = "Depth Image";

	bool OnePointisSelected = true;

public:

    ~KinectApp()
    {
        // Kinectの動作を終了する
        if ( kinect != nullptr ){
            kinect->Close();
        }
    }

    // 初期化
    void initialize()
    {
        // デフォルトのKinectを取得する
        ERROR_CHECK( ::GetDefaultKinectSensor( &kinect ) );
        ERROR_CHECK( kinect->Open() );

        // 赤外線画像リーダーを取得する
        CComPtr<IInfraredFrameSource> infraredFrameSource;
        ERROR_CHECK( kinect->get_InfraredFrameSource( &infraredFrameSource ) );
        ERROR_CHECK( infraredFrameSource->OpenReader( &infraredFrameReader ) );

        // 赤外線画像のサイズを取得する
        CComPtr<IFrameDescription> infraredFrameDescription;
        ERROR_CHECK( infraredFrameSource->get_FrameDescription(
                                                &infraredFrameDescription ) );
        ERROR_CHECK( infraredFrameDescription->get_Width( &infraredWidth ) );
        ERROR_CHECK( infraredFrameDescription->get_Height( &infraredHeight ) );

        // バッファーを作成する
        infraredBuffer.resize( infraredWidth * infraredHeight );


		// Depthリーダーを取得する
		CComPtr<IDepthFrameSource> depthFrameSource;
		ERROR_CHECK(kinect->get_DepthFrameSource(&depthFrameSource));
		ERROR_CHECK(depthFrameSource->OpenReader(&depthFrameReader));

		// Depth画像のサイズを取得する
		CComPtr<IFrameDescription> depthFrameDescription;
		ERROR_CHECK(depthFrameSource->get_FrameDescription(
			&depthFrameDescription));
		ERROR_CHECK(depthFrameDescription->get_Width(&depthWidth));
		ERROR_CHECK(depthFrameDescription->get_Height(&depthHeight));

		x1 = depthWidth / 2;
		y1 = depthHeight / 2;

		x2 = 300;
		y2 = 300;
		// Depthの最大値、最小値を取得する
		UINT16 minDepthReliableDistance;
		UINT16 maxDepthReliableDistance;
		ERROR_CHECK(depthFrameSource->get_DepthMinReliableDistance(
			&minDepthReliableDistance));
		ERROR_CHECK(depthFrameSource->get_DepthMaxReliableDistance(
			&maxDepthReliableDistance));
		std::cout << "Depth最小値       : " << minDepthReliableDistance << endl;
		cout << "Depth最大値       : " << maxDepthReliableDistance << endl;

		// バッファーを作成する
		depthBuffer.resize(depthWidth * depthHeight);

		cv::namedWindow(DepthWindowName);
		cv::setMouseCallback(DepthWindowName, &KinectApp::mouseCallback, this);

    }
	static void mouseCallback(int event, int x, int y, int flags, void* userdata)
	{
		// 引数に渡したthisポインタを経由してメンバ関数に渡す
		auto pThis = (KinectApp*)userdata;
		pThis->mouseCallback(event, x, y, flags);
	}

	// マウスイベントのコールバック(実処理)
	void mouseCallback(int event, int x, int y, int flags)
	{
		if (event == CV_EVENT_LBUTTONDOWN) {
			x1 = x;
			y1 = y;
			
		}

		if (event == CV_EVENT_RBUTTONDOWN){
			x2= x;
			y2= y;
		}
	}

    void run()
    {
        while ( 1 ) {
            update();
            draw();

            auto key = cv::waitKey( 10 );
            if ( key == 'q' ){
                break;
            }
        }
    }

private:

    // データの更新処理
    void update()
    {
        updateInfrared();
		updateDepthFrame();
    }



    void updateInfrared()
    {
        // フレームを取得する
        CComPtr<IInfraredFrame> infraredFrame;
        auto ret = infraredFrameReader->AcquireLatestFrame( &infraredFrame );
        if ( FAILED( ret ) ){
            return;
        }

        // データを取得する
        ERROR_CHECK( infraredFrame->CopyFrameDataToArray(
            infraredBuffer.size(), &infraredBuffer[0] ) );
    }

	void updateDepthFrame()
	{
		// Depthフレームを取得
		CComPtr<IDepthFrame> depthFrame;
		auto ret = depthFrameReader->AcquireLatestFrame(&depthFrame);
		if (ret != S_OK){
			return;
		}

		// データを取得する
		ERROR_CHECK(depthFrame->CopyFrameDataToArray(
			depthBuffer.size(), &depthBuffer[0]));


	}
    void draw()
    {
		drawInfraredFrame();
		drawDepthFrame();
    }

	void drawInfraredFrame()
	{
		// カラーデータを表示する
		cv::Mat colorImage(infraredHeight, infraredWidth,
			CV_16UC1, &infraredBuffer[0]);
		cv::rectangle(colorImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(65535, 65535, 65535), 1, 8, 0);

		cv::imshow("Infrared Image", colorImage);

	}

	void drawDepthFrame()
	{
		// Depthデータを表示するかコレ?
		cv::Mat depthImage(depthHeight, depthWidth, CV_8UC1);
		// フィルタ後
		// Depthデータを0-255のグレーデータにする


		for (int i = 0; i < depthImage.total(); ++i){
			depthImage.data[i] = depthBuffer[i] * 255 / 8000;
		}

		
		// Depthデータのインデックスを取得して、その場所の距離を表示する
		int index = (y1 * depthWidth) + x1;
		std::stringstream ss;
		ss << depthBuffer[index] << "mm X=" << x1 << " Y= " << y1;

		cv::circle(depthImage, cv::Point(x1, y1), 3,
			cv::Scalar(255, 255, 255), 2);

		cv::circle(depthImage, cv::Point(x2, y2), 3,
			cv::Scalar(255, 255, 255), 2);
		cv::putText(depthImage, ss.str(), cv::Point(x1, y1),
			0, 0.5, cv::Scalar(255, 255, 255));
		cv::putText(depthImage, ss.str(), cv::Point(x2, y2),
			0, 0.5, cv::Scalar(255, 255, 255));
		cv::rectangle(depthImage, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 255, 255), 1, 8, 0);


		cv::imshow(DepthWindowName, depthImage);

	}
};

void main()
{
    try {
        KinectApp app;
        app.initialize();
        app.run();
    }
    catch ( std::exception& ex ){
        std::cout << ex.what() << std::endl;
    }
}
