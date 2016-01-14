#include <iostream>
#include <sstream>
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <atlbase.h>
#include <windef.h>
#include <iostream>
#include <fstream>
using namespace std;

#define WRITEFRAMENUM 1024
#define FILENAME 2000
#define MAXIRVALUE 4000

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

	POINT R1;
	POINT R2;
	POINT Begin, End;
	POINT SizeofCaputuredFrame;

	vector<UINT16> saveDepthArray;
	vector<UINT16> saveInfraredArray;
	vector<UINT16> centerDepthArray;
	vector<UINT16> centerInfraredArray;

	std::ofstream centerDepth;
	std::ofstream centerInfrared;
	std::ofstream DepthArray;
	std::ofstream InfraredArray;
	std::ofstream WriteFrameSize;
	vector<stringstream> ss;

	const char* DepthWindowName = "Depth Image";

	bool OnePointisSelected = true;
	bool savingFlag = false;
	bool arrayResized = false;
	int frameCounter = 0;

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

		R1.x = depthWidth / 2;
		R1.y = depthHeight / 2;

		R2.x = 300;
		R2.y = 300;
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
		ss.resize(4);
		
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
			R1.x = x;
			R1.y = y;
			
		}

		if (event == CV_EVENT_RBUTTONDOWN){
			R2.x= x;
			R2.y= y;
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

			if (key =='s')
			{
				savingFlag = true;
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
		setTextOver();
		
		

		if (savingFlag){
			if (!arrayResized){
				initializeForSave();
			}
			saveIntoArray();
		}

		else{
			drawInfraredFrame();
			drawDepthFrame();
		}
    }

	void drawInfraredFrame()
	{
		// カラーデータを表示する
		
		cv::Mat colorImage(infraredHeight, infraredWidth,
			CV_16UC1, &infraredBuffer[0]);


		cv::circle(colorImage, cv::Point(R1.x, R1.y), 3,
			cv::Scalar(65535, 65535, 65535), 2);
		cv::circle(colorImage, cv::Point(R2.x, R2.y), 3,
			cv::Scalar(65535, 65535, 65535), 2);
		cv::putText(colorImage, ss[2].str(), cv::Point(R1.x, R1.y),
			0, 0.5, cv::Scalar(65535, 65535, 65535));
		cv::putText(colorImage, ss[3].str(), cv::Point(R2.x, R2.y),
			0, 0.5, cv::Scalar(65535, 65535, 65535));
		cv::rectangle(colorImage, cv::Point(R1.x, R1.y), cv::Point(R2.x, R2.y), cv::Scalar(65535, 65535, 65535), 1, 8, 0);

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
		

		cv::circle(depthImage, cv::Point(R1.x, R1.y), 3,
			cv::Scalar(255, 255, 255), 2);
		cv::circle(depthImage, cv::Point(R2.x, R2.y), 3,
			cv::Scalar(255, 255, 255), 2);
		cv::putText(depthImage, ss[0].str(), cv::Point(R1.x, R1.y),
			0, 0.5, cv::Scalar(255, 255, 255));
		cv::putText(depthImage, ss[1].str(), cv::Point(R2.x, R2.y),
			0, 0.5, cv::Scalar(255, 255, 255));
		cv::rectangle(depthImage, cv::Point(R1.x, R1.y), cv::Point(R2.x, R2.y), cv::Scalar(255, 255, 255), 1, 8, 0);
		//cv effect for infrared image


		cv::imshow(DepthWindowName, depthImage);

	}
	void initializeForSave(){

		centerDepth.open("V:\\Eng\\FrameData\\DepthCenter" + to_string(FILENAME) + ".dat");
		centerInfrared.open("V:\\Eng\\FrameData\\InfraredCenter" + to_string(FILENAME) + ".dat");
		DepthArray.open("V:\\Eng\\FrameData\\DepthMeasure" + to_string(FILENAME) + ".dat");
		InfraredArray.open("V:\\Eng\\FrameData\\InfraredMeasure" + to_string(FILENAME) + ".dat");
		WriteFrameSize.open("V:\\Eng\\FrameData\\sizeofframe" + to_string(FILENAME) + ".dat");
		
		Begin.x = R1.x <= R2.x ? R1.x : R2.x;
		End.x = R1.x > R2.x ? R1.x : R2.x;
		Begin.y = R1.y <= R2.y ? R1.y : R2.y;
		End.y = R1.y > R2.y ? R1.y : R2.y;
		SizeofCaputuredFrame.x = (End.x - Begin.x + 1);
		SizeofCaputuredFrame.y = (End.y - Begin.y + 1);
		saveDepthArray.resize( SizeofCaputuredFrame.x* SizeofCaputuredFrame.y * WRITEFRAMENUM);
		saveInfraredArray.resize(SizeofCaputuredFrame.x * SizeofCaputuredFrame.y * WRITEFRAMENUM);

		centerDepthArray.resize(WRITEFRAMENUM);
		centerInfraredArray.resize(WRITEFRAMENUM);
		arrayResized = true;



	}

	void saveIntoArray()
	{
		int index;
		int index_of_array = 0;
		for (int j = Begin.y; j <= End.y; j++){
			for (int i = Begin.x; i <= End.x; i++){
				index = j * depthWidth + i;
				saveDepthArray[index_of_array + frameCounter * SizeofCaputuredFrame.x * SizeofCaputuredFrame.y] = depthBuffer[index];
				saveInfraredArray[index_of_array + frameCounter * SizeofCaputuredFrame.x * SizeofCaputuredFrame.y] = infraredBuffer[index];
				index_of_array++;
			}
		}
		centerDepthArray[frameCounter] = depthBuffer[Begin.y * depthWidth + Begin.x];
		centerInfraredArray[frameCounter] = infraredBuffer[Begin.y * infraredWidth + Begin.x];
		frameCounter++;
		if (frameCounter == WRITEFRAMENUM - 1){
			savingFlag = false;
			arrayResized = false;
			writedownToFile();

		}
	}

	void writedownToFile(){
		for (int i = 0; i < SizeofCaputuredFrame.x * SizeofCaputuredFrame.y * WRITEFRAMENUM; i++){
			DepthArray << saveDepthArray[i] << "\r\n";
			InfraredArray << saveInfraredArray[i] << "\r\n";
			
		}
		for (int j = 0; j < WRITEFRAMENUM; j++){
			centerDepth << centerDepthArray[j] << "\r\n";
			centerInfrared << centerInfraredArray[j] << "\r\n";
		}
		DepthArray << std::endl;
		InfraredArray << std::endl;
		centerDepth << std::endl;
		centerInfrared << std::endl;

		WriteFrameSize << SizeofCaputuredFrame.x << "\r\n" << SizeofCaputuredFrame.y << "\r\n" << WRITEFRAMENUM << endl;


	}
	// for visible ごみ関数


	void setTextOver(){
		vector<int> index(2);

		index[0] = R1.y * depthWidth + R1.x;
		index[1] = R2.y * depthWidth + R2.x;
		for (int i = 0; i < 4; i++){
			ss[i].str("");
		}
		//depth overray
		ss[0] << depthBuffer[index[0]] << "mm X=" << R1.x << " Y= " << R1.y << " " << frameCounter + 1;
		ss[1] << depthBuffer[index[1]] << "mm X=" << R2.x << " Y= " << R2.y;
		//ir overray
		ss[2] << infraredBuffer[index[0]] << " X=" << R1.x << " Y= " << R1.y << " " << frameCounter + 1;
		ss[3] << infraredBuffer[index[1]] << " X=" << R2.x << " Y= " << R2.y;
		//cv effect for depth image
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

