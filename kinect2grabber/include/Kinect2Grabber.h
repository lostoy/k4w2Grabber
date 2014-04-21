//kinect2grabber, written by taoyu
#ifndef KINECT_2_GRABBER_H_
#define KINECT_2_GRABBER_H_

#include <string>
#include <vector>

#include <pcl/common/io.h>
#include <pcl/io/grabber.h>

#include <pcl/common/time_trigger.h>
#include <pcl/conversions.h>

#include <boost/signals2.hpp>

#include <Kinect.h>

#include <thread>

class Kinect2Grabber : public pcl::Grabber
{
public:

    template<class Interface>
    inline void SafeRelease(Interface *& pInterfaceToRelease)
    {
        if (pInterfaceToRelease != NULL)
        {
            pInterfaceToRelease->Release();
            pInterfaceToRelease = NULL;
        }
    }

    Kinect2Grabber() : m_isInitialized(false), m_isStarted(false)
    {
        m_pSignal = createSignal<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&)>();
        m_pDepthBufferSignal = createSignal<void(UINT16*)>();
        m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];
    }

    virtual ~Kinect2Grabber()
    {
        stop();
        if (m_pColorRGBX)
        {
            delete m_pColorRGBX;
            m_pColorRGBX = NULL;
        }
    }

    virtual void start();

    virtual void stop();

    virtual bool isRunning() const
    {
        return m_isStarted;
    };

    virtual std::string getName() const
    {
        return std::string("Kinect2Grabber");
    };

    virtual float getFramesPerSecond() const
    {
        //TODO
        return 0;
    };
private:

    HRESULT InitializeDefaultSensor();

    void Update();

    void ProcessFrame(IColorFrame* pColorFrame, IDepthFrame* pDepthFrame);

    void MainLoop()
    {
        while (m_isStarted)
        {
            Update();
            Sleep(100);
        }
    }

    boost::signals2::signal<void(const boost::shared_ptr<const pcl::PointCloud<pcl::PointXYZRGBA> >&)>* m_pSignal;
    boost::signals2::signal<void(UINT16*)>* m_pDepthBufferSignal;

    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;

    IMultiSourceFrameReader*		m_pMultiSourceFrameReader;
    RGBQUAD* m_pColorRGBX;

    bool m_isStarted;
    bool m_isInitialized;
    std::unique_ptr<std::thread> m_pThread;


    static const int cColorWidth = 1920;
    static const int cColorHeight = 1080;
    static const int cDepthWidth = 512;
    static const int cDepthHeight = 424;
};

#endif
