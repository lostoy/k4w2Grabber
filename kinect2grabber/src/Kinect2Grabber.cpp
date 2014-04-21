#include "Kinect2Grabber.h"

void Kinect2Grabber::start()
{
    if (!m_isInitialized)
    {
        InitializeDefaultSensor();
        m_isInitialized = true;
    }
    if (!m_isStarted)
    {
        m_isStarted = true;
        m_pThread.reset(new std::thread([&]()
        {
            MainLoop();
        }));
    }
}

void Kinect2Grabber::stop()
{
	if (m_isStarted)
	{
		m_isStarted = false;
		m_pThread->detach();
		m_pThread.release();
	}
}

HRESULT Kinect2Grabber::InitializeDefaultSensor()
{
    HRESULT hr;

    hr = GetDefaultKinectSensor(&m_pKinectSensor);
    if (FAILED(hr))
    {
        return hr;
    }

    if (m_pKinectSensor)
    {
        // Initialize the Kinect and get the depth reader
        IDepthFrameSource* pDepthFrameSource = NULL;

        hr = m_pKinectSensor->Open();

        if (SUCCEEDED(hr))
        {
            hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
        }

        hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

        hr = m_pKinectSensor->OpenMultiSourceFrameReader(FrameSourceTypes_Color | FrameSourceTypes_Depth, &m_pMultiSourceFrameReader);

        SafeRelease(pDepthFrameSource);
    }

    if (!m_pKinectSensor || FAILED(hr))
    {
        std::cout << "No ready Kinect found!" << std::endl;
        return E_FAIL;
    }

    return hr;
}

void Kinect2Grabber::Update()
{
    if (!m_pMultiSourceFrameReader)
    {
        return;
    }

    IMultiSourceFrame* pMultiSourceFrame = NULL;
    HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

    if (SUCCEEDED(hr))
    {
        IDepthFrameReference* pDepthFrameReference = NULL;
        IDepthFrame* pDepthFrame = NULL;

        HRESULT hrDepth = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
        if (SUCCEEDED(hrDepth))
        {
            hrDepth = pDepthFrameReference->AcquireFrame(&pDepthFrame);
        }

        IColorFrameReference* pColorFrameReference = NULL;
        IColorFrame* pColorFrame = NULL;

        HRESULT hrColor = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
        if (SUCCEEDED(hrColor))
        {
            hrColor = pColorFrameReference->AcquireFrame(&pColorFrame);
        }

        if (SUCCEEDED(hrDepth) && SUCCEEDED(hrColor))
        {
            ProcessFrame(pColorFrame, pDepthFrame);
        }
        SafeRelease(pDepthFrame);
        SafeRelease(pDepthFrameReference);
        SafeRelease(pColorFrame);
        SafeRelease(pColorFrameReference);
    }
    SafeRelease(pMultiSourceFrame);
}

void Kinect2Grabber::ProcessFrame(IColorFrame* pColorFrame, IDepthFrame* pDepthFrame)
{
    IFrameDescription* pFrameDescription = NULL;

    int nDepthWidth = 0;
    int nDepthHeight = 0;
    USHORT nDepthMinReliableDistance = 0;
    USHORT nDepthMaxReliableDistance = 0;
    UINT nDepthBufferSize = 0;
    UINT16 *pDepthBuffer = NULL;

    HRESULT hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
    if (SUCCEEDED(hr))
    {
        pFrameDescription->get_Width(&nDepthWidth);
        pFrameDescription->get_Height(&nDepthHeight);
        pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
        pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
        pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
    }
    SafeRelease(pFrameDescription);

    m_pDepthBufferSignal->operator() (pDepthBuffer);

    int nColorWidth = 0;
    int nColorHeight = 0;
    ColorImageFormat imageFormat = ColorImageFormat_None;
    UINT nColorBufferSize = 0;
    RGBQUAD *pColorBuffer = NULL;

    hr = pColorFrame->get_FrameDescription(&pFrameDescription);
    if (SUCCEEDED(hr))
    {
        pFrameDescription->get_Width(&nColorWidth);
        pFrameDescription->get_Height(&nColorHeight);
        pColorFrame->get_RawColorImageFormat(&imageFormat);

        if (imageFormat == ColorImageFormat_Bgra)
        {
            hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
        }
        else
        {
            pColorBuffer = m_pColorRGBX;
            nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
            hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
        }
    }
    SafeRelease(pFrameDescription);

    pcl::PointCloud<pcl::PointXYZRGBA>* pCloud = new pcl::PointCloud<pcl::PointXYZRGBA>();
    pCloud->width = cColorWidth;
    pCloud->height = cColorHeight;
    pCloud->is_dense = false;
    pCloud->points.resize(cColorWidth * cColorHeight);

    if (pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight))
    {
        for (int y = 0; y < cColorHeight; ++y)
        {
            for (int x = 0; x < cColorWidth; ++x)
            {
                pcl::PointXYZRGBA &pt = pCloud->at(x, y);
                pt.r = pColorBuffer[y * cColorWidth + x].rgbRed;
                pt.g = pColorBuffer[y * cColorWidth + x].rgbGreen;
                pt.b = pColorBuffer[y * cColorWidth + x].rgbBlue;
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    /*const float centerX = ((float)cDepthWidth - 1.f) / 2.f;
    const float centerY = ((float)cDepthHeight - 1.f) / 2.f;
    const float scaleFactorX = 1 / 525.f * 640.f / static_cast<float> (dims[0]);
    const float scaleFactorY =*/

    const float centerX = 253.892f;
    const float centerY = 212.845f;
    const float scaleFactorX = 1.f / 375.f;
    const float scaleFactorY = 1.f / 375.f;
    const float depth_image_units = 1E-3f;

    if (pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight))
    {
        for (int y = 0; y < cDepthHeight; ++y)
        {
            for (int x = 0; x < cDepthWidth; ++x)
            {
                UINT16 nDepth;
                nDepth = pDepthBuffer[y * cDepthWidth + x];

                DepthSpacePoint ptDepth;
                ptDepth.X = x;
                ptDepth.Y = y;
                ColorSpacePoint ptColor;
                m_pCoordinateMapper->MapDepthPointToColorSpace(ptDepth, nDepth, &ptColor);

                if (ptColor.X <0 || ptColor.Y <0 || ptColor.X > cColorWidth - 1 || ptColor.Y>cColorHeight - 1)
                {
                }
                else
                {
                    pcl::PointXYZRGBA &pt = pCloud->at((int)ptColor.X, (int)ptColor.Y);
                    float depth = static_cast<float> (nDepth)* depth_image_units;
                    if (depth == 0.0f)
                        pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                    else
                    {
                        pt.x = ((float)x - centerX) * scaleFactorX * depth;
                        pt.y = -((float)y - centerY) * scaleFactorY * depth;
                        pt.z = depth;
                    }
                }
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(pCloud);
    m_pSignal->operator() (cloud);
}