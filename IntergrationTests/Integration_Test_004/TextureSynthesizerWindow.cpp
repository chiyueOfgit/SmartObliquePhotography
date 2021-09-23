#include "pch.h"
#include "TextureSynthesizerWindow.h"
#include <QtWidgets/qmdisubwindow.h>
#include <QtWidgets/QFileDialog>
#include <qobject.h>
#define STB_IMAGE_STATIC
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_STATIC
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

CTextureSynthesizerWindow::CTextureSynthesizerWindow(QWidget * vParent)
    : QMainWindow(vParent)
{
    m_WindowUI.setupUi(this);
    __connectSignals();
    
}

CTextureSynthesizerWindow::~CTextureSynthesizerWindow()
{
}

void CTextureSynthesizerWindow::__connectSignals()
{
    QObject::connect(m_WindowUI.actionLoadInput, SIGNAL(triggered()), this, SLOT(__onActionLoadInput()));
    QObject::connect(m_WindowUI.actionLoadOutput, SIGNAL(triggered()), this, SLOT(__onActionLoadOutput()));
    QObject::connect(m_WindowUI.actionLoadMask, SIGNAL(triggered()), this, SLOT(__onActionLoadMask()));
    QObject::connect(m_WindowUI.actionExecute, SIGNAL(clicked()), this, SLOT(__onActionExecute()));
}

void CTextureSynthesizerWindow::__onActionLoadInput()
{
    auto Input = QFileDialog::getOpenFileName(this, tr("Load Input"), QString::fromStdString(m_Path), tr("Image File(*.png *.jpg)")).toStdString();
    if (Input != "")
    {
        QImage* Image = new QImage;
        if (!(Image->load(QString::fromStdString(Input))))
        {
            delete Image;
            return;
        }
        m_WindowUI.Input->setPixmap(QPixmap::fromImage(*Image));
        __loadImage(Input, m_Input);
    }  
}

void CTextureSynthesizerWindow::__onActionLoadOutput()
{
    auto Output = QFileDialog::getOpenFileName(this, tr("Load Output"), QString::fromStdString(m_Path), tr("Image File(*.png *.jpg)")).toStdString();
    if (Output != "")
    {
        QImage* Image = new QImage;
        if (!(Image->load(QString::fromStdString(Output))))
        {
            delete Image;
            return;
        }
        m_WindowUI.Output->setPixmap(QPixmap::fromImage(*Image));
        __loadImage(Output, m_Output);
        m_OutputName = Output.substr(Output.find_last_of('/') + 1, Output.find_last_of('.') - Output.find_last_of('/') - 1);
    }
}

void CTextureSynthesizerWindow::__onActionLoadMask()
{
    auto Mask = QFileDialog::getOpenFileName(this, tr("Load Mask"), QString::fromStdString(m_Path), tr("Image File(*.png *.jpg)")).toStdString();
    if (Mask != "")
    {
        QImage* Image = new QImage;
        if (!(Image->load(QString::fromStdString(Mask))))
        {
            delete Image;
            return;
        }
        m_WindowUI.Mask->setPixmap(QPixmap::fromImage(*Image));
        __loadMask(Mask, m_Mask);
    }
}

void CTextureSynthesizerWindow::__onActionExecute()
{
    if (m_Input.rows() && m_Output.rows() && m_Mask.rows())
    {
        CTreeBasedTextureSynthesizer<int, 3> TextureSynthesizer;
        TextureSynthesizer.execute(m_Input, m_Mask, m_Output);
        auto SavePath = m_Path + "/Results/" + m_OutputName + ".png";
        __saveImage(m_Output, SavePath);
        QImage* Image = new QImage;
        if (!(Image->load(QString::fromStdString(SavePath))))
        {
            delete Image;
            return;
        }
        m_WindowUI.Result->setPixmap(QPixmap::fromImage(*Image));
    }
}

void CTextureSynthesizerWindow::__loadImage(const std::string& vImagePath, Eigen::Matrix<Eigen::Vector3i, -1, -1>& voTexture)
{
    const char* filepath = vImagePath.c_str();
    int Width, Height, BytesPerPixel;
    unsigned char* ImageData = stbi_load(filepath, &Width, &Height, &BytesPerPixel, 0);

    if (ImageData)
    {
        voTexture.resize(Height, Width);
        for (int i = 0; i < Height; i++)
            for (int k = 0; k < Width; k++)
                for (int Offset = 0; Offset < 3; Offset++)
                    voTexture(i, k)[Offset] = ImageData[(i * Width + k) * BytesPerPixel + Offset];
    }
}

void CTextureSynthesizerWindow::__loadMask(const std::string& vImagePath, Eigen::MatrixXi& voMask)
{
    const char* filepath = vImagePath.c_str();
    int Width, Height, BytesPerPixel;
    unsigned char* ImageData = stbi_load(filepath, &Width, &Height, &BytesPerPixel, 0);

    if (ImageData)
    {
        voMask.resize(Height, Width);
        for (int i = 0; i < Height; i++)
            for (int k = 0; k < Width; k++)
                voMask(i, k) = ImageData[(i * Width + k) * BytesPerPixel] / 255;
    }
}

void CTextureSynthesizerWindow::__saveImage(const Eigen::Matrix<Eigen::Vector3i, -1, -1>& vTexture, const std::string& vOutputImagePath)
{
    const auto Width = vTexture.cols();
    const auto Height = vTexture.rows();
    const auto BytesPerPixel = 3;
    auto ResultImage = new unsigned char[Width * Height * BytesPerPixel];
    for (auto i = 0; i < Height; i++)
        for (auto k = 0; k < Width; k++)
        {
            auto Offset = (i * Width + k) * BytesPerPixel;
            ResultImage[Offset] = vTexture.coeff(i, k)[0];
            ResultImage[Offset + 1] = vTexture.coeff(i, k)[1];
            ResultImage[Offset + 2] = vTexture.coeff(i, k)[2];
        }

    auto Extension = vOutputImagePath.substr(vOutputImagePath.find_last_of(".") + 1, vOutputImagePath.size());
    if (Extension == "jpg")
        stbi_write_jpg(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
    else if (Extension == "png")
        stbi_write_png(vOutputImagePath.c_str(), Width, Height, BytesPerPixel, ResultImage, 0);
    stbi_image_free(ResultImage);
}
