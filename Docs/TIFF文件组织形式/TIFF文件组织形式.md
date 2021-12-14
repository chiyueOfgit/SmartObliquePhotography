#  TIFF文件组织形式

2021.11.26

参考资料：[https://www.cnblogs.com/gywei/p/3393816.html](https://www.cnblogs.com/gywei/p/3393816.html)和[https://www.awaresystems.be/imaging/tiff/specification/TIFF6.pdf](https://www.awaresystems.be/imaging/tiff/specification/TIFF6.pdf)

#### 一、简介

- TIFF文件以 .tif 或 .tiff 为扩展名。其数据格式是一种3级体系结构，TIFF 文件内部结构可以分成三个部分，分别是：文件头信息区（IFH）、图像文件目录（IFD）和图像数据区。

- 在TIFF文件中的每个IFD代表一张图像，一个TIFF文件可以有多个IFD，也就是可以有多张图像。一个文件的图像之间不需要相关。TIFF可以利用多帧的特点描述三维图像

- 每个IFD中有许多标签（tag），所有的标签都是以升序排列，这些标签信息是用来保存和处理文件中的图像信息的。可以通过设置图像的IFD中的SamplesPerPixel、BitsPerSample、SampelFormat、PhotometricInterpretation等标签，分别表示二值图像、8位和16位灰度图、浮点数灰度图、RGB以及RGBA等图像。例如16位灰度图中SamplesPerPixel=1，BitsPerSample=16；浮点数灰度图中SamplesPerPixel=1，BitsPerSample=32，SampelFormat=3。如下图:

  <img src="pic\tags_16bits_grey.png" style="zoom:50%;" />

  <img src="pic\tags_32bits_grey_tiled.png" style="zoom:50%;" />

- TIFF中可以将图片中图像数据按**固定数目的行数合并成一个 strip** 或者**分为固定长宽的小块（tile）**进行进行独立的存储、压缩，读取，可以快速读取文件中指定区域的图片信息，即不需要加载整个文件，加快了访问速度。对于strip，有StripOffsets、RowsPerStrip（每个strip的行数）等tag来表示属性；对于tile，有TileWidth和TileLength来表示一个tile的大小，以及一些其他tag，如上图。（详细可见下方对各个tag的含义和值介绍）

#### 二、TIFF 图像文件的一般组织形式是：IFH -- 图像数据 -- IFD

1. ###### IFH$(Image File Header)$——图像文件头

   8字节的IFH一定在文件的最开始位置，其中$Offset$  $to$ $first$ $IFD$是4字节，所以TIFF文件最大为4GB。

   | 名称                          | 字节数 | 数据类型          | 说明                                                         |
   | ----------------------------- | ------ | ----------------- | ------------------------------------------------------------ |
   | $Byteorder$                   | 2      | $Interger$        | TIF标记，位于文件最开始，其取值为 0x4d4d或0x4949<br/>H4d4d表示该图是摩托罗拉整数格式，H4949表示该图是Intel整数格式。 |
   | $Version$                     | 2      | $Interger$        | 版本号，其值恒为2A00                                         |
   | $Offset$  $to$ $first$  $IFD$ | 4      | $Unsigned$ $Long$ | 第一个 IFD 相对于文件起始位置的偏移量（对于多页 TIFF 可以存在多个 IFD） |

2. ###### IFD$(Image File Directory)$——图像文件目录

   一个图像的目录有关图像数据格式和内容的所有信息。一个图像（帧）有一个目录。

   | 名称                          | 字节数 | 数据类型          | 说明                                                         |
   | ----------------------------- | ------ | ----------------- | ------------------------------------------------------------ |
   | $Directory$ $Entry$ $Count$   | 2      | $Interger$        | 本IFD中DE的数量                                              |
   | $Directory$ $Entry(1)$        | 12     | $Interger$        | 表示图像的某一个属性，共有 Directory Entry Count 个          |
   | $Directory$ $Entry(2)$        | 12     | $Interger$        |                                                              |
   | ...                           |        |                   |                                                              |
   | $Directory$ $Entry(n)$        | 12     | $Interger$        |                                                              |
   | $Offset$  $to$  $next$  $IFD$ | 4      | $Unsigned$ $Long$ | 下一个 IFD 的偏移量，如果已经是最后一个IFD，则此值为 NULL(0x00000000) |

3. ###### $Directory$ $Entry$

   Directory Entry 简称 DE, 简单的说，一个 DE 记录一个图像的属性，例如图像的 长、宽、分辨率等。

   | 名称          | 字节数 | 数据类型          | 说明                                                         |
   | ------------- | ------ | ----------------- | ------------------------------------------------------------ |
   | $tag$         | 2      | $Interger$        | 本属性的标签编号.<br />在 IFD 中，多个 DE 的 Tag 值是升序排列的，此编号可以在 TIFF 的规范中找到 |
   | $type$        | 2      | $Interger$        | 该属性的数据的数据类型，该值可以在 TIFF 规范中找到           |
   | $length$      | 4      | $Unsigned$ $Long$ | 该类型数据的数量（此处为数据的数量而不是数据的字节数）       |
   | $valueOffset$ | 4      | $Unsigned$ $Long$ | 若数据的数量乘以数据类型的字节长度（即实际数据的字节数）小于等于 4，则此处存储数据，否则，此处存储数据位置的偏移量（相对于文件头的偏移量） |

4. ###### $tag$

   更多 $tagID$ 参见[https://www.awaresystems.be/imaging/tiff/tifftags/baseline.html](https://www.awaresystems.be/imaging/tiff/tifftags/baseline.html)和TIFF6.0.pdf

   每个标签都有对应的标识（码），他们通常是被注册的。文件中遇到的任何未知标识都会被忽略。
   
   | 码（十进制） | $TagID$                     | 说明                                                         |
   | ------------ | --------------------------- | ------------------------------------------------------------ |
   | 256          | $ImageWidth$                | $SHORT$ OR $LONG$<br/>图像宽度，$length$为1                  |
   | 257          | $ImageLength$               | $SHORT$ OR $LONG$<br/>图像高度，$length$为1                  |
   | 258          | $BitsPerSample$             | $SHORT$<br/>每个分量的Bit数<br/>$length$为$SamplesPerPixel$  |
   | 259          | $Compression$               | $SHORT$<br/>压缩类型，$length$为1                            |
   | 262          | $PhotometricInterpretation$ | $SHORT$ 类型，$length$为1<br/>**0 = WhiteIsZero.** 对于二值图像和灰度图像：0表示最亮灰度<br />**1 = BlackIsZero.** 对于二值图像和灰度图像：0表示最暗灰度<br />**2 = RGB.** 正常RGB图像，存储顺序为R，G，B<br />**3 = Palette color.** 索引图像，其中，$ColorMap$必须定义，$SamplesPerPixel$必须为1 |
   | 273          | $StripOffsets$              | $SHORT$ OR $LONG$<br/>每个Strip的字节偏移量                  |
   | 277          | $SamplesPerPixel$           | $SHORT$<br/>每个象素的通道数，$length$为1                    |
   | 278          | $RowsPerStrip$              | $SHORT$ OR $LONG$<br/>每个Strip的行数<br/>$StripsPerImage = floor ((ImageLength + RowsPerStrip - 1) / RowsPerStrip)$ |
   | 284          | $PlanarConfig$              | **1 = Chunky format**。每个像素的通道值是连续存储的。例如，对于 RGB 数据，数据存储为 RGBRGBRGB<br/>**2 = Planar format**。每种通道分别存储。例如，在RGB 数据的存储中，红色通道的值一起存储在一个plane中，绿色分量存储在另一个plane中，蓝色分量存储在另一个plane中。 |
   | 339          | $SampleFormat$              | $SHORT$<br/>每个像素数值的解译方式<br/>**1 = unsigned integer data<br/>2 = two's complement signed integer data<br/>3 = IEEE floating point data(对float类型数据使用)<br/>4 = undefined data format** |

#### 三、TIFF 的读写

1. ##### libtiff库

   libtiff库的最新的最新版本可以从[http://www.libtiff.org/](http://www.libtiff.org/)下载

   ```
   #include "tiff.h"
   #include "tiffio.h"
   ```

   ###### 1.1 读写函数

   ###### 参见[http://www.libtiff.org/libtiff.html](http://www.libtiff.org/libtiff.html)和[http://www.libtiff.org/man/](http://www.libtiff.org/man/)

   - 读取图像

     ```c++
     //只读的方式打开文件
     TIFF* tif = TIFFOpen("foo.tif", "r");
     //创建或覆盖 TIFF 图像,如果文件已经存在，它首先被截断为零长度
     TIFF* tif = TIFFOpen("foo.tif", "w");
     //关闭
     TIFFClose(tif);
     ```

   - TIFF目录读写

     ```c++
     //获得图像的总目录数量
     int TIFFNumberOfDirectories(TIFF* tif)
     //选中第s帧,可以用从0开始的序号选择任意一帧图像
     int TIFFSetDirectory(TIFF* tif, tdir_t dirnum);
     //读取指定文件中的下一个目录并使其成为当前目录,使用while循环，用于顺序读取每一目录
     int TIFFReadDirectory(TIFF* tif)
     //使用while循环，用于顺序写每一帧
     int TIFFWriteDirectory(TIFF* tif)
     //返回当前目录的索引
     int TIFFCurrentDirectory((TIFF* tif))
     ```
     
   - 图像的属性的读写

     ```c++
     //获得单帧图像的许多参数，包括单帧图像的长、宽、图像位深、通道数、方向等
     int TIFFGetField(TIFF* tif, ttag_t tag, ...)
     //设置单帧图像的许多参数，包括单帧图像的长、宽、图像位深、通道数、方向等
     int TIFFSetField(TIFF* tif, ttag_t tag, ...)
     //标签的含义和可能值的信息可以参看tiff.h文件
     
     //获得单帧图像的像素点数目
     int TIFFStripSize(TIFF* tif)  
     ```
     
   - strip和Tile图像

     ```c++
     //若返回一个非零值，则文件是tile，否则是strip。
     TIFFIsTiled(tif)
     ```
     
     *Tile-oriented图像的读写*
     
     ```c++
     int TIFFReadEncodedTile(TIFF* tif, u_long tile, u_char* buf, u_long size)
     int TIFFWriteEncodedTile(TIFF* tif, u_long tile, u_char* buf, u_long size)
     ```
     
     *Strip-oriented图像的读写*
     
     ```c++
     //按行读取和解码的图像，这种方式只能用于读写非压缩格式的Strip-oriented图像。不可用于tile-oriented图像
     //参数tsample_t sample用于PlanarConfig=2时，表示通道(RGB图像的顺序是R、G、B)
     int TIFFReadScanline(TIFF* tif, tdata_t buf, uint32 row, tsample_t sample=0)
     int TIFFWriteScanline(TIFF* tif, tdata_t buf, uint32 row, tsample_t sample=0)
     
     //从打开的TIFF文件中读取和解码每一个strip，数据可以被压缩或未压缩地读写
     tsize_t TIFFReadEncodedStrip(TIFF* tif, tstrip_t strip, tdata_t buf, tsize_t size)
     tsize_t TIFFWriteEncodedStrip(TIFF* tif, tstrip_t strip, tdata_t buf, tsize_t size)
     ```
     

2. #### TinyTIFF

   一个轻量级的 C++ 库，相比于libtiff可以读写得更快，但是实现的功能有限，对TIFF文件也有一定的要求（不能压缩、大小限制、数据类型等）。
   
   - 读取图像
   
   ```c++
   //读取
   TinyTIFFReaderFile* tiffr = NULL;
   tiffr = TinyTIFFReader_open("foo.tif);
   //关闭
   TinyTIFFReader_close(tiffr);
   ```
   
   - 读取所有的帧
   
   ```c++
   //获取总帧数
   uint32_t frames = TinyTIFFReader_countFrames(tiffr);
   //从第2帧开始循环读取所有帧
   for (uint32_t i = 1; i < frames; i++) 
   {
       TinyTIFFReader_readNext(tiffr);
   }
   ```
   
   - 获取当前帧的信息
   
   ```C++
   const uint32_t width = TinyTIFFReader_getWidth(tiffr);
   const uint32_t height = TinyTIFFReader_getHeight(tiffr);
   //samples为颜色通道数
   const uint16_t samples = TinyTIFFReader_getSamplesPerPixel(tiffr);
   //颜色通道所占用的bit数
   const uint16_t bitspersample = TinyTIFFReader_getBitsPerSample(tiffr, 0);
   //循环获取不同颜色通道的二进制流
   uint8_t* image = (uint8_t*)calloc(width * height, bitspersample / 8);
   for (uint16_t sample = 0; sample < samples; sample++)
   {
       TinyTIFFReader_getSampleData(tiffr, image, sample);
   }
   free(image);
   ```
   
   - 写入相对复杂，请参考[GitHub - jkriege2/TinyTIFF: lightweight TIFF reader/writer library (C/C++)](https://github.com/jkriege2/TinyTIFF)