# InverseCSG环境配置
根据网络环境的不同，以下某些步骤可能需要尝试多次，最好翻个墙...  
注意，子系统中单击右键就是粘贴。

## 1. 启动“适用于Linux的Windows子系统”功能
在Windows的设置页面中搜索打开`启用或关闭Windows功能`。  
![](pic/20211028103057.png)  
启动`适用于Linux的Windows子系统`，完成后需要重启。

## 2. 安装WSL(Windows Subsystem for Linux)
子系统需要使用Ubuntu16.04，在命令行中输入`wsl --install -d Ubuntu-16.04`。  
![](pic/20211028104833.png)  
安装后在命令行中输入wsl或直接点击开始菜单中的Ubuntu快捷方式即可打开子系统。  
第一次打开需要设置账号名和密码。

## 3. 安装Miniconda
前往conda官网下载[Miniconda](https://docs.conda.io/en/latest/miniconda.html)，最好下载Python 3.7的版本。
![](pic/20211028110129.png)  
记住下载路径，我这里是E:/download。  
打开子系统，前往Miniconda的下载路径，使用`bash`命令安装。
![](pic/20211028110721.png)  
一路按`Enter`或输入`yes`确认即可。安装完成后需要重启命令行。

## 4. 加载conda环境
在子系统中输入`conda create -n inv_csg_env python=3.7`，从远程加载conda环境。
![](pic/20211028111809.png)  
仍然是一路输入`y`确认即可。

## 5. 下载安装Sketch库
为了使用以前版本而非最新版的Sketch库，需要手动下载安装。  
前往官网下载[Sketch库](https://people.csail.mit.edu/asolar/sketch-1.7.6.tar.gz)。并按照压缩包内的README文件的指引安装。
![](pic/20211028113924.png)  
注意，需要按照指引添加环境变量。

## 6. 安装其他依赖
在命令行输入`sudo apt-get install unzip`；
`sudo apt-get install openjdk-8-jdk-headless`；

## 7. 下载InverseCSG库
前往github下载[InverseCSG库](https://github.com/yijiangh/InverseCSG)即可。  
我这里放在了E:/repos/InverseCSG-master。

## 8. 运行安装脚本
先输入`conda activate inv_csg_env`启用conda环境。  
再进入InverseCSG的下载路径。
最后输入`python3 install.py -d ./build`即可安装InverseCSG。
![](pic/20211028114506.png)  
一路输入`y`确认，最后输入sketch-frontend与sketch-backend的地址即可。
![](pic/20211028163001.png)