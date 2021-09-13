QT内存管理 
=================
###### 2021.09.13

## 要点
1. QObject 的构造函数通常需要 Parent 指针，默认为 NULL  
    a. Parent 指针不为空：  
    把当前的对象实例加入父指针指定的QObject及其派生类，当一个QObject被delete或者调用了它的析构函数时，所有加入的children也会全部被析构  
    b. Parent 指针为 NULL：  
    使用 delete 或者 deleteLater  
2. QT 对象析构的核心原则是保证子对象先析构，把子对象从父对象列表中删除，二者取消关联，避免子对象 double free  
3. QWidget 的关闭的实质为 隐藏，在内存中缓存，通过设置 Qt::WA_DeleteOnClose 来释放内存  
4. QT 半自动内存机制的缺陷在于 Parent 不区分它的 Child 是分配在 Stack 上还是 Heap 上  
```C++
// Incorrect Example
QApplication Application(argc, argv);
QLabel Label("Hello QT!");
QWidget Widget;
Label.setParent(&Widget);
Widget.show();
return Application.exec();
```

<font color=FF0000> 注意：本地对象的析构函数的调用顺序与他们的构造顺序相反 </font>  
修改方法：  
1. 把 QLabel 和 QWidget 的构造语句交换一下  
2. 把 QLabel 对象创建在 Heap 上  
```C++
// Correct Example
QLabel *pLabel = new QLabel("Hello QT!");
QWidget Widget;
pLabel->setParent(&Widget);
```