# rosbag_transform
This Project is intend to process a folder of rosbags that some of topics or all topics need to be modified. 
Set source path and target path in 
```sh
std::string path="/home/user/old/";
std::string new_path="/home/user/new/";
```

program will get every rosbag in source path, then process the needed topics. 
You can change these topics contents as you want.
Last,output same name rosbag in target path.


本项目旨在处理需要修改部分topics或所有topic的rosbag文件夹。

修改old文件夹目录，与新生成的topic的new目录
```sh
std::string path="/home/user/old/";
std::string new_path="/home/user/new/";
```
程序将获取源路径中的每个rosbag，然后处理所需的topics。
您可以根据需要更改这些主题内容。
最后，在目标路径中输出同名的rosbag。



