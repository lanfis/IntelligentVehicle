# IntelligentVehicle
Using "git clone --recursive ..." instead of "git clone ..." to clone submodules.

If submodules directories still empty, type : 

"git submodule init"

"git submodule update"

Inside program is divided into 3 types : "package", "node" and "nodelet".

"package" is put in the "ros_iv/src", and every directories is "node". Only algorithm or passive program is used nodelet which is writen as plugins in the directories where the "node" is.

"node" and "nodelet" is wraped by the basic header or cpp fils as a "tobe-compiled" module which is put in the "ROS/ros_package".

"node" can be header or cpp, but "nodelet" should be saved as cpp to be compile as a executable file.

The main algorithm or the ros program is put in the "ROS/ros_object" as a completely a object which is included by the "tobe-compiled" module. Only you need
is to focus construct your program in this directories.

The basic algorithm or object is put in the "ROS/object". Suggest algorithms should be put in this directories because you can write new programs and modify the best algorithm for whole your other programs.
