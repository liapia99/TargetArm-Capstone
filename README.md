# TargetArm-Capstone

## Getting Started
We followed this tutorial in order to set up the Jetson Nano to be able to code using C : https://automaticaddison.com/how-to-write-a-c-c-program-for-nvidia-jetson-nano/ 

Here are the steps for easy access:

First open the NVIDIA Terminal


Creating and moving files into a folder:

```mkdir folder_name ```


```cd folder_name```



Creating a new C programming file:

``` gedit file_name.c ```

**Remember to always save the file!**

To view, compile and run the file:

``` ls```


``` gcc file_name.c -o file_name ```


``` ./file_name ```

To connect the RPLiDar to the Jetson Nano: https://collabnix.com/getting-started-with-the-low-cost-rplidar-using-jetson/#:~:text=Connect%20the%20RPlidar%20to%20the%20Jetson%20Nano%3A%20Plug%20one%20end,not%20required%20for%20the%20project.


1/24/24
Looking more into ROS ecosystems and how to communicate with the RPLiDar, we determined that we will use C++, ROS Noetic and Google's Cartographer in order to write a program that sends a warning signal to a Pixhawk drone if an object is detected. 
