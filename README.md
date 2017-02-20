Implementation of Time Lapse (Real time) Project using V4L2 and openCV libraries on RTLinux

• Developed a Real-Time POSIX based Multi-threaded system scheduled using Earliest Deadline First policy

• The project involves image capture in raw PPM format at various rates (1 Hz, 5 Hz, and 10 Hz) with high precision and lowest latency

• Infinite frame capture is supported by compressing the images and transferring them via Ethernet using TCP protocol.

• In parallel to this, Image processing features such as Centroid detection, Background implementation, and Histogram analysis is done

• The entire process of creation of Time lapsing is automated using Shell scripting 


Hardware's Used: Altera DE1- SOC, Logitech C200

Software's Used: OpenCv, C, V4l2 Library, FFmpeg
