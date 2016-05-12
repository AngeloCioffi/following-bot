# Research
Here are a few good webpages with some solid code:

Code:
http://www.magicandlove.com/blog/2011/08/26/people-detection-in-opencv-again/

Explanation of how it works:
http://mccormickml.com/2013/05/09/hog-person-detector-tutorial/

Now only to fix it up with ROS.....


the person detection program keeps track of a rectangle, use this rect to folow person
to face person, point towards cener of rectangle
if the rect gets smaller than a certain size, move forward
if the rect gets too big, stop moving 
start moving again as robot gets smaller
no rect -- don't move

Here's code for a PCL color histogram posted on a forum:
http://www.pcl-users.org/Kinect-Color-Histogram-td4037263.html

PCL tutorial on histograms, seems more based on geometric features though. Might be helpful:
http://pointclouds.org/documentation/tutorials/pfh_estimation.php#pfh-estimation

OpenCV histogram detection (since we are already using OpenCV, might as well use its histogram calculation):
http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_calculation/histogram_calculation.html

OpenCV histogram comparison tutorial:
http://docs.opencv.org/2.4/doc/tutorials/imgproc/histograms/histogram_comparison/histogram_comparison.html

Converting the full matrix of an image to a matrix only from a specified rectangle:
http://docs.opencv.org/2.4/doc/tutorials/core/mat_the_basic_image_container/mat_the_basic_image_container.html


BWI file that shows how to move the robot and get its current location
https://github.com/utexas-bwi/segbot/blob/master/segbot_navigation/src/move_base_interruptable_server.cpp

HOG Person Descriptor Paper:
http://lear.inrialpes.fr/people/triggs/pubs/Dalal-cvpr05.pdf

