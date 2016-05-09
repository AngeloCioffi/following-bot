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
