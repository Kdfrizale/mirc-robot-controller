# mirc-robot-controller

Framework for creating controllers for various robotic designs.

Listens to ROS messages published by mirc-leap-ros project that describe the position and orientation of the user's hand.
Depending on the type of robotic design, the project either executes an RRT path finding algorithm for the robotic arm or a Twist message 
that a wheel driven robot can use for direction and speed.

