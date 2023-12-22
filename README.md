# Squash Ball Tracker
## Outcome

[![Player Movement](https://img.youtube.com/vi/FFiENsmIYyo/0.jpg)](https://www.youtube.com/watch?v=FFiENsmIYyo&ab_channel=OwainJones)

https://www.youtube.com/watch?v=FFiENsmIYyo&ab_channel=OwainJones

## Objectives
1) From a single still camera, track the motion of a squash ball from a high level game of squash
2) The squash ball must be detected and tracked between frames
3) The number of "Possible Squash Balls" must be kept at a minimum at all times.

## Method
### Image Processing
Processing the image to optimise for ball tracking involved finding the difference between consecutive frames from the footage. The main problem stemmed from the ball being a fast moving object, so was heavily blurred throughout. Initally I used the inbuilt CUDA MOG2 background subtracter method but this led to the ball being classified as the background for the majority of time that it was moving so not being shown throughout.

To combat this I removed the background manually by sligitly blurring the image to remove noise as, converting the BGR image to gray, then using the subtract method to only leave pixles that were different between frames. This image was finally passed into the threshold method to remove any pixles below a certian white value, again removing noise.

![Pasted image 20220126120210](https://user-images.githubusercontent.com/54110810/151198878-53567ab9-832d-47e1-b164-d0adb6533197.png)

This was repeated to isolate just the players except a hight threshold value was used. As the players are more distinct from the background they show a greater change and in turn provide a greater white value. Detecting the players separately was key to optimising the ball tracking algorithm as edges found in the image optimised to detect the ball had many eronious points that existed within the bounds of the players. If these could be removed then only a fraction of the remaining points needed to be checked to see if they were the ball.

![Pasted image 20220126122514](https://user-images.githubusercontent.com/54110810/151198984-74dd9a77-0f99-4707-850e-60789cdb5179.png)

Another method I tried to isolate the players from the image was to create a mask from their HSV colour values and thresholding the image from this. This proved to work well in determining where the players t-shirts and shoes were but failed to pick out their white and black shorts. The image was hard to threshold for white and black as these values clashed with the background and didn't create a good enough mask. Where the mask was not of the whole player, it was difficult to reduce the number of points which could have been the ball. The outcome from this method can be seen in the video at the end of this document.

![Pasted image 20220126162343](https://user-images.githubusercontent.com/54110810/151203505-07856e67-02e1-4609-ae47-741dd8896534.png)

The edges that could be the ball were then passed into the object tracking algrothim that I created.

### Object Tracking
The objects were passed into the tracking algorithm. This consisted of mapping each point in one frame, to a set of points the the next frame that have the possibility of being the same object. In this case 2 objects have the possibility of being the same if their direct distance is less than 300px. This is repeated for as many frames as is needed.

![GraphFrames](https://user-images.githubusercontent.com/54110810/151198808-44705816-aa0d-48ee-97d3-fe606068e94f.jpg)

This then creates a graph of paths each object could take. 

![GraphTree](https://user-images.githubusercontent.com/54110810/151198755-34c025d4-3ffb-4f62-ba15-b24f9f0a7ffc.jpg)

To determine which path corresponds to the ball, the x and y points from each stage in the path are used to calculate the equation of a parabola that these points would be fitted to. The x point is then fed back into the equation and the calculated y value is compared to the actual y value. 

In the case of the ball, the path would fit closely to the parabola as this is the path that a projectile would follow through the air.

![Pasted image 20220126155151](https://user-images.githubusercontent.com/54110810/151198711-82b04702-a45e-41c6-830f-92fa01093e89.png)

Objects which aren't following a parabolic arc would not fit well to the equation derived from the points in their path. This would lead to a plot that would not fit well to the actual data

![Pasted image 20220126154924](https://user-images.githubusercontent.com/54110810/151198644-b93ca1a9-1a96-49a9-bf63-6548518ff061.png)

When an object that follows the trajectory is found, it is drawn to the screen with the path it has taken.

## Improvements
There are a handful of areas that I would work on immediately when returning to this project, most of which relate to the object tracking. First there is an oversight which I discovered that the direction of motion is not taken into account when mapping the points to the parabola. This can lead to a case where a set of points can be deemed to follow the arc because they all lie on the line, when in fact the order of the points along the line is wrong. When this issue is resolved I think that most of the false positive lines will be eliminated.

![Pasted image 20220126170957](https://user-images.githubusercontent.com/54110810/151258213-ec094b89-1948-41da-a060-1c2a5e84bed5.png)

In the image processing portion I think there could be an improvement when it comes to creating a mask of the players. Maybe by combining both movement detection and HSV detection together could find a middle ground by finding both a large portions of the torso of each player as well as detecting legs and shorts by movement. The better and tighter the boundary of the players would lead to removing many of the extra edges that are not the ball.

## Appendix
### Ball tracking - players detected by HSV values
[![Player HSV](https://img.youtube.com/vi/PXNhsshctMU/0.jpg)](https://www.youtube.com/watch?v=PXNhsshctMU&ab_channel=OwainJones)

### Ball tracking by selecting first nearby object
[![Naive Ball Tracking](https://img.youtube.com/vi/4hJD5qIBbTQ/0.jpg)](https://www.youtube.com/watch?v=4hJD5qIBbTQ&ab_channel=OwainJones)
