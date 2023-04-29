import cv2 # Import OpenCV
import os
import sys
   
def binary_image( path ):
    
    if not os.path.exists( path ):
        sys.exit( f"Cannot find image {path}")
    
    # read the image file
    img = cv2.imread( path )
    
    ret, bw_img = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)
    
    # Display and save image 
    cv2.imshow("Binary", bw_img)
    cv2.imwrite("map_binary.png", bw_img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#==============================================================================
if __name__ == "__main__":
    
    path = "/home/degobuntu/Wanda/ROS/src/navigation_stack/maps/map.png"
    
    binary_image( path )