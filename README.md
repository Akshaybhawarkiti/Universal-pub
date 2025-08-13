# 1. Publish floting value    

from unipub import unipub  
unipub(    
    mode="number",    
    data_type="float",   
    nodename="universal_publisher",   
    topic_name="my_topic",   
    queue_size=10,    
    delay_period=1.0,    
    value=32.5    
)


# 2. Publish an integer  

unipub(    
    mode="number",   
    data_type="int",   
    nodename="universal_publisher",   
    topic_name="int_topic",   
    delay_period=1.0,   
    value=42  
) 

# 3. Publish a string   

unipub(   
    mode="string",  
    nodename="universal_publisher",   
    topic_name="string_topic",   
    delay_period=1.0,   
    value="Hello ROS2!"   
)


# 4. Publish a boolean   

unipub(   
    mode="bool",   
    nodename="universal_publisher",   
    topic_name="bool_topic",   
    delay_period=1.0,   
    value=True   
) 

# 5. Publish a list of floats  

unipub(   
    mode="multi_float",   
    nodename="universal_publisher",   
    topic_name="multi_topic",   
    delay_period=1.0,   
    value=[1.23, 4.56, 7.89]   
)



# 6. Publish an OpenCV image from a variable    

import cv2   
from unipub import unipub   
frame = cv2.imread("test.jpg")   
unipub(  
    mode="image",  
    data_type="opencv",   
    nodename="universal_publisher",   
    topic_name="image_topic",   
    delay_period=1.0,   
    value=frame   
)   



# 7. Publish webcam video    

unipub(   
    mode="video",   
    data_type="opencv",   
    nodename="universal_publisher",   
    topic_name="video_topic",   
    delay_period=0.033   
)   
