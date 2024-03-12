import time, sys, os
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2
import math

DEFAULT_TOPIC = 'camera/image_raw/compressed'

class ConfigParams:
    MODE_UNKNOWN = 0
    MODE_VIDEO = 1
    MODE_IMAGES = 2

    def __init__(self):
        self._mode = ConfigParams.MODE_UNKNOWN
        self._fps = 24
        self.path = None
        self.compressed = True
        self.topic_name = DEFAULT_TOPIC
        self.output_file = None
        
    @property
    def fps(self):
        return int(self._fps)
        
    @fps.setter
    def fps(self, fps):
        self._fps = int(fps)

    @property
    def mode(self):
        return self._mode
        
    @mode.setter
    def mode(self, mode):
        if (mode != ConfigParams.MODE_IMAGES and mode != ConfigParams.MODE_VIDEO):
            self._mode = ConfigParams.MODE_UNKNOWN
        else:
            self._mode = mode

    def is_valid(self):
        if (self.mode != ConfigParams.MODE_VIDEO and self.mode != ConfigParams.MODE_IMAGES):
            return False
        if (self.path is None):
            return False
        else:
            if (self.mode == ConfigParams.MODE_VIDEO and not os.path.isfile(self.path)):
                return False
            if (self.mode == ConfigParams.MODE_IMAGES and not os.path.isdir(self.path)):
                return False
        if (not isinstance(self.compressed, bool)):
            return False
        if (self.output_file is None):
            return False        
        return True  
    
def create_bag_from_video(config:ConfigParams):
    videopath = config.path
    topic = config.topic_name
    bagname = config.output_file

    #Creates a bag file from a video file
    print("Creating bagfile ["+bagname+"] from video ["+videopath+"] at "+str(config.fps)+" fps.")

    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    bridge = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    
    if math.isnan(prop_fps) or prop_fps <= 0:
        print ("Warning: can't get FPS from video. Using "+str(config.fps)+".")
        prop_fps = config.fps
    
    ret = True
    frame_number = 0
    while(ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_number) / prop_fps)
        frame_number += 1
        if (config.compressed):
            image = bridge.cv2_to_compressed_imgmsg(frame)
        else:
            image = bridge.cv2_to_imgmsg(frame)
        
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(topic, image, stamp)
    cap.release()
    bag.close()

def create_bag_from_images(config: ConfigParams):
    path = config.path
    topic = config.topic_name
    bagname = config.output_file
    fps = config.fps

    #Creates a bag file from images
    print("Creating bagfile ["+bagname+"] from images in ["+path+"] at "+str(fps)+" fps.")

    bag = rosbag.Bag(bagname, 'w')

    bridge = CvBridge()
    
    prop_fps = fps
    
    frame_number = 0
    filelist = sorted(os.listdir(path))
    for filename in filelist:
        filepath = os.path.join(path,filename)
        if (os.path.isfile(filepath)):
            frame = cv2.imread(filepath)
            print("Adding file "+str(filename))
        else:
            continue
                
        stamp = rospy.rostime.Time.from_sec(float(frame_number) / prop_fps)
        frame_number += 1
        if (config.compressed):
            image = bridge.cv2_to_compressed_imgmsg(frame)
        else:
            image = bridge.cv2_to_imgmsg(frame)
        
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(topic, image, stamp)

    bag.close()

def help(message: str=""):
    if (message != ""):
        print(message)
    print( "Usage: video2bag [-v video_path | -i images_path] [fps] [compressed] [topic_name] bagfilename")
    print("Parameters:")
    print("   fps: the number of frames per second in the created bag file.")
    print("   compressed: If present, the image messages in the bag file will be compressed.")
    print("   topic_name: The name of the topic containing the image messages in the bag file.")
    exit


def process_params(argv):
    config = ConfigParams()

    for id in range(1,len(argv),2):
        param = argv[id]
        if param.startswith('-'):
            option = param[1]
            value = argv[id+1]            
            if (option =='v'):
                config.mode = ConfigParams.MODE_VIDEO
                config.path = value
            elif (option =='i'):
                config.mode = ConfigParams.MODE_IMAGES
                config.path = value
            elif (option =='r'):
                config.fps = value
            elif (option =='c'):
                try:
                    config.compressed = bool(value)
                except ValueError:
                    config.compressed = None
            elif (option =='t'):
                config.topic_name = value
            elif (option =='o'):
                config.output_file = value
            else:
                help()
        else:
            help()

    return config

if __name__ == "__main__":

    config: ConfigParams
    config = process_params(sys.argv)
    
    if (not config.is_valid()):
        help("Invalid parameters")

    if (config.mode == ConfigParams.MODE_VIDEO):
        create_bag_from_video(config)
    elif (config.mode == ConfigParams.MODE_IMAGES):
        create_bag_from_images(config)
    else:
        help("Invalid operation mode (valid options are: -v, -i)")
    
    