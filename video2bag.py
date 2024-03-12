import time, sys, os
from ros import rosbag
import roslib, rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2

TOPIC = 'camera/image_raw/compressed'

def create_bag_from_video(videopath, bagname):
    #Creates a bag file from a video file
    bag = rosbag.Bag(bagname, 'w')
    cap = cv2.VideoCapture(videopath)
    cb = CvBridge()
    prop_fps = cap.get(cv2.CAP_PROP_FPS)
    if prop_fps != prop_fps or prop_fps <= 1e-2:
        print ("Warning: can't get FPS. Assuming 24.")
        prop_fps = 24
    ret = True
    frame_id = 0
    while(ret):
        ret, frame = cap.read()
        if not ret:
            break
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame_id += 1
        image = cb.cv2_to_compressed_imgmsg(frame)
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(TOPIC, image, stamp)
    cap.release()
    bag.close()

def create_bag_from_images(path, bagname, fps=24):
    #Creates a bag file from images
    print("Creating bagfile ["+bagname+"] from images in ["+path+"] at "+str(fps)+" fps.")

    bag = rosbag.Bag(bagname, 'w')

    cb = CvBridge()
    
    prop_fps = fps
    
    frame_id = 0
    filelist = sorted(os.listdir(path))
    for filename in filelist:
        filepath = os.path.join(path,filename)
        if (os.path.isfile(filepath)):
            frame = cv2.imread(filepath)
            print("Adding file "+str(filename))
        else:
            continue
                
        stamp = rospy.rostime.Time.from_sec(float(frame_id) / prop_fps)
        frame_id += 1
        image = cb.cv2_to_compressed_imgmsg(frame)
        image.header.stamp = stamp
        image.header.frame_id = "camera"
        bag.write(TOPIC, image, stamp)

    bag.close()

def help():
    print( "Usage: video2bag [-v video_path | -i images_path] bagfilename [fps]")

if __name__ == "__main__":
    if len( sys.argv ) >= 4:
        if (sys.argv[1]=='-v'):
            create_bag_from_video(sys.argv[2], sys.argv[3])
        elif (sys.argv[1]=='-i'):
            if len( sys.argv ) > 4:
                try:
                    fps = int(sys.argv[4])
                    create_bag_from_images(sys.argv[2], sys.argv[3], fps=fps)
                except ValueError:
                    print('Invalid fps value ['+str(sys.argv[4])+']')
                    help()
            else:
                create_bag_from_images(sys.argv[2], sys.argv[3], fps=24)
        else:
            print ("Invalid option ["+sys.argv[1]+"].")
            help()
    else:
        help()