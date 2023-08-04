import sys
import pyzed.sl as sl
from signal import signal, SIGINT
import cv2


cam = sl.Camera()

def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    cv2.destroyAllWindows() 
    sys.exit(0)

signal(SIGINT, handler)

def main():
    if not sys.argv or len(sys.argv) != 2:
        print("MEU PATRAO, COLOCA UM NOME PRO VIDEO PFVR (ex:manha.svo) (NAO ESQUECE O .svo NO FINAL)")
        exit(1)

    init = sl.InitParameters()

    init.camera_resolution = sl.RESOLUTION.HD1080 
    init.camera_fps = 30                       
    init.depth_mode = sl.DEPTH_MODE.ULTRA 

    runtime = sl.RuntimeParameters()

    runtime.sensing_mode = sl.SENSING_MODE.STANDARD  

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        #print(repr(status))
        exit(1)

    image = sl.Mat()

    path_output = sys.argv[1]
    recording_param = sl.RecordingParameters(path_output, sl.SVO_COMPRESSION_MODE.H264)
    err = cam.enable_recording(recording_param)
    if err != sl.ERROR_CODE.SUCCESS:
        #print(repr(status))
        exit(1)
    print("##################################################")
    print("\n","GRAVANDO DOIDERA, USA Ctrl-C PARA PARAR.","\n")
    print("##################################################")
    frames_recorded = 0
    
    while True:
        err = cam.grab(runtime)
        frames_recorded += 1  
        cam.retrieve_image(image, sl.VIEW.LEFT)
        img = image.get_data()
        cv2.imshow('Frame', img) 
        cv2.waitKey(5)
        
        print("Frame count: " + str(frames_recorded), end="\r")

if __name__ == "__main__":
    main()