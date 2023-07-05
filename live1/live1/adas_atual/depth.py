import math
import numpy as np
import vector as vc
import cv2
import pyzed.sl as sl

#Automatic emergency braking (AEB)
def AEB(distance, distanceBreak, distanceStop): 
    retorno = False
    msgCanId = 0x00
    try:
        if not np.isnan(distance):
            if np.isfinite(distance):
                if distance < distanceStop:
                    print("STOP!!! - distance: {}".format(distance))               
                    #TRAVA RODA - FREIA PWM
                    msgCanId = 0x5C
                    retorno = True

                elif distance < distanceBreak:
                    print("Break!!! - distance: {}".format(distance))                    
                    #FREIA PID
                    msgCanId = 0x56
                    retorno = True
            else:
                print("STOP!!! infinite - distance: {}".format(distance))                   
                #TRAVA RODA - FREIA PWM
                msgCanId = 0x5C
                retorno = True
                
            #[Direcao Esq, PWM Esq, #Direcao Dir, PWM Dir]
            param = [1, 0, 1, 0]
            vc.sendMsg(msgCanId, param)

            return retorno

    except Exception as ex:
        print("Exception: {}".format(ex)) 
        return retorno

def getDistance(depth):
    # Get and print distance value in mm at the center of the image
    #x_distances = [0.25, 0.5, 0.75]
    print('getdistance')
    distance = math.inf
    #for i in x_distances:
    #   x = round(depth.get_width() * i)
    #   y = round(depth.get_height() * 0.8)
    for i in range(35, 70, 5):#range(15, 90, 15):
        for j in range(55, 85, 5):
            x = round(depth.get_width() * i/100)
            y = round(depth.get_height() * j/100)        
            err, depth_value = depth.get_value(x, y)
            print('depth.get_value(x, y)')
            print(depth.get_value(x, y))
            print(depth)
            if depth_value < distance:
                distance = depth_value

    if math.isfinite(distance):
        #calibration by regression
        distance = distance * 1.1051 - 0.4602 - 0.19
        
    return distance

def main():
    cam = sl.Camera()
    init = sl.InitParameters()

    # Resolution: HD2K, HD1080, HD720, VGA
    init.camera_resolution = sl.RESOLUTION.HD720  
    # FPS: 15, 30, 60, 100 
    init.camera_fps = 30
    # ULTRA: offers the highest depth range and better preserves Z-accuracy along the sensing range.
    # QUALITY: has a strong filtering stage giving smooth surfaces.
    # PERFORMANCE: designed to be smooth, can miss some details.
    # Generally speaking, we recommend using the ULTRA mode for both desktop and embedded applications. 
    # If your application requires a lot of resources, switch to PERFORMANCE mode.
    init.depth_mode = sl.DEPTH_MODE.ULTRA 
    # Use meter units (for depth measurements)
    init.coordinate_units = sl.UNIT.METER  
    # Set the minimum depth perception distance to 15cm
    #init.depth_minimum_distance = 0.15 
    init.coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP

    if not cam.is_opened():
        print("Opening ZED Camera...")

    status = cam.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        print(repr(status))
        exit()

    depth = sl.Mat()
    image = sl.Mat()

    # Set runtime parameters
    runtime = sl.RuntimeParameters()
    # Use STANDARD sensing mode
    #runtime.sensing_mode = sl.SENSING_MODE.STANDARD  
    # Setting the depth confidence parameters
    #runtime.confidence_threshold = 100
    #runtime.textureness_confidence_threshold = 100

    while True:
        try:  
            err = cam.grab(runtime)

            # Retrieve left image
            cam.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            #cv2.imshow("ZED", frame) 
            for i in range(35, 70, 5):
                for j in range(55, 85, 5):
                    x = round(depth.get_width() * i/100)
                    y = round(depth.get_height() * j/100)
                    line_image = np.zeros_like(frame)
                    cv2.line(line_image,(x, y),(x+10, y),(255,0,0), 10)
                    frame = cv2.addWeighted(frame, 0.97, line_image, 5, 0)
                
            cv2.imshow("ProfundidadeRange", frame)
            

            cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
            #print('Depth_slmeasure.depth: ')
            #print(depth)
            cv2.imshow('Profudindade', depth.get_data())
            #print('Depth:')
            #print(type(depth.get_data()))
            #print(depth.get_data())
            cv2.waitKey(1)
            distance = getDistance(depth)
            print(depth.get_width(),depth.get_height())


            if not np.isnan(distance):# and not np.isinf(distance):
                print("{}\n".format(distance))
 
            
        
        except KeyboardInterrupt:
            # Retrieve depth left image
            cam.retrieve_image(depth, sl.VIEW.DEPTH)
            depth_show = depth.get_data()
            cv2.imshow('depth', depth_show) 
            break

if __name__ == "__main__":
    main()