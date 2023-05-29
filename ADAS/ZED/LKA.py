import gc
import cv2
import numpy as np
import pyzed.sl as sl
import time
import vector as vc
import laneWarp as ln
import depth as dp
import math
import controle_lateral as cl

def cinematicaVP(distanciaLat, rpm):
    L = 0.74
    offsetY = 0.45

    dist = abs(0.41*distanciaLat)
    
    ld = math.sqrt(pow(dist, 2) + pow(L+offsetY, 2))

    if(distanciaLat > 0):
        ang =  math.atan(2*L*dist/(ld*ld))
    else:
        ang = -math.atan(2*L*dist/(ld*ld))

    angDir   =  (int) ((-ang*68.21) + 25)
    velRodaD = rpm
    velRodaE = rpm

    return angDir, velRodaD, velRodaE

def getBuffer(buffer, newValue, tolerance, method):
    retorno = False
    if(len(buffer) == 0):
        buffer.append(newValue)

    elif (newValue < np.max(buffer) + tolerance) and (newValue > np.min(buffer) - tolerance):
        buffer.append(newValue)
    
        #print("BUFFER: {}".format(buffer))
        while len(buffer) >= 5:
            buffer.pop(0)

        if(method=='mean'):
            retorno = int(np.mean(buffer))
        elif(method=='last'):
            retorno = buffer[-1]
        
    return buffer, retorno

def lineValidate(linex, liney, linebufferx, linebuffery):
    if(len(linex)==0 or len(liney)==0):
        print('No line')
        linex = linebufferx
        liney = linebuffery 

    else:
        linebufferx = linex
        linebuffery = liney

    return linex, liney, linebufferx, linebuffery


def initCar(rpm_can, angle_can):
    try:
        #ACELERA PID
        msgCanId = 0x56
        #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
        param = [1, rpm_can, 1, rpm_can]
        vc.sendMsg(msgCanId, param)
  
        #Ajustar Angulo Direcao
        msgCanId = 0x82
        param = [angle_can]
        vc.sendMsg(msgCanId, param)
    except Exception as ex:
        print("Exception: {}".format(ex))

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

#cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 0)
image = sl.Mat()
depth = sl.Mat()

# Set runtime parameters
runtime = sl.RuntimeParameters() 

rpm_can = 0
angle_can = 25
distanceBreak = 1.2
distanceStop = 0.5

tSendMsgCAN = time.time()

bufferLC = []
bufferRC = []

bufferVP = []

bufferLX = []
bufferLY = []
bufferRX = []
bufferRY = []

bufferA  = []

initCar(rpm_can, angle_can)

while True:
    try:  
        err = cam.grab(runtime)
        
        # Retrieve left image
        cam.retrieve_image(image, sl.VIEW.LEFT)
        img = image.get_data()
        #cv2.imshow('img', img)

        left_fit_hist = np.array([])
        right_fit_hist = np.array([])

        #Bird Eye
        img_be, M = ln.warp(img)
        #cv2.imshow('be', img_be)

        # imagem binarizadda
        img_bin = ln.binary_thresholder(img_be)
        #cv2.imshow('bin', img_bin)

        # exibindo pontos da linha esquerda e direita
        leftx, lefty, rightx, righty = ln.find_lane_pixels_using_histogram(img_bin)
        leftx, lefty, bufferLX, bufferLY = lineValidate(leftx, lefty, bufferLX, bufferLY)
        rightx, righty, bufferRX, bufferRY = lineValidate(rightx, righty, bufferRX, bufferRY)

        left_fit, right_fit, left_fitx, right_fitx, ploty = ln.fit_poly(img_bin, leftx, lefty, rightx, righty)

        left_curverad, right_curverad = ln.measure_curvature_meters(img_bin, left_fitx, right_fitx, ploty)
        #bufferLC, left_curverad = getBuffer(bufferLC, left_curverad , 10, 'last')
        #bufferRC, right_curverad = getBuffer(bufferRC, right_curverad , 10, 'last')

        veh_pos, center = ln.measure_position_meters(img_bin, left_fit, right_fit)
        #bufferVP, veh_pos = getBuffer(bufferVP, veh_pos , 0.2, 'last')

        out_img = ln.project_lane_info(img[:, :, 0:3], img_bin, ploty, left_fitx, right_fitx, M, left_curverad/100, right_curverad/100, veh_pos)
        cv2.imshow('Resultado', out_img)

        if 0.05 < time.time() - tSendMsgCAN:
            tSendMsgCAN = time.time()    

            R = ((left_curverad + right_curverad) / 2)/100

            print("Curvatura: {}; Curvatura Esquerda: {}; Curvatura Direita: {};".format(R, left_curverad/100, right_curverad/100))
            
            if(R<10):
                if(left_curverad < right_curverad ):
                   direcao = 0
                else:
                    direcao = 1

                angDir, velRodaD, velRodaE = cinematicaVP(veh_pos, rpm_can)

                #bufferA, angDir = getBuffer(bufferA, angDir, 5, 'mean')
                angDir = cl.smooth_steering(angDir, 5)
                #print(veh_pos)
                print("Angulo Direcao: {}; Curvatura: {}; Curvatura Esquerda: {}; Curvatura Direita: {};".format(angDir, R, left_curverad/100, right_curverad/100))

            else:
                angDir = 25

            
                if(veh_pos > 0.5):
                    angDir = angDir - 10
                elif(veh_pos < -0.5):
                    angDir = angDir + 10
                elif(veh_pos > -0.2 and veh_pos < 0.2):
                    angDir = 25

                if(angDir>50):
                    angDir = 50
                elif(angDir<0):
                    angDir = 0

                velRodaE = rpm_can
                velRodaD = rpm_can

                print("Posicao: {}; Angulo NOVO: {}".format(veh_pos, angDir))

            
            #Ajustar Angulo Direcao
            msgCanId = 0x82
            param = [angDir]
            vc.sendMsg(msgCanId, param)


            # Retrieve depth
            cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
            distance = dp.getDistance(depth)
            #print("\nDistance: {}".format(distance))
            
            # Automatic emergency braking (AEB)
            breaking = dp.AEB(distance, distanceBreak, distanceStop)

            if(not breaking):        
                #Ajustar Velocidade PID
                msgCanId = 0x56
                #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
                
                param = [1, velRodaE, 1, velRodaD]
                vc.sendMsg(msgCanId, param)
            else:
                cv2.imshow("Depth", depth.get_data())
                cv2.imshow('Resultado', img)            

        cv2.waitKey(1)
        gc.collect()
             
    except KeyboardInterrupt:
        print("Exception: KeyboardInterrupt") 
        
        #FREIA PID
        msgCanId = 0x56
        #[Direcao Esq, RPM Esq, #Direcao Dir, RPM Dir]
        param = [1, 0, 1, 0]
        vc.sendMsg(msgCanId, param)

        #Ajustar Angulo Direcao
        #msgCanId = 0x82
        #param = [0x19]
        #vc.sendMsg(msgCanId, param)

        break

    except Exception as ex:
        print("Exception: {}".format(ex))
        #pass
        break