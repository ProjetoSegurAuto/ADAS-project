#!/usr/bin/env python3

##########
#Informações sobre esse Node:
#Nome: Lanewarp
#Descrição: Pegar a imagem da camera zed e aplicar o lanewarp

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image #tipo de mensagem para enviar imagem
from std_msgs.msg import Float64        #tipo de mensagem que sera enviado
from std_msgs.msg import Float64MultiArray        #tipo de mensagem que sera enviado
from cv_bridge import CvBridge, CvBridgeError #converter open cv para imagem ros
import math
import vector as vc

flagImageReceived = False       #variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem

class NodeLanewarp():
    
    def __init__(self):

        #Atributos ROS
        self.msgTPC1Camera = None           #Recebera uma imagem.
        self.bridge = CvBridge()                #Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.msgVehiclePosition = Float64()         #instancia um objeto do tipo float que servira par enviar a posição da veiculo
        self.msgSteering = Float64()
        self.msgCurveRadius = Float64MultiArray()             #instancia um objeto do tipo float que servira par enviar o raio de curvatura

        self.sub = rospy.Subscriber('TPC1Camera',Image,self.callback)       #inscrição no topico de imagens

        self.pubVehiclePosition = rospy.Publisher('TPC4VehiclePosition', Float64, queue_size=1)
        self.pubSteering = rospy.Publisher('TPC4Steering', Float64, queue_size=1)
        self.pubCurveRadius = rospy.Publisher('TPC5CurveRadius', Float64MultiArray, queue_size=1)
        self.pubLKAroi = rospy.Publisher('TPC6LKAroi', Image, queue_size=1)
        self.pubLKAresult = rospy.Publisher('TPC7LKAresult', Image, queue_size=1)

    def callback(self, msg_camera):

        self.msgTPC1Camera = self.bridge.imgmsg_to_cv2(msg_camera,'bgra8')      #quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        
        global flagImageReceived    
        flagImageReceived = True        #Como a mensagem chegou, ativa a flag que permite o tratamento da imagem
    

#classe para processo de Lane Warp
class LaneWarp():

    def __init__(self):
        self.angMin = 1
        self.angMax = 50
        self.bufferLX = []
        self.bufferLY = []
        self.bufferRX = []
        self.bufferRY = []  
        self.bufferVP = []
        self.bufferA  = []
        self.logVP = []
        self.logA = []

    #Transformando imagem da camera em Bird Eye
    def warp(self, imagem):

        height = imagem.shape[0]        
        width = imagem.shape[1]
        img_size = (1280, 720)

        src = np.float32([[0, height], [width, height], [200, 530], [(width-200), 530]])
        #src = np.float32([[0, height], [width, height], [200, 630], [(width-200), 630]])
        dst = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

        M = cv2.getPerspectiveTransform(src, dst)               #[estudar]
        M_inv = cv2.getPerspectiveTransform(dst, src)           #[estudar]
        warp = cv2.warpPerspective(imagem, M, img_size)         #[estudar]
        # warp_inv = cv2.warpPerspective(img, M_inv, img_size)

        return warp, M_inv
    
    ###  Thresholded da imagem ###        
    def binary_thresholder(self,img):
        img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        img_v = img_yuv[:, :, 2]  # destaca as linhas verdes

        blurred = cv2.GaussianBlur(img_v, (7, 7), 0)

        #(T, thresh) = cv2.threshold(blurred, 123, 255, cv2.THRESH_BINARY_INV) #outdoor
        #(T, thresh) = cv2.threshold(blurred, 110, 255, cv2.THRESH_BINARY_INV) #indoor
        (T, thresh) = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY_INV) #indoor
        #(T, thresh) = cv2.threshold(blurred, 124, 255, cv2.THRESH_BINARY_INV) #outdoor 12-14hrs

        kernel = np.ones((7, 7), np.uint8)
        img_dilate = cv2.dilate(thresh, kernel, iterations=1)
        img_erode = cv2.erode(img_dilate, kernel, iterations=1)

        maskYUV = img_erode

        return maskYUV
    
    def find_lane_pixels_using_histogram(self,binary_warped):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0] // 2:, :], axis=0)

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = int(histogram.shape[0] // 2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 9
        # Set the width of the windows +/- margin
        margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Set height of windows - based on nwindows above and image shape
        window_height = int(binary_warped.shape[0] // nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window + 1) * window_height
            win_y_high = binary_warped.shape[0] - window * window_height
            win_xleft_low = leftx_current - margin
            win_xleft_high = leftx_current + margin
            win_xright_low = rightx_current - margin
            win_xright_high = rightx_current + margin

            # Identify the nonzero pixels in x and y within the window #
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                            (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_left_inds) > minpix:
                leftx_current = int(np.mean(nonzerox[good_left_inds]))
            if len(good_right_inds) > minpix:
                rightx_current = int(np.mean(nonzerox[good_right_inds]))

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]

        return leftx, lefty, rightx, righty
    
    def fit_poly(self,binary_warped, leftx, lefty, rightx, righty):
        ### Fit a second order polynomial to each with np.polyfit() ###
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        try:
            left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            print('The function failed to fit a line!')
            left_fitx = 1 * ploty ** 2 + 1 * ploty
            right_fitx = 1 * ploty ** 2 + 1 * ploty

        return left_fit, right_fit, left_fitx, right_fitx, ploty
    
    def measure_curvature_meters(self, binary_warped, left_fitx, right_fitx, ploty):
        # Define conversions in x and y from pixels space to meters
        ym_per_pix = 30 / 720  # meters per pixel in y dimension
        xm_per_pix = 3.7 / 700 # meters per pixel in x dimension

        left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(ploty * ym_per_pix, right_fitx * xm_per_pix, 2)
        # Define y-value where we want radius of curvature
        # We'll choose the maximum y-value, corresponding to the bottom of the image
        y_eval = np.max(ploty)

        # Calculation of R_curve (radius of curvature)
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        return left_curverad, right_curverad
    
    def measure_position_meters(self, binary_warped, left_fit, right_fit):
        # Define conversion in x from pixels space to meters
        xm_per_pix = 3.7 / 700  # meters per pixel in x dimension
        # Choose the y value corresponding to the bottom of the image
        #y_max = binary_warped.shape[0]
        y_max = binary_warped.shape[0] // 2
        #y_max = binary_warped.shape[0] // 4
        #y_max = binary_warped.shape[0] // 3
        #y_max = 0
       
        # Calculate left and right line positions at the bottom of the image
        left_x_pos = left_fit[0] * y_max ** 2 + left_fit[1] * y_max + left_fit[2]
        right_x_pos = right_fit[0] * y_max ** 2 + right_fit[1] * y_max + right_fit[2]
        # Calculate the x position of the center of the lane
        center_lanes_x_pos = (left_x_pos + right_x_pos) // 2
        # Calculate the deviation between the center of the lane and the center of the picture
        # The car is assumed to be placed in the center of the picture
        # If the deviation is negative, the car is on the felt hand side of the center of the lane
        veh_pos = ((binary_warped.shape[1] // 2) - center_lanes_x_pos) * xm_per_pix
        
        return veh_pos, center_lanes_x_pos

    def getBuffer(self, buffer, newValue, tolerance, method):
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
    
    def project_lane_info(self,img, binary_warped, ploty, left_fitx, right_fitx, M_inv, left_curverad, right_curverad, veh_pos, angDir):
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(color_warp, M_inv, (img.shape[1], img.shape[0]))

        # Combine the result with the original image
        out_img = cv2.addWeighted(img, 1, newwarp, 0.3, 0)

        cv2.putText(out_img, 'Curve Radius [m]: ' + str((left_curverad + right_curverad) / 2)[:7], (40, 50),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(out_img, 'Center Offset [m]: ' + str(veh_pos)[:7], (40, 100), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6,
                    (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(out_img, 'Steering: ' + str(angDir)[:7], (40, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6,
                    (255, 0, 0), 2, cv2.LINE_AA)

        return out_img
    
    def angValidate(self, angDir):
        angDir = int(angDir)
        if(angDir < self.angMin):
            angDir = self.angMin 
        elif(angDir > self.angMax):
            angDir = self.angMax

        return angDir
    
    def lineValidate(self, linex, liney, linebufferx, linebuffery):
        if(len(linex)==0 or len(liney)==0):
            print('No line')
            linex = linebufferx
            liney = linebuffery 
        else:
            linebufferx = linex
            linebuffery = liney

        return linex, liney, linebufferx, linebuffery

    def cinematicaPurePursuit(self, distance_to_center):
        offSet = 1.05  
        wheelbase = 0.75  
        target_point = [offSet, distance_to_center]

        alpha = math.atan2(target_point[1], target_point[0])
        wheel_angle = math.atan(2 * wheelbase * math.sin(alpha) / offSet)

        #convertendo angulo da roda de [-pi/2, pi/2] para o intervalo [1, 50]
        wheel_angle_mapped = (math.degrees(wheel_angle) + 90) / 180 * 49 + 1

        #Converte distance_to_center para o intervalo [-1, 1]
        normalized_distance = distance_to_center / 0.8

        #Mapeia normalized_distance para o intervalo [0, 1]
        normalized_distance = (normalized_distance + 1) / 2

        #Mapeia normalized_distance para o intervalo [1, 50]
        wheel_angle_mapped = (1 - normalized_distance) * 49 + 1
        
        wheel_angle_mapped = self.angValidate(wheel_angle_mapped)

        return wheel_angle_mapped
    
    def showLog(self, angDir, distanciaLat, left_curverad, right_curverad, center): 
        #Calcula o raio médio da curva
        R = ((left_curverad + right_curverad) / 2)/100

        #Direção que o carro vai virar
        if(left_curverad < right_curverad):
            direcao = 'Left'
        else:
            direcao = 'Right'

        print("Angulo Direcao: {}; Distância lateral: {}; Curvatura: {};'Direção: {}; Curvatura Esquerda: {}; Curvatura Direita: {}; Centro das linhas: {}".format(angDir, distanciaLat, R, direcao, left_curverad/100, right_curverad/100, center))

    def working(self, imagem, NodeRos): 
        #Imagem Bird Eye View
        img_be, M = self.warp(imagem)
        #cv2.imshow('be', img_be)

        #Imagem Binarizada
        img_bin = self.binary_thresholder(img_be)
        cv2.imshow('Bird Eye View Binarizada', img_bin)

        # Resgatando e validando pontos da linha esquerda e direita
        leftx, lefty, rightx, righty = self.find_lane_pixels_using_histogram(img_bin)
        leftx, lefty, self.bufferLX, self.bufferLY = self.lineValidate(leftx, lefty, self.bufferLX, self.bufferLY)
        rightx, righty, self.bufferRX, self.bufferRY = self.lineValidate(rightx, righty, self.bufferRX, self.bufferRY)

        left_fit, right_fit, left_fitx, right_fitx, ploty = self.fit_poly(img_bin, leftx, lefty, rightx, righty)
        left_curverad, right_curverad = self.measure_curvature_meters(img_bin, left_fitx, right_fitx, ploty)

        veh_pos, center = self.measure_position_meters(img_bin, left_fit, right_fit)
        self.bufferVP, veh_pos = self.getBuffer(self.bufferVP, veh_pos, 0.1, 'last')
        
        angDir = self.cinematicaPurePursuit(veh_pos)
        self.bufferA, angDir = self.getBuffer(self.bufferA, angDir, 5, 'mean')
         
        #self.showLog(angDir, veh_pos, left_curverad, right_curverad, center)

        out_img = self.project_lane_info(imagem[:, :, 0:3], img_bin, ploty, left_fitx, right_fitx, M, left_curverad, right_curverad, veh_pos, angDir)
        cv2.circle(out_img, (int(center), 530), 10, (0, 0, 255), 5)
        cv2.circle(out_img, (int(center), 625), 10, (0, 255, 0), 5)
        cv2.circle(out_img, (int(center), 720), 10, (255, 0, 0), 5)

        #publicar a mensagens:
        NodeRos.pubVehiclePosition.publish(veh_pos)
        NodeRos.pubSteering.publish(angDir)
        NodeRos.msgCurveRadius.data = [left_curverad,right_curverad]
        NodeRos.pubCurveRadius.publish(NodeRos.msgCurveRadius)

        NodeRos.msgLKAroi = self.bridge.cv2_to_imgmsg(img_bin,"bgra8")
        NodeRos.pubLKAroi.publish(NodeRos.msgLKAroi)

        NodeRos.msgLKAresult = self.bridge.cv2_to_imgmsg(out_img,"bgra8")
        NodeRos.pubLKAresult.publish(NodeRos.msgLKAresult)

        cv2.imshow('resultado', out_img)
        #cv2.imshow('Poly Lines', self.draw_poly_lines(img_bin, left_fitx, right_fitx, ploty))
        #cv2.imwrite('resultado_'+str(i)+'.jpg', out_img)
        cv2.waitKey(1)
 
def main():
    #Setup ROS
    rospy.init_node('Lanewarp')         #inicia o Node
    rospy.loginfo('O node Lanewarp foi iniciado!')

    nodeLanewarp = NodeLanewarp()       #instanciando o objeto
    lanewarp = LaneWarp()
    
    while(not rospy.is_shutdown()):
        if(flagImageReceived):
            try:                
                lanewarp.working(nodeLanewarp.msgTPC1Camera, nodeLanewarp)

            except Exception as ex:
                print("Exception: {}".format(ex))
                pass

            except KeyboardInterrupt:
                print("Exception: KeyboardInterrupt Lane") 

if(__name__=="__main__"):
    main()