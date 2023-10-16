#!/usr/bin/env python3

##########
# Informações sobre esse Node:
# Nome: Lanewarp
# Descrição: Pegar a imagem da camera zed e aplicar o lanewarp

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image  # tipo de mensagem para enviar imagem
from std_msgs.msg import Float64  # tipo de mensagem que sera enviado
from std_msgs.msg import Float64MultiArray  # tipo de mensagem que sera enviado
# converter open cv para imagem ros
from cv_bridge import CvBridge, CvBridgeError
import math
import vector as vc

# variável global para garantir que o tratamento da imagem so ira começar se tiver recebido imagem
flagImageReceived = False


class NodeLanewarp():

    def __init__(self):

        # Atributos ROS
        self.msgTPC1Camera = None  # Recebera uma imagem.
        # Instancia um objeto do tipo bridge para fazer a conversão de opencv para msg_Image
        self.bridge = CvBridge()
        # instancia um objeto do tipo float que servira par enviar a posição da veiculo
        self.msgVehiclePosition = Float64()
        self.msgSteering = Float64()
        # instancia um objeto do tipo float que servira par enviar o raio de curvatura
        self.msgCurveRadius = Float64MultiArray()

        # inscrição no topico de imagens
        self.sub = rospy.Subscriber('TPC1Camera', Image, self.callback)

        self.pubVehiclePosition = rospy.Publisher(
            'TPC4VehiclePosition', Float64, queue_size=1)
        self.pubSteering = rospy.Publisher(
            'TPC4Steering', Float64, queue_size=1)
        self.pubCurveRadius = rospy.Publisher(
            'TPC5CurveRadius', Float64MultiArray, queue_size=1)
        self.pubLKAroi = rospy.Publisher('TPC6LKAroi', Image, queue_size=1)
        self.pubLKAresult = rospy.Publisher(
            'TPC7LKAresult', Image, queue_size=1)
        self.pubLKAnave = rospy.Publisher('TPC8LKAnave', Image, queue_size=1)

    def callback(self, msg_camera):

        # quando a imagem for recebida ela será convertida de msgImage para objeto do opencv
        self.msgTPC1Camera = self.bridge.imgmsg_to_cv2(msg_camera, 'bgra8')

        global flagImageReceived
        # Como a mensagem chegou, ativa a flag que permite o tratamento da imagem
        flagImageReceived = True
        # cv2.imshow('zed', self.msgTPC1Camera)
        # cv2.waitKey(1)


class LaneWarp():

    def __init__(self):
        self.angMin = 1
        self.angMax = 50
        self.bufferLX = []
        self.bufferLY = []
        self.bufferNA = []
        self.bufferRX = []
        self.bufferRY = []
        self.bufferVP = []
        self.bufferA = []
        self.logVP = []
        self.logA = []
        self.biasA = 0
        self.biasAR = False
        self.biasAL = False
        self.center_lane_positions = []
        self.gja_center_lane_positions = None
        self.bufferRight = []
        self.bufferLeft = []

    # Transformando imagem da camera em Bird Eye

    def warp(self, imagem):

        height = imagem.shape[0]
        width = imagem.shape[1]
        img_size = (width, height)

        # src = np.float32([[0, height], [width, height], [200, 530], [(width-200), 530]])
        # dst = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

        src = np.float32([[0, height], [width, height], [
                         210, 530], [(width-235), 530]])
        dst = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

        img = imagem
        cv2.circle(img, (0, height), 10, (0, 0, 255), 5)
        cv2.circle(img, (width, height), 10, (0, 0, 255), 5)
        cv2.circle(img, (235, 530), 10, (0, 0, 255), 5)
        cv2.circle(img, ((width-210), 530), 10, (0, 0, 255), 5)

        M = cv2.getPerspectiveTransform(src, dst)  # [estudar]
        M_inv = cv2.getPerspectiveTransform(dst, src)  # [estudar]
        warp = cv2.warpPerspective(imagem, M, img_size)  # [estudar]

        return warp, M_inv

    ###  Thresholded da imagem ###
    def binary_thresholder(self, img):
        img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
        img_v = img_yuv[:, :, 2]  # destaca as linhas verdes

        blurred = cv2.GaussianBlur(img_v, (7, 7), 0)

        # (T, thresh) = cv2.threshold(blurred, 123, 255, cv2.THRESH_BINARY_INV) #outdoor
        (T, thresh) = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY_INV) #indoor 12-14hrs
        # (T, thresh) = cv2.threshold(blurred, 122, 255, cv2.THRESH_BINARY_INV)   #indoor
        # (T, thresh) = cv2.threshold(blurred, 124, 255, cv2.THRESH_BINARY_INV) #outdoor 12-14hrs

        kernel = np.ones((7, 7), np.uint8)
        img_dilate = cv2.dilate(thresh, kernel, iterations=1)
        img_erode = cv2.erode(img_dilate, kernel, iterations=1)

        maskYUV = img_erode

        return maskYUV

    def find_lane_pixels_using_histogram(self, binary_warped):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(
            binary_warped[binary_warped.shape[0] // 2:, :], axis=0)

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

    def lineValidate(self, left, right, leftBuffer, rightBuffer):
        minLen = 10000
        minDist = 450
        flagBL = False
        flagBR = False
        
        if(len(left[0]) <= 0):
            left = leftBuffer
            flagBL = True

        if(len(right[0]) <= 0):
            right = rightBuffer
            flagBR = True
        
        if(not flagBR or not flagBL):
            '''
            print('flagBL {}'.format(flagBL))
            print('flagBR {}'.format(flagBR))
            maxXLi = np.argmax(left[0])
            minXRi = np.argmin(right[0])
        
            dist = ((right[0][minXRi]-left[0][maxXLi])**2 + (right[1][minXRi]-left[1][maxXLi])**2)**0.5
            print(dist)
            print('len(left[0]) {}'.format(len(left[0])))
            print('len(right[0]) {}'.format(len(right[0])))
            '''
            if (len(left[0]) <= minLen):#or dist < minDist or flagBL):
                left = leftBuffer
            else:
                leftBuffer = left

            if (len(right[0]) <= minLen):#  or dist < minDist or flagBR):
                right = rightBuffer
            else:
                rightBuffer = right
         
        return left, right, leftBuffer, rightBuffer

    def fit_poly(self, binary_warped, leftx, lefty, rightx, righty):
        #def fit_poly(self, binary_warped, lane1, lane2):
        ### Fit a second order polynomial to each with np.polyfit() ###
        left, right, self.bufferLeft, self.bufferRight = self.lineValidate([leftx, lefty], [rightx, righty], self.bufferLeft, self.bufferRight)

        # if self.biasAR:
        #    self.biasA = 10
        # elif self.biasAL:
        #    self.biasA = -10
        # else:
        #    self.biasA = 0

        # print("rightx: {} | righty: {}".format(len(rightx), len(righty)))
        # print("leftx: {} | lefty: {}".format(len(leftx), len(lefty)))

        laneValid = np.zeros(binary_warped.shape[:2], np.uint8)
        laneValid[left[1], left[0]] = 255
        laneValid[right[1], right[0]] = 255

        left_fit = np.polyfit(left[1], left[0], 2)
        right_fit = np.polyfit(right[1], right[0], 2)

        # Generate x and y values for plotting
        ploty = np.linspace(
            0, binary_warped.shape[0] - 1, binary_warped.shape[0])
        try:
            left_fitx = left_fit[0] * ploty ** 2 + \
                left_fit[1] * ploty + left_fit[2]
            right_fitx = right_fit[0] * ploty ** 2 + \
                right_fit[1] * ploty + right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            #print('The function failed to fit a line!')
            left_fitx = 1 * ploty ** 2 + 1 * ploty
            right_fitx = 1 * ploty ** 2 + 1 * ploty

        return left_fit, right_fit, left_fitx, right_fitx, ploty, laneValid

    def navegation_area_validate(self, navegation):
        contours, _ = cv2.findContours(
            navegation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            peri = cv2.arcLength(cnt, True)
            vertices = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            sides = len(vertices)
            if (sides == 4):
                self.bufferNA = navegation

            return self.bufferNA

    def measure_curvature_meters(self, binary_warped, left_fitx, right_fitx, ploty):
        # Define conversions in x and y from pixels space to meters
        # ym_per_pix = 30 / 720  # meters per pixel in y dimension
        ym_per_pix = 1.35 / 720  # meters per pixel in y dimension
        # xm_per_pix = 3.7 / 700 # meters per pixel in x dimension
        xm_per_pix = 2.50 / 1280  # meters per pixel in x dimension
        left_fit_cr = np.polyfit(ploty * ym_per_pix, left_fitx * xm_per_pix, 2)
        right_fit_cr = np.polyfit(
            ploty * ym_per_pix, right_fitx * xm_per_pix, 2)
        # Define y-value where we want radius of curvature
        # We'll choose the maximum y-value, corresponding to the bottom of the image
        y_eval = np.max(ploty)

        # Calculation of R_curve (radius of curvature)
        left_curverad = ((1 + (2 * left_fit_cr[0] * y_eval * ym_per_pix + left_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * left_fit_cr[0])
        right_curverad = ((1 + (2 * right_fit_cr[0] * y_eval * ym_per_pix + right_fit_cr[1]) ** 2) ** 1.5) / np.absolute(
            2 * right_fit_cr[0])

        return left_curverad, right_curverad
    ''' 
    def measure_position_meters(self, binary_warped, left_fit, right_fit):
        # Define conversion in x from pixels space to meters
        xm_per_pix = 2.50 / 1280   # meters per pixel in x dimension
        # Define a list of y positions at which to calculate the x positions
        y_positions = np.linspace(0, binary_warped.shape[0], num=10)

        center_lane_positions = []
        for y in y_positions:
            # Calculate left and right line positions
            left_x_pos = left_fit[0] * y ** 2 + left_fit[1] * y + left_fit[2]
            right_x_pos = right_fit[0] * y ** 2 + right_fit[1] * y + right_fit[2]
            # Calculate the x position of the center of the lane
            center_lane_x_pos = (left_x_pos + right_x_pos) // 2
            center_lane_positions.append((center_lane_x_pos, y))
        
        y_max = int(binary_warped.shape[0]*0.85)
        # Calculate left and right line positions at the bottom of the image
        left_x_pos = left_fit[0] * y_max ** 2 + left_fit[1] * y_max + left_fit[2]
        right_x_pos = right_fit[0] * y_max ** 2 + right_fit[1] * y_max + right_fit[2]
        # Calculate the x position of the center of the lane
        center_lanes_x_pos = (left_x_pos + right_x_pos) // 2
        
        #veh_pos = ((binary_warped.shape[1] // 2) - center_lane_positions[2][0]) * xm_per_pix + 0.06
        veh_pos = ((binary_warped.shape[1] // 2) - center_lanes_x_pos) * xm_per_pix + 0.06

        return veh_pos, center_lane_positions, y_positions
    '''
    def evaluate_trajectory_quality(self, center_lane_positions, left_lane_positions=None, right_lane_positions=None, previous_center_lane_positions=None):
    
        MIN_DISTANCE = 100
        #print(center_lane_positions)
        
        #for i in range(len(center_lane_positions)):
            #center_x, center_y = center_lane_positions[i]
        center_x = center_lane_positions

            #print(len(center_lane_positions))
            #distancia da trajetoria central para as faixas esquerda e direita
        if left_lane_positions and right_lane_positions:
            #print(left_lane_positions)
            left_x = left_lane_positions
            right_x = right_lane_positions
            left_distance = abs(center_x - left_x)
            right_distance = abs(center_x - right_x)
            #print('distancia x da linha da direita: {}'.format(right_distance))
            #print('distancia x da linha da esquerda: {}'.format(left_distance))
            
        if left_distance < MIN_DISTANCE or right_distance < MIN_DISTANCE:
            #print('False')
            return False
        #print('True')
        return True

    def measure_position_meters(self, binary_warped, left_fit, right_fit, gja_center_lane_positions=None):
        # Define conversion in x from pixels space to meters
        xm_per_pix = 2.50 / 1280  # meters per pixel in x dimension
        y_positions = np.linspace(0, binary_warped.shape[0], num=10)

        new_center_lane_positions = []
        for y in y_positions:

            left_x_pos = left_fit[0] * y ** 2 + \
                left_fit[1] * y + left_fit[2]
            right_x_pos = right_fit[0] * y ** 2 + \
                right_fit[1] * y + right_fit[2]
            
            #print('posição x da linha da direita: {}'.format(right_x_pos))
            #print('posição x da linha da esqeurda: {}'.format(left_x_pos))
            center_lane_x_pos = (left_x_pos + right_x_pos) // 2
            if self.evaluate_trajectory_quality(center_lane_x_pos, left_lane_positions=left_x_pos, right_lane_positions=right_x_pos) == True: 
                self.center_lane_positions.append((center_lane_x_pos, y))
            
            if len(self.center_lane_positions) >= 10:
                self.center_lane_positions.pop(0)

        # print(len(self.center_lane_positions))

        # gja = grojoba => media movel exponencial
        if gja_center_lane_positions is not None:
            new_center_lane_positions = [
                0.9 * np.array(self.center_lane_positions[8][0]) + 0.1 * np.array(gja_center_lane_positions[0][0])]
            # for i in range(len(center_lane_positions))]
            # new_center_lane_positions = np.array(self.center_lane_positions[8][0])#mais proxio do carrinho
            veh_pos = ((binary_warped.shape[1] // 2) - np.array(
                new_center_lane_positions[0])) * xm_per_pix + 0.06
            #print(new_center_lane_positions)
        else:
            veh_pos = ((binary_warped.shape[1] // 2) - np.array(
                self.center_lane_positions[0][0])) * xm_per_pix + 0.06

        return veh_pos, self.center_lane_positions, y_positions

    def getBuffer(self, buffer, newValue, tolerance, method):
        retorno = False
        if (len(buffer) == 0):
            buffer.append(newValue)

        elif (newValue < np.max(buffer) + tolerance) and (newValue > np.min(buffer) - tolerance):
            buffer.append(newValue)

            # print("BUFFER: {}".format(buffer))
            while len(buffer) >= 5:
                buffer.pop(0)

        if (method == 'mean'):
            retorno = int(np.mean(buffer))
        elif (method == 'last'):
            retorno = buffer[-1]

        return buffer, retorno

    def project_lane_info(self, img, binary_warped, ploty, left_fitx, right_fitx, M_inv, left_curverad, right_curverad, veh_pos, angDir):
        # Create an image to draw the lines on
        warp_zero = np.zeros_like(binary_warped).astype(np.uint8)
        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))

        # Recast the x and y points into usable format for cv2.fillPoly()
        pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
        pts_right = np.array(
            [np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
        pts = np.hstack((pts_left, pts_right))

        # Draw the lane onto the warped blank image
        cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

        # Warp the blank back to original image space using inverse perspective matrix (Minv)
        newwarp = cv2.warpPerspective(
            color_warp, M_inv, (img.shape[1], img.shape[0]))
        # warp image binarized
        newwarpbin = cv2.cvtColor(newwarp, cv2.COLOR_BGR2GRAY)
        newwarpbin = cv2.threshold(newwarpbin, 1, 255, cv2.THRESH_BINARY)[1]

        # Combine the result with the original image
        out_img = cv2.addWeighted(img, 1, newwarp, 0.3, 0)

        cv2.putText(out_img, 'Curve Radius [m]: ' + str((left_curverad + right_curverad) / 2)[:7], (40, 50),
                    cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(out_img, 'Center Offset [m]: ' + str(veh_pos)[:7], (40, 100), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6,
                    (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(out_img, 'Steering: ' + str(angDir)[:7], (40, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6,
                    (255, 0, 0), 2, cv2.LINE_AA)

        return out_img, newwarpbin

    def angValidate(self, angDir):
        angDir = int(angDir)  # + self.biasA
        if (angDir < self.angMin):
            angDir = self.angMin
        elif (angDir > self.angMax):
            angDir = self.angMax

        return angDir

    def transform_point(self, point, M_inv):
        point = np.array([[point[0], point[1]]], dtype='float32')
        point = np.array([point])
        transformed_point = cv2.perspectiveTransform(point, M_inv)

        return int(transformed_point[0][0][0]), int(transformed_point[0][0][1])

    def cinematicaPurePursuit(self, distance_to_center):
        offSet = 1.1 #0.9  #1.05
        wheelbase = 0.75
        target_point = [offSet, distance_to_center]

        alpha = math.atan2(target_point[1], target_point[0])
        wheel_angle = math.atan(2 * wheelbase * math.sin(alpha) / offSet)

        # convertendo angulo da roda de [-pi/2, pi/2] para o intervalo [1, 50]
        wheel_angle_mapped = (math.degrees(wheel_angle) + 90) / 180 * 49 + 1

        # Converte distance_to_center para o intervalo [-1, 1]
        normalized_distance = distance_to_center / 0.4

        # Mapeia normalized_distance para o intervalo [0, 1]
        normalized_distance = (normalized_distance + 1) / 2

        # Mapeia normalized_distance para o intervalo [1, 50]
        wheel_angle_mapped = (1 - normalized_distance) * 49 + 1

        wheel_angle_mapped = self.angValidate(wheel_angle_mapped)

        return wheel_angle_mapped

    def showLog(self, angDir, distanciaLat, left_curverad, right_curverad, center):
        # Calcula o raio médio da curva
        R = ((left_curverad + right_curverad) / 2)/100

        # Direção que o carro vai virar
        if (left_curverad < right_curverad):
            direcao = 'Left'
        else:
            direcao = 'Right'

        print("Angulo Direcao: {}; Distância lateral: {}; Curvatura: {};'Direção: {}; Curvatura Esquerda: {}; Curvatura Direita: {}; Centro das linhas: {}".format(
            angDir, distanciaLat, R, direcao, left_curverad/100, right_curverad/100, center))

    def working(self, imagem, NodeRos):
        # Imagem Bird Eye View
        img_be, M = self.warp(imagem)
        #cv2.imshow('be', img_be)

        # Imagem Binarizada
        img_bin = self.binary_thresholder(img_be)
        # cv2.imshow('Bird Eye View Binarizada', img_bin)

        
        # Resgatando e validando pontos da linha esquerda e direita
        leftx, lefty, rightx, righty = self.find_lane_pixels_using_histogram(img_bin)

        left_fit, right_fit, left_fitx, right_fitx, ploty, laneValid = self.fit_poly(img_bin, leftx, lefty, rightx, righty)
        left_curverad, right_curverad = self.measure_curvature_meters(img_bin, left_fitx, right_fitx, ploty)

        '''
        (numLabels, labels, stats, centroids) = cv2.connectedComponentsWithStats(img_bin, 8, cv2.CV_32S)
        mask = np.zeros(img_bin.shape, dtype="uint8")
        sorted_areas = np.sort(stats[1:numLabels, cv2.CC_STAT_AREA])
        fitxy = []
        componentXY = []
        for i in range(1, numLabels):
            # extract the connected component statistics for the current label
            #x = stats[i, cv2.CC_STAT_LEFT]
            #y = stats[i, cv2.CC_STAT_TOP]
            #w = stats[i, cv2.CC_STAT_WIDTH]
            #h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            
            # ensure the width, height, and area are all neither too small or too big
            keepArea = area > 5000 and (area == sorted_areas[-1] or area == sorted_areas[-2])
            if keepArea:
                #print("i: {} | x: {} | y: {} | w: {} | h: {} | area: {} | centroid: {}".format(i, x, y, w, h, area, centroids[i]))
                #componentMask = (labels == i).astype("uint8") * 255
                #mask = cv2.bitwise_or(mask, componentMask)  
                componentXY = np.argwhere(labels == i)
                fitxy.append(componentXY)
                   

        #cv2.imshow("Connected", mask)
        #cv2.waitKey(1)

        print(fitxy[0][0])
        left_fit, right_fit, left_fitx, right_fitx, ploty, laneValid = self.fit_poly(img_bin, fitxy[0], fitxy[1,0], fitxy[0,1], fitxy[1,1])
        left_curverad, right_curverad = self.measure_curvature_meters(img_bin, left_fitx, right_fitx, ploty)
        '''

        #veh_pos, center, center_y = self.measure_position_meters(img_bin, left_fit, right_fit)
        veh_pos, center, center_y = self.measure_position_meters(img_bin, left_fit, right_fit, self.gja_center_lane_positions)

        self.gja_center_lane_positions = center
        # self.bufferVP, veh_pos = self.getBuffer(self.bufferVP, veh_pos, 0.1, 'last')

        angDir = self.cinematicaPurePursuit(veh_pos)
        # self.current_state, self.current_covariance = self.kf.filter_update(self.current_state, self.current_covariance, angDir)
        #angDirAux=angDir
        #self.bufferA, angDir = self.getBuffer(self.bufferA, angDir, 8, 'mean')
        # angDir = self.current_state[0]
        #print('angDir: {} | angDirBuffer: {}'.format(angDirAux, angDir))

        # angDir = self.angValidate(angDir)
        out_img, navegation = self.project_lane_info(
            imagem[:, :, 0:3], laneValid, ploty, left_fitx, right_fitx, M, left_curverad, right_curverad, veh_pos, angDir)
        #cv2.line(out_img, (out_img.shape[1] // 2, 0), (out_img.shape[1] // 2, out_img.shape[0]), (0, 0, 255), 5) 

        # navegation = self.navegation_area_validate(navegation)

        for i in range(len(center)):
            cv2.circle(laneValid, (int(center[i][0]), int(
                center[i][1])), 10, (255, 255, 255), 5)

            # cv2.imshow('Bird Eye View Binarizada', img_bin)

        for point in center:
            # Apply inverse perspective transform to the point
            transformed_point = self.transform_point(point, M)
            cv2.circle(out_img, transformed_point, 10, (255, 0, 0), 5)
        '''
        for left in range(len(leftx)):
            # Apply inverse perspective transform to the point
            transformed_point = self.transform_point((leftx[left],lefty[left]), M)
            cv2.circle(out_img, transformed_point, 1, (219, 203, 255), 1)
        
        for right in range(len(rightx)):
            # Apply inverse perspective transform to the point
            transformed_point = self.transform_point((rightx[right],righty[right]), M)
            cv2.circle(out_img, transformed_point, 1, (0, 165, 255), 1)
        '''
        cv2.line(laneValid, (laneValid.shape[1] // 2, 0), (laneValid.shape[1] // 2, laneValid.shape[0]), (255, 255, 255), 5) 

        # publicar a mensagens:
        NodeRos.pubVehiclePosition.publish(veh_pos)
        NodeRos.pubSteering.publish(angDir)
        NodeRos.msgCurveRadius.data = [left_curverad, right_curverad]
        NodeRos.pubCurveRadius.publish(NodeRos.msgCurveRadius)

        NodeRos.msgLKAnave = NodeRos.bridge.cv2_to_imgmsg(navegation, "8UC1")
        NodeRos.pubLKAnave.publish(NodeRos.msgLKAnave)

        NodeRos.msgLKAroi = NodeRos.bridge.cv2_to_imgmsg(laneValid, "8UC1")
        NodeRos.pubLKAroi.publish(NodeRos.msgLKAroi)

        NodeRos.msgLKAresult = NodeRos.bridge.cv2_to_imgmsg(out_img, "8UC3")
        NodeRos.pubLKAresult.publish(NodeRos.msgLKAresult)

        #cv2.imshow('resultado', out_img)
        #cv2.imshow('roi', laneValid)
        # cv2.imshow('Poly Lines', self.draw_poly_lines(img_bin, left_fitx, right_fitx, ploty))
        # cv2.imwrite('resultado_'+str(i)+'.jpg', out_img)
        #cv2.waitKey(1)


def main():
    # Setup ROS
    rospy.init_node('Lanewarp')  # inicia o Node
    rospy.loginfo('O node Lanewarp foi iniciado!')

    nodeLanewarp = NodeLanewarp()  # instanciando o objeto
    lanewarp = LaneWarp()

    while (not rospy.is_shutdown()):
        if (flagImageReceived):
            try:
                lanewarp.working(nodeLanewarp.msgTPC1Camera, nodeLanewarp)

            except Exception as ex:
                print("Exception: {}".format(ex))
                pass

            #except KeyboardInterrupt:
            #    print("Exception: KeyboardInterrupt Lane")


if (__name__ == "__main__"):
    main()
