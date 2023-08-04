import cv2
import numpy as np
import math
from pykalman import KalmanFilter

kf = KalmanFilter(transition_matrices=[1],
                  observation_matrices=[1],
                  initial_state_mean=0,
                  transition_covariance=0.01,
                  observation_covariance=0.03)


def warp(imagem):

    height = imagem.shape[0]
    width = imagem.shape[1]
    img_size = (1280, 720)

    src = np.float32([[0, height], [width, height], [
                     200, 530], [(width-200), 530]])
    # src = np.float32([[0, height], [width, height], [200, 630], [(width-200), 630]])
    dst = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

    M = cv2.getPerspectiveTransform(src, dst)  # [estudar]
    M_inv = cv2.getPerspectiveTransform(dst, src)  # [estudar]
    warp = cv2.warpPerspective(imagem, M, img_size)  # [estudar]
    # warp_inv = cv2.warpPerspective(img, M_inv, img_size)

    return warp, M_inv

###  Thresholded da imagem ###


def binary_thresholder(img):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_v = img_yuv[:, :, 2]  # destaca as linhas verdes

    blurred = cv2.GaussianBlur(img_v, (7, 7), 0)

    # (T, thresh) = cv2.threshold(blurred, 123, 255, cv2.THRESH_BINARY_INV) #outdoor
    # (T, thresh) = cv2.threshold(blurred, 110, 255, cv2.THRESH_BINARY_INV) #indoor
    (T, thresh) = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY_INV)  # indoor
    # (T, thresh) = cv2.threshold(blurred, 124, 255, cv2.THRESH_BINARY_INV) #outdoor 12-14hrs

    kernel = np.ones((7, 7), np.uint8)
    img_dilate = cv2.dilate(thresh, kernel, iterations=1)
    img_erode = cv2.erode(img_dilate, kernel, iterations=1)

    maskYUV = img_erode

    return maskYUV


def find_lane_pixels_using_histogram(binary_warped):
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


def fit_poly(binary_warped, leftx, lefty, rightx, righty):
    ### Fit a second order polynomial to each with np.polyfit() ###
    if len(leftx) == 0 or len(lefty) == 0 or len(rightx) == 0 or len(righty) == 0:
        # Use the last known good values
        leftx = last_known_good_leftx
        lefty = last_known_good_lefty
        rightx = last_known_good_rightx
        righty = last_known_good_righty
    else:
        # Update the last known good values
        last_known_good_leftx = leftx
        last_known_good_lefty = lefty
        last_known_good_rightx = rightx
        last_known_good_righty = righty

    left_fit = np.polyfit(lefty, leftx, 2)
    right_fit = np.polyfit(righty, rightx, 2)

    # Generate x and y values for plotting
    ploty = np.linspace(0, binary_warped.shape[0] - 1, binary_warped.shape[0])
    try:
        left_fitx = left_fit[0] * ploty ** 2 + \
            left_fit[1] * ploty + left_fit[2]
        right_fitx = right_fit[0] * ploty ** 2 + \
            right_fit[1] * ploty + right_fit[2]
    except TypeError:
        # Avoids an error if `left` and `right_fit` are still none or incorrect
        print('The function failed to fit a line!')
        left_fitx = 1 * ploty ** 2 + 1 * ploty
        right_fitx = 1 * ploty ** 2 + 1 * ploty

    return left_fit, right_fit, left_fitx, right_fitx, ploty


def measure_curvature_meters(binary_warped, left_fitx, right_fitx, ploty):
    # Define conversions in x and y from pixels space to meters
    ym_per_pix = 30 / 720  # meters per pixel in y dimension
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

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


def measure_position_meters(binary_warped, left_fit, right_fit):
    # Define conversion in x from pixels space to meters
    xm_per_pix = 3.7 / 700  # meters per pixel in x dimension

    # Define a list of y positions at which to calculate the x positions
    # change num=10 to any number of points you want
    y_positions = np.linspace(0, binary_warped.shape[0], num=10)

    center_lane_positions = []
    for y in y_positions:
        # Calculate left and right line positions
        left_x_pos = left_fit[0] * y ** 2 + left_fit[1] * y + left_fit[2]
        right_x_pos = right_fit[0] * y ** 2 + right_fit[1] * y + right_fit[2]
        # Calculate the x position of the center of the lane
        center_lane_x_pos = (left_x_pos + right_x_pos) // 2
        center_lane_positions.append((center_lane_x_pos, y))

    # Choose the y value corresponding to the bottom of the image
    # y_max = int(binary_warped.shape[0]*0.85)
    # Calculate the deviation between the center of the lane and the center of the picture
    # The car is assumed to be placed in the center of the picture
    # If the deviation is negative, the car is on the felt hand side of the center of the lane
    # veh_pos = ((binary_warped.shape[1] // 2) - center_lane_positions[-1][1]) * xm_per_pix
    veh_pos = ((binary_warped.shape[1] // 2) -
               np.mean(center_lane_positions[-1])) * xm_per_pix

    return veh_pos, center_lane_positions, y_positions


def project_lane_info(img, binary_warped, ploty, left_fitx, right_fitx, M_inv, left_curverad, right_curverad, veh_pos, angDir):
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


def cinematicaPurePursuit(distance_to_center):
    offSet = 1.05  # 1.2
    wheelbase = 0.75
    target_point = [offSet, distance_to_center]

    alpha = math.atan2(target_point[1], target_point[0])
    wheel_angle = math.atan(2 * wheelbase * math.sin(alpha) / offSet)

    # convertendo angulo da roda de [-pi/2, pi/2] para o intervalo [1, 50]
    wheel_angle_mapped = (math.degrees(wheel_angle) + 90) / 180 * 49 + 1

    # Converte distance_to_center para o intervalo [-1, 1]
    normalized_distance = distance_to_center / 0.9

    # Mapeia normalized_distance para o intervalo [0, 1]
    normalized_distance = (normalized_distance + 1) / 2

    # Mapeia normalized_distance para o intervalo [1, 50]
    wheel_angle_mapped = (1 - normalized_distance) * 49 + 1

    return wheel_angle_mapped


def angValidate(angDir):
    angMin = 1
    angMax = 50
    angDir = int(angDir)
    if (angDir < angMin):
        angDir = angMin
    elif (angDir > angMax):
        angDir = angMax

    return angDir


def transform_point(point, M_inv):
    point = np.array([[point[0], point[1]]], dtype='float32')
    point = np.array([point])
    transformed_point = cv2.perspectiveTransform(point, M_inv)

    return int(transformed_point[0][0][0]), int(transformed_point[0][0][1])


def working(imagem):
    # Imagem Bird Eye View
    img_be, M = warp(imagem)
    # cv2.imshow('be', img_be)

    # Imagem Binarizada
    img_bin = binary_thresholder(img_be)

    # Resgatando e validando pontos da linha esquerda e direita
    leftx, lefty, rightx, righty = find_lane_pixels_using_histogram(
        img_bin)

    left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(
        img_bin, leftx, lefty, rightx, righty)
    left_curverad, right_curverad = measure_curvature_meters(
        img_bin, left_fitx, right_fitx, ploty)

    veh_pos, center, center_y = measure_position_meters(
        img_bin, left_fit, right_fit)

    current_state = np.array([0])
    current_covariance = np.array([1])

    angDir = cinematicaPurePursuit(veh_pos)

    current_state, current_covariance = kf.filter_update(
        current_state, current_covariance, angDir)

    angDir = current_state[0]

    # angDir = cinematicaPurePursuit(veh_pos)
    angDir = angValidate(angDir)

    out_img, navegation = project_lane_info(
        imagem[:, :, 0:3], img_bin, ploty, left_fitx, right_fitx, M, left_curverad, right_curverad, veh_pos, angDir)

    for i in range(len(center)):
        cv2.circle(img_bin, (int(center[i][0]), int(
            center[i][1])), 10, (255, 255, 255), 5)

    cv2.imshow('Bird Eye View Binarizada', img_bin)

    for point in center:
        # Apply inverse perspective transform to the point
        transformed_point = transform_point(point, M)
        cv2.circle(out_img, transformed_point, 10, (255, 0, 0), 5)

    return out_img


img = cv2.imread('img_zed2.jpg')
out_img = working(img)
cv2.imshow("resultado", out_img)
cv2.waitKey(0)
