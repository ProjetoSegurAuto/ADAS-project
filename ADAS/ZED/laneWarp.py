import cv2
import numpy as np
import pyzed.sl as sl

### Transformando imagem da camera em Bird Eye ###
def warp(img):
    height, width = img.shape[:2]
    img_size = (1280, 720)

    src = np.float32([[0, height], [width, height], [200, 530], [(width-200), 530]])
    dst = np.float32([[0, height], [width, height], [0, 0], [width, 0]])

    M = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    warp = cv2.warpPerspective(img, M, img_size)

    return warp, M_inv

###  Thresholded da imagem ###
def binary_thresholder(img):
    img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    img_v = img_yuv[:, :, 2]
    
    # Aplicar equalização de histograma para melhorar a iluminação
    img_v_eq = cv2.equalizeHist(img_v)

    # Aplicar desfoque adaptativo para reduzir o ruído e preservar as bordas
    blurred = cv2.bilateralFilter(img_v_eq, 7, 75, 75)

    # Limiar adaptativo para segmentar as linhas de forma mais adaptável
    thresh = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 11, 5)

    # Operações morfológicas para remover pequenos ruídos e preencher lacunas
    kernel = np.ones((3, 3), np.uint8)
    img_open = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=2)
    img_close = cv2.morphologyEx(img_open, cv2.MORPH_CLOSE, kernel, iterations=2)

    return img_close

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


def draw_poly_lines(binary_warped, left_fitx, right_fitx, ploty):
    # Create an image to draw on and an image to show the selection window
    out_img = np.dstack((binary_warped, binary_warped, binary_warped)) * 255
    window_img = np.zeros_like(out_img)

    margin = 100
    # Generate a polygon to illustrate the search window area
    # And recast the x and y points into usable format for cv2.fillPoly()
    left_line_window1 = np.array([np.transpose(np.vstack([left_fitx - margin, ploty]))])
    left_line_window2 = np.array([np.flipud(np.transpose(np.vstack([left_fitx + margin,
                                                                    ploty])))])
    left_line_pts = np.hstack((left_line_window1, left_line_window2))
    right_line_window1 = np.array([np.transpose(np.vstack([right_fitx - margin, ploty]))])
    right_line_window2 = np.array([np.flipud(np.transpose(np.vstack([right_fitx + margin,
                                                                     ploty])))])
    right_line_pts = np.hstack((right_line_window1, right_line_window2))

    # Draw the lane onto the warped blank image
    cv2.fillPoly(window_img, np.int_([left_line_pts]), (100, 100, 0))
    cv2.fillPoly(window_img, np.int_([right_line_pts]), (100, 100, 0))
    result = cv2.addWeighted(out_img, 1, window_img, 0.3, 0)

    # Plot the polynomial lines onto the image
    plt.plot(left_fitx, ploty, color='green')
    plt.plot(right_fitx, ploty, color='blue')
    ## End visualization steps ##
    return result


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
    # Choose the y value corresponding to the bottom of the image
    y_max = binary_warped.shape[0]
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


def project_lane_info(img, binary_warped, ploty, left_fitx, right_fitx, M_inv, left_curverad, right_curverad, veh_pos):
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

    cv2.putText(out_img, 'Curve Radius [m]: ' + str((left_curverad + right_curverad) / 2)[:7], (40, 70),
                cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6, (255, 0, 0), 2, cv2.LINE_AA)
    cv2.putText(out_img, 'Center Offset [m]: ' + str(veh_pos)[:7], (40, 150), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1.6,
                (255, 0, 0), 2, cv2.LINE_AA)

    return out_img

def main():
    cam = sl.Camera()
    init = sl.InitParameters()

    # Resolution: HD2K, HD1080, HD720, VGA
    init.camera_resolution = sl.RESOLUTION.HD1080

    # FPS: 15, 30, 60, 100 
    init.camera_fps = 30
    # ULTRA: offers the highest depth range and better preserves Z-accuracy along the sensing range.
    # QUALITY: has a strong filtering stage giving smooth surfaces.
    # PERFORMANCE: designed to be smooth, can miss some details.
    # Generally speaking, we recommend using the ULTRA mode for both desktop and embedded applications. 
    # If your application requires a lot of resources, switch to PERFORMANCE mode.
    init.depth_mode = sl.DEPTH_MODE.ULTRA 
    init.sdk_verbose = True
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


    # Define the Object Detection module parameters
    detection_parameters = sl.ObjectDetectionParameters()
    detection_parameters.image_sync = True
    detection_parameters.enable_tracking = True
    detection_parameters.enable_mask_output = True

    # Object tracking requires camera tracking to be enabled
    if detection_parameters.enable_tracking:
        cam.enable_positional_tracking()

    # Configure object detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    obj_runtime_param.detection_confidence_threshold = 60
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]    # Only detect Persons
    
    # Use STANDARD sensing mode
    

    # Create ZED objects filled in the main loop
    objects = sl.Objects()

    #cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 0)
    image = sl.Mat()

    # Set runtime parameters
    runtime = sl.RuntimeParameters()
    runtime.sensing_mode = sl.SENSING_MODE.STANDARD  

    while True:
        try:  
            err = cam.grab(runtime)
            #err2 = cam.enable_object_detection(detection_parameters)

            # Retrieve left image
            cam.retrieve_image(image, sl.VIEW.LEFT)
            #cam.retrieve_objects(objects, obj_runtime_param)
            #cam.retrieve_image(image, objects)
            #viewer.update_view(image, objects)
            
            img = image.get_data()
            
            #path = 'image'+str(i)+'.jpg'
            #print(path)
            # imagem original
            #img = cv2.imread(path)
            #cv2.imshow('original', img)



            left_fit_hist = np.array([])
            right_fit_hist = np.array([])

            #BE
            img_be, M = warp(img)
            cv2.imshow('be', img_be)

            # imagem binarizadda
            img_bin = binary_thresholder(img_be)
            cv2.imshow('bin', img_bin)
            #cv2.waitKey(0)
     
            # exibindo pontos da linha esquerda e direita
            leftx, lefty, rightx, righty = find_lane_pixels_using_histogram(img_bin)
            #print(leftx, lefty, rightx, righty)
            left_fit, right_fit, left_fitx, right_fitx, ploty = fit_poly(img_bin, leftx, lefty, rightx, righty)
            #print(left_fitx)
            left_curverad, right_curverad = measure_curvature_meters(img_bin, left_fitx, right_fitx, ploty)
            #print(left_curverad, right_curverad)
            veh_pos, center = measure_position_meters(img_bin, left_fit, right_fit)
            #print("posição do veiculo em metros: {}".format(veh_pos))
            
            out_img = project_lane_info(img[:, :, 0:3], img_bin, ploty, left_fitx, right_fitx, M, left_curverad, right_curverad, veh_pos)
            #viewer.update_view(out_img, objects)
            cv2.circle(out_img, (int(center),820), radius=10, color=(0, 0, 255), thickness=-1)
            #print(center)
            cv2.imshow('resultado', out_img)

            #cv2.imwrite('resultado_'+str(i)+'.jpg', out_img)
            cv2.waitKey(1)
            
                 
        except KeyboardInterrupt:
            print("Exception: KeyboardInterrupt") 
            #viewer.exit()
            break

        except Exception as ex:
            print("Exception: {}".format(ex))
            #viewer.exit()
            #pass
   
        
if __name__ == "__main__":
    main()