import math

def smooth_steering(new_steering, window_size):
    window_size = window_size
    steering_history = []

    steering_history.append(new_steering)
    if len(steering_history) > window_size:
        steering_history = steering_history[1:]

    smoothed_steering = sum(steering_history) / len(steering_history)
    return smoothed_steering

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
    angDir = smooth_steering(angDir, 5)
    velRodaD = rpm
    velRodaE = rpm

    return angDir, velRodaD, velRodaE