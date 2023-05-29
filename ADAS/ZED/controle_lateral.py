
def smooth_steering(new_steering, window_size):
    window_size = window_size
    steering_history = []

    steering_history.append(new_steering)
    if len(steering_history) > window_size:
        steering_history = steering_history[1:]

    smoothed_steering = sum(steering_history) / len(steering_history)
    return smoothed_steering