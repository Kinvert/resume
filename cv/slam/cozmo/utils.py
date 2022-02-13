def add_zeros(num_string, digits):
    num_string = str(num_string)
    while len(num_string) < digits:
        num_string = '0' + num_string
    return num_string

# Trying to pull camera calibration values

#cal = cozmo.camera.CameraConfig
#calib = np.array([[cal.focal_length.x, 0, cozmo.camera.CameraConfig.center.x],
#                      [0, cozmo.camera.CameraConfig.focal_length.y, cozmo.camera.CameraConfig.center.y],
#                      [0, 0, 1]])