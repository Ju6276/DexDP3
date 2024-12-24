import cv2
import numpy as np
import pyrealsense2 as rs
import time

'''
This script is used to estimate the extrinsic parameters of the ArUco markers.
'''

# initialize RealSense camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

# get camera intrinsic parameters
color_stream = profile.get_stream(rs.stream.color)
intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

# build camera intrinsic matrix
camera_matrix = np.array([
    [intrinsics.fx, 0, intrinsics.ppx],
    [0, intrinsics.fy, intrinsics.ppy],
    [0, 0, 1]
])
dist_coeffs = np.array(intrinsics.coeffs)

# define ArUco parameters - TAG36H11
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
parameters = cv2.aruco.DetectorParameters()

# create ArUco detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# define marker actual size (meter)
marker_size = 0.08  

try:
    while True:
        # get frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # detect ArUco markers
        corners, ids, rejected = detector.detectMarkers(color_image)

        if ids is not None:
            print(f"detected markers, ID: {ids}")
            # draw detected markers
            cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

            for i in range(len(ids)):
                # get marker corners
                marker_corners = corners[i]
                
                # estimate pose
                success, rvec, tvec = cv2.solvePnP(
                    objectPoints=np.array([[-marker_size/2, marker_size/2, 0],
                                         [marker_size/2, marker_size/2, 0],
                                         [marker_size/2, -marker_size/2, 0],
                                         [-marker_size/2, -marker_size/2, 0]], dtype=np.float32),
                    imagePoints=marker_corners.reshape(-1, 2),
                    cameraMatrix=camera_matrix,
                    distCoeffs=dist_coeffs
                )

                if success:
                    # draw coordinate axes
                    length = marker_size  
                    cv2.drawFrameAxes(color_image, camera_matrix, dist_coeffs, rvec, tvec, length)
                    
                    # convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvec)
                    
                    # build 4x4 extrinsic matrix (transform from camera coordinate system to marker coordinate system)
                    extrinsic_matrix = np.eye(4)
                    extrinsic_matrix[:3, :3] = R
                    extrinsic_matrix[:3, 3] = tvec.reshape(3)

                    print(f"\nmarker {ids[i][0]} information:")
                    print("1. extrinsic matrix (4x4):")
                    print(extrinsic_matrix)
                    
                    # decompose and display position and pose information
                    position = tvec.reshape(3)
                    euler_angles = cv2.RQDecomp3x3(R)[0]
                    
                    print("\n2. decomposed information:")
                    print(f"position (in camera coordinate system):")
                    print(f"X: {position[0]:.3f} m (right direction)")
                    print(f"Y: {position[1]:.3f} m (up direction)")
                    print(f"Z: {position[2]:.3f} m (vertical to marker plane)")
                    print(f"Euler angles (degree):")
                    print(f"Roll  (around X axis): {euler_angles[0]:.2f}°")
                    print(f"Pitch (around Y axis): {euler_angles[1]:.2f}°")
                    print(f"Yaw   (around Z axis): {euler_angles[2]:.2f}°")
                    
                    # add coordinate system direction description
                    print("\n3. marker coordinate system description:")
                    print("- red axis (X): right direction in marker plane")
                    print("- green axis (Y): up direction in marker plane")
                    print("- blue axis (Z): vertical to marker plane")
                    print("\nmarker placement suggestion:")
                    print("- marker should be flat (not tilted)")
                    print("- ID code readable")
                    print("- ensure sufficient lighting, avoid reflections")
        else:
            print("no markers detected")

        # display image
        cv2.imshow('RealSense', color_image)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()