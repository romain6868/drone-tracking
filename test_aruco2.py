import cv2.legacy
import numpy as np
from picamera2 import Picamera2, Preview
import time
from picamera2.encoders import H264Encoder
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time
from math import sqrt
from array import array
from datetime import datetime
from picamera import PiCamera,Color
from picamera.array import PiRGBArray

def __init__(self, camera, id_to_find):

    #--------------- Boolean variables --------------------
    self.aruco_seen = False
    self.good_aruco_found = False
    self.white_square_seen = False

    #--------------- Counter variables --------------------
    # self.found_count = 0
    # self.not_found_count = 0
    
    #--------------- Camera parametres --------------------
    self.camera = camera
    #self.camera.brightness = 50
    #self.camera.saturation = -50
    #self.camera.contrast = 50
    self.camera.resolution = (640, 480)
    #camera.resolution = (1920, 1080)
    self.camera.framerate = 32
    self.rawCapture = PiRGBArray(self.camera, size=(640, 480))
  
    #--------------- ArUco parametres --------------------
    self.id_to_find = id_to_find
    self.marker_size = 5 #- [cm]
    
    #--------------- Resolution ---------------------------
    # Focal length and sensors dimensions for Pi camera
    # See: https://www.raspberrypi.com/documentation/accessories/camera.html 
    focal_length = 3.60   # Focal length [mm]
    horizotal_res = 640   # Horizontal resolution (x dimension) [px] 
    vertical_res = 480    # Vertical resolution (y dimension) [px]
    sensor_length = 3.76  # Sensor length (x dimension) [mm]
    sensor_height = 2.74  # Sensor length (y dimension) [mm]  
    self.dist_coeff_x = sensor_length/(focal_length*horizotal_res)
    self.dist_coeff_y = sensor_height/(focal_length*vertical_res)
    self.x_imageCenter = int(horizotal_res/2)
    self.y_imageCenter = int(vertical_res/2)
  
    self.img_compteur = 0
    dossier = datetime.now().strftime("%Y_%m_%d-%I:%M:%S_%p")
    parent_dir = '/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images'
    self.path = os.path.join(parent_dir, dossier)
    os.mkdir(self.path)
    # self.f = open(str(self.path) + "log.txt", "a")
  
    #--- Camera calibration path
    calib_path  = "/home/housso97/Desktop/camera_01/cameraa_02/"
    self.camera_matrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
    self.camera_distortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
  
    #--- Definir le dictionnaire aruco 
    self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
    self.parameters  = aruco.DetectorParameters_create()
  
    self.closeToAruco = False

    #--------------- Saved Markers ---------------------------
    # self.saved_markers = {-1: (LocationGlobalRelative(48.58111,7.764722,0), False)}

def Detection_aruco(self, latitude, longitude, altitude, heading, saved_markers, id_to_test, research_whiteSquare):
    # Start time to measure image processing delay
    start_time = time.time()
    
    # Boolean variables reset
    self.aruco_seen = False
    self.good_aruco_found = False
    self.white_square_seen = False
    print("Id to test input detection: %s" % id_to_test)
    #print("[visiont] New image. ID to test: %s. Number of saved items: %s." % (id_to_test, len(saved_markers)))
    x_pixel_target_out = None
    y_pixel_target_out = None
    name = "Text"
    new_location_found = False
    
    #--- Capturer le videocamera 
    self.camera.capture(self.rawCapture, format="bgr")
    frame = self.rawCapture.array
    self.rawCapture.truncate(0)
    
    #frame = cv2.imread("/home/housso97/Desktop/code_IMAV2022_Thomas/saved_images/2022_09_07-12:17:38_PM/mask_closingTest_1_Img_29_lat_48.5809508lon_7.7648142alt_23.8799991607666.png")
    
    font = cv2.FONT_HERSHEY_PLAIN  # Text font for frame annotation
    
    """print("Captured frame: " + str(type(frame)))  
    frame = cv2.flip (frame,-1)
    print("Flipped frame: " + str(type(frame)))
    cv2.waitKey(33)"""
    
    self.img_compteur+=1
    
    ########################## traitement pour aruco
    gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red
    
    #blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
    #hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
    #lower_bound = (0,200,0)     # Select white color in HLS space
    #upper_bound = (255,255,255)
    #mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
  
    #-- Trouver tous les marquers dans l'image
    corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                              cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
    # print(ids.flatten()[0])
    # print(type(ids))
    # print("ids : "+str(ids))

    #--------------- Detection ArUco Tags ---------------------------
    if ids is not None : # and ids[0] == Detection.id_to_find:
      # Boolean update
      self.aruco_seen = True
      # self.found_count += 1
      
      aruco_id = ids.flatten()[0]  # Select the first ArUco id from the list
      x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
      y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
      x_centerPixel_target = int(x_sum*.25)
      y_centerPixel_target = int(y_sum*.25)
      x_pixel_target_out = x_centerPixel_target
      y_pixel_target_out = y_centerPixel_target
      arrete_marker_pxl = sqrt((corners[0][0][0][0]-corners[0][0][1][0])**2+(corners[0][0][0][1]-corners[0][0][1][1])**2)
        
      cv2.line(frame, (x_centerPixel_target, y_centerPixel_target-20), (x_centerPixel_target, y_centerPixel_target+20), (0, 0, 255), 2)
      cv2.line(frame, (x_centerPixel_target-20, y_centerPixel_target), (x_centerPixel_target+20, y_centerPixel_target), (0, 0, 255), 2)
      cv2.putText(frame, str(aruco_id)+"a", (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)
      # Estimating marker location from vision
      distance_vision, angle_vision = get_distance_angle_picture(self.x_imageCenter, self.y_imageCenter,
                                                                 x_centerPixel_target, y_centerPixel_target,
                                                                 altitude, self.dist_coeff_x, self.dist_coeff_y)
      current_location = LocationGlobalRelative(latitude, longitude, 0)
      estimated_location = get_GPS_location(current_location, heading + angle_vision, distance_vision)

      # saved_location = saved_markers[id_to_test][0]  # Compare to id_to_test saved location
      # distance_meters = get_distance_metres(estimated_location, saved_location)
      # cv2.putText(frame, str(aruco_id), (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)
      #print(distance_meters) 
      # If the white square of interest is located at the ArUco place
      saved_markers[aruco_id] = (estimated_location, True)  # Save Aruco id and its location
        # print(saved_markers[aruco_id])
        # saved_markers[aruco_id][1] = True  # Saved Aruco markers explored, not interesting
        #saved_markers.pop(id_to_test)                      # Remove current white square id

      if aruco_id == self.id_to_find:
        self.good_aruco_found = True
        
    #--------------- Detection White Squares ------------------------
    elif research_whiteSquare == True :      
      #------------- Image processing for white squares -------------
      blur = cv2.GaussianBlur(frame,(5,5),0)       # Gaussian blur filter  
      hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)  # Convert from BGR to HLS color space  
      lower_bound = (0,230,0)     # Select white color in HLS space
      upper_bound = (255,255,255)
      mask_hls = cv2.inRange(hls, lower_bound, upper_bound)
      name = "Test_1_Img_" + str(self.img_compteur) + "_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) + "head_" + str(heading)
      cv2.imwrite(os.path.join(self.path, "hls_"+name +".png"), mask_hls)
      # Closing detected elements
      closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))
      mask_closing = cv2.morphologyEx(mask_hls, cv2.MORPH_CLOSE, closing_kernel)
      contours, hierarchy = cv2.findContours(mask_closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      #print ("aire max : "+str(30000*altitude**-1.743))
      #print ("aire min : "+str(25000*altitude**-1.743))

      #--------------White square corners ---------------------------
      for c in contours:
        # pour identifier un carre
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)
        
        #print ("aire du carre : "+str(area))
        #print( "alt : "+ str(altitude))
        
        #--------------- Altitude and square filters ------------------
        if altitude == 0.0:
          print("[visiont] WARNING! Null altitude null sent by rangefinder.")
          # self.f.write("[visiont] WARNING! Null altitude null sent by rangefinder.")
          break
        if area < 60000*altitude**-2  and area > 10000*altitude**-2 and len(approx) ==4 and altitude > 3:
          # print("Detection area correspondant")
          (x, y, w, h) = cv2.boundingRect(approx)
          ar = w / float(h)
          #print("ar : "+str(ar))
          if ar >= 0.90 and ar <= 1.10:  # Square filter
            # print ("Detection carre blanc OK")
            # cv2.drawContours(frame, [c], -1, (0, 0, 255), 1)
            x_centerPixel_target = np.mean(c, axis=0)[0][0]
            y_centerPixel_target = np.mean(c, axis=0)[0][1]
            arrete_marker_pxl = sqrt(area)
            
            pixelTest = mask_closing[int(y_centerPixel_target),int(x_centerPixel_target)]
            #print("pixelTest : "+str(pixelTest))
            if pixelTest == 255 :  #verifie couleur du carre detecte 255 c est blanc
              # Boolean and counter update
              self.white_square_seen = True
              # self.found_count += 1
              
              # cv2.drawContours(frame, [c], -1, (255, 0, 0), 1)
              cv2.line(frame, (int(x_centerPixel_target), int(y_centerPixel_target)-20), (int(x_centerPixel_target), int(y_centerPixel_target)+20), (0, 0, 255), 2)
              cv2.line(frame, (int(x_centerPixel_target)-20, int(y_centerPixel_target)), (int(x_centerPixel_target)+20, int(y_centerPixel_target)), (0, 0, 255), 2)

              # Estimating marker location from vision
              distance_vision, angle_vision = get_distance_angle_picture(self.x_imageCenter, self.y_imageCenter,
                                                                         x_centerPixel_target, y_centerPixel_target,
                                                                         altitude, self.dist_coeff_x, self.dist_coeff_y)
              current_location = LocationGlobalRelative(latitude, longitude, 0)
              estimated_location = get_GPS_location(current_location, heading + angle_vision, distance_vision)

              # White square found and compared to dictionary
              new_location_found = True
              white_square_id = 0
              for id_markers in saved_markers:
                saved_location = saved_markers[id_markers][0]
                distance_meters = get_distance_metres(estimated_location, saved_location)
                print(distance_meters)
                # print("distance_meters entre la position du marker teste et l estimation de la nouvelle image : "+str(distance_meters))

                # White square already checked with location fusion
                if distance_meters < 10:
                  new_location_found = False
                  white_square_id = id_markers
                  # print("Distance less than 3m")
                  # new_location_found = False
                  if white_square_id == id_to_test:
                    # print("Pixel target out sent: %s" % white_square_id)
                    x_pixel_target_out = x_centerPixel_target
                    y_pixel_target_out = y_centerPixel_target

                  # Location already found
                #else:
                #  new_location_found = True
                  # x_pixel_target_out = x_centerPixel_target
                  # y_pixel_target_out = y_centerPixel_target

              # Storing new white squares in dictionary
              if new_location_found:
                if max(saved_markers.keys()) <= 1000:
                  white_square_id = 1001        # First white square with id 1001
                else:
                  max_id = max(saved_markers.keys())
                  white_square_id = max_id + 1  # Others white square with growing ids
                print("[visiont] New location found with id %s." % white_square_id)
                # self.f.write("[visiont] New location found with id %s." % white_square_id)
                saved_markers[white_square_id] = (estimated_location, False)

              if white_square_id > 1000:
                cv2.putText(frame, str(white_square_id), (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)
              else:
                self.white_square_seen = False
                cv2.putText(frame, str(white_square_id)+"b", (int(x_centerPixel_target), int(y_centerPixel_target)), font, 1, (0, 0, 0), 2)

              #cv2.imwrite(os.path.join(self.path, "mask_hls"+name), mask_hls)
              #cv2.imwrite(os.path.join(self.path, "mask_closing"+name), mask_closing)

    if self.aruco_seen == False and self.white_square_seen == False:
      # self.not_found_count += 1
      x_pixel_target_out = None
      y_pixel_target_out = None
      name = "Test_1_Img_" + str(self.img_compteur) + "_no_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) + "head_" + str(heading)
      #print ("aruco and white square likely not found")
      # print("notfound_count : "+str(self.notfound_count))
    else:
      name = "Test_1_Img_" + str(self.img_compteur) + "_yes_lat_" + str(latitude)+ "lon_" + str(longitude) + "alt_" + str(altitude) + "head_" + str(heading)

    cv2.circle(frame, (320, 240), 75, (255,255,255), 1)
    cv2.line(frame, (self.x_imageCenter, self.y_imageCenter-20), (self.x_imageCenter, self.y_imageCenter+20), (255, 0, 0), 2)
    cv2.line(frame, (self.x_imageCenter-20, self.y_imageCenter), (self.x_imageCenter+20, self.y_imageCenter), (255, 0, 0), 2)
    
    # End time to measure image processing delay
    end_time = time.time()
    delay = end_time - start_time
    cv2.imwrite(os.path.join(self.path, name+"delay_"+str(delay)+".png"), frame)
    print("Image saved (%s)!" % self.img_compteur)
    
    return x_pixel_target_out, y_pixel_target_out, self.aruco_seen, self.good_aruco_found, self.white_square_seen, saved_markers

picam2 = Picamera2()

camera_config = picam2.create_still_configuration(main={"size": (1920, 1080)}, lores={"size": (640, 480)}, display="lores")
picam2.configure(camera_config)

picam2.start_preview(Preview.QTGL)
picam2.start()

while True :
	picam2.capture_file("test.jpg")
	
	print("[INFO] loading image...")
	image = cv2.imread("test.jpg")
	image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	# loop over the types of ArUco dictionaries
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
	arucoParams = cv2.aruco.DetectorParameters_create()
	corners, ids, rejected = cv2.aruco.detectMarkers(
		image, arucoDict, parameters=arucoParams)
	# if at least one ArUco marker was detected display the ArUco
	# name to our terminal
	if len(corners) > 0:
		print("[INFO] detected marker detected")
		print("id of the first maker",ids.flatten()[0])
	if ids is not None :
		aruco_id = ids.flatten()[0]  # Select the first ArUco id from the list
		x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
		y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
		x_centerPixel_target = int(x_sum*.25)
		print(x_centerPixel_target)
		y_centerPixel_target = int(y_sum*.25)
		print(y_centerPixel_target)
		x_pixel_target_out = x_centerPixel_target
		y_pixel_target_out = y_centerPixel_target
		arrete_marker_pxl = sqrt((corners[0][0][0][0]-corners[0][0][1][0])**2+(corners[0][0][0][1]-corners[0][0][1][1])**2)
       
		cv2.line(image, (x_centerPixel_target, y_centerPixel_target-20), (x_centerPixel_target, y_centerPixel_target+20), (0, 0, 255), 2)
		cv2.line(image, (x_centerPixel_target-20, y_centerPixel_target), (x_centerPixel_target+20, y_centerPixel_target), (0, 0, 255), 2)
		cv2.putText(image, str(aruco_id)+"a", (int(x_centerPixel_target), int(y_centerPixel_target)), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
		cv2.rectangle(image, (x_centerPixel_target+25, y_centerPixel_target+25), (x_centerPixel_target-25, y_centerPixel_target-25), (0, 0, 255), 2)
		cv2.imshow('Test',image)
