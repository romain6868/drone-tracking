#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022
@author: Thomas Pavot
"""
import os
import numpy as np
import cv2
import cv2.aruco as aruco
import math
import sys
from picamera2 import PiCamera2
from utilities import *
import imutils

class Detection:
    def __init__(self):

        #--------------- Resolution ---------------------------
        # Focal length and sensors dimensions for Pi camera
        # See: https://www.raspberrypi.com/documentation/accessories/camera.html 
        self.horizontal_res = 1920   # Horizontal resolution (x dimension) [px] 
        self.vertical_res = 1080    # Vertical resolution (y dimension) [px]
        focal_length = 3.04   # Focal length [mm]
        sensor_length = 3.68  # Sensor length (x dimension) [mm]
        sensor_height = 2.76  # Sensor length (y dimension) [mm]  
        self.horizontal_field_view = 62.2 # [°] Angle horizontal du champ de vision de la caméra
        self.vertical_field_view = 48.8 # [°] Angle vertical du champ de vision de la caméra
        self.dist_coeff_x = sensor_length/(focal_length*self.horizontal_res)
        self.dist_coeff_y = sensor_height/(focal_length*self.vertical_res)

        # Intialisation de la picamera
        self.camera = PiCamera2()
        self.camera.resolution = (self.horizontal_res, self.vertical_res)
        self.camera.framerate = 30
        #self.rawCapture = PiRGBArray(self.camera, size=(self.horizontal_res, self.vertical_res))

        # Récupération du chemin d'accès global
        self.package_path = os.getcwd()
        while self.package_path[-9:] != "S_IOT_2023":
            self.package_path = os.path.dirname(self.package_path)
        sys.path.insert(0, self.package_path)

        # Camera calibration path
        calib_camera_path = self.package_path + "/config/camera/" + str(self.horizontal_res) + "x" + str(self.vertical_res) + "/"
        self.camera_matrix = np.loadtxt(calib_camera_path+'cameraMatrix.txt', delimiter=',')
        self.camera_distortion = np.loadtxt(calib_camera_path+'cameraDistortion.txt', delimiter=',')
        self.matrice_camera_corrigee, self.ROI_camera_corrigee = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.camera_distortion, self.camera.resolution, 1, self.camera.resolution)
        
        # Calucl du centre de l'image après correction de l'image
        self.horizontal_res_corrigee = self.ROI_camera_corrigee[2]-self.ROI_camera_corrigee[0]
        self.vertical_res_corrigee = self.ROI_camera_corrigee[3]-self.ROI_camera_corrigee[1]
        self.x_imageCenter = int(self.horizontal_res_corrigee/2)
        self.y_imageCenter = int(self.vertical_res_corrigee/2)

        # Paramètres pour la détection d'aruco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
        self.parameters = aruco.DetectorParameters_create()
        
        # Paramètres pour la détection de carré blanc
        # Définition des limites maximales et minimales de filtre pour garder la couleur blanche en HLS
        self.lower_bound_filtre_blanc = (0,175,0)
        self.upper_bound_filtre_blanc = (255,255,255)
        self.closing_kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(7,7))

        # On définit la gamme de couleur de bleu que l'on souhaite
        self.lower_bound_filtre_bleu = np.array([90, 90, 25])
        self.upper_bound_filtre_bleu = np.array([160, 255, 200])

        # On définit la gamme de couleur de rouge que l'on souhaite
        self.lower_bound_filtre_red = np.array([159, 105, 25])
        self.upper_bound_filtre_red = np.array([180, 255, 255])   




    # Fonction permettant de prendre une photo avec la camera
    def prise_photo(self):
        photo = np.empty((self.vertical_res * self.horizontal_res * 3), dtype=np.uint8)
        # Prise de la photo
        self.camera.capture(photo, 'bgr')
        # Remaniage de la forme de la photo pour pouvoir la corrigée
        photo = photo.reshape((self.vertical_res, self.horizontal_res, 3))
        # Correction de la photo avec les matrices de correction
        photo_corrigee = cv2.undistort(photo, self.camera_matrix, self.camera_distortion, None, self.matrice_camera_corrigee)
        # Rognage de la matrice pour ne garder que la partie corrigée
        photo_corrigee = photo_corrigee[self.ROI_camera_corrigee[1]:self.ROI_camera_corrigee[1]+self.ROI_camera_corrigee[3],
                                        self.ROI_camera_corrigee[0]:self.ROI_camera_corrigee[0]+self.ROI_camera_corrigee[2]]
        # Renvoi de la photo corrigée
        return photo_corrigee




    def detection_position(self, altitude):

        image = self.prise_photo()

        #Analyse de l'image 
        taille_min_forme_gros = 300  # Seuil pour exclure les formes trop petites, dans la pratique , on peut placer cette taille Ã  0
        taille_min_forme_petit = 300    

        #Mannequin bleu 
        petite_seuil_min_bleu = 0 
        petite_seuil_max_bleu = 800
        moyenne_seuil_min_bleu = 800
        moyenne_seuil_max_bleu = 1200

        #Mannequin rouge
        petite_seuil_min_rouge = 0 
        petite_seuil_max_rouge = 650
        moyenne_seuil_min_rouge = 650
        moyenne_seuil_max_rouge= 1000


        # La variable image représente la vidéo, il est redimensionné avec resize
        image = imutils.resize(image, width=800)
        # Conversion de l'image de l'espace de couleurs BGR (Bleu-Vert-Rouge) à l'espace de couleurs HSV (Teinte-Saturation-Value)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # On dÃ©finit la gamme de couleur de bleu que l'on souhaite ( H va de 0 Ã  180 , S et V de 0 Ã  255)
        lower_blue = np.array([105, 105, 25])
        upper_blue = np.array([150, 255, 255])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        # On dÃ©finit la gamme de couleur de rouge que l'on souhaite
        lower_red = np.array([159, 105, 25])
        upper_red = np.array([180, 255, 255])
        mask_red = cv2.inRange(hsv, lower_red, upper_red)

        # On dÃ©finit la gamme de couleur de blanc/jaune que l'on souhaite
        lower_white = np.array([20, 0, 100])
        upper_white = np.array([35, 20, 255])
        mask_yellow = cv2.inRange(hsv, lower_white, upper_white)

        # Dilater les contours pour fusionner les taches bleues/rouges proches
        dilated_mask_blue = cv2.dilate(mask_blue, None, iterations=3)
        dilated_mask_red = cv2.dilate(mask_red, None, iterations=3)
        dilated_mask_yellow = cv2.dilate(mask_yellow, None, iterations=3)

        # Création d'un masque binaire inverse pour le reste de l'image (masque blanc)
        mask_white = cv2.bitwise_not(cv2.bitwise_or(dilated_mask_blue, dilated_mask_red))
        # On applique le masque binaire bleu/rouge à l'image RGB pour conserver les zones bleues/rouges
        seg_img_blue = cv2.bitwise_and(image, image, mask=dilated_mask_blue)
        seg_img_red = cv2.bitwise_and(image, image, mask=dilated_mask_red)
        seg_img_yellow = cv2.bitwise_and(image, image, mask=dilated_mask_yellow)

        # On crée une image blanche de la même taille que l'image d'origine
        white_img = np.ones_like(image, dtype=np.uint8) * 255
        # On applique le masque binaire inverse à l'image blanche pour avoir le reste en blanc
        seg_img_white = cv2.bitwise_and(white_img, white_img, mask=mask_white)

        # On combine les images segmentées bleues et rouges en les additionnant
        result = cv2.add(seg_img_blue, seg_img_red, seg_img_yellow)
        result = cv2.bitwise_or(result, seg_img_white)
        # On cherche le contour des objets bleu et rouge
        contours_blue, _ = cv2.findContours(dilated_mask_blue.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(dilated_mask_red.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_yellow, _ = cv2.findContours(dilated_mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Nombre de mannequins
        nb_mannequins = 0
        distance_min = 100
        blue_zones_count = 0
        red_zones_count = 0

        #Parmis toutes les formes bleues dÃ©tectÃ©es : 
        for contour in contours_blue:
            x, y, w, h = cv2.boundingRect(contour) #On dÃ©termine ses coordonnÃ©es sur l'image, sa hauteur et sa largeur 
            area = cv2.contourArea(contour) # On calule l'aire de la forme en pixels

            for contour_yellow in contours_yellow:

                area_yellow = cv2.contourArea(contour_yellow) # On calule l'aire de la forme en pixels

                if area_yellow > taille_min_forme_gros :

                    # Calculer la distance entre le centre du contour blanc et le contour bleu
                    center_yellow = np.mean(contour_yellow, axis=0)[0]
                    center_blue = np.mean(contour, axis=0)[0]
                    distance = np.linalg.norm(center_yellow - center_blue)

                    if distance < distance_min:
                        area = area + area_yellow

            for contour_red in contours_red:    

                area_red = cv2.contourArea(contour_red) # On calule l'aire de la forme en pixels

                if area_red > taille_min_forme_petit :

                    # Calculer la distance entre le centre du contour blanc et le contour bleu
                    center_red = np.mean(contour_red, axis=0)[0]
                    center_blue = np.mean(contour, axis=0)[0]
                    distance = np.linalg.norm(center_red - center_blue)

                    if distance < distance_min:
                        area = area + area_red
                        
            # VÃ©rifier si l'aire est supÃ©rieure au seuil de taille minimum
            if area > taille_min_forme_gros:
                
                
                # CatÃ©goriser l'aire en debout, assis et allongÃ© en fonction des seuils de taille
                if petite_seuil_min_bleu < area < petite_seuil_max_bleu:
                    category = 'stand'
                elif moyenne_seuil_min_bleu < area < moyenne_seuil_max_bleu:
                    category = 'sit'
                else:
                    category = 'lie'

                # Dessiner le rectangle autour de l'objet bleu
                cv2.drawContours(result, [contour], 0, (0, 255, 255), 3)
                # Ã‰crire la catÃ©gorie au centre de l'objet bleu
                cv2.putText(result, category, (x + int(w / 2), y + int(h / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255,0), 2)

                # IncrÃ©menter le compteur de zones bleues
                blue_zones_count += 1

        #Parmis toutes les formes rouges dÃ©tectÃ©es : 
        for contour in contours_red:
            x, y, w, h = cv2.boundingRect(contour) #On dÃ©termine ses coordonnÃ©es sur l'image, sa hauteur et sa largeur 
            area = cv2.contourArea(contour) # On calcule l'aire de la forme en pixels

            # VÃ©rifier si l'aire est supÃ©rieure au seuil de taille minimum
            if area > taille_min_forme_petit:
                # CatÃ©goriser l'aire en debout, assis et allongÃ© en fonction des seuils de taille
                if petite_seuil_min_rouge < area < petite_seuil_max_rouge:
                    category = 'stand'
                elif moyenne_seuil_min_rouge < area < moyenne_seuil_max_rouge:
                    category = 'sit'
                else:
                    category = 'lie'

                # Dessiner le rectangle autour de l'objet rouge
                cv2.drawContours(result, [contour], 0, (0, 255, 255), 3)
                # Ã‰crire la catÃ©gorie au centre de l'objet rouge
                cv2.putText(result, category, (x + int(w / 2), y + int(h / 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # IncrÃ©menter le compteur de zones bleues
                red_zones_count += 1
                
        #Parmis toutes les formes blanches/jaunes dÃ©tectÃ©es : 
        for contour in contours_yellow:
            x, y, w, h = cv2.boundingRect(contour) #On dÃ©termine ses coordonnÃ©es sur l'image, sa hauteur et sa largeur 
            area = cv2.contourArea(contour) # On calcule l'aire de la forme en pixels

            # VÃ©rifier si l'aire est supÃ©rieure au seuil de taille minimum
            if area > taille_min_forme_gros:
                
                for contour_blue in contours_blue:
                    
                    # Calculer la distance entre le centre du contour blanc et le contour bleu
                    center_white = np.mean(contour, axis=0)[0]
                    center_blue = np.mean(contour_blue, axis=0)[0]
                    distance = np.linalg.norm(center_white - center_blue)
                    
                    
                    if distance < distance_min:
                        # Dessiner le rectangle autour de l'objet blanc
                        cv2.drawContours(result, [contour], 0, (0, 255, 255), 3)

        #Calculer le nombre de mannequins
        nb_mannequins = blue_zones_count + red_zones_count
        
        # Afficher le nombre de zones bleues identifiées
        cv2.putText(result, f"Mannequins : {nb_mannequins}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return nb_mannequins, image, result



          

    # Fonction servant trouver les arucos vus par la caméra et renvoyant les coordonnées de son centre
    # Si aucun n'aruco n'est détecté, on renvoie des variables vides
    # On mettant le paramètre "return_image" à True, on renvoie également l'image acquise avec visualisation de l'aruco et de son ID
    # On trace alors les contours de l'aruco et on écrit son ID à côté de lui.
    # Si l'aruco n'a pas été détecté et que "return_image=True", on renvoie simplement l'image acquise    
    def detection_aruco(self, return_image = False):

        # Prise de la photo avec la PiCamera
        image = self.prise_photo()
        # Conversion de l'image en nuances de gris
        gray  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # Recherche d'aruco dans l'image prise
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters)
        
        # Si la longueur du tuple "corners", n'est pas de vide, i.e. si au moins un aruco est détecté            
        if len(corners) != 0 :      

            # On calcule la moyenne des positions en x et y des arrêtes de l'aruco
            x_aruco_center = int((corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0])*.25)
            y_aruco_center = int((corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1])*.25)
            # On récupère l'ID de l'aruco détecté
            aruco_id = ids.flatten()[0]

            # Si l'on ne souhaite pas renvoyer l'image, on renvoie uniquement les coordonnées du centre de l'aruco et son ID
            if return_image == False:
                return x_aruco_center, y_aruco_center, aruco_id
            
            # Si l'on souhaite renvoyer l'image, on trace les éléments graphiques permettant de montrer que la détection a bien été réalisée
            # On trace des lignes entourant l'Aruco marker
            cv2.line(image, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), (int(corners[0][0][1][0]), int(corners[0][0][1][1])), (0, 255, 0), 2)
            cv2.line(image, (int(corners[0][0][1][0]), int(corners[0][0][1][1])), (int(corners[0][0][2][0]), int(corners[0][0][2][1])), (0, 255, 0), 2)
            cv2.line(image, (int(corners[0][0][2][0]), int(corners[0][0][2][1])), (int(corners[0][0][3][0]), int(corners[0][0][3][1])), (0, 255, 0), 2)
            cv2.line(image, (int(corners[0][0][3][0]), int(corners[0][0][3][1])), (int(corners[0][0][0][0]), int(corners[0][0][0][1])), (0, 255, 0), 2)
            # On trace un point rouge au centre de l'Aruco
            cv2.circle(image, (x_aruco_center, y_aruco_center), 4, (0, 255, 0), -1)
            # On écrit l'ID de l'aruco détecté au-dessus de l'aruco 
            cv2.putText(image, str(aruco_id), (x_aruco_center, y_aruco_center-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            # On renvoie les coordronnées calculées, l'ID de l'aurco et l'image modifiée
            return x_aruco_center, y_aruco_center, aruco_id, image
 
        # Si l'aruco n'a pas été détecté, on renvoie des variables vides
        else:
            return (None, None, None, image) if return_image == True else (None, None, None)




    def calcul_radian_aruco(self,  return_image = True):
        #Vérification que la matrice générée est une matrice de rotation
        def isRotationMatrix(R) : 
            Rt=np.transpose(R)
            shouldBeIdentity = np.dot(Rt,R)
            I = np.identity(3, dtype = R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6

        #Conversion de la matrice de rotation pour en déduire les angles d'Euler
        def rotationMatrixToEulerAngles(R) :
            assert(isRotationMatrix(R))
            sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
            singular = sy <1e-6
            if not singular :
                x=math.atan2(R[2,1] ,R[2,2])
                y = math.atan2(-R[2,0], sy)
                z = math.atan2(R[1,0], R[0,0])
            else :
                x = math.atan2(-R[1,2], R[1,1])
                y = math.atan2(-R[2,0], sy)
                z = 0
            return np.array([x, y, z])
         
        #Je mettrai ce bordel dans une sous-fonction plus tard
        #Modifications pour tester la détection de l'orientation de l'ArUco via quelques transformations   
        #STEP1: faut chercher les matrices de calibrations et de distortion     
        dist_coef = self.camera_distortion
        cam_mat = self.matrice_camera_corrigee
        MARKER_SIZE = 100  # centimeters
        marker_dict = self.aruco_dict
        param_markers = self.parameters
        
        #STEP2: il faut une image à travailler        
        image = self.prise_photo()
        
        #STEP3: detection de l'aruco et prise de ses coordonnées
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) #nuances de gris de l'image
        marker_corners, marker_IDs, reject = aruco.detectMarkers(gray_image, marker_dict, parameters=param_markers)#détection des maruqueurs sur l'image (ID + coordonées en pixels de           leurs coins)
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            ) #extraction des matrices de translation tVec et de rotation rVec
            
            R , _=cv2.Rodrigues(rVec)#transformation de Rodrigues-Euler
            _,_,z = rotationMatrixToEulerAngles(R)#extraction des angles d'Euler de la matrice de rotation
            total_markers = range(0, marker_IDs.size)
            #Step4: t'inquiètes pas ça va bien se passer
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv2.polylines(
                    image, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                
                # Draw the pose of the marker
                point = cv2.drawFrameAxes(image, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                print("AruCo a un angle de " + str(float(z))+" radians")
        
        return z,image
    


    def detection_carre_bleu(self):

        # Détection de carré bleus à 30m
        
        # Prise de la photo avec la PiCamera
        image = self.prise_photo()
        # Redimensionnement de l'image
        image = imutils.resize(image, width=600)

        # Convertir l'image de l'espace de couleurs BGR (Bleu-Vert-Rouge) à l'espace de couleurs HSV (Teinte-Saturation-Value)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Définir la plage de couleur de bleu que l'on souhaite détecter
        lower_blue = np.array([90, 100, 100])
        upper_blue = np.array([120, 255, 255])
        
        # Créer un masque binaire en fonction de l'espace de couleur HSV pour les zones bleues
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Trouver les contours dans le masque binaire
        contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Parcourir tous les contours détectés et dessiner les rectangles autour des formes détectées
        for contour in contours:
            # Approximer la forme du contour en un rectangle
            x, y, w, h = cv2.boundingRect(contour)

            # Ignorer les petits contours qui pourraient être du bruit
            if w > 10 and h > 10:
                # Dessiner un rectangle autour de la forme détectée (dans ce cas, le carré bleu)
                # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # mise en commentaire pour eviter d avoir le rectangle sur les images pour ne pas fausser la carto
                # Mettre le nom de la forme au centre du rectangle
                # cv2.putText(image, 'Carre Bleu', (x + int(w/2) - 50, y + int(h/2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                return True, image

        return False, image
