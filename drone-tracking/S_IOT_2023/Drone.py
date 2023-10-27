#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on 2022

@author: Thomas Pavot
"""

import cv2
from cv2 import aruco as aruco
from time import sleep
from math import atan2, cos, sin, sqrt
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from utilities import *
from Detection import Detection



class Drone:

#-------------------------------------------------------------------------------------------------------------------------------------------
# Initialisation de la classe 
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Constructeur de la classe se connectant au drone
    def __init__(self):     
        
        # Coefficients de l'asservissement PID de l'atterrissage
        self.kp_atterrissage = 0 # Coefficient mis à 0 car initialisé plus tard
        self.kd_atterrissage = 0.0002
        self.ki_atterrissage = 0.000001
        self.coefficient_kp_atterrissage = 0.5

        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleEst_atterrissage = 0
        self.erreurIntegraleNord_atterrissage = 0
        self.erreurAnterieureEst_atterrissage = 0
        self.erreurAnterieureNord_atterrissage = 0

        # Coefficients de l'asservissement PID du suivi de véhicule
        self.kp_suivi_vehicule = 0.0125
        self.kd_suivi_vehicule = 0.000625
        self.ki_suivi_vehicule = 0.000002
        self.compteur_non_detection = 0
        self.stored_vEst = 0
        self.stored_vNord = 0
        
        # Initialisation des coefficients pour le calcul des erreurs dérivées et intégrales
        self.erreurIntegraleEst_suivi_vehicule = 0
        self.erreurIntegraleNord_suivi_vehicule = 0
        self.erreurAnterieureEst_suivi_vehicule = 0
        self.erreurAnterieureNord_suivi_vehicule = 0

        self.previousLatitude = 0
        self.previousLongitude = 0
        self.previousCarPosition = None
        self.previousMeasuredTime = None

        
        # Connexion au drone et initialisation de la caméra
        print("Connexion au drone et initialisation de la caméra")
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=57600, heartbeat_timeout=2)
        self.camera = Detection()
        print("Connexion et initialisation terminées")

        # Envoi de consigne de servo pour la condition d'atterrissage
        msg = self.vehicle.message_factory.command_long_encode(0, 0,  # target_system, target_component
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,  # command
            0,  # confirmation
            10,  # servo number
            1900,  # servo position between 1000 ferme and 2000
            0, 0, 0, 0, 0)  # param 3 ~ 7 not used
        self.vehicle.send_mavlink(msg)


    #set_mode - set the mode of the vehicle as long as we are in control
    def set_mode(self, mode):
        self.vehicle.mode = VehicleMode(mode)
        self.vehicle.flush()
            
    #get_mode - get current mode of vehicle 
    def get_mode(self):
        return self.vehicle.mode.name




#-------------------------------------------------------------------------------------------------------------------------------------------
# Fonctions de pilotage du drone
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction demandant au drone de passer en mode "STABILIZE" puis en mode "AUTO" et enfin passe le drone en mode "GUIDED"
    # Cette fonction est utilisée comme sécurité avant un code de pilotage de drone 
    # Elle permet de passer à la suite du code en changement adéquatement de mode avec la télécommande
    # Après avoir obtenu la bonne séquence de modes, elle passage également le drone en mode "GUIDED" pour contrôle le drone avec dronekit
    def attente_stabilize_auto(self):

        # Attente du mode "STABIIZE"
        while self.get_mode() != "STABILIZE":
            print("En attente du mode STABILIZE")
            msg = self.vehicle.message_factory.play_tune_encode(0, 0, str.encode("A>A>A"))
            self.vehicle.send_mavlink(msg)
            sleep(1)

        # Attente du mode "AUTO"
        while self.get_mode() != "AUTO":    
            print("En attente du mode AUTO")
            msg = self.vehicle.message_factory.play_tune_encode(0, 0, str.encode("A>A>A"))
            self.vehicle.send_mavlink(msg)
            sleep(1)

        # Passage en mode "GUIDED"    
        self.set_mode("GUIDED")
        # Attente du passage effectif en mode "GUIDED"
        while self.get_mode() != "GUIDED":
            pass



    # Fonction faisant décoler le drone à l'altitude passée en argument
    def takeoff(self, altitude):
        # Ordre de décollage du drone
        self.vehicle.simple_takeoff(altitude)
        # On attend que le drone soit à 95% de l'altitude souhaitée pour sortir de la fonction,
        # car la fonction "simple_takeoff" ne bloque pas le déroulement du programme 
        while self.vehicle.rangefinder.distance < 0.95*altitude:
            pass


    # Décollage du drone jusqu'à la distance fournie en argument
    def arm_and_takeoff(self, aTargetAltitude):
        print("Basic pre-arm checks")
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            sleep(1)
        print("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            sleep(1)

        print("Taking off!")
        print(aTargetAltitude)
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while self.vehicle.rangefinder.distance <aTargetAltitude*0.9:
            print(" Altitude: ", self.vehicle.rangefinder.distance)
            sleep(1)
        print("Reached target altitude")


    def goto(self, targetLocation, distanceAccuracy):
        # Simple goto DroneKit function
        self.vehicle.simple_goto(targetLocation)

        # Stop action if we are no longer in GUIDED mode
        while self.vehicle.mode.name == "GUIDED": 
            remainingDistance = get_distance_metres(self.vehicle.location.global_relative_frame, targetLocation)
            print("Distance to the GPS target: %.2fm" % remainingDistance)

            # If the distance to the target verifies the distance accuracy
            if remainingDistance <= distanceAccuracy:
                print("[mission] Reached GPS target!")
                break  # Then break the waiting loop
            sleep(1)
                  
   
    # Définition de la consigne de vitesse selon le repère x,y,z du drone
    def set_velocity(self, velocity_x, velocity_y, velocity_z):
        # create the SET_POSITION_TARGET_LOCAL_NED command
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
          0,  # time_boot_ms (not used)
          0, 0,  # target system, target component
          mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
          0x0DC7,  # type_mask (ignore pos | ignore acc)
          0, 0, 0,  # x, y, z positions (not used)
          velocity_x, velocity_y, velocity_z,
          # x, y, z velocity in m/s -- X positive forward or North/ Y positive right or East / Z positive down
          0, 0, 0,  # x, y, z acceleration (not used)
          0, 0)  # yaw, yaw_rate (not used)
        # Envoie de la consigne de vitesse au drone 
        self.vehicle.send_mavlink(msg)


    # Fonction permettant de choisir le yaw du drone
    def set_yaw(self, heading, relative=False):
        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            heading,    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            1,          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
        # send command to vehicle
        self.vehicle.send_mavlink(msg)


#-------------------------------------------------------------------------------------------------------------------------------------------
# Asservissements
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction prenant en entrée les coordonnées en x et y de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_suivi_vehicule(self, aruco_center_x, aruco_center_y, altitude):

        # Distance en mètre entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = (self.camera.x_imageCenter - aruco_center_x) #* altitude * self.camera.dist_coeff_x
        erreurY = (self.camera.y_imageCenter - aruco_center_y) #* altitude * self.camera.dist_coeff_y
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurEst = - dist_center * cos(alpha)
        erreurNord = dist_center * sin(alpha)

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        erreurDeriveeEst = (erreurEst - self.erreurAnterieureEst_suivi_vehicule)
        erreurDeriveeNord = (erreurNord - self.erreurAnterieureNord_suivi_vehicule)
        # Erreur intégrale
        self.erreurIntegraleEst_suivi_vehicule += erreurEst
        self.erreurIntegraleNord_suivi_vehicule += erreurNord
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureEst_suivi_vehicule = erreurEst
        self.erreurAnterieureNord_suivi_vehicule = erreurNord

        # Calcul de la vitesse corrigée 
        vitesseEst = self.kp_suivi_vehicule * erreurEst + self.kd_suivi_vehicule * erreurDeriveeEst + self.ki_suivi_vehicule * self.erreurIntegraleEst_suivi_vehicule
        vitesseNord = self.kp_suivi_vehicule * erreurNord + self.kd_suivi_vehicule * erreurDeriveeNord + self.ki_suivi_vehicule * self.erreurIntegraleNord_suivi_vehicule        
        # Renvoie des valeurs
        return vitesseEst, vitesseNord


    # Fonction prenant en entrée les coordonnées en x et y, en pixels, de l'aruco détecté par la cameré 
    # et calcule la vitesse du drone permettant de s'en rapprocher par asservissement PID
    def asservissement_atterrissage(self, aruco_center_x, aruco_center_y):

        # Si l'aruco n'est pas détecté, on l'affiche et on quitte la fonction
        if aruco_center_x == None:
            self.set_velocity(0, 0, 0.2) #sens z positif -> vers le sol
            return None, None, None, None

        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance        
        # Calcul de la valeur du coefficient du correcteur P en fonction de l'altitude du drone       
        self.kp_atterrissage = 0.005 if altitude < 5 else 0.008
        self.kp_atterrissage *= self.coefficient_kp_atterrissage

        # Distance en pixel entre le centre de l'aruco trouvé et le centre de la caméra selon les axes x et y de la camera
        erreurX = self.camera.x_imageCenter - aruco_center_x
        erreurY = self.camera.y_imageCenter - aruco_center_y
        # Passage en coordonnées cylindriques avec comme origine le centre de la caméra
        dist_center = sqrt(erreurX**2+erreurY**2)
        dist_angle = atan2(erreurY, erreurX)
        # Rotation de la base pour correspondre au repère du drone
        alpha = dist_angle + self.vehicle.attitude.yaw
        erreurEst = dist_center * cos(alpha)
        erreurNord = dist_center * sin(alpha)

        # Calcul des erreurs intégrale et dérivée
        # Erreur dérivée 
        erreurDeriveeEst = (erreurEst - self.erreurAnterieureEst_atterrissage)
        erreurDeriveeNord = (erreurNord - self.erreurAnterieureNord_atterrissage)
        # Erreur intégrale
        self.erreurIntegraleEst_atterrissage += erreurEst
        self.erreurIntegraleNord_atterrissage += erreurNord
        # Stockage des erreurs en X et Y pour le future calcul de l'erreur dérivée 
        self.erreurAnterieureEst_atterrissage = erreurEst
        self.erreurAnterieureNord_atterrissage = erreurNord

        # Calcul de la vitesse corrigée 
        vEst = self.kp_atterrissage * erreurEst + self.kd_atterrissage * erreurDeriveeEst + self.ki_atterrissage * self.erreurIntegraleEst_atterrissage
        vNord = self.kp_atterrissage * erreurNord + self.kd_atterrissage * erreurDeriveeNord + self.ki_atterrissage * self.erreurIntegraleNord_atterrissage        
        # Bornage des vitesses à +/- 5 m/s
        vEst = -min(max(vEst, -5.0), 5.0) # Inversion de signe pour que ça marche
        vNord = min(max(vNord, -5.0), 5.0)
        
        # Calcul de la distance planaire à partir de laquelle on considère que le drone est au-dessus du drone 
        dist_center_threshold = 50 if altitude < 2 else 1000        
        # Si n'est dans un rayon d'un mètre autour du drone, il ne change pas son altitude 
        if dist_center > dist_center_threshold :
            vz = 0
        # Sinon on le fait se rapprocher du sol avec une vitesse variant en fonction de l'altitude du drone
        else:
        #Choix de la vitesse verticale en fonction de l'altitude
            if altitude > 8:
                vz = 1.5 
            elif altitude > 5:
                vz = 1
            elif altitude > 1:
                vz = 0.5
            else:
                vz = 0
        
        #Envoie de la consigne de vitesse au drone
        print("Consigne en vitesse : VEst = " + str(vEst) + " ; VNord = " + str(vNord) + " ; VZ = " + str(vz))
        self.set_velocity(vNord, vEst, vz)  # Pour le sense de la camera, X controle le 'east' et Y controle le 'North'
        return erreurX, erreurY, vEst, vNord             

        
#-------------------------------------------------------------------------------------------------------------------------------------------
# Atterissage Aruco
# ------------------------------------------------------------------------------------------------------------------------------------------
        
    # Fonction permettant au drone de réaliser un atterrissage de précision sur un aruco marker
    # Pour s'assurer de détecter l'aruco marker, on fait descendre le drone à une altitude de 7 mètres
    # Une fois cette atlitude atteinte, on asservit la position du drone par rapport au centre de l'aruco.
    # Simultanément à l'asserivessement en position, on fait descendre le drone jusqu'à 1.5 mètre.
    # Une fois cette atltitude atteinte, il atterrira en passant en "LAND"
    # Dans le cas où le drone perdrait l'aruco marker, on le force à passer en "LAND" si la manoeuvre d'asservissement a été lancée depuis plus de 30 secondes
    def atterrissage_aruco(self, chemin_dossier=""):
        
        # Récupération de l'altitude du drone
        altitude = self.vehicle.rangefinder.distance
        # Début du chronomètre
        start_time = time.time()
        
        # Tant que le drone n'est pas à 2 m du sol ou que le temps écoulé est inférieur à 30 secondes, 
        # on lance l'asservissement du drone
        while altitude > 1 and (time.time()-start_time) < 30:
            
            # Récupération de l'altitude du drone
            altitude = self.vehicle.rangefinder.distance
            print(datetime.now().strftime("%d-%m %H:%M:%S") + " : Altitude = " + str(altitude))
            
            # Si le robot est à plus de 10 mètres du sol on le fait descendre
            if altitude > 7.5:
                print("Descente du drone")
                self.set_velocity(0, 0, 1) #sens z positif -> vers le sol
                continue
                        
            # Si le robot est à moins de 7.5 mètres on détecte directement l'aruco et on récupère les coordonnées de son centre            
            else:  
                centre_aruco_X, centre_aruco_Y, _, image = self.camera.detection_aruco(True)
                print("Aruco détecté" if centre_aruco_X != None else "Aruco non détecté")
                # Asservissement par rapport au centre de l'aruco
                erreurX, erreurY, vx, vy = self.asservissement_atterrissage(centre_aruco_X, centre_aruco_Y)
                if chemin_dossier != "":
                    # Affichage de l'erreur et de la vitesse
                    image = cv2.putText(image, "Erreur : EX = " + str(erreurX) + " ; EY = " + str(erreurY), (0, 25), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
                    image = cv2.putText(image, "Vitesse : Vx = " + str(vx) + " ; Vy = " + str(vy), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)
                    # Traçage d'un cercle au centre de l'image
                    cv2.circle(image, (self.camera.x_imageCenter, self.camera.y_imageCenter), 4, (0, 255, 0), -1)
                    # Sauvegarde de la photo
                    enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if centre_aruco_X != None else "no")
            
            
        # Une fois que le robot est assez bas, on le fait atterrir
        print("Atterrissage")
        self.set_mode("LAND")


#-------------------------------------------------------------------------------------------------------------------------------------------
# Suivi de vehicule
# ------------------------------------------------------------------------------------------------------------------------------------------

    # Fonction permettant d'accomplir la mission de suivi de véhicule.
    # Cette fonction prend en paramètre le chemin vers le dossier permettant de stocker les logs
    # On commence par détecter l'aruco d'ID 700. 
    # Une fois cet aruco trouvé, on initialise un tracker avec un carré de même centre que celui de l'aruco, ce qui correspond à la voiture à suivre
    # Une fois le trackeur initialisé, on tracke la voiture pour en trouver son centre. 
    # On asservit ensuite la vitesse du drone pour que le centre de la caméra et de la voiture coïncident
    # En plus de cette vitesse, on additionne la vitesse du drone pondérée pour prendre en compte la vitesse de la voiture. 
    def suivi_vehicule(self, chemin_dossier=""):

        # Initialisation des variables du drone
        tracker = cv2.TrackerCSRT_create()
        bbox = (0,0,0,0)
        taille_carré = 75
        
        # Boucle infinie servant à chercher l'aruco pour initialiser le tracker
        while True:
            # Détection de l'aruco
            aruco_center_x, aruco_center_y, id, image = self.camera.detection_aruco(True)
            # Si un aruco a été détecté et que son ID est 700, on intialise le tracker grace au centre trouvé et on sort de la bouche
            if aruco_center_x != None and id == 700: #WATCH OUT DESACTIVATION VERIF ID ARUCO!!!
                # Détermination de la zone d'intérêt : un carré de même centre que celui de l'aruco et de côté "taille_carré"
                bbox = (aruco_center_x-int(taille_carré/2),aruco_center_y-int(taille_carré/2), taille_carré, taille_carré)
                # Initialisation du tracker
                tracker.init(image, bbox)
                # Sortie de boucle
                break
        print("Aruco détecté")
        
        z,image=self.camera.calcul_radian_aruco(self)
        #Final step : enregistrement photo
        enregistrement_photo_date_position(self, image, chemin_dossier)
        self.setyaw(degrees(-z))
        sleep(5)

        # Boucle infinie asservisement le drone par rapport au centre de l'objet tracké
        while True:

            # Prise de photo
            image = self.camera.prise_photo()
            # Tracking 
            ok, bbox = tracker.update(image)
            # Initialisation des vitesses
            vitesseEst = 0
            vitesseNord = 0

                            
            # Si on détecte on calcule la vitesse du drone par rapport au centre de l'objet tracké et à la vitesse du drone
            if ok:
                
                # Calcul du centre de la voiture en x et en y
                car_center_x = int(bbox[0]+bbox[2]/2)
                car_center_y = int(bbox[1]+bbox[3]/2)
                altitude = self.vehicle.rangefinder.distance

                ### Calcul de la vitesse totale
                # Calcul de la vitesse de la voiture
                #vitesseVoitureEst, vitesseVoitureNord, carYaw = self.get_car_speed(car_center_x, car_center_y)
                #vitesseDroneEst, vitesseDroneNord = self.calcul_vitesse_drone()
                # Rotation du drone pour être dans le même sens que la voiture
                #self.set_yaw(degrees(carYaw+pi/2))
                # Asservissement par rapport au centre de l'objet tracké
                vitesseAsservEst, vitesseAsservNord = self.asservissement_suivi_vehicule(car_center_x, car_center_y, altitude)
                # Ajout et pondération des vitesses
                vitesseEst = vitesseAsservEst #+ vitesseDroneEst #* (1.0 if vitesseAsservEst*vitesseDroneEst > 0 else 4.0)
                vitesseNord = vitesseAsservNord #+ vitesseDroneNord #* (1.0 if vitesseAsservNord*vitesseDroneNord > 0 else 4.0)
                
                # Remise à zéro du compteur de non détection
                self.compteur_non_detection = 0
                # Stockage des vitesse
                self.stored_vEst = vitesseEst
                self.stored_vNord = vitesseNord

                # Affichage des vitesses
                #print("Vitesse vEst : " + str(vitesseAsservEst) + " ; vvEst = " + str(vitesseDroneEst) + " ; vNord = " + str(vitesseAsservNord) + " ; vvNord = " + str(vitesseDroneNord))

            # Si on ne détécte pas on adapte la vitesse en fonction du nombre d'images où on ne détecte pas
            else:
                # Incrément du compteur
                self.compteur_non_detection += 1
                # Si le compteur est inférieur à 5, on envoie les vitesses obtenues à la dernière détection
                if self.compteur_non_detection < 5:
                    vitesseEst = self.stored_vEst
                    vitesseNord = self.stored_vNord
                # Si le compteur est supérieur à 5, on laisse les vitesses nulles


            #Envoie de la consigne de vitesse au drone
            self.set_velocity(vitesseNord, vitesseEst, 0)
            # Affichage des consignes sur le terminal
            print("Consigne en vitesse : VEst = " + str(vitesseEst) + " ; VNord = " + str(vitesseNord))     
            print(self.vehicle.rangefinder.distance)

            if chemin_dossier != "" and self.get_mode == "GUIDED":
                # On entoure l'objet tracké sur la photo
                cv2.rectangle(image, (int(bbox[0]), int(bbox[1])), (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3])), (0, 0, 255), 2, 2)  
                # Affichage de la vitesse sur la photo
                image = cv2.putText(image, "VEst = " + str(vitesseEst) + " ; VNord = " + str(vitesseNord), (0, 50), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 0), 2)              
                # Traçage d'un cercle au centre de l'image
                cv2.circle(image, (self.camera.x_imageCenter, self.camera.y_imageCenter), 4, (0, 255, 0), -1)
                # Sauvegarde de la photo
                enregistrement_photo_date_position(self, image, chemin_dossier, "yes" if ok else "no")
       




    # Fonction permettant calculer la vitesse de la voiture à tracker 
    def get_car_speed(self, car_center_x, car_center_y):
        # Récupération des coordonnées de la voiture et du temps actuel
        currentMeasuredTime = time.time()
        currentCarPosition = get_GPS_through_picture(self, car_center_x, car_center_y)
        # Initialisation des vitesses
        vitesseVoitureEst = 0
        vitesseVoitureNord = 0
        angle = self.vehicle.attitude.yaw
        # Si l'on dispose d'une position antérieure pour effectuer le calcul de la position 
        if self.previousCarPosition != None:
            # Calcul de la vitesse de la voitrue en calculant la distance entre la position actuelle et précédente de la voiture
            carSpeed = get_distance_metres(currentCarPosition, self.previousCarPosition) / (currentMeasuredTime - self.previousMeasuredTime)
            # Récupération de l'ange entre le 
            angle = atan2(currentCarPosition.lat - self.previousCarPosition.lat, currentCarPosition.lon - self.previousCarPosition.lon)
            # Dissociation de la vitesse de la voiture selon une composante "Nord" et une composante "Est"
            vitesseVoitureEst = carSpeed * cos(angle)
            vitesseVoitureNord = carSpeed * sin(angle)
        # Raffraichissement des valeurs des précédentes coordonnées GPS et de temps
        self.previousCarPosition = currentCarPosition
        self.previousMeasuredTime = currentMeasuredTime
        # Retour des valeurs de vitesses
        return vitesseVoitureEst, vitesseVoitureNord, angle


    def calcul_vitesse_drone(self):
        vitesseEst = 0
        vitesseNord = 0
        if self.previousLatitude != 0:
            groundSpeed = self.vehicle.groundspeed
            angle = atan2(self.vehicle.location._lat - self.previousLatitude, self.vehicle.location._lon - self.previousLongitude)
            vitesseEst = groundSpeed * cos(angle)
            vitesseNord = groundSpeed * sin(angle)         
        self.previousLatitude = self.vehicle.location._lat
        self.previousLongitude = self.vehicle.location._lon
        return vitesseEst, vitesseNord
