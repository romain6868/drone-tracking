import os, sys, traceback
from Detection import *
from utilities import *
from Drone import *
from Logger import *

# Création de l'objet drone
drone = Drone()


# Listerner déclanchant la manoeuvre d'atterissage
@drone.vehicle.on_message('SERVO_OUTPUT_RAW')
def listener(self, name, message):

    # Condition de déclenchement de la manoeuvre d'atterissage
    if int(message.servo10_raw) == 1350:
        
        # Passage et attente en mode "GUIDED"    
        drone.set_mode("GUIDED")
        while drone.get_mode() != "GUIDED":
            pass

        # Atterrissage
        print("Début de la manoeuvre d'atterissage")
        try:
            chemin_dossier = creation_dossier_photo("Atterrissage aruco : " + datetime.now().strftime("%d-%m %H:%M:%S"))
            drone.atterrissage_aruco(chemin_dossier)
        except Exception as e:
            print(e)
        finally:
            sys.exit(0) 


# Choix de la mission
numero_mission = int(input("Quel mission voulez-vous lancer ?\n"+
    "1) Atterissage asservi\n" + 
    "2) Suivi de vehicvule\n"  +
    "3) Arrêt du programme\n"))
while numero_mission not in range(1,6):
    numero_mission = input("Numéro de mission non reconnu. Veuillez resaisir le numéro")


#Atterissage Aruco
if numero_mission == 1:
            print("Début de la manoeuvre d'atterissage")
        try:
            chemin_dossier = creation_dossier_photo("Atterrissage aruco : " + datetime.now().strftime("%d-%m %H:%M:%S"))
            drone.atterrissage_aruco(chemin_dossier)
        except Exception as e:
            print(e)
        # Interruption de la boucle par ctrl+C     
        except KeyboardInterrupt:
            print("Interruption de la cartographie")
        finally:
            sys.exit(0) 
                    
        

# Suivi de véhicule
elif numero_mission == 2:
    with Logger("Suivi de véhicule : " + datetime.now().strftime("%d-%m %H:%M:%S") + ".txt"):
        #altitude = 25
        # Attente du mode stabilize puis du mode auto
        drone.attente_stabilize_auto()
        # Décollage
        #drone.arm_and_takeoff(altitude)
        #Vol vers la zone où se trouvent les mannequins (coordonnées de la compète)
        #drone.goto(LocationGlobalRelative(50.910031, 6.226700, 25), 0.5)
        # Création du dossier recevant les photos
        chemin_dossier = creation_dossier_photo("Suivi de véhicule : " + datetime.now().strftime("%d-%m %H:%M:%S"))
        # Initialisation du suivi de véhicule
        drone.suivi_vehicule(chemin_dossier)


# Arret du programme
elif numero_mission == 3:
    #msg = drone.vehicle.message_factory.play_tune_encode(0, 0, str.encode("FEFGAA#A"))
    #msg = drone.vehicle.message_factory.play_tune_encode(0, 0, str.encode("A>A>A"))
    #drone.vehicle.send_mavlink(msg)
    while True:
      print("Attente atterissageé")
      sleep(1)
    sys.exit(0)
            
