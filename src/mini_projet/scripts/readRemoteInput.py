#! /usr/bin/python3
import socket
import re
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import sys
from mini_projet.srv import SurveillanceService

ADRESSE = '192.168.1.165'
PORT = 6789

move_sensivity_factor = 0.5

standardMsgRegex = "##(.*\r?\n)+##"
propertyRegex = "PROP:\s*(.*)"


def getProperty(p, donnees):
    regex = propertyRegex.replace("PROP", p)
    return str(re.search(regex,donnees).group(1))

def interpretCmd(cmd):
    if("LAND" in cmd):
        print("should trigger land")
        landPub.publish(Empty())
    elif("TAKEOFF"in cmd):
        print("should trigger takeoff")
        takeofPub.publish(Empty())
    elif("SURVEILLANCE"in cmd):
        print("should trigger takeoff")
        surveillancePub.publish(Empty())
    elif("EMERGENCY" in cmd):
        print("should trigger emergency")
        emergencyPub.publish(Empty())
def updateInput(heightInput, rollInput, translateXInput, translateYInput):
    print("sould send input: " + str(heightInput) + " " + str(rollInput) + " " + str(translateXInput) + " " + str(translateYInput))
    t = Twist()
    t.linear.z = heightInput * move_sensivity_factor
    t.linear.x = translateYInput * move_sensivity_factor
    t.linear.y = -translateXInput * move_sensivity_factor
    t.angular.z = -rollInput
    pilotePub.publish(t)


print('starting server')
serveur = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
serveur.bind((ADRESSE, PORT))
serveur.listen(1)
client, adresseClient = serveur.accept()

rospy.init_node("server_pilote")

takeofPub = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
landPub = rospy.Publisher('/bebop/land', Empty, queue_size=10)
emergencyPub = rospy.Publisher('/bebop/reset', Empty, queue_size=10)
pilotePub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size=10)
surveillancePub = rospy.Publisher('/surveillance', Empty, queue_size=10)

is_surveillance = rospy.ServiceProxy('surveillance_service', SurveillanceService)



print('Connexion de ', adresseClient)
while(serveur):
    donnees = client.recv(1024)
    if not donnees:
        print('Erreur de reception.')
        break
    else:
        #print donnees
        strData = donnees.decode()
        if(re.match(standardMsgRegex, strData)):
            #valid data recieved
            dataType = getProperty("type", strData)
            
            if("COMMAND" in dataType):
                cmd = getProperty("command", strData)
                interpretCmd(cmd)
            elif("INPUT" in dataType):
                resp = is_surveillance()
                if(resp.isSurveillance):
                    heightInput =  0
                    rollInput = 1
                    translateXInput = 0
                    translateYInput = 0
                    updateInput(heightInput, rollInput, translateXInput, translateYInput)
                else:
                    heightInput =  float(getProperty("input_height", strData))
                    rollInput = float(getProperty("input_roll", strData))
                    translateXInput = float(getProperty("input_translate_x", strData))
                    translateYInput = float(getProperty("input_translate_y", strData))
                    updateInput(heightInput, rollInput, translateXInput, translateYInput)





print('Fermeture de la connexion avec le client.')
client.close()
print('Arret du serveur.')
serveur.close()