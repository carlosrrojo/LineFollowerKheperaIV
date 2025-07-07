from controller import Robot  # Módulo de Webots para el control el robot.
from controller import Camera  # Módulo de Webots para el control de la cámara.

import time  # Si queremos utilizar time.sleep().
import numpy as np  # Si queremos utilizar numpy para procesar la imagen.
import cv2  # Si queremos utilizar OpenCV para procesar la imagen.

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Velocidad por defecto para este comportamiento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32

UMBRAL = 200

SLEEP = 0.05

GAMMA = 0.5

VISITAS = np.zeros((3,3),np.uint8)

CHANCE = 1

def enable_sensors(robot, timeStep):
    """
    Obtener y activar los sensores del robot.

    Return: lista con los sensores activados.
    """
    
    sensors = {
        "rear left infrared sensor":robot.getDevice("rear left infrared sensor"),
        "left infrared sensor":robot.getDevice("left infrared sensor"),
        "front left infrared sensor":robot.getDevice("front left infrared sensor"),
        "front infrared sensor":robot.getDevice("front infrared sensor"),
        "front right infrared sensor":robot.getDevice("front right infrared sensor"),
        "right infrared sensor":robot.getDevice("right infrared sensor"),
        "rear right infrared sensor":robot.getDevice("rear right infrared sensor"),
        "rear infrared sensor":robot.getDevice("rear infrared sensor"),
        "ground left infrared sensor":robot.getDevice("ground left infrared sensor"),
        "ground front left infrared sensor":robot.getDevice("ground front left infrared sensor"),
        "ground front right infrared sensor":robot.getDevice("ground front right infrared sensor"),
        "ground right infrared sensor":robot.getDevice("ground right infrared sensor")
    }
    
    for sensor in sensors.values():
        sensor.enable(timeStep)

    return sensors


def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.

    timeStep: tiempo (en milisegundos) de actualización por defecto para los sensores/actuadores
      (cada dispositivo puede tener un valor diferente).
    """

    # referencia al robot
    robot = Robot()

    # Obtener dispositivos correspondientes a los motores de las ruedas.
    
    motores = {
        "derecha":robot.getDevice("right wheel motor"),
        "izquierda":robot.getDevice("left wheel motor")
    }

    # Configuración inicial para utilizar movimiento por posición (necesario para odometría).
    # En movimiento por velocidad, establecer posición a infinito (wheel.setPosition(float('inf'))).
    
    for wheel in motores.values():
        wheel.setPosition(float('+inf'))
        wheel.setVelocity(0)

    # Obtener una lista con los sensores infrarrojos ya activados
    sensorList = enable_sensors(robot, timeStep)

    return robot, motores, sensorList

def turn_right( wheels):
    print("Derecha")
    wheels["izquierda"].setVelocity(CRUISE_SPEED*2)
    wheels["derecha"].setVelocity(CRUISE_SPEED)

def turn_left(wheels):
    print("Izquierda")
    wheels["izquierda"].setVelocity(CRUISE_SPEED)
    wheels["derecha"].setVelocity(CRUISE_SPEED*2)


def go_straight(wheels):
    print("Recto")
    wheels["izquierda"].setVelocity(CRUISE_SPEED)
    wheels["derecha"].setVelocity(CRUISE_SPEED)



"""
Programar función que determine el estado actual.
"""
def current_state(sensors):
    if sensors["ground front left infrared sensor"].getValue() > 750 and sensors["ground right infrared sensor"].getValue() < 500:
        return 0
    elif sensors["ground front left infrared sensor"].getValue() > 750 and sensors["ground left infrared sensor"].getValue() < 500:
        return 1
    else:
        return 2


"""
Después de realizar una acción, programar función que establece un valor numérico de
refuerzo (positivo o negativo).
"""
def action_result(antes, ahora):
    if (antes[0] > 750 and ahora[0] < 500) and (ahora[1]):
        return 1
    if antes[1] > 750 and ahora[1] < 500:
        return 1
    return 0


"""
Actualizar la matriz Q (3x3), con la fórmula de entornos no deterministas
state = fila,
action = columna
"""
def update_matrix(matrix, state:int, action:int, state_prima:int, r):
    VISITAS[state, action]+=1
    alphaN=1/(1+VISITAS(state, action)) # Numero de veces que el par stado-accion ha sido visitado

    matrix[state,action] = (1 - alphaN)*matrix[state,action] + alphaN*(r + GAMMA*np.max(matrix[state_prima,:]))
    return matrix


def evitar_paredes(sensors, wheels):
    # simple obstacle avoidance algorithm
    # based on the front infrared sensors
    speed_offset = 0.2 * (MAX_SPEED - 0.03 * sensors["front infrared sensor"].getValue())
    speed_delta = 0.03 * sensors["front left infrared sensor"].getValue() - 0.03 * sensors["front right infrared sensor"].getValue()
    
    wheels["izquierda"].setVelocity(speed_offset + speed_delta)
    wheels["derecha"].setVelocity(speed_offset - speed_delta)


def decision(estado, matrix):
    if np.random.rand() <= CHANCE:
        #accion aleatoria
        return np.random.randint(0, 3)
    return np.max(matrix[estado,:])


def leer_sensores(sensores):
    valores = []
    for i, name in enumerate(sensores):
        if i > 7:
            valores.append(sensores[name].getValue())
    return valores


def realizar_accion(action):
    match action:
        case 0:
            turn_right()
        case 1:
            turn_left()
        case 2:
            go_straight()


def main():
    # Activamos los dispositivos necesarios y obtenemos referencias a ellos.
    robot, wheels, sensorList = init_devices(TIME_STEP)

    robot.step(TIME_STEP)

    matrizQ = np.zeros((3,3), np.float16)

    iterations = 0

    # Tomamos una accion aleatoria
    s = current_state(sensorList)
    a = decision(s, matrizQ)
    while(robot.step(TIME_STEP) != -1):
        if (sensorList["front infrared sensor"].getValue() > UMBRAL or sensorList["front left infrared sensor"].getValue() > UMBRAL or sensorList["front right infrared sensor"].getValue() > UMBRAL):
            evitar_paredes(sensorList, wheels)
        else:
            antes = leer_sensores(sensorList)
            realizar_accion(a)
            despues = leer_sensores(sensorList)
            r = action_result(antes, despues)
            s_prima = current_state(sensorList)
            matrizQ = update_matrix(matrizQ, s, a, s_prima, r)
            s = s_prima
            a = decision(s, matrizQ)
            iterations += 1
        if iterations / 100 < 0:
            CHANCE = CHANCE - 0.1
            

if __name__ == "__main__":
    main()

"""
Hacer exploracion para refinar Q.
"""