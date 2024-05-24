#!/usr/bin/env python3

# Importar las librerías necesarias
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,  CvBridgeError
import cv2
import mediapipe as mp
import ackermann_msgs.msg

# TODO Declarar el detector de pose de mediapipe a utilizar
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
# Publicador de mensajes de control
ackermann_command_publisher = None
# current_speed = 0.0
# current_steering_angle = 0.0

#Procesar la imagen del operador
def image_callback(msg):
    global ackermann_command_publisher#, current_speed, current_steering_angle
    bridge = CvBridge()
    try:
        # Convertir la imagen de ROS a una imagen de OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # TODO Procesar la imagen con MediaPipe
    results = hands.process(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # TODO Dibujar los landsmarks sobre la imagen
            mp.solutions.drawing_utils.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            # TODO Reconocer el gesto mediante alguna clasificación a partir de los landmarks
            gesture = recognize_gesture(hand_landmarks)

           # TODO Interpretar el gesto obtenido y enviar la orden de control ackermann
            if gesture == "move_forward":
                send_ackermann_command(speed=0.6, steering_angle=0.0)
            elif gesture == "turn_left":
                send_ackermann_command(speed=0.3, steering_angle=0.5)
            elif gesture == "turn_right":
                send_ackermann_command(speed=0.3, steering_angle=-0.5)

    # Mostrar la imagen con los landmarks/gestos detectados
    cv2.imshow("Hand pose Estimation", cv_image)
    cv2.waitKey(1)
    
def recognize_gesture(hand_landmarks):
    # Obtener los landmarks específicos
    index_tip = hand_landmarks.landmark[8]
    index_base = hand_landmarks.landmark[5]
    
    middle_tip = hand_landmarks.landmark[12]
    middle_base = hand_landmarks.landmark[9]
    
    ring_tip = hand_landmarks.landmark[16]
    ring_base = hand_landmarks.landmark[13]
    
    pinky_tip = hand_landmarks.landmark[20]
    pinky_base = hand_landmarks.landmark[17]


    # Verificar puño cerrado (move_forward)
    if (index_tip.y > index_base.y and
        middle_tip.y > middle_base.y and
        ring_tip.y > ring_base.y and
        pinky_tip.y > pinky_base.y):
        return "move_forward"

    # Verificar dedo indice hacia arriba (turn_left)
    elif (index_tip.y < index_base.y and
        middle_tip.y > middle_base.y and
        ring_tip.y > ring_base.y):
        return "turn_left"

    # Verificar mano abierta (turn_right)
    elif (index_tip.y < index_base.y and
          middle_tip.y < middle_base.y and
          ring_tip.y < ring_base.y and
          pinky_tip.y < pinky_base.y):
        return "turn_right"
    # Gesto desconocido
    else:
        return "unknown"
       
def send_ackermann_command(speed, steering_angle):
    global ackermann_command_publisher
    if ackermann_command_publisher is not None:
        ackermann_cmd = ackermann_msgs.msg.AckermannDrive()
        ackermann_cmd.speed = speed
        ackermann_cmd.steering_angle = steering_angle
        ackermann_command_publisher.publish(ackermann_cmd)
        print(f"Published command: speed={speed}, steering_angle={steering_angle}")  # Mensaje de depuración
    else:
        print("ackermann_command_publisher is not initialized")

def main():
    global ackermann_command_publisher
    rospy.init_node('pose_estimation', anonymous=True)
    rospy.Subscriber("/operator/image", Image, image_callback)

    ## Publisher definition
    ackermann_command_publisher = rospy.Publisher(
            "/blue/preorder_ackermann_cmd",
            ackermann_msgs.msg.AckermannDrive,
            queue_size=10,
        )

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()



# def recognize_speed_gesture(hand_landmarks):
#     # Obtener los landmarks específicos
#     index_tip = hand_landmarks.landmark[8]
#     index_base = hand_landmarks.landmark[5]

#     middle_tip = hand_landmarks.landmark[12]
#     middle_base = hand_landmarks.landmark[9]

#     ring_tip = hand_landmarks.landmark[16]
#     ring_base = hand_landmarks.landmark[13]

#     # Verificar velocidad baja (solo índice levantado)
#     if (index_tip.y < index_base.y and
#         middle_tip.y > middle_base.y and
#         ring_tip.y > ring_base.y):
#         return "low_speed"

#     # Verificar velocidad media (índice y medio levantados)
#     elif (index_tip.y < index_base.y and
#           middle_tip.y < middle_base.y and
#           ring_tip.y > ring_base.y):
#         return "medium_speed"

#     # Verificar velocidad alta (índice, medio y anular levantados)
#     elif (index_tip.y < index_base.y and
#           middle_tip.y < middle_base.y and
#           ring_tip.y < ring_base.y):
#         return "high_speed"

#     # Gesto desconocido
#     else:
#         return "unknown"
# if results.multi_hand_landmarks:
    #     if len(results.multi_hand_landmarks) > 1:
    #         right_hand_landmarks = results.multi_hand_landmarks[0]
    #         left_hand_landmarks = results.multi_hand_landmarks[1]
    #         # Dibujar los landmarks sobre la imagen
    #         mp.solutions.drawing_utils.draw_landmarks(cv_image, right_hand_landmarks, mp_hands.HAND_CONNECTIONS)
    #         mp.solutions.drawing_utils.draw_landmarks(cv_image, left_hand_landmarks, mp_hands.HAND_CONNECTIONS)

    #         # Reconocer el gesto de la mano derecha para dirección
    #         direction_gesture = recognize_gesture(right_hand_landmarks)
    #         # Reconocer el gesto de la mano izquierda para velocidad
    #         speed_gesture = recognize_speed_gesture(left_hand_landmarks)

    #         # Interpretar los gestos obtenidos y enviar la orden de control ackermann
    #         if direction_gesture == "move_forward":
    #             current_steering_angle = 0.0
    #         elif direction_gesture == "turn_left":
    #             current_steering_angle = 0.5
    #         elif direction_gesture == "turn_right":
    #             current_steering_angle = -0.5

    #         if speed_gesture == "low_speed":
    #             current_speed = 0.5
    #         elif speed_gesture == "medium_speed":
    #             current_speed = 1.0
    #         elif speed_gesture == "high_speed":
    #             current_speed = 1.5

    #         send_ackermann_command(current_speed, current_steering_angle)
    #     else:
    #         hand_landmarks = results.multi_hand_landmarks[0]
    #         mp.solutions.drawing_utils.draw_landmarks(cv_image, hand_landmarks, mp_hands.HAND_CONNECTIONS)
    #         direction_gesture = recognize_gesture(hand_landmarks)
    #         if direction_gesture == "move_forward":
    #             send_ackermann_command(speed=1.0, steering_angle=0.0)
    #         elif direction_gesture == "turn_left":
    #             send_ackermann_command(speed=0.5, steering_angle=0.5)
    #         elif direction_gesture == "turn_right":
    #             send_ackermann_command(speed=0.5, steering_angle=-0.5)