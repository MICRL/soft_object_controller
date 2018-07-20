import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Bool
import tensorflow as tf
from tensorflow import keras
from numpy import sqrt, sign
import numpy as np
from copy import copy

global FPS, M, N, cart_v_real, contorl_gain, MAX_VEL, TOLERANCE, p, received, getFeedbackPoint, moved, startGo, target, MINIMUM_DATA_SIZE, model

def build_model():
    m = keras.Sequential([
        keras.layers.Dense(16, activation=tf.nn.relu, input_shape=(M,)),
        keras.layers.Dense(16, activation=tf.nn.relu),
        keras.layers.Dense(N)
    ])
    optimizer = tf.train.RMSPropOptimizer(0.001)
    m.compile(loss='mse', optimizer=optimizer, metrics=['mae'])
    return m

def fit_model(train_data, train_labels):
    global model
    model.fit(train_data, train_labels, epochs=10, validation_split=0.2, verbose=0)

def norm(x):
    scale = 2.2250738585072014E-308
    y = 0.0
    for k in range(3):
        absxk = abs(x[k])
        if absxk > scale:
            t = scale / absxk
            y = 1.0 + y * t * t
            scale = absxk
        else:
            t = absxk / scale
            y += t * t
    return scale * sqrt(y)

def getFeatures():
    c_p = copy(p)
    if c_p[0] > c_p[3]:
        for i in range(3):
            c_p[i], c_p[i+3] = c_p[i+3], c_p[i]
    return c_p

def highPass(l, threshold):
    return list(0.0 if i < threshold else i for i in l)

def cartVelCallback(msg):
    global cart_v_real, moved, received
    cart_v_real = copy(msg.data)
    cart_v_real = highPass(cart_v_real, 1e-4)
    for i in cart_v_real:
        if not i == 0:
            moved = True
    moved = True
    received = True

def feedbackCallback(msg):
    global p, getFeedbackPoint
    p = copy(msg.data)
    getFeedbackPoint = True

def startCallback(msg):
    global startGo
    startGo = msg.data

def NN_controller():
    global received, moved, getFeedbackPoint, model

    rospy.init_node("soft_object_controller_demo", anonymous=True)
    rospy.Subscriber("/yumi/ikSloverVel_controller/state", Float64MultiArray, cartVelCallback)
    rospy.Subscriber("/soft_object_tracking/centroid", Float64MultiArray, feedbackCallback)
    rospy.Subscriber("/yumi/ikSloverVel_controller/go", Bool, startCallback)
    cartVelPublisher = rospy.Publisher('/yumi/ikSloverVel_controller/command', Float64MultiArray, queue_size=1)
    rate = rospy.Rate(FPS)

    x = x_old = x_v = x_star = xd = y = y_predict = [0] * 6
    train_data = None
    label = None

    first_time = True
    stop_flag = False
    count = 0
    model = None
    minimum_data_requirement = False
    while not rospy.is_shutdown():
        if stop_flag:
            if startGo:
                count += 1
                cartVelPublisher.publish((0,)*M)
            print "Reached the target SUCCESSFULLY"
            # # Only for hole-task
            # if count*1.0/FPS > 0.5 and count*1.0/FPS < 1.2:
            #     armCommand = [0] * 6
            #     armCommand[2] = armCommand[5] = -0.06
            #     cartVelPublisher.publish(tuple(armCommand))
            # elif count*1.0/FPS >= 1.2 and count*1.0/FPS < 2:
            #     cartVelPublisher.publish((0,)*M)
            # elif count*1.0/FPS >= 2:
            #     break
            rate.sleep()
            # continue
            break

        if received and getFeedbackPoint:
            x_old = copy(x)
            x = getFeatures()
            x_v = list((x[i] - x_old[i]) * FPS for i in range(N))
            x_star = list(target[i] - x[i] for i in range(N))
            y = copy(cart_v_real)
            # y = highPass(y, 1e-4)
            if moved:
                if first_time:
                    model = build_model()
                    train_data = np.array([x_star])
                    label = np.array([x_star])
                    first_time = False
                else:
                    train_data = np.concatenate((train_data, np.asarray([x_v])))
                    label = np.concatenate((label, np.asarray([y])))
                    fit_model(train_data, label)
                    print train_data.shape
                    if train_data.shape[0] > MINIMUM_DATA_SIZE:
                        minimum_data_requirement = True
                        y_predict = model.predict(np.array([x_star])).flatten()
            received = False
            getFeedbackPoint = False
            moved = False

            stopFlag = True
            print "deltX:",
            for i in range(N):
                if abs(target[i] - x[i]) > TOLERANCE:
                    stopFlag = False
                print target[i] - x[i],
            print

            if not minimum_data_requirement:
                print "Need more data"
            else:
                print "Velocity: ",
                armCommand = list(i*contorl_gain if abs(i*contorl_gain) < MAX_VEL else MAX_VEL*sign(i) for i in y_predict)
                print armCommand

                if startGo:
                    cartVelPublisher.publish(tuple(armCommand))
                    print "sended velocity command"
            print "-----------------------"
            print

        else:
            print "Didn't receive the cart_v_real or get feedback points"

        rate.sleep()

if __name__ == "__main__":
    global FPS, M, N, cart_v_real, contorl_gain, MAX_VEL, TOLERANCE, p, received, getFeedbackPoint, moved, startGo, target, MINIMUM_DATA_SIZE
    FPS = 10
    N = 6
    M = 6
    cart_v_real = []
    contorl_gain = 1
    MAX_VEL = 0.06
    TOLERANCE = 0.006
    p = []
    startGo = received = getFeedbackPoint = moved = False
    target = [-0.06378774717450142, -0.03290320187807083, 0.4924301207065582, 0.017935208044946194, -0.03451043087989092, 0.49555233120918274]
    MINIMUM_DATA_SIZE = 50

    try:
        NN_controller()
    except rospy.ROSInterruptException:
        pass
