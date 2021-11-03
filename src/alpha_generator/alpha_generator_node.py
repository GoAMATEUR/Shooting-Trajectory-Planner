#!/usr/bin/env python
import numpy as np
import numba
import argparse
import rospy
from trajectory_generator.msg import coeff_msgs
from trajectory_generator.msg import alpha_msgs

@numba.jit
def single_predict(alpha, h, vx, vy, dt=0.001):
    v0 = 3.0
    gamma = 0.25
    D = 0.03
    m = 0.1
    g = 9.81
    x = 0
    y = h
    x1 = vx + v0 * np.cos(alpha)
    y1 = vy + v0 * np.sin(alpha)
    while y > 0:
        x += x1 * dt
        y += y1 * dt
        v = np.sqrt(x1 ** 2 + y1 ** 2)
        k = gamma * D * D / m * v
        x2 = -k * x1
        y2 = -g - k * y1
        x1 += x2 * dt
        y1 += y2 * dt
    return x

@numba.jit
def find_alpha(s, h, vx, vy, l, r, descending, err=1e-2):
    alpha = (l + r) / 2
    while True:
        s_pred = single_predict(alpha, h, vx, vy)
        if abs(s_pred - s) <= err:
            return alpha
        elif s_pred > s:
            if descending:
                l, alpha = alpha, (alpha + r) / 2
            else:
                r, alpha = alpha, (l + alpha) / 2
        else:
            if descending:
                r, alpha = alpha, (l + alpha) / 2
            else:
                l, alpha = alpha, (alpha + r) / 2

@numba.jit
def find_alpha_3(s, h, vx, vy, err=1e-2 , max_itr = 20):
    l, r, mid = -np.pi / 2, np.pi / 2, 0
    # pred = partial(single_predict, h=h, vx=vx, vy=vy)
    step = np.pi / 360
    count = 0
    while True:
        #print(1)
        if single_predict(mid, h, vx, vy) < single_predict(mid + step, h, vx, vy):
            l, mid = mid, (mid + r) / 2
        elif single_predict(mid, h, vx, vy) < single_predict(mid - step, h, vx, vy):
            r, mid = mid, (l + mid) / 2
        else:
            break
        count+=1
        if count > max_itr:
            return None, None
        
    
    l, r = -np.pi / 2, np.pi / 2
    if single_predict(mid, h, vx, vy) < s:
        return None, None
    else:
        if single_predict(l, h, vx, vy) > s:
            s1 = None
        else:
            s1 = find_alpha(s, h, vx, vy, -np.pi / 2, mid, descending=False, err=err)
        if single_predict(r, h, vx, vy) > s:
            s2 = None
        else:
            s2 = find_alpha(s, h, vx, vy, mid, np.pi / 2, descending=True, err=err)
        return s1, s2

'''
def main(args):
    alpha = find_alpha_3(args.s, args.h, args.vx, args.vy, args.err)
    print(alpha)
    return alpha
'''

#Publish Alpha Array
alphaPub = rospy.Publisher('/trajectory_generator_node/alpha', alpha_msgs, queue_size=2000)

def alphaPublisher(msg):
    #rate = rospy.Rate(50)
    global alphaPub
    alphaPub.publish(msg)
    #print("[INFO] Alphas Generated.")
    #rate.sleep()


def coeffInfoParser(msg):
    coeff_num = msg.coeff_num
    _target = msg.target
    segment = msg.segment
    _coeff = np.array(msg.coeff, dtype=float)
    _coeff = _coeff.reshape(int(_coeff.size/segment), segment).T 
    _time = np.array(msg.time, dtype=float)
    #print(_coeff)
    return _coeff[:,0:coeff_num],_coeff[:,coeff_num:],_time,_target
      

def polyPosVel(_coeff_x, k, t, coeff_num=8):
    time = np.array([t**i for i in range(coeff_num)],dtype=float)
    pos = _coeff_x[k].dot(time)
    _vel_coeff = _coeff_x[k][1:]
    time = np.array([(i+1)*t**i for i in range(coeff_num-1)],dtype=float)
    vel = _vel_coeff.dot(time)
    return pos, vel

def coeffInfoCallback(msg):
    print("AG CALL")
    _coeff_x,_coeff_z, _time, _target = coeffInfoParser(msg)
    alphas = list()
    count = 0
    #print(_time)
    cont = True
    for k in range(_time.size):
        t = 0.
        if not cont:
            break
        while t <= _time[k]:
        
            pos_x, vel_x =  polyPosVel(_coeff_x,k,t)
            if pos_x > _target:
                cont = False
                break
            pos_z, vel_z = polyPosVel(_coeff_z, k,t)
            print(_target - pos_x, pos_z, vel_x, vel_z)
            alpha1, alpha2 = find_alpha_3(_target - pos_x, pos_z, vel_x, vel_z)
            
            count += 2
            alpha1 = alpha1 if alpha1 else 114514.
            alpha2 = alpha2 if alpha2 else 114514.
            alphas.extend([alpha1, alpha2])
            #print(_target - pos_x, pos_z, vel_x, vel_z, alpha1, alpha2)
            # with open("/home/hsy/alpha.txt", "a") as f:
            #     f.write("{} {} {} {} {} {}\n".format(pos_x, pos_z, vel_x, vel_z, alpha1, alpha2))
            t += 0.05
    alphaMsg = alpha_msgs()
    alphaMsg.sample_num = count
    alphaMsg.alphas = alphas
    #print(alphaMsg.alphas)
    print("[INFO] Alphas: ", len(alphaMsg.alphas))
    alphaPublisher(alphaMsg)
    #Sample
    
def main():   
    rospy.init_node('alpha_generator', anonymous=False)
    rospy.Subscriber("/trajectory_generator_node/coeff", coeff_msgs, coeffInfoCallback)
    rospy.spin()
    

if __name__ == "__main__":
    a = find_alpha_3(2.5-0.670392 , 0.735811 , 0.767273 , 0.674149)
    print(a)
    dx = single_predict(a[0], 0.735811 , 0.767273 , 0.674149)
    x = 0.670392+dx
    print(2.5 - x)
    '''
    parser = argparse.ArgumentParser()
    parser.add_argument('--h',
                        help='initial height',
                        type=float)
    parser.add_argument('--s',
                        help='shooting distance',
                        type=float)
    parser.add_argument('--vx',
                        help='drone velocity in x',
                        type=float)
    parser.add_argument('--vy',
                        help='drone velocity in y',
                        type=float)
    parser.add_argument('--err',
                        help='err in distance control',
                        default=1e-2,
                        type=float)
    args = parser.parse_args()
    main(args)
   '''
