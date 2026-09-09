import numpy as np
from os import listdir
from os.path import isfile, join
import cv2 as cv
import sys

def chronometre(func):
    def wrapper(self, *args, **kwargs):
        start = cv.getTickCount()
        result = func(self,*args, **kwargs)
        end = cv.getTickCount()
        t = (end - start) /cv.getTickFrequency() 
        print(t)
    return wrapper

def analyse_line_fast(gray, canny, v):
    w = 0.05
    wd = w * 0.2
    beta_x = 0.11499
    beta_y = 35.68
    u0 = 312.0
    vh = 116.18

    edges = canny[v:v+1, :].ravel()
    color = gray[v:v+1, :].ravel()

    bord = np.where(edges == 255)[0]
    if bord.shape[0]<3: return -1 # we want to have at least 4 edges.
    bord = bord[np.ediff1d(bord,to_end=20) > 9] # keeps only the large difference
    delta = np.ediff1d(bord)
    mean_color=np.empty(len(bord)-1,dtype=int)
    for idx,b in enumerate(bord[:-1]):
        mean_color[idx]=int(np.mean(color[b:bord[idx+1]]))

    for idx in range(1,len(mean_color)-1):
        if mean_color[idx] > 200 and mean_color[idx-1]<80 and mean_color[idx+1]<80 and abs(delta[idx]-delta[idx-1])<delta[idx]/2 and abs(delta[idx]-delta[idx+1])<delta[idx]/2:  
            u=(bord[idx]+bord[idx+1]) // 2
            x = beta_x * (u -u0) / (vh-v)
            y = beta_y / (vh-v)
            return u
    return -1

def analyse_line(gray, canny, h):
    global img
    w = 0.05
    wd = w * 0.2
    beta_x = 0.11499
    beta_y = 35.68
    u0 = 312.0
    vh = 116.18

    """
    eq = cv.equalizeHist(gray[h:h+1,:])
    eq[eq <= 50] = 0
    eq[(eq < 170) & (eq > 50)] = 125
    eq[eq >= 170] = 255
    edges = cv.Canny(eq, param_canny_min, param_canny_max).ravel()
    """
    edges = canny[h:h+1, :].ravel()
    color = gray[h:h+1, :].ravel()
    #edges = np.mean(canny[h-2:h+3, :],axis=0).ravel()

    bord = np.where(edges == 255)[0]
    if bord.shape[0]<3: # we want to have at least 4 edges.
        return -1
    fil = np.ediff1d(bord,to_end=20) > 9
    bord = bord[fil] # keeps only the large difference
    print("----------------")
    print(h,"coord",bord)
    for idx,b in enumerate(bord):
        cv.circle(img,(b,h), 3, (0,0,255), 1)
        cv.circle(img,(b,h), 4, (0,0,255), 1)
        cv.line(img,(0,h), (gray.shape[1],h),(255,0,0), 1)
        font = cv.FONT_HERSHEY_SIMPLEX
        if h % 50 == 0:
            cv.putText(img,str(h),(10,h), font, .3,(255,255,255),1,cv.LINE_AA)
    r=np.empty(len(bord)-1,dtype=int)
    ii=0
    for idx,b in enumerate(bord[:-1]):
        c=np.mean(color[b:bord[idx+1]])
        r[idx]=int(c)


    delta = np.ediff1d(bord)
    delta_x = (delta -u0) / (vh-h) * beta_x

    print(h,"color",r)
    print(h,"delta",delta)
    for idx in range(1,len(r)-1):
        if r[idx] > 200 and r[idx-1]<80 and r[idx+1]<80:
            print(h,"ok color",r[idx-1],r[idx],r[idx+1])
            cv.line(img,(bord[idx],h), (bord[idx+1],h),(0,255,255), 1)
        else:
            print(h,"nok color",r[idx-1],r[idx],r[idx+1])
        if abs(delta[idx]-delta[idx-1])<delta[idx]/2 and abs(delta[idx]-delta[idx+1])<delta[idx]/2:  
            print(h,"ok dist",delta[idx-1],delta[idx],delta[idx+1])
            cv.line(img,(bord[idx],h+1), (bord[idx+1],h+1),(255,255,0), 1)
        else:
            print(h,"nok dist",delta[idx-1],delta[idx],delta[idx+1])
        if r[idx] > 200 and r[idx-1]<80 and r[idx+1]<80 and abs(delta[idx]-delta[idx-1])<delta[idx]/2 and abs(delta[idx]-delta[idx+1])<delta[idx]/2:  
            cv.line(img,(bord[idx],h), (bord[idx+1],h),(0,255,0), 3)
            v=(bord[idx]+bord[idx+1]) //2
            return v

    target = np.where(abs(delta_x - w) < wd)[0]
    band_dist = np.ediff1d(target)

    if len(target) == 3 and band_dist[0] == 1 and band_dist[1] == 1:
        return (bord[target[1]] + bord[target[1]+1]) // 2
    elif len(target) == 222 and band_dist[0] == 1:
        c0, c1 = (bord[target[0]] + bord[target[0]+1]) // 2, (bord[target[1]] + bord[target[1]+1]) // 2
        g_vals = gray[h:h+1, :].ravel()
        return c1 if g_vals[c0] < g_vals[c1] else c0
    return -1

def prepare_image_internal(img):
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    blur= cv.GaussianBlur(gray, (5, 5), 0)
    denois = cv.fastNlMeansDenoising(blur)
    eq = cv.equalizeHist(denois)
    return eq

@chronometre
def find_band(gray, canny, way):
    height = gray.shape[0]
    h = height // 8
    r_name = "B_find+" if way == 1 else "B_find-"
    k = 0
    output=[]
    while 0 <= h < height:
        k += 1
        m = analyse_line_fast(gray, canny, h)
        if m != -1:
            output.append((m,h))
        h += 10 * way
    for o in output:
        cv.circle(img,(o[0],o[1]), 3, (0,0,255), 1)

    return -1, 0


    
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv.setTrackbarPos(low_H_name, window_detection_name, low_H)



def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv.setTrackbarPos(high_H_name, window_detection_name, high_H)


def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv.setTrackbarPos(high_V_name, window_detection_name, high_V)

def find_redline(img):
    h, w = img.shape[:2]
    img2=cv.resize(img, (w//4, h//4), interpolation=cv.INTER_AREA)
    hsv = cv.cvtColor(img2, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, (120, 123, 0), (240, 255, 255))
    row_density = mask[::10].sum(axis=1)
    col_density = mask[:,::10].sum(axis=0)
    y = np.where(row_density>200)[0]
    x = np.where(col_density>50)[0]
    if len(x) and len(y):
        x1, x2 = x[0]*40, x[-1]*40
        y1, y2 = y[0]*40, y[-1]*40


print(cv.__version__)
window_detection_name = "detect"
max_value = 255
max_value_H = 360
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value


low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'

mypath = "/home/arnaud/Projet/road/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
for file in onlyfiles:
    if file[:3] != "red":
        continue
    print(file)
    img = cv.imread(mypath+file)
    if img is None:
        sys.exit("Could not read the image.")

    cv.namedWindow("img", cv.WINDOW_NORMAL)
    """
    cv.namedWindow("detect", cv.WINDOW_NORMAL)

    cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
    cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
    cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
    cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
    cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
    cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)

    #detect = find_redline(img)
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    """
    #print(hsv[1477:1478,hsv.shape[1]//2:hsv.shape[1]//2+1,:])
    #cv.imshow("detect", detect)
    find_redline(img)   
    cv.imshow("img", img)
    k = cv.waitKey(0)

    """
    while True:
        detect = cv.inRange(hsv, (low_H, low_S, low_V), (high_H, high_S, high_V))
        cv.imshow("detect", detect)
        k = cv.waitKey(0)

        if k == ord("s"):
            cv.destroyWindow()
    """
