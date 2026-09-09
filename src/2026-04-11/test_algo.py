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


param_canny_min = 150
param_canny_max = 200
mypath = "/home/arnaud/Projet/road/"
onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
for file in onlyfiles:
    print(file)
    img = cv.imread(mypath+file)
    if img is None:
        sys.exit("Could not read the image.")

    gray = prepare_image_internal(img)
    canny = cv.Canny(gray, param_canny_min, param_canny_max)
    find_band(gray, canny, 1)

    cv.namedWindow("canny", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("gray", cv.WINDOW_AUTOSIZE)
    cv.namedWindow("img", cv.WINDOW_AUTOSIZE)
    cannyS = cv.resize(canny, (1280, 740))                # Resize image
    grayS = cv.resize(gray, (1280, 740))                # Resize image
    imgS = cv.resize(img, (1280, 740))                # Resize image


    cv.imshow("canny", cannyS)
    cv.imshow("gray", grayS)
    cv.imshow("img", imgS)
    k = cv.waitKey(0)

    if k == ord("s"):
        cv.destroyWindow()

