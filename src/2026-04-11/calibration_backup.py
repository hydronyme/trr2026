import cv2
import numpy as np
import math

filename = "/home/arnaud/Projet/homographie/img/cam_2369973011473.jpg"
img = cv2.imread(filename)
img[:,:300,:]=255
img[:,450:,:]=255
img[:180,:,:]=255 # partie haute
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
"""
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
height=img.shape[0]
width=img.shape[1]
img_yuv = cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
vec_channels = cv2.split(img_yuv);
vec_channels = list(vec_channels)
vec_channels[0] = cv2.equalizeHist(vec_channels[0])
img_yuv = cv2.merge(vec_channels)
img_norm = cv2.cvtColor(img_yuv, cv2.COLOR_YUV2BGR)
#img_gray = cv2.cvtColor(img_norm,cv2.COLOR_BGR2GRAY)
cv2.imshow("Display window", img)
cv2.imshow("Normalize window", img_norm)

img_hue = hsv[...,0]
cv2.imshow("img_hue", img_hue)
img_value = hsv[...,2]
#cv2.imshow("img_value", img_value)
#cv2.imshow("img_green", img[...,1])
#cv2.imshow("img_red", img[...,2])
#cv2.imshow("img_blue", img[...,0])

img_filter0 = np.zeros_like(img_gray)
img_filter0 = np.where(img[..., 1] > img[...,0],np.full_like(img_filter0, 1), img_filter0)
img_filter2 = np.zeros_like(img_gray)
img_filter2 = np.where(img[..., 1] > img[...,2],np.full_like(img_filter2, 1), img_filter2)
img_filter = np.multiply(img_filter0, img_filter2)

img_dst = cv2.filter2D(img, -1, img_filter)
cv2.imshow("Filter window", img_dst)


img_rb = img.copy()
img_rb[:,:300,:]=255
img_rb[:,450:,:]=255
img_rb[:180,:,:]=255 # partie haute
img_rb[:,:,1]=0
#img_gray = cv2.cvtColor(img_rb,cv2.COLOR_BGR2GRAY)
cv2.imshow("img_gray", img_gray)
img_lap = cv2.Laplacian(img_gray,-1,ksize=3,borderType=cv2.BORDER_CONSTANT)
ret, img_lap = cv2.threshold(img_lap, 80, 255, cv2.THRESH_BINARY)
cv2.imshow("Laplacian", img_lap)

#img_lap2 = cv2.Laplacian(img_hue,-1,ksize=5,borderType=cv2.BORDER_CONSTANT)
#cv2.imshow("Laplacian2", img_lap2)

#hsv = cv2.cvtColor(img_green, cv2.COLOR_BGR2HSV)
#hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

#img_gray = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
#img_gray = cv2.cvtColor(img_gray, cv2.COLOR_BGR2GRAY)
## Slice the green
#imask = mask > 0
#green = np.zeros_like(img_gray)
#green[imask] = img_gray[imask]


img_cnt = img.copy()

"""
# find contours in the binary image
ret, thresh = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY_INV)
cv2.imshow("Thresh", thresh)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print("-------------")
k=0
r= {}
delta = 0.03

objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

for c in contours:
   M = cv2.moments(c)

   if M["m00"] == 0.:
       continue
   u = int(M["m10"] / M["m00"])
   v = int(M["m01"] / M["m00"])
   cv2.circle(img, (u, v), 5, (255, 255, 255), 1)
   x = (k % 3)*delta
   y = 0.15 + math.floor(k/3)*delta
   imgpoints.append((u,v))
   objpoints.append((x,y,0))
   print(u,v,k % 3,math.floor(k/3),x,y)
   r[str(k%3)+"-"+str(math.floor(k/3))]={"x":x,"y":y,"u":u,"v":v}
   cv2.putText(img, str(k%3)+","+str(math.floor(k/3)), (u - 150, v+20 ),cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
   k+=1

try:
    u0 = r["0-0"]["u"]
    print("u0",u0)
    vh = (r["0-0"]["y"]*r["0-0"]["v"]-r["0-3"]["y"]*r["0-3"]["v"]) / (r["0-0"]["y"]-r["0-3"]["y"])
    print("vh",vh)
    beta_v = r["0-0"]["y"] * (r["0-0"]["v"] - vh)
    print("beta_v",beta_v)
    beta_u = r["2-0"]["x"] * (r["2-0"]["v"] - vh) / (r["2-0"]["u"] - u0)
    print("beta_u",beta_u)

    for k in r.keys():
        x= beta_u * (r[k]["u"] - u0) / ( r[k]["v"] - vh)
        y= beta_v / ( r[k]["v"] - vh)
        print(k,r[k]["x"],x)
        print(k,r[k]["y"],y)

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, img_gray.shape[::-1], None, None)
    print(mtx)
except:
    pass
         
cv2.imshow("countours", img)
k = cv2.waitKey(0)
