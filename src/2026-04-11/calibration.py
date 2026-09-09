import cv2
import numpy as np
import math

def draw_position(event,u,v,flags,param):
   if event == cv2.EVENT_LBUTTONDBLCLK:
       x=beta_u * ( u - u0) / (v-vh)
       y=beta_v / (v-vh)
       cv2.rectangle(img,(480,350),(600,480),(255,255,255),-1) 
       cv2.putText(img, "u="+str(u), (500,380),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
       cv2.putText(img, "v="+str(v), (500,400),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
       cv2.putText(img, "x="+str(x), (500,420),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
       cv2.putText(img, "y="+str(y), (500,440),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)


filename = "/home/arnaud/Projet/homographie/img/cam_2369973011473.jpg"
v1=300
v2=450
u1=180
img = cv2.imread(filename)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_gray[:,:v1]=255
img_gray[:,v2:]=255
img_gray[:u1,:]=255 # partie haute
cv2.line(img,(v1,0),(v1,img.shape[0]),(255,0,0),1)
cv2.line(img,(v2,0),(v2,img.shape[0]),(255,0,0),1)
cv2.line(img,(0,u1),(img.shape[1],u1),(255,0,0),1)
# find contours in the binary image
ret, thresh = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY_INV)
cv2.imshow("Thresh", thresh)
contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print("-------------")
k=0
r= {}
y0= 0.162
delta = 0.0265

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
   y = y0 + math.floor(k/3)*delta
   imgpoints.append((u,v))
   objpoints.append((x,y,0))
   print(u,v,k % 3,math.floor(k/3),x,y)
   r[str(k%3)+"-"+str(math.floor(k/3))]={"x":x,"y":y,"u":u,"v":v}
   cv2.putText(img, str(k%3)+","+str(math.floor(k/3)), (u - 150, v+20 ),cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 0), 1)
   k+=1

u0 = r["0-0"]["u"]
print("u0",u0)
vh = (r["0-0"]["y"]*r["0-0"]["v"]-r["0-4"]["y"]*r["0-4"]["v"]) / (r["0-0"]["y"]-r["0-4"]["y"])
print("vh",vh)
beta_v = r["0-0"]["y"] * (r["0-0"]["v"] - vh)
print("beta_v",beta_v)
beta_u = r["2-0"]["x"] * (r["2-0"]["v"] - vh) / (r["2-0"]["u"] - u0)
print("beta_u",beta_u)
cv2.putText(img, "u0="+str(u0), (10,400),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
cv2.putText(img, "vh="+str(vh), (10,420),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
cv2.putText(img, "beta_u="+str(beta_u), (10,440),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
cv2.putText(img, "beta_v="+str(beta_v), (10,460),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

for k in r.keys():
   x= beta_u * (r[k]["u"] - u0) / ( r[k]["v"] - vh)
   y= beta_v / ( r[k]["v"] - vh)
   print(k,r[k]["x"],x)
   print(k,r[k]["y"],y)

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera([objpoints], [imgpoints], img_gray.shape[::-1], None, None)
print(mtx)
         
while(1):
    cv2.imshow("image", img)
    cv2.namedWindow('image')
    cv2.setMouseCallback('image',draw_position)
    if cv2.waitKey(20) & 0xFF == 27:
        break
cv2.destroyAllWindows()
