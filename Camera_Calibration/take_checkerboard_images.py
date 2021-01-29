import cv2

camera = cv2.VideoCapture(1)

cv2.namedWindow("Checkerboard Images")

img_counter = 27

while True:
    ret,frame = camera.read()
    cv2.imshow("test", frame)
    k = cv2.waitKey(1)
    if k%256 == 27: #if Escape is pressed
        break
    elif k%256 == 32: #if Space is pressed
        img_name ="checkerboard_{}..png".format(img_counter)
        cv2.imwrite(img_name, frame)
        print("Image taken")
        img_counter += 1

camera.release()
cv2.destroyAllWindows()



