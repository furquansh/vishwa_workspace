import cv2

# changes made in classify_signs function to pricesly tune the arrow detection and minimise the noise amap
def classify_signs(image, cascade_classifier):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    classified = cascade_classifier.detectMultiScale(image, minNeighbors=4, minSize=(10,10))
    return classified    ### config for astra pro plus

# def classify_signs(image, cascade_classifier):
#     # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     classified = cascade_classifier.detectMultiScale(image, minNeighbors=1, minSize=(40,40))
#     return classified   #####this config works for webcam on my pc

# def classify_signs(image, cascade_classifier):
#     # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     classified = cascade_classifier.detectMultiScale(image)
#     return classified

# changes made by Furquan Shiekh for Autex 2024

def show_box(image, classified, color=(150, 150, 255), thickness=2):
    for x,y,w,h in classified:
        cv2.rectangle(image, (x, y), (x+w, y+h), color, thickness)
        midpoint = ((x+(x+w))/2, y+(y+h)/2)
    #     point = (x, y)
        return midpoint 

def show_dist(image, classified):
    for x,y,w,h in classified:
        point = (x, y)
    return point