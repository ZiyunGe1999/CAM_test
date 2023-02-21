import cv2
import os
import numpy as np

class sheetDetection:
    def __init__(self):
        print("Constructor for sheetDetection was called")
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        self.captures_path= os.path.join(self.dir_path, "images/captures")
        self.images_path= os.path.join(self.dir_path, "images/opencv_images")
        self.detected_images_path= os.path.join(self.dir_path, "images/detected_images")

    def __del__(self):
        print("Destructor for sheetDetection was called")
    
    def detectMarkersAndCrop(self):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_7X7_100)
        parameters = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        corners, ids, _ = detector.detectMarkers(self.img1)
        x_coords = np.array([corners[i][0][0][0] for i in range(len(corners))])
        sort_indices = np.argsort(x_coords)
        sorted_corners = np.array([corners[i] for i in sort_indices][-4:])
        sorted_ids = np.array([ids[i] for i in sort_indices][-4:])
        img_with_markers = cv2.aruco.drawDetectedMarkers(self.img1.copy(), sorted_corners, sorted_ids)
        # cv2.imwrite(f'{self.images_path}/markers_detected.jpg', img_with_markers)

        x, y, w, h = cv2.boundingRect(sorted_corners.reshape(-1, 2))
        self.img1 = self.img1[y:y+h, x:x+w]


    def load_image(self, img_file):
        print("load_image:")
        self.img_file=img_file
        self.img1= cv2.imread(os.path.join(self.captures_path, self.img_file))
        self.detectMarkersAndCrop()
        # cv2.imwrite(os.path.join(self.images_path, "1-img.jpg"), self.img1)
        self.image_size=self.img1.shape
        self.line_thickness=int(self.image_size[0]/300)

    def find_corners(self):
        print("find_corners:")
        # self.mycorners=[(1500,1000),(2500,2000)]
        blue_lower = (90, 25, 25)
        blue_upper = (130, 255, 255)
        hsv = cv2.cvtColor(self.img1, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, blue_lower, blue_upper)
        # cv2.imwrite(os.path.join(self.images_path, "mask.jpg"), mask)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        largest_contour = max(contours, key=cv2.contourArea)
        perimeter = cv2.arcLength(largest_contour, True)
        corners = cv2.approxPolyDP(largest_contour, 0.002 * perimeter, True)
        contours_img = self.img1.copy()
        cv2.drawContours(contours_img, largest_contour, -1, (0, 255, 0), 2)
        # cv2.imwrite(os.path.join(self.images_path, "contours_img.jpg"), contours_img)
        self.mycorners = [corner[0] for corner in corners]
        print(self.mycorners)


    def output_image(self):
        print("output_image:")
        for point in self.mycorners:
            self.img1= cv2.drawContours(self.img1,np.array([[point]]),-1,(0,0,255),2)
            self.img1= cv2.circle(self.img1,(point[0],point[1]),self.line_thickness*5,(0,0,255),self.line_thickness*2)
        cv2.imwrite(os.path.join(self.detected_images_path, "output.jpg"), self.img1)
        

def main(input_image):
        mysheet= sheetDetection()
        mysheet.load_image(input_image)
        mysheet.find_corners()
        mysheet.output_image()
        return mysheet.mycorners

if __name__ == '__main__':
        input_image="71.jpg"
        output_points=main(input_image)
        print(output_points)