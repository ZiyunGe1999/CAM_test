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

    def load_image(self, img_file):
        print("load_image:")
        self.img_file=img_file
        self.img1= cv2.imread(os.path.join(self.captures_path, self.img_file))
        cv2.imwrite(os.path.join(self.images_path, "1-img.jpg"), self.img1)
        self.image_size=self.img1.shape
        self.line_thickness=int(self.image_size[0]/300)

    def find_corners(self):
        print("find_corners:")
        self.mycorners=[(1500,1000),(2500,2000)]

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