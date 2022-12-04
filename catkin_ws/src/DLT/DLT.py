import cv2
import numpy as np  
import os
from numpy import matlib

'''
DLT class

Takes in file paths to two images, a left and right view of a scene, and a number of points to use for calibration (defaults to 7).

If the scene has never been calibrated, run the getCalibrationPoints method. This will open a window for each image, and allow you to click on the points to use for calibration. The points should be clicked in the same order in both images. Select a calibration point and then press space bar to confirm the choice. FIXME: add a marker to the selected point before confirming. You are then prompted to enter the physical xyz coordinates for the same points. The solveLR method is then called by the function and the calibration is complete.

Running the getXYZ method will open a window for the left and right images. Select the same point in both images. Remember to press space after selecting the point in each image. The xyz coordinates of the point are then returned.

If wanted, run the DLT_Save_To_File method to save the calibration data to a file. This will save the calibration data to a file corresponding to the names of the images used. This file can then be used to load the calibration data for future use with the DLT_Load_From_File method.

The origin is at the bottom corner of the grid boards where all three meet with positive Z going up positive x on the left and positive y on the right as you look at the images. This means all objects are in the positive x, y, and z (octant 1) 

'''

class DLT:

    def __init__(self, left_img, right_img, numPoints = 7):
        self.left = left_img
        self.right = right_img
        self.numPoints = numPoints
        self.uL = np.array([])
        self.vL = np.array([])
        self.uR = np.array([])
        self.vR = np.array([])
        self.X = -1
        self.Y = -1
        self.calibrationPointsXYZ = np.zeros((self.numPoints, 3))     

    # Function to get uL, vL, uR, vR, calibrationPointsXYZ from a file
    def DLT_Load_From_File(self, filename):
        
        # Open file
        f = open(filename, "r")

        # Read in uL, vL, uR, vR, calibrationPointsXYZ
        self.uL = np.array([float(x) for x in f.readline().split()])
        self.vL = np.array([float(x) for x in f.readline().split()])
        self.uR = np.array([float(x) for x in f.readline().split()])
        self.vR = np.array([float(x) for x in f.readline().split()])
        self.calibrationPointsXYZ = np.array([[float(x) for x in f.readline().split()] for i in range(self.numPoints)])

        # Close file
        f.close()
        self.solveLR()

    # Function to save uL, vL, uR, vR, calibrationPointsXYZ to a file in a subfolder called DLT_Data with filename taken from the image file names
    def DLT_Save_To_File(self):
        # Create folder to store data
        folder = "DLT_Data"
        if not os.path.exists(folder):
            os.makedirs(folder)

        # Create filename
        filename = folder + "/" + self.left.split("/")[-1].split(".")[0] + "_" + self.right.split("/")[-1].split(".")[0] + ".txt"

        # Open file
        f = open(filename, "w")

        # Write uL, vL, uR, vR, calibrationPointsXYZ to file
        f.write(" ".join(str(x) for x in self.uL) + "\n")        
        f.write(" ".join(str(x) for x in self.vL) + "\n")
        f.write(" ".join(str(x) for x in self.uR) + "\n")
        f.write(" ".join(str(x) for x in self.vR) + "\n")
        for i in range(self.numPoints):
            f.write(" ".join(str(x) for x in self.calibrationPointsXYZ[i, :]) + "\n")

        f.close()
                
    # Function to get pixel coordinates and enter XYZ real world coordinates of calibration points
    def getCalibrationPoints(self):

        #Do left image calibration points
        for i in range(self.numPoints):

                left = cv2.imread(self.left)
                cv2.putText(left, f"Select calibration point {i} on the left image then press a key to continue to the next point", (10, 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 0, 255), 1)
                cv2.imshow("left", left)
                cv2.setMouseCallback("left", self.mousePoints)

                # Display calibration points
                if len(self.uL) > 0:
                    for j in range(self.uL.shape[0]):
                        cv2.circle(left, (int(self.uL[j]), int(self.vL[j])), 3, (255, 0, 0), -1)

                cv2.imshow("left", left)

                k = cv2.waitKey(0) & 0xFF

                if k == 27:
                    break

                self.uL = np.append(self.uL, self.X)
                self.vL = np.append(self.vL, self.Y)

        #Do right image calibration points
        for i in range(self.numPoints):

                right = cv2.imread(self.right)
                cv2.putText(right, f"Select calibration point {i} on the right image then press a key to continue to the next point", (10, 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 0, 255), 1)
                cv2.imshow("right", right)
                cv2.setMouseCallback("right", self.mousePoints)

                # Display calibration points
                if len(self.uR) > 0:
                    for j in range(self.uR.shape[0]):
                        cv2.circle(right, (int(self.uR[j]), int(self.vR[j])), 3, (255, 0, 0), -1)

                cv2.imshow("right", right)

                k = cv2.waitKey(0) & 0xFF

                if k == 27:
                    break

                self.uR = np.append(self.uR, self.X)
                self.vR = np.append(self.vR, self.Y)

        # Enter calibration points physical coordinates
        for i in range(self.numPoints):
            prompt = input(f"Enter in units of cm the X,Y,Z coordinates of calibration point {i} as X Y Z: ")
            x, y, z = prompt.split()
            self.calibrationPointsXYZ[i, 0] = x
            self.calibrationPointsXYZ[i, 1] = y
            self.calibrationPointsXYZ[i, 2] = z

        self.solveLR()

    # Loads previously saved calibration points if the numpy arrays are supplied.
    def loadCalibrationPoints(self, uL, vL, uR, vR, calibrationPointsXYZ):
        self.uL = uL
        self.vL = vL
        self.uR = uR
        self.vR = vR
        self.calibrationPointsXYZ = calibrationPointsXYZ

    # Solve for L and R
    def solveLR(self):
        self.tu = matlib.repmat(np.array([1,0,0,0,0]), self.numPoints, 1)
        self.tv = matlib.repmat(np.array([0,0,0,0]), self.numPoints, 1)
        self.tv2 = matlib.repmat(np.array([1]), self.numPoints, 1)

        self.XL = np.hstack((self.calibrationPointsXYZ, self.tu,
                            (-self.uL*self.calibrationPointsXYZ.conj().T[0,:]).reshape(self.numPoints, 1),
                            (-self.uL*self.calibrationPointsXYZ.conj().T[1,:]).reshape(self.numPoints,1),
                            (-self.uL*self.calibrationPointsXYZ.conj().T[2,:]).reshape(self.numPoints,1)))
        self.XL = np.hstack((self.calibrationPointsXYZ, self.tu,
                            (-self.uL*self.calibrationPointsXYZ.conj().T[0,:]).reshape(self.numPoints,1),
                            (-self.uL*self.calibrationPointsXYZ.conj().T[1,:]).reshape(self.numPoints,1),
                            (-self.uL*self.calibrationPointsXYZ.conj().T[2,:]).reshape(self.numPoints,1)))
        self.YL = np.hstack((self.tv, self.calibrationPointsXYZ, self.tv2,
                            (-self.vL*self.calibrationPointsXYZ.conj().T[0,:]).reshape(self.numPoints,1),
                            (-self.vL*self.calibrationPointsXYZ.conj().T[1,:]).reshape(self.numPoints,1),
                            (-self.vL*self.calibrationPointsXYZ.conj().T[2,:]).reshape(self.numPoints,1)))

        self.XR = np.hstack((self.calibrationPointsXYZ, self.tu,
                            (-self.uR*self.calibrationPointsXYZ.conj().T[0,:]).reshape(self.numPoints,1),
                            (-self.uR*self.calibrationPointsXYZ.conj().T[1,:]).reshape(self.numPoints,1),
                            (-self.uR*self.calibrationPointsXYZ.conj().T[2,:]).reshape(self.numPoints,1)))
        self.YR = np.hstack((self.tv, self.calibrationPointsXYZ, self.tv2,
                            (-self.vR*self.calibrationPointsXYZ.conj().T[0,:]).reshape(self.numPoints,1),
                            (-self.vR*self.calibrationPointsXYZ.conj().T[1,:]).reshape(self.numPoints,1),
                            (-self.vR*self.calibrationPointsXYZ.conj().T[2,:]).reshape(self.numPoints,1)))

        self.FL = np.vstack((self.XL[0,:], self.YL[0,:]))
        self.gL = np.vstack((self.uL[0], self.vL[0]))

        self.FR = np.vstack((self.XR[0,:], self.YR[0,:]))
        self.gR = np.vstack((self.uR[0], self.vR[0]))

        for i in range(1, self.numPoints):
            self.gL = np.vstack((self.gL, self.uL[i], self.vL[i]))
            self.FL = np.vstack((self.FL, self.XL[i,:], self.YL[i,:]))

            self.gR = np.vstack((self.gR, self.uR[i], self.vR[i]))
            self.FR = np.vstack((self.FR, self.XR[i,:], self.YR[i,:]))

        self.L = np.linalg.solve(self.FL.conj().T @ self.FL, self.FL.conj().T @ self.gL)
        self.R = np.linalg.solve(self.FR.conj().T @ self.FR, self.FR.conj().T @ self.gR)

    # Get coordinates of a point of interest from the image
    def getXYZ(self):

        self.solveLR()
        # Get point from left image
        left = cv2.imread(self.left)
        cv2.putText(left, f"Select the point to measure", (10, 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 0, 255), 1)
        cv2.imshow("left", left)
        cv2.setMouseCallback("left", self.mousePoints)

        cv2.waitKey(0)

        UL = self.X
        VL = self.Y

        # Get point from right image
        right = cv2.imread(self.right)
        cv2.putText(right, f"Select the point to measure", (10, 10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.5, (0, 0, 255), 1)
        cv2.imshow("right", right)
        cv2.setMouseCallback("right", self.mousePoints)

        cv2.waitKey(0)

        UR = self.X
        VR = self.Y

        Q = np.array([[self.L[0]-self.L[8]*UL,  self.L[1]-self.L[9]*UL,  self.L[2]-self.L[10]*UL],
                      [self.L[4]-self.L[8]*VL,  self.L[5]-self.L[9]*VL,  self.L[6]-self.L[10]*VL],
                      [self.R[0]-self.R[8]*UR,  self.R[1]-self.R[9]*UR,  self.R[2]-self.R[10]*UR],
                      [self.R[4]-self.R[8]*VR,  self.R[5]-self.R[9]*VR,  self.R[6]-self.R[10]*VR]]).reshape(4,3)

        q = np.array([[UL-self.L[3]], 
                      [VL-self.L[7]],
                      [UR-self.R[3]],
                      [VR-self.R[7]]]).reshape(4,1)
    
        #Calculates xyz values
        a_xyz =  np.linalg.inv(Q.T @ Q) @ Q.T @ q;

        return a_xyz

    # Callback for mouse events
    def mousePoints(self, event, x,y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.X = x
            self.Y = y


if __name__ == "__main__":

    # ENTER FULL PATH TO IMAGES
    left_path = '/Users/ikas/Documents/ME 537/Robotics_Final_Project/catkin_ws/src/DLT/left.JPG'
    right_path = '/Users/ikas/Documents/ME 537/Robotics_Final_Project/catkin_ws/src/DLT/right.JPG'

    # Calibration data for provided images to test the code
    uL = np.array([1914,  539, 1781, 1423, 1271, 2028, 2184])
    vL = np.array([ 654,  360, 1947, 1082, 1564, 1333, 2069])
    uR = np.array([1528,  285,  801, 1009,  664, 1428, 1428])
    vR = np.array([ 636,  314, 1734,  984, 1353, 1303, 1303])
    calibrationPointsXYZ = np.array([[0, 9, 9],
                                    [12.5, 0, 13.5],
                                    [12.5, 14, 0],
                                    [5.5, 5.5, 4.8],
                                    [11.5, 7.5, 1.7],
                                    [3.5, 12.5, 2.4],
                                    [13, 18, .75]])


    # dltobj.calibrate()
    dltobj = DLT(left_path, right_path)
    dltobj.loadCalibrationPoints(uL, vL, uR, vR, calibrationPointsXYZ)
    xyz = dltobj.getXYZ()


    # dltobj.DLT_Save_To_File()
    # secondObject = DLT(left, right)
    # secondObject.DLT_Load_From_File('')

    print(xyz)
