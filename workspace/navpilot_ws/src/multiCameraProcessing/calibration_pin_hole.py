import numpy as np
import cv2 as cv
import glob
import os
import matplotlib.pyplot as plt






def calibrate(showPics=True):
    # Read Image
    root = os.getcwd()
    calibrationDir = os.path.join(root,'/workspace/navpilot_ws/src/camera_modules/cameraphotos/obsbot')
    imgPathList = glob.glob(os.path.join(calibrationDir,'*.jpg'))
    
    # Initialize
    # Calibration board: 8 rows x 11 cols, 20mm checker size (160mm x 220mm)
    # Inner corners = (rows-1) x (cols-1) = 7 vertical x 10 horizontal
    # OpenCV convention: patternSize = (columns, rows) = (horizontal, vertical)
    nCols = 10  # inner corners horizontally (11 squares - 1)
    nRows = 7   # inner corners vertically (8 squares - 1)
    patternSize = (nCols, nRows)  # OpenCV expects (cols, rows)
    checkerSize = 20  # mm
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # World points: corners are detected row by row, left to right
    worldPtsCur = np.zeros((nRows * nCols, 3), np.float32)
    worldPtsCur[:, :2] = np.mgrid[0:nCols, 0:nRows].T.reshape(-1, 2) * checkerSize
    worldPtsList = []
    imgPtsList = []
    
    # Find Corners
    for curImgPath in imgPathList:
        imgBGR = cv.imread(curImgPath)
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
        cornersFound, cornersOrg = cv.findChessboardCorners(imgGray, patternSize, None)
        
        if cornersFound == True:
            worldPtsList.append(worldPtsCur)
            cornersRefined = cv.cornerSubPix(imgGray, cornersOrg, (11, 11), (-1, -1), termCriteria)
            imgPtsList.append(cornersRefined)
            if showPics:
                cv.drawChessboardCorners(imgBGR, patternSize, cornersRefined, cornersFound)
                cv.imshow('Chessboard', imgBGR)
                cv.waitKey(500)
    cv.destroyAllWindows()
    
    # Calibrate
    repError,camMatrix,distCoeff,rvecs,tvecs = cv.calibrateCamera(worldPtsList, imgPtsList, imgGray.shape[::-1],None,None)
    print('Camera Matrix:\n',camMatrix)
    print("Reproj Error (pixels): {:.4f}".format(repError))
    
    # Save Calibration Parameters to YAML
    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder,'calibration.yaml')
    
    fs = cv.FileStorage(paramPath, cv.FILE_STORAGE_WRITE)
    fs.write('image_width', imgGray.shape[1])
    fs.write('image_height', imgGray.shape[0])
    fs.write('camera_matrix', camMatrix)
    fs.write('distortion_coefficients', distCoeff)
    fs.write('reprojection_error', repError)
    fs.write('rvecs', np.array(rvecs))
    fs.write('tvecs', np.array(tvecs))
    fs.release()
    
    print(f"Calibration saved to: {paramPath}")
    
    return camMatrix,distCoeff

def loadCalibration(yamlPath=None):
    """Load calibration parameters from YAML file."""
    if yamlPath is None:
        curFolder = os.path.dirname(os.path.abspath(__file__))
        yamlPath = os.path.join(curFolder, 'calibration.yaml')
    
    fs = cv.FileStorage(yamlPath, cv.FILE_STORAGE_READ)
    camMatrix = fs.getNode('camera_matrix').mat()
    distCoeff = fs.getNode('distortion_coefficients').mat()
    repError = fs.getNode('reprojection_error').real()
    imageWidth = int(fs.getNode('image_width').real())
    imageHeight = int(fs.getNode('image_height').real())
    fs.release()
    
    print(f"Loaded calibration from: {yamlPath}")
    print(f"Image size: {imageWidth}x{imageHeight}")
    print(f"Reprojection error: {repError:.4f} pixels")
    
    return camMatrix, distCoeff

def removeDistortion(camMatrix, distCoeff, imgPath=None):
    """Remove distortion from an image using calibration parameters."""
    if imgPath is None:
        root = os.getcwd()
        imgPath = os.path.join(root, '/workspace/navpilot_ws/src/camera_modules/cameraphotos/obsbot/obsbot_20260203_052606_131.jpg')
    
    img = cv.imread(imgPath)
    if img is None:
        print(f"Error: Could not load image from {imgPath}")
        return
    
    height, width = img.shape[:2]
    camMatrixNew, roi = cv.getOptimalNewCameraMatrix(camMatrix, distCoeff, (width, height), 1, (width, height))
    imgUndist = cv.undistort(img, camMatrix, distCoeff, None, camMatrixNew)
    
    # Draw Line to See Distortion Change
    cv.line(img, (1769, 103), (1780, 922), (255, 255, 255), 2)
    cv.line(imgUndist, (1769, 103), (1780, 922), (255, 255, 255), 2)
    
    plt.figure()
    plt.subplot(121)
    plt.title('Original')
    plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
    plt.subplot(122)
    plt.title('Undistorted')
    plt.imshow(cv.cvtColor(imgUndist, cv.COLOR_BGR2RGB))
    plt.show()

def runCalibration():
    calibrate(showPics=True)

def runRemoveDistortion():
    """Load calibration from YAML and test undistortion."""
    camMatrix, distCoeff = loadCalibration()
    removeDistortion(camMatrix, distCoeff)

if __name__ == '__main__':
    runCalibration()
    runRemoveDistortion()
