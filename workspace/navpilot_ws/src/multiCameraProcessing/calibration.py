import numpy as np
import cv2 as cv
import glob
import os
import matplotlib.pyplot as plt

# =============================================================================
# CALIBRATION SETTINGS
# =============================================================================
# Pattern type: 1 = Chessboard, 2 = ChArUco
PATTERN_TYPE = 2

# Lens model: 1 = Pinhole (standard), 2 = Fisheye (wide-angle/fisheye lenses)
LENS_MODEL = 2

# =============================================================================
# BOARD CONFIGURATION
# =============================================================================
class BoardConfig:
    # Board dimensions (number of squares, not inner corners)
    ROWS = 8
    COLS = 11
    
    # Size in mm
    CHECKER_SIZE = 20.0  # mm
    MARKER_SIZE = 15.0   # mm (only used for ChArUco)
    
    # ArUco dictionary (only used for ChArUco)
    ARUCO_DICT = cv.aruco.DICT_4X4_250
    
    @classmethod
    def get_chessboard_size(cls):
        """Returns (cols, rows) inner corners for OpenCV."""
        return (cls.COLS - 1, cls.ROWS - 1)
    
    @classmethod
    def get_charuco_board(cls):
        """Creates and returns a ChArUco board (supports both old and new OpenCV API)."""
        dictionary = cv.aruco.getPredefinedDictionary(cls.ARUCO_DICT)
        
        try:
            board = cv.aruco.CharucoBoard(
                (cls.COLS, cls.ROWS),
                cls.CHECKER_SIZE / 1000.0,
                cls.MARKER_SIZE / 1000.0,
                dictionary
            )
        except AttributeError:
            board = cv.aruco.CharucoBoard_create(
                cls.COLS, cls.ROWS,
                cls.CHECKER_SIZE / 1000.0,
                cls.MARKER_SIZE / 1000.0,
                dictionary
            )
        return board, dictionary
    
    @classmethod
    def get_charuco_object_points(cls, charucoIds):
        """Get 3D object points for detected ChArUco corner IDs."""
        board, _ = cls.get_charuco_board()
        
        # Get all chessboard corners from board
        try:
            # New API
            allObjPoints = board.getChessboardCorners()
        except AttributeError:
            # Old API
            allObjPoints = board.chessboardCorners
        
        # Select only the detected corner IDs
        objPoints = allObjPoints[charucoIds.flatten()]
        return objPoints


# =============================================================================
# CORNER DETECTION FUNCTIONS
# =============================================================================

def detect_chessboard_corners(imgPathList, showPics=True):
    """Detect chessboard corners in images."""
    patternSize = BoardConfig.get_chessboard_size()
    nCols, nRows = patternSize
    checkerSize = BoardConfig.CHECKER_SIZE
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # World points template
    worldPtsCur = np.zeros((nRows * nCols, 3), np.float32)
    worldPtsCur[:, :2] = np.mgrid[0:nCols, 0:nRows].T.reshape(-1, 2) * checkerSize / 1000.0  # Convert to meters
    
    allObjectPoints = []
    allImagePoints = []
    imgSize = None
    
    print(f"Chessboard detection: {nCols}x{nRows} inner corners, {checkerSize}mm squares")
    print(f"Processing {len(imgPathList)} images...")
    
    for curImgPath in imgPathList:
        imgBGR = cv.imread(curImgPath)
        if imgBGR is None:
            print(f"  Warning: Could not read {curImgPath}")
            continue
            
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
        imgSize = imgGray.shape[::-1]
        
        cornersFound, cornersOrg = cv.findChessboardCorners(imgGray, patternSize, None)
        
        if cornersFound:
            cornersRefined = cv.cornerSubPix(imgGray, cornersOrg, (11, 11), (-1, -1), termCriteria)
            allObjectPoints.append(worldPtsCur.reshape(1, -1, 3))
            allImagePoints.append(cornersRefined.reshape(1, -1, 2))
            print(f"  ✓ Found corners in: {os.path.basename(curImgPath)}")
            
            if showPics:
                cv.drawChessboardCorners(imgBGR, patternSize, cornersRefined, cornersFound)
                cv.imshow('Chessboard', imgBGR)
                cv.waitKey(300)
        else:
            print(f"  ✗ No corners in: {os.path.basename(curImgPath)}")
    
    cv.destroyAllWindows()
    return allObjectPoints, allImagePoints, imgSize


def detect_charuco_corners(imgPathList, showPics=True):
    """Detect ChArUco corners in images."""
    board, dictionary = BoardConfig.get_charuco_board()
    
    try:
        detector = cv.aruco.ArucoDetector(dictionary)
        use_new_api = True
    except AttributeError:
        use_new_api = False
    
    allObjectPoints = []
    allImagePoints = []
    imgSize = None
    
    print(f"ChArUco detection: {BoardConfig.COLS}x{BoardConfig.ROWS} board")
    print(f"  Checker: {BoardConfig.CHECKER_SIZE}mm, Marker: {BoardConfig.MARKER_SIZE}mm")
    print(f"Processing {len(imgPathList)} images...")
    
    for curImgPath in imgPathList:
        imgBGR = cv.imread(curImgPath)
        if imgBGR is None:
            print(f"  Warning: Could not read {curImgPath}")
            continue
            
        imgGray = cv.cvtColor(imgBGR, cv.COLOR_BGR2GRAY)
        imgSize = imgGray.shape[::-1]
        
        # Detect ArUco markers
        if use_new_api:
            markerCorners, markerIds, rejected = detector.detectMarkers(imgGray)
        else:
            markerCorners, markerIds, rejected = cv.aruco.detectMarkers(imgGray, dictionary)
        
        if markerIds is not None and len(markerIds) > 0:
            retval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(
                markerCorners, markerIds, imgGray, board
            )
            
            if retval > 4:
                # Get corresponding object points for detected corner IDs
                objPoints = BoardConfig.get_charuco_object_points(charucoIds)
                
                allObjectPoints.append(objPoints.reshape(1, -1, 3).astype(np.float64))
                allImagePoints.append(charucoCorners.reshape(1, -1, 2).astype(np.float64))
                print(f"  ✓ Found {retval} corners in: {os.path.basename(curImgPath)}")
                
                if showPics:
                    cv.aruco.drawDetectedMarkers(imgBGR, markerCorners, markerIds)
                    cv.aruco.drawDetectedCornersCharuco(imgBGR, charucoCorners, charucoIds)
                    cv.imshow('ChArUco', imgBGR)
                    cv.waitKey(300)
            else:
                print(f"  ✗ Not enough corners ({retval}) in: {os.path.basename(curImgPath)}")
        else:
            print(f"  ✗ No markers in: {os.path.basename(curImgPath)}")
    
    cv.destroyAllWindows()
    return allObjectPoints, allImagePoints, imgSize


# =============================================================================
# CALIBRATION FUNCTIONS
# =============================================================================

def calibrate_pinhole(objectPoints, imagePoints, imgSize):
    """Calibrate using standard pinhole model (Brown-Conrady distortion)."""
    print("\nRunning PINHOLE calibration...")
    
    # Convert list format for standard calibration
    objPts = [pts.reshape(-1, 3).astype(np.float32) for pts in objectPoints]
    imgPts = [pts.reshape(-1, 1, 2).astype(np.float32) for pts in imagePoints]
    
    repError, camMatrix, distCoeff, rvecs, tvecs = cv.calibrateCamera(
        objPts, imgPts, imgSize, None, None
    )
    
    return camMatrix, distCoeff, repError, rvecs, tvecs, objPts, imgPts


def calibrate_fisheye(objectPoints, imagePoints, imgSize):
    """Calibrate using fisheye model (equidistant projection)."""
    print("\nRunning FISHEYE calibration...")
    
    # Initialize camera matrix and distortion
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    
    # Fisheye calibration flags
    calibration_flags = (
        cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
        cv.fisheye.CALIB_CHECK_COND |
        cv.fisheye.CALIB_FIX_SKEW
    )
    
    # Fisheye needs specific array format
    objPts = [pts.astype(np.float64) for pts in objectPoints]
    imgPts = [pts.astype(np.float64) for pts in imagePoints]
    
    rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in objectPoints]
    tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for _ in objectPoints]
    
    try:
        repError, K, D, rvecs, tvecs = cv.fisheye.calibrate(
            objPts,
            imgPts,
            imgSize,
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
    except cv.error as e:
        print(f"Warning: Fisheye calibration failed with flags, trying without CALIB_CHECK_COND...")
        # Try without condition check (can help with difficult calibrations)
        calibration_flags = (
            cv.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
            cv.fisheye.CALIB_FIX_SKEW
        )
        repError, K, D, rvecs, tvecs = cv.fisheye.calibrate(
            objPts,
            imgPts,
            imgSize,
            K,
            D,
            rvecs,
            tvecs,
            calibration_flags,
            (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        )
    
    return K, D, repError, rvecs, tvecs, objPts, imgPts


# =============================================================================
# REPROJECTION ERROR ANALYSIS
# =============================================================================

def fisheye_reprojection_error_by_radius(objectPoints, imagePoints, rvecs, tvecs, K, D, imgSize):
    """Compute per-point reprojection error vs distance from image center.
    
    This reveals if calibration quality degrades at image edges (common with fisheye).
    
    Args:
        objectPoints: List of object points per image
        imagePoints: List of image points per image
        rvecs: Rotation vectors per image
        tvecs: Translation vectors per image
        K: Camera matrix
        D: Distortion coefficients
        imgSize: (width, height)
    
    Returns:
        numpy array of shape (N, 2) where each row is [radius, error]
    """
    cx, cy = imgSize[0] / 2, imgSize[1] / 2
    all_errors = []
    
    for objPts, imgPts, rvec, tvec in zip(objectPoints, imagePoints, rvecs, tvecs):
        # Ensure correct shape for fisheye.projectPoints
        # objPts should be (N, 1, 3), rvec/tvec should be (3, 1) or (1, 1, 3)
        objPts_reshaped = objPts.reshape(-1, 1, 3).astype(np.float64)
        rvec_reshaped = rvec.reshape(3, 1).astype(np.float64)
        tvec_reshaped = tvec.reshape(3, 1).astype(np.float64)
        
        # Project with fisheye model
        projected, _ = cv.fisheye.projectPoints(
            objPts_reshaped, rvec_reshaped, tvec_reshaped, K, D
        )
        
        projected = projected.reshape(-1, 2)
        measured = imgPts.reshape(-1, 2)
        
        for p_proj, p_meas in zip(projected, measured):
            # Reprojection error
            err = np.linalg.norm(p_proj - p_meas)
            
            # Radial distance from image center
            r = np.linalg.norm(p_meas - np.array([cx, cy]))
            
            all_errors.append((r, err))
    
    return np.array(all_errors)


def pinhole_reprojection_error_by_radius(objectPoints, imagePoints, rvecs, tvecs, camMatrix, distCoeff, imgSize):
    """Compute per-point reprojection error vs distance from image center (pinhole model)."""
    cx, cy = imgSize[0] / 2, imgSize[1] / 2
    all_errors = []
    
    for objPts, imgPts, rvec, tvec in zip(objectPoints, imagePoints, rvecs, tvecs):
        objPts_reshaped = objPts.reshape(-1, 3).astype(np.float32)
        
        # Project with pinhole model
        projected, _ = cv.projectPoints(objPts_reshaped, rvec, tvec, camMatrix, distCoeff)
        
        projected = projected.reshape(-1, 2)
        measured = imgPts.reshape(-1, 2)
        
        for p_proj, p_meas in zip(projected, measured):
            err = np.linalg.norm(p_proj - p_meas)
            r = np.linalg.norm(p_meas - np.array([cx, cy]))
            all_errors.append((r, err))
    
    return np.array(all_errors)


def plot_reprojection_error_by_radius(errors, title="Reprojection error vs radius"):
    """Plot reprojection error as a function of distance from image center.
    
    What to look for:
    - Flat curve: good calibration
    - Slight increase near edges: normal for fisheye
    - Explosions (>2-3 px): calibration problem
    - Edge errors <1 px: excellent calibration
    """
    r = errors[:, 0]
    e = errors[:, 1]
    
    # Compute statistics
    mean_err = np.mean(e)
    max_err = np.max(e)
    std_err = np.std(e)
    
    # Bin by radius for trend line
    num_bins = 20
    bin_edges = np.linspace(0, np.max(r), num_bins + 1)
    bin_centers = (bin_edges[:-1] + bin_edges[1:]) / 2
    bin_means = []
    bin_stds = []
    
    for i in range(num_bins):
        mask = (r >= bin_edges[i]) & (r < bin_edges[i + 1])
        if np.sum(mask) > 0:
            bin_means.append(np.mean(e[mask]))
            bin_stds.append(np.std(e[mask]))
        else:
            bin_means.append(np.nan)
            bin_stds.append(np.nan)
    
    bin_means = np.array(bin_means)
    bin_stds = np.array(bin_stds)
    
    # Plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    # Scatter plot
    ax1.scatter(r, e, s=5, alpha=0.3, c='blue', label='Per-point error')
    ax1.plot(bin_centers, bin_means, 'r-', linewidth=2, label='Mean trend')
    ax1.fill_between(bin_centers, bin_means - bin_stds, bin_means + bin_stds, 
                     alpha=0.2, color='red', label='±1 std')
    ax1.axhline(y=1.0, color='green', linestyle='--', alpha=0.7, label='1 px threshold')
    ax1.axhline(y=2.0, color='orange', linestyle='--', alpha=0.7, label='2 px threshold')
    ax1.set_xlabel("Radius from image center (px)")
    ax1.set_ylabel("Reprojection error (px)")
    ax1.set_title(title)
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim(0, min(max_err * 1.1, 5))  # Cap at 5px for visibility
    
    # Histogram
    ax2.hist(e, bins=50, edgecolor='black', alpha=0.7)
    ax2.axvline(x=mean_err, color='red', linestyle='-', linewidth=2, label=f'Mean: {mean_err:.3f} px')
    ax2.axvline(x=1.0, color='green', linestyle='--', alpha=0.7, label='1 px')
    ax2.set_xlabel("Reprojection error (px)")
    ax2.set_ylabel("Count")
    ax2.set_title("Error Distribution")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    print("\n" + "="*50)
    print("REPROJECTION ERROR ANALYSIS")
    print("="*50)
    print(f"Total points: {len(e)}")
    print(f"Mean error: {mean_err:.4f} px")
    print(f"Std error: {std_err:.4f} px")
    print(f"Max error: {max_err:.4f} px")
    print(f"Points with error > 1px: {np.sum(e > 1)} ({100*np.sum(e > 1)/len(e):.1f}%)")
    print(f"Points with error > 2px: {np.sum(e > 2)} ({100*np.sum(e > 2)/len(e):.1f}%)")
    
    # Quality assessment
    if mean_err < 0.5 and max_err < 2:
        print("\n✓ EXCELLENT calibration quality")
    elif mean_err < 1.0 and max_err < 3:
        print("\n✓ GOOD calibration quality")
    elif mean_err < 2.0:
        print("\n⚠ ACCEPTABLE calibration, consider recapturing images")
    else:
        print("\n✗ POOR calibration, recapture images or check lens model")
    
    return mean_err, std_err, max_err


def calibrate(showPics=True):
    """Main calibration function."""
    # Read images
    root = os.getcwd()
    calibrationDir = os.path.join(root, '/workspace/navpilot_ws/src/camera_modules/cameraphotos/obsbot')
    imgPathList = glob.glob(os.path.join(calibrationDir, '*.jpg'))
    
    if len(imgPathList) == 0:
        print(f"Error: No images found in {calibrationDir}")
        return None, None
    
    patternName = 'Chessboard' if PATTERN_TYPE == 1 else 'ChArUco'
    lensName = 'Pinhole' if LENS_MODEL == 1 else 'Fisheye'
    
    print(f"Found {len(imgPathList)} images in {calibrationDir}")
    print(f"Pattern: {patternName}, Lens model: {lensName}\n")
    
    # Detect corners based on pattern type
    if PATTERN_TYPE == 1:
        objectPoints, imagePoints, imgSize = detect_chessboard_corners(imgPathList, showPics)
    else:
        objectPoints, imagePoints, imgSize = detect_charuco_corners(imgPathList, showPics)
    
    if len(objectPoints) < 3:
        print("Error: Need at least 3 valid images for calibration")
        return None, None
    
    print(f"\nCalibrating with {len(objectPoints)} valid images...")
    
    # Run calibration based on lens model
    if LENS_MODEL == 1:
        camMatrix, distCoeff, repError, rvecs, tvecs, objPts, imgPts = calibrate_pinhole(objectPoints, imagePoints, imgSize)
    else:
        camMatrix, distCoeff, repError, rvecs, tvecs, objPts, imgPts = calibrate_fisheye(objectPoints, imagePoints, imgSize)
    
    print('\n' + '='*50)
    print('CALIBRATION RESULTS')
    print('='*50)
    print(f'Lens model: {lensName}')
    print(f'Image size: {imgSize[0]}x{imgSize[1]}')
    print(f'\nCamera Matrix (K):\n{camMatrix}')
    print(f'\nDistortion Coefficients (D):\n{distCoeff.T}')
    print(f'\nReprojection Error (OpenCV): {repError:.4f} pixels')
    
    # Per-point reprojection error analysis by radius
    print("\nAnalyzing reprojection error by radius from image center...")
    if LENS_MODEL == 1:
        errors = pinhole_reprojection_error_by_radius(
            objPts, imgPts, rvecs, tvecs, camMatrix, distCoeff, imgSize
        )
        title = "Pinhole: Reprojection error vs radius"
    else:
        errors = fisheye_reprojection_error_by_radius(
            objPts, imgPts, rvecs, tvecs, camMatrix, distCoeff, imgSize
        )
        title = "Fisheye: Reprojection error vs radius"
    
    # Plot error analysis (this tells the truth about edge quality)
    plot_reprojection_error_by_radius(errors, title)
    
    # Save calibration
    curFolder = os.path.dirname(os.path.abspath(__file__))
    paramPath = os.path.join(curFolder, 'calibration.yaml')
    
    fs = cv.FileStorage(paramPath, cv.FILE_STORAGE_WRITE)
    fs.write('lens_model', lensName.lower())
    fs.write('pattern_type', patternName.lower())
    fs.write('image_width', imgSize[0])
    fs.write('image_height', imgSize[1])
    fs.write('camera_matrix', camMatrix)
    fs.write('distortion_coefficients', distCoeff)
    fs.write('reprojection_error', repError)
    fs.write('board_rows', BoardConfig.ROWS)
    fs.write('board_cols', BoardConfig.COLS)
    fs.write('checker_size_mm', BoardConfig.CHECKER_SIZE)
    if PATTERN_TYPE == 2:
        fs.write('marker_size_mm', BoardConfig.MARKER_SIZE)
    fs.release()
    
    print(f'\nCalibration saved to: {paramPath}')
    
    return camMatrix, distCoeff


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
    lensModel = fs.getNode('lens_model').string()
    fs.release()
    
    print(f"Loaded calibration from: {yamlPath}")
    print(f"Lens model: {lensModel}")
    print(f"Image size: {imageWidth}x{imageHeight}")
    print(f"Reprojection error: {repError:.4f} pixels")
    
    return camMatrix, distCoeff, lensModel, (imageWidth, imageHeight)


# =============================================================================
# UNDISTORTION FUNCTIONS
# =============================================================================

def undistort_pinhole(img, camMatrix, distCoeff):
    """Undistort image using pinhole model."""
    height, width = img.shape[:2]
    camMatrixNew, roi = cv.getOptimalNewCameraMatrix(
        camMatrix, distCoeff, (width, height), 1, (width, height)
    )
    imgUndist = cv.undistort(img, camMatrix, distCoeff, None, camMatrixNew)
    return imgUndist


def undistort_fisheye(img, camMatrix, distCoeff, balance=0.0):
    """Undistort image using fisheye model.
    
    Args:
        img: Input image
        camMatrix: Camera matrix K
        distCoeff: Distortion coefficients D (4 parameters for fisheye)
        balance: 0.0 = most cropped (no black), 1.0 = least cropped (shows all pixels)
    """
    height, width = img.shape[:2]
    imgSize = (width, height)
    
    # Estimate new camera matrix for undistortion
    # balance: 0 = crop to valid pixels, 1 = keep all pixels (with black borders)
    newK = cv.fisheye.estimateNewCameraMatrixForUndistortRectify(
        camMatrix, distCoeff, imgSize, np.eye(3), balance=balance
    )
    
    # Create undistortion maps
    map1, map2 = cv.fisheye.initUndistortRectifyMap(
        camMatrix, distCoeff, np.eye(3), newK, imgSize, cv.CV_16SC2
    )
    
    # Apply remapping
    imgUndist = cv.remap(img, map1, map2, interpolation=cv.INTER_LINEAR, borderMode=cv.BORDER_CONSTANT)
    
    return imgUndist


def removeDistortion(imgPath=None, saveOutput=True, showPics=True, balance=1.0):
    """Remove distortion from an image using saved calibration.
    
    Args:
        imgPath: Path to input image (optional)
        saveOutput: If True, saves the undistorted image
        showPics: If True, displays comparison
        balance: For fisheye only - 0.0=crop, 1.0=keep all
    
    Returns:
        imgUndist: The undistorted image
    """
    # Load calibration
    camMatrix, distCoeff, lensModel, imgSize = loadCalibration()
    
    if imgPath is None:
        root = os.getcwd()
        imgPath = os.path.join(root, '/workspace/navpilot_ws/src/camera_modules/cameraphotos/obsbot/obsbot_20260203_060903_844.jpg')
    
    img = cv.imread(imgPath)
    if img is None:
        print(f"Error: Could not load image from {imgPath}")
        return None
    
    print(f"\nUndistorting with {lensModel} model...")
    
    # Apply appropriate undistortion
    if lensModel == 'fisheye':
        imgUndist = undistort_fisheye(img, camMatrix, distCoeff, balance)
    else:
        imgUndist = undistort_pinhole(img, camMatrix, distCoeff)
    
    # Save undistorted image
    if saveOutput:
        baseName, ext = os.path.splitext(imgPath)
        outputPath = f"{baseName}_undistorted{ext}"
        cv.imwrite(outputPath, imgUndist)
        print(f"Undistorted image saved to: {outputPath}")
    
    # Display comparison
    if showPics:
        plt.figure(figsize=(14, 6))
        plt.subplot(121)
        plt.title(f'Original ({lensModel} lens)')
        plt.imshow(cv.cvtColor(img, cv.COLOR_BGR2RGB))
        plt.axis('off')
        plt.subplot(122)
        plt.title(f'Undistorted (balance={balance})')
        plt.imshow(cv.cvtColor(imgUndist, cv.COLOR_BGR2RGB))
        plt.axis('off')
        plt.tight_layout()
        plt.show()
    
    return imgUndist


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def generateCharucoBoard(outputPath=None):
    """Generate and save a ChArUco board image for printing."""
    board, _ = BoardConfig.get_charuco_board()
    
    try:
        boardImg = board.generateImage((2480, 3508), marginSize=50, borderBits=1)
    except AttributeError:
        boardImg = board.draw((2480, 3508), marginSize=50, borderBits=1)
    
    if outputPath is None:
        curFolder = os.path.dirname(os.path.abspath(__file__))
        outputPath = os.path.join(curFolder, 'charuco_board.png')
    
    cv.imwrite(outputPath, boardImg)
    print(f"ChArUco board saved to: {outputPath}")
    print(f"Board: {BoardConfig.COLS}x{BoardConfig.ROWS} squares")
    print(f"Checker: {BoardConfig.CHECKER_SIZE}mm, Marker: {BoardConfig.MARKER_SIZE}mm")
    
    return boardImg


def runCalibration():
    calibrate(showPics=True)


def runRemoveDistortion():
    """Test undistortion with saved calibration."""
    removeDistortion(saveOutput=True, showPics=True, balance=1.0)


if __name__ == '__main__':
    # Generate ChArUco board for printing (uncomment if needed)
    # generateCharucoBoard()
    
    runCalibration()
    runRemoveDistortion()
