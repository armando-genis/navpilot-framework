import numpy as np
import cv2
import yaml
import argparse
from tqdm import tqdm
import os
from dataclasses import dataclass
from typing import Optional



class Camera:

  K = np.zeros([3, 3])
  R = np.zeros([3, 3])
  t = np.zeros([3, 1])
  P = np.zeros([3, 4])

  def setK(self, fx, fy, px, py):
    self.K[0, 0] = fx
    self.K[1, 1] = fy
    self.K[0, 2] = px
    self.K[1, 2] = py
    self.K[2, 2] = 1.0

  def setR(self, y, p, r):

    Rz = np.array([[np.cos(-y), -np.sin(-y), 0.0], [np.sin(-y), np.cos(-y), 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[np.cos(-p), 0.0, np.sin(-p)], [0.0, 1.0, 0.0], [-np.sin(-p), 0.0, np.cos(-p)]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, np.cos(-r), -np.sin(-r)], [0.0, np.sin(-r), np.cos(-r)]])
    Rs = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]) # switch axes (x = -y, y = -z, z = x)
    self.R = Rs.dot(Rz.dot(Ry.dot(Rx)))

  def setT(self, XCam, YCam, ZCam):
    X = np.array([XCam, YCam, ZCam])
    self.t = -self.R.dot(X)

  def updateP(self):
    Rt = np.zeros([3, 4])
    Rt[0:3, 0:3] = self.R
    Rt[0:3, 3] = self.t
    self.P = self.K.dot(Rt)

  def __init__(self, config):
    self.setK(config["fx"], config["fy"], config["px"], config["py"])
    self.setR(np.deg2rad(config["yaw"]), np.deg2rad(config["pitch"]), np.deg2rad(config["roll"]))
    self.setT(config["XCam"], config["YCam"], config["ZCam"])
    self.updateP()

@dataclass
class CameraConfig:
    camera: Camera
    image_path: str
    homography_matrix: Optional[np.ndarray] = None  # original homography
    adjusted_cv_homography: Optional[np.ndarray] = None  # adjusted OpenCV homography
    adjusted_stn_homography: Optional[np.ndarray] = None  # adjusted SpatialTransformer homography

class homography_converter:
    def __init__(self, homography_matrix, old_resolution, new_resolution):
        self.homography_matrix = homography_matrix
        self.old_resolution = old_resolution
        self.new_resolution = new_resolution
        self.is_new_input_wide = None
        self.intermediate_shape_input = None
        self.intermediate_shape_output = None
        self.Ti = None
        self.Ki = None
        self.To = None
        self.Ko = None
        self.Si = None
        self.So = None


    def intermediate_shape(self):
        new_input_aspect_ratio = self.new_resolution[0] / self.new_resolution[1]
        self.is_new_input_wide = new_input_aspect_ratio <= 1
        if self.is_new_input_wide:
            new_input_resolution_at_old_input_aspect_ratio = (self.new_resolution[1] / self.old_resolution[1] * self.old_resolution[0], self.new_resolution[1])
            old_output_resolution_at_new_output_aspect_ratio = (new_input_aspect_ratio * self.old_resolution[1], self.old_resolution[1])
        else:
            new_input_resolution_at_old_input_aspect_ratio = (self.new_resolution[0], self.new_resolution[0] / self.old_resolution[0] * self.old_resolution[1])
            old_output_resolution_at_new_output_aspect_ratio = (self.old_resolution[0], self.old_resolution[0] / new_input_aspect_ratio)

        return new_input_resolution_at_old_input_aspect_ratio, old_output_resolution_at_new_output_aspect_ratio

    def correct_aspect_ratio(self):
        px = (self.intermediate_shape_input[1] - self.new_resolution[1]) / 2 if self.is_new_input_wide else 0
        py = (self.intermediate_shape_input[0] - self.new_resolution[0]) / 2 if self.is_new_input_wide else 0
        Ti = np.array([[ 1,  0, px],
                       [ 0,  1, py],
                       [ 0,  0,  1]], dtype=np.float32)
        self.Ti = Ti

    def scale_input(self):
        fx = self.old_resolution[1] / self.intermediate_shape_input[1]
        fy = self.old_resolution[0] / self.intermediate_shape_input[0]
        Ki = np.array([[fx,  0, 0],
                       [ 0, fy, 0],
                       [ 0,  0, 1]], dtype=np.float32)
        self.Ki = Ki
    
    def crop_output(self):
        px = -(self.old_resolution[1] - self.intermediate_shape_output[1]) / 2
        py = -(self.old_resolution[0] - self.intermediate_shape_output[0]) / 2
        To = np.array([[ 1,  0, px],
                       [ 0,  1, py],
                       [ 0,  0,  1]], dtype=np.float32)
        self.To = To

    def scale_output(self):
        fx = self.new_resolution[1] / self.intermediate_shape_output[1]
        fy = self.new_resolution[0] / self.intermediate_shape_output[0]
        Ko = np.array([[fx,  0, 0],
                       [ 0, fy, 0],
                       [ 0,  0, 1]], dtype=np.float32)
        self.Ko = Ko

    
    def scale_from_unit_grid_to_input(self):
        fx = self.new_resolution[1] / 2
        fy = self.new_resolution[0] / 2
        px = self.new_resolution[1] / 2
        py = self.new_resolution[0] / 2
        Si = np.array([[fx,  0, px],
                       [ 0, fy, py],
                       [ 0,  0,  1]], dtype=np.float32)
        self.Si = Si
    
    def scale_from_unit_grid_to_output(self):
        fx = 2 / self.new_resolution[1]
        fy = 2 / self.new_resolution[0]
        px = -1
        py = -1
        So = np.array([[fx,  0, px],
                       [ 0, fy, py],
                       [ 0,  0,  1]], dtype=np.float32)
        self.So = So


    def run(self) -> list[tuple[np.ndarray, np.ndarray]]:
        """
        Run the homography conversion.
        
        Returns:
            List of tuples (adjusted_cv_homography, adjusted_stn_homography) for each input homography.
        """
        self.intermediate_shape_input, self.intermediate_shape_output = self.intermediate_shape()
        self.correct_aspect_ratio()
        self.scale_input()
        self.crop_output()
        self.scale_output()
        self.scale_from_unit_grid_to_input()
        self.scale_from_unit_grid_to_output()

        results = []
        
        # assemble adjusted homography
        for homography in self.homography_matrix: 
            cvH = np.array(homography)
            cvHr = self.Ko.dot(self.To.dot(cvH.dot(self.Ki.dot(self.Ti))))
            stnHr = np.linalg.inv(self.So.dot(cvHr.dot(self.Si)))

            print(f"\nOriginal OpenCV homography used for resolution {self.old_resolution[0]}x{self.old_resolution[1]} -> {self.new_resolution[0]}x{self.new_resolution[1]}:")
            print(cvH.tolist())

            print(f"\nAdjusted OpenCV homography usable for resolution {self.new_resolution[0]}x{self.new_resolution[1]} -> {self.old_resolution[0]}x{self.old_resolution[1]}:")
            print(cvHr.tolist())

            print(f"\nAdjusted SpatialTransformer homography usable for resolution {self.new_resolution[0]}x{self.new_resolution[1]} -> {self.old_resolution[0]}x{self.old_resolution[1]}:")
            print(stnHr.tolist())

            print("--------------------------------")
            
            results.append((cvHr, stnHr))
        
        return results


class BirdEyeView():
    def __init__(self, images_dir, camera_configs_dir, interpMode, input_resolution, output_resolution):
        self.images_dir = images_dir
        self.camera_configs_dir = camera_configs_dir
        self.camera_configs = []  # raw yaml configs
        self.images = []  # image paths
        self.camera_config_list: list[CameraConfig] = []  # populated CameraConfig objects
        self.drone_camera_object = None
        self.droneConfig = None
        self.IPMs = []
        self.masks = []
        self.outputRes = None
        self.pxPerM = None
        self.interpMode = interpMode
        self.input_resolution = input_resolution
        self.output_resolution = output_resolution

        self.load_camera_configs()
        self.create_camera_objects()

    def load_camera_configs(self):
        yaml_files = sorted([os.path.splitext(f)[0] for f in os.listdir(self.camera_configs_dir) if f.endswith('.yaml') and f != 'drone.yaml'])
        camera_files = sorted([os.path.splitext(f)[0] for f in os.listdir(self.images_dir) if f.endswith('.png')])

        yaml_set = set(yaml_files)
        missing = [name for name in camera_files if name not in yaml_set]
        if missing:
              raise FileNotFoundError(
                  f"No YAML config for image(s): {missing}. "
                  f"Add {[n + '.yaml' for n in missing]} to {self.camera_configs_dir}"
              )
        else:
            print(f"All camera configs found for {len(yaml_files)} images")
            # load camera configs and images
            for yaml_file in yaml_files:
                config_path = os.path.join(self.camera_configs_dir, yaml_file + '.yaml')
                with open(config_path) as stream:
                    self.camera_configs.append(yaml.safe_load(stream))
                image_path = os.path.join(self.images_dir, yaml_file + '.png')
                self.images.append(image_path)

            # load drone config
            drone_config_path = os.path.join(self.camera_configs_dir, 'drone.yaml')
            with open(drone_config_path) as stream:
                self.droneConfig = yaml.safe_load(stream)

            print(self.images)

    def create_camera_objects(self):
        for config, image_path in zip(self.camera_configs, self.images):
            camera = Camera(config)
            camera_config = CameraConfig(
                camera=camera,
                image_path=image_path,
                homography_matrix=None
            )
            self.camera_config_list.append(camera_config)
        self.drone_camera_object = Camera(self.droneConfig)
        
    def calculate_output_shape(self):
        if not self.droneConfig:
            self.pxPerM = (self.droneConfig["r"], self.droneConfig["r"])
            self.outputRes = (int(self.droneConfig["wm"] * self.pxPerM[0]), int(self.droneConfig["hm"] * self.pxPerM[1]))
        else:
            self.outputRes = (int(2 * self.droneConfig["py"]), int(2 * self.droneConfig["px"]))
            dx = self.outputRes[1] / self.droneConfig["fx"] * self.droneConfig["ZCam"]
            dy = self.outputRes[0] / self.droneConfig["fy"] * self.droneConfig["ZCam"]
            self.pxPerM = (self.outputRes[0] / dy, self.outputRes[1] / dx)

        # setup mapping from street/top-image plane to world coords
        shift = (self.outputRes[0] / 2.0, self.outputRes[1] / 2.0)
        if self.droneConfig:
            shift = shift[0] + self.droneConfig["YCam"] * self.pxPerM[0], shift[1] - self.droneConfig["XCam"] * self.pxPerM[1]
            M = np.array([[1.0 / self.pxPerM[1], 0.0, -shift[1] / self.pxPerM[1]], [0.0, -1.0 / self.pxPerM[0], shift[0] / self.pxPerM[0]], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0]])

        return M

    # calculate IPM for each camera and print the homographies
    def calculate_ipm(self):
        M = self.calculate_output_shape()
        
        for camera_config in self.camera_config_list:
            ipm = np.linalg.inv(camera_config.camera.P.dot(M))
            camera_config.homography_matrix = ipm
            self.IPMs.append(ipm)
            print(f"OpenCV homography for {camera_config.image_path}:")
            print(ipm.tolist())

        homography_matrices = [cc.homography_matrix for cc in self.camera_config_list]
        converter = homography_converter(homography_matrices, self.input_resolution, self.output_resolution)
        adjusted_homographies = converter.run()
        
        # Store adjusted homographies in CameraConfig objects
        for camera_config, (cv_h, stn_h) in zip(self.camera_config_list, adjusted_homographies):
            camera_config.adjusted_cv_homography = cv_h
            camera_config.adjusted_stn_homography = stn_h

    # That mask is used to hide "behind the camera" regions in each warped image before stitching.
    def invalid_mask(self):
        # Create coordinate grids once (vectorized) - much faster than nested loops
        j_coords = np.arange(self.outputRes[0])  # rows
        i_coords = np.arange(self.outputRes[1])  # cols
        i_grid, j_grid = np.meshgrid(i_coords, j_coords)
        
        # Compute theta for all pixels at once
        y_offset = -j_grid + self.outputRes[0] / 2 - self.droneConfig["YCam"] * self.pxPerM[0]
        x_offset = i_grid - self.outputRes[1] / 2 + self.droneConfig["XCam"] * self.pxPerM[1]
        theta = np.rad2deg(np.arctan2(y_offset, x_offset))
        
        for config in self.camera_configs:
            yaw = config["yaw"]
            # Compute angular difference with proper wrapping
            diff = theta - yaw
            # Normalize to [-180, 180]
            diff = (diff + 180) % 360 - 180
            diff = np.abs(diff)
            # Check if angle is in "behind camera" range (> 90 degrees from camera direction)
            mask_2d = diff > 90
            # Expand to 3 channels
            mask = np.stack([mask_2d, mask_2d, mask_2d], axis=-1)
            self.masks.append(mask)

    def warp_images(self):
        images = []
        for camera_config in self.camera_config_list:
            filename = os.path.basename(camera_config.image_path)
            print(f"Warping image: {filename}")
            images.append(cv2.imread(camera_config.image_path))

        # warp input images
        warpedImages = []
        for camera_config, img in zip(self.camera_config_list, images):
            warpedImages.append(cv2.warpPerspective(img, camera_config.homography_matrix, (self.outputRes[1], self.outputRes[0]), flags=self.interpMode))

        # remove invalid areas (behind the camera) from warped images
        for warpedImg, mask in zip(warpedImages, self.masks):
            warpedImg[mask] = 0

        # stitch separate images to total bird's-eye-view with averaging in overlap regions
        birdsEyeView = np.zeros(warpedImages[0].shape, dtype=np.float32)
        count = np.zeros((self.outputRes[0], self.outputRes[1]), dtype=np.float32)
        
        for warpedImg in warpedImages:
            img_float = warpedImg.astype(np.float32)
            valid_mask = np.any(warpedImg != (0, 0, 0), axis=-1)
            birdsEyeView[valid_mask] += img_float[valid_mask]
            count[valid_mask] += 1
        
        # Avoid division by zero
        count[count == 0] = 1
        birdsEyeView = birdsEyeView / count[:, :, np.newaxis]
        birdsEyeView = np.clip(birdsEyeView, 0, 255).astype(np.uint8)

        # display or export bird's-eye-view
        cv2.imshow("Bird's-eye-view", birdsEyeView)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        # export bird's-eye-view
        cv2.imwrite("birds_eye_view.png", birdsEyeView)


if __name__ == "__main__":
    images_dir = "images"
    camera_configs_dir = "camera_configs"
    interpMode = cv2.INTER_LINEAR  # Use bilinear interpolation for smooth results (INTER_CUBIC for even smoother)
    input_resolution = (604, 964)
    output_resolution = (256, 512)
    drone_camera = BirdEyeView(images_dir, camera_configs_dir, interpMode, input_resolution, output_resolution)
    drone_camera.calculate_ipm()
    drone_camera.invalid_mask()
    drone_camera.warp_images()
