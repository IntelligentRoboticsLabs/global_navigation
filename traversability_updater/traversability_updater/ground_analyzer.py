"""
GroundAnalyzer class for analyzing ground traversability using various methods.

Uses image and elevation features to analyze ground traversability.
"""

#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ctypes

from PIL import Image

from ament_index_python.packages import get_package_share_directory

import cv2 as cv

from net import VAE

import numpy as np

from scipy import ndimage as ndi

import torch
import torch.utils.data

import torchvision.transforms as transforms


class GroundAnalyzer():
    """
    A class used to analyze ground traversability using various methods.

    Attributes
    ----------
    img_mode : str
        The mode of image analysis ('HC' or 'VAE').
    zsize : int
        The size of the latent space for VAE.
    img_min_dist_ : float
        Minimum distance between image features to be considered different.
    elev_min_dist_ : float
        Minimum distance between elevation features to be considered different.
    recompute_step_ : int
        Step size for moving the submap.
    elev_features_ : np.ndarray
        Array to store elevation features.
    max_unkown_ : int
        Maximum number of unknown pixels allowed in a submap.
    features_filename_ : str
        Filename to save/load features.
    img_model : str
        The mode of image analysis ('HC' or 'VAE').
    submap_size_ : int
        Size of the submap.
    resolution_ : float
        Resolution of the grid map.
    size_x_ : int
        Size of the grid map in the x direction.
    size_y_ : int
        Size of the grid map in the y direction.
    features_ : np.ndarray
        Array to store image features.
    transform : torchvision.transforms.Compose
        Transformations to apply to the images.
    vae : VAE
        Variational Autoencoder model for feature extraction.
    """

    def __init__(self, img_mode='HC'):
        """Initialize the GroundAnalyzer with the specified image mode.

        Parameters
        ----------
        img_mode : str, optional
            The mode of image analysis ('HC' or 'VAE'), by default 'HC'.
        """
        self.zsize = 128

        # Distance between features to be considered different
        self.img_min_dist_ = 0.05
        self.elev_min_dist_ = 0.05

        # Step to move the submap (greater step faster, but less accuracy)
        self.recompute_step_ = 2

        self.elev_features_ = np.array([[0, 0, 0]])
        self.max_unkown_ = 16

        self.features_filename_ = 'features.npy'

        self.img_model = img_mode

        self.submap_size_ = 16

        self.resolution_ = 0.2
        self.size_x_ = 500
        self.size_y_ = 500

        if self.img_model == 'HC':
            self.features_ = np.empty((0, 15))

        if img_mode == 'VAE':
            pkg_dir = get_package_share_directory('traversability_updater')
            self.features_ = np.empty((0, self.zsize))
            self.transform = transforms.Compose([transforms.Resize(
                [self.submap_size_, self.submap_size_]), transforms.ToTensor()])
            self.vae = VAE(zsize=self.zsize, layer_count=2, channels=4)
            self.vae.load_state_dict(torch.load(
                pkg_dir + '/traversability_updater/VAEmodel_h_16_128.pkl'))
            self.vae.cuda()
            self.vae.eval()
            print('VAE loaded')

    def load_features(self, filename=None):
        if filename is None:
            filename = self.features_filename_
        self.features_ = np.load(filename)
        print('Features loaded')

    def insert_sample_elev(self, grid_map):

        print('Inserting sample elev')
        self.resolution_ = grid_map.info.resolution
        elev_map = self.map_layer_to_numpy(grid_map, 'elevation')
        feature = self.get_elev_features(elev_map)
        self.add_feature_elev(feature)

    def insert_sample_img(self, grid_map):

        print('Inserting sample')
        self.resolution_ = grid_map.info.resolution
        img_map = self.map_rgb_layer_to_numpy(grid_map, 'RGB')
        feature = self.get_img_feature(img_map)
        self.add_feature_img(np.expand_dims(feature, axis=0))

    def insert_sample_vae(self, grid_map):

        self.resolution_ = grid_map.info.resolution
        img_map = self.map_rgb_layer_to_numpy(grid_map, 'RGB')
        map_elev = self.map_rgb_layer_to_numpy(grid_map, 'elevation')
        feature = self.get_vae_feature(img_map, map_elev)
        self.add_feature_img(feature)

    def recompute_transversality_img(self, msg, threshold=1.0):

        print('Recomputing traversality')
        layer_name = 'RGB'
        layer_index = msg.layers.index(layer_name)
        img_map = self.map_rgb_layer_to_numpy(
            msg, layer_name).astype(np.uint32)

        nav_map = np.zeros(
            (msg.data[layer_index].layout.dim[0].size,
             msg.data[layer_index].layout.dim[1].size)).astype(np.float32)

        for i in np.arange(0, img_map.shape[0] - self.submap_size_,
                           self.recompute_step_):
            for j in np.arange(0, img_map.shape[1] - self.submap_size_,
                               self.recompute_step_):
                submap = np.copy(
                    img_map[i:i+self.submap_size_, j:j+self.submap_size_])
                submap = self.get_rgb_image(submap)
                if np.sum(submap == 0) < self.max_unkown_:
                    if (np.sum(submap == 0) > 0):
                        cv.inpaint(
                            submap,
                            (np.any(submap == 0, axis=2)).astype(np.uint8),
                            3,
                            cv.INPAINT_NS,
                            dst=submap
                        )

                    # submap = submap.astype(np.float32) / 255.0
                    img_features = self.get_img_feature_submap(submap)

                    if (np.sum(np.isnan(submap)) > 0):
                        print('Feature is navigable')
                    # print('BBB', np.mean(submap), np.max(submap))
                    nav_map[i:i+self.submap_size_, j:j+self.submap_size_] += (
                        (self.is_feature_img_nav(img_features, self.features_,
                                                 threshold))
                        * ((self.recompute_step_ / self.submap_size_) ** 2)
                    )

        return nav_map

    def recompute_transversality_elev(self, msg, threshold=0.3):

        print('Recomputing Elev traversality')
        layer_name = 'elevation'
        layer_index = msg.layers.index(layer_name)
        elev_map = self.map_layer_to_numpy(msg, 'elevation')

        nav_map_elev = np.zeros((msg.data[layer_index].layout.dim[0].size,
                                msg.data[layer_index].layout.dim[1].size)).astype(np.float32)

        for i in np.arange(0, elev_map.shape[0] - self.submap_size_, self.recompute_step_*2):
            for j in np.arange(0, elev_map.shape[1] - self.submap_size_, self.recompute_step_*2):
                submap = np.copy(
                    elev_map[i:i+int(self.submap_size_/2), j:j+int(self.submap_size_/2)])
                if np.sum(np.isnan(submap)) < self.max_unkown_/2:

                    elev_features = self.get_elev_features(submap)
                    submap_print = np.nan_to_num(submap, nan=0)
                    if (np.max(submap_print) > 10):
                        print('Submap stats: ', np.mean(submap_print), np.max(
                            submap_print), np.min(submap_print), submap_print.shape)
                    nav_map_elev[i:i+int(self.submap_size_/2), j:j+int(self.submap_size_/2)] += ((self.is_feature_elev_nav(
                        elev_features, self.elev_features_, threshold)))*((self.recompute_step_*2/self.submap_size_)**2)

        return nav_map_elev

    def recompute_transversality_vae(self, msg, threshold=1.5):

        print('Recomputing traversality VAE')
        layer_name = 'RGB'
        layer_index = msg.layers.index(layer_name)
        map_rgb = self.map_rgb_layer_to_numpy(
            msg, layer_name).astype(np.uint32)
        map_rgb = (self.get_rgb_image(map_rgb) * 255).astype(np.uint8)
        layer_name = 'elevation'
        layer_index = msg.layers.index(layer_name)
        map_elev = self.map_layer_to_numpy(msg, layer_name).astype(np.uint32)
        map_elev = np.clip(map_elev * 128 + 64, 0, 254).astype(np.uint8)
        map = np.concatenate(
            [map_rgb, np.expand_dims(map_elev, axis=2)], axis=2)

        print('Map stats 1: ', np.mean(map[:, :, 0]), np.max(
            map[:, :, 0]), np.min(map[:, :, 0]))
        print('Map stats 1: ', np.mean(map[:, :, 1]), np.max(
            map[:, :, 1]), np.min(map[:, :, 1]))
        print('Map stats 1: ', np.mean(map[:, :, 2]), np.max(
            map[:, :, 2]), np.min(map[:, :, 2]))
        print('Map stats 1: ', np.mean(map[:, :, 3]), np.max(
            map[:, :, 3]), np.min(map[:, :, 3]))

        nav_map = np.zeros((msg.data[layer_index].layout.dim[0].size,
                           msg.data[layer_index].layout.dim[1].size)).astype(np.float32)

        img_tensor = img_tensor = torch.zeros(
            0, 4, self.submap_size_, self.submap_size_)
        ij_array = []
        for i in np.arange(0, map.shape[0] - self.submap_size_, self.recompute_step_):
            for j in np.arange(0, map.shape[1] - self.submap_size_, self.recompute_step_):
                submap = np.copy(
                    map[i:i+self.submap_size_, j:j+self.submap_size_])
                if np.sum(submap == 0) < self.max_unkown_:

                    image = Image.fromarray(submap.astype('uint8'))
                    image = self.transform(image)
                    img_tensor = torch.cat(
                        [img_tensor, image[None, :, :, :]], dim=0)
                    ij_array.append([i, j])

        print('map segmentes array completed ', len(ij_array))

        feats_vae = self.vae.encode(img_tensor.cuda())
        feats_vae = feats_vae[0].cpu().detach().numpy()

        # print('VAE features computed', feats_vae)

        nav_map = np.zeros((map.shape[0], map.shape[1])).astype(np.float32)
        ij = 0
        for i, j in ij_array:
            nav_map[i:i+self.submap_size_, j:j+self.submap_size_] += ((self.is_feature_img_nav(
                feats_vae[ij, :], self.features_, threshold)))*((self.recompute_step_/self.submap_size_)**2)
            ij += 1
        return nav_map

    def get_rgb_image(self, submap):
        # Get 3 dims RGB image from submap
        return np.stack(((submap & 255), ((submap >> 8) & 255), (submap >> 16) & 255), axis=-1).astype(np.uint8)

    def get_sobel_feat(self, img, wheigth=0.5):
        sob = cv.Sobel(img, cv.CV_32F, 1, 0, ksize=3)
        sob = (sob + 255) / 2
        sob_std_1 = np.std(sob)
        sob = cv.Sobel(img, cv.CV_32F, 0, 1, ksize=3)
        sob = (sob + 255) / 2
        sob_std_2 = np.std(sob)
        return [np.clip((np.abs((max(sob_std_1, sob_std_2) + 1e-9)/((min(sob_std_1, sob_std_2)) + 1e-9)) - 1) * wheigth, 0, 1)]

    def get_rgb_feat(self, img):
        return [np.mean(img[:, :, 0]), np.std(img[:, :, 0]), np.mean(img[:, :, 1]), np.std(img[:, :, 1]), np.mean(img[:, :, 2]), np.std(img[:, :, 2])]

    def hval_to_vector(self, hval):
        # Convert to 0-360 in radians
        hval = np.deg2rad(hval * 2)
        return [np.cos(hval), np.sin(hval)]

    def get_hsv_feat(self, img):
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        h_mean = self.hval_to_vector(np.mean(hsv[:, :, 0]))
        h_std = self.hval_to_vector(np.std(hsv[:, :, 0]))
        return [h_mean[0], h_mean[1], h_std[0], h_std[1], np.mean(hsv[:, :, 1]), np.std(hsv[:, :, 1]), np.mean(hsv[:, :, 2]), np.std(hsv[:, :, 2])]

    def get_img_feature(self, submap, normalize=False):
        image = self.get_rgb_image(
            submap)[:self.submap_size_, :self.submap_size_, :]

        return self.get_img_feature_submap(image, normalize)

    def get_img_feature_submap(self, image, normalize=False):

        if (np.sum(image == 0) >= self.max_unkown_):
            return np.nan

        if (np.sum(image == 0) > 0) and (np.sum(image == 0) < self.max_unkown_):
            cv.inpaint(image, (np.any(image == 0, axis=2)).astype(
                np.uint8), 3, cv.INPAINT_NS, dst=image)

        if normalize:
            alpha = 255/(np.max(image) - np.min(image))
            beta = -np.min(image)*alpha
            image = cv.convertScaleAbs(
                image, alpha=alpha, beta=beta).astype(np.float32)
        image = image.astype(np.float32) / 255.0
        return np.concatenate(
            [self.get_sobel_feat(cv.cvtColor(image, cv.COLOR_RGB2GRAY)),
             self.get_rgb_feat(image), self.get_hsv_feat(image)])

    def get_vae_feature(self, submap, submap_elev, normalize=False):

        image = self.get_rgb_image(
            submap)[:self.submap_size_, :self.submap_size_, :]

        return self.get_vae_feature_submap(image, submap_elev, normalize)

    def get_vae_feature_submap(self, img, submap_elev, normalize=False):

        if (np.sum(img == 0) >= self.max_unkown_):
            return np.nan

        if ((np.sum(img == 0) > 0) and (np.sum(img == 0) < self.max_unkown_)):
            cv.inpaint(img, (np.any(img == 0, axis=2)).astype(
                np.uint8), 3, cv.INPAINT_NS, dst=img)

        # Temporal, should be inpainted
        submap_elev = np.nan_to_num(submap_elev, nan=0)[
            :self.submap_size_, :self.submap_size_]

        img = (img * 255).astype(np.uint8)
        submap_elev = np.clip(submap_elev * 128 + 64, 0, 254).astype(np.uint8)
        img = np.append(img, np.expand_dims(submap_elev, axis=2), axis=2)

        print('Map stats 2: ', np.mean(img[:, :, 0]), np.max(
            img[:, :, 0]), np.min(img[:, :, 0]))
        print('Map stats 2: ', np.mean(img[:, :, 1]), np.max(
            img[:, :, 1]), np.min(img[:, :, 1]))
        print('Map stats 2: ', np.mean(img[:, :, 2]), np.max(
            img[:, :, 2]), np.min(img[:, :, 2]))
        print('Map stats 2: ', np.mean(img[:, :, 3]), np.max(
            img[:, :, 3]), np.min(img[:, :, 3]))

        img_tensor = torch.zeros(0, 4, self.submap_size_, self.submap_size_)
        image = Image.fromarray(img.astype('uint8'))
        image = self.transform(image)
        img_tensor = torch.cat([img_tensor, image[None, :, :, :]], dim=0)
        feats_vae = self.vae.encode(img_tensor.cuda())
        feats_vae = feats_vae[0].cpu().detach().numpy()
        return feats_vae

    def get_elev_features(self, submap):

        if (np.sum(np.isnan(submap)) >= self.max_unkown_):
            return np.nan

        if (np.sum(np.isnan(submap)) > 0) and (np.sum(np.isnan(submap)) < self.max_unkown_):
            # print('INPAINT!!!', submap)
            cv.inpaint(submap, (np.isnan(submap)).astype(
                np.uint8), 3, cv.INPAINT_NS, dst=submap)

        max_dist = np.clip((np.max(submap) - np.min(submap)/2), 0, 1)
        std = np.clip(np.std(submap)*20, 0, 1)
        mean = np.clip(np.mean(submap) + 1, 0, 1)
        return np.array([max_dist, std, mean])

    def power(self, image, kernel):
        # Normalize images for better comparison.
        image = (image - image.mean()) / image.std()
        return np.sqrt(
            ndi.convolve(image, np.real(kernel), mode='wrap') ** 2
            + ndi.convolve(image, np.imag(kernel), mode='wrap') ** 2
        )

    def add_feature_elev(self, feature):
        if np.sum(np.isnan(feature)) > 0:
            print('Feature is nan')
            return
        if self.elev_features_.shape[0] == 0:
            print('First feature added')
            self.elev_features_ = np.append(self.elev_features_, feature)
            return
        print('shapes: ', self.elev_features_.shape, feature.shape)
        for feat in self.elev_features_:
            if np.linalg.norm(feat - feature) < self.elev_min_dist_:
                return
            else:
                print(self.elev_features_.shape, feature.shape)
                self.elev_features_ = np.append(
                    self.elev_features_, np.expand_dims(feature, axis=0), axis=0)
                # print('Elev Feature added', self.elev_features_)
                return

    def add_feature_img(self, feature):
        if np.isnan(feature).any():
            return
        if self.features_.shape[0] == 0:
            print('First feature added', self.features_.shape, feature.shape)
            self.features_ = np.append(self.features_, feature, axis=0)
            return
        for feat in self.features_:
            if np.linalg.norm(feat - feature) < self.img_min_dist_:
                return
            else:
                self.features_ = np.append(self.features_, feature, axis=0)
                # print('Feature added', self.features_)
                np.save(self.features_filename_, self.features_)
                return

    def is_feature_img_nav(self, feature, features, threshold):
        min_dist = np.inf
        for feat in features:
            dist = np.linalg.norm(feat - feature)
            if dist < min_dist:
                min_dist = dist
        if min_dist == np.inf:
            min_dist = 999
        return 255 - np.clip(min_dist * (255/threshold), 0, 255)

    def is_feature_elev_nav(self, feature, features, threshold):
        min_dist = np.inf
        for feat in features:
            dist = np.linalg.norm(feat - feature)
            if dist < min_dist:
                min_dist = dist
        if min_dist == np.inf:
            min_dist = 999
        if min_dist < threshold:
            return 255
        else:
            return 0

    def map_layer_to_numpy(self, msg, layer_name):
        layer_index = msg.layers.index(layer_name)
        self.size_x_ = int(msg.info.length_x)
        self.size_y_ = int(msg.info.length_x)
        return np.array(msg.data[layer_index].data).reshape(msg.data[layer_index].layout.dim[0].size, msg.data[layer_index].layout.dim[1].size)

    def map_rgb_layer_to_numpy(self, msg, layer_name):
        layer_index = msg.layers.index(layer_name)
        self.size_x_ = int(msg.info.length_x)
        self.size_y_ = int(msg.info.length_x)
        data = np.array([ctypes.c_uint32.from_buffer(
            ctypes.c_float(val)).value for val in msg.data[layer_index].data])
        return np.array(data).reshape(msg.data[layer_index].layout.dim[0].size, msg.data[layer_index].layout.dim[1].size)
