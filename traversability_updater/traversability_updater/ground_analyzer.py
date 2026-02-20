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

import numpy as np

import torch

import torchvision.transforms as transforms

import yaml

from PIL import Image

from scipy import ndimage as ndi

from sklearn.cluster import Birch

import cv2 as cv

from traversability_updater.third_party.PyTorch_VAE.experiment import VAEXperiment

from traversability_updater.third_party.PyTorch_VAE.models import *

from ament_index_python.packages import get_package_share_directory


def hval_to_vector(hval):
    # Convert to 0-360 in radians
    hval = np.deg2rad(hval * 2)
    return [np.cos(hval), np.sin(hval)]


def get_hsv_feat(img):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    h_mean = hval_to_vector(np.mean(hsv[:, :, 0]))
    h_std = hval_to_vector(np.std(hsv[:, :, 0]))
    return [h_mean[0], h_mean[1], h_std[0], h_std[1], np.mean(hsv[:, :, 1]),
            np.std(hsv[:, :, 1]), np.mean(hsv[:, :, 2]), np.std(hsv[:, :, 2])]


def get_sobel_feat(img, wheigth=0.5):
    sob = cv.Sobel(img, cv.CV_32F, 1, 0, ksize=3)
    sob = (sob + 255) / 2
    sob_std_1 = np.std(sob)
    sob = cv.Sobel(img, cv.CV_32F, 0, 1, ksize=3)
    sob = (sob + 255) / 2
    sob_std_2 = np.std(sob)
    return [np.clip((np.abs((max(sob_std_1, sob_std_2) + 1e-9) /
            ((min(sob_std_1, sob_std_2)) + 1e-9)) - 1) * wheigth, 0, 1)]


def get_img_feature_submap(image, max_unkown_, normalize=False):

    if (np.sum(image == 0) >= max_unkown_):
        return np.nan

    image = np.copy(image).astype(np.uint8)

    if (np.sum(image == 0) > 0) and (np.sum(image == 0) < max_unkown_):
        cv.inpaint(image, (np.any(image == 0, axis=2)).astype(
            np.uint8), 3, cv.INPAINT_NS, dst=image)

    if normalize:
        alpha = 255/(np.max(image) - np.min(image))
        beta = -np.min(image)*alpha
        image = cv.convertScaleAbs(
            image, alpha=alpha, beta=beta).astype(np.float32)
    image = image.astype(np.float32) / 255.0
    return np.concatenate(
        [get_sobel_feat(cv.cvtColor(image, cv.COLOR_RGB2GRAY)),
            get_rgb_feat(image), get_hsv_feat(image)])


def get_rgb_feat(img):
    return [np.mean(img[:, :, 0]), np.std(img[:, :, 0]), np.mean(img[:, :, 1]),
            np.std(img[:, :, 1]), np.mean(img[:, :, 2]), np.std(img[:, :, 2])]


def is_feature_img_nav(feature, birch_model, threshold, weights=None,
                       use_birch=True, centroids=None, logvar=None):
    """
    Calcula qué tan cerca está una feature del centroide del subcluster
    asignado por Birch usando birch_model.predict().

    Parámetros:
    - feature: np.array de forma (32,) o (1, 32)
    - birch_model: instancia entrenada de sklearn.cluster.Birch
    - threshold: valor de distancia máxima esperada
    - weights: np.array de pesos opcional de longitud 32

    Retorna:
    - puntuación entre 0 y 255 (mayor = más similar)
    """
    if use_birch:
        # Asegurar forma correcta
        if feature.ndim == 1:
            feature = feature.reshape(1, -1)

        # Aplicar pesos si se han proporcionado
        if weights is not None:
            feature_weighted = feature * weights
            centers_weighted = birch_model.subcluster_centers_ * weights
        else:
            feature_weighted = feature
            centers_weighted = birch_model.subcluster_centers_

        # Obtener el índice del subcluster asignado
        cluster_idx = birch_model.predict(feature)[0]
        cluster_center = centers_weighted[cluster_idx]

        # Calcular distancia euclídea al centro del subcluster asignado
        dist = np.linalg.norm(cluster_center - feature_weighted)
    else:
        # Si no se usa Birch, calcular la distancia al centroide del subcluster
        if logvar is not None:
            var = np.exp(0.5*logvar)
            diff = centroids - feature
            # Avoid division by zero
            var_safe = np.maximum(var, 1e-10)
            # Calculate Mahalanobis distance
            dist = np.sqrt(np.sum((diff ** 2) / var_safe, axis=1))
            dist = np.min(dist)
        else:
            dist = np.linalg.norm(centroids - feature, axis=1)
            dist = np.min(dist)

    # Convertir distancia a puntuación en rango 0–255
    return 255 - np.clip(dist * (255 / threshold), 0, 255)


def is_feature_elev_nav(feature, features, threshold):
    min_dist = np.inf
    for feat in features:
        dist = np.linalg.norm(feat - feature)
        if dist < min_dist:
            min_dist = dist
    if min_dist == np.inf:
        min_dist = 999
    return 255 - np.clip(min_dist * (255/threshold), 0, 255)


def get_heatgrid(score, size=16):
    center = size // 2
    x, y = np.meshgrid(np.arange(size), np.arange(size))
    center_radius = 8
    distance = np.clip(np.sqrt((x - center + 0.5)**2 + (y - center + 0.5)**2) -
                       center_radius, 0, 12 - center_radius)
    # distance = np.sqrt((x - center + 0.5)**2 + (y - center + 0.5)**2)
    min_range = 0.5

    # Normalize the distances to range between 1 and 0.5
    max_distance = np.max(distance)
    min_distance = np.min(distance)
    normalized = 1 - (
        (distance - min_distance) / (max_distance - min_distance)
    ) * (1 - min_range)

    return normalized * score


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

    def __init__(
            self,
            img_mode='HC',
            feat_min_dist=0.05,
            version=0,
            rgbh=False,
            alpha=0.5,
            use_birch=True
    ):
        """Initialize the GroundAnalyzer with the specified image mode.

        Parameters
        ----------
        img_mode : str, optional
            The mode of image analysis ('HC' or 'VAE'), by default 'HC'.
        """
        self.zsize = 64

        self.rgbh = rgbh

        # Distance between features to be considered different
        self.img_min_dist_ = feat_min_dist
        self.elev_min_dist_ = feat_min_dist

        # Step to move the submap (greater step faster, but less accuracy)
        self.recompute_step_ = 2

        self.elev_features_ = np.array([[0, 0, 0]])
        self.max_unkown_ = 32

        self.weights = None

        self.features_filename_ = 'features.npy'

        self.img_model = img_mode

        self.submap_size_ = 16

        self.birch_model = Birch(n_clusters=None,
                                 threshold=self.img_min_dist_,
                                 branching_factor=50)
        self.elev_birch_model = Birch(n_clusters=None,
                                      threshold=self.elev_min_dist_,
                                      branching_factor=50)
        self.features_ = None
        self.var_ = None
        self.use_birch = use_birch

        if img_mode == 'VAE':
            self.transform = transforms.Compose([
                transforms.Resize(16),
                transforms.ToTensor(),
                transforms.GaussianBlur(3, sigma=(5.0, 5.0))
            ])

            pkg_folder = get_package_share_directory('traversability_updater')

            if not self.rgbh:
                # NO ROTATION MODEL LOAD
                folder = pkg_folder + '/checkpoints/VanillaVAE/version_' + str(version) + '/'
                config = yaml.safe_load(open(folder + 'vae_rgb.yaml'))
                model = vae_models[config['model_params']['name']](**config['model_params'])
                ckpt = torch.load(folder + 'last.ckpt')
                print(ckpt['state_dict'].keys())
                self.experiment = VAEXperiment(model, config['exp_params'])
                self.experiment.load_state_dict(ckpt['state_dict'])
                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.experiment.model.to(device)
                self.experiment.model.eval()
                print('VAE loaded')

            else:
                # ROTATION MODEL LOAD
                folder = pkg_folder + '/checkpoints/VanillaVAERGBH/version_' + str(version) + '/'
                config = yaml.safe_load(open(folder + 'vae_rgbh.yaml'))
                model = vae_models[config['model_params']['name']](**config['model_params'])
                ckpt = torch.load(folder + 'last.ckpt')
                missing_keys, unexpected_keys = model.load_state_dict(ckpt['state_dict'], strict=False)
                self.experiment = VAEXperiment(model, config['exp_params'])
                device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
                self.experiment.model.to(device)
                self.experiment.model.eval()
                print('VAE loaded')

            n_feats_rgb = 16
            n_feats_h = 16
            # Create weights based on alpha
            w_rgb = 2 * alpha
            w_h = 2 * (1 - alpha)
            # Create the array with weights
            weights = np.concatenate([
                np.ones(n_feats_rgb) * w_rgb,  # RGB feature weights
                np.ones(n_feats_h) * w_h       # Height feature weights
            ])
            self.weights = weights / weights.sum() * len(weights)

            folder = pkg_folder + '/checkpoints/VanillaVAEH/version_' + str(0) + '/'
            config = yaml.safe_load(open(folder + 'vae_h.yaml'))
            model = vae_models[config['model_params']['name']](**config['model_params'])
            ckpt = torch.load(folder + 'last.ckpt')
            self.elev_experiment = VAEXperiment(model, config['exp_params'])
            self.elev_experiment.load_state_dict(ckpt['state_dict'])
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            self.elev_experiment.model.to(device)
            self.elev_experiment.model.eval()
            print('VAE Elev loaded')

    def my_adjust_contrast(self):
        def _func(img):
            return transforms.functional.adjust_contrast(img,
                                                         contrast_factor=2.0)
        return _func

    def load_features(self, filename=None):
        if filename is None:
            filename = self.features_filename_
        self.features_ = np.load(filename)
        print('Features loaded')

    def insert_sample_elev(self, elev_map):

        # print('Inserting sample elev')
        feature = self.get_elev_features(elev_map)
        self.add_feature_elev(feature)

    def insert_sample_img(self, img_map, normalize=False):

        image = img_map[:self.submap_size_, :self.submap_size_, :]
        # print('Inserting sample')
        feature = get_img_feature_submap(image, self.max_unkown_, normalize)
        self.add_feature_img(np.expand_dims(feature, axis=0))

    def insert_sample_img_elev(self, img_map, elev_map, normalize=False):

        image = img_map[:self.submap_size_, :self.submap_size_, :]
        feature_img = get_img_feature_submap(image, self.max_unkown_,
                                             normalize)
        feature_elev = self.get_elev_features(elev_map)
        if np.isnan(feature_img).any() or np.isnan(feature_elev).any():
            print('Feature is nan')
            return
        feature = np.concatenate((feature_img, feature_elev))
        self.add_feature_img(np.expand_dims(feature, axis=0))

    def insert_sample(self, img_map, map_elev, normalize=False, n_samples=10):

        map_elev = map_elev - np.nanmean(map_elev) + 255/2
        if self.img_model == 'HC+':
            self.insert_sample_img_elev(img_map, map_elev, normalize)
        if self.img_model == 'HC':
            self.insert_sample_img(img_map, normalize)
            self.insert_sample_elev(map_elev)
        if self.img_model == 'VAE':
            self.insert_sample_vae(img_map, map_elev, normalize, n_samples)

    def insert_sample_vae(self,
                          img_map,
                          map_elev,
                          normalize=False,
                          n_samples=10):

        img = img_map[:self.submap_size_, :self.submap_size_, :]

        feature, logvar = self.get_vae_feature_submap(img, map_elev, normalize)
        var = np.exp(0.5*logvar)
        self.add_feature_img(feature, var)

        for i in range(n_samples):
            feat = np.random.normal(feature, var)
            self.add_feature_img(feat)

    def recompute_transversality_img(self, img_map, threshold=1.0):

        print('Recomputing traversality')

        map_size = img_map.shape
        
        nav_map = np.zeros(
            (map_size[0], map_size[1])).astype(np.float32)
        
        if not hasattr(self.birch_model, 'subcluster_centers_'):
            return nav_map

        for i in np.arange(0, img_map.shape[0] - self.submap_size_,
                           self.recompute_step_):
            for j in np.arange(0, img_map.shape[1] - self.submap_size_,
                               self.recompute_step_):
                submap = np.copy(
                    img_map[i:i+self.submap_size_, j:j+self.submap_size_, :])
                if np.sum(submap == 0) < self.max_unkown_:
                    if (np.sum(submap == 0) > 0):
                        cv.inpaint(
                            submap,
                            (np.any(submap == 0, axis=2)).astype(np.uint8),
                            3,
                            cv.INPAINT_NS,
                            dst=submap
                        )

                    img_features = get_img_feature_submap(submap, self.max_unkown_, normalize=False)

                    if (np.sum(np.isnan(submap)) > 0):
                        print('Feature is navigable')
                    nav_map[i:i+self.submap_size_, j:j+self.submap_size_] += (
                        (is_feature_img_nav(img_features, self.birch_model,
                                                 threshold), self.weights, self.use_birch, self.features_)
                        * ((self.recompute_step_ / self.submap_size_) ** 2)
                    )

        return nav_map

    def recompute_transversality_img_elev(self,
                                          img_map,
                                          elev_map,
                                          threshold=1.0):

        print('Recomputing traversality')

        map_size = img_map.shape
        
        nav_map = np.zeros(
            (map_size[0], map_size[1])).astype(np.float32)
        
        if not hasattr(self.birch_model, 'subcluster_centers_'):
            return nav_map

        for i in np.arange(0, img_map.shape[0] - self.submap_size_,
                           self.recompute_step_):
            for j in np.arange(0, img_map.shape[1] - self.submap_size_,
                               self.recompute_step_):
                submap = np.copy(
                    img_map[i:i+self.submap_size_, j:j+self.submap_size_, :])
                submap_elev = np.copy(
                    elev_map[i:i+self.submap_size_, j:j+self.submap_size_])
                if np.sum(submap == 0) < self.max_unkown_:
                    if (np.sum(submap == 0) > 0):
                        cv.inpaint(
                            submap,
                            (np.any(submap == 0, axis=2)).astype(np.uint8),
                            3,
                            cv.INPAINT_NS,
                            dst=submap
                        )

                    feature_img = get_img_feature_submap(submap, self.max_unkown_, normalize=False)

                    feature_elev = self.get_elev_features(submap_elev)

                    if np.isnan(feature_img).any() or np.isnan(feature_elev).any():
                        continue

                    feature = np.concatenate((feature_img, feature_elev))

                    nav_map[i:i+self.submap_size_, j:j+self.submap_size_] += (
                        (is_feature_img_nav(feature, self.birch_model, threshold, self.weights, self.use_birch, self.features_))
                        * ((self.recompute_step_ / self.submap_size_) ** 2)
                    )

        return nav_map
    
    def recompute_transversality(self,
                                 img_map,
                                 elev_map,
                                 threshold=0.3,
                                 alpha=0.5):

        if self.img_model == 'HC+':
            return self.recompute_transversality_img_elev(img_map, elev_map, threshold)
        if self.img_model == 'HC':
            img_trav = self.recompute_transversality_img(img_map, threshold)
            elev_trav = self.recompute_transversality_elev(elev_map, threshold)
            return (img_trav * alpha + elev_trav * (1 - alpha))
        if self.img_model == 'VAE':
            return self.recompute_transversality_vae(img_map, elev_map, threshold)
    
    def recompute_transversality_elev(self, elev_map, threshold=0.3):

        print('Recomputing Elev traversality')

        map_size = elev_map.shape

        nav_map_elev = np.zeros(
            (map_size[0], map_size[1])).astype(np.float32)
        
        if not hasattr(self.elev_birch_model, 'subcluster_centers_'):
            return nav_map_elev

        for i in np.arange(0, elev_map.shape[0] - self.submap_size_, self.recompute_step_):
            for j in np.arange(0, elev_map.shape[1] - self.submap_size_, self.recompute_step_):
                submap = np.copy(
                    elev_map[i:i+int(self.submap_size_/2), j:j+int(self.submap_size_/2)])
                if np.sum(np.isnan(submap)) < self.max_unkown_/2:

                    elev_features = self.get_elev_features(submap)
                    # submap_print = np.nan_to_num(submap, nan=0)
                    # if (np.max(submap_print) > 10):
                        # print('Submap stats: ', np.mean(submap_print), np.max(submap_print), np.min(submap_print), submap_print.shape)
                    nav_map_elev[i:i+int(self.submap_size_/2), j:j+int(self.submap_size_/2)] += ((is_feature_elev_nav(
                        elev_features, self.elev_birch_model.subcluster_centers_, threshold)))*((self.recompute_step_*2/self.submap_size_)**2)

        return nav_map_elev

    def recompute_transversality_vae(self, map_rgb, map_elev, threshold=1.5):

        mode = 'RGB'
        n_channels = 3
        if self.rgbh:
            mode = 'RGBA'
            n_channels = 4

        print('Recomputing traversality VAE')

        nav_map = np.zeros(
            (map_rgb.shape[0], map_rgb.shape[1])).astype(np.float32)
        
        if not hasattr(self.birch_model, 'subcluster_centers_') and self.use_birch:
            print('WARNING: No subcluster added')
            return nav_map
        if self.features_ is None and not self.use_birch:
            print('WARNING: No subcluster added')
            return nav_map

        # map_rgb = map_rgb.astype(np.float32) / 127.5 - 1
        # map_elev = np.clip(map_elev - map_elev.mean(), -1, 1)
        map = np.concatenate((map_rgb, np.expand_dims(map_elev, axis=-1)), axis=-1)

        img_tensor = img_tensor = torch.zeros(
            0, n_channels, self.submap_size_, self.submap_size_)
        ij_array = []
        submap_list = []  # List to store the submap tensors
        elev_img_tensor = torch.zeros(0, 1, self.submap_size_, self.submap_size_)
        elev_img_list = []  # List to store the submap tensors

        for i in np.arange(0, map.shape[0] - self.submap_size_, self.recompute_step_):
            for j in np.arange(0, map.shape[1] - self.submap_size_, self.recompute_step_):
                submap = np.copy(
                    map[i:i+self.submap_size_, j:j+self.submap_size_, :n_channels]).astype(np.uint8)
                submap_elev = np.copy(
                    map_elev[i:i+self.submap_size_, j:j+self.submap_size_])
                submap_elev = submap_elev - np.nanmean(submap_elev) + 255/2
                submap_elev = np.nan_to_num(submap_elev, nan=0)[
                    :self.submap_size_, :self.submap_size_]
                if np.sum(np.all(submap == [0, 0, 0], axis=-1)) < self.max_unkown_:
                    if np.sum(np.all(submap == [0, 0, 0], axis=-1)) > 0:
                        cv.inpaint(
                            submap,
                            (np.any(submap == 0, axis=2)).astype(np.uint8),
                            3,
                            cv.INPAINT_NS,
                            dst=submap
                        )
                    if (np.sum(submap_elev == 0) > 0):
                        cv.inpaint(
                            submap_elev,
                            (submap_elev == 0).astype(np.uint8),
                            3,
                            cv.INPAINT_NS,
                            dst=submap_elev
                        )
                    submap = Image.fromarray(submap.astype(np.uint8), mode=mode)
                    submap_tensor = self.transform(submap).unsqueeze(0)
                    submap_list.append(submap_tensor)
                    submap_elev = Image.fromarray(submap_elev.astype(np.uint8), mode='L')
                    submap_elev_tensor = self.transform(submap_elev).unsqueeze(0)
                    elev_img_list.append(submap_elev_tensor)

                    ij_array.append([i, j])

        # if submap_list & elev_img_list:
        img_tensor = torch.cat(submap_list, dim=0)
        elev_img_tensor = torch.cat(elev_img_list, dim=0)

        print('map segmentes array completed ', len(ij_array))

        with torch.no_grad():
            feats_vae, logvar_vae = self.experiment.model.encode(img_tensor.cuda())
            feats_vae = feats_vae.cpu().detach().numpy()
            logvar_vae = logvar_vae.cpu().detach().numpy()

        torch.cuda.empty_cache()

        # VAE elev ---

        # print('VAE elev img_tensor minmax', torch.min(elev_img_tensor), torch.max(elev_img_tensor))
        with torch.no_grad():
            feats_vae_elev, logvar_vae_elev = self.elev_experiment.model.encode(elev_img_tensor.cuda())
            feats_vae_elev = feats_vae_elev.cpu().detach().numpy()
            logvar_vae_elev = logvar_vae_elev.cpu().detach().numpy()

        torch.cuda.empty_cache()

        feats_vae = np.concatenate((feats_vae, feats_vae_elev), axis=-1)
        logvar_vae = np.concatenate((logvar_vae, logvar_vae_elev), axis=-1)

        nav_map = np.zeros((map.shape[0], map.shape[1])).astype(np.float32)
        ij = 0
        for i, j in ij_array:
            
            score = is_feature_img_nav(
                feats_vae[ij, :],
                self.birch_model,
                threshold,
                weights=self.weights,
                use_birch=self.use_birch,
                centroids=self.features_,
                # logvar = logvar_vae[ij, :],
                logvar = None,
                )

            nav_map[i:i+self.submap_size_, j:j+self.submap_size_] = np.maximum(
                nav_map[i:i+self.submap_size_, j:j+self.submap_size_],
                get_heatgrid(score))
            
            ij += 1
        print('Recomputing traversality VAE Finished')
        return nav_map
    

    def get_vae_feature_submap(self, img, submap_elev, normalize=False):

        mode = 'RGB'
        n_channels = 3

        img = np.copy(img).astype(np.uint8)

        if (np.sum(np.sum(img, axis=2) == 0) >= self.max_unkown_):
            return np.nan, np.nan

        if ((np.sum(np.sum(img, axis=2) == 0) > 0) and (np.sum(np.sum(img, axis=2) == 0) < self.max_unkown_)):
            cv.inpaint(img, (np.any(img == 0, axis=2)).astype(
                np.uint8), 3, cv.INPAINT_NS, dst=img)
            

        # Temporal, should be inpainted
        submap_elev = np.nan_to_num(submap_elev, nan=0)[
            :self.submap_size_, :self.submap_size_]
        if ((np.sum(submap_elev == 0) > 0) and (np.sum(submap_elev == 0) < self.max_unkown_)):
            cv.inpaint(submap_elev, (submap_elev == 0).astype(
                np.uint8), 3, cv.INPAINT_NS, dst=submap_elev)
            
        submap_elev = np.clip(submap_elev, 0, 255).astype(np.uint8)
        img = img.astype(np.uint8)

        if self.rgbh:
            if len(submap_elev.shape) < 3:
                submap_elev = np.expand_dims(submap_elev, axis=-1)
            img = np.concatenate((img, submap_elev), axis=-1)
            mode = 'RGBA'
            n_channels = 4

        img_tensor = torch.zeros(0, n_channels, self.submap_size_, self.submap_size_)

        img = Image.fromarray(img, mode=mode)
        img = self.transform(img).unsqueeze(0)
        img_tensor = torch.cat([img_tensor, img], dim=0)

        with torch.no_grad():
            feats_vae, logvar_vae = self.experiment.model.encode(img_tensor.cuda())
            feats_vae = feats_vae.cpu().detach().numpy()
            logvar_vae = logvar_vae.cpu().detach().numpy()

        elev_img_tensor = torch.zeros(0, 1, self.submap_size_, self.submap_size_)
        elev_img = Image.fromarray(submap_elev[:, :, 0], mode='L')
        elev_img = self.transform(elev_img).unsqueeze(0)
        elev_img_tensor = torch.cat([elev_img_tensor, elev_img], dim=0)

        with torch.no_grad():
            feats_vae_elev, logvar_vae_elev = self.elev_experiment.model.encode(elev_img_tensor.cuda())
            feats_vae_elev = feats_vae_elev.cpu().detach().numpy()
            logvar_vae_elev = logvar_vae_elev.cpu().detach().numpy()

        feats_vae = np.concatenate((feats_vae, feats_vae_elev), axis=-1)
        logvar_vae = np.concatenate((logvar_vae, logvar_vae_elev), axis=-1)

        return feats_vae, logvar_vae

    def get_elev_features(self, submap):

        if (np.sum(np.isnan(submap)) >= self.max_unkown_):
            return np.nan

        if (np.sum(np.isnan(submap)) > 0) and (np.sum(np.isnan(submap)) < self.max_unkown_):
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
        feature = np.expand_dims(feature, axis=0)
        self.elev_birch_model.partial_fit(feature)

    def add_feature_img(self, feature, var=None):
        if np.isnan(feature).any():
            print('Feature is nan', feature)
            return
        if self.use_birch:
            self.birch_model.partial_fit(feature)
        else:
            if self.features_ is None:
                self.features_ = feature
                if var is not None:
                    self.var_ = None
            else:
                self.var_ = None
                var = None
                # Efficiently compute distances to all existing features
                # Calculate Mahalanobis distance if variance is available
                if self.var_ is not None:
                    diff = self.features_ - feature
                    var_safe = np.maximum(self.var_, 1e-10)
                    dists = np.sqrt(np.sum((diff ** 2) / var_safe, axis=1))
                else:
                    dists = np.linalg.norm(self.features_ - feature, axis=1)
                if np.all(dists > self.img_min_dist_):
                    self.features_ = np.vstack([self.features_, feature])
                    if var is not None:
                        self.var_ = np.vstack([self.var_, var])
