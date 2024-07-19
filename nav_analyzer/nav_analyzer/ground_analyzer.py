import torch.utils.data
import numpy as np
import torch
from PIL import Image
import torchvision.transforms as transforms
from net import *
import ctypes

from skimage.filters import gabor_kernel
from scipy import ndimage as ndi
import cv2 as cv

import numpy as np


class GroundAnalyzer():
    def __init__(self, img_mode='gabor'):

        self.zsize = 256

        self.elev_features_ = np.empty((0,0))
        self.elev_threshold_ = 0.2

        self.img_model = img_mode

        if self.img_model == 'gabor':
            self.features_ = np.empty((0, 11))
        if self.img_model == 'vae':
            self.features_ = np.empty((0, self.zsize))
        self.threshold_ = 0.1

        self.feat_std = 0.2

        self.submap_size_ = 16

        self.resolution_ = 0.2
        self.size_x_ = 500
        self.size_y_ = 500

        self.transform = transforms.Compose([transforms.Resize([self.submap_size_, self.submap_size_]), transforms.ToTensor()])

        self.vae = VAE(zsize=self.zsize, layer_count=2)
        self.vae.load_state_dict(torch.load("/mnt/fedora/home/migueldm/Documentos/VAE/VAEmodel_16_z128.pkl"))
        self.vae.cuda()
        self.vae.eval()
        print('VAE loaded')

        s = 3
        self.kernels = [gabor_kernel(frequency=0.1, theta=theta, sigma_x=s, sigma_y=s) for theta in [0, np.pi/4, np.pi/2, 3*np.pi/4]]

        self.n = 0
        self.nn = 0
    
    def insert_sample(self, grid_map, pose):
        
        self.resolution_ = grid_map.info.resolution
        map = self.map_layer_to_numpy(grid_map, 'elevation')
        feature = self.get_features(map, pose)
        self.add_feature(feature)
    
    def insert_sample_img(self, grid_map, pose):
        
        self.resolution_ = grid_map.info.resolution
        map = self.map_rgb_layer_to_numpy(grid_map, 'RGB')
        feature = self.get_feature_img(map, pose)
        self.add_feature_img(feature)
    


    def calculate_elev_feat(self, submap):
        return np.nanstd(submap)
    
    def check_feature(self, feature):
        for feat in self.elev_features_:
            if np.abs(feat - feature) < self.elev_threshold_:
                return 255
        return 0
    

    def recompute_transversality(self, msg):

        step = 1
        print('Recomputing transversality')
        layer_name = 'elevation'
        layer_index = msg.layers.index(layer_name)
        elev_map = self.map_layer_to_numpy(msg, layer_name)
        map = np.zeros((msg.data[layer_index].layout.dim[0].size, msg.data[layer_index].layout.dim[1].size))
        for i in np.arange(0, msg.data[layer_index].layout.dim[0].size - self.submap_size_, step):
            for j in np.arange(0, msg.data[layer_index].layout.dim[1].size - self.submap_size_, step):
                submap = elev_map[i:i+self.submap_size_, j:j+self.submap_size_]
                if(np.sum(~np.isnan(submap)) > 1):
                    feature = self.calculate_elev_feat(submap)
                    navegability = self.check_feature(feature)
                    map[i:i+self.submap_size_, j:j+self.submap_size_] += navegability * ((step/self.submap_size_)**2)

        print('Done!')

        # Normalize map values between 0 and 255
        map = (map - np.min(map))/(np.max(map) - np.min(map))*255


        return map.flatten().tolist()
    
    def recompute_transversality_img(self, msg):

        step = int(self.submap_size_ / 4)


        print('Recomputing transversality')
        layer_name = 'RGB'
        layer_index = msg.layers.index(layer_name)
        img_map = self.map_rgb_layer_to_numpy(msg, layer_name).astype(np.uint32)
        map = np.zeros((msg.data[layer_index].layout.dim[0].size, msg.data[layer_index].layout.dim[1].size))

        if self.img_model == 'gabor':
            for i in np.arange(0, msg.data[layer_index].layout.dim[0].size - self.submap_size_, step):
                # print(i)
                for j in np.arange(0, msg.data[layer_index].layout.dim[1].size - self.submap_size_, step):
                    submap = img_map[i:i+self.submap_size_, j:j+self.submap_size_]
                    # print(i, j, np.sum(np.isnan(submap)), np.sum(submap))
                    if((np.sum(np.isnan(submap)) == 0) and (np.sum(submap) >= 0.1)):
                        image = self.get_rgb_image(submap)
                        img_features = np.concatenate((np.array(self.gabor(cv.cvtColor(image, cv.COLOR_RGB2GRAY), self.kernels)[0]), self.mean_color_img(image)), axis=0)[np.newaxis, :]
                        map[i:i+self.submap_size_, j:j+self.submap_size_] += ((self.is_feature_img_nav(img_features, 0.15)))*((step/self.submap_size_)**2)


        if self.img_model == 'vae':

            img_tensor = img_tensor = torch.zeros(0, 3, self.submap_size_, self.submap_size_)
            tensor_pose_array = []
            print('N Feats: ', self.features_.shape)
            
            for i in np.arange(0, msg.data[layer_index].layout.dim[0].size - self.submap_size_, step):
                # print(i)
                for j in np.arange(0, msg.data[layer_index].layout.dim[1].size - self.submap_size_, step):
                    submap = img_map[i:i+self.submap_size_, j:j+self.submap_size_]
                    # print(i, j, np.sum(np.isnan(submap)), np.sum(submap))
                    if((np.sum(np.isnan(submap)) == 0) and (np.sum(submap) >= 0.1)):

                        # print('is not nan', np.sum(submap))

                        image = self.get_rgb_image(submap)
                        # plt.imsave('/home/migueldm/Documents/img_debug2/' + str(self.nn).zfill(5) + '.png', image)
                        # self.nn += 1
                        image = Image.fromarray(image.astype('uint8'))
                        image = self.transform(image)
                        img_tensor = torch.cat([image[None, :, :, :], img_tensor], dim=0)
                        tensor_pose_array.append([i, j])
            

            feats = self.vae.encode(img_tensor.cuda())
            all_features = np.flip(feats[0].cpu().detach().numpy()) # Inference output is flipped
            
            for i, pose in enumerate(tensor_pose_array):
                
                map[pose[0]:pose[0]+self.submap_size_, pose[1]:pose[1]+self.submap_size_] += ((self.is_feature_img_nav(all_features[i], 7)))*((step/self.submap_size_)**2)
            

        print('Done!', np.max(map), np.min(map))

       

        # # Normalize map values between 0 and 255
        # map = (map - np.min(map))/(np.max(map) - np.min(map))*255


        return map.flatten().tolist()
    
    def get_rgb_image(self, submap):
        # Get 3 dims RGB image from submap
        return np.stack(((submap & 255), ((submap >> 8) & 255), (submap >> 16) & 255), axis=-1).astype(np.uint8)

    
    def get_features(self, elev_map, pose):
        pose_center = np.array(pose) / self.resolution_
        center = [int((self.size_x_ / 2) / self.resolution_), int((self.size_y_ / 2) / self.resolution_)]
        
        submap = elev_map[center[1] - int(pose_center[1] + self.submap_size_ / 2):1 + center[1] - int(pose_center[1] - self.submap_size_ / 2), center[0] - int(pose_center[0] + self.submap_size_ / 2):1 + center[0] - int(pose_center[0] - self.submap_size_ / 2)]

        return self.calculate_elev_feat(submap)
    
    def power(self, image, kernel):
        # Normalize images for better comparison.
        image = (image - image.mean()) / image.std()
        return np.sqrt(
            ndi.convolve(image, np.real(kernel), mode='wrap') ** 2
            + ndi.convolve(image, np.imag(kernel), mode='wrap') ** 2
        )


    def gabor(self, image, kernels):
        # Prepare the output
        feats = []
        debug_imgs = []
        for k, kernel in enumerate(kernels):
            # filtered = ndi.convolve(image, np.real(kernel), mode='wrap')
            # plt.imshow(image, cmap='gray')
            # plt.show()
            filtered = self.power(image, kernel)
            # plt.imshow(filtered, cmap='gray')
            # plt.show()
            feats.append(np.mean(filtered))
            feats.append(np.std(filtered))
            debug_imgs.append(filtered)
            
        return np.array(feats), debug_imgs
    
    def mean_color_img(self, image):
        R = np.mean(image[:,:,0])
        G = np.mean(image[:,:,1])
        B = np.mean(image[:,:,2])

        total = R + G + B

        # Normalize the values
        R = R / total
        G = G / total
        B = B / total

        return [R, G, B]
    
    def get_feature_img(self, rgb_map, pose):
        pose_center = np.array(pose) / self.resolution_
        center = [int((self.size_x_ / 2) / self.resolution_), int((self.size_y_ / 2) / self.resolution_)]
        submap = rgb_map[center[1] - int(pose_center[1] + self.submap_size_ / 2):1 + center[1] - int(pose_center[1] - self.submap_size_ / 2), center[0] - int(pose_center[0] + self.submap_size_ / 2):1 + center[0] - int(pose_center[0] - self.submap_size_ / 2)]


        image = self.get_rgb_image(submap)
        
        if self.img_model == 'vae':
            image = Image.fromarray(image.astype('uint8'))

            img_tensor = self.transform(image)
            img_tensor = img_tensor[None, :, :, :]

            return self.vae.encode(img_tensor.cuda())[0].cpu().detach().numpy()
        
        elif self.img_model == 'gabor':
            img_features = np.concatenate((np.array(self.gabor(cv.cvtColor(image, cv.COLOR_RGB2GRAY), self.kernels)[0]), self.mean_color_img(image)), axis=0)[np.newaxis, :]
            return img_features
    
    def add_feature(self, feature):
        if feature != feature:
            print('Feature is nan')
            return
        if self.elev_features_.shape[0] == 0:
            print('First feature added')
            self.elev_features_ = np.append(self.elev_features_, feature)
            return
        for feat in self.elev_features_:
            if np.abs(feat - feature) < self.elev_threshold_:
                return
            else:
                self.elev_features_ = np.append(self.elev_features_, feature)
                return
        
    def is_feature_img_nav(self, feature, threshold):
        dist_arr = []
        for feat in self.features_:
            dist = np.sqrt(np.sum((feat - feature)**2))
            dist_arr.append(dist)
            if dist < threshold:
                # print('Feature found ', dist)
                return 255

        # print('No feature found ', np.min(dist_arr), )
        # print(feature)
        # print(self.features_.shape, self.features_[0])
        return 0

    def add_feature_img(self, feature):
        if np.isnan(feature).any():
            return
        if self.features_.shape[0] == 0:
            print('First feature added', self.features_.shape, feature.shape)
            self.features_ = np.append(self.features_, feature, axis=0)
            return
        for feat in self.features_:
            if np.sqrt(np.sum((feat - feature)**2)) < self.threshold_:
                return
            else:
                self.features_ = np.append(self.features_, feature, axis=0)
                print('Feature added', self.features_)
                return

    def map_layer_to_numpy(self, msg, layer_name):
        layer_index = msg.layers.index(layer_name)
        self.size_x_ = int(msg.info.length_x)
        self.size_y_ = int(msg.info.length_x)
        return np.array(msg.data[layer_index].data).reshape(msg.data[layer_index].layout.dim[0].size, msg.data[layer_index].layout.dim[1].size)


    def map_rgb_layer_to_numpy(self, msg, layer_name):
        layer_index = msg.layers.index(layer_name)
        self.size_x_ = int(msg.info.length_x)
        self.size_y_ = int(msg.info.length_x)
        data = np.array([ctypes.c_uint32.from_buffer(ctypes.c_float(val)).value for val in msg.data[layer_index].data])
        return np.array(data).reshape(msg.data[layer_index].layout.dim[0].size, msg.data[layer_index].layout.dim[1].size)

