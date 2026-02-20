import torch
from traversability_updater.third_party.PyTorch_VAE.models import BaseVAE
from torch import nn
from torch.nn import functional as F
import torchvision.transforms.functional as TF
from .types_ import *
from torchvision import transforms
from e2cnn import gspaces
from e2cnn import nn as enn


class VanillaVAERot(BaseVAE):
    def __init__(self,
                 in_channels: int,
                 latent_dim: int,
                 hidden_dims: List = None,
                 **kwargs) -> None:
        super(VanillaVAERot, self).__init__()

        self.in_channels = in_channels
        self.latent_dim = latent_dim

        # --- Definimos el grupo de simetría rotacional discreta ---
        self.r2_act = gspaces.Rot2dOnR2(N=4)  # rotaciones de 0°, 90°, 180°, 270°

        modules = []
        if hidden_dims is None:
            hidden_dims = [32, 64, 128]

        # --- Build Encoder usando e2cnn ---
        in_type = enn.FieldType(self.r2_act, in_channels * [self.r2_act.trivial_repr])
        for h_dim in hidden_dims:
            out_type = enn.FieldType(self.r2_act, h_dim * [self.r2_act.regular_repr])
            modules.append(
                enn.SequentialModule(
                    enn.R2Conv(in_type, out_type, kernel_size=3, stride=2, padding=1),
                    enn.InnerBatchNorm(out_type),
                    enn.ReLU(out_type, inplace=True)
                )
            )
            in_type = out_type  # para la siguiente capa

        self.encoder = enn.SequentialModule(*modules)

        # Al final del encoder guardamos el tipo de salida
        self.final_out_type = out_type

        # Suponemos que la salida tiene tamaño 2x2 (por stride=2 en cada capa)
        self.flatten_dim = hidden_dims[-1] * 4 * 4  # 2x2 * channels
        self.fc_mu = nn.Linear(self.flatten_dim, latent_dim)
        self.fc_var = nn.Linear(self.flatten_dim, latent_dim)

        self.transforms = transforms.Compose([
            transforms.GaussianBlur(3, sigma=(5.0, 5.0)),
        ])

        # --- Decoder normal ---
        modules = []
        self.decoder_input = nn.Linear(latent_dim, hidden_dims[-1] * 4)
        hidden_dims.reverse()

        for i in range(len(hidden_dims) - 1):
            modules.append(
                nn.Sequential(
                    nn.ConvTranspose2d(hidden_dims[i],
                                       hidden_dims[i + 1],
                                       kernel_size=3,
                                       stride=2,
                                       padding=1,
                                       output_padding=1),
                    nn.BatchNorm2d(hidden_dims[i + 1]),
                    nn.LeakyReLU())
            )

        self.decoder = nn.Sequential(*modules)

        self.final_layer = nn.Sequential(
            nn.ConvTranspose2d(hidden_dims[-1],
                               hidden_dims[-1],
                               kernel_size=3,
                               stride=2,
                               padding=1,
                               output_padding=1),
            nn.BatchNorm2d(hidden_dims[-1]),
            nn.LeakyReLU(),
            nn.Conv2d(hidden_dims[-1], out_channels=self.in_channels,
                      kernel_size=3, padding=1),
            nn.Tanh()
        )

    def encode(self, input: Tensor) -> List[Tensor]:
        """
        Encode input tensor using e2cnn encoder and return mu, log_var
        """
        # Envolver entrada como GeometricTensor
        x = enn.GeometricTensor(input, enn.FieldType(self.r2_act, self.in_channels * [self.r2_act.trivial_repr]))

        # Encoder con e2cnn
        x = self.encoder(x)

        # Obtener tensor normal y aplanarlo
        x_flat = x.tensor.flatten(start_dim=1)

        mu = self.fc_mu(x_flat)
        log_var = self.fc_var(x_flat)

        return [mu, log_var]

    def decode(self, z: Tensor) -> Tensor:
        """
        Maps the given latent codes onto the image space.
        :param z: (Tensor) [B x D]
        :return: (Tensor) [B x C x H x W]
        """
        result = self.decoder_input(z)
        
        # Determinar cuántos canales tiene la primera capa del decoder
        decoder_first_channels = self.decoder[0][0].in_channels  # tipo nn.ConvTranspose2d
        result = result.view(-1, decoder_first_channels, 2, 2)
        
        result = self.decoder(result)
        result = self.final_layer(result)
        return result

    def reparameterize(self, mu: Tensor, logvar: Tensor) -> Tensor:
        """
        Reparameterization trick to sample from N(mu, var) from
        N(0,1).
        :param mu: (Tensor) Mean of the latent Gaussian [B x D]
        :param logvar: (Tensor) Standard deviation of the latent Gaussian [B x D]
        :return: (Tensor) [B x D]
        """
        std = torch.exp(0.5 * logvar)
        eps = torch.randn_like(std)
        return eps * std + mu

    def forward(self, input: Tensor, **kwargs) -> List[Tensor]:
        original_input = input.clone()
        # input = self.transforms(input)
        mu, log_var = self.encode(input)
        z = self.reparameterize(mu, log_var)
        return  [self.decode(z), original_input, mu, log_var]

    def loss_function(self,
                      *args,
                      **kwargs) -> dict:
        """
        Computes the VAE loss function.
        KL(N(\mu, \sigma), N(0, 1)) = \log \frac{1}{\sigma} + \frac{\sigma^2 + \mu^2}{2} - \frac{1}{2}
        :param args:
        :param kwargs:
        :return:
        """
        recons = args[0]
        input = args[1]
        mu = args[2]
        log_var = args[3]

        kld_weight = kwargs['M_N'] # Account for the minibatch samples from the dataset
        recons_loss =F.mse_loss(recons, input)


        kld_loss = torch.mean(-0.5 * torch.sum(1 + log_var - mu ** 2 - log_var.exp(), dim = 1), dim = 0)

        loss = recons_loss + kld_weight * kld_loss
        return {'loss': loss, 'Reconstruction_Loss':recons_loss.detach(), 'KLD':kld_loss.detach()}

    def sample(self,
               num_samples:int,
               current_device: int, **kwargs) -> Tensor:
        """
        Samples from the latent space and return the corresponding
        image space map.
        :param num_samples: (Int) Number of samples
        :param current_device: (Int) Device to run the model
        :return: (Tensor)
        """
        z = torch.randn(num_samples,
                        self.latent_dim)

        z = z.to(current_device)

        samples = self.decode(z)
        return samples

    def generate(self, x: Tensor, **kwargs) -> Tensor:
        """
        Given an input image x, returns the reconstructed image
        :param x: (Tensor) [B x C x H x W]
        :return: (Tensor) [B x C x H x W]
        """

        return self.forward(x)[0]