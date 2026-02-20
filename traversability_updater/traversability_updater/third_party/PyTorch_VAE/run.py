import os
import yaml
import argparse

from pathlib import Path
from traversability_updater.third_party.PyTorch_VAE.models import *
from experiment import VAEXperiment

from pytorch_lightning import Trainer
from pytorch_lightning.loggers import TensorBoardLogger
from pytorch_lightning import seed_everything
from pytorch_lightning.callbacks import LearningRateMonitor, ModelCheckpoint
from dataset import VAEDataset
from pytorch_lightning.strategies import DDPStrategy

parser = argparse.ArgumentParser(description='Generic runner for VAE models')
parser.add_argument('--config',  '-c',
                    dest="filename",
                    metavar='FILE',
                    help='path to the config file',
                    default='configs/vae.yaml')

args = parser.parse_args()
with open(args.filename, 'r') as file:
    try:
        config = yaml.safe_load(file)
    except yaml.YAMLError as exc:
        print(exc)

tb_logger = TensorBoardLogger(save_dir=config['logging_params']['save_dir'],
                              name=config['model_params']['name'])

# For reproducibility
seed_everything(config['exp_params']['manual_seed'], True)

model = vae_models[config['model_params']['name']](**config['model_params'])
experiment = VAEXperiment(model, config['exp_params'])

# Extract GPU settings and setup DataLoader accordingly
trainer_params = config['trainer_params']
if 'gpus' in trainer_params:
    gpus = trainer_params.pop('gpus')
    trainer_params['accelerator'] = 'gpu'
    trainer_params['devices'] = gpus
    pin_memory = True if gpus else False
else:
    pin_memory = False

data = VAEDataset(**config["data_params"], pin_memory=pin_memory)
data.setup()

runner = Trainer(logger=tb_logger,
                 callbacks=[
                     LearningRateMonitor(),
                     ModelCheckpoint(save_top_k=2,
                                     dirpath=os.path.join(tb_logger.log_dir, "checkpoints"),
                                     monitor="val_loss",
                                     save_last=True),
                 ],
                 strategy=DDPStrategy(find_unused_parameters=False),
                 **trainer_params)

Path(f"{tb_logger.log_dir}/Samples").mkdir(exist_ok=True, parents=True)
Path(f"{tb_logger.log_dir}/Reconstructions").mkdir(exist_ok=True, parents=True)

print(f"======= Training {config['model_params']['name']} =======")
runner.fit(experiment, datamodule=data)