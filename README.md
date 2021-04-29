# VisualDet3D_ros
VisualDet3D_ros is a project to build a ros interface for [VisualDet3D](https://github.com/Owen-Liuyuxuan/visualDet3D).
## Training VisualDet3D
### 1. Prerequisites
1.1 **Ubuntu** 18.04 and **ROS** Melodic.

1.2 [**anacoda3**](https://docs.anaconda.com/anaconda/install/linux/) to manager package and deployment.

1.3 build anaconda ```visualDet3D``` environment.
```bash
conda create --name visualDet3d python=3.6
conda activative visualDet3d
pip3 install -r requirement.txt
```

### 2. training
Follow the [mono3d](https://github.com/Owen-Liuyuxuan/visualDet3D/blob/master/docs/mono3d.md) tutorials to train the network using KITTI object dataset.
### 3. testing our dataset
3.1 Edit variance ```path.test_path``` in the ```config/config.py``` file.

3.2 The results are shown in ```workdirs/Mono3D/output/test/data``` folder.