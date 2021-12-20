
## Dependencies

- [`YARP`](https://github.com/robotology/YARP)
- `WearableActuators` from [`wearables`](https://github.com/robotology/wearables)

## Build

```bash
WEIGHT_RETARGETING_DIR=<select-your-directory>
git clone https://github.com/ami-iit/element_weight-retargeting.git ${WEIGHT_RETARGETING_DIR}
mkdir ${WEIGHT_RETARGETING_DIR}/build && cd ${WEIGHT_RETARGETING_DIR}/build
cmake .. -DCMAKE_INSTALL_PREFIX="./install" -DCMAKE_BUILD_TYPE="Release"
make install
```

## Configure the environment 

Once the installation is completed, make the WeightRetargetingModule visible:

### Linux
Append the following lines to your `.bashrc`:

```bash
export YARP_DATA_DIRS=$YARP_DATA_DIRS:<weight-retargeting-directory>/build/install/share
export PATH=$PATH:<weight-retargeting-directory>/build/install/bin
```
