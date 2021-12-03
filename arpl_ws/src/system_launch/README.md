# system_launch

## Install 

install snap_vio related dependencies from ATLFlight/snap_vio into ~/ws_workspace/src
1. snap_imu
```
$ git clone https://github.com/ATLFlight/snap_imu.git
$ cmr snap_imu
```
2. snap_cpa
```
$ git clone https://github.com/ATLFlight/snap_cpa.git
$ cmr snap_cpa
```

3. qflight_descriptions
```
$ git clone https://github.com/ATLFlight/qflight_descriptions.git
$ cmr qflight_descriptions
```

4. snap_vio
```
$ git clone https://github.com/arplaboratory/snap_vio.git
$ git checkout snapvio_arpl
$ cmr snap_vio
```


## Running
```
$ roslaunch system_launch state_control_use_snapvio.launch
```
