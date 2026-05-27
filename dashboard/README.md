# Go2 RTAB-Map Control Dashboard

Static dashboard and Python backend based on `html.pen`.

## Run

Static preview can be opened directly in a browser:

```bash
xdg-open dashboard/index.html
```

For real ROS2 control, run the backend after sourcing ROS and this workspace:

```bash
cd dashboard
source /opt/ros/humble/setup.bash
source ../install/setup.bash
python3 server.py --host 127.0.0.1 --port 8080
```

Open `http://127.0.0.1:8080`.

The backend redirects `/` to `/?api=1` so the frontend polls `/api/status`.

For static preview without ROS:

```bash
cd dashboard
python3 -m http.server 8080
```

Open `http://127.0.0.1:8080`.

## API

The frontend calls these endpoints:

- `GET /api/status`
- `POST /api/mapping/start`
- `POST /api/mapping/stop`
- `POST /api/localization/start`
- `POST /api/localization/pose`
- `POST /api/localization/stop`
- `POST /api/correction/align`
- `POST /api/correction/lock`

`GET /api/status` may return:

```json
{
  "rtabmap": {
    "proximityId": 55,
    "loopClosureId": 0
  },
  "tf": {
    "status": "map->odom live",
    "lastSeenSec": 0.2
  },
  "poseStream": "/rtabmap/localization_pose<br>map frame · 1 Hz",
  "correctionEnabled": true
}
```

`POST /api/mapping/start` sends:

```json
{
  "sessionName": "odom_mapping",
  "databasePath": "maps/sessions/odom_mapping/rtabmap.db",
  "rtabmapViz": true
}
```

Without a backend, the UI uses local fallback state so the Phase 6 control flow can be tested visually.

Backend behavior:

- Mapping starts `slam.launch.py` with `database_path:=maps/sessions/<sessionName>/rtabmap.db`.
- Mapping refuses to start if that session DB already exists.
- Localization starts `localization.launch.py` with `database_path:=maps/active/rtabmap.db`.
- `Align Mode` applies an RTAB-Map LiDAR-example shaped localization profile: `RGBD/ProximityBySpace='true'`, `RGBD/ProximityPathMaxNeighbors='1'`, `RGBD/ProximityMaxGraphDepth='0'`, `RGBD/LinearUpdate='0.05'`, `RGBD/AngularUpdate='0.05'`, `Icp/CorrespondenceRatio='0.2'`, `Icp/MaxCorrespondenceDistance='1.0'`, and `Icp/MaxTranslation='3.0'`.
- `Lock Tracking` restores conservative odom-only tracking, including `RGBD/ProximityBySpace='false'`, proximity neighbor/depth limits of `0`, `RGBD/LinearUpdate='0.1'`, `RGBD/AngularUpdate='0.1'`, `RGBD/MaxOdomCacheSize='10'`, and `Icp/MaxTranslation='0.25'`.
- `Send Pose` publishes a `geometry_msgs/PoseWithCovarianceStamped` to `/rtabmap/initialpose`.
