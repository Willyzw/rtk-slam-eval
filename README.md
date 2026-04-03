# RTK-SLAM Evaluation

Evaluation scripts for the paper:

> **An RTK-SLAM Dataset for Absolute Accuracy Evaluation in GNSS-Degraded Environments**  
> Wei Zhang, Vincent Ress, David Skuddis, Uwe Soergel, Norbert Haala  
> Institute for Photogrammetry and Geoinformatics, University of Stuttgart

[[Project Page]](https://rtk-slam-dataset.github.io/) · [[Dataset (HuggingFace)]](https://huggingface.co/datasets/Willyzw/rtk-slam-dataset)

---

## Repository Structure

```
rtk-slam-eval/
├── eval/
│   ├── eval.py        # Batch evaluation script
│   ├── readers.py     # File I/O and timestamp matching
│   ├── metrics.py     # Error computation (absolute ATE, SE3-aligned ATE)
│   └── coords.py      # Coordinate conversion (ENU → UTM)
├── ground_truth/
│   ├── {sequence}.csv             # Surveyed checkpoint coordinates (UTM Zone 32N, ETRS89)
│   └── {sequence}_timestamps.csv  # SLAM timestamps at each checkpoint
├── calib/
│   └── calib.yaml     # Sensor calibration (camera, LiDAR, GNSS offsets)
├── data/
│   └── {sequence}/
│       └── rtk.txt    # RTK GNSS measurements (included)
├── trajectories/
│   └── {method}/{sequence}/
│       ├── traj_offline.txt   # Offline/batch result (TUM format)
│       ├── traj_online.txt    # Online/causal result (TUM format, optional)
│       └── enu_origin.json    # ENU reference origin [lat, lon, alt]
├── download_data.py   # Dataset download script
└── requirements.txt
```

---

## Running the Evaluation

The script auto-discovers all methods and sequences from the `trajectories/` directory.

```bash
# All methods, all sequences
python eval/eval.py

# One method or one sequence
python eval/eval.py --method fast_lio_sam
python eval/eval.py --seq stadtgarten_seq1
```

---

## Adding a New Method

Drop the trajectory files into `trajectories/` — no code changes needed:

```
trajectories/
└── my_method/
    ├── stadtgarten_seq1/
    │   ├── traj_offline.txt   # required — TUM format: timestamp x y z qx qy qz qw
    │   ├── traj_online.txt    # optional
    │   └── enu_origin.json    # [lat, lon, alt] of the ENU frame origin
    └── ...
```

Re-run `python eval/eval.py` and the new method appears in the table automatically.

---

## Ground Truth Format

Checkpoint coordinates (`ground_truth/{sequence}.csv`):

```
# id  easting  northing  height  [env]
4000   512672.397  5403255.195  254.648  outdoor
MS001  512666.706  5403222.923  254.539  indoor
```

Coordinate system: **UTM Zone 32N (ETRS89)**, matching the SAPOS RTK correction service.  
The optional `env` column (`outdoor` / `transition` / `indoor`) is used for per-environment breakdown.

Checkpoint timestamps (`ground_truth/{sequence}_timestamps.csv`):

```
point_id,timestamp
4000,1764589650.123
```

---

## Evaluation Metrics

| Metric | Description |
|--------|-------------|
| **Offline / Online** | Absolute 3D RMSE against surveyed checkpoints, no alignment. Requires the trajectory to be in the global UTM frame. |
| **SE3** | RMSE after optimal rigid-body (SE3) alignment — standard ATE [Sturm et al., 2012]. |
| **Gap [%]** | `(Offline − SE3) / Offline × 100`. How much global error is hidden by alignment. |

---

## Citation

```bibtex
@article{zhang2025rtkslam,
  title   = {An RTK-SLAM Dataset for Absolute Accuracy Evaluation in GNSS-Degraded Environments},
  author  = {Zhang, Wei and Ress, Vincent and Skuddis, David and Soergel, Uwe and Haala, Norbert},
  journal = {ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences},
  year    = {2025},
}
```
