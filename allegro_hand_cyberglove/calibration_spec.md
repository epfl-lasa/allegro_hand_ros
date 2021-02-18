# Calibration Specifications
6 `.npz` files are needed for the calibration process. 3 for the cyberglove and 3 for the allegro hand. Replace `*` in following by `allegro` and `cyberglove`.

- `*_base.npz`
    - At the base of the fingers
    - Recommended positions along lateral hand directions with 0 at the base of middle finger
        &rightarrow; {00, 20, 40, 60, 80, 100, -20,-40} [mm]
    - Recommended height above the palm
        &rightarrow; {28.0, 24.5, 21.0, 17.5, 14.0, 10.5, 7.0, 3.5} [mm]
    - Recordings available for allegro hand &rightarrow; all lateral positions at a height repeated twice, and then repeated for all heights

- `*_center.npz`
    - At the center of the palm
    - Recommended positions along lateral hand directions with 0 directly below middle finger
        &rightarrow; {00, 20, 40, 60, 80, 100, -20,-40} [mm]
    - Recommended height above the palm
        &rightarrow; {28.0, 24.5, 21.0, 17.5, 14.0, 10.5, 7.0, 3.5} [mm]
    - Recordings available for allegro hand &rightarrow; all lateral positions at a height repeated twice, and then repeated for all heights

- `*_thumb.npz`
    - Move thumb to some standard positions
    - 10 recordings at zero pose
    - 10 recordings with thumb parallel to the fingers
    - 10 recordings with thumb perpendicular to the fingers