# Cyberglove control for allegro hand
This package is used to map `JointState` input from the cyberglove to `JointState` commands compatible with allegro hand.

## Calibration for Thumb
1. Record data on both allegro hand and cyberglove according to the specifications given in [`calibration_spec.md`](calibration_spec.md). Save all generated `.npz` files in `data` folder.
2. Use [`train.ipynb`](src/train.ipynb) to train for a GPR model for thumb control. This would generate `data/trainedModel_GPR.joblib` containing the trained model.

## Run Instructions
Launch node `cyberglove_controller.py` under the allegro hand namespace. Remap `/cyberglove/joint_states` to appropiate topic from the cyberglove.

## Authors/Maintainers
- [Vaibhav Gupta](https://github.com/guptavaibhav0)
- [Saurav Aryan](https://github.com/aryansaurav) (Maintainer)
