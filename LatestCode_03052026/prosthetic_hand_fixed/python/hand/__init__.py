"""hand: laptop-side library for the split-architecture prosthetic hand.

Modules:
    constants    - numeric constants (motor range, thresholds, feature count)
    protocol     - serial line parsing (sensor, motor, event markers)
    controller   - dual-serial manager with threaded readers
    motor_field  - motor-field profile build / save / load / subtract
    grasp        - record_grasp() orchestration (one grasp end-to-end)
    features     - extract_features() (57-feature vector — single source of truth)
"""
