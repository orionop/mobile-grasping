# mobile-grasping

Closed-loop reactive control for on-the-move grasping on TurtleBot 3 Waffle + OpenManipulator-X.

Collaboration with Prof. Takuya Kiyokawa (Osaka University) and Dr. Arpita Sinha (IIT Bombay).
Extending Kiyokawa et al., "Self-Supervised Learning of Grasping Arbitrary Objects On-the-Move," IEEE/SICE SII 2025.

## Status

| Component | Status |
|---|---|
| Reference Holistic QP (Frankie sim, Mac) | In progress |
| 4-DOF OMX-X adaptation | Not started |
| TB3 + OMX-X URDF integration | Not started |
| Predictor interface (FCN / GG-CNN / OpenVLA) | Not started |
| Base pose estimation (wheel + IMU + VO) | Not started |
| ROS 2 integration | Not started |
| Real-hardware experiments | Not started |

## Repo layout

```
mobile-grasping/
├── docs/              # Architecture, derivations, design decisions
├── src/
│   ├── controller/    # QP-based reactive controller
│   ├── predictor/     # Swappable grasp predictor interface
│   └── pipeline.py    # End-to-end wiring
├── tests/             # Unit + integration tests
├── notebooks/         # Math derivations, numerical sanity checks
├── experiments/       # Experiment scripts and results
└── pyproject.toml
```

## Quick start

```bash
# Create Python environment
python3 -m venv .venv
source .venv/bin/activate
pip install -e .

# Run the Haviland-Corke Holistic reference example (Frankie in Swift simulator)
python experiments/exp01_holistic_reference.py
```

## References

1. T. Kiyokawa et al., "Self-Supervised Learning of Grasping Arbitrary Objects On-the-Move," IEEE/SICE SII, 2025.
2. J. Haviland, N. Sünderhauf, P. Corke, "A Holistic Approach to Reactive Mobile Manipulation," IEEE RA-L, 7(2):3122-3129, 2022.
3. J. Haviland, P. Corke, "NEO: A Novel Expeditious Optimisation Algorithm for Reactive Motion Control of Manipulators," IEEE RA-L, 6(2):1043-1050, 2021.
4. B. Burgess-Limerick, C. Lehnert, J. Leitner, P. Corke, "An Architecture for Reactive Mobile Manipulation On-The-Move," IEEE ICRA, 2023.
