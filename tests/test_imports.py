"""Verify all dependencies import correctly on Apple Silicon."""

def test_numpy_scipy():
    import numpy as np
    import scipy
    assert np.__version__ >= "1.24"
    print(f"numpy {np.__version__}, scipy {scipy.__version__}")


def test_roboticstoolbox():
    import roboticstoolbox as rtb
    print(f"roboticstoolbox {rtb.__version__}")


def test_spatialmath():
    import spatialmath as sm
    from spatialmath import SE3
    T = SE3.Trans(0.5, 0.0, 0.3)
    assert T.t.shape == (3,)
    print(f"spatialmath OK, sample SE3: t={T.t}")


def test_qpsolvers():
    import qpsolvers
    import numpy as np
    # Tiny QP sanity: min 0.5 x^T x  s.t.  x >= 1
    P = np.eye(2)
    q = np.zeros(2)
    lb = np.ones(2)
    x = qpsolvers.solve_qp(P, q, lb=lb, solver="osqp")
    assert x is not None
    print(f"qpsolvers OK, solution: {x}")


def test_panda_model():
    """Load the Panda robot model used by Haviland-Corke."""
    import roboticstoolbox as rtb
    panda = rtb.models.Panda()
    print(f"Panda DOF: {panda.n}, links: {len(panda.links)}")
    assert panda.n == 7


if __name__ == "__main__":
    test_numpy_scipy()
    test_roboticstoolbox()
    test_spatialmath()
    test_qpsolvers()
    test_panda_model()
    print("\nAll imports OK on Apple Silicon.")
