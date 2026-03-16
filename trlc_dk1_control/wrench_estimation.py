"""External wrench estimation using MuJoCo Jacobian computation."""

from __future__ import annotations

import logging

import numpy as np

logger = logging.getLogger(__name__)

NUM_ARM_JOINTS = 6


class WrenchEstimator:
    """
    Estimates the 6D external wrench at the end-effector from joint-space
    external torques using the kinematic Jacobian.

    Uses a *separate* MuJoCo data instance (sharing the same model) so that
    forward kinematics for the Jacobian don't interfere with the gravity
    compensator's state.

    Args:
        mj_model: MuJoCo model (shared with GravityCompensator).
        ee_body_name: Name of the end-effector body in the model.
    """

    def __init__(self, mj_model, ee_body_name: str = "tool0") -> None:
        import mujoco

        self._mujoco = mujoco
        self._mj_model = mj_model
        self._mj_data = mujoco.MjData(mj_model)

        self._ee_body_id = mujoco.mj_name2id(
            mj_model, mujoco.mjtObj.mjOBJ_BODY, ee_body_name
        )
        if self._ee_body_id < 0:
            raise ValueError(
                f"Body '{ee_body_name}' not found in MuJoCo model. "
                f"Available bodies: {[mj_model.body(i).name for i in range(mj_model.nbody)]}"
            )

        # Pre-allocate Jacobian buffers: (3, nv) each
        nv = mj_model.nv
        self._jacp = np.zeros((3, nv))
        self._jacr = np.zeros((3, nv))

        logger.info(
            "WrenchEstimator ready: ee_body='%s' (id=%d)",
            ee_body_name,
            self._ee_body_id,
        )

    def compute(self, q: np.ndarray, tau_ext: np.ndarray) -> np.ndarray:
        """
        Estimate the 6D external wrench at the end-effector.

        Args:
            q: Current arm joint positions, shape (6,).
            tau_ext: External joint torques (measured - gravity), shape (6,).

        Returns:
            6D wrench [fx, fy, fz, tx, ty, tz] in world-aligned frame.
        """
        mujoco = self._mujoco
        data = self._mj_data

        # Set joint configuration and run forward kinematics
        data.qpos[:NUM_ARM_JOINTS] = q[:NUM_ARM_JOINTS]
        mujoco.mj_forward(self._mj_model, data)

        # Compute end-effector Jacobian
        self._jacp[:] = 0.0
        self._jacr[:] = 0.0
        mujoco.mj_jacBody(
            self._mj_model, data,
            self._jacp, self._jacr,
            self._ee_body_id,
        )

        # Extract arm columns (first 6 DoFs) → 6×6 Jacobian
        J_arm = np.vstack([
            self._jacp[:, :NUM_ARM_JOINTS],
            self._jacr[:, :NUM_ARM_JOINTS],
        ])

        # wrench = -(J^T)^{-1} * tau_ext
        return -np.linalg.solve(J_arm.T, tau_ext)


class NoWrenchEstimator:
    """Drop-in replacement when wrench estimation is disabled."""

    def compute(self, q: np.ndarray, tau_ext: np.ndarray) -> np.ndarray:
        return np.zeros(6)
