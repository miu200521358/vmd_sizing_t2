import os

import numpy as np
from numpy.linalg import inv

from mlib.core.logger import MLogger
from mlib.core.math import MMatrix4x4
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class ShoulderStanceUsecase:
    def sizing_shoulder_stance(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
        direction: str,
    ) -> tuple[int, VmdMotion]:
        SHOULDER_BONE_NAMES = [f"{direction}肩", f"{direction}腕"]

        """肩スタンス補正によるサイジング"""
        if set(SHOULDER_BONE_NAMES) - set(src_model.bones.names):
            logger.warning(
                "【No.{i}】モーション作成元モデルに肩・腕の左右ボーンがないため、肩スタンス補正をスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        if set(SHOULDER_BONE_NAMES) - set(dest_model.bones.names):
            logger.warning(
                "【No.{i}】サイジング先モデルに肩・腕の左右ボーンがないため、肩スタンス補正をスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        logger.info("【No.{i}】{d}肩スタンス補正", i=sizing_idx + 1, d=direction, decoration=MLogger.Decoration.LINE)

        shoulder_fnos = sorted(
            set(motion.bones[f"{direction}肩"].indexes)
            | set(motion.bones[f"{direction}肩P"].indexes)
            | set(motion.bones[f"{direction}腕"].indexes)
        )

        if not shoulder_fnos:
            return sizing_idx, motion

        src_matrixes = motion.animate_bone(shoulder_fnos, src_model, [f"{direction}手首"], out_fno_log=True)
        dest_matrixes = motion.animate_bone(shoulder_fnos, dest_model, [f"{direction}手首"], out_fno_log=True)

        shoulder_fidxs: dict[int, int] = {}
        shoulder_rotations: list[np.ndarray] = []
        for fidx, fno in enumerate(shoulder_fnos):
            shoulder_rotations.append(motion.bones[f"{direction}肩"][fno].rotation.to_matrix4x4().vector)
            shoulder_fidxs[fno] = fidx

        arm_rotations: list[np.ndarray] = []
        for fno in shoulder_fnos:
            arm_rotations.append(motion.bones[f"{direction}腕"][fno].rotation.to_matrix4x4().vector)

        offset_rotations: list[np.ndarray] = []
        for fidx, fno in enumerate(shoulder_fnos):
            logger.count("{d}肩補正計算", d=direction, index=fidx, total_index_count=len(shoulder_fnos), display_block=1000)

            src_local_arm_position = (
                src_matrixes[fno, f"{direction}肩根元"].global_matrix.inverse() * src_matrixes[fno, f"{direction}腕"].position
            )
            dest_global_arm_position = dest_matrixes[fno, f"{direction}肩根元"].global_matrix * src_local_arm_position
            offset_shoulder_qq = (
                (dest_matrixes[fno, f"{direction}腕"].position - dest_matrixes[fno, f"{direction}肩"].position).to_local_matrix4x4()
                @ (dest_global_arm_position - dest_matrixes[fno, f"{direction}肩"].position).to_local_matrix4x4().inverse()
            ).to_quaternion()
            logger.debug(
                f"[{direction}肩][{fno}]"
                + f"元肩根元{src_matrixes[fno, f'{direction}肩根元'].position}, 元肩{src_matrixes[fno, f'{direction}肩'].position}, "
                + f"元腕{src_matrixes[fno, f'{direction}腕'].position}, 元ベクトル {src_local_arm_position.normalized()}, "
                + f"先肩根元{dest_matrixes[fno, f'{direction}肩根元'].position}, 先肩{dest_matrixes[fno, f'{direction}肩'].position}, "
                + f"先腕{dest_matrixes[fno, f'{direction}腕'].position}, 先腕理想 {dest_global_arm_position}, "
                + f"offset{offset_shoulder_qq.to_euler_degrees()}"
            )
            offset_rotations.append(offset_shoulder_qq.to_matrix4x4().vector)

        shoulder_sizing_matrixes = np.array(shoulder_rotations) @ inv(np.array(offset_rotations))
        arm_sizing_matrixes = np.array(offset_rotations) @ np.array(arm_rotations)

        for bf in motion.bones[f"{direction}肩"]:
            bf.rotation = MMatrix4x4(shoulder_sizing_matrixes[shoulder_fidxs[bf.index]]).to_quaternion()

        for bf in motion.bones[f"{direction}腕"]:
            bf.rotation = MMatrix4x4(arm_sizing_matrixes[shoulder_fidxs[bf.index]]).to_quaternion()

        return sizing_idx, motion
