import os

import numpy as np
from numpy.linalg import inv

from mlib.core.logger import MLogger
from mlib.core.math import MMatrix4x4
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


ARM_BONE_NAMES = ["右腕", "右ひじ", "右手首", "左腕", "左ひじ", "左手首"]


class ArmStanceUsecase:
    def sizing_arm_stance(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
    ) -> tuple[int, VmdMotion]:
        """腕スタンス補正によるサイジング"""
        if set(ARM_BONE_NAMES) - set(src_model.bones.names):
            logger.warning(
                "【No.{i}】モーション作成元モデルに腕・ひじ・手首の左右ボーンがないため、腕スタンス補正をスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        if set(ARM_BONE_NAMES) - set(dest_model.bones.names):
            logger.warning(
                "【No.{i}】サイジング先モデルに腕・ひじ・手首の左右ボーンがないため、腕スタンス補正をスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        offset_from_slope_matrixes, offset_to_slope_matrixes = self.get_slope_qq(src_model, dest_model)

        logger.info(
            "【No.{i}】腕スタンス補正  腕[{a}] ひじ[{e}] 手首[{w}]",
            i=sizing_idx + 1,
            a=MMatrix4x4(offset_to_slope_matrixes["左腕"]).to_quaternion().to_euler_degrees(),
            e=MMatrix4x4(offset_to_slope_matrixes["左ひじ"]).to_quaternion().to_euler_degrees(),
            w=MMatrix4x4(offset_to_slope_matrixes["左手首"]).to_quaternion().to_euler_degrees(),
            decoration=MLogger.Decoration.LINE,
        )

        from_offsets: list[np.ndarray] = []
        to_offsets: list[np.ndarray] = []
        rotations: list[np.ndarray] = []
        for bone_name in ARM_BONE_NAMES:
            if bone_name not in motion.bones:
                continue
            for bf in motion.bones[bone_name]:
                rotations.append(bf.rotation.to_matrix4x4().vector)
                from_offsets.append(offset_from_slope_matrixes[bone_name])
                to_offsets.append(offset_to_slope_matrixes[bone_name])

        if 0 < len(rotations):
            from_offset_matrixes = np.array(from_offsets)
            to_offset_matrixes = np.array(to_offsets)
            rot_matrixes = np.array(rotations)

            rot_sizing_matrixes = from_offset_matrixes @ rot_matrixes @ to_offset_matrixes
            n = 0
            for bone_name in ARM_BONE_NAMES:
                if bone_name not in motion.bones:
                    continue
                for bf in motion.bones[bone_name]:
                    bf.rotation = MMatrix4x4(rot_sizing_matrixes[n]).to_quaternion()
                    n += 1

        return sizing_idx, motion

    def get_slope_qq(self, src_model: PmxModel, dest_model: PmxModel) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        """移動補正用比率算出"""
        left_finger_bone_names = ["左中指１"] if "左中指１" in src_model.bones and "左中指１" in dest_model.bones else []
        right_finger_bone_names = ["右中指１"] if "右中指１" in src_model.bones and "右中指１" in dest_model.bones else []

        src_matrixes = VmdMotion().animate_bone([0], src_model, ARM_BONE_NAMES + left_finger_bone_names + right_finger_bone_names)
        dest_matrixes = VmdMotion().animate_bone([0], dest_model, ARM_BONE_NAMES + left_finger_bone_names + right_finger_bone_names)

        offset_from_slope_matrixes: dict[str, np.ndarray] = {}
        offset_to_slope_matrixes: dict[str, np.ndarray] = {}

        for direction in ("右", "左"):
            for from_bone_suffix, target_bone_suffix, to_bone_suffix in (("", "腕", "ひじ"), ("腕", "ひじ", "手首"), ("ひじ", "手首", "中指１")):
                from_bone_name = f"{direction}{from_bone_suffix}"
                target_bone_name = f"{direction}{target_bone_suffix}"
                to_bone_name = f"{direction}{to_bone_suffix}"

                if from_bone_suffix:
                    # 逆回転をかける
                    offset_from_slope_matrixes[target_bone_name] = inv(offset_to_slope_matrixes[from_bone_name])
                else:
                    offset_from_slope_matrixes[target_bone_name] = np.eye(4)

                if to_bone_name not in src_model.bones or to_bone_name not in dest_model.bones:
                    # 指とかがなければ空欄
                    offset_to_slope_matrixes[target_bone_name] = np.eye(4)
                    continue

                src_from_bone_position = src_matrixes[0, target_bone_name].position
                src_to_bone_position = src_matrixes[0, to_bone_name].position
                src_bone_vector = (src_to_bone_position - src_from_bone_position).normalized()
                src_slope_qq = src_bone_vector.to_local_matrix4x4().to_quaternion()

                dest_from_bone_position = dest_matrixes[0, target_bone_name].position
                dest_to_bone_position = dest_matrixes[0, to_bone_name].position
                dest_bone_vector = (dest_to_bone_position - dest_from_bone_position).normalized()
                dest_slope_qq = dest_bone_vector.to_local_matrix4x4().to_quaternion()

                offset_qq = src_slope_qq * dest_slope_qq.inverse()

                # X軸（捩り）成分を除去した値のみ保持
                _, _, _, offset_yz_qq = offset_qq.separate_by_axis(dest_bone_vector)

                offset_to_slope_matrixes[target_bone_name] = offset_yz_qq.to_matrix4x4().vector

        return offset_from_slope_matrixes, offset_to_slope_matrixes
