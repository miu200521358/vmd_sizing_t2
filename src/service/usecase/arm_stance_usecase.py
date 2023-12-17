import os
from typing import Optional

import numpy as np
from mlib.core.logger import MLogger
from mlib.core.math import MMatrix4x4
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion
from numpy.linalg import inv
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


ARM_BONE_NAMES = [
    BoneNames.arm("右"),
    BoneNames.elbow("右"),
    BoneNames.wrist("右"),
    BoneNames.arm("左"),
    BoneNames.elbow("左"),
    BoneNames.wrist("左"),
]


class ArmStanceUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        show_message: bool = False,
    ) -> bool:
        if not src_model or not dest_model:
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        if set(ARM_BONE_NAMES) - set(src_model.bones.names) or True in [
            BoneFlg.NOTHING in src_model.bones[bone_name].bone_flg
            for bone_name in ARM_BONE_NAMES
        ]:
            if show_message:
                logger.warning(
                    "【No.{i}】モーション作成元モデルに腕・ひじ・手首の左右ボーンがないため、腕スタンス補正をスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if set(ARM_BONE_NAMES) - set(dest_model.bones.names) or True in [
            BoneFlg.NOTHING in dest_model.bones[bone_name].bone_flg
            for bone_name in ARM_BONE_NAMES
        ]:
            if show_message:
                logger.warning(
                    "【No.{i}】サイジング先モデルに腕・ひじ・手首の左右ボーンがないため、腕スタンス補正をスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_arm_stance(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
    ) -> tuple[int, VmdMotion]:
        """腕スタンス補正によるサイジング"""

        offset_from_slope_matrixes, offset_to_slope_matrixes = self.get_slope_qq(
            src_model, dest_model, motion
        )

        logger.info(
            "【No.{i}】腕スタンス補正  腕[{a}] ひじ[{e}] 手首[{w}]",
            i=sizing_idx + 1,
            a=MMatrix4x4(offset_to_slope_matrixes[BoneNames.arm("左")])
            .to_quaternion()
            .to_euler_degrees()
            .mmd,
            e=MMatrix4x4(offset_to_slope_matrixes[BoneNames.elbow("左")])
            .to_quaternion()
            .to_euler_degrees()
            .mmd,
            w=MMatrix4x4(offset_to_slope_matrixes[BoneNames.wrist("左")])
            .to_quaternion()
            .to_euler_degrees()
            .mmd,
            decoration=MLogger.Decoration.LINE,
        )

        from_offsets: list[np.ndarray] = []
        to_offsets: list[np.ndarray] = []
        rotations: list[np.ndarray] = []
        for bone_name in (
            ARM_BONE_NAMES + BoneNames.fingers("左") + BoneNames.fingers("右")
        ):
            if not (
                bone_name in motion.bones
                and bone_name in offset_from_slope_matrixes
                and bone_name in offset_to_slope_matrixes
            ):
                continue
            for bf in motion.bones[bone_name]:
                rotations.append(bf.rotation.to_matrix4x4().vector)
                from_offsets.append(offset_from_slope_matrixes[bone_name])
                to_offsets.append(offset_to_slope_matrixes[bone_name])

        if 0 < len(rotations):
            from_offset_matrixes = np.array(from_offsets)
            to_offset_matrixes = np.array(to_offsets)
            rot_matrixes = np.array(rotations)

            rot_sizing_matrixes = (
                from_offset_matrixes @ rot_matrixes @ to_offset_matrixes
            )
            n = 0
            for bone_name in (
                ARM_BONE_NAMES + BoneNames.fingers("左") + BoneNames.fingers("右")
            ):
                if not (
                    bone_name in motion.bones
                    and bone_name in offset_from_slope_matrixes
                    and bone_name in offset_to_slope_matrixes
                ):
                    continue
                for bf in motion.bones[bone_name]:
                    bf.rotation = MMatrix4x4(rot_sizing_matrixes[n]).to_quaternion()
                    n += 1

        return sizing_idx, motion

    def get_slope_qq(
        self,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
    ) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
        """スタンス補正用比率算出"""
        offset_from_slope_matrixes: dict[str, np.ndarray] = {}
        offset_to_slope_matrixes: dict[str, np.ndarray] = {}

        for direction in ("右", "左"):
            for from_bone_suffix, target_bone_suffix, to_bone_suffix in (
                ("", "腕", "ひじ"),
                ("腕", "ひじ", "手首"),
                ("ひじ", "手首", "中指１"),
                ("", "親指１", "親指先"),
                ("", "人指１", "人指先"),
                ("", "中指１", "中指先"),
                ("", "薬指１", "薬指先"),
                ("", "小指１", "小指先"),
            ):
                from_bone_name = f"{direction}{from_bone_suffix}"
                target_bone_name = f"{direction}{target_bone_suffix}"
                to_bone_name = f"{direction}{to_bone_suffix}"

                if (
                    from_bone_suffix
                    and from_bone_name in src_model.bones
                    and from_bone_name in dest_model.bones
                    and target_bone_name in src_model.bones
                    and target_bone_name in dest_model.bones
                    and from_bone_name in motion.bones
                ):
                    # モデルとモーションにボーンがある場合、逆回転をかける
                    offset_from_slope_matrixes[target_bone_name] = inv(
                        offset_to_slope_matrixes[from_bone_name]
                    )
                else:
                    offset_from_slope_matrixes[target_bone_name] = np.eye(4)

                if (
                    to_bone_name not in src_model.bones
                    or to_bone_name not in dest_model.bones
                ):
                    # 指とかがなければ空欄
                    if target_bone_name not in offset_to_slope_matrixes:
                        offset_to_slope_matrixes[target_bone_name] = np.eye(4)
                    continue

                src_from_bone_position = src_model.bones[target_bone_name].position
                src_to_bone_position = src_model.bones[to_bone_name].position
                src_bone_vector = (
                    src_to_bone_position - src_from_bone_position
                ).normalized()
                src_slope_qq = src_bone_vector.to_local_matrix4x4().to_quaternion()

                dest_from_bone_position = dest_model.bones[target_bone_name].position
                dest_to_bone_position = dest_model.bones[to_bone_name].position
                dest_bone_vector = (
                    dest_to_bone_position - dest_from_bone_position
                ).normalized()
                dest_slope_qq = dest_bone_vector.to_local_matrix4x4().to_quaternion()

                offset_qq = src_slope_qq * dest_slope_qq.inverse()

                # X軸（捩り）成分を除去した値のみ保持
                _, offset_y_qq, _, offset_yz_qq = offset_qq.separate_by_axis(
                    dest_bone_vector
                )

                if "指" in target_bone_suffix:
                    # 指はローカルY（指の広がり）だけ参照する
                    # そのため手首までの逆回転を無視する
                    offset_to_slope_matrixes[
                        target_bone_name
                    ] = offset_y_qq.to_matrix4x4().vector
                else:
                    offset_to_slope_matrixes[
                        target_bone_name
                    ] = offset_yz_qq.to_matrix4x4().vector

        return offset_from_slope_matrixes, offset_to_slope_matrixes
