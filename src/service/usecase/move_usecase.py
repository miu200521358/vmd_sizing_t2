import os
from typing import Optional

import numpy as np
from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


MOVE_ALL_BONE_NAMES = {
    BoneNames.root(),
    BoneNames.center(),
    BoneNames.groove(),
    BoneNames.leg_ik_parent("右"),
    BoneNames.leg_ik_parent("左"),
    BoneNames.leg_ik("右"),
    BoneNames.leg_ik("左"),
    BoneNames.toe_ik("右"),
    BoneNames.toe_ik("左"),
}

MOVE_CHECK_BONE_NAMES = {
    BoneNames.leg("右"),
    BoneNames.knee("右"),
    BoneNames.ankle("右"),
    BoneNames.leg_ik("右"),
    BoneNames.toe_ik("右"),
    BoneNames.leg("左"),
    BoneNames.knee("左"),
    BoneNames.ankle("左"),
    BoneNames.leg_ik("左"),
    BoneNames.toe_ik("左"),
}


class MoveUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        show_message: bool = False,
    ) -> bool:
        if not src_model or not dest_model:
            # モデルが揃ってない場合、スルー
            return False

        if MOVE_CHECK_BONE_NAMES - set(src_model.bones.names) or True in [
            BoneFlg.NOTHING in src_model.bones[bone_name].bone_flg
            for bone_name in MOVE_CHECK_BONE_NAMES
        ]:
            if show_message:
                logger.warning(
                    "【No.{i}】モーション作成元モデルに足・ひざ・足首・足ＩＫ・つま先ＩＫの左右ボーンがないため、移動補正をスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if MOVE_CHECK_BONE_NAMES - set(dest_model.bones.names) or True in [
            BoneFlg.NOTHING in dest_model.bones[bone_name].bone_flg
            for bone_name in MOVE_CHECK_BONE_NAMES
        ]:
            if show_message:
                logger.warning(
                    "【No.{i}】サイジング先モデルに足・ひざ・足首・足ＩＫ・つま先ＩＫの左右ボーンがないため、移動補正をスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_move(
        self,
        sizing_idx: int,
        xz_leg_ratio: float,
        leg_ratio: MVector3D,
        center_offset: MVector3D,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
    ) -> tuple[int, VmdMotion]:
        """移動補正"""

        logger.info(
            "【No.{i}】移動補正  縮尺: XZ[{x:.5f}](元: {ox:.5f}), Y[{y:.5f}] センターオフセット[{c}]",
            i=sizing_idx + 1,
            x=leg_ratio.x,
            y=leg_ratio.y,
            ox=xz_leg_ratio,
            c=center_offset,
            decoration=MLogger.Decoration.LINE,
        )

        offset_positions: list[np.ndarray] = []
        move_sizing_positions: list[np.ndarray] = []
        for bone_name in MOVE_ALL_BONE_NAMES:
            if bone_name not in motion.bones:
                continue
            for bf in motion.bones[bone_name]:
                move_sizing_positions.append(bf.position.vector)
                if bone_name == BoneNames.center():
                    offset_positions.append(center_offset.vector)
                else:
                    offset_positions.append(np.zeros(3))

        if 0 < len(move_sizing_positions):
            move_sizing_matrixes = np.full(
                (len(move_sizing_positions), 4, 4), np.eye(4)
            )
            move_sizing_matrixes[..., :3, 3] = np.array(move_sizing_positions)

            offset_matrixes = np.full((len(offset_positions), 4, 4), np.eye(4))
            offset_matrixes[..., :3, 3] = np.array(offset_positions)

            scale_mat = np.diag(leg_ratio.vector4)

            move_scaled_matrixes = offset_matrixes @ scale_mat @ move_sizing_matrixes
            n = 0
            for bone_name in MOVE_ALL_BONE_NAMES:
                if bone_name not in motion.bones:
                    continue
                for bf in motion.bones[bone_name]:
                    bf.position.vector = move_scaled_matrixes[n, :3, 3]
                    n += 1

        return sizing_idx, motion

    def get_all_leg_xz_ratio(self, xz_leg_ratios: list[float]) -> float:
        """全体の足XZ比率取得"""
        if len(xz_leg_ratios) == 1:
            return xz_leg_ratios[0]

        return float(np.min([np.mean(xz_leg_ratios), 1.2]))

    def get_move_ratio(
        self, src_model: PmxModel, dest_model: PmxModel
    ) -> tuple[float, float, MVector3D]:
        """移動補正用比率算出"""
        # 足からひざまでの長さ
        src_upper_length = float(
            np.mean(
                [
                    src_model.bones[BoneNames.knee("左")].position.distance(
                        src_model.bones[BoneNames.leg("左")].position
                    ),
                    src_model.bones[BoneNames.knee("右")].position.distance(
                        src_model.bones[BoneNames.leg("右")].position
                    ),
                ]
            )
        )

        # ひざから足首までの長さ
        src_lower_length = float(
            np.mean(
                [
                    src_model.bones[BoneNames.ankle("左")].position.distance(
                        src_model.bones[BoneNames.knee("左")].position
                    ),
                    src_model.bones[BoneNames.ankle("右")].position.distance(
                        src_model.bones[BoneNames.knee("右")].position
                    ),
                ]
            )
        )

        # XZ比率は足の長さの合計を参照する
        src_xz_leg_length = src_upper_length + src_lower_length

        # 足からひざまでの長さ
        dest_upper_length = float(
            np.mean(
                [
                    dest_model.bones[BoneNames.knee("左")].position.distance(
                        dest_model.bones[BoneNames.leg("左")].position
                    ),
                    dest_model.bones[BoneNames.knee("右")].position.distance(
                        dest_model.bones[BoneNames.leg("右")].position
                    ),
                ]
            )
        )

        # ひざから足首までの長さ
        dest_lower_length = float(
            np.mean(
                [
                    dest_model.bones[BoneNames.ankle("左")].position.distance(
                        dest_model.bones[BoneNames.knee("左")].position
                    ),
                    dest_model.bones[BoneNames.ankle("右")].position.distance(
                        dest_model.bones[BoneNames.knee("右")].position
                    ),
                ]
            )
        )

        # XZ比率は足の長さの合計を参照する
        dest_xz_leg_length = dest_upper_length + dest_lower_length

        # XZ比率(足の長さ)
        xz_leg_ratio = (
            dest_xz_leg_length / src_xz_leg_length
            if src_xz_leg_length and dest_xz_leg_length
            else 1
        )

        # ---------------------
        src_y_leg_length = (
            (
                src_model.bones[BoneNames.leg("左")].position
                - src_model.bones[BoneNames.ankle("左")].position
            ).y
            + (
                src_model.bones[BoneNames.leg("右")].position
                - src_model.bones[BoneNames.ankle("右")].position
            ).y
        ) / 2

        dest_y_leg_length = (
            (
                dest_model.bones[BoneNames.leg("左")].position
                - dest_model.bones[BoneNames.ankle("左")].position
            ).y
            + (
                dest_model.bones[BoneNames.leg("右")].position
                - dest_model.bones[BoneNames.ankle("右")].position
            ).y
        ) / 2

        # Y比率(股下のY差)
        y_leg_ratio = (
            dest_y_leg_length / src_y_leg_length
            if src_y_leg_length and dest_y_leg_length
            else 1
        )

        # センターYオフセット -------------------------

        # 元モデルの足ボーンの長さとIKの長さ比
        src_leg_ratio = src_y_leg_length / src_xz_leg_length

        # 元モデルの長さ比から、先モデルの想定される足IKの長さを再算出
        recalc_dest_y_leg_length = src_leg_ratio * dest_xz_leg_length

        # 足の辺比率を同じにする
        center_y_offset = recalc_dest_y_leg_length - dest_y_leg_length

        # センターZオフセット -------------------------

        src_leg_z = (
            src_model.bones[BoneNames.leg("左")].position.z
            + src_model.bones[BoneNames.leg("右")].position.z
        ) / 2
        src_ankle_z = (
            src_model.bones[BoneNames.ankle("左")].position.z
            + src_model.bones[BoneNames.ankle("右")].position.z
        ) / 2
        src_toe_z = (
            src_model.bones[BoneNames.toe_ik("左")].position.z
            + src_model.bones[BoneNames.toe_ik("右")].position.z
        ) / 2

        dest_leg_z = (
            dest_model.bones[BoneNames.leg("左")].position.z
            + dest_model.bones[BoneNames.leg("右")].position.z
        ) / 2
        dest_ankle_z = (
            dest_model.bones[BoneNames.ankle("左")].position.z
            + dest_model.bones[BoneNames.ankle("右")].position.z
        ) / 2
        dest_toe_z = (
            dest_model.bones[BoneNames.toe_ik("左")].position.z
            + dest_model.bones[BoneNames.toe_ik("右")].position.z
        ) / 2

        # 元モデルの足の長さ
        src_foot_length = src_toe_z - src_ankle_z
        # 元モデルの重心
        src_center_gravity = (src_ankle_z - src_leg_z) / (src_ankle_z - src_toe_z)

        # 先モデルの足の長さ
        dest_foot_length = dest_toe_z - dest_ankle_z
        # 先モデルの重心
        dest_center_gravity = (dest_ankle_z - dest_leg_z) / (dest_ankle_z - dest_toe_z)

        # センターZオフセット
        center_z_offset = (dest_center_gravity - src_center_gravity) * (
            dest_foot_length / src_foot_length
        )

        return xz_leg_ratio, y_leg_ratio, MVector3D(0, center_y_offset, center_z_offset)
