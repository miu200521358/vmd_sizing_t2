import os
from enum import StrEnum
from logging.handlers import QueueHandler
from multiprocessing import Queue
from typing import Optional

import numpy as np

from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import BoneMorphOffset, Morph
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_part import VmdMorphFrame
from mlib.pmx.pmx_part import MorphType
from mlib.vmd.vmd_collection import VmdBoneFrames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


MOVE_BONE_NAMES = ("全ての親", "センター", "グルーブ", "右足IK親", "左足IK親", "右足ＩＫ", "左足ＩＫ", "右つま先ＩＫ", "左つま先ＩＫ")


class SizingMoveMorphs(StrEnum):
    LEG_Y_RATIO = "足Y比率"
    LEG_XZ_RATIO = "足XZ比率"


class MoveUsecase:
    def fit_move_sizing(
        self,
        sizing_idx: int,
        xz_leg_ratio: float,
        y_leg_ratio: float,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
        log_queue: Queue,
    ) -> tuple[int, VmdMotion]:
        """移動系モーフによるサイジングフィッティング"""
        MLogger.queue_handler = QueueHandler(log_queue)

        move_sizing_positions: list[np.ndarray] = []
        for bone_name in MOVE_BONE_NAMES:
            if bone_name not in motion.bones:
                continue
            for bf in motion.bones[bone_name]:
                move_sizing_positions.append(bf.position.vector)
        move_sizing_mats = np.full((len(move_sizing_positions), 4, 4), np.eye(4))
        move_sizing_mats[..., :3, 3] = np.array(move_sizing_positions)

        scale_mat = np.eye(4)
        scale_mat[0, 0] = xz_leg_ratio
        scale_mat[1, 1] = y_leg_ratio
        scale_mat[2, 2] = xz_leg_ratio

        move_scaled_mats = scale_mat @ move_sizing_mats
        n = 0
        for bone_name in MOVE_BONE_NAMES:
            if bone_name not in motion.bones:
                continue
            for bf in motion.bones[bone_name]:
                bf.position.vector = move_scaled_mats[n, :3, 3]
                n += 1

        return sizing_idx, motion

    def fit_move_sizing2(
        self,
        sizing_idx: int,
        fit_bone_name: str,
        src_model: PmxModel,
        dest_model: PmxModel,
        motion: VmdMotion,
        log_queue: Queue,
    ) -> tuple[int, str, Optional[VmdBoneFrames]]:
        """移動系モーフによるサイジングフィッティング"""
        MLogger.queue_handler = QueueHandler(log_queue)

        for morph in SizingMoveMorphs:
            motion.append_morph_frame(VmdMorphFrame(0, morph.value, 1.0))

        target_fnos: set[int] = {0}
        target_bone_names: list[str] = [*MOVE_BONE_NAMES]
        if fit_bone_name in dest_model.bones and fit_bone_name in motion.bones:
            target_bone_names.append(fit_bone_name)
            if dest_model.bones[fit_bone_name].is_ik:
                # IKの場合はIK根元も対象とする
                target_bone_names.append(dest_model.bones[dest_model.bones[fit_bone_name].ik.links[-1].bone_index].name)
            target_fnos |= set(motion.bones[fit_bone_name].indexes)

        if not target_bone_names:
            return sizing_idx, fit_bone_name, None

        src_bone_matrixes = motion.animate_bone(sorted(target_fnos), src_model, target_bone_names, append_ik=False, out_fno_log=True)
        dest_bone_matrixes = motion.animate_bone(sorted(target_fnos), dest_model, target_bone_names, append_ik=False, out_fno_log=True)

        for bone_name in target_bone_names:
            # 親INDEXが有効な場合、親ボーン名を取得
            parent_bone_name = (
                dest_model.bones[dest_model.bones[bone_name].parent_index].name
                if dest_model.bones[bone_name].parent_index in dest_model.bones
                else None
            )
            # IKの場合はIKの根元を取得
            ik_root_bone_name = (
                dest_model.bones[dest_model.bones[bone_name].ik.links[-1].bone_index].name if dest_model.bones[bone_name].is_ik else None
            )

            for fno in motion.bones[bone_name].indexes:
                if dest_bone_matrixes.exists(fno, bone_name):
                    if parent_bone_name:
                        # 親ボーンがある場合は、親からの相対位置
                        motion.bones[bone_name][fno].position = (
                            dest_bone_matrixes[fno, parent_bone_name].local_matrix_no_scale.inverse()
                            * dest_bone_matrixes[fno, bone_name].local_matrix_no_scale.to_position()
                        )
                    else:
                        # 親ボーンがない場合は自身の位置
                        motion.bones[bone_name][fno].position = dest_bone_matrixes[fno, bone_name].position

        return sizing_idx, fit_bone_name, motion.bones[fit_bone_name]

    def create_leg_y_ratio_morph(self, dest_model: PmxModel, y_leg_ratio: float) -> None:
        """足Y比率ボーンモーフ追加"""
        morph = Morph(name=SizingMoveMorphs.LEG_Y_RATIO.value)
        morph.is_system = True
        morph.morph_type = MorphType.BONE

        for bone_name in MOVE_BONE_NAMES:
            if bone_name in dest_model.bones:
                morph.offsets.append(BoneMorphOffset(dest_model.bones[bone_name].index, local_scale=MVector3D(0, y_leg_ratio - 1, 0)))

        dest_model.morphs.append(morph)

    def create_leg_xz_ratio_morph(self, dest_model: PmxModel, xz_leg_ratio: float) -> None:
        """足XZ比率ボーンモーフ追加"""
        morph = Morph(name=SizingMoveMorphs.LEG_XZ_RATIO.value)
        morph.is_system = True
        morph.morph_type = MorphType.BONE

        for bone_name in MOVE_BONE_NAMES:
            if bone_name in dest_model.bones:
                morph.offsets.append(
                    BoneMorphOffset(dest_model.bones[bone_name].index, local_scale=MVector3D(xz_leg_ratio - 1, 0, xz_leg_ratio - 1))
                )

        dest_model.morphs.append(morph)

    def get_all_leg_xz_ratio(self, xz_leg_ratios: list[float]) -> float:
        """全体の足XZ比率取得"""
        if len(xz_leg_ratios) == 1:
            return xz_leg_ratios[0]

        return float(np.min([np.mean(xz_leg_ratios), 1.2]))

    def get_leg_ratio(self, src_model: PmxModel, dest_model: PmxModel) -> tuple[float, float]:
        """足の比率"""
        target_bone_names = {"右足", "右ひざ", "右足首", "左足", "左ひざ", "左足首"}

        if not target_bone_names.issubset(src_model.bones.names):
            logger.warning("モーション作成元モデルに足・ひざ・足首の左右ボーンがないため、足の比率が測れません", decoration=MLogger.Decoration.BOX)
            return 1.0, 1.0

        if not target_bone_names.issubset(dest_model.bones.names):
            logger.warning("サイジング先モデルに足・ひざ・足首の左右ボーンがないため、足の比率が測れません", decoration=MLogger.Decoration.BOX)
            return 1.0, 1.0

        src_xz_leg_length = float(
            np.mean(
                [
                    (
                        (src_model.bones["左足首"].position - src_model.bones["左ひざ"].position)
                        + (src_model.bones["左ひざ"].position - src_model.bones["左足"].position)
                    ).length(),
                    (
                        (src_model.bones["右足首"].position - src_model.bones["右ひざ"].position)
                        + (src_model.bones["右ひざ"].position - src_model.bones["右足"].position)
                    ).length(),
                ]
            )
        )

        dest_xz_leg_length = float(
            np.mean(
                [
                    (
                        (dest_model.bones["左足首"].position - dest_model.bones["左ひざ"].position)
                        + (dest_model.bones["左ひざ"].position - dest_model.bones["左足"].position)
                    ).length(),
                    (
                        (dest_model.bones["右足首"].position - dest_model.bones["右ひざ"].position)
                        + (dest_model.bones["右ひざ"].position - dest_model.bones["右足"].position)
                    ).length(),
                ]
            )
        )

        # XZ比率(足の長さ)
        xz_leg_ratio = dest_xz_leg_length / src_xz_leg_length if src_xz_leg_length and dest_xz_leg_length else 1

        src_y_leg_length = (
            ((src_model.bones["左足首"].position - src_model.bones["左足"].position)).y
            + ((src_model.bones["右足首"].position - src_model.bones["右足"].position)).y
        ) / 2

        dest_y_leg_length = (
            ((dest_model.bones["左足首"].position - dest_model.bones["左足"].position)).y
            + ((dest_model.bones["右足首"].position - dest_model.bones["右足"].position)).y
        ) / 2

        # Y比率(股下のY差)
        y_leg_ratio = dest_y_leg_length / src_y_leg_length if src_y_leg_length and dest_y_leg_length else 1

        return xz_leg_ratio, y_leg_ratio
