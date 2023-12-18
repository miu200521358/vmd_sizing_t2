import os
from typing import Optional

import numpy as np
from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlotReference, Ik, IkLink
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text

ARM_BONE_NAMES = {
    BoneNames.shoulder("右"),
    BoneNames.arm("右"),
    BoneNames.elbow("右"),
    BoneNames.wrist("右"),
    BoneNames.shoulder("左"),
    BoneNames.arm("左"),
    BoneNames.elbow("左"),
    BoneNames.wrist("左"),
}

# 逆指
FINGER_REVERSE_Y_RAD = 7


class AlignArmUsecase:
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

        """腕位置合わせ"""
        if ARM_BONE_NAMES - set(src_model.bones.names) or True in [
            BoneFlg.NOTHING in src_model.bones[bone_name].bone_flg
            for bone_name in ARM_BONE_NAMES
        ]:
            if show_message:
                logger.warning(
                    "【No.{x}】モーション作成元モデルに肩・腕・ひじ・手首の左右ボーンのいずれかがないため、腕位置合わせをスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if ARM_BONE_NAMES - set(dest_model.bones.names) or True in [
            BoneFlg.NOTHING in src_model.bones[bone_name].bone_flg
            for bone_name in ARM_BONE_NAMES
        ]:
            if show_message:
                logger.warning(
                    "【No.{x}】サイジング先モデルに肩・腕・ひじ・手首の左右ボーンのいずれかがないため、腕位置合わせをスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def calc_align_arm_positions(
        self,
        dist_align_sizing_sets: dict[
            int, tuple[PmxModel, PmxModel, VmdMotion, VmdBoneFrameTrees]
        ],
    ) -> tuple[list[int], list[str], list[tuple[int, str]], dict[int, np.ndarray]]:
        logger.info(
            "腕位置合わせ - 全体位置取得",
            decoration=MLogger.Decoration.LINE,
        )

        fnos_set: set[int] = {0}
        for sizing_idx, (
            src_model,
            dest_model,
            dest_motion,
            src_matrixes,
        ) in dist_align_sizing_sets.items():
            for bone_name in (
                list(ARM_BONE_NAMES) + BoneNames.fingers("右") + BoneNames.fingers("左")
            ):
                fnos_set |= set(dest_motion.bones[bone_name].register_indexes)

        fnos = sorted(fnos_set)

        align_bone_names = [BoneNames.wrist("右"), BoneNames.wrist("左")]
        for direction in ("右", "左"):
            align_bone_names.append(BoneNames.thumb_tail(direction))
            align_bone_names.append(BoneNames.index_tail(direction))
            align_bone_names.append(BoneNames.middle_tail(direction))
            align_bone_names.append(BoneNames.ring_tail(direction))
            align_bone_names.append(BoneNames.pinky_tail(direction))

        bone_distances: np.ndarray = {}

        for fidx, fno in enumerate(fnos):
            logger.count(
                "腕位置合わせ - 全体位置取得",
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            align_bone_indexes: dict[int, tuple[int, str]] = {}
            align_bone_positions: list[np.ndarray] = []

            for sizing_idx, (
                src_model,
                dest_model,
                dest_motion,
                src_matrixes,
            ) in dist_align_sizing_sets.items():
                for bone_name in align_bone_names:
                    align_bone_indexes[len(align_bone_positions)] = (
                        sizing_idx,
                        bone_name,
                    )
                    align_bone_positions.append(
                        src_matrixes[bone_name, fno].position.vector
                    )

            # https://blog.shikoan.com/distance-without-for-loop/
            align_bone_positions_ary = np.array(align_bone_positions)
            bone_diffs = np.expand_dims(
                align_bone_positions_ary, axis=1
            ) - np.expand_dims(align_bone_positions_ary, axis=0)
            bone_distances = np.sqrt(np.sum(bone_diffs**2, axis=-1))

            bone_distances[fno] = bone_distances

        return fnos, align_bone_names, align_bone_indexes, bone_distances

    def sizing_align_arm(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        src_matrixes: VmdBoneFrameTrees,
        all_dest_matrixes: dict[tuple[int, str], VmdBoneFrameTrees],
        fnos: list[int],
        align_bone_names: list[str],
        align_bone_indexes: dict[tuple[int, str], int],
        bone_distances: np.ndarray,
        is_finger: bool,
        is_middle: bool,
        middle_threshold: float,
    ) -> tuple[int, str, VmdMotion]:
        for direction in ("右", "左"):
            self.sizing_arm_align_direction(
                sizing_idx,
                src_model,
                dest_model,
                dest_motion,
                src_matrixes,
                all_dest_matrixes[sizing_idx, direction],
                fnos,
                align_bone_names,
                align_bone_indexes,
                bone_distances,
                is_finger,
                is_middle,
                middle_threshold,
                direction,
            )

    def sizing_arm_align_direction(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        src_matrixes: VmdBoneFrameTrees,
        dest_matrixes: VmdBoneFrameTrees,
        fnos: list[int],
        align_bone_names: list[str],
        align_bone_indexes: dict[tuple[int, str], int],
        bone_distances: np.ndarray,
        is_finger: bool,
        is_middle: bool,
        middle_threshold: float,
        direction: str,
    ) -> tuple[int, str, VmdMotion]:
        logger.info(
            "【No.{i}】【{d}】腕位置合わせ",
            i=sizing_idx + 1,
            d=__(direction),
            decoration=MLogger.Decoration.LINE,
        )

        # 比率を測る ------------------------

        shoulder_ratio = dest_model.bones[
            BoneNames.shoulder_root(direction)
        ].position.distance(
            dest_model.bones[BoneNames.arm(direction)].position
        ) / src_model.bones[
            BoneNames.shoulder_root(direction)
        ].position.distance(
            src_model.bones[BoneNames.arm(direction)].position
        )

        logger.debug(f"shoulder_ratio[{shoulder_ratio:.3f}]")

        arm_ratio = (
            dest_model.bones[BoneNames.shoulder_center(direction)].position.distance(
                dest_model.bones[BoneNames.arm(direction)].position
            )
            + dest_model.bones[BoneNames.arm(direction)].position.distance(
                dest_model.bones[BoneNames.elbow(direction)].position
            )
            + dest_model.bones[BoneNames.elbow(direction)].position.distance(
                dest_model.bones[BoneNames.wrist(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.shoulder_center(direction)].position.distance(
                src_model.bones[BoneNames.arm(direction)].position
            )
            + src_model.bones[BoneNames.arm(direction)].position.distance(
                src_model.bones[BoneNames.elbow(direction)].position
            )
            + src_model.bones[BoneNames.elbow(direction)].position.distance(
                src_model.bones[BoneNames.wrist(direction)].position
            )
        )

        logger.debug(f"arm_ratio[{arm_ratio:.3f}]")

        wrist_ratio = (
            dest_model.bones[BoneNames.wrist_tail(direction)].position.distance(
                dest_model.bones[BoneNames.wrist(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.wrist_tail(direction)].position.distance(
                src_model.bones[BoneNames.wrist(direction)].position
            )
        )

        logger.debug(f"wrist_ratio[{wrist_ratio:.3f}]")

        thumb_ratio = (
            dest_model.bones[BoneNames.wrist(direction)].position.distance(
                dest_model.bones[BoneNames.thumb1(direction)].position
            )
            + dest_model.bones[BoneNames.thumb1(direction)].position.distance(
                dest_model.bones[BoneNames.thumb2(direction)].position
            )
            + dest_model.bones[BoneNames.thumb2(direction)].position.distance(
                dest_model.bones[BoneNames.thumb_tail(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.wrist(direction)].position.distance(
                src_model.bones[BoneNames.thumb1(direction)].position
            )
            + src_model.bones[BoneNames.thumb1(direction)].position.distance(
                src_model.bones[BoneNames.thumb2(direction)].position
            )
            + src_model.bones[BoneNames.thumb2(direction)].position.distance(
                src_model.bones[BoneNames.thumb_tail(direction)].position
            )
        )

        logger.debug(f"thumb_ratio[{thumb_ratio:.3f}]")

        index_ratio = (
            dest_model.bones[BoneNames.index1(direction)].position.distance(
                dest_model.bones[BoneNames.index2(direction)].position
            )
            + dest_model.bones[BoneNames.index2(direction)].position.distance(
                dest_model.bones[BoneNames.index3(direction)].position
            )
            + dest_model.bones[BoneNames.index3(direction)].position.distance(
                dest_model.bones[BoneNames.index_tail(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.index1(direction)].position.distance(
                src_model.bones[BoneNames.index2(direction)].position
            )
            + src_model.bones[BoneNames.index2(direction)].position.distance(
                src_model.bones[BoneNames.index3(direction)].position
            )
            + src_model.bones[BoneNames.index3(direction)].position.distance(
                src_model.bones[BoneNames.index_tail(direction)].position
            )
        )

        logger.debug(f"index_ratio[{index_ratio:.3f}]")

        middle_ratio = (
            dest_model.bones[BoneNames.middle1(direction)].position.distance(
                dest_model.bones[BoneNames.middle2(direction)].position
            )
            + dest_model.bones[BoneNames.middle2(direction)].position.distance(
                dest_model.bones[BoneNames.middle3(direction)].position
            )
            + dest_model.bones[BoneNames.middle3(direction)].position.distance(
                dest_model.bones[BoneNames.middle_tail(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.middle1(direction)].position.distance(
                src_model.bones[BoneNames.middle2(direction)].position
            )
            + src_model.bones[BoneNames.middle2(direction)].position.distance(
                src_model.bones[BoneNames.middle3(direction)].position
            )
            + src_model.bones[BoneNames.middle3(direction)].position.distance(
                src_model.bones[BoneNames.middle_tail(direction)].position
            )
        )

        logger.debug(f"middle_ratio[{middle_ratio:.3f}]")

        ring_ratio = (
            dest_model.bones[BoneNames.ring1(direction)].position.distance(
                dest_model.bones[BoneNames.ring2(direction)].position
            )
            + dest_model.bones[BoneNames.ring2(direction)].position.distance(
                dest_model.bones[BoneNames.ring3(direction)].position
            )
            + dest_model.bones[BoneNames.ring3(direction)].position.distance(
                dest_model.bones[BoneNames.ring_tail(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.ring1(direction)].position.distance(
                src_model.bones[BoneNames.ring2(direction)].position
            )
            + src_model.bones[BoneNames.ring2(direction)].position.distance(
                src_model.bones[BoneNames.ring3(direction)].position
            )
            + src_model.bones[BoneNames.ring3(direction)].position.distance(
                src_model.bones[BoneNames.ring_tail(direction)].position
            )
        )

        logger.debug(f"ring_ratio[{ring_ratio:.3f}]")

        pinky_ratio = (
            dest_model.bones[BoneNames.pinky1(direction)].position.distance(
                dest_model.bones[BoneNames.pinky2(direction)].position
            )
            + dest_model.bones[BoneNames.pinky2(direction)].position.distance(
                dest_model.bones[BoneNames.pinky3(direction)].position
            )
            + dest_model.bones[BoneNames.pinky3(direction)].position.distance(
                dest_model.bones[BoneNames.pinky_tail(direction)].position
            )
        ) / (
            src_model.bones[BoneNames.pinky1(direction)].position.distance(
                src_model.bones[BoneNames.pinky2(direction)].position
            )
            + src_model.bones[BoneNames.pinky2(direction)].position.distance(
                src_model.bones[BoneNames.pinky3(direction)].position
            )
            + src_model.bones[BoneNames.pinky3(direction)].position.distance(
                src_model.bones[BoneNames.pinky_tail(direction)].position
            )
        )

        logger.debug(f"pinky_ratio[{pinky_ratio:.3f}]")

        # 処理対象ボーン名取得
        target_bone_names = dest_motion.bones.get_animate_bone_names(
            dest_model, align_bone_names
        )

        # 処理対象ボーンの行列取得
        (
            bone_dict,
            bone_offset_matrixes,
            bone_pos_matrixes,
        ) = dest_motion.bones.create_bone_matrixes(dest_model, target_bone_names)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】腕位置合わせ - 準備",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            # --------------

            # 腕系があるところは事前にキーフレを登録しておく
            shoulder_bf = dest_motion.bones[BoneNames.shoulder(direction)][fno]
            shoulder_bf.register = True
            dest_motion.insert_bone_frame(shoulder_bf)

            arm_bf = dest_motion.bones[BoneNames.arm(direction)][fno]
            arm_bf.register = True
            dest_motion.insert_bone_frame(arm_bf)

            elbow_bf = dest_motion.bones[BoneNames.elbow(direction)][fno]
            elbow_bf.register = True
            dest_motion.insert_bone_frame(elbow_bf)

            wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
            wrist_bf.register = True
            dest_motion.insert_bone_frame(wrist_bf)

            shoulder_ik_bf = dest_motion.bones[BoneNames.shoulder_ik(direction)][fno]
            shoulder_ik_bf.register = True
            dest_motion.insert_bone_frame(shoulder_ik_bf)

            arm_ik_bf = dest_motion.bones[BoneNames.arm_ik(direction)][fno]
            arm_ik_bf.register = True
            dest_motion.insert_bone_frame(arm_ik_bf)

            wrist_ik_bf = dest_motion.bones[BoneNames.wrist_ik(direction)][fno]
            wrist_ik_bf.register = True
            dest_motion.insert_bone_frame(wrist_ik_bf)

            if is_finger:
                thumb_ik_bf = dest_motion.bones[BoneNames.thumb_ik(direction)][fno]
                thumb_ik_bf.register = True
                dest_motion.insert_bone_frame(thumb_ik_bf)

                thumb0_bf = dest_motion.bones[BoneNames.thumb0(direction)][fno]
                thumb0_bf.register = True
                dest_motion.insert_bone_frame(thumb0_bf)

                thumb1_bf = dest_motion.bones[BoneNames.thumb1(direction)][fno]
                thumb1_bf.register = True
                dest_motion.insert_bone_frame(thumb1_bf)

                thumb2_bf = dest_motion.bones[BoneNames.thumb2(direction)][fno]
                thumb2_bf.register = True
                dest_motion.insert_bone_frame(thumb2_bf)

                index_ik_bf = dest_motion.bones[BoneNames.index_ik(direction)][fno]
                index_ik_bf.register = True
                dest_motion.insert_bone_frame(index_ik_bf)

                index1_bf = dest_motion.bones[BoneNames.index1(direction)][fno]
                index1_bf.register = True
                dest_motion.insert_bone_frame(index1_bf)

                index2_bf = dest_motion.bones[BoneNames.index2(direction)][fno]
                index2_bf.register = True
                dest_motion.insert_bone_frame(index2_bf)

                index3_bf = dest_motion.bones[BoneNames.index3(direction)][fno]
                index3_bf.register = True
                dest_motion.insert_bone_frame(index3_bf)

                middle_ik_bf = dest_motion.bones[BoneNames.middle_ik(direction)][fno]
                middle_ik_bf.register = True
                dest_motion.insert_bone_frame(middle_ik_bf)

                middle1_bf = dest_motion.bones[BoneNames.middle1(direction)][fno]
                middle1_bf.register = True
                dest_motion.insert_bone_frame(middle1_bf)

                middle2_bf = dest_motion.bones[BoneNames.middle2(direction)][fno]
                middle2_bf.register = True
                dest_motion.insert_bone_frame(middle2_bf)

                middle3_bf = dest_motion.bones[BoneNames.middle3(direction)][fno]
                middle3_bf.register = True
                dest_motion.insert_bone_frame(middle3_bf)

                ring_ik_bf = dest_motion.bones[BoneNames.ring_ik(direction)][fno]
                ring_ik_bf.register = True
                dest_motion.insert_bone_frame(ring_ik_bf)

                ring1_bf = dest_motion.bones[BoneNames.ring1(direction)][fno]
                ring1_bf.register = True
                dest_motion.insert_bone_frame(ring1_bf)

                ring2_bf = dest_motion.bones[BoneNames.ring2(direction)][fno]
                ring2_bf.register = True
                dest_motion.insert_bone_frame(ring2_bf)

                ring3_bf = dest_motion.bones[BoneNames.ring3(direction)][fno]
                ring3_bf.register = True
                dest_motion.insert_bone_frame(ring3_bf)

                pinky_ik_bf = dest_motion.bones[BoneNames.pinky_ik(direction)][fno]
                pinky_ik_bf.register = True
                dest_motion.insert_bone_frame(pinky_ik_bf)

                pinky1_bf = dest_motion.bones[BoneNames.pinky1(direction)][fno]
                pinky1_bf.register = True
                dest_motion.insert_bone_frame(pinky1_bf)

                pinky2_bf = dest_motion.bones[BoneNames.pinky2(direction)][fno]
                pinky2_bf.register = True
                dest_motion.insert_bone_frame(pinky2_bf)

                pinky3_bf = dest_motion.bones[BoneNames.pinky3(direction)][fno]
                pinky3_bf.register = True
                dest_motion.insert_bone_frame(pinky3_bf)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】腕位置合わせ",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(fnos),
                display_block=50,
            )

            # モーションボーンの初期値を取得
            (
                is_motion_identity_poses,
                is_motion_identity_qqs,
                is_motion_identity_scales,
                is_motion_identity_local_poses,
                is_motion_identity_local_qqs,
                is_motion_identity_local_scales,
                motion_bone_poses,
                motion_bone_qqs,
                motion_bone_scales,
                motion_bone_local_poses,
                motion_bone_local_qqs,
                motion_bone_local_scales,
                motion_bone_fk_qqs,
            ) = dest_motion.bones.get_bone_matrixes(
                [fno],
                dest_model,
                target_bone_names,
                out_fno_log=False,
            )

            # 腕位置合わせ用行列取得
            self.sizing_arm_align_direction_frame(
                sizing_idx,
                src_model,
                dest_model,
                dest_motion,
                src_matrixes,
                dest_matrixes,
                align_bone_names,
                align_bone_indexes,
                bone_distances,
                is_finger,
                direction,
                bone_dict,
                bone_offset_matrixes,
                bone_pos_matrixes,
                is_motion_identity_poses,
                is_motion_identity_qqs,
                is_motion_identity_scales,
                is_motion_identity_local_poses,
                is_motion_identity_local_qqs,
                is_motion_identity_local_scales,
                motion_bone_poses,
                motion_bone_qqs,
                motion_bone_scales,
                motion_bone_local_poses,
                motion_bone_local_qqs,
                motion_bone_local_scales,
                motion_bone_fk_qqs,
                shoulder_ratio,
                arm_ratio,
                wrist_ratio,
                thumb_ratio,
                index_ratio,
                middle_ratio,
                ring_ratio,
                pinky_ratio,
                fidx,
                fno,
            )

        if is_middle:
            # 腕の長さの一定範囲をズレ許容範囲とする
            threshold = dest_model.bones[f"{direction}腕"].position.distance(
                dest_model.bones[f"{direction}手首"].position
            ) * (0.03 * middle_threshold)

            logger.info(
                "【No.{x}】【{d}】中間キーフレームチェック 【閾値: {t:.3f}】",
                x=sizing_idx + 1,
                d=__(direction),
                t=threshold,
                decoration=MLogger.Decoration.LINE,
            )

            # 全フレームから既に位置合わせ済みのフレームを除く
            middle_fnos = sorted(set(range(dest_motion.bones.max_fno + 1)) - set(fnos))

            for fidx, fno in enumerate(middle_fnos):
                logger.count(
                    "【No.{x}】【{d}】中間腕位置合わせ",
                    x=sizing_idx + 1,
                    d=__(direction),
                    index=fidx,
                    total_index_count=len(middle_fnos),
                    display_block=50,
                )

                # モーションボーンの初期値を取得
                (
                    is_motion_identity_poses,
                    is_motion_identity_qqs,
                    is_motion_identity_scales,
                    is_motion_identity_local_poses,
                    is_motion_identity_local_qqs,
                    is_motion_identity_local_scales,
                    motion_bone_poses,
                    motion_bone_qqs,
                    motion_bone_scales,
                    motion_bone_local_poses,
                    motion_bone_local_qqs,
                    motion_bone_local_scales,
                    motion_bone_fk_qqs,
                ) = dest_motion.bones.get_bone_matrixes(
                    [fno],
                    dest_model,
                    target_bone_names,
                    out_fno_log=False,
                )

                # 初回行列取得
                start_matrixes = dest_motion.bones.calc_bone_matrixes(
                    [fno],
                    dest_model,
                    bone_dict,
                    bone_offset_matrixes,
                    bone_pos_matrixes,
                    is_motion_identity_poses,
                    is_motion_identity_qqs,
                    is_motion_identity_scales,
                    is_motion_identity_local_poses,
                    is_motion_identity_local_qqs,
                    is_motion_identity_local_scales,
                    motion_bone_poses,
                    motion_bone_qqs,
                    motion_bone_scales,
                    motion_bone_local_poses,
                    motion_bone_local_qqs,
                    motion_bone_local_scales,
                    motion_bone_fk_qqs,
                    matrixes=None,
                    out_fno_log=False,
                    description="",
                )

                # 手首の理想位置を求める
                src_arm_local_position = (
                    src_matrixes[BoneNames.wrist(direction), fno].position
                    - src_matrixes[BoneNames.shoulder_center(direction), fno].position
                )
                dest_wrist_global_position = start_matrixes[
                    BoneNames.shoulder_center(direction), fno
                ].position + (src_arm_local_position * arm_ratio)

                wrist_distance = (
                    start_matrixes[f"{direction}手首", fno].position
                ).distance(dest_wrist_global_position)

                if wrist_distance >= threshold:
                    logger.info(
                        "【No.{x}】【{d}】中間キーフレーム追加【{f}F】【{w:.3f} >= {t:.3f}】",
                        x=sizing_idx + 1,
                        d=__(direction),
                        f=fno,
                        w=wrist_distance,
                        t=threshold,
                        decoration=MLogger.Decoration.LINE,
                    )

                    # 腕位置合わせ実行
                    self.sizing_arm_align_direction_frame(
                        sizing_idx,
                        src_model,
                        dest_model,
                        dest_motion,
                        src_matrixes,
                        dest_matrixes,
                        align_bone_names,
                        align_bone_indexes,
                        bone_distances,
                        is_finger,
                        direction,
                        bone_dict,
                        bone_offset_matrixes,
                        bone_pos_matrixes,
                        is_motion_identity_poses,
                        is_motion_identity_qqs,
                        is_motion_identity_scales,
                        is_motion_identity_local_poses,
                        is_motion_identity_local_qqs,
                        is_motion_identity_local_scales,
                        motion_bone_poses,
                        motion_bone_qqs,
                        motion_bone_scales,
                        motion_bone_local_poses,
                        motion_bone_local_qqs,
                        motion_bone_local_scales,
                        motion_bone_fk_qqs,
                        shoulder_ratio,
                        arm_ratio,
                        wrist_ratio,
                        thumb_ratio,
                        index_ratio,
                        middle_ratio,
                        ring_ratio,
                        pinky_ratio,
                        fidx,
                        fno,
                    )

        # 終わったらIKボーンのキーフレを削除
        del dest_motion.bones[BoneNames.shoulder_ik(direction)]
        del dest_motion.bones[BoneNames.arm_ik(direction)]
        del dest_motion.bones[BoneNames.wrist_ik(direction)]
        del dest_motion.bones[BoneNames.thumb_ik(direction)]
        del dest_motion.bones[BoneNames.index_ik(direction)]
        del dest_motion.bones[BoneNames.middle_ik(direction)]
        del dest_motion.bones[BoneNames.ring_ik(direction)]
        del dest_motion.bones[BoneNames.pinky_ik(direction)]

        return sizing_idx, direction, dest_motion

    def sizing_arm_align_direction_frame(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        src_matrixes: VmdBoneFrameTrees,
        dest_matrixes: VmdBoneFrameTrees,
        align_bone_names: list[str],
        align_bone_indexes: dict[tuple[int, str], int],
        bone_distances: np.ndarray,
        is_finger: bool,
        direction: str,
        bone_dict: dict[str, int],
        bone_offset_matrixes: list[tuple[int, np.ndarray]],
        bone_pos_matrixes: np.ndarray,
        is_motion_identity_poses: bool,
        is_motion_identity_qqs: bool,
        is_motion_identity_scales: bool,
        is_motion_identity_local_poses: bool,
        is_motion_identity_local_qqs: bool,
        is_motion_identity_local_scales: bool,
        motion_bone_poses: np.ndarray,
        motion_bone_qqs: np.ndarray,
        motion_bone_scales: np.ndarray,
        motion_bone_local_poses: np.ndarray,
        motion_bone_local_qqs: np.ndarray,
        motion_bone_local_scales: np.ndarray,
        motion_bone_fk_qqs: np.ndarray,
        shoulder_ratio: float,
        arm_ratio: float,
        wrist_ratio: float,
        thumb_ratio: float,
        index_ratio: float,
        middle_ratio: float,
        ring_ratio: float,
        pinky_ratio: float,
        fidx: int,
        fno: int,
    ) -> None:
        # 肩IK --------------------
        src_shoulder_local_position = (
            src_matrixes[BoneNames.arm(direction), fno].position
            - src_matrixes[BoneNames.shoulder_root(direction), fno].position
        )
        dest_shoulder_global_position = dest_matrixes[
            BoneNames.shoulder_root(direction), fno
        ].position + (src_shoulder_local_position * shoulder_ratio)

        shoulder_ik_bf = dest_motion.bones[BoneNames.shoulder_ik(direction)][fno]
        shoulder_ik_bf.position = dest_shoulder_global_position
        dest_motion.append_bone_frame(shoulder_ik_bf)

        # # ■ --------------
        # from datetime import datetime

        # from mlib.vmd.vmd_writer import VmdWriter

        # VmdWriter(
        #     dest_motion,
        #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}肩_{fno:04d}.vmd",
        #     model_name="Test Model",
        # ).save()
        # # ■ --------------

        # IK解決する
        _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
            fidx,
            fno,
            dest_model,
            dest_model.bones[BoneNames.shoulder_ik(direction)],
        )

        shoulder_bf = dest_motion.bones[BoneNames.shoulder(direction)][fno]
        shoulder_bf.rotation = ik_qqs[
            dest_model.bones[BoneNames.shoulder(direction)].index
        ]
        dest_motion.insert_bone_frame(shoulder_bf)

        motion_bone_qqs[0, dest_model.bones[BoneNames.shoulder(direction)].index] = (
            ik_qqs[dest_model.bones[BoneNames.shoulder(direction)].index]
            .to_matrix4x4()
            .vector
        )

        # 肩解決後の行列取得
        shoulder_matrixes = dest_motion.bones.calc_bone_matrixes(
            [fno],
            dest_model,
            bone_dict,
            bone_offset_matrixes,
            bone_pos_matrixes,
            is_motion_identity_poses,
            is_motion_identity_qqs,
            is_motion_identity_scales,
            is_motion_identity_local_poses,
            is_motion_identity_local_qqs,
            is_motion_identity_local_scales,
            motion_bone_poses,
            motion_bone_qqs,
            motion_bone_scales,
            motion_bone_local_poses,
            motion_bone_local_qqs,
            motion_bone_local_scales,
            motion_bone_fk_qqs,
            matrixes=None,
            out_fno_log=False,
            description="",
        )

        # 腕IK --------------------
        src_arm_local_position = (
            src_matrixes[BoneNames.wrist(direction), fno].position
            - src_matrixes[BoneNames.shoulder_center(direction), fno].position
        )
        dest_arm_global_position = shoulder_matrixes[
            BoneNames.shoulder_center(direction), fno
        ].position + (src_arm_local_position * arm_ratio)

        arm_ik_bf = dest_motion.bones[BoneNames.arm_ik(direction)][fno]
        arm_ik_bf.position = dest_arm_global_position
        dest_motion.append_bone_frame(arm_ik_bf)

        # # ■ --------------
        # VmdWriter(
        #     dest_motion,
        #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}腕_{fno:04d}.vmd",
        #     model_name="Test Model",
        # ).save()
        # # ■ --------------

        # IK解決する
        _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
            fidx,
            fno,
            dest_model,
            dest_model.bones[BoneNames.arm_ik(direction)],
        )

        arm_bf = dest_motion.bones[BoneNames.arm(direction)][fno]
        arm_bf.rotation = ik_qqs[dest_model.bones[BoneNames.arm(direction)].index]
        dest_motion.insert_bone_frame(arm_bf)

        elbow_bf = dest_motion.bones[BoneNames.elbow(direction)][fno]
        elbow_bf.rotation = ik_qqs[dest_model.bones[BoneNames.elbow(direction)].index]
        dest_motion.insert_bone_frame(elbow_bf)

        motion_bone_qqs[0, dest_model.bones[BoneNames.arm(direction)].index] = (
            ik_qqs[dest_model.bones[BoneNames.arm(direction)].index]
            .to_matrix4x4()
            .vector
        )

        motion_bone_qqs[0, dest_model.bones[BoneNames.elbow(direction)].index] = (
            ik_qqs[dest_model.bones[BoneNames.elbow(direction)].index]
            .to_matrix4x4()
            .vector
        )

        # 腕解決後の行列取得
        arm_matrixes = dest_motion.bones.calc_bone_matrixes(
            [fno],
            dest_model,
            bone_dict,
            bone_offset_matrixes,
            bone_pos_matrixes,
            is_motion_identity_poses,
            is_motion_identity_qqs,
            is_motion_identity_scales,
            is_motion_identity_local_poses,
            is_motion_identity_local_qqs,
            is_motion_identity_local_scales,
            motion_bone_poses,
            motion_bone_qqs,
            motion_bone_scales,
            motion_bone_local_poses,
            motion_bone_local_qqs,
            motion_bone_local_scales,
            motion_bone_fk_qqs,
            matrixes=None,
            out_fno_log=False,
            description="",
        )

        # 手首IK --------------------
        src_wrist_local_position = (
            src_matrixes[BoneNames.wrist_tail(direction), fno].position
            - src_matrixes[BoneNames.wrist(direction), fno].position
        )
        dest_wrist_global_position = arm_matrixes[
            BoneNames.wrist(direction), fno
        ].position + (src_wrist_local_position * wrist_ratio)

        wrist_ik_bf = dest_motion.bones[BoneNames.wrist_ik(direction)][fno]
        wrist_ik_bf.position = dest_wrist_global_position
        dest_motion.append_bone_frame(wrist_ik_bf)

        # # ■ --------------
        # VmdWriter(
        #     dest_motion,
        #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}手首_{fno:04d}.vmd",
        #     model_name="Test Model",
        # ).save()
        # # ■ --------------

        # IK解決する
        _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
            fidx,
            fno,
            dest_model,
            dest_model.bones[BoneNames.wrist_ik(direction)],
        )

        wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
        wrist_bf.rotation = ik_qqs[dest_model.bones[BoneNames.wrist(direction)].index]
        dest_motion.insert_bone_frame(wrist_bf)

        motion_bone_qqs[0, dest_model.bones[BoneNames.wrist(direction)].index] = (
            ik_qqs[dest_model.bones[BoneNames.wrist(direction)].index]
            .to_matrix4x4()
            .vector
        )

        if is_finger:
            # 手首解決後の行列取得
            wrist_matrixes = dest_motion.bones.calc_bone_matrixes(
                [fno],
                dest_model,
                bone_dict,
                bone_offset_matrixes,
                bone_pos_matrixes,
                is_motion_identity_poses,
                is_motion_identity_qqs,
                is_motion_identity_scales,
                is_motion_identity_local_poses,
                is_motion_identity_local_qqs,
                is_motion_identity_local_scales,
                motion_bone_poses,
                motion_bone_qqs,
                motion_bone_scales,
                motion_bone_local_poses,
                motion_bone_local_qqs,
                motion_bone_local_scales,
                motion_bone_fk_qqs,
                matrixes=None,
                out_fno_log=False,
                description="",
            )

            # 親指IK --------------------
            src_thumb_local_position = (
                src_matrixes[BoneNames.thumb_tail(direction), fno].position
                - src_matrixes[BoneNames.wrist(direction), fno].position
            )
            dest_thumb_global_position = wrist_matrixes[
                BoneNames.wrist(direction), fno
            ].position + (src_thumb_local_position * thumb_ratio)

            thumb_ik_bf = dest_motion.bones[BoneNames.thumb_ik(direction)][fno]
            thumb_ik_bf.position = dest_thumb_global_position
            dest_motion.append_bone_frame(thumb_ik_bf)

            # 人指IK --------------------
            src_index_local_position = (
                src_matrixes[BoneNames.index_tail(direction), fno].position
                - src_matrixes[BoneNames.index1(direction), fno].position
            )
            dest_index_global_position = wrist_matrixes[
                BoneNames.index1(direction), fno
            ].position + (src_index_local_position * index_ratio)

            index_ik_bf = dest_motion.bones[BoneNames.index_ik(direction)][fno]
            index_ik_bf.position = dest_index_global_position
            dest_motion.append_bone_frame(index_ik_bf)

            # 中指IK --------------------
            src_middle_local_position = (
                src_matrixes[BoneNames.middle_tail(direction), fno].position
                - src_matrixes[BoneNames.middle1(direction), fno].position
            )
            dest_middle_global_position = wrist_matrixes[
                BoneNames.middle1(direction), fno
            ].position + (src_middle_local_position * middle_ratio)

            middle_ik_bf = dest_motion.bones[BoneNames.middle_ik(direction)][fno]
            middle_ik_bf.position = dest_middle_global_position
            dest_motion.append_bone_frame(middle_ik_bf)

            # 薬指IK --------------------
            src_ring_local_position = (
                src_matrixes[BoneNames.ring_tail(direction), fno].position
                - src_matrixes[BoneNames.ring1(direction), fno].position
            )
            dest_ring_global_position = wrist_matrixes[
                BoneNames.ring1(direction), fno
            ].position + (src_ring_local_position * ring_ratio)

            ring_ik_bf = dest_motion.bones[BoneNames.ring_ik(direction)][fno]
            ring_ik_bf.position = dest_ring_global_position
            dest_motion.append_bone_frame(ring_ik_bf)

            # 小指IK --------------------
            src_pinky_local_position = (
                src_matrixes[BoneNames.pinky_tail(direction), fno].position
                - src_matrixes[BoneNames.pinky1(direction), fno].position
            )
            dest_pinky_global_position = wrist_matrixes[
                BoneNames.pinky1(direction), fno
            ].position + (src_pinky_local_position * pinky_ratio)

            pinky_ik_bf = dest_motion.bones[BoneNames.pinky_ik(direction)][fno]
            pinky_ik_bf.position = dest_pinky_global_position
            dest_motion.append_bone_frame(pinky_ik_bf)

            # # ■ --------------
            # VmdWriter(
            #     dest_motion,
            #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}指_{fno:04d}.vmd",
            #     model_name="Test Model",
            # ).save()
            # # ■ --------------

            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.thumb_ik(direction)],
            )

            if dest_model.bones[BoneNames.thumb0(direction)].index in ik_qqs:
                thumb0_bf = dest_motion.bones[BoneNames.thumb0(direction)][fno]
                thumb0_bf.rotation = ik_qqs[
                    dest_model.bones[BoneNames.thumb0(direction)].index
                ]
                dest_motion.insert_bone_frame(thumb0_bf)

            thumb1_bf = dest_motion.bones[BoneNames.thumb1(direction)][fno]
            thumb1_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.thumb1(direction)].index
            ]
            dest_motion.insert_bone_frame(thumb1_bf)

            thumb2_bf = dest_motion.bones[BoneNames.thumb2(direction)][fno]
            thumb2_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.thumb2(direction)].index
            ]
            dest_motion.insert_bone_frame(thumb2_bf)

            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.index_ik(direction)],
                ik_qqs,
            )

            index1_bf = dest_motion.bones[BoneNames.index1(direction)][fno]
            index1_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.index1(direction)].index
            ]
            dest_motion.insert_bone_frame(index1_bf)

            index2_bf = dest_motion.bones[BoneNames.index2(direction)][fno]
            index2_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.index2(direction)].index
            ]
            dest_motion.insert_bone_frame(index2_bf)

            index3_bf = dest_motion.bones[BoneNames.index3(direction)][fno]
            index3_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.index3(direction)].index
            ]
            dest_motion.insert_bone_frame(index3_bf)

            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.middle_ik(direction)],
                ik_qqs,
            )

            middle1_bf = dest_motion.bones[BoneNames.middle1(direction)][fno]
            middle1_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.middle1(direction)].index
            ]
            dest_motion.insert_bone_frame(middle1_bf)

            middle2_bf = dest_motion.bones[BoneNames.middle2(direction)][fno]
            middle2_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.middle2(direction)].index
            ]
            dest_motion.insert_bone_frame(middle2_bf)

            middle3_bf = dest_motion.bones[BoneNames.middle3(direction)][fno]
            middle3_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.middle3(direction)].index
            ]
            dest_motion.insert_bone_frame(middle3_bf)

            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.ring_ik(direction)],
                ik_qqs,
            )

            ring1_bf = dest_motion.bones[BoneNames.ring1(direction)][fno]
            ring1_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.ring1(direction)].index
            ]
            dest_motion.insert_bone_frame(ring1_bf)

            ring2_bf = dest_motion.bones[BoneNames.ring2(direction)][fno]
            ring2_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.ring2(direction)].index
            ]
            dest_motion.insert_bone_frame(ring2_bf)

            ring3_bf = dest_motion.bones[BoneNames.ring3(direction)][fno]
            ring3_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.ring3(direction)].index
            ]
            dest_motion.insert_bone_frame(ring3_bf)

            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.pinky_ik(direction)],
                ik_qqs,
            )

            pinky1_bf = dest_motion.bones[BoneNames.pinky1(direction)][fno]
            pinky1_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.pinky1(direction)].index
            ]
            dest_motion.insert_bone_frame(pinky1_bf)

            pinky2_bf = dest_motion.bones[BoneNames.pinky2(direction)][fno]
            pinky2_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.pinky2(direction)].index
            ]
            dest_motion.insert_bone_frame(pinky2_bf)

            pinky3_bf = dest_motion.bones[BoneNames.pinky3(direction)][fno]
            pinky3_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.pinky3(direction)].index
            ]
            dest_motion.insert_bone_frame(pinky3_bf)

        # # ■ --------------
        # VmdWriter(
        #     dest_motion,
        #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}腕位置合わせ解決_{fno:04d}.vmd",
        #     model_name="Test Model",
        # ).save()
        # # ■ --------------

    def setup_model_ik(
        self,
        sizing_idx: int,
        model: PmxModel,
        is_finger: bool,
    ) -> tuple[int, bool, PmxModel]:
        logger.info(
            "【No.{x}】腕位置合わせ：追加IKセットアップ({m})",
            x=sizing_idx + 1,
            m=__("サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        ik_model = model.copy()
        sizing_display_slot = ik_model.display_slots[BoneNames.sizing_display_slot()]

        for direction in ("左", "右"):
            # 肩IK ---------------
            shoulder_ik_bone = Bone(
                index=ik_model.bones[BoneNames.arm(direction)].index + 1,
                name=BoneNames.shoulder_ik(direction),
            )
            shoulder_ik_bone.position = MVector3D()
            shoulder_ik_bone.is_system = True
            shoulder_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(shoulder_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=shoulder_ik_bone.index)
            )

            shoulder_ik = Ik()
            shoulder_ik.bone_index = ik_model.bones[BoneNames.arm(direction)].index
            shoulder_ik.loop_count = 20
            shoulder_ik.unit_rotation.degrees = MVector3D(180, 0, 0)

            # 肩
            shoulder_ik_shoulder = IkLink()
            shoulder_ik_shoulder.bone_index = ik_model.bones[
                BoneNames.shoulder(direction)
            ].index
            shoulder_ik.links.append(shoulder_ik_shoulder)

            shoulder_ik_bone.ik = shoulder_ik

            # 腕IK ---------------
            arm_ik_bone = Bone(
                index=ik_model.bones[BoneNames.wrist(direction)].index + 1,
                name=BoneNames.arm_ik(direction),
            )
            arm_ik_bone.position = MVector3D()
            arm_ik_bone.is_system = True
            arm_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(arm_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_ik_bone.index)
            )

            arm_ik = Ik()
            arm_ik.bone_index = ik_model.bones[BoneNames.wrist(direction)].index
            arm_ik.loop_count = 100
            arm_ik.unit_rotation.degrees = MVector3D(3, 0, 0)

            arm_ik_elbow = IkLink()
            arm_ik_elbow.bone_index = ik_model.bones[BoneNames.elbow(direction)].index
            # ここではZひじも許容する（捩り分散をする場合はそっちで綺麗にする）
            arm_ik.links.append(arm_ik_elbow)

            arm_ik_arm = IkLink()
            arm_ik_arm.bone_index = ik_model.bones[BoneNames.arm(direction)].index
            arm_ik.links.append(arm_ik_arm)

            arm_ik_bone.ik = arm_ik

            # 手首IK ---------------
            wrist_ik_bone = Bone(
                index=ik_model.bones[BoneNames.wrist_tail(direction)].index + 1,
                name=BoneNames.wrist_ik(direction),
            )
            wrist_ik_bone.position = MVector3D()
            wrist_ik_bone.is_system = True
            wrist_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(wrist_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_ik_bone.index)
            )

            wrist_ik = Ik()
            wrist_ik.bone_index = ik_model.bones[BoneNames.wrist_tail(direction)].index
            wrist_ik.loop_count = 20
            wrist_ik.unit_rotation.degrees = MVector3D(180, 0, 0)

            wrist_ik_wrist = IkLink()
            wrist_ik_wrist.bone_index = ik_model.bones[BoneNames.wrist(direction)].index
            wrist_ik.links.append(wrist_ik_wrist)

            wrist_ik_bone.ik = wrist_ik

            if is_finger:
                # 親指IK ---------------
                thumb_ik_bone = Bone(
                    index=ik_model.bones[BoneNames.thumb_tail(direction)].index + 1,
                    name=BoneNames.thumb_ik(direction),
                )
                thumb_ik_bone.position = MVector3D()
                thumb_ik_bone.is_system = True
                thumb_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK
                    | BoneFlg.CAN_TRANSLATE
                    | BoneFlg.CAN_ROTATE
                    | BoneFlg.CAN_MANIPULATE
                    | BoneFlg.IS_VISIBLE
                )
                ik_model.insert_bone(thumb_ik_bone)
                sizing_display_slot.references.append(
                    DisplaySlotReference(display_index=thumb_ik_bone.index)
                )

                thumb_ik = Ik()
                thumb_ik.bone_index = ik_model.bones[
                    BoneNames.thumb_tail(direction)
                ].index
                thumb_ik.loop_count = 100
                thumb_ik.unit_rotation.degrees = MVector3D(3, 0, 0)

                # 親指2
                thumb_ik_thumb2 = IkLink()
                thumb_ik_thumb2.bone_index = ik_model.bones[
                    BoneNames.thumb2(direction)
                ].index
                # ローカル軸での角度制限を行う
                thumb_ik_thumb2.local_angle_limit = True
                thumb_ik_thumb2.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                thumb_ik_thumb2.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                thumb_ik.links.append(thumb_ik_thumb2)

                # 親指1
                thumb_ik_thumb1 = IkLink()
                thumb_ik_thumb1.bone_index = ik_model.bones[
                    BoneNames.thumb1(direction)
                ].index
                thumb_ik.links.append(thumb_ik_thumb1)

                # 親指0(存在する場合のみ)
                if (
                    BoneFlg.NOTHING
                    not in ik_model.bones[BoneNames.thumb0(direction)].bone_flg
                ):
                    thumb_ik_thumb0 = IkLink()
                    thumb_ik_thumb0.bone_index = ik_model.bones[
                        BoneNames.thumb0(direction)
                    ].index
                    thumb_ik.links.append(thumb_ik_thumb0)

                thumb_ik_bone.ik = thumb_ik

                # 人指IK ---------------
                index_ik_bone = Bone(
                    index=ik_model.bones[BoneNames.index_tail(direction)].index + 1,
                    name=BoneNames.index_ik(direction),
                )
                index_ik_bone.position = MVector3D()
                index_ik_bone.is_system = True
                index_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK
                    | BoneFlg.CAN_TRANSLATE
                    | BoneFlg.CAN_ROTATE
                    | BoneFlg.CAN_MANIPULATE
                    | BoneFlg.IS_VISIBLE
                )
                ik_model.insert_bone(index_ik_bone)
                sizing_display_slot.references.append(
                    DisplaySlotReference(display_index=index_ik_bone.index)
                )

                index_ik = Ik()
                index_ik.bone_index = ik_model.bones[
                    BoneNames.index_tail(direction)
                ].index
                index_ik.loop_count = 100
                index_ik.unit_rotation.degrees = MVector3D(3, 0, 0)

                # 人指3
                index_ik_index3 = IkLink()
                index_ik_index3.bone_index = ik_model.bones[
                    BoneNames.index3(direction)
                ].index
                # ローカル軸での角度制限を行う
                index_ik_index3.local_angle_limit = True
                index_ik_index3.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                index_ik_index3.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                index_ik.links.append(index_ik_index3)

                # 人指2
                index_ik_index2 = IkLink()
                index_ik_index2.bone_index = ik_model.bones[
                    BoneNames.index2(direction)
                ].index
                # ローカル軸での角度制限を行う
                index_ik_index2.local_angle_limit = True
                index_ik_index2.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                index_ik_index2.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                index_ik.links.append(index_ik_index2)

                # 人指1
                index_ik_index1 = IkLink()
                index_ik_index1.bone_index = ik_model.bones[
                    BoneNames.index1(direction)
                ].index
                index_ik.links.append(index_ik_index1)

                index_ik_bone.ik = index_ik

                # 中指IK ---------------
                middle_ik_bone = Bone(
                    index=ik_model.bones[BoneNames.middle_tail(direction)].index + 1,
                    name=BoneNames.middle_ik(direction),
                )
                middle_ik_bone.position = MVector3D()
                middle_ik_bone.is_system = True
                middle_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK
                    | BoneFlg.CAN_TRANSLATE
                    | BoneFlg.CAN_ROTATE
                    | BoneFlg.CAN_MANIPULATE
                    | BoneFlg.IS_VISIBLE
                )
                ik_model.insert_bone(middle_ik_bone)
                sizing_display_slot.references.append(
                    DisplaySlotReference(display_index=middle_ik_bone.index)
                )

                middle_ik = Ik()
                middle_ik.bone_index = ik_model.bones[
                    BoneNames.middle_tail(direction)
                ].index
                middle_ik.loop_count = 100
                middle_ik.unit_rotation.degrees = MVector3D(3, 0, 0)

                # 中指3
                middle_ik_middle3 = IkLink()
                middle_ik_middle3.bone_index = ik_model.bones[
                    BoneNames.middle3(direction)
                ].index
                # ローカル軸での角度制限を行う
                middle_ik_middle3.local_angle_limit = True
                middle_ik_middle3.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                middle_ik_middle3.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                middle_ik.links.append(middle_ik_middle3)

                # 中指2
                middle_ik_middle2 = IkLink()
                middle_ik_middle2.bone_index = ik_model.bones[
                    BoneNames.middle2(direction)
                ].index
                # ローカル軸での角度制限を行う
                middle_ik_middle2.local_angle_limit = True
                middle_ik_middle2.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                middle_ik_middle2.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                middle_ik.links.append(middle_ik_middle2)

                # 中指1
                middle_ik_middle1 = IkLink()
                middle_ik_middle1.bone_index = ik_model.bones[
                    BoneNames.middle1(direction)
                ].index
                middle_ik.links.append(middle_ik_middle1)

                middle_ik_bone.ik = middle_ik

                # 薬指IK ---------------
                ring_ik_bone = Bone(
                    index=ik_model.bones[BoneNames.ring_tail(direction)].index + 1,
                    name=BoneNames.ring_ik(direction),
                )
                ring_ik_bone.position = MVector3D()
                ring_ik_bone.is_system = True
                ring_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK
                    | BoneFlg.CAN_TRANSLATE
                    | BoneFlg.CAN_ROTATE
                    | BoneFlg.CAN_MANIPULATE
                    | BoneFlg.IS_VISIBLE
                )
                ik_model.insert_bone(ring_ik_bone)
                sizing_display_slot.references.append(
                    DisplaySlotReference(display_index=ring_ik_bone.index)
                )

                ring_ik = Ik()
                ring_ik.bone_index = ik_model.bones[
                    BoneNames.ring_tail(direction)
                ].index
                ring_ik.loop_count = 100
                ring_ik.unit_rotation.degrees = MVector3D(3, 0, 0)

                # 薬指3
                ring_ik_ring3 = IkLink()
                ring_ik_ring3.bone_index = ik_model.bones[
                    BoneNames.ring3(direction)
                ].index
                # ローカル軸での角度制限を行う
                ring_ik_ring3.local_angle_limit = True
                ring_ik_ring3.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                ring_ik_ring3.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                ring_ik.links.append(ring_ik_ring3)

                # 薬指2
                ring_ik_ring2 = IkLink()
                ring_ik_ring2.bone_index = ik_model.bones[
                    BoneNames.ring2(direction)
                ].index
                # ローカル軸での角度制限を行う
                ring_ik_ring2.local_angle_limit = True
                ring_ik_ring2.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                ring_ik_ring2.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                ring_ik.links.append(ring_ik_ring2)

                # 薬指1
                ring_ik_ring1 = IkLink()
                ring_ik_ring1.bone_index = ik_model.bones[
                    BoneNames.ring1(direction)
                ].index
                ring_ik.links.append(ring_ik_ring1)

                ring_ik_bone.ik = ring_ik

                # 小指IK ---------------
                pinky_ik_bone = Bone(
                    index=ik_model.bones[BoneNames.pinky_tail(direction)].index + 1,
                    name=BoneNames.pinky_ik(direction),
                )
                pinky_ik_bone.position = MVector3D()
                pinky_ik_bone.is_system = True
                pinky_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK
                    | BoneFlg.CAN_TRANSLATE
                    | BoneFlg.CAN_ROTATE
                    | BoneFlg.CAN_MANIPULATE
                    | BoneFlg.IS_VISIBLE
                )
                ik_model.insert_bone(pinky_ik_bone)
                sizing_display_slot.references.append(
                    DisplaySlotReference(display_index=pinky_ik_bone.index)
                )

                pinky_ik = Ik()
                pinky_ik.bone_index = ik_model.bones[
                    BoneNames.pinky_tail(direction)
                ].index
                pinky_ik.loop_count = 100
                pinky_ik.unit_rotation.degrees = MVector3D(3, 0, 0)

                # 小指3
                pinky_ik_pinky3 = IkLink()
                pinky_ik_pinky3.bone_index = ik_model.bones[
                    BoneNames.pinky3(direction)
                ].index
                # ローカル軸での角度制限を行う
                pinky_ik_pinky3.local_angle_limit = True
                pinky_ik_pinky3.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                pinky_ik_pinky3.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                pinky_ik.links.append(pinky_ik_pinky3)

                # 小指2
                pinky_ik_pinky2 = IkLink()
                pinky_ik_pinky2.bone_index = ik_model.bones[
                    BoneNames.pinky2(direction)
                ].index
                # ローカル軸での角度制限を行う
                pinky_ik_pinky2.local_angle_limit = True
                pinky_ik_pinky2.local_min_angle_limit.radians = MVector3D(
                    0, -FINGER_REVERSE_Y_RAD, 0
                )
                pinky_ik_pinky2.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                pinky_ik.links.append(pinky_ik_pinky2)

                # 小指1
                pinky_ik_pinky1 = IkLink()
                pinky_ik_pinky1.bone_index = ik_model.bones[
                    BoneNames.pinky1(direction)
                ].index
                pinky_ik.links.append(pinky_ik_pinky1)

                pinky_ik_bone.ik = pinky_ik

        for direction in ("左", "右"):
            # 肩IK
            ik_model.bones[
                BoneNames.shoulder_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # 腕IK
            ik_model.bones[BoneNames.arm_ik(direction)].parent_index = ik_model.bones[
                BoneNames.root()
            ].index

            # 手首IK
            ik_model.bones[BoneNames.wrist_ik(direction)].parent_index = ik_model.bones[
                BoneNames.root()
            ].index

            if is_finger:
                # 指系IK
                ik_model.bones[
                    BoneNames.thumb_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

                ik_model.bones[
                    BoneNames.index_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

                ik_model.bones[
                    BoneNames.middle_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

                ik_model.bones[
                    BoneNames.ring_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

                ik_model.bones[
                    BoneNames.pinky_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

            # ひじの親は腕捩
            ik_model.bones[BoneNames.elbow(direction)].parent_index = ik_model.bones[
                BoneNames.arm(direction)
            ].index

            # 腕捩の親は腕
            ik_model.bones[
                BoneNames.arm_twist(direction)
            ].parent_index = ik_model.bones[BoneNames.arm(direction)].index

            # 手捩の親はひじ
            ik_model.bones[
                BoneNames.wrist_twist(direction)
            ].parent_index = ik_model.bones[BoneNames.elbow(direction)].index

            # 足IK親の親は全親
            ik_model.bones[
                BoneNames.leg_ik_parent(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # 指根元の親は手首
            ik_model.bones[BoneNames.thumb0(direction)].parent_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index

            ik_model.bones[BoneNames.index1(direction)].parent_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index

            ik_model.bones[BoneNames.middle1(direction)].parent_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index

            ik_model.bones[BoneNames.ring1(direction)].parent_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index

            ik_model.bones[BoneNames.pinky1(direction)].parent_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index

        ik_model.setup()

        if 10 >= logger.total_level:
            # デバッグレベルの場合、モデルを出力する
            from mlib.pmx.pmx_writer import PmxWriter

            PmxWriter(
                ik_model,
                os.path.join(
                    os.path.dirname(ik_model.path),
                    f"align_{os.path.basename(ik_model.path)}",
                ),
                include_system=True,
            ).save()

        return sizing_idx, ik_model

    def get_dest_matrixes(
        self,
        sizing_idx: int,
        model: PmxModel,
        motion: VmdMotion,
        direction: str,
        fnos: list[int],
        align_bone_names: list[str],
    ) -> tuple[int, str, VmdBoneFrameTrees]:
        model_type = __("サイジング先モデル")

        logger.info(
            "【No.{x}】【{d}】初期位置取得({m})",
            x=sizing_idx + 1,
            d=__(direction),
            m=model_type,
            decoration=MLogger.Decoration.LINE,
        )

        initial_matrixes = motion.animate_bone(
            fnos,
            model,
            align_bone_names,
            out_fno_log=True,
            is_calc_ik=False,
            description=f"{sizing_idx + 1}|{__(direction)}|{__('初期位置取得')}|{model_type}",
        )

        return (
            sizing_idx,
            direction,
            initial_matrixes,
        )
