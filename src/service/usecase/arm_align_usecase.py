from math import degrees
import os
from typing import Optional

import numpy as np

from mlib.core.logger import MLogger
from mlib.core.math import MQuaternion, MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlot, DisplaySlotReference, Ik, IkLink
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_part import VmdBoneFrame
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class ArmAlignUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        is_align: bool,
        is_align_finger: bool,
        is_align_finger_tail: bool,
        is_twist: bool,
        show_message: bool = False,
    ) -> bool:
        BONE_NAMES = ["右肩", "右腕", "右ひじ", "右手首", "左肩", "左腕", "左ひじ", "左手首"]

        if not (src_model and dest_model) or (not is_align and not is_align_finger and not is_align_finger_tail and not is_twist):
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        """腕系位置合わせ"""
        if set(BONE_NAMES) - set(src_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{i}】モーション作成元モデルに肩・腕・ひじ・手首の左右ボーンがないため、腕位置合わせをスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if set(BONE_NAMES) - set(dest_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{i}】サイジング先モデルに肩・腕・ひじ・手首の左右ボーンがないため、腕位置合わせをスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_arm_align(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        src_motion: VmdMotion,
        dest_motion: VmdMotion,
        src_initial_matrixes: VmdBoneFrameTrees,
        dest_initial_matrixes: VmdBoneFrameTrees,
        direction: str,
        is_align_finger: bool,
        is_align_finger_tail: bool,
        is_twist: bool,
    ) -> tuple[int, VmdMotion]:
        fnos = dest_initial_matrixes.indexes

        # 比率を測る ------------------------

        shoulder_ratio = dest_model.bones[BoneNames.shoulder_root(direction)].position.distance(
            dest_model.bones[BoneNames.arm(direction)].position
        ) / src_model.bones[BoneNames.shoulder_root(direction)].position.distance(src_model.bones[BoneNames.arm(direction)].position)

        logger.debug(f"shoulder_ratio[{shoulder_ratio:.3f}]")

        arm_ratio = dest_model.bones[BoneNames.shoulder_center(direction)].position.distance(
            dest_model.bones[BoneNames.elbow(direction)].position
        ) / src_model.bones[BoneNames.shoulder_center(direction)].position.distance(src_model.bones[BoneNames.elbow(direction)].position)

        logger.debug(f"arm_ratio[{arm_ratio:.3f}]")

        elbow_ratio = dest_model.bones[BoneNames.arm(direction)].position.distance(
            dest_model.bones[BoneNames.wrist(direction)].position
        ) / src_model.bones[BoneNames.arm(direction)].position.distance(src_model.bones[BoneNames.wrist(direction)].position)

        logger.debug(f"elbow_ratio[{elbow_ratio:.3f}]")

        wrist_ratio = dest_model.bones[BoneNames.elbow(direction)].position.distance(
            dest_model.bones[BoneNames.wrist_tail(direction)].position
        ) / src_model.bones[BoneNames.elbow(direction)].position.distance(src_model.bones[BoneNames.wrist_tail(direction)].position)

        logger.debug(f"wrist_ratio[{wrist_ratio:.3f}]")

        if BoneNames.thumb_tail(direction) in src_model.bones and BoneNames.thumb_tail(direction) in dest_model.bones:
            thumb_ratio = (
                dest_model.bones[BoneNames.wrist(direction)].position - dest_model.bones[BoneNames.thumb_tail(direction)].position
            ) / (src_model.bones[BoneNames.wrist(direction)].position - src_model.bones[BoneNames.thumb_tail(direction)].position)
        else:
            thumb_ratio = 1.0

        logger.debug(f"thumb_ratio[{thumb_ratio:.3f}]")

        if BoneNames.index3(direction) in src_model.bones and BoneNames.index3(direction) in dest_model.bones:
            index_ratio = (
                dest_model.bones[BoneNames.wrist_tail(direction)].position.distance(dest_model.bones[BoneNames.index3(direction)].position)
            ) / (src_model.bones[BoneNames.wrist_tail(direction)].position.distance(src_model.bones[BoneNames.index3(direction)].position))

            index_src_ratio = (
                src_model.bones[BoneNames.middle1(direction)].position.distance(src_model.bones[BoneNames.middle2(direction)].position)
                + src_model.bones[BoneNames.middle2(direction)].position.distance(src_model.bones[BoneNames.middle3(direction)].position)
            ) / (
                src_model.bones[BoneNames.index1(direction)].position.distance(src_model.bones[BoneNames.index2(direction)].position)
                + src_model.bones[BoneNames.index2(direction)].position.distance(src_model.bones[BoneNames.index3(direction)].position)
            )
        else:
            index_ratio = 1.0
            index_src_ratio = 1.0

        logger.debug(f"index_ratio[{index_ratio:.3f}], index_src_ratio[{index_src_ratio:.3f}]")

        if BoneNames.middle3(direction) in src_model.bones and BoneNames.middle3(direction) in dest_model.bones:
            middle_ratio = (
                dest_model.bones[BoneNames.wrist_tail(direction)].position.distance(dest_model.bones[BoneNames.middle3(direction)].position)
            ) / (src_model.bones[BoneNames.wrist_tail(direction)].position.distance(src_model.bones[BoneNames.middle3(direction)].position))
            middle_src_ratio = 1.0
        else:
            middle_ratio = 1.0
            middle_src_ratio = 1.0

        logger.debug(f"middle_ratio[{middle_ratio:.3f}], middle_src_ratio[{middle_src_ratio:.3f}]")

        if BoneNames.ring3(direction) in src_model.bones and BoneNames.ring3(direction) in dest_model.bones:
            ring_ratio = (
                dest_model.bones[BoneNames.wrist_tail(direction)].position.distance(dest_model.bones[BoneNames.ring3(direction)].position)
            ) / (src_model.bones[BoneNames.wrist_tail(direction)].position.distance(src_model.bones[BoneNames.ring3(direction)].position))

            ring_src_ratio = (
                src_model.bones[BoneNames.middle1(direction)].position.distance(src_model.bones[BoneNames.middle2(direction)].position)
                + src_model.bones[BoneNames.middle2(direction)].position.distance(src_model.bones[BoneNames.middle3(direction)].position)
            ) / (
                src_model.bones[BoneNames.ring1(direction)].position.distance(src_model.bones[BoneNames.ring2(direction)].position)
                + src_model.bones[BoneNames.ring2(direction)].position.distance(src_model.bones[BoneNames.ring3(direction)].position)
            )
        else:
            ring_ratio = 1.0
            ring_src_ratio = 1.0

        logger.debug(f"ring_ratio[{ring_ratio:.3f}], ring_src_ratio[{ring_src_ratio:.3f}]")

        if BoneNames.pinky3(direction) in src_model.bones and BoneNames.pinky3(direction) in dest_model.bones:
            pinky_ratio = (
                dest_model.bones[BoneNames.wrist_tail(direction)].position.distance(dest_model.bones[BoneNames.pinky3(direction)].position)
                + dest_model.bones[BoneNames.pinky3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[BoneNames.wrist_tail(direction)].position.distance(src_model.bones[BoneNames.pinky3(direction)].position)
                + src_model.bones[BoneNames.pinky3(direction)].tail_relative_position.length()
            )

            pinky_src_ratio = (
                src_model.bones[BoneNames.middle1(direction)].position.distance(src_model.bones[BoneNames.middle2(direction)].position)
                + src_model.bones[BoneNames.middle2(direction)].position.distance(src_model.bones[BoneNames.middle3(direction)].position)
                + src_model.bones[BoneNames.middle3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[BoneNames.pinky1(direction)].position.distance(src_model.bones[BoneNames.pinky2(direction)].position)
                + src_model.bones[BoneNames.pinky2(direction)].position.distance(src_model.bones[BoneNames.pinky3(direction)].position)
                + src_model.bones[BoneNames.pinky3(direction)].tail_relative_position.length()
            )
        else:
            pinky_ratio = 1.0
            pinky_src_ratio = 1.0

        logger.debug(f"pinky_ratio[{pinky_ratio:.3f}], pinky_src_ratio[{pinky_src_ratio:.3f}]")

        # 腕位置計算 ----------------------------
        logger.info("【No.{i}】{d}腕位置計算", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】{d}腕位置計算",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            # 肩IK --------------------
            shoulder_ik_bf = VmdBoneFrame(fno, BoneNames.shoulder_ik(direction))
            shoulder_ik_bf.position = dest_initial_matrixes[fno, BoneNames.arm(direction)].position
            shoulder_ik_bf.register = True
            dest_motion.append_bone_frame(shoulder_ik_bf)

            # 腕IK --------------------
            arm_ik_bf = VmdBoneFrame(fno, BoneNames.arm_ik(direction))
            arm_ik_bf.position = dest_initial_matrixes[fno, BoneNames.elbow(direction)].position
            arm_ik_bf.register = True
            dest_motion.append_bone_frame(arm_ik_bf)

            # 腕IK --------------------
            elbow_ik_bf = VmdBoneFrame(fno, BoneNames.elbow_ik(direction))
            elbow_ik_bf.position = dest_initial_matrixes[fno, BoneNames.wrist(direction)].position
            elbow_ik_bf.register = True
            dest_motion.append_bone_frame(elbow_ik_bf)

            # 手首IK --------------------
            wrist_ik_bf = VmdBoneFrame(fno, BoneNames.wrist_ik(direction))
            wrist_ik_bf.position = dest_initial_matrixes[fno, BoneNames.wrist_tail(direction)].position
            wrist_ik_bf.register = True
            dest_motion.append_bone_frame(wrist_ik_bf)

            if BoneNames.thumb_tail(direction) in dest_model.bones:
                # 親指IK --------------------
                thumb0_ik_bf = VmdBoneFrame(fno, BoneNames.thumb_ik(direction))
                thumb0_ik_bf.position = dest_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position
                thumb0_ik_bf.register = True
                dest_motion.append_bone_frame(thumb0_ik_bf)

        # 腕位置合わせ ----------------------------
        logger.info("【No.{i}】{d}腕位置合わせ", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】{d}腕位置合わせ", x=sizing_idx + 1, d=__(direction), index=fidx, total_index_count=len(fnos), display_block=1000
            )

            # 肩IK親 --------------------
            src_shoulder_local_position = (
                src_initial_matrixes[fno, BoneNames.arm(direction)].position
                - src_initial_matrixes[fno, BoneNames.shoulder_root(direction)].position
                # src_initial_matrixes[fno, BoneNames.shoulder_root(direction)].global_matrix.inverse()
                # * src_initial_matrixes[fno, BoneNames.arm(direction)].position
            )
            # dest_arm_global_position = dest_initial_matrixes[fno, BoneNames.shoulder_root(direction)].global_matrix * (
            #     src_arm_local_position * shoulder_ratio
            # )
            dest_shoulder_global_position = dest_initial_matrixes[fno, BoneNames.shoulder_root(direction)].position + (
                src_shoulder_local_position * shoulder_ratio
            )

            shoulder_ik_parent_bf = VmdBoneFrame(fno, BoneNames.shoulder_ik_parent(direction))
            shoulder_ik_parent_bf.position = dest_shoulder_global_position - dest_initial_matrixes[fno, BoneNames.arm(direction)].position
            shoulder_ik_parent_bf.register = True
            dest_motion.append_bone_frame(shoulder_ik_parent_bf)
            logger.debug(
                f"[{direction}肩][{fno}][src_arm_local={src_shoulder_local_position}]"
                + f"[dest_arm_global={dest_shoulder_global_position}]"
                + f"[initial_arm_global={dest_initial_matrixes[fno, BoneNames.arm(direction)].position}]"
            )

            # 腕IK親 --------------------
            src_arm_local_position = (
                src_initial_matrixes[fno, BoneNames.elbow(direction)].position
                - src_initial_matrixes[fno, BoneNames.shoulder_center(direction)].position
                # src_initial_matrixes[fno, BoneNames.shoulder_center(direction)].global_matrix.inverse()
                # * src_initial_matrixes[fno, BoneNames.wrist(direction)].position
            )
            dest_arm_global_position = dest_initial_matrixes[fno, BoneNames.shoulder_center(direction)].position + (
                src_arm_local_position * arm_ratio
            )
            # dest_wrist_global_position = dest_initial_matrixes[fno, BoneNames.shoulder_center(direction)].global_matrix * MVector3D(
            #     src_wrist_local_position.x * arm_ratio,
            #     src_wrist_local_position.y * shoulder_ratio,
            #     src_wrist_local_position.z * shoulder_ratio,
            # )

            arm_ik_parent_bf = VmdBoneFrame(fno, BoneNames.arm_ik_parent(direction))
            arm_ik_parent_bf.position = (
                dest_arm_global_position - dest_initial_matrixes[fno, BoneNames.elbow(direction)].position + shoulder_ik_parent_bf.position
            )
            arm_ik_parent_bf.register = True
            dest_motion.append_bone_frame(arm_ik_parent_bf)

            logger.debug(
                f"[{direction}腕][{fno}][src_arm_local_position={src_arm_local_position}]"
                + f"[dest_arm_global_position={dest_arm_global_position}]"
                + f"[initial_elbow_global={dest_initial_matrixes[fno, BoneNames.elbow(direction)].position}]"
            )

            # ひじIK親 --------------------
            src_elbow_local_position = (
                src_initial_matrixes[fno, BoneNames.wrist(direction)].position
                - src_initial_matrixes[fno, BoneNames.arm(direction)].position
            )
            dest_elbow_global_position = dest_initial_matrixes[fno, BoneNames.arm(direction)].position + (
                src_elbow_local_position * elbow_ratio
            )

            elbow_ik_parent_bf = VmdBoneFrame(fno, BoneNames.elbow_ik_parent(direction))
            elbow_ik_parent_bf.position = (
                dest_elbow_global_position - dest_initial_matrixes[fno, BoneNames.wrist(direction)].position + arm_ik_parent_bf.position
            )
            elbow_ik_parent_bf.register = True
            dest_motion.append_bone_frame(elbow_ik_parent_bf)

            logger.debug(
                f"[{direction}腕][{fno}][src_elbow_local_position={src_elbow_local_position}]"
                + f"[dest_elbow_global_position={dest_elbow_global_position}]"
                + f"[initial_elbow_global={dest_initial_matrixes[fno, BoneNames.elbow(direction)].position}]"
            )

            # 手首IK親 --------------------
            src_wrist_tail_local_position = (
                # src_initial_matrixes[fno, BoneNames.wrist(direction)].global_matrix.inverse()
                # * src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].position
                src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].position
                - src_initial_matrixes[fno, BoneNames.elbow(direction)].position
            )
            # dest_wrist_tail_global_position = dest_initial_matrixes[fno, BoneNames.wrist(direction)].global_matrix * MVector3D(
            #     src_wrist_tail_local_position.x * wrist_ratio,
            #     src_wrist_tail_local_position.y * shoulder_ratio,
            #     src_wrist_tail_local_position.z * shoulder_ratio,
            # )
            dest_wrist_tail_global_position = dest_initial_matrixes[fno, BoneNames.elbow(direction)].position + (
                src_wrist_tail_local_position * wrist_ratio
            )

            wrist_ik_parent_bf = VmdBoneFrame(fno, BoneNames.wrist_ik_parent(direction))
            wrist_ik_parent_bf.position = (
                dest_wrist_tail_global_position - dest_initial_matrixes[fno, BoneNames.wrist_tail(direction)].position
            ) + arm_ik_parent_bf.position
            wrist_ik_parent_bf.register = True
            dest_motion.append_bone_frame(wrist_ik_parent_bf)

            logger.debug(
                f"[{direction}手首][{fno}][src_wrist_tail_local={src_wrist_tail_local_position}]"
                + f"[dest_wrist_tail_global={dest_wrist_tail_global_position}]"
                + f"[initial_wrist_tail_global={dest_initial_matrixes[fno, BoneNames.wrist_tail(direction)].position}]"
            )

            if (
                is_align_finger_tail
                and BoneNames.thumb_tail(direction) in src_model.bones
                and BoneNames.thumb_tail(direction) in dest_model.bones
            ):
                # 親指IK親 --------------------

                src_thumb_tail_local_position = (
                    src_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position
                    - src_initial_matrixes[fno, BoneNames.wrist(direction)].position
                )
                dest_thumb_tail_global_position = dest_initial_matrixes[fno, BoneNames.wrist(direction)].position + (
                    src_thumb_tail_local_position * thumb_ratio
                )

                thumb0_ik_parent_bf = VmdBoneFrame(fno, BoneNames.thumb_ik_parent(direction))
                thumb0_ik_parent_bf.position = (
                    dest_thumb_tail_global_position - dest_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position
                ) + wrist_ik_parent_bf.position
                thumb0_ik_parent_bf.register = True
                dest_motion.append_bone_frame(thumb0_ik_parent_bf)

                logger.debug(
                    f"[{direction}親指０][{fno}][src_thumb_tail_local={src_thumb_tail_local_position}]"
                    + f"[dest_thumb_tail_global={dest_thumb_tail_global_position}]"
                    + f"[initial_thumb_tail_global={dest_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position}]"
                )

                # src_thumb_tail_local_position = (
                #     src_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position
                #     - src_initial_matrixes[fno, BoneNames.thumb0(direction)].position
                # )
                # dest_thumb_tail_global_position = dest_initial_matrixes[fno, BoneNames.thumb0(direction)].position + (
                #     src_thumb_tail_local_position * thumb_ratio
                # )

                # thumb_ik_parent_bf = VmdBoneFrame(fno, BoneNames.thumb_ik_parent(direction))
                # thumb_ik_parent_bf.position = (
                #     dest_thumb_tail_global_position - dest_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position
                # ) + wrist_ik_parent_bf.position
                # thumb_ik_parent_bf.register = True
                # dest_motion.append_bone_frame(thumb_ik_parent_bf)

                # logger.debug(
                #     f"[{direction}親指][{fno}][src_thumb_tail_local={src_thumb_tail_local_position}]"
                #     + f"[dest_thumb_tail_global={dest_thumb_tail_global_position}]"
                #     + f"[initial_thumb_tail_global={dest_initial_matrixes[fno, BoneNames.thumb_tail(direction)].position}]"
                # )

        # src_thumb2_local_position = (
        #     src_initial_matrixes[fno, BoneNames.thumb0(direction)].global_matrix_no_scale.inverse()
        #     * src_initial_matrixes[fno, BoneNames.thumb2(direction)].position
        # )
        # dest_thumb2_global_position = dest_initial_matrixes[fno, BoneNames.thumb0(direction)].global_matrix_no_scale * MVector3D(
        #     src_thumb2_local_position.x * thumb0_ratio, src_thumb2_local_position.y, src_thumb2_local_position.z
        # )

        # thumb0_ik_parent_bf = VmdBoneFrame(fno, BoneNames.thumb_ik_parent(direction))
        # thumb0_ik_parent_bf.position = dest_initial_matrixes[
        #     fno, BoneNames.shoulder_center(direction)
        # ].global_matrix.to_quaternion().inverse() * (
        #     dest_thumb2_global_position - dest_initial_matrixes[fno, BoneNames.thumb2(direction)].position
        # )
        # thumb0_ik_parent_bf.register = True
        # dest_motion.append_bone_frame(thumb0_ik_parent_bf)

        # if not (
        #     {
        #         BoneNames.thumb_2(direction),
        #         BoneNames.index_3(direction),
        #         BoneNames.middle_3(direction),
        #         BoneNames.ring_3(direction),
        #         BoneNames.pinky_3(direction),
        #     }
        #     - set(src_model.bones.names)
        # ) and not (
        #     {
        #         BoneNames.thumb_2(direction),
        #         BoneNames.index_3(direction),
        #         BoneNames.middle_3(direction),
        #         BoneNames.ring_3(direction),
        #         BoneNames.pinky_3(direction),
        #     }
        #     - set(dest_model.bones.names)
        # ):
        #     # 指がある場合、指位置合わせ --------------------
        #     src_thumb_tail_position = (
        #         src_initial_matrixes[fno, BoneNames.thumb_2(direction)].global_matrix_no_scale
        #         * src_model.bones[BoneNames.thumb_2(direction)].tail_relative_position
        #     )
        #     src_index_tail_position = (
        #         src_initial_matrixes[fno, BoneNames.index_3(direction)].global_matrix_no_scale
        #         * src_model.bones[BoneNames.index_3(direction)].tail_relative_position
        #     )
        #     src_middle_tail_position = (
        #         src_initial_matrixes[fno, BoneNames.middle_3(direction)].global_matrix_no_scale
        #         * src_model.bones[BoneNames.middle_3(direction)].tail_relative_position
        #     )
        #     src_ring_tail_position = (
        #         src_initial_matrixes[fno, BoneNames.ring_3(direction)].global_matrix_no_scale
        #         * src_model.bones[BoneNames.ring_3(direction)].tail_relative_position
        #     )
        #     src_pinky_tail_position = (
        #         src_initial_matrixes[fno, BoneNames.pinky_3(direction)].global_matrix_no_scale
        #         * src_model.bones[BoneNames.pinky_3(direction)].tail_relative_position
        #     )

        #     # 各指の手首先から見た指先ローカル位置
        #     src_thumb_tail_local_position = (
        #         src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].global_matrix_no_scale.inverse() * src_thumb_tail_position
        #     )
        #     src_index_tail_local_position = (
        #         src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].global_matrix_no_scale.inverse() * src_index_tail_position
        #     )
        #     src_middle_tail_local_position = (
        #         src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].global_matrix_no_scale.inverse() * src_middle_tail_position
        #     )
        #     src_ring_tail_local_position = (
        #         src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].global_matrix_no_scale.inverse() * src_ring_tail_position
        #     )
        #     src_pinky_tail_local_position = (
        #         src_initial_matrixes[fno, BoneNames.wrist_tail(direction)].global_matrix_no_scale.inverse() * src_pinky_tail_position
        #     )

        #     dest_thumb_tail_local_position = src_thumb_tail_local_position * thumb_ratio
        #     dest_index_tail_local_position = src_index_tail_local_position * index_ratio
        #     dest_middle_tail_local_position = src_middle_tail_local_position * middle_ratio
        #     dest_ring_tail_local_position = src_ring_tail_local_position * ring_ratio
        #     dest_pinky_tail_local_position = src_pinky_tail_local_position * pinky_ratio

        # else:

        # if not (
        #     {BoneNames.thumb_2(direction), BoneNames.index_3(direction), BoneNames.middle_3(direction), BoneNames.ring_3(direction), BoneNames.pinky_3(direction)} - set(src_model.bones.names)
        # ) and not (
        #     {BoneNames.thumb_2(direction), BoneNames.index_3(direction), BoneNames.middle_3(direction), BoneNames.ring_3(direction), BoneNames.pinky_3(direction)} - set(dest_model.bones.names)
        # ):
        #     # 指位置合わせ ----------------------------
        #     logger.info("【No.{i}】{d}指位置合わせ", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        #     dest_motion.cache_clear()
        #     dest_finger_matrixes = dest_motion.animate_bone(
        #         finger_fnos,
        #         dest_model,
        #         [
        #             BoneNames.wrist(direction),
        #             BoneNames.thumb_2(direction),
        #             BoneNames.index_3(direction),
        #             BoneNames.middle_3(direction),
        #             BoneNames.ring_3(direction),
        #             BoneNames.pinky_3(direction),
        #             f"{SIZING_BONE_PREFIX}{direction}指IK親",
        #         ],
        #         out_fno_log=True,
        #     )

        #     for fidx, fno in enumerate(finger_fnos):
        #         logger.count(
        #             "【No.{x}】{d}指位置合わせ", x=sizing_idx + 1, d=__(direction), index=fidx, total_index_count=len(finger_fnos), display_block=1000
        #         )

        #         index_src_tail_position = (
        #             src_initial_matrixes[fno, BoneNames.index_3(direction)].global_matrix_no_scale * -src_model.bones[BoneNames.index_3(direction)].tail_relative_position
        #         )
        #         middle_src_tail_position = (
        #             src_initial_matrixes[fno, BoneNames.middle_3(direction)].global_matrix_no_scale * -src_model.bones[BoneNames.middle_3(direction)].tail_relative_position
        #         )
        #         ring_src_tail_position = (
        #             src_initial_matrixes[fno, BoneNames.ring_3(direction)].global_matrix_no_scale * -src_model.bones[BoneNames.ring_3(direction)].tail_relative_position
        #         )
        #         pinky_src_tail_position = (
        #             src_initial_matrixes[fno, BoneNames.pinky_3(direction)].global_matrix_no_scale * -src_model.bones[BoneNames.pinky_3(direction)].tail_relative_position
        #         )

        #         index_dest_tail_position = (
        #             dest_finger_matrixes[fno, BoneNames.index_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[BoneNames.index_3(direction)].tail_relative_position
        #         )
        #         middle_dest_tail_position = (
        #             dest_finger_matrixes[fno, BoneNames.middle_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[BoneNames.middle_3(direction)].tail_relative_position
        #         )
        #         ring_dest_tail_position = (
        #             dest_finger_matrixes[fno, BoneNames.ring_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[BoneNames.ring_3(direction)].tail_relative_position
        #         )
        #         pinky_dest_tail_position = (
        #             dest_finger_matrixes[fno, BoneNames.pinky_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[BoneNames.pinky_3(direction)].tail_relative_position
        #         )

        #         index_src_distance = index_src_tail_position.distance(src_initial_matrixes[fno, BoneNames.wrist(direction)].position) / index_src_ratio
        #         middle_src_distance = middle_src_tail_position.distance(src_initial_matrixes[fno, BoneNames.wrist(direction)].position) / middle_src_ratio
        #         ring_src_distance = ring_src_tail_position.distance(src_initial_matrixes[fno, BoneNames.wrist(direction)].position) / ring_src_ratio
        #         pinky_src_distance = pinky_src_tail_position.distance(src_initial_matrixes[fno, BoneNames.wrist(direction)].position) / pinky_src_ratio
        #         # 最も手首から遠い指を探す
        #         src_far_finger_index = np.argmax([index_src_distance, middle_src_distance, ring_src_distance, pinky_src_distance])
        #         src_far_finger_position = [
        #             index_src_tail_position,
        #             middle_src_tail_position,
        #             ring_src_tail_position,
        #             pinky_src_tail_position,
        #         ][src_far_finger_index]
        #         src_far_finger_ratio = [index_ratio, middle_ratio, ring_ratio, pinky_ratio][src_far_finger_index]

        #         src_finger_local_position = src_initial_matrixes[fno, BoneNames.wrist(direction)].global_matrix_no_scale.inverse() * src_far_finger_position
        #         dest_finger_global_position = dest_finger_matrixes[fno, BoneNames.wrist(direction)].global_matrix_no_scale * (
        #             src_finger_local_position * src_far_finger_ratio
        #         )

        #         far_dest_finger_position = [
        #             index_dest_tail_position,
        #             middle_dest_tail_position,
        #             ring_dest_tail_position,
        #             pinky_dest_tail_position,
        #         ][src_far_finger_index]

        #         finger_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}指IK親")
        #         finger_bf.position = dest_finger_global_position - far_dest_finger_position
        #         dest_motion.append_bone_frame(finger_bf)

        if is_twist:
            # 腕捩
            self.prepare_twist(
                sizing_idx,
                dest_model,
                dest_motion,
                BoneNames.arm(direction),
                BoneNames.arm_twist(direction),
            )
            # 手捩
            self.prepare_twist(
                sizing_idx,
                dest_model,
                dest_motion,
                BoneNames.elbow(direction),
                BoneNames.hand_twist(direction),
            )

        if 10 >= logger.total_level:
            # デバッグレベルの場合、IKのみのVMDも出力する
            from mlib.vmd.vmd_writer import VmdWriter

            path = os.path.join(
                os.path.dirname(dest_motion.path),
                f"IK_{os.path.basename(dest_model.path)}_{direction}_{os.path.basename(dest_motion.path)}",
            )

            os.makedirs(os.path.dirname(path), exist_ok=True)
            VmdWriter(dest_motion, path, dest_model.name).save()

        # IK計算
        dest_ik_result_matrixes = dest_motion.animate_bone(
            fnos,
            dest_model,
            [
                BoneNames.shoulder_root(direction),
                BoneNames.shoulder_center(direction),
                BoneNames.shoulder(direction),
                BoneNames.arm(direction),
                BoneNames.elbow(direction),
                BoneNames.wrist(direction),
                BoneNames.thumb_tail(direction),
            ],
            clear_ik=True,
            out_fno_log=True,
            description=f"{sizing_idx + 1}|{__(direction)}|{__('IK計算')}",
        )

        # IK回転の焼き込み -------------------
        for bone_name in (
            BoneNames.thumb0(direction),
            BoneNames.wrist(direction),
            BoneNames.hand_twist(direction),
            BoneNames.elbow(direction),
            BoneNames.arm_twist(direction),
            BoneNames.arm(direction),
            BoneNames.shoulder(direction),
        ):
            if (
                bone_name not in dest_motion.bones
                or (BoneNames.thumb0(direction) == bone_name and not is_align_finger_tail)
                or (bone_name in (BoneNames.hand_twist(direction), BoneNames.arm_twist(direction)) and not is_twist)
            ):
                continue

            fnos = dest_motion.bones[bone_name].register_indexes
            if bone_name == BoneNames.hand_twist(direction):
                fnos = sorted(
                    set(dest_motion.bones[bone_name].register_indexes) | set(dest_motion.bones[BoneNames.elbow(direction)].register_indexes)
                )
            elif bone_name == BoneNames.arm_twist(direction):
                fnos = sorted(
                    set(dest_motion.bones[bone_name].register_indexes) | set(dest_motion.bones[BoneNames.arm(direction)].register_indexes)
                )

            for fidx, fno in enumerate(fnos):
                logger.count(
                    "【No.{x}】{b}位置合わせ",
                    x=sizing_idx + 1,
                    b=__(bone_name),
                    index=fidx,
                    total_index_count=len(fnos),
                    display_block=1000,
                )

                bf = dest_motion.bones[bone_name][fno]
                # src_vector = (
                #     src_initial_matrixes[fno, tail_bone_name].position - src_initial_matrixes[fno, bone_name].position
                # ).normalized()
                # dest_vector = (
                #     dest_ik_result_matrixes[fno, tail_bone_name].position - dest_ik_result_matrixes[fno, bone_name].position
                # ).normalized()
                # vector_dot = src_vector.dot(dest_vector)

                # if vector_dot < 0.8:
                #     logger.info(
                #         "【No.{i}】[{f}F] {b}位置合わせ失敗 ({d:.3f}:{s}:{e})",
                #         i=sizing_idx + 1,
                #         f=fno,
                #         b=__(bone_name),
                #         d=vector_dot,
                #         s=src_vector,
                #         e=dest_vector,
                #         decoration=MLogger.Decoration.LINE,
                #     )
                # else:
                bf.rotation = dest_ik_result_matrixes[fno, bone_name].frame_rotation
                if bone_name == BoneNames.shoulder(direction):
                    bf.rotation *= dest_motion.bones[BoneNames.shoulder_p(direction)][fno].rotation.inverse()

                if not bf.register:
                    # 捩りとかは登録されてない可能性があるので挿入する
                    bf.register = True
                    dest_motion.insert_bone_frame(bf)

        # 終わったらIKボーンキーフレ削除
        del dest_motion.bones[BoneNames.shoulder_ik_parent(direction)]
        del dest_motion.bones[BoneNames.shoulder_ik(direction)]
        del dest_motion.bones[BoneNames.arm_ik_parent(direction)]
        del dest_motion.bones[BoneNames.arm_ik(direction)]
        del dest_motion.bones[BoneNames.elbow_ik_parent(direction)]
        del dest_motion.bones[BoneNames.elbow_ik(direction)]
        del dest_motion.bones[BoneNames.wrist_ik_parent(direction)]
        del dest_motion.bones[BoneNames.wrist_ik(direction)]
        del dest_motion.bones[BoneNames.thumb_ik_parent(direction)]
        del dest_motion.bones[BoneNames.thumb_ik(direction)]

        return sizing_idx, dest_motion

    def prepare_twist(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        above_bone_name: str,
        twist_bone_name: str,
    ) -> None:
        if not (twist_bone_name in dest_model.bones and above_bone_name in dest_model.bones):
            return

        logger.info("【No.{i}】{b}:捩り分散事前準備", i=sizing_idx + 1, b=twist_bone_name, decoration=MLogger.Decoration.LINE)

        fnos = sorted(set(dest_motion.bones[twist_bone_name].register_indexes) | set(dest_motion.bones[above_bone_name].register_indexes))
        above_local_x_axis = dest_model.bones[above_bone_name].corrected_local_x_vector
        above_local_y_axis = dest_model.bones[above_bone_name].corrected_local_y_vector
        twist_fixed_axis = dest_model.bones[twist_bone_name].corrected_fixed_axis

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】{b}:捩り分散事前準備",
                x=sizing_idx + 1,
                b=twist_bone_name,
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            above_bf = dest_motion.bones[above_bone_name][fno]
            twist_bf = dest_motion.bones[twist_bone_name][fno]

            above_x_qq, _, _, above_yz_qq = above_bf.rotation.separate_by_axis(above_local_x_axis)

            if "ひじ" in above_bone_name:
                above_bf.rotation = above_yz_qq.to_other_axis_rotation(above_local_y_axis)
            else:
                above_bf.rotation = above_yz_qq
            above_bf.register = True
            dest_motion.insert_bone_frame(above_bf)

            twist_qq: MQuaternion = above_x_qq * twist_bf.rotation
            twist_bf.rotation = twist_qq.to_other_axis_rotation(twist_fixed_axis)
            twist_bf.register = True
            dest_motion.insert_bone_frame(twist_bf)

    def get_initial_matrixes(
        self, sizing_idx: int, is_src: bool, model: PmxModel, motion: VmdMotion, direction: str
    ) -> tuple[int, bool, str, VmdBoneFrameTrees]:
        model_type = __("作成元モデル" if is_src else "サイジング先モデル")

        logger.info(
            "【No.{x}】{d}腕位置合わせ：初期位置取得({m})",
            x=sizing_idx + 1,
            d=__(direction),
            m=model_type,
            decoration=MLogger.Decoration.LINE,
        )

        fnos = sorted(
            {0}
            | set(motion.bones[BoneNames.shoulder(direction)].indexes)
            | set(motion.bones[BoneNames.shoulder_p(direction)].indexes)
            | set(motion.bones[BoneNames.arm(direction)].indexes)
            | set(motion.bones[BoneNames.arm_twist(direction)].indexes)
            | set(motion.bones[BoneNames.elbow(direction)].indexes)
            | set(motion.bones[BoneNames.hand_twist(direction)].indexes)
            | set(motion.bones[BoneNames.wrist(direction)].indexes)
            | set(motion.bones[BoneNames.thumb0(direction)].indexes)
        )

        return (
            sizing_idx,
            is_src,
            direction,
            motion.animate_bone(
                fnos,
                model,
                [
                    BoneNames.shoulder_root(direction),
                    BoneNames.shoulder(direction),
                    BoneNames.shoulder_center(direction),
                    BoneNames.arm(direction),
                    BoneNames.elbow(direction),
                    BoneNames.wrist(direction),
                    BoneNames.wrist_tail(direction),
                    BoneNames.thumb_tail(direction),
                    BoneNames.index_tail(direction),
                    BoneNames.middle_tail(direction),
                    BoneNames.ring_tail(direction),
                    BoneNames.pinky_tail(direction),
                ],
                clear_ik=True,
                out_fno_log=True,
                description=f"{sizing_idx + 1}|{__(direction)}|{__('初期位置取得')}|{model_type}",
            ),
        )

    def setup_model_ik(
        self,
        sizing_idx: int,
        is_src: bool,
        model: PmxModel,
        is_align_finger: bool,
        is_align_finger_tail: bool,
        is_twist: bool,
    ) -> None:
        logger.info(
            "【No.{x}】腕位置合わせ：追加IKセットアップ({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        sizing_display_slot = model.display_slots["SIZING"]

        for direction in ("左", "右"):
            if not (
                {
                    BoneNames.shoulder_root(direction),
                    BoneNames.shoulder(direction),
                    BoneNames.arm(direction),
                    BoneNames.wrist(direction),
                }
                - set(model.bones.names)
            ):
                # 肩IK親追加 ---------------
                shoulder_ik_parent_bone = Bone(
                    index=model.bones[BoneNames.arm(direction)].index, name=BoneNames.shoulder_ik_parent(direction)
                )
                shoulder_ik_parent_bone.parent_index = model.bones[BoneNames.root()].index
                shoulder_ik_parent_bone.position = MVector3D()
                shoulder_ik_parent_bone.is_system = True
                shoulder_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(shoulder_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_ik_parent_bone.index))

                # 肩IK追加 ---------------
                shoulder_ik_bone = Bone(index=model.bones[BoneNames.arm(direction)].index, name=BoneNames.shoulder_ik(direction))
                shoulder_ik_bone.parent_index = shoulder_ik_parent_bone.index
                shoulder_ik_bone.position = MVector3D()
                shoulder_ik_bone.is_system = True
                shoulder_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                shoulder_ik = Ik()
                shoulder_ik.bone_index = model.bones[BoneNames.arm(direction)].index
                shoulder_ik.loop_count = 16
                shoulder_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                shoulder_ik_link_shoulder = IkLink()
                shoulder_ik_link_shoulder.bone_index = model.bones[BoneNames.shoulder(direction)].index
                shoulder_ik.links.append(shoulder_ik_link_shoulder)

                shoulder_ik_bone.ik = shoulder_ik
                model.insert_bone(shoulder_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_ik_bone.index))

                # 腕IK親追加 ---------------

                arm_ik_parent_bone = Bone(index=model.bones[BoneNames.elbow(direction)].index, name=BoneNames.arm_ik_parent(direction))
                arm_ik_parent_bone.parent_index = model.bones[BoneNames.root()].index
                arm_ik_parent_bone.position = MVector3D()
                arm_ik_parent_bone.is_system = True
                arm_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(arm_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=arm_ik_parent_bone.index))

                # 腕IK追加 ---------------
                arm_ik_bone = Bone(index=model.bones[BoneNames.elbow(direction)].index, name=BoneNames.arm_ik(direction))
                arm_ik_bone.parent_index = arm_ik_parent_bone.index
                arm_ik_bone.position = MVector3D()
                arm_ik_bone.is_system = True
                arm_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                arm_ik = Ik()
                arm_ik.bone_index = model.bones[BoneNames.elbow(direction)].index
                arm_ik.loop_count = 16
                arm_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                arm_ik_link_arm = IkLink()
                arm_ik_link_arm.bone_index = model.bones[BoneNames.arm(direction)].index
                arm_ik.links.append(arm_ik_link_arm)

                arm_ik_bone.ik = arm_ik
                model.insert_bone(arm_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=arm_ik_bone.index))

                # ひじIK親追加 ---------------

                elbow_ik_parent_bone = Bone(index=model.bones[BoneNames.wrist(direction)].index, name=BoneNames.elbow_ik_parent(direction))
                elbow_ik_parent_bone.parent_index = model.bones[BoneNames.root()].index
                elbow_ik_parent_bone.position = MVector3D()
                elbow_ik_parent_bone.is_system = True
                elbow_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(elbow_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_ik_parent_bone.index))

                # ひじIK追加 ---------------

                elbow_ik_bone = Bone(index=model.bones[BoneNames.wrist(direction)].index, name=BoneNames.elbow_ik(direction))
                elbow_ik_bone.parent_index = elbow_ik_parent_bone.index
                elbow_ik_bone.position = MVector3D()
                elbow_ik_bone.is_system = True
                elbow_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                elbow_ik = Ik()
                elbow_ik.bone_index = model.bones[BoneNames.wrist(direction)].index
                elbow_ik.loop_count = 32
                elbow_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                elbow_ik_link_elbow = IkLink()
                elbow_ik_link_elbow.bone_index = model.bones[BoneNames.elbow(direction)].index
                if is_twist:
                    elbow_ik_link_elbow.local_angle_limit = True
                    elbow_ik_link_elbow.local_min_angle_limit.degrees = MVector3D(0, -3, 0)
                    elbow_ik_link_elbow.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
                elbow_ik.links.append(elbow_ik_link_elbow)

                if BoneNames.arm_twist(direction) in model.bones:
                    elbow_ik_link_arm_twist = IkLink()
                    elbow_ik_link_arm_twist.bone_index = model.bones[BoneNames.arm_twist(direction)].index
                    if not is_twist:
                        elbow_ik_link_arm_twist.angle_limit = True
                    elbow_ik.links.append(elbow_ik_link_arm_twist)

                    for b in model.bones:
                        if BoneNames.arm_twist(direction) in b.name and BoneNames.arm_twist(direction) != b.name:
                            b.layer += 1

                elbow_ik_bone.ik = elbow_ik
                model.insert_bone(elbow_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_ik_bone.index))

                # 手首IK親追加 ---------------

                wrist_ik_parent_bone = Bone(
                    index=model.bones[BoneNames.wrist_tail(direction)].index, name=BoneNames.wrist_ik_parent(direction)
                )
                wrist_ik_parent_bone.parent_index = model.bones[BoneNames.root()].index
                wrist_ik_parent_bone.position = MVector3D()
                wrist_ik_parent_bone.is_system = True
                wrist_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(wrist_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_ik_parent_bone.index))

                # 手首IK追加 ---------------

                wrist_ik_bone = Bone(index=model.bones[BoneNames.wrist_tail(direction)].index, name=BoneNames.wrist_ik(direction))
                wrist_ik_bone.parent_index = wrist_ik_parent_bone.index
                wrist_ik_bone.position = MVector3D()
                wrist_ik_bone.is_system = True
                wrist_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                wrist_ik = Ik()
                wrist_ik.bone_index = model.bones[BoneNames.wrist_tail(direction)].index
                wrist_ik.loop_count = 32
                wrist_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                wrist_ik_link_wrist = IkLink()
                wrist_ik_link_wrist.bone_index = model.bones[BoneNames.wrist(direction)].index
                wrist_ik.links.append(wrist_ik_link_wrist)

                if BoneNames.hand_twist(direction) in model.bones:
                    wrist_ik_link_wrist_twist = IkLink()
                    wrist_ik_link_wrist_twist.bone_index = model.bones[BoneNames.hand_twist(direction)].index
                    if not is_twist:
                        wrist_ik_link_wrist_twist.angle_limit = True
                    wrist_ik.links.append(wrist_ik_link_wrist_twist)

                    for b in model.bones:
                        if BoneNames.hand_twist(direction) in b.name and BoneNames.hand_twist(direction) != b.name:
                            b.layer += 1

                wrist_ik_bone.ik = wrist_ik
                model.insert_bone(wrist_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_ik_bone.index))

            if is_align_finger_tail and BoneNames.thumb0(direction) in model.bones:
                # 親指IK親 追加 ---------------
                thumb0_ik_parent_bone = Bone(
                    index=model.bones[BoneNames.thumb_tail(direction)].index, name=BoneNames.thumb_ik_parent(direction)
                )
                thumb0_ik_parent_bone.parent_index = model.bones[BoneNames.root()].index
                thumb0_ik_parent_bone.position = MVector3D()
                thumb0_ik_parent_bone.is_system = True
                thumb0_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(thumb0_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb0_ik_parent_bone.index))

                # 親指IK追加 ---------------
                thumb0_ik_bone = Bone(index=model.bones[BoneNames.thumb_tail(direction)].index, name=BoneNames.thumb_ik(direction))
                thumb0_ik_bone.parent_index = thumb0_ik_parent_bone.index
                thumb0_ik_bone.position = MVector3D()
                thumb0_ik_bone.is_system = True
                thumb0_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                thumb0_ik = Ik()
                thumb0_ik.bone_index = model.bones[BoneNames.thumb_tail(direction)].index
                thumb0_ik.loop_count = 16
                thumb0_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                thumb0_ik_link_thumb2 = IkLink()
                thumb0_ik_link_thumb2.bone_index = model.bones[BoneNames.thumb2(direction)].index
                # thumb0_ik_link_thumb2.angle_limit = True
                thumb0_ik.links.append(thumb0_ik_link_thumb2)

                thumb0_ik_link_thumb1 = IkLink()
                thumb0_ik_link_thumb1.bone_index = model.bones[BoneNames.thumb1(direction)].index
                # thumb0_ik_link_thumb1.angle_limit = True
                thumb0_ik.links.append(thumb0_ik_link_thumb1)

                thumb0_ik_link_thumb0 = IkLink()
                thumb0_ik_link_thumb0.bone_index = model.bones[BoneNames.thumb0(direction)].index
                thumb0_ik.links.append(thumb0_ik_link_thumb0)

                thumb0_ik_bone.ik = thumb0_ik
                model.insert_bone(thumb0_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb0_ik_bone.index))

        for direction in ("左", "右"):
            if "全ての親" in model.bones:
                model.bones["全ての親"].parent_index = model.bones[BoneNames.root()].index
            # 肩 -------
            if BoneNames.shoulder_p(direction) in model.bones:
                model.bones[BoneNames.shoulder_p(direction)].parent_index = model.bones[BoneNames.shoulder_root(direction)].index
            if BoneNames.shoulder(direction) in model.bones:
                model.bones[BoneNames.shoulder(direction)].parent_index = model.bones[BoneNames.shoulder_root(direction)].index
            # 腕 -------
            if BoneNames.shoulder_ik_parent(direction) in model.bones:
                model.bones[BoneNames.shoulder_ik_parent(direction)].parent_index = model.bones[BoneNames.root()].index
            if BoneNames.shoulder_ik(direction) in model.bones:
                model.bones[BoneNames.shoulder_ik(direction)].parent_index = model.bones[BoneNames.shoulder_ik_parent(direction)].index
                # model.bones[BoneNames.shoulder_ik(direction)].layer = model.bones[BoneNames.shoulder_ik_parent(direction)].layer + 1
            if BoneNames.arm(direction) in model.bones:
                if BoneNames.shoulder_c(direction) in model.bones:
                    model.bones[BoneNames.arm(direction)].parent_index = model.bones[BoneNames.shoulder_c(direction)].index
                else:
                    model.bones[BoneNames.arm(direction)].parent_index = model.bones[BoneNames.shoulder(direction)].index
            if BoneNames.elbow_center(direction) in model.bones:
                model.bones[BoneNames.elbow_center(direction)].parent_index = model.bones[BoneNames.elbow(direction)].index
            # model.bones[BoneNames.arm(direction)].layer = model.bones[BoneNames.shoulder_ik(direction)].layer + 1
            # if BoneNames.arm_twist(direction) in model.bones:
            #     model.bones[BoneNames.arm_twist(direction)].layer = model.bones[BoneNames.arm(direction)].layer

            #     for b in model.bones:
            #         if BoneNames.arm_twist(direction) in b.name and BoneNames.arm_twist(direction) != b.name:
            #             b.layer = model.bones[BoneNames.arm_twist(direction)].layer + 1

            # 手首 -------
            if BoneNames.arm_ik_parent(direction) in model.bones:
                model.bones[BoneNames.arm_ik_parent(direction)].parent_index = model.bones[BoneNames.root()].index
                # model.bones[BoneNames.arm_ik_parent(direction)].layer = (
                #     model.bones[BoneNames.arm_twist(direction)].layer if BoneNames.arm_twist(direction) in model.bones else model.bones[BoneNames.arm(direction)].layer
                # ) + 1
            if BoneNames.arm_ik(direction) in model.bones:
                model.bones[BoneNames.arm_ik(direction)].parent_index = model.bones[BoneNames.arm_ik_parent(direction)].index
                # model.bones[BoneNames.arm_ik(direction)].layer = model.bones[BoneNames.arm_ik_parent(direction)].layer + 1
            # if BoneNames.elbow(direction) in model.bones and BoneNames.arm_ik(direction) in model.bones:
            #     model.bones[BoneNames.elbow(direction)].parent_index = model.bones[BoneNames.arm_ik(direction)].index
            # model.bones[BoneNames.elbow(direction)].layer = model.bones[BoneNames.arm_ik(direction)].layer + 1

            # if BoneNames.hand_twist(direction) in model.bones:
            #     model.bones[BoneNames.hand_twist(direction)].layer = model.bones[BoneNames.elbow(direction)].layer

            #     for b in model.bones:
            #         if BoneNames.hand_twist(direction) in b.name and BoneNames.hand_twist(direction) != b.name:
            #             b.layer = model.bones[BoneNames.hand_twist(direction)].layer + 1

            # 手首 -------
            # if BoneNames.elbow_ik_parent(direction) in model.bones:
            #     model.bones[BoneNames.elbow_ik_parent(direction)].parent_index = model.bones[BoneNames.root()].index
            #     # model.bones[BoneNames.elbow_ik_parent(direction)].layer = (
            #     #     model.bones[BoneNames.hand_twist(direction)].layer if BoneNames.hand_twist(direction) in model.bones else model.bones[BoneNames.elbow(direction)].layer
            #     # ) + 1
            # if BoneNames.elbow_ik(direction) in model.bones:
            #     model.bones[BoneNames.elbow_ik(direction)].parent_index = model.bones[BoneNames.elbow_ik_parent(direction)].index
            # model.bones[BoneNames.elbow_ik(direction)].layer = model.bones[BoneNames.elbow_ik_parent(direction)].layer + 1
            # if BoneNames.wrist(direction) in model.bones and BoneNames.elbow_ik(direction) in model.bones:
            #     model.bones[BoneNames.wrist(direction)].parent_index = model.bones[BoneNames.elbow_ik(direction)].index
            # model.bones[BoneNames.wrist(direction)].layer = model.bones[BoneNames.elbow_ik(direction)].layer + 1
            if BoneNames.wrist_tail(direction) in model.bones:
                model.bones[BoneNames.wrist_tail(direction)].parent_index = model.bones[BoneNames.wrist(direction)].index
            if BoneNames.wrist_ik_parent(direction) in model.bones:
                model.bones[BoneNames.wrist_ik_parent(direction)].parent_index = model.bones[BoneNames.root()].index
                # model.bones[BoneNames.wrist_ik_parent(direction)].layer = model.bones[BoneNames.wrist(direction)].layer + 1
            if BoneNames.wrist_ik(direction) in model.bones:
                model.bones[BoneNames.wrist_ik(direction)].parent_index = model.bones[BoneNames.wrist_ik_parent(direction)].index
                # model.bones[BoneNames.wrist_ik(direction)].layer = model.bones[BoneNames.wrist_ik_parent(direction)].layer + 1
            # 親指０ -------
            if BoneNames.thumb0(direction) in model.bones:
                model.bones[BoneNames.thumb0(direction)].parent_index = model.bones[BoneNames.wrist_tail(direction)].index
            if BoneNames.thumb1(direction) in model.bones:
                if BoneNames.thumb0(direction) in model.bones:
                    model.bones[BoneNames.thumb1(direction)].parent_index = model.bones[BoneNames.thumb0(direction)].index
                else:
                    model.bones[BoneNames.thumb1(direction)].parent_index = model.bones[BoneNames.wrist_tail(direction)].index
            if BoneNames.thumb_ik_parent(direction) in model.bones:
                model.bones[BoneNames.thumb_ik_parent(direction)].parent_index = model.bones[BoneNames.root()].index
                # model.bones[BoneNames.thumb_0_ik_parent(direction)].layer = model.bones[BoneNames.thumb_0(direction)].layer + 1
            if BoneNames.thumb_ik(direction) in model.bones:
                model.bones[BoneNames.thumb_ik(direction)].parent_index = model.bones[BoneNames.thumb_ik_parent(direction)].index
                # model.bones[BoneNames.thumb_0_ik(direction)].layer = model.bones[BoneNames.thumb_0_ik_parent(direction)].layer + 1
            if BoneNames.thumb2(direction) in model.bones:
                model.bones[BoneNames.thumb2(direction)].parent_index = model.bones[BoneNames.thumb1(direction)].index
            # model.bones[BoneNames.thumb_0(direction)].layer = model.bones[BoneNames.wrist_ik(direction)].layer + 1
            # for bname in ("親指１", "親指２"):
            #     if f"{direction}{bname}" in model.bones:
            #         model.bones[f"{direction}{bname}"].layer = (
            #             model.bones[BoneNames.thumb_0(direction)].layer
            #             if BoneNames.thumb_0(direction) in model.bones
            #             else model.bones[BoneNames.wrist(direction)].layer + 1
            #         )
            # 指 -------
            # model.bones[BoneNames.thumb_2(direction)].layer = model.bones[BoneNames.thumb_0_ik(direction)].layer + 1
            # if model.bones[BoneNames.thumb_2(direction)].tail_index in model.bones:
            #     model.bones[model.bones[BoneNames.thumb_2(direction)].tail_index].layer = model.bones[BoneNames.thumb_2(direction)].layer
            for bname in ("人指１", "中指１", "薬指１", "小指１"):
                if f"{direction}{bname}" in model.bones:
                    model.bones[f"{direction}{bname}"].parent_index = model.bones[BoneNames.wrist_tail(direction)].index
            if BoneNames.middle2(direction) in model.bones:
                model.bones[BoneNames.middle2(direction)].parent_index = model.bones[BoneNames.middle1(direction)].index
                # model.bones[f"{direction}{bname}"].layer = model.bones[BoneNames.wrist(direction)].layer + 1
                # if model.bones[f"{direction}{bname}"].tail_index in model.bones:
                #     model.bones[model.bones[f"{direction}{bname}"].tail_index].layer = model.bones[f"{direction}{bname}"].layer
            # for bname in ("人指２", "中指２", "薬指２", "小指２", "人指３", "中指３", "薬指３", "小指３"):
            #     if f"{direction}{bname}" in model.bones:
            #         model.bones[f"{direction}{bname}"].layer = model.bones[BoneNames.wrist(direction)].layer + 1
            #         if model.bones[f"{direction}{bname}"].tail_index in model.bones:
            #             model.bones[model.bones[f"{direction}{bname}"].tail_index].layer = model.bones[f"{direction}{bname}"].layer

        model.setup()

        if 10 >= logger.total_level:
            # デバッグレベルの場合、モデルを出力する
            from mlib.pmx.pmx_writer import PmxWriter

            PmxWriter(
                model, os.path.join(os.path.dirname(model.path), f"sizing_{os.path.basename(model.path)}"), include_system=True
            ).save()
