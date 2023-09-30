import os

from mlib.core.logger import MLogger
from mlib.core.math import MQuaternion, MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlot, DisplaySlotReference, Ik, IkLink
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_part import VmdBoneFrame
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.usecase.io_usecase import SIZING_BONE_PREFIX

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class ArmAlignUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
    ) -> bool:
        BONE_NAMES = ["右肩", "右腕", "右ひじ", "右手首", "左肩", "左腕", "左ひじ", "左手首"]

        """腕系位置合わせ"""
        if set(BONE_NAMES) - set(src_model.bones.names):
            logger.warning(
                "【No.{i}】モーション作成元モデルに肩・腕・ひじ・手首の左右ボーンがないため、腕位置合わせをスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return False

        if set(BONE_NAMES) - set(dest_model.bones.names):
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
        max_worker: int,
    ) -> tuple[int, VmdMotion]:
        fnos = dest_initial_matrixes.indexes

        # 比率を測る ------------------------

        shoulder_ratio = dest_model.bones[f"{direction}肩根元"].position.distance(
            dest_model.bones[f"{direction}腕"].position
        ) / src_model.bones[f"{direction}肩根元"].position.distance(src_model.bones[f"{direction}腕"].position)

        logger.debug(f"shoulder_ratio[{shoulder_ratio:.3f}]")

        arm_ratio = dest_model.bones[f"{direction}腕"].position.distance(dest_model.bones[f"{direction}ひじ"].position) / src_model.bones[
            f"{direction}腕"
        ].position.distance(src_model.bones[f"{direction}ひじ"].position)

        logger.debug(f"arm_ratio[{arm_ratio:.3f}]")

        elbow_ratio = dest_model.bones[f"{direction}ひじ"].position.distance(dest_model.bones[f"{direction}手首"].position) / src_model.bones[
            f"{direction}ひじ"
        ].position.distance(src_model.bones[f"{direction}手首"].position)

        logger.debug(f"elbow_ratio[{elbow_ratio:.3f}]")

        wrist_ratio = dest_model.bones[f"{direction}手首"].position.distance(
            dest_model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].position
        ) / src_model.bones[f"{direction}手首"].position.distance(src_model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].position)

        logger.debug(f"wrist_ratio[{wrist_ratio:.3f}]")

        if f"{direction}親指０" in src_model.bones and f"{direction}親指０" in dest_model.bones:
            thumb0_ratio = dest_model.bones[f"{direction}親指０"].position.distance(
                dest_model.bones[f"{direction}親指１"].position
            ) / src_model.bones[f"{direction}親指０"].position.distance(src_model.bones[f"{direction}親指１"].position)
        else:
            thumb0_ratio = 1.0

        logger.debug(f"thumb0_ratio[{thumb0_ratio:.3f}]")

        if f"{direction}親指２" in src_model.bones and f"{direction}親指２" in dest_model.bones:
            thumb_ratio = (
                dest_model.bones[f"{direction}親指１"].position.distance(dest_model.bones[f"{direction}親指２"].position)
                + dest_model.bones[f"{direction}親指２"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}親指１"].position.distance(src_model.bones[f"{direction}親指２"].position)
                + src_model.bones[f"{direction}親指２"].tail_relative_position.length()
            )
            thumb_src_ratio = (
                src_model.bones[f"{direction}中指１"].position.distance(src_model.bones[f"{direction}中指２"].position)
                + src_model.bones[f"{direction}中指２"].position.distance(src_model.bones[f"{direction}中指３"].position)
                + src_model.bones[f"{direction}中指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}親指１"].position.distance(src_model.bones[f"{direction}親指２"].position)
                + src_model.bones[f"{direction}親指２"].tail_relative_position.length()
            )
        else:
            thumb_ratio = 1.0
            thumb_src_ratio = 1.0

        logger.debug(f"thumb_ratio[{thumb_ratio:.3f}], thumb_src_ratio[{thumb_src_ratio:.3f}]")

        if f"{direction}人指３" in src_model.bones and f"{direction}人指３" in dest_model.bones:
            index_ratio = (
                dest_model.bones[f"{direction}人指１"].position.distance(dest_model.bones[f"{direction}人指２"].position)
                + dest_model.bones[f"{direction}人指２"].position.distance(dest_model.bones[f"{direction}人指３"].position)
                + dest_model.bones[f"{direction}人指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}人指１"].position.distance(src_model.bones[f"{direction}人指２"].position)
                + src_model.bones[f"{direction}人指２"].position.distance(src_model.bones[f"{direction}人指３"].position)
                + src_model.bones[f"{direction}人指３"].tail_relative_position.length()
            )

            index_src_ratio = (
                src_model.bones[f"{direction}中指１"].position.distance(src_model.bones[f"{direction}中指２"].position)
                + src_model.bones[f"{direction}中指２"].position.distance(src_model.bones[f"{direction}中指３"].position)
                + src_model.bones[f"{direction}中指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}人指１"].position.distance(src_model.bones[f"{direction}人指２"].position)
                + src_model.bones[f"{direction}人指２"].position.distance(src_model.bones[f"{direction}人指３"].position)
                + src_model.bones[f"{direction}人指３"].tail_relative_position.length()
            )
        else:
            index_ratio = 1.0
            index_src_ratio = 1.0

        logger.debug(f"index_ratio[{index_ratio:.3f}], index_src_ratio[{index_src_ratio:.3f}]")

        if f"{direction}中指３" in src_model.bones and f"{direction}中指３" in dest_model.bones:
            middle_ratio = (
                dest_model.bones[f"{direction}中指１"].position.distance(dest_model.bones[f"{direction}中指２"].position)
                + dest_model.bones[f"{direction}中指２"].position.distance(dest_model.bones[f"{direction}中指３"].position)
                + dest_model.bones[f"{direction}中指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}中指１"].position.distance(src_model.bones[f"{direction}中指２"].position)
                + src_model.bones[f"{direction}中指２"].position.distance(src_model.bones[f"{direction}中指３"].position)
                + src_model.bones[f"{direction}中指３"].tail_relative_position.length()
            )
            middle_src_ratio = 1.0
        else:
            middle_ratio = 1.0
            middle_src_ratio = 1.0

        logger.debug(f"middle_ratio[{middle_ratio:.3f}], middle_src_ratio[{middle_src_ratio:.3f}]")

        if f"{direction}薬指３" in src_model.bones and f"{direction}薬指３" in dest_model.bones:
            ring_ratio = (
                dest_model.bones[f"{direction}薬指１"].position.distance(dest_model.bones[f"{direction}薬指２"].position)
                + dest_model.bones[f"{direction}薬指２"].position.distance(dest_model.bones[f"{direction}薬指３"].position)
                + dest_model.bones[f"{direction}薬指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}薬指１"].position.distance(src_model.bones[f"{direction}薬指２"].position)
                + src_model.bones[f"{direction}薬指２"].position.distance(src_model.bones[f"{direction}薬指３"].position)
                + src_model.bones[f"{direction}薬指３"].tail_relative_position.length()
            )

            ring_src_ratio = (
                src_model.bones[f"{direction}中指１"].position.distance(src_model.bones[f"{direction}中指２"].position)
                + src_model.bones[f"{direction}中指２"].position.distance(src_model.bones[f"{direction}中指３"].position)
                + src_model.bones[f"{direction}中指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}薬指１"].position.distance(src_model.bones[f"{direction}薬指２"].position)
                + src_model.bones[f"{direction}薬指２"].position.distance(src_model.bones[f"{direction}薬指３"].position)
                + src_model.bones[f"{direction}薬指３"].tail_relative_position.length()
            )
        else:
            ring_ratio = 1.0
            ring_src_ratio = 1.0

        logger.debug(f"ring_ratio[{ring_ratio:.3f}], ring_src_ratio[{ring_src_ratio:.3f}]")

        if f"{direction}小指３" in src_model.bones and f"{direction}小指３" in dest_model.bones:
            pinky_ratio = (
                dest_model.bones[f"{direction}小指１"].position.distance(dest_model.bones[f"{direction}小指２"].position)
                + dest_model.bones[f"{direction}小指２"].position.distance(dest_model.bones[f"{direction}小指３"].position)
                + dest_model.bones[f"{direction}小指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}小指１"].position.distance(src_model.bones[f"{direction}小指２"].position)
                + src_model.bones[f"{direction}小指２"].position.distance(src_model.bones[f"{direction}小指３"].position)
                + src_model.bones[f"{direction}小指３"].tail_relative_position.length()
            )

            pinky_src_ratio = (
                src_model.bones[f"{direction}中指１"].position.distance(src_model.bones[f"{direction}中指２"].position)
                + src_model.bones[f"{direction}中指２"].position.distance(src_model.bones[f"{direction}中指３"].position)
                + src_model.bones[f"{direction}中指３"].tail_relative_position.length()
            ) / (
                src_model.bones[f"{direction}小指１"].position.distance(src_model.bones[f"{direction}小指２"].position)
                + src_model.bones[f"{direction}小指２"].position.distance(src_model.bones[f"{direction}小指３"].position)
                + src_model.bones[f"{direction}小指３"].tail_relative_position.length()
            )
        else:
            pinky_ratio = 1.0
            pinky_src_ratio = 1.0

        logger.debug(f"pinky_ratio[{pinky_ratio:.3f}], pinky_src_ratio[{pinky_src_ratio:.3f}]")

        # 腕位置計算 ----------------------------
        logger.info("【No.{i}】{d}腕位置計算", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        dest_initial_ik_matrixes = dest_motion.animate_bone(
            fnos,
            dest_model,
            [
                f"{direction}肩根元",
            ],
            out_fno_log=True,
        )

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
            shoulder_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}肩IK")
            shoulder_ik_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, f"{direction}腕"].position
                - (dest_initial_ik_matrixes[fno, f"{direction}肩根元"].local_matrix * dest_model.bones[f"{direction}腕"].position)
            )
            shoulder_ik_bf.register = True
            dest_motion.append_bone_frame(shoulder_ik_bf)

            # 腕IK --------------------
            arm_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}腕IK")
            arm_ik_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, f"{direction}ひじ"].position
                - (dest_initial_ik_matrixes[fno, f"{direction}肩根元"].local_matrix * dest_model.bones[f"{direction}ひじ"].position)
            )
            arm_ik_bf.register = True
            dest_motion.append_bone_frame(arm_ik_bf)

            # ひじIK --------------------
            elbow_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}ひじIK")
            elbow_ik_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, f"{direction}手首"].position
                - (dest_initial_ik_matrixes[fno, f"{direction}肩根元"].local_matrix * dest_model.bones[f"{direction}手首"].position)
            )
            elbow_ik_bf.register = True
            dest_motion.append_bone_frame(elbow_ik_bf)

            # 手首IK --------------------
            wrist_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}手首IK")
            wrist_ik_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, f"{SIZING_BONE_PREFIX}{direction}手首先"].position
                - (
                    dest_initial_ik_matrixes[fno, f"{direction}肩根元"].local_matrix
                    * dest_model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].position
                )
            )
            wrist_ik_bf.register = True
            dest_motion.append_bone_frame(wrist_ik_bf)

            if f"{direction}親指０" in dest_model.bones:
                # 親指０IK --------------------
                thumb0_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}親指０IK")
                thumb0_ik_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                    dest_initial_matrixes[fno, f"{direction}親指２"].position
                    - (dest_initial_ik_matrixes[fno, f"{direction}肩根元"].local_matrix * dest_model.bones[f"{direction}親指２"].position)
                )
                thumb0_ik_bf.register = True
                dest_motion.append_bone_frame(thumb0_ik_bf)

        # 肩位置合わせ ----------------------------
        logger.info("【No.{i}】{d}肩位置合わせ", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】{d}肩位置合わせ", x=sizing_idx + 1, d=__(direction), index=fidx, total_index_count=len(fnos), display_block=1000
            )

            # 肩IK親 --------------------
            src_arm_local_position = (
                src_initial_matrixes[fno, f"{direction}肩根元"].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, f"{direction}腕"].position
            )
            dest_arm_global_position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix_no_scale * (
                src_arm_local_position * shoulder_ratio
            )

            shoulder_ik_parent_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}肩IK親")
            shoulder_ik_parent_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_arm_global_position - dest_initial_matrixes[fno, f"{direction}腕"].position
            )
            shoulder_ik_parent_bf.register = True
            dest_motion.append_bone_frame(shoulder_ik_parent_bf)

            # 腕IK親 --------------------
            src_elbow_local_position = (
                src_initial_matrixes[fno, f"{direction}腕"].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, f"{direction}ひじ"].position
            )
            dest_elbow_global_position = dest_initial_matrixes[fno, f"{direction}腕"].global_matrix_no_scale * (
                src_elbow_local_position * arm_ratio
            )

            arm_ik_parent_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}腕IK親")
            arm_ik_parent_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_elbow_global_position - dest_initial_matrixes[fno, f"{direction}ひじ"].position
            )
            arm_ik_parent_bf.register = True
            dest_motion.append_bone_frame(arm_ik_parent_bf)

            # ひじIK親 --------------------
            src_wrist_local_position = (
                src_initial_matrixes[fno, f"{direction}ひじ"].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, f"{direction}手首"].position
            )
            dest_wrist_global_position = dest_initial_matrixes[fno, f"{direction}ひじ"].global_matrix_no_scale * (
                src_wrist_local_position * elbow_ratio
            )

            elbow_ik_parent_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}ひじIK親")
            elbow_ik_parent_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_wrist_global_position - dest_initial_matrixes[fno, f"{direction}手首"].position
            )
            elbow_ik_parent_bf.register = True
            dest_motion.append_bone_frame(elbow_ik_parent_bf)

            # 手首IK親 --------------------
            src_wrist_tail_local_position = (
                src_initial_matrixes[fno, f"{direction}ひじ"].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, f"{direction}手首"].position
            )
            dest_wrist_tail_global_position = dest_initial_matrixes[fno, f"{direction}ひじ"].global_matrix_no_scale * (
                src_wrist_tail_local_position * wrist_ratio
            )

            wrist_ik_parent_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}ひじIK親")
            wrist_ik_parent_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                dest_wrist_tail_global_position - dest_initial_matrixes[fno, f"{direction}手首"].position
            )
            wrist_ik_parent_bf.register = True
            dest_motion.append_bone_frame(wrist_ik_parent_bf)

            if f"{direction}親指０" in src_model.bones and f"{direction}親指０" in dest_model.bones:
                # 親指０IK親 --------------------
                src_thumb2_local_position = (
                    src_initial_matrixes[fno, f"{direction}親指０"].global_matrix_no_scale.inverse()
                    * src_initial_matrixes[fno, f"{direction}親指２"].position
                )
                dest_thumb2_global_position = dest_initial_matrixes[fno, f"{direction}親指０"].global_matrix_no_scale * (
                    src_thumb2_local_position * thumb0_ratio
                )

                thumb0_ik_parent_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}親指０IK親")
                thumb0_ik_parent_bf.position = dest_initial_matrixes[fno, f"{direction}肩根元"].global_matrix.to_quaternion().inverse() * (
                    dest_thumb2_global_position - dest_initial_matrixes[fno, f"{direction}親指２"].position
                )
                thumb0_ik_parent_bf.register = True
                dest_motion.append_bone_frame(thumb0_ik_parent_bf)

        # if not (
        #     {f"{direction}親指２", f"{direction}人指３", f"{direction}中指３", f"{direction}薬指３", f"{direction}小指３"} - set(src_model.bones.names)
        # ) and not (
        #     {f"{direction}親指２", f"{direction}人指３", f"{direction}中指３", f"{direction}薬指３", f"{direction}小指３"} - set(dest_model.bones.names)
        # ):
        #     # 指位置合わせ ----------------------------
        #     logger.info("【No.{i}】{d}指位置合わせ", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        #     dest_motion.cache_clear()
        #     dest_finger_matrixes = dest_motion.animate_bone(
        #         finger_fnos,
        #         dest_model,
        #         [
        #             f"{direction}手首",
        #             f"{direction}親指２",
        #             f"{direction}人指３",
        #             f"{direction}中指３",
        #             f"{direction}薬指３",
        #             f"{direction}小指３",
        #             f"{SIZING_BONE_PREFIX}{direction}指IK親",
        #         ],
        #         out_fno_log=True,
        #     )

        #     for fidx, fno in enumerate(finger_fnos):
        #         logger.count(
        #             "【No.{x}】{d}指位置合わせ", x=sizing_idx + 1, d=__(direction), index=fidx, total_index_count=len(finger_fnos), display_block=1000
        #         )

        #         index_src_tail_position = (
        #             src_initial_matrixes[fno, f"{direction}人指３"].global_matrix_no_scale * -src_model.bones[f"{direction}人指３"].tail_relative_position
        #         )
        #         middle_src_tail_position = (
        #             src_initial_matrixes[fno, f"{direction}中指３"].global_matrix_no_scale * -src_model.bones[f"{direction}中指３"].tail_relative_position
        #         )
        #         ring_src_tail_position = (
        #             src_initial_matrixes[fno, f"{direction}薬指３"].global_matrix_no_scale * -src_model.bones[f"{direction}薬指３"].tail_relative_position
        #         )
        #         pinky_src_tail_position = (
        #             src_initial_matrixes[fno, f"{direction}小指３"].global_matrix_no_scale * -src_model.bones[f"{direction}小指３"].tail_relative_position
        #         )

        #         index_dest_tail_position = (
        #             dest_finger_matrixes[fno, f"{direction}人指３"].global_matrix_no_scale
        #             * -dest_model.bones[f"{direction}人指３"].tail_relative_position
        #         )
        #         middle_dest_tail_position = (
        #             dest_finger_matrixes[fno, f"{direction}中指３"].global_matrix_no_scale
        #             * -dest_model.bones[f"{direction}中指３"].tail_relative_position
        #         )
        #         ring_dest_tail_position = (
        #             dest_finger_matrixes[fno, f"{direction}薬指３"].global_matrix_no_scale
        #             * -dest_model.bones[f"{direction}薬指３"].tail_relative_position
        #         )
        #         pinky_dest_tail_position = (
        #             dest_finger_matrixes[fno, f"{direction}小指３"].global_matrix_no_scale
        #             * -dest_model.bones[f"{direction}小指３"].tail_relative_position
        #         )

        #         index_src_distance = index_src_tail_position.distance(src_initial_matrixes[fno, f"{direction}手首"].position) / index_src_ratio
        #         middle_src_distance = middle_src_tail_position.distance(src_initial_matrixes[fno, f"{direction}手首"].position) / middle_src_ratio
        #         ring_src_distance = ring_src_tail_position.distance(src_initial_matrixes[fno, f"{direction}手首"].position) / ring_src_ratio
        #         pinky_src_distance = pinky_src_tail_position.distance(src_initial_matrixes[fno, f"{direction}手首"].position) / pinky_src_ratio
        #         # 最も手首から遠い指を探す
        #         far_src_finger_index = np.argmax([index_src_distance, middle_src_distance, ring_src_distance, pinky_src_distance])
        #         far_src_finger_position = [
        #             index_src_tail_position,
        #             middle_src_tail_position,
        #             ring_src_tail_position,
        #             pinky_src_tail_position,
        #         ][far_src_finger_index]
        #         far_src_finger_ratio = [index_ratio, middle_ratio, ring_ratio, pinky_ratio][far_src_finger_index]

        #         src_finger_local_position = src_initial_matrixes[fno, f"{direction}手首"].global_matrix_no_scale.inverse() * far_src_finger_position
        #         dest_finger_global_position = dest_finger_matrixes[fno, f"{direction}手首"].global_matrix_no_scale * (
        #             src_finger_local_position * far_src_finger_ratio
        #         )

        #         far_dest_finger_position = [
        #             index_dest_tail_position,
        #             middle_dest_tail_position,
        #             ring_dest_tail_position,
        #             pinky_dest_tail_position,
        #         ][far_src_finger_index]

        #         finger_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}指IK親")
        #         finger_bf.position = dest_finger_global_position - far_dest_finger_position
        #         dest_motion.append_bone_frame(finger_bf)

        # IK計算
        dest_motion.animate_bone(
            fnos,
            dest_model,
            [
                f"{direction}肩根元",
                f"{direction}肩",
                f"{direction}腕",
                f"{direction}ひじ",
                f"{direction}手首",
                f"{direction}親指２",
            ],
            clear_ik=True,
            out_fno_log=True,
        )

        # IK回転の焼き込み -------------------
        for bone_name in (f"{direction}親指０", f"{direction}手首", f"{direction}ひじ", f"{direction}腕", f"{direction}肩"):
            if bone_name not in dest_motion.bones:
                continue
            for fno in dest_motion.bones[bone_name].register_indexes:
                dest_motion.bones[bone_name][fno].rotation *= dest_motion.bones[bone_name][fno].ik_rotation or MQuaternion()
            # 終わったらIKボーンキーフレ削除
            del dest_motion.bones[f"{SIZING_BONE_PREFIX}{bone_name}IK親"]
            del dest_motion.bones[f"{SIZING_BONE_PREFIX}{bone_name}IK"]

        return sizing_idx, dest_motion

    def get_initial_matrixes(
        self, sizing_idx: int, is_src: bool, model: PmxModel, motion: VmdMotion, direction: str
    ) -> tuple[int, bool, str, VmdBoneFrameTrees]:
        logger.info(
            "【No.{x}】{d}腕位置合わせ：初期位置取得({m})",
            x=sizing_idx + 1,
            d=__(direction),
            m=__("作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        fnos = sorted(
            {0}
            | set(motion.bones["センター"].indexes)
            | set(motion.bones["グルーブ"].indexes)
            | set(motion.bones["腰"].indexes)
            | set(motion.bones["上半身"].indexes)
            | set(motion.bones["上半身2"].indexes)
            | set(motion.bones["上半身3"].indexes)
            | set(motion.bones[f"{direction}肩"].indexes)
            | set(motion.bones[f"{direction}肩P"].indexes)
            | set(motion.bones[f"{direction}腕"].indexes)
            | set(motion.bones[f"{direction}腕捩"].indexes)
            | set(motion.bones[f"{direction}ひじ"].indexes)
            | set(motion.bones[f"{direction}手捩"].indexes)
            | set(motion.bones[f"{direction}手首"].indexes)
            | set(motion.bones[f"{direction}親指０"].indexes)
            | set(motion.bones[f"{direction}親指１"].indexes)
            | set(motion.bones[f"{direction}親指２"].indexes)
        )

        return (
            sizing_idx,
            is_src,
            direction,
            motion.animate_bone(
                fnos,
                model,
                [
                    f"{direction}肩根元",
                    f"{direction}肩",
                    f"{direction}腕",
                    f"{direction}ひじ",
                    f"{direction}手首",
                    f"{direction}親指２",
                    f"{direction}人指３",
                    f"{direction}中指３",
                    f"{direction}薬指３",
                    f"{direction}小指３",
                ],
                clear_ik=True,
                out_fno_log=True,
            ),
        )

    def setup_model(self, sizing_idx: int, is_src: bool, model: PmxModel) -> None:
        logger.info(
            "【No.{x}】腕位置合わせ：追加セットアップ({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        sizing_display_slot = DisplaySlot(name="SIZING")
        sizing_display_slot.is_system = True
        model.display_slots.append(sizing_display_slot)

        if "全ての親" not in model.bones:
            root_bone = Bone(index=0, name="全ての親")
            root_bone.parent_index = -1
            root_bone.is_system = True
            root_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

            model.insert_bone(root_bone)
            sizing_display_slot.references.append(DisplaySlotReference(display_index=root_bone.index))

        for direction in ("左", "右"):
            if not ({f"{direction}肩根元", f"{direction}肩", f"{direction}腕", f"{direction}手首"} - set(model.bones.names)):
                # 手首先追加 ---------------

                wrist_tail_bone = Bone(index=model.bones[f"{direction}手首"].index + 1, name=f"{SIZING_BONE_PREFIX}{direction}手首先")
                wrist_tail_bone.parent_index = model.bones[f"{direction}手首"].index
                wrist_tail_bone.position = (
                    model.bones[f"{direction}手首"].position
                    + (model.bones[f"{direction}手首"].position - model.bones[f"{direction}ひじ"].position) / 2
                )
                wrist_tail_bone.is_system = True
                wrist_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(wrist_tail_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_tail_bone.index))

        model.setup()

        for direction in ("左", "右"):
            if f"{direction}肩P" in model.bones:
                model.bones[f"{direction}肩P"].parent_index = model.bones[f"{direction}肩根元"].index
            if f"{direction}肩" in model.bones:
                model.bones[f"{direction}肩"].parent_index = model.bones[f"{direction}肩根元"].index
            if f"{SIZING_BONE_PREFIX}{direction}手首先" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].parent_index = model.bones[f"{direction}手首"].index

    def setup_model_ik(self, sizing_idx: int, is_src: bool, model: PmxModel) -> None:
        logger.info(
            "【No.{x}】腕位置合わせ：追加IKセットアップ({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        sizing_display_slot = model.display_slots["SIZING"]

        for direction in ("左", "右"):
            if not ({f"{direction}肩根元", f"{direction}肩", f"{direction}腕", f"{direction}手首"} - set(model.bones.names)):
                # 肩IK親追加 ---------------
                shoulder_ik_parent_bone = Bone(index=model.bones[f"{direction}腕"].index, name=f"{SIZING_BONE_PREFIX}{direction}肩IK親")
                shoulder_ik_parent_bone.parent_index = model.bones[f"{direction}肩根元"].index
                shoulder_ik_parent_bone.position = model.bones[f"{direction}腕"].position.copy()
                shoulder_ik_parent_bone.is_system = True
                shoulder_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(shoulder_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_ik_parent_bone.index))

                # 肩IK追加 ---------------
                shoulder_ik_bone = Bone(index=model.bones[f"{direction}腕"].index, name=f"{SIZING_BONE_PREFIX}{direction}肩IK")
                shoulder_ik_bone.parent_index = shoulder_ik_parent_bone.index
                shoulder_ik_bone.position = model.bones[f"{direction}腕"].position.copy()
                shoulder_ik_bone.is_system = True
                shoulder_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                shoulder_ik = Ik()
                shoulder_ik.bone_index = model.bones[f"{direction}腕"].index
                shoulder_ik.loop_count = 10
                shoulder_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                shoulder_ik_link_shoulder = IkLink()
                shoulder_ik_link_shoulder.bone_index = model.bones[f"{direction}肩"].index
                shoulder_ik.links.append(shoulder_ik_link_shoulder)

                shoulder_ik_bone.ik = shoulder_ik
                model.insert_bone(shoulder_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_ik_bone.index))

                # 腕IK親追加 ---------------

                arm_ik_parent_bone = Bone(index=model.bones[f"{direction}ひじ"].index, name=f"{SIZING_BONE_PREFIX}{direction}腕IK親")
                arm_ik_parent_bone.parent_index = model.bones[f"{direction}肩根元"].index
                arm_ik_parent_bone.position = model.bones[f"{direction}ひじ"].position.copy()
                arm_ik_parent_bone.is_system = True
                arm_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(arm_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=arm_ik_parent_bone.index))

                # 腕IK追加 ---------------
                arm_ik_bone = Bone(index=model.bones[f"{direction}ひじ"].index, name=f"{SIZING_BONE_PREFIX}{direction}腕IK")
                arm_ik_bone.parent_index = arm_ik_parent_bone.index
                arm_ik_bone.position = model.bones[f"{direction}ひじ"].position.copy()
                arm_ik_bone.is_system = True
                arm_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                arm_ik = Ik()
                arm_ik.bone_index = model.bones[f"{direction}ひじ"].index
                arm_ik.loop_count = 10
                arm_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                if f"{direction}腕捩" in model.bones:
                    arm_ik_link_arm_twist = IkLink()
                    arm_ik_link_arm_twist.bone_index = model.bones[f"{direction}腕捩"].index
                    arm_ik_link_arm_twist.angle_limit = True
                    arm_ik.links.append(arm_ik_link_arm_twist)

                    for b in model.bones:
                        if f"{direction}腕捩" in b.name and f"{direction}腕捩" != b.name:
                            b.layer += 1

                arm_ik_link_arm = IkLink()
                arm_ik_link_arm.bone_index = model.bones[f"{direction}腕"].index
                arm_ik.links.append(arm_ik_link_arm)

                arm_ik_bone.ik = arm_ik
                model.insert_bone(arm_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=arm_ik_bone.index))

                # ひじIK親追加 ---------------

                elbow_ik_parent_bone = Bone(index=model.bones[f"{direction}手首"].index, name=f"{SIZING_BONE_PREFIX}{direction}ひじIK親")
                elbow_ik_parent_bone.parent_index = model.bones[f"{direction}肩根元"].index
                elbow_ik_parent_bone.position = model.bones[f"{direction}手首"].position.copy()
                elbow_ik_parent_bone.is_system = True
                elbow_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(elbow_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_ik_parent_bone.index))

                # ひじIK追加 ---------------

                elbow_ik_bone = Bone(index=model.bones[f"{direction}手首"].index, name=f"{SIZING_BONE_PREFIX}{direction}ひじIK")
                elbow_ik_bone.parent_index = elbow_ik_parent_bone.index
                elbow_ik_bone.position = model.bones[f"{direction}手首"].position.copy()
                elbow_ik_bone.is_system = True
                elbow_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                elbow_ik = Ik()
                elbow_ik.bone_index = model.bones[f"{direction}手首"].index
                elbow_ik.loop_count = 10
                elbow_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                if f"{direction}手捩" in model.bones:
                    elbow_ik_link_wrist_twist = IkLink()
                    elbow_ik_link_wrist_twist.bone_index = model.bones[f"{direction}手捩"].index
                    elbow_ik_link_wrist_twist.angle_limit = True
                    elbow_ik.links.append(elbow_ik_link_wrist_twist)

                    for b in model.bones:
                        if f"{direction}手捩" in b.name and f"{direction}手捩" != b.name:
                            b.layer += 1

                elbow_ik_link_elbow = IkLink()
                elbow_ik_link_elbow.bone_index = model.bones[f"{direction}ひじ"].index
                elbow_ik.links.append(elbow_ik_link_elbow)

                elbow_ik_bone.ik = elbow_ik
                model.insert_bone(elbow_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_ik_bone.index))

                # 手首IK親追加 ---------------

                wrist_ik_parent_bone = Bone(
                    index=model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].index, name=f"{SIZING_BONE_PREFIX}{direction}手首IK親"
                )
                wrist_ik_parent_bone.parent_index = model.bones[f"{direction}肩根元"].index
                wrist_ik_parent_bone.position = model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].position.copy()
                wrist_ik_parent_bone.is_system = True
                wrist_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(wrist_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_ik_parent_bone.index))

                # 手首IK追加 ---------------

                wrist_ik_bone = Bone(
                    index=model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].index, name=f"{SIZING_BONE_PREFIX}{direction}手首IK"
                )
                wrist_ik_bone.parent_index = wrist_ik_parent_bone.index
                wrist_ik_bone.position = model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].position.copy()
                wrist_ik_bone.is_system = True
                wrist_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                wrist_ik = Ik()
                wrist_ik.bone_index = model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].index
                wrist_ik.loop_count = 10
                wrist_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                wrist_ik_link_wrist = IkLink()
                wrist_ik_link_wrist.bone_index = model.bones[f"{direction}手首"].index
                wrist_ik.links.append(wrist_ik_link_wrist)

                wrist_ik_bone.ik = wrist_ik
                model.insert_bone(wrist_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_ik_bone.index))

                if f"{direction}親指０" in model.bones:
                    # 親指０IK親 追加 ---------------
                    thumb0_ik_parent_bone = Bone(index=model.bones[f"{direction}親指２"].index, name=f"{SIZING_BONE_PREFIX}{direction}親指０IK親")
                    thumb0_ik_parent_bone.parent_index = model.bones[f"{direction}肩根元"].index
                    thumb0_ik_parent_bone.position = model.bones[f"{direction}親指２"].position.copy()
                    thumb0_ik_parent_bone.is_system = True
                    thumb0_ik_parent_bone.bone_flg |= (
                        BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                    )

                    model.insert_bone(thumb0_ik_parent_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb0_ik_parent_bone.index))

                    # 親指０IK追加 ---------------
                    thumb0_ik_bone = Bone(index=model.bones[f"{direction}親指２"].index, name=f"{SIZING_BONE_PREFIX}{direction}親指０IK")
                    thumb0_ik_bone.parent_index = thumb0_ik_parent_bone.index
                    thumb0_ik_bone.position = model.bones[f"{direction}親指２"].position.copy()
                    thumb0_ik_bone.is_system = True
                    thumb0_ik_bone.bone_flg |= (
                        BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                    )

                    thumb0_ik = Ik()
                    thumb0_ik.bone_index = model.bones[f"{direction}親指２"].index
                    thumb0_ik.loop_count = 10
                    thumb0_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                    thumb0_ik_link_thumb1 = IkLink()
                    thumb0_ik_link_thumb1.bone_index = model.bones[f"{direction}親指１"].index
                    thumb0_ik_link_thumb1.angle_limit = True
                    thumb0_ik.links.append(thumb0_ik_link_thumb1)

                    thumb0_ik_link_thumb0 = IkLink()
                    thumb0_ik_link_thumb0.bone_index = model.bones[f"{direction}親指０"].index
                    thumb0_ik.links.append(thumb0_ik_link_thumb0)

                    thumb0_ik_bone.ik = thumb0_ik
                    model.insert_bone(thumb0_ik_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb0_ik_bone.index))

        for direction in ("左", "右"):
            # 肩 -------
            if f"{direction}肩P" in model.bones:
                model.bones[f"{direction}肩P"].parent_index = model.bones[f"{direction}肩根元"].index
            if f"{direction}肩" in model.bones:
                model.bones[f"{direction}肩"].parent_index = model.bones[f"{direction}肩根元"].index
            # 腕 -------
            if f"{SIZING_BONE_PREFIX}{direction}肩IK親" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK親"].parent_index = model.bones[f"{direction}肩根元"].index
            if f"{SIZING_BONE_PREFIX}{direction}肩IK" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK"].parent_index = model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK親"].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK親"].layer + 1
            # if f"{direction}腕" in model.bones and f"{SIZING_BONE_PREFIX}{direction}肩IK" in model.bones:
            #     model.bones[f"{direction}腕"].parent_index = model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK"].index
            # model.bones[f"{direction}腕"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}肩IK"].layer + 1
            # if f"{direction}腕捩" in model.bones:
            #     model.bones[f"{direction}腕捩"].layer = model.bones[f"{direction}腕"].layer

            #     for b in model.bones:
            #         if f"{direction}腕捩" in b.name and f"{direction}腕捩" != b.name:
            #             b.layer = model.bones[f"{direction}腕捩"].layer + 1

            # ひじ -------
            if f"{SIZING_BONE_PREFIX}{direction}腕IK親" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK親"].parent_index = model.bones[f"{direction}肩根元"].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK親"].layer = (
                #     model.bones[f"{direction}腕捩"].layer if f"{direction}腕捩" in model.bones else model.bones[f"{direction}腕"].layer
                # ) + 1
            if f"{SIZING_BONE_PREFIX}{direction}腕IK" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK"].parent_index = model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK親"].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK親"].layer + 1
            # if f"{direction}ひじ" in model.bones and f"{SIZING_BONE_PREFIX}{direction}腕IK" in model.bones:
            #     model.bones[f"{direction}ひじ"].parent_index = model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK"].index
            # model.bones[f"{direction}ひじ"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}腕IK"].layer + 1

            # if f"{direction}手捩" in model.bones:
            #     model.bones[f"{direction}手捩"].layer = model.bones[f"{direction}ひじ"].layer

            #     for b in model.bones:
            #         if f"{direction}手捩" in b.name and f"{direction}手捩" != b.name:
            #             b.layer = model.bones[f"{direction}手捩"].layer + 1

            # 手首 -------
            if f"{SIZING_BONE_PREFIX}{direction}ひじIK親" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK親"].parent_index = model.bones[f"{direction}肩根元"].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK親"].layer = (
                #     model.bones[f"{direction}手捩"].layer if f"{direction}手捩" in model.bones else model.bones[f"{direction}ひじ"].layer
                # ) + 1
            if f"{SIZING_BONE_PREFIX}{direction}ひじIK" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK"].parent_index = model.bones[
                    f"{SIZING_BONE_PREFIX}{direction}ひじIK親"
                ].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK親"].layer + 1
            # if f"{direction}手首" in model.bones and f"{SIZING_BONE_PREFIX}{direction}ひじIK" in model.bones:
            #     model.bones[f"{direction}手首"].parent_index = model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK"].index
            # model.bones[f"{direction}手首"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}ひじIK"].layer + 1
            # if f"{SIZING_BONE_PREFIX}{direction}手首先" in model.bones:
            #     model.bones[f"{SIZING_BONE_PREFIX}{direction}手首先"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK"].layer
            # 親指０ -------
            if f"{SIZING_BONE_PREFIX}{direction}手首IK親" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK親"].parent_index = model.bones[f"{direction}肩根元"].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK親"].layer = model.bones[f"{direction}手首"].layer + 1
            if f"{SIZING_BONE_PREFIX}{direction}手首IK" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK"].parent_index = model.bones[
                    f"{SIZING_BONE_PREFIX}{direction}手首IK親"
                ].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK親"].layer + 1
            if f"{direction}親指０" in model.bones:
                model.bones[f"{direction}親指０"].parent_index = model.bones[f"{direction}手首"].index
            if f"{direction}親指１" in model.bones:
                model.bones[f"{direction}親指１"].parent_index = model.bones[f"{direction}親指０"].index
            # model.bones[f"{direction}親指０"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}手首IK"].layer + 1
            # for bname in ("親指１", "親指２"):
            #     if f"{direction}{bname}" in model.bones:
            #         model.bones[f"{direction}{bname}"].layer = (
            #             model.bones[f"{direction}親指０"].layer
            #             if f"{direction}親指０" in model.bones
            #             else model.bones[f"{direction}手首"].layer + 1
            #         )
            # 指 -------
            if f"{SIZING_BONE_PREFIX}{direction}親指０IK親" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}親指０IK親"].parent_index = model.bones[f"{direction}肩根元"].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}親指０IK親"].layer = model.bones[f"{direction}親指０"].layer + 1
            if f"{SIZING_BONE_PREFIX}{direction}親指０IK" in model.bones:
                model.bones[f"{SIZING_BONE_PREFIX}{direction}親指０IK"].parent_index = model.bones[
                    f"{SIZING_BONE_PREFIX}{direction}親指０IK親"
                ].index
                # model.bones[f"{SIZING_BONE_PREFIX}{direction}親指０IK"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}親指０IK親"].layer + 1
            if f"{direction}親指２" in model.bones:
                model.bones[f"{direction}親指２"].parent_index = model.bones[f"{direction}親指１"].index
            # model.bones[f"{direction}親指２"].layer = model.bones[f"{SIZING_BONE_PREFIX}{direction}親指０IK"].layer + 1
            # if model.bones[f"{direction}親指２"].tail_index in model.bones:
            #     model.bones[model.bones[f"{direction}親指２"].tail_index].layer = model.bones[f"{direction}親指２"].layer
            for bname in ("人指１", "中指１", "薬指１", "小指１"):
                if f"{direction}{bname}" in model.bones:
                    model.bones[f"{direction}{bname}"].parent_index = model.bones[f"{direction}手首"].index
            if f"{direction}中指２" in model.bones:
                model.bones[f"{direction}中指２"].parent_index = model.bones[f"{direction}中指１"].index
                # model.bones[f"{direction}{bname}"].layer = model.bones[f"{direction}手首"].layer + 1
                # if model.bones[f"{direction}{bname}"].tail_index in model.bones:
                #     model.bones[model.bones[f"{direction}{bname}"].tail_index].layer = model.bones[f"{direction}{bname}"].layer
            # for bname in ("人指２", "中指２", "薬指２", "小指２", "人指３", "中指３", "薬指３", "小指３"):
            #     if f"{direction}{bname}" in model.bones:
            #         model.bones[f"{direction}{bname}"].layer = model.bones[f"{direction}手首"].layer + 1
            #         if model.bones[f"{direction}{bname}"].tail_index in model.bones:
            #             model.bones[model.bones[f"{direction}{bname}"].tail_index].layer = model.bones[f"{direction}{bname}"].layer

        model.setup()

        # if 10 >= logger.total_level:
        #     # デバッグレベルの場合、モデルを出力する
        #     from mlib.pmx.pmx_writer import PmxWriter

        #     PmxWriter(
        #         model, os.path.join(os.path.dirname(model.path), f"sizing_{os.path.basename(model.path)}"), include_system=True
        #     ).save()
