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


class ArmBoneName:
    @staticmethod
    def shoulder_root(cls, direction: str):
        return ArmBoneName.shoulder_root(direction)

    @staticmethod
    def shoulder_p(cls, direction: str):
        return ArmBoneName.shoulder_p(direction)

    @staticmethod
    def shoulder(cls, direction: str):
        return ArmBoneName.shoulder(direction)

    @staticmethod
    def shoulder_c(cls, direction: str):
        return f"{direction}肩C"

    @staticmethod
    def shoulder_ik_parent(cls, direction: str):
        return ArmBoneName.shoulder_ik_parent(direction)

    @staticmethod
    def shoulder_ik(cls, direction: str):
        return ArmBoneName.shoulder_ik(direction)

    @staticmethod
    def arm(cls, direction: str):
        return ArmBoneName.arm(direction)

    @staticmethod
    def arm_twist(cls, direction: str, index: int = 0):
        return ArmBoneName.arm_twist(direction) if index == 0 else f"{direction}腕捩{index}"

    @staticmethod
    def arm_ik_parent(cls, direction: str):
        return ArmBoneName.arm_ik_parent(direction)

    @staticmethod
    def arm_ik(cls, direction: str):
        return ArmBoneName.arm_ik(direction)

    @staticmethod
    def elbow(cls, direction: str):
        return ArmBoneName.elbow(direction)

    @staticmethod
    def hand_twist(cls, direction: str, index: int = 0):
        return ArmBoneName.hand_twist(direction) if index == 0 else f"{direction}手捩{index}"

    @staticmethod
    def elbow_ik_parent(cls, direction: str):
        return ArmBoneName.elbow_ik_parent(direction)

    @staticmethod
    def elbow_ik(cls, direction: str):
        return ArmBoneName.elbow_ik(direction)

    @staticmethod
    def wrist(cls, direction: str):
        return ArmBoneName.wrist(direction)

    @staticmethod
    def wrist_tail(cls, direction: str):
        return ArmBoneName.wrist_tail(direction)

    @staticmethod
    def wrist_ik_parent(cls, direction: str):
        return ArmBoneName.wrist_ik_parent(direction)

    @staticmethod
    def wrist_ik(cls, direction: str):
        return ArmBoneName.wrist_ik(direction)

    @staticmethod
    def thumb_0(cls, direction: str):
        return ArmBoneName.thumb_0(direction)

    @staticmethod
    def thumb_0_ik_parent(cls, direction: str):
        return ArmBoneName.thumb_0_ik_parent(direction)

    @staticmethod
    def thumb_0_ik(cls, direction: str):
        return ArmBoneName.thumb_0_ik(direction)

    @staticmethod
    def thumb_1(cls, direction: str):
        return ArmBoneName.thumb_1(direction)

    @staticmethod
    def thumb_2(cls, direction: str):
        return ArmBoneName.thumb_2(direction)

    @staticmethod
    def index_1(cls, direction: str):
        return ArmBoneName.index_1(direction)

    @staticmethod
    def index_2(cls, direction: str):
        return ArmBoneName.index_2(direction)

    @staticmethod
    def index_3(cls, direction: str):
        return ArmBoneName.index_3(direction)

    @staticmethod
    def middle_1(cls, direction: str):
        return ArmBoneName.middle_1(direction)

    @staticmethod
    def middle_2(cls, direction: str):
        return ArmBoneName.middle_2(direction)

    @staticmethod
    def middle_3(cls, direction: str):
        return ArmBoneName.middle_3(direction)

    @staticmethod
    def ring_1(cls, direction: str):
        return ArmBoneName.ring_1(direction)

    @staticmethod
    def ring_2(cls, direction: str):
        return ArmBoneName.ring_2(direction)

    @staticmethod
    def ring_3(cls, direction: str):
        return ArmBoneName.ring_3(direction)

    @staticmethod
    def pinky_1(cls, direction: str):
        return ArmBoneName.pinky_1(direction)

    @staticmethod
    def pinky_2(cls, direction: str):
        return ArmBoneName.pinky_2(direction)

    @staticmethod
    def pinky_3(cls, direction: str):
        return ArmBoneName.pinky_3(direction)


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

        shoulder_ratio = dest_model.bones[ArmBoneName.shoulder_root(direction)].position.distance(
            dest_model.bones[ArmBoneName.arm(direction)].position
        ) / src_model.bones[ArmBoneName.shoulder_root(direction)].position.distance(src_model.bones[ArmBoneName.arm(direction)].position)

        logger.debug(f"shoulder_ratio[{shoulder_ratio:.3f}]")

        arm_ratio = dest_model.bones[ArmBoneName.arm(direction)].position.distance(
            dest_model.bones[ArmBoneName.elbow(direction)].position
        ) / src_model.bones[ArmBoneName.arm(direction)].position.distance(src_model.bones[ArmBoneName.elbow(direction)].position)

        logger.debug(f"arm_ratio[{arm_ratio:.3f}]")

        elbow_ratio = dest_model.bones[ArmBoneName.elbow(direction)].position.distance(
            dest_model.bones[ArmBoneName.wrist(direction)].position
        ) / src_model.bones[ArmBoneName.elbow(direction)].position.distance(src_model.bones[ArmBoneName.wrist(direction)].position)

        logger.debug(f"elbow_ratio[{elbow_ratio:.3f}]")

        wrist_ratio = dest_model.bones[ArmBoneName.wrist(direction)].position.distance(
            dest_model.bones[ArmBoneName.wrist_tail(direction)].position
        ) / src_model.bones[ArmBoneName.wrist(direction)].position.distance(src_model.bones[ArmBoneName.wrist_tail(direction)].position)

        logger.debug(f"wrist_ratio[{wrist_ratio:.3f}]")

        if ArmBoneName.thumb_0(direction) in src_model.bones and ArmBoneName.thumb_0(direction) in dest_model.bones:
            thumb0_ratio = dest_model.bones[ArmBoneName.thumb_0(direction)].position.distance(
                dest_model.bones[ArmBoneName.thumb_1(direction)].position
            ) / src_model.bones[ArmBoneName.thumb_0(direction)].position.distance(src_model.bones[ArmBoneName.thumb_1(direction)].position)
        else:
            thumb0_ratio = 1.0

        logger.debug(f"thumb0_ratio[{thumb0_ratio:.3f}]")

        if ArmBoneName.thumb_2(direction) in src_model.bones and ArmBoneName.thumb_2(direction) in dest_model.bones:
            thumb_ratio = (
                dest_model.bones[ArmBoneName.thumb_1(direction)].position.distance(
                    dest_model.bones[ArmBoneName.thumb_2(direction)].position
                )
                + dest_model.bones[ArmBoneName.thumb_2(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.thumb_1(direction)].position.distance(src_model.bones[ArmBoneName.thumb_2(direction)].position)
                + src_model.bones[ArmBoneName.thumb_2(direction)].tail_relative_position.length()
            )
            thumb_src_ratio = (
                src_model.bones[ArmBoneName.middle_1(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_2(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_3(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.thumb_1(direction)].position.distance(src_model.bones[ArmBoneName.thumb_2(direction)].position)
                + src_model.bones[ArmBoneName.thumb_2(direction)].tail_relative_position.length()
            )
        else:
            thumb_ratio = 1.0
            thumb_src_ratio = 1.0

        logger.debug(f"thumb_ratio[{thumb_ratio:.3f}], thumb_src_ratio[{thumb_src_ratio:.3f}]")

        if ArmBoneName.index_3(direction) in src_model.bones and ArmBoneName.index_3(direction) in dest_model.bones:
            index_ratio = (
                dest_model.bones[ArmBoneName.index_1(direction)].position.distance(
                    dest_model.bones[ArmBoneName.index_2(direction)].position
                )
                + dest_model.bones[ArmBoneName.index_2(direction)].position.distance(
                    dest_model.bones[ArmBoneName.index_3(direction)].position
                )
                + dest_model.bones[ArmBoneName.index_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.index_1(direction)].position.distance(src_model.bones[ArmBoneName.index_2(direction)].position)
                + src_model.bones[ArmBoneName.index_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.index_3(direction)].position
                )
                + src_model.bones[ArmBoneName.index_3(direction)].tail_relative_position.length()
            )

            index_src_ratio = (
                src_model.bones[ArmBoneName.middle_1(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_2(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_3(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.index_1(direction)].position.distance(src_model.bones[ArmBoneName.index_2(direction)].position)
                + src_model.bones[ArmBoneName.index_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.index_3(direction)].position
                )
                + src_model.bones[ArmBoneName.index_3(direction)].tail_relative_position.length()
            )
        else:
            index_ratio = 1.0
            index_src_ratio = 1.0

        logger.debug(f"index_ratio[{index_ratio:.3f}], index_src_ratio[{index_src_ratio:.3f}]")

        if ArmBoneName.middle_3(direction) in src_model.bones and ArmBoneName.middle_3(direction) in dest_model.bones:
            middle_ratio = (
                dest_model.bones[ArmBoneName.middle_1(direction)].position.distance(
                    dest_model.bones[ArmBoneName.middle_2(direction)].position
                )
                + dest_model.bones[ArmBoneName.middle_2(direction)].position.distance(
                    dest_model.bones[ArmBoneName.middle_3(direction)].position
                )
                + dest_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.middle_1(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_2(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_3(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position.length()
            )
            middle_src_ratio = 1.0
        else:
            middle_ratio = 1.0
            middle_src_ratio = 1.0

        logger.debug(f"middle_ratio[{middle_ratio:.3f}], middle_src_ratio[{middle_src_ratio:.3f}]")

        if ArmBoneName.ring_3(direction) in src_model.bones and ArmBoneName.ring_3(direction) in dest_model.bones:
            ring_ratio = (
                dest_model.bones[ArmBoneName.ring_1(direction)].position.distance(dest_model.bones[ArmBoneName.ring_2(direction)].position)
                + dest_model.bones[ArmBoneName.ring_2(direction)].position.distance(
                    dest_model.bones[ArmBoneName.ring_3(direction)].position
                )
                + dest_model.bones[ArmBoneName.ring_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.ring_1(direction)].position.distance(src_model.bones[ArmBoneName.ring_2(direction)].position)
                + src_model.bones[ArmBoneName.ring_2(direction)].position.distance(src_model.bones[ArmBoneName.ring_3(direction)].position)
                + src_model.bones[ArmBoneName.ring_3(direction)].tail_relative_position.length()
            )

            ring_src_ratio = (
                src_model.bones[ArmBoneName.middle_1(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_2(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_3(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.ring_1(direction)].position.distance(src_model.bones[ArmBoneName.ring_2(direction)].position)
                + src_model.bones[ArmBoneName.ring_2(direction)].position.distance(src_model.bones[ArmBoneName.ring_3(direction)].position)
                + src_model.bones[ArmBoneName.ring_3(direction)].tail_relative_position.length()
            )
        else:
            ring_ratio = 1.0
            ring_src_ratio = 1.0

        logger.debug(f"ring_ratio[{ring_ratio:.3f}], ring_src_ratio[{ring_src_ratio:.3f}]")

        if ArmBoneName.pinky_3(direction) in src_model.bones and ArmBoneName.pinky_3(direction) in dest_model.bones:
            pinky_ratio = (
                dest_model.bones[ArmBoneName.pinky_1(direction)].position.distance(
                    dest_model.bones[ArmBoneName.pinky_2(direction)].position
                )
                + dest_model.bones[ArmBoneName.pinky_2(direction)].position.distance(
                    dest_model.bones[ArmBoneName.pinky_3(direction)].position
                )
                + dest_model.bones[ArmBoneName.pinky_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.pinky_1(direction)].position.distance(src_model.bones[ArmBoneName.pinky_2(direction)].position)
                + src_model.bones[ArmBoneName.pinky_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.pinky_3(direction)].position
                )
                + src_model.bones[ArmBoneName.pinky_3(direction)].tail_relative_position.length()
            )

            pinky_src_ratio = (
                src_model.bones[ArmBoneName.middle_1(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_2(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.middle_3(direction)].position
                )
                + src_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position.length()
            ) / (
                src_model.bones[ArmBoneName.pinky_1(direction)].position.distance(src_model.bones[ArmBoneName.pinky_2(direction)].position)
                + src_model.bones[ArmBoneName.pinky_2(direction)].position.distance(
                    src_model.bones[ArmBoneName.pinky_3(direction)].position
                )
                + src_model.bones[ArmBoneName.pinky_3(direction)].tail_relative_position.length()
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
                ArmBoneName.shoulder_root(direction),
                ArmBoneName.shoulder(direction),
                ArmBoneName.arm(direction),
                ArmBoneName.elbow(direction),
                ArmBoneName.wrist(direction),
                ArmBoneName.thumb_2(direction),
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
            shoulder_ik_bf = VmdBoneFrame(fno, ArmBoneName.shoulder_ik(direction))
            shoulder_ik_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, ArmBoneName.arm(direction)].position
                - (
                    dest_initial_ik_matrixes[fno, ArmBoneName.shoulder_root(direction)].local_matrix
                    * dest_model.bones[ArmBoneName.arm(direction)].position
                )
            )
            shoulder_ik_bf.register = True
            dest_motion.append_bone_frame(shoulder_ik_bf)

            # 腕IK --------------------
            arm_ik_bf = VmdBoneFrame(fno, ArmBoneName.arm_ik(direction))
            arm_ik_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, ArmBoneName.elbow(direction)].position
                - (
                    dest_initial_ik_matrixes[fno, ArmBoneName.shoulder_root(direction)].local_matrix
                    * dest_model.bones[ArmBoneName.elbow(direction)].position
                )
            )
            arm_ik_bf.register = True
            dest_motion.append_bone_frame(arm_ik_bf)

            # ひじIK --------------------
            elbow_ik_bf = VmdBoneFrame(fno, ArmBoneName.elbow_ik(direction))
            elbow_ik_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, ArmBoneName.wrist(direction)].position
                - (
                    dest_initial_ik_matrixes[fno, ArmBoneName.shoulder_root(direction)].local_matrix
                    * dest_model.bones[ArmBoneName.wrist(direction)].position
                )
            )
            elbow_ik_bf.register = True
            dest_motion.append_bone_frame(elbow_ik_bf)

            # 手首IK --------------------
            wrist_ik_bf = VmdBoneFrame(fno, ArmBoneName.wrist_ik(direction))
            wrist_ik_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_initial_matrixes[fno, ArmBoneName.wrist_tail(direction)].position
                - (
                    dest_initial_ik_matrixes[fno, ArmBoneName.shoulder_root(direction)].local_matrix
                    * dest_model.bones[ArmBoneName.wrist_tail(direction)].position
                )
            )
            wrist_ik_bf.register = True
            dest_motion.append_bone_frame(wrist_ik_bf)

            if ArmBoneName.thumb_0(direction) in dest_model.bones:
                # 親指０IK --------------------
                thumb0_ik_bf = VmdBoneFrame(fno, ArmBoneName.thumb_0_ik(direction))
                thumb0_ik_bf.position = dest_initial_matrixes[
                    fno, ArmBoneName.shoulder_root(direction)
                ].global_matrix.to_quaternion().inverse() * (
                    dest_initial_matrixes[fno, ArmBoneName.thumb_2(direction)].position
                    - (
                        dest_initial_ik_matrixes[fno, ArmBoneName.shoulder_root(direction)].local_matrix
                        * dest_model.bones[ArmBoneName.thumb_2(direction)].position
                    )
                )
                thumb0_ik_bf.register = True
                dest_motion.append_bone_frame(thumb0_ik_bf)

        # 腕位置合わせ ----------------------------
        logger.info("【No.{i}】{d}腕位置合わせ", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】{d}腕位置合わせ", x=sizing_idx + 1, d=__(direction), index=fidx, total_index_count=len(fnos), display_block=1000
            )

            # 肩IK親 --------------------
            src_arm_local_position = (
                src_initial_matrixes[fno, ArmBoneName.shoulder_root(direction)].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, ArmBoneName.arm(direction)].position
            )
            dest_arm_global_position = dest_initial_matrixes[fno, ArmBoneName.shoulder_root(direction)].global_matrix_no_scale * (
                src_arm_local_position * shoulder_ratio
            )

            shoulder_ik_parent_bf = VmdBoneFrame(fno, ArmBoneName.shoulder_ik_parent(direction))
            shoulder_ik_parent_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_arm_global_position - dest_initial_matrixes[fno, ArmBoneName.arm(direction)].position
            )
            shoulder_ik_parent_bf.register = True
            dest_motion.append_bone_frame(shoulder_ik_parent_bf)

            # 腕IK親 --------------------
            src_elbow_local_position = (
                src_initial_matrixes[fno, ArmBoneName.arm(direction)].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, ArmBoneName.elbow(direction)].position
            )
            dest_elbow_global_position = dest_initial_matrixes[fno, ArmBoneName.arm(direction)].global_matrix_no_scale * (
                src_elbow_local_position * arm_ratio
            )

            arm_ik_parent_bf = VmdBoneFrame(fno, ArmBoneName.arm_ik_parent(direction))
            arm_ik_parent_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_elbow_global_position - dest_initial_matrixes[fno, ArmBoneName.elbow(direction)].position
            )
            arm_ik_parent_bf.register = True
            dest_motion.append_bone_frame(arm_ik_parent_bf)

            # ひじIK親 --------------------
            src_wrist_local_position = (
                src_initial_matrixes[fno, ArmBoneName.elbow(direction)].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, ArmBoneName.wrist(direction)].position
            )
            dest_wrist_global_position = dest_initial_matrixes[fno, ArmBoneName.elbow(direction)].global_matrix_no_scale * (
                src_wrist_local_position * elbow_ratio
            )

            elbow_ik_parent_bf = VmdBoneFrame(fno, ArmBoneName.elbow_ik_parent(direction))
            elbow_ik_parent_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_wrist_global_position - dest_initial_matrixes[fno, ArmBoneName.wrist(direction)].position
            )
            elbow_ik_parent_bf.register = True
            dest_motion.append_bone_frame(elbow_ik_parent_bf)

            # 手首IK親 --------------------
            src_wrist_tail_local_position = (
                src_initial_matrixes[fno, ArmBoneName.elbow(direction)].global_matrix_no_scale.inverse()
                * src_initial_matrixes[fno, ArmBoneName.wrist(direction)].position
            )
            dest_wrist_tail_global_position = dest_initial_matrixes[fno, ArmBoneName.elbow(direction)].global_matrix_no_scale * (
                src_wrist_tail_local_position * wrist_ratio
            )

            wrist_ik_parent_bf = VmdBoneFrame(fno, ArmBoneName.wrist_ik_parent(direction))
            wrist_ik_parent_bf.position = dest_initial_matrixes[
                fno, ArmBoneName.shoulder_root(direction)
            ].global_matrix.to_quaternion().inverse() * (
                dest_wrist_tail_global_position - dest_initial_matrixes[fno, ArmBoneName.wrist(direction)].position
            )
            wrist_ik_parent_bf.register = True
            dest_motion.append_bone_frame(wrist_ik_parent_bf)

            if ArmBoneName.thumb_0(direction) in src_model.bones and ArmBoneName.thumb_0(direction) in dest_model.bones:
                # 親指０IK親 --------------------
                src_thumb2_local_position = (
                    src_initial_matrixes[fno, ArmBoneName.thumb_0(direction)].global_matrix_no_scale.inverse()
                    * src_initial_matrixes[fno, ArmBoneName.thumb_2(direction)].position
                )
                dest_thumb2_global_position = dest_initial_matrixes[fno, ArmBoneName.thumb_0(direction)].global_matrix_no_scale * (
                    src_thumb2_local_position * thumb0_ratio
                )

                thumb0_ik_parent_bf = VmdBoneFrame(fno, ArmBoneName.thumb_0_ik_parent(direction))
                thumb0_ik_parent_bf.position = dest_initial_matrixes[
                    fno, ArmBoneName.shoulder_root(direction)
                ].global_matrix.to_quaternion().inverse() * (
                    dest_thumb2_global_position - dest_initial_matrixes[fno, ArmBoneName.thumb_2(direction)].position
                )
                thumb0_ik_parent_bf.register = True
                dest_motion.append_bone_frame(thumb0_ik_parent_bf)

        # if not (
        #     {ArmBoneName.thumb_2(direction), ArmBoneName.index_3(direction), ArmBoneName.middle_3(direction), ArmBoneName.ring_3(direction), ArmBoneName.pinky_3(direction)} - set(src_model.bones.names)
        # ) and not (
        #     {ArmBoneName.thumb_2(direction), ArmBoneName.index_3(direction), ArmBoneName.middle_3(direction), ArmBoneName.ring_3(direction), ArmBoneName.pinky_3(direction)} - set(dest_model.bones.names)
        # ):
        #     # 指位置合わせ ----------------------------
        #     logger.info("【No.{i}】{d}指位置合わせ", i=sizing_idx + 1, d=__(direction), decoration=MLogger.Decoration.LINE)

        #     dest_motion.cache_clear()
        #     dest_finger_matrixes = dest_motion.animate_bone(
        #         finger_fnos,
        #         dest_model,
        #         [
        #             ArmBoneName.wrist(direction),
        #             ArmBoneName.thumb_2(direction),
        #             ArmBoneName.index_3(direction),
        #             ArmBoneName.middle_3(direction),
        #             ArmBoneName.ring_3(direction),
        #             ArmBoneName.pinky_3(direction),
        #             f"{SIZING_BONE_PREFIX}{direction}指IK親",
        #         ],
        #         out_fno_log=True,
        #     )

        #     for fidx, fno in enumerate(finger_fnos):
        #         logger.count(
        #             "【No.{x}】{d}指位置合わせ", x=sizing_idx + 1, d=__(direction), index=fidx, total_index_count=len(finger_fnos), display_block=1000
        #         )

        #         index_src_tail_position = (
        #             src_initial_matrixes[fno, ArmBoneName.index_3(direction)].global_matrix_no_scale * -src_model.bones[ArmBoneName.index_3(direction)].tail_relative_position
        #         )
        #         middle_src_tail_position = (
        #             src_initial_matrixes[fno, ArmBoneName.middle_3(direction)].global_matrix_no_scale * -src_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position
        #         )
        #         ring_src_tail_position = (
        #             src_initial_matrixes[fno, ArmBoneName.ring_3(direction)].global_matrix_no_scale * -src_model.bones[ArmBoneName.ring_3(direction)].tail_relative_position
        #         )
        #         pinky_src_tail_position = (
        #             src_initial_matrixes[fno, ArmBoneName.pinky_3(direction)].global_matrix_no_scale * -src_model.bones[ArmBoneName.pinky_3(direction)].tail_relative_position
        #         )

        #         index_dest_tail_position = (
        #             dest_finger_matrixes[fno, ArmBoneName.index_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[ArmBoneName.index_3(direction)].tail_relative_position
        #         )
        #         middle_dest_tail_position = (
        #             dest_finger_matrixes[fno, ArmBoneName.middle_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[ArmBoneName.middle_3(direction)].tail_relative_position
        #         )
        #         ring_dest_tail_position = (
        #             dest_finger_matrixes[fno, ArmBoneName.ring_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[ArmBoneName.ring_3(direction)].tail_relative_position
        #         )
        #         pinky_dest_tail_position = (
        #             dest_finger_matrixes[fno, ArmBoneName.pinky_3(direction)].global_matrix_no_scale
        #             * -dest_model.bones[ArmBoneName.pinky_3(direction)].tail_relative_position
        #         )

        #         index_src_distance = index_src_tail_position.distance(src_initial_matrixes[fno, ArmBoneName.wrist(direction)].position) / index_src_ratio
        #         middle_src_distance = middle_src_tail_position.distance(src_initial_matrixes[fno, ArmBoneName.wrist(direction)].position) / middle_src_ratio
        #         ring_src_distance = ring_src_tail_position.distance(src_initial_matrixes[fno, ArmBoneName.wrist(direction)].position) / ring_src_ratio
        #         pinky_src_distance = pinky_src_tail_position.distance(src_initial_matrixes[fno, ArmBoneName.wrist(direction)].position) / pinky_src_ratio
        #         # 最も手首から遠い指を探す
        #         far_src_finger_index = np.argmax([index_src_distance, middle_src_distance, ring_src_distance, pinky_src_distance])
        #         far_src_finger_position = [
        #             index_src_tail_position,
        #             middle_src_tail_position,
        #             ring_src_tail_position,
        #             pinky_src_tail_position,
        #         ][far_src_finger_index]
        #         far_src_finger_ratio = [index_ratio, middle_ratio, ring_ratio, pinky_ratio][far_src_finger_index]

        #         src_finger_local_position = src_initial_matrixes[fno, ArmBoneName.wrist(direction)].global_matrix_no_scale.inverse() * far_src_finger_position
        #         dest_finger_global_position = dest_finger_matrixes[fno, ArmBoneName.wrist(direction)].global_matrix_no_scale * (
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
        dest_ik_result_matrixes = dest_motion.animate_bone(
            fnos,
            dest_model,
            [
                ArmBoneName.shoulder_root(direction),
                ArmBoneName.shoulder(direction),
                ArmBoneName.arm(direction),
                ArmBoneName.elbow(direction),
                ArmBoneName.wrist(direction),
                ArmBoneName.thumb_2(direction),
            ],
            clear_ik=True,
            out_fno_log=True,
        )

        # IK回転の焼き込み -------------------
        for bone_name in (
            ArmBoneName.thumb_0(direction),
            ArmBoneName.wrist(direction),
            ArmBoneName.elbow(direction),
            ArmBoneName.arm(direction),
            ArmBoneName.shoulder(direction),
        ):
            if bone_name not in dest_motion.bones:
                continue
            for fno in dest_motion.bones[bone_name].register_indexes:
                dest_motion.bones[bone_name][fno].rotation = (
                    dest_ik_result_matrixes[fno, bone_name].frame_rotation
                    * dest_initial_ik_matrixes[fno, bone_name].frame_rotation.inverse()
                )
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
            | set(motion.bones[ArmBoneName.shoulder(direction)].indexes)
            | set(motion.bones[ArmBoneName.shoulder_p(direction)].indexes)
            | set(motion.bones[ArmBoneName.arm(direction)].indexes)
            | set(motion.bones[ArmBoneName.arm_twist(direction)].indexes)
            | set(motion.bones[ArmBoneName.elbow(direction)].indexes)
            | set(motion.bones[ArmBoneName.hand_twist(direction)].indexes)
            | set(motion.bones[ArmBoneName.wrist(direction)].indexes)
            | set(motion.bones[ArmBoneName.thumb_0(direction)].indexes)
            | set(motion.bones[ArmBoneName.thumb_1(direction)].indexes)
            | set(motion.bones[ArmBoneName.thumb_2(direction)].indexes)
        )

        return (
            sizing_idx,
            is_src,
            direction,
            motion.animate_bone(
                fnos,
                model,
                [
                    ArmBoneName.shoulder_root(direction),
                    ArmBoneName.shoulder(direction),
                    ArmBoneName.arm(direction),
                    ArmBoneName.elbow(direction),
                    ArmBoneName.wrist(direction),
                    ArmBoneName.thumb_2(direction),
                    ArmBoneName.index_3(direction),
                    ArmBoneName.middle_3(direction),
                    ArmBoneName.ring_3(direction),
                    ArmBoneName.pinky_3(direction),
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
            if not (
                {
                    ArmBoneName.shoulder_root(direction),
                    ArmBoneName.shoulder(direction),
                    ArmBoneName.arm(direction),
                    ArmBoneName.wrist(direction),
                }
                - set(model.bones.names)
            ):
                # 手首先追加 ---------------

                wrist_tail_bone = Bone(index=model.bones[ArmBoneName.wrist(direction)].index + 1, name=ArmBoneName.wrist_tail(direction))
                wrist_tail_bone.parent_index = model.bones[ArmBoneName.wrist(direction)].index
                wrist_tail_bone.position = (
                    model.bones[ArmBoneName.wrist(direction)].position
                    + (model.bones[ArmBoneName.wrist(direction)].position - model.bones[ArmBoneName.elbow(direction)].position) / 2
                )
                wrist_tail_bone.is_system = True
                wrist_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(wrist_tail_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_tail_bone.index))

        model.setup()

        for direction in ("左", "右"):
            if ArmBoneName.shoulder_p(direction) in model.bones:
                model.bones[ArmBoneName.shoulder_p(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
            if ArmBoneName.shoulder(direction) in model.bones:
                model.bones[ArmBoneName.shoulder(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
            if ArmBoneName.wrist_tail(direction) in model.bones:
                model.bones[ArmBoneName.wrist_tail(direction)].parent_index = model.bones[ArmBoneName.wrist(direction)].index

    def setup_model_ik(self, sizing_idx: int, is_src: bool, model: PmxModel) -> None:
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
                    ArmBoneName.shoulder_root(direction),
                    ArmBoneName.shoulder(direction),
                    ArmBoneName.arm(direction),
                    ArmBoneName.wrist(direction),
                }
                - set(model.bones.names)
            ):
                # 肩IK親追加 ---------------
                shoulder_ik_parent_bone = Bone(
                    index=model.bones[ArmBoneName.arm(direction)].index, name=ArmBoneName.shoulder_ik_parent(direction)
                )
                shoulder_ik_parent_bone.parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                shoulder_ik_parent_bone.position = model.bones[ArmBoneName.arm(direction)].position.copy()
                shoulder_ik_parent_bone.is_system = True
                shoulder_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(shoulder_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_ik_parent_bone.index))

                # 肩IK追加 ---------------
                shoulder_ik_bone = Bone(index=model.bones[ArmBoneName.arm(direction)].index, name=ArmBoneName.shoulder_ik(direction))
                shoulder_ik_bone.parent_index = shoulder_ik_parent_bone.index
                shoulder_ik_bone.position = model.bones[ArmBoneName.arm(direction)].position.copy()
                shoulder_ik_bone.is_system = True
                shoulder_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                shoulder_ik = Ik()
                shoulder_ik.bone_index = model.bones[ArmBoneName.arm(direction)].index
                shoulder_ik.loop_count = 10
                shoulder_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                shoulder_ik_link_shoulder = IkLink()
                shoulder_ik_link_shoulder.bone_index = model.bones[ArmBoneName.shoulder(direction)].index
                shoulder_ik.links.append(shoulder_ik_link_shoulder)

                shoulder_ik_bone.ik = shoulder_ik
                model.insert_bone(shoulder_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_ik_bone.index))

                # 腕IK親追加 ---------------

                arm_ik_parent_bone = Bone(index=model.bones[ArmBoneName.elbow(direction)].index, name=ArmBoneName.arm_ik_parent(direction))
                arm_ik_parent_bone.parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                arm_ik_parent_bone.position = model.bones[ArmBoneName.elbow(direction)].position.copy()
                arm_ik_parent_bone.is_system = True
                arm_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(arm_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=arm_ik_parent_bone.index))

                # 腕IK追加 ---------------
                arm_ik_bone = Bone(index=model.bones[ArmBoneName.elbow(direction)].index, name=ArmBoneName.arm_ik(direction))
                arm_ik_bone.parent_index = arm_ik_parent_bone.index
                arm_ik_bone.position = model.bones[ArmBoneName.elbow(direction)].position.copy()
                arm_ik_bone.is_system = True
                arm_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                arm_ik = Ik()
                arm_ik.bone_index = model.bones[ArmBoneName.elbow(direction)].index
                arm_ik.loop_count = 10
                arm_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                if ArmBoneName.arm_twist(direction) in model.bones:
                    arm_ik_link_arm_twist = IkLink()
                    arm_ik_link_arm_twist.bone_index = model.bones[ArmBoneName.arm_twist(direction)].index
                    arm_ik_link_arm_twist.angle_limit = True
                    arm_ik.links.append(arm_ik_link_arm_twist)

                    for b in model.bones:
                        if ArmBoneName.arm_twist(direction) in b.name and ArmBoneName.arm_twist(direction) != b.name:
                            b.layer += 1

                arm_ik_link_arm = IkLink()
                arm_ik_link_arm.bone_index = model.bones[ArmBoneName.arm(direction)].index
                arm_ik.links.append(arm_ik_link_arm)

                arm_ik_bone.ik = arm_ik
                model.insert_bone(arm_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=arm_ik_bone.index))

                # ひじIK親追加 ---------------

                elbow_ik_parent_bone = Bone(
                    index=model.bones[ArmBoneName.wrist(direction)].index, name=ArmBoneName.elbow_ik_parent(direction)
                )
                elbow_ik_parent_bone.parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                elbow_ik_parent_bone.position = model.bones[ArmBoneName.wrist(direction)].position.copy()
                elbow_ik_parent_bone.is_system = True
                elbow_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(elbow_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_ik_parent_bone.index))

                # ひじIK追加 ---------------

                elbow_ik_bone = Bone(index=model.bones[ArmBoneName.wrist(direction)].index, name=ArmBoneName.elbow_ik(direction))
                elbow_ik_bone.parent_index = elbow_ik_parent_bone.index
                elbow_ik_bone.position = model.bones[ArmBoneName.wrist(direction)].position.copy()
                elbow_ik_bone.is_system = True
                elbow_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                elbow_ik = Ik()
                elbow_ik.bone_index = model.bones[ArmBoneName.wrist(direction)].index
                elbow_ik.loop_count = 10
                elbow_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                if ArmBoneName.hand_twist(direction) in model.bones:
                    elbow_ik_link_wrist_twist = IkLink()
                    elbow_ik_link_wrist_twist.bone_index = model.bones[ArmBoneName.hand_twist(direction)].index
                    elbow_ik_link_wrist_twist.angle_limit = True
                    elbow_ik.links.append(elbow_ik_link_wrist_twist)

                    for b in model.bones:
                        if ArmBoneName.hand_twist(direction) in b.name and ArmBoneName.hand_twist(direction) != b.name:
                            b.layer += 1

                elbow_ik_link_elbow = IkLink()
                elbow_ik_link_elbow.bone_index = model.bones[ArmBoneName.elbow(direction)].index
                elbow_ik.links.append(elbow_ik_link_elbow)

                elbow_ik_bone.ik = elbow_ik
                model.insert_bone(elbow_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_ik_bone.index))

                # 手首IK親追加 ---------------

                wrist_ik_parent_bone = Bone(
                    index=model.bones[ArmBoneName.wrist_tail(direction)].index, name=ArmBoneName.wrist_ik_parent(direction)
                )
                wrist_ik_parent_bone.parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                wrist_ik_parent_bone.position = model.bones[ArmBoneName.wrist_tail(direction)].position.copy()
                wrist_ik_parent_bone.is_system = True
                wrist_ik_parent_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(wrist_ik_parent_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_ik_parent_bone.index))

                # 手首IK追加 ---------------

                wrist_ik_bone = Bone(index=model.bones[ArmBoneName.wrist_tail(direction)].index, name=ArmBoneName.wrist_ik(direction))
                wrist_ik_bone.parent_index = wrist_ik_parent_bone.index
                wrist_ik_bone.position = model.bones[ArmBoneName.wrist_tail(direction)].position.copy()
                wrist_ik_bone.is_system = True
                wrist_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                wrist_ik = Ik()
                wrist_ik.bone_index = model.bones[ArmBoneName.wrist_tail(direction)].index
                wrist_ik.loop_count = 10
                wrist_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                wrist_ik_link_wrist = IkLink()
                wrist_ik_link_wrist.bone_index = model.bones[ArmBoneName.wrist(direction)].index
                wrist_ik.links.append(wrist_ik_link_wrist)

                wrist_ik_bone.ik = wrist_ik
                model.insert_bone(wrist_ik_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_ik_bone.index))

                if ArmBoneName.thumb_0(direction) in model.bones:
                    # 親指０IK親 追加 ---------------
                    thumb0_ik_parent_bone = Bone(
                        index=model.bones[ArmBoneName.thumb_2(direction)].index, name=ArmBoneName.thumb_0_ik_parent(direction)
                    )
                    thumb0_ik_parent_bone.parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                    thumb0_ik_parent_bone.position = model.bones[ArmBoneName.thumb_2(direction)].position.copy()
                    thumb0_ik_parent_bone.is_system = True
                    thumb0_ik_parent_bone.bone_flg |= (
                        BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                    )

                    model.insert_bone(thumb0_ik_parent_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb0_ik_parent_bone.index))

                    # 親指０IK追加 ---------------
                    thumb0_ik_bone = Bone(index=model.bones[ArmBoneName.thumb_2(direction)].index, name=ArmBoneName.thumb_0_ik(direction))
                    thumb0_ik_bone.parent_index = thumb0_ik_parent_bone.index
                    thumb0_ik_bone.position = model.bones[ArmBoneName.thumb_2(direction)].position.copy()
                    thumb0_ik_bone.is_system = True
                    thumb0_ik_bone.bone_flg |= (
                        BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                    )

                    thumb0_ik = Ik()
                    thumb0_ik.bone_index = model.bones[ArmBoneName.thumb_2(direction)].index
                    thumb0_ik.loop_count = 10
                    thumb0_ik.unit_rotation.radians = MVector3D(1, 0, 0)

                    thumb0_ik_link_thumb1 = IkLink()
                    thumb0_ik_link_thumb1.bone_index = model.bones[ArmBoneName.thumb_1(direction)].index
                    thumb0_ik_link_thumb1.angle_limit = True
                    thumb0_ik.links.append(thumb0_ik_link_thumb1)

                    thumb0_ik_link_thumb0 = IkLink()
                    thumb0_ik_link_thumb0.bone_index = model.bones[ArmBoneName.thumb_0(direction)].index
                    thumb0_ik.links.append(thumb0_ik_link_thumb0)

                    thumb0_ik_bone.ik = thumb0_ik
                    model.insert_bone(thumb0_ik_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb0_ik_bone.index))

        for direction in ("左", "右"):
            # 肩 -------
            if ArmBoneName.shoulder_p(direction) in model.bones:
                model.bones[ArmBoneName.shoulder_p(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
            if ArmBoneName.shoulder(direction) in model.bones:
                model.bones[ArmBoneName.shoulder(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
            # 腕 -------
            if ArmBoneName.shoulder_ik_parent(direction) in model.bones:
                model.bones[ArmBoneName.shoulder_ik_parent(direction)].parent_index = model.bones[
                    ArmBoneName.shoulder_root(direction)
                ].index
            if ArmBoneName.shoulder_ik(direction) in model.bones:
                model.bones[ArmBoneName.shoulder_ik(direction)].parent_index = model.bones[ArmBoneName.shoulder_ik_parent(direction)].index
                # model.bones[ArmBoneName.shoulder_ik(direction)].layer = model.bones[ArmBoneName.shoulder_ik_parent(direction)].layer + 1
            if ArmBoneName.arm(direction) in model.bones and ArmBoneName.shoulder_ik(direction) in model.bones:
                model.bones[ArmBoneName.arm(direction)].parent_index = model.bones[ArmBoneName.shoulder_ik(direction)].index
            # model.bones[ArmBoneName.arm(direction)].layer = model.bones[ArmBoneName.shoulder_ik(direction)].layer + 1
            # if ArmBoneName.arm_twist(direction) in model.bones:
            #     model.bones[ArmBoneName.arm_twist(direction)].layer = model.bones[ArmBoneName.arm(direction)].layer

            #     for b in model.bones:
            #         if ArmBoneName.arm_twist(direction) in b.name and ArmBoneName.arm_twist(direction) != b.name:
            #             b.layer = model.bones[ArmBoneName.arm_twist(direction)].layer + 1

            # ひじ -------
            if ArmBoneName.arm_ik_parent(direction) in model.bones:
                model.bones[ArmBoneName.arm_ik_parent(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                # model.bones[ArmBoneName.arm_ik_parent(direction)].layer = (
                #     model.bones[ArmBoneName.arm_twist(direction)].layer if ArmBoneName.arm_twist(direction) in model.bones else model.bones[ArmBoneName.arm(direction)].layer
                # ) + 1
            if ArmBoneName.arm_ik(direction) in model.bones:
                model.bones[ArmBoneName.arm_ik(direction)].parent_index = model.bones[ArmBoneName.arm_ik_parent(direction)].index
                # model.bones[ArmBoneName.arm_ik(direction)].layer = model.bones[ArmBoneName.arm_ik_parent(direction)].layer + 1
            if ArmBoneName.elbow(direction) in model.bones and ArmBoneName.arm_ik(direction) in model.bones:
                model.bones[ArmBoneName.elbow(direction)].parent_index = model.bones[ArmBoneName.arm_ik(direction)].index
            # model.bones[ArmBoneName.elbow(direction)].layer = model.bones[ArmBoneName.arm_ik(direction)].layer + 1

            # if ArmBoneName.hand_twist(direction) in model.bones:
            #     model.bones[ArmBoneName.hand_twist(direction)].layer = model.bones[ArmBoneName.elbow(direction)].layer

            #     for b in model.bones:
            #         if ArmBoneName.hand_twist(direction) in b.name and ArmBoneName.hand_twist(direction) != b.name:
            #             b.layer = model.bones[ArmBoneName.hand_twist(direction)].layer + 1

            # 手首 -------
            if ArmBoneName.elbow_ik_parent(direction) in model.bones:
                model.bones[ArmBoneName.elbow_ik_parent(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                # model.bones[ArmBoneName.elbow_ik_parent(direction)].layer = (
                #     model.bones[ArmBoneName.hand_twist(direction)].layer if ArmBoneName.hand_twist(direction) in model.bones else model.bones[ArmBoneName.elbow(direction)].layer
                # ) + 1
            if ArmBoneName.elbow_ik(direction) in model.bones:
                model.bones[ArmBoneName.elbow_ik(direction)].parent_index = model.bones[ArmBoneName.elbow_ik_parent(direction)].index
                # model.bones[ArmBoneName.elbow_ik(direction)].layer = model.bones[ArmBoneName.elbow_ik_parent(direction)].layer + 1
            if ArmBoneName.wrist(direction) in model.bones and ArmBoneName.elbow_ik(direction) in model.bones:
                model.bones[ArmBoneName.wrist(direction)].parent_index = model.bones[ArmBoneName.elbow_ik(direction)].index
            # model.bones[ArmBoneName.wrist(direction)].layer = model.bones[ArmBoneName.elbow_ik(direction)].layer + 1
            # if ArmBoneName.wrist_tail(direction) in model.bones:
            #     model.bones[ArmBoneName.wrist_tail(direction)].layer = model.bones[ArmBoneName.wrist_ik(direction)].layer
            # 親指０ -------
            if ArmBoneName.wrist_ik_parent(direction) in model.bones:
                model.bones[ArmBoneName.wrist_ik_parent(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                # model.bones[ArmBoneName.wrist_ik_parent(direction)].layer = model.bones[ArmBoneName.wrist(direction)].layer + 1
            if ArmBoneName.wrist_ik(direction) in model.bones:
                model.bones[ArmBoneName.wrist_ik(direction)].parent_index = model.bones[ArmBoneName.wrist_ik_parent(direction)].index
                # model.bones[ArmBoneName.wrist_ik(direction)].layer = model.bones[ArmBoneName.wrist_ik_parent(direction)].layer + 1
            if ArmBoneName.thumb_0(direction) in model.bones:
                model.bones[ArmBoneName.thumb_0(direction)].parent_index = model.bones[ArmBoneName.wrist_ik(direction)].index
            if ArmBoneName.thumb_1(direction) in model.bones:
                model.bones[ArmBoneName.thumb_1(direction)].parent_index = model.bones[ArmBoneName.thumb_0(direction)].index
            # model.bones[ArmBoneName.thumb_0(direction)].layer = model.bones[ArmBoneName.wrist_ik(direction)].layer + 1
            # for bname in ("親指１", "親指２"):
            #     if f"{direction}{bname}" in model.bones:
            #         model.bones[f"{direction}{bname}"].layer = (
            #             model.bones[ArmBoneName.thumb_0(direction)].layer
            #             if ArmBoneName.thumb_0(direction) in model.bones
            #             else model.bones[ArmBoneName.wrist(direction)].layer + 1
            #         )
            # 指 -------
            if ArmBoneName.thumb_0_ik_parent(direction) in model.bones:
                model.bones[ArmBoneName.thumb_0_ik_parent(direction)].parent_index = model.bones[ArmBoneName.shoulder_root(direction)].index
                # model.bones[ArmBoneName.thumb_0_ik_parent(direction)].layer = model.bones[ArmBoneName.thumb_0(direction)].layer + 1
            if ArmBoneName.thumb_0_ik(direction) in model.bones:
                model.bones[ArmBoneName.thumb_0_ik(direction)].parent_index = model.bones[ArmBoneName.thumb_0_ik_parent(direction)].index
                # model.bones[ArmBoneName.thumb_0_ik(direction)].layer = model.bones[ArmBoneName.thumb_0_ik_parent(direction)].layer + 1
            if ArmBoneName.thumb_2(direction) in model.bones:
                model.bones[ArmBoneName.thumb_2(direction)].parent_index = model.bones[ArmBoneName.thumb_1(direction)].index
            # model.bones[ArmBoneName.thumb_2(direction)].layer = model.bones[ArmBoneName.thumb_0_ik(direction)].layer + 1
            # if model.bones[ArmBoneName.thumb_2(direction)].tail_index in model.bones:
            #     model.bones[model.bones[ArmBoneName.thumb_2(direction)].tail_index].layer = model.bones[ArmBoneName.thumb_2(direction)].layer
            for bname in ("人指１", "中指１", "薬指１", "小指１"):
                if f"{direction}{bname}" in model.bones:
                    model.bones[f"{direction}{bname}"].parent_index = model.bones[ArmBoneName.wrist_ik(direction)].index
            if ArmBoneName.middle_2(direction) in model.bones:
                model.bones[ArmBoneName.middle_2(direction)].parent_index = model.bones[ArmBoneName.middle_1(direction)].index
                # model.bones[f"{direction}{bname}"].layer = model.bones[ArmBoneName.wrist(direction)].layer + 1
                # if model.bones[f"{direction}{bname}"].tail_index in model.bones:
                #     model.bones[model.bones[f"{direction}{bname}"].tail_index].layer = model.bones[f"{direction}{bname}"].layer
            # for bname in ("人指２", "中指２", "薬指２", "小指２", "人指３", "中指３", "薬指３", "小指３"):
            #     if f"{direction}{bname}" in model.bones:
            #         model.bones[f"{direction}{bname}"].layer = model.bones[ArmBoneName.wrist(direction)].layer + 1
            #         if model.bones[f"{direction}{bname}"].tail_index in model.bones:
            #             model.bones[model.bones[f"{direction}{bname}"].tail_index].layer = model.bones[f"{direction}{bname}"].layer

        model.setup()

        # if 10 >= logger.total_level:
        #     # デバッグレベルの場合、モデルを出力する
        #     from mlib.pmx.pmx_writer import PmxWriter

        #     PmxWriter(
        #         model, os.path.join(os.path.dirname(model.path), f"sizing_{os.path.basename(model.path)}"), include_system=True
        #     ).save()
