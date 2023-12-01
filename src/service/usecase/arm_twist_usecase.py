import os
from typing import Optional

from service.usecase.bone_names import BoneNames

from mlib.core.logger import MLogger
from mlib.core.math import MQuaternion, MVector3D
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_part import VmdBoneFrame
from mlib.vmd.vmd_tree import VmdBoneFrameTrees

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text

ARM_BONE_NAMES = [
    BoneNames.arm("右"),
    BoneNames.arm_twist("右"),
    BoneNames.elbow("右"),
    BoneNames.wrist_twist("右"),
    BoneNames.wrist("右"),
    BoneNames.arm("左"),
    BoneNames.arm_twist("左"),
    BoneNames.elbow("左"),
    BoneNames.wrist_twist("左"),
    BoneNames.wrist("左"),
]


class ArmTwistUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        is_twist: bool,
        show_message: bool = False,
    ) -> bool:
        if not (src_model and dest_model) or (not is_twist):
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        """腕系位置合わせ"""
        if set(ARM_BONE_NAMES) - set(src_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{i}】モーション作成元モデルに肩・腕捩・腕・ひじ・手捩・手首の左右ボーンがないため、捩り分散をスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if set(ARM_BONE_NAMES) - set(dest_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{i}】サイジング先モデルに肩・腕捩・腕・ひじ・手捩・手首の左右ボーンがないため、捩り分散をスキップします",
                    i=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_arm_twist(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        src_motion: VmdMotion,
        dest_motion: VmdMotion,
        src_initial_matrixes: VmdBoneFrameTrees,
        dest_initial_matrixes: VmdBoneFrameTrees,
        direction: str,
    ) -> tuple[int, VmdMotion]:
        # プラス方向に捩った場合
        (
            arm_twist_plus_degree,
            arm_twist_plus_cross_vector,
            wrist_twist_plus_degree,
            wrist_twist_plus_cross_vector,
        ) = self.get_initial_twist_degree(dest_model, direction, 1)

        # マイナス方向に捩った場合
        (
            arm_twist_minus_degree,
            arm_twist_minus_cross_vector,
            wrist_twist_minus_degree,
            wrist_twist_minus_cross_vector,
        ) = self.get_initial_twist_degree(dest_model, direction, -1)

        # 腕捩
        self.separate_twist(
            sizing_idx,
            dest_model,
            dest_motion,
            dest_initial_matrixes,
            BoneNames.shoulder(direction),
            BoneNames.arm(direction),
            BoneNames.arm_twist(direction),
            BoneNames.elbow_rotate(direction),
            arm_twist_plus_degree,
            arm_twist_plus_cross_vector,
            arm_twist_minus_degree,
            arm_twist_minus_cross_vector,
        )
        # 手捩
        self.separate_twist(
            sizing_idx,
            dest_model,
            dest_motion,
            dest_initial_matrixes,
            BoneNames.arm(direction),
            BoneNames.elbow(direction),
            BoneNames.wrist_twist(direction),
            BoneNames.wrist_rotate(direction),
            wrist_twist_plus_degree,
            wrist_twist_plus_cross_vector,
            wrist_twist_minus_degree,
            wrist_twist_minus_cross_vector,
        )

        return sizing_idx, dest_motion

    def get_initial_twist_degree(
        self,
        dest_model: PmxModel,
        direction: str,
        initial_degree: float,
    ) -> tuple[float, MVector3D, float, MVector3D]:
        # 腕捩りの回転量
        arm_twist_degree, arm_twist_cross_vector = self.get_initial_twist_degree_inner(
            dest_model,
            initial_degree,
            dest_model.bones[BoneNames.arm_twist(direction)],
            dest_model.bones[BoneNames.elbow_rotate(direction)],
        )

        # 手捩りの回転量
        (
            wrist_twist_degree,
            wrist_twist_cross_vector,
        ) = self.get_initial_twist_degree_inner(
            dest_model,
            initial_degree,
            dest_model.bones[BoneNames.wrist_twist(direction)],
            dest_model.bones[BoneNames.wrist_rotate(direction)],
        )

        return (
            arm_twist_degree,
            arm_twist_cross_vector,
            wrist_twist_degree,
            wrist_twist_cross_vector,
        )

    def get_initial_twist_degree_inner(
        self,
        dest_model: PmxModel,
        initial_degree: float,
        twist_bone: Bone,
        rotate_bone: Bone,
    ) -> tuple[float, MVector3D, float, MVector3D]:
        initial_twist_motion = VmdMotion()
        twist_bf = VmdBoneFrame(0, twist_bone.name)
        twist_bf.rotation = MQuaternion.from_axis_angles(
            twist_bone.corrected_fixed_axis, initial_degree
        )
        initial_twist_motion.append_bone_frame(twist_bf)

        initial_twist_matrixes = initial_twist_motion.animate_bone(
            [0],
            dest_model,
            (
                twist_bone.name,
                rotate_bone.name,
            ),
        )

        # 1度捩った時の腕捩りの相対位置
        local_twist_default_position = rotate_bone.position - twist_bone.position
        local_twist_initial_twist_position = (
            initial_twist_matrixes[rotate_bone.name, 0].position
            - initial_twist_matrixes[twist_bone.name, 0].position
        )

        # 1度捩った時の腕捩りの回転角度
        normalized_local_twist_default_position = (
            local_twist_default_position.normalized()
        )
        normalized_local_twist_initial_twist_position = (
            local_twist_initial_twist_position.normalized()
        )
        twist_cross_vector = normalized_local_twist_initial_twist_position.cross(
            normalized_local_twist_default_position
        ).normalized()
        twist_rotation_dot = normalized_local_twist_initial_twist_position.dot(
            normalized_local_twist_default_position
        )
        twist_degree = MQuaternion.scalar_to_degrees(twist_rotation_dot)

        return twist_degree, twist_cross_vector

    def separate_twist(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        dest_initial_matrixes: VmdBoneFrameTrees,
        begin_bone_name: str,
        above_bone_name: str,
        twist_bone_name: str,
        rotate_bone_name: str,
        twist_plus_degree: float,
        twist_plus_cross_vector: MVector3D,
        twist_minus_degree: float,
        twist_minus_cross_vector: MVector3D,
    ) -> None:
        if not (
            twist_bone_name in dest_model.bones and above_bone_name in dest_model.bones
        ):
            return

        logger.info(
            "【No.{i}】{b}:捩り分散",
            i=sizing_idx + 1,
            b=twist_bone_name,
            decoration=MLogger.Decoration.LINE,
        )

        fnos = sorted(
            set(dest_motion.bones[twist_bone_name].register_indexes)
            | set(dest_motion.bones[above_bone_name].register_indexes)
        )
        above_local_x_axis = dest_model.bones[above_bone_name].corrected_local_x_vector
        above_local_y_axis = dest_model.bones[above_bone_name].corrected_local_y_vector
        twist_fixed_axis = dest_model.bones[twist_bone_name].corrected_fixed_axis

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】{b}:捩り分散",
                x=sizing_idx + 1,
                b=twist_bone_name,
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            above_bf = dest_motion.bones[above_bone_name][fno]
            twist_bf = dest_motion.bones[twist_bone_name][fno]

            _, _, _, above_yz_qq = above_bf.rotation.separate_by_axis(
                above_local_x_axis
            )

            if "ひじ" in above_bone_name:
                above_bf.rotation = MQuaternion.from_axis_angles(
                    above_local_y_axis, above_yz_qq.to_degrees()
                )
            else:
                # 腕はYZをそのまま入れる
                above_bf.rotation = above_yz_qq
            above_bf.register = True
            dest_motion.insert_bone_frame(above_bf)

            # IK計算と同じように「どのくらい回転すれば元の位置に近付くか」を求める

            mat = dest_initial_matrixes[fno, begin_bone_name].global_matrix.copy()
            # 肩-腕、腕-ひじ、のベクトル
            mat.translate(
                dest_model.bones[above_bone_name].position
                - dest_model.bones[begin_bone_name].position
            )
            # 捩りを除去した捩りの上にあるボーンの回転量
            mat.rotate(above_bf.rotation)
            # # 腕-ひじ、ひじ-手首、のベクトル
            # mat.translate(dest_model.bones[twist_bone_name].position - dest_model.bones[above_bone_name].position)

            # 捩りを除去した場合のひじ回転もしくは手首回転
            twist_off_above_tail_position = mat * (
                dest_model.bones[rotate_bone_name].position
                - dest_model.bones[above_bone_name].position
            )

            # モーションをそのまま読み込んだ場合のひじ回転もしくは手首回転の位置
            initial_above_tail_position = dest_initial_matrixes[
                fno, rotate_bone_name
            ].position

            # 捩り除去時のローカル座標
            inv_mat = mat.inverse()
            local_twist_off_above_tail_position = (
                inv_mat * twist_off_above_tail_position
            )
            local_initial_above_tail_position = inv_mat * initial_above_tail_position

            normalized_local_twist_off_above_tail_position = (
                local_twist_off_above_tail_position.normalized()
            )
            normalized_local_initial_above_tail_position = (
                local_initial_above_tail_position.normalized()
            )

            # 回転角度
            rotation_vector = normalized_local_initial_above_tail_position.cross(
                normalized_local_twist_off_above_tail_position
            ).normalized()
            rotation_dot = normalized_local_initial_above_tail_position.dot(
                normalized_local_twist_off_above_tail_position
            )
            rotation_degree = MQuaternion.vector_to_degrees(
                normalized_local_twist_off_above_tail_position,
                normalized_local_initial_above_tail_position,
            )
            if rotation_vector.dot(twist_minus_cross_vector) > rotation_vector.dot(
                twist_plus_cross_vector
            ):
                twist_rotation_degree = rotation_degree / -twist_minus_degree
            else:
                twist_rotation_degree = rotation_degree / twist_plus_degree
            if "腕" in above_bone_name:
                twist_rotation_degree *= 2 * (1 if "右" in twist_bone_name else -1)

            logger.debug(
                f"[{twist_bone_name}][{fno}][rotation_vector={rotation_vector}][rotation_dot={rotation_dot:.2f}]"
                + f"[rotation_degree={rotation_degree:.2f}][twist_rotation_degree={twist_rotation_degree:.2f}]"
                + f"[twist_plus_cross_vector={twist_plus_cross_vector}({rotation_vector.dot(twist_plus_cross_vector):.2f})]"
                + f"[twist_minus_cross_vector={twist_minus_cross_vector}({rotation_vector.dot(twist_minus_cross_vector):.2f})]"
            )

            twist_bf.rotation = MQuaternion.from_axis_angles(
                twist_fixed_axis, twist_rotation_degree
            )
            twist_bf.register = True
            dest_motion.insert_bone_frame(twist_bf)

    def get_initial_matrixes(
        self,
        sizing_idx: int,
        is_src: bool,
        model: PmxModel,
        motion: VmdMotion,
        direction: str,
    ) -> tuple[int, bool, str, VmdBoneFrameTrees]:
        model_type = __("作成元モデル" if is_src else "サイジング先モデル")

        logger.info(
            "【No.{x}】{d}腕：初期位置取得({m})",
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
            | set(motion.bones[BoneNames.wrist_twist(direction)].indexes)
            | set(motion.bones[BoneNames.wrist(direction)].indexes)
            | set(motion.bones[BoneNames.thumb0(direction)].indexes)
            | set(motion.bones[BoneNames.thumb1(direction)].indexes)
            | set(motion.bones[BoneNames.thumb2(direction)].indexes)
            | set(motion.bones[BoneNames.index1(direction)].indexes)
            | set(motion.bones[BoneNames.index2(direction)].indexes)
            | set(motion.bones[BoneNames.index3(direction)].indexes)
            | set(motion.bones[BoneNames.middle1(direction)].indexes)
            | set(motion.bones[BoneNames.middle2(direction)].indexes)
            | set(motion.bones[BoneNames.middle3(direction)].indexes)
            | set(motion.bones[BoneNames.ring1(direction)].indexes)
            | set(motion.bones[BoneNames.ring2(direction)].indexes)
            | set(motion.bones[BoneNames.ring3(direction)].indexes)
            | set(motion.bones[BoneNames.pinky1(direction)].indexes)
            | set(motion.bones[BoneNames.pinky2(direction)].indexes)
            | set(motion.bones[BoneNames.pinky3(direction)].indexes)
        )

        initial_matrixes = motion.animate_bone(
            fnos,
            model,
            (
                BoneNames.shoulder_root(direction),
                BoneNames.shoulder(direction),
                BoneNames.shoulder_center(direction),
                BoneNames.arm(direction),
                BoneNames.arm_twist(direction),
                BoneNames.elbow(direction),
                BoneNames.elbow_center(direction),
                BoneNames.elbow_rotate(direction),
                BoneNames.wrist(direction),
                BoneNames.wrist_twist(direction),
                BoneNames.wrist_tail(direction),
                BoneNames.wrist_rotate(direction),
                BoneNames.thumb_tail(direction),
                BoneNames.index_tail(direction),
                BoneNames.middle_tail(direction),
                BoneNames.ring_tail(direction),
                BoneNames.pinky_tail(direction),
            ),
            clear_ik=True,
            out_fno_log=True,
            description=f"{sizing_idx + 1}|{__(direction)}|{__('初期位置取得')}|{model_type}",
        )

        return (
            sizing_idx,
            is_src,
            direction,
            initial_matrixes,
        )
