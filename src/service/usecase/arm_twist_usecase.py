import os
from math import radians
from typing import Optional

import numpy as np
from mlib.core.logger import MLogger
from mlib.core.math import MQuaternion, MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlotReference, Ik, IkLink
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.usecase.bone_names import BoneNames

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

# 逆ひじを許可する角度
ELBOW_REVERSE_Y_RAD = radians(5)


class ArmTwistUsecase:
    def validate(
        self,
        sizing_idx: int,
        dest_model: Optional[PmxModel],
        is_twist: bool,
        show_message: bool = False,
    ) -> bool:
        if not dest_model or (not is_twist):
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        """捩り分散"""
        if set(ARM_BONE_NAMES) - set(dest_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{x}】サイジング先モデルに肩・腕捩・腕・ひじ・手捩・手首の左右ボーンがないため、捩り分散をスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_arm_twist(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        initial_matrixes: dict[tuple[int, bool, str], VmdBoneFrameTrees],
    ) -> tuple[int, VmdMotion]:
        for direction in ("右", "左"):
            self.sizing_arm_twist_direction(
                sizing_idx,
                dest_model,
                dest_motion,
                initial_matrixes[(sizing_idx, False, direction)],
                direction,
            )

        return sizing_idx, dest_motion

    def sizing_arm_twist_direction(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        dest_initial_matrixes: VmdBoneFrameTrees,
        direction: str,
    ) -> tuple[int, str, VmdMotion]:
        logger.info(
            "【No.{i}】【{d}】捩り分散",
            i=sizing_idx + 1,
            d=__(direction),
            decoration=MLogger.Decoration.LINE,
        )

        fnos = sorted(
            {0}
            | set(dest_motion.bones[BoneNames.arm(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.arm_twist(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.elbow(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.wrist_twist(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.wrist(direction)].register_indexes)
        )

        elbow_y_axis = dest_model.bones[
            BoneNames.elbow(direction)
        ].corrected_local_y_vector

        arm_twist_fixed_axis = dest_model.bones[
            BoneNames.arm_twist(direction)
        ].corrected_fixed_axis

        wrist_twist_fixed_axis = dest_model.bones[
            BoneNames.wrist_twist(direction)
        ].corrected_fixed_axis

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】捩り分散 - ひじ登録",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            # --------------

            # キーフレを登録しておく
            arm_bf = dest_motion.bones[BoneNames.arm(direction)][fno]
            arm_bf.register = True
            dest_motion.insert_bone_frame(arm_bf)

            arm_twist_bf = dest_motion.bones[BoneNames.arm_twist(direction)][fno]
            arm_twist_bf.register = True
            dest_motion.insert_bone_frame(arm_twist_bf)

            elbow_bf = dest_motion.bones[BoneNames.elbow(direction)][fno]
            elbow_bf.register = True
            dest_motion.insert_bone_frame(elbow_bf)

            wrist_twist_bf = dest_motion.bones[BoneNames.wrist_twist(direction)][fno]
            wrist_twist_bf.register = True
            dest_motion.insert_bone_frame(wrist_twist_bf)

            wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
            wrist_bf.register = True
            dest_motion.insert_bone_frame(wrist_bf)

            arm_direction_ik_bf = dest_motion.bones[
                BoneNames.arm_direction_ik(direction)
            ][fno]
            arm_direction_ik_bf.register = True
            dest_motion.insert_bone_frame(arm_direction_ik_bf)

            arm_rotate_ik_bf = dest_motion.bones[BoneNames.arm_rotate_ik(direction)][
                fno
            ]
            arm_rotate_ik_bf.register = True
            dest_motion.insert_bone_frame(arm_rotate_ik_bf)

            elbow_direction_ik_bf = dest_motion.bones[
                BoneNames.elbow_direction_ik(direction)
            ][fno]
            elbow_direction_ik_bf.register = True
            dest_motion.insert_bone_frame(elbow_direction_ik_bf)

            elbow_rotate_ik_bf = dest_motion.bones[
                BoneNames.elbow_rotate_ik(direction)
            ][fno]
            elbow_rotate_ik_bf.register = True
            dest_motion.insert_bone_frame(elbow_rotate_ik_bf)

            wrist_direction_ik_bf = dest_motion.bones[
                BoneNames.wrist_direction_ik(direction)
            ][fno]
            wrist_direction_ik_bf.register = True
            dest_motion.insert_bone_frame(wrist_direction_ik_bf)

            wrist_rotate_ik_bf = dest_motion.bones[
                BoneNames.wrist_rotate_ik(direction)
            ][fno]
            wrist_rotate_ik_bf.register = True
            dest_motion.insert_bone_frame(wrist_rotate_ik_bf)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】捩り分散",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(fnos),
                display_block=50,
            )

            # --------------

            # 腕方向(ひじ位置)
            arm_direction_ik_bf = dest_motion.bones[
                BoneNames.arm_direction_ik(direction)
            ][fno]
            arm_direction_ik_bf.position = (
                dest_initial_matrixes[
                    BoneNames.arm_direction_ik(direction), fno
                ].global_matrix.inverse()
                * dest_initial_matrixes[BoneNames.elbow(direction), fno].position
            )
            dest_motion.insert_bone_frame(arm_direction_ik_bf)

            # ■ --------------
            from datetime import datetime

            from mlib.vmd.vmd_writer import VmdWriter

            VmdWriter(
                dest_motion,
                f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}腕方向_{fno:04d}.vmd",
                model_name="Test Model",
            ).save()
            # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.arm_direction_ik(direction)],
            )

            # ■ --------------
            arm_direction_bf = dest_motion.bones[BoneNames.arm_direction(direction)][
                fno
            ]
            arm_direction_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.arm_direction(direction)].index
            ]
            arm_direction_bf.register = True
            dest_motion.insert_bone_frame(arm_direction_bf)
            # ■ --------------

            # 腕方向 解決結果を設定
            arm_bf = dest_motion.bones[BoneNames.arm(direction)][fno]
            arm_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.arm_direction(direction)].index
            ]
            dest_motion.insert_bone_frame(arm_bf)

            # 腕方向 結果解決後の行列取得
            after_matrixes = dest_motion.animate_bone(
                [fno],
                dest_model,
                [
                    BoneNames.elbow(direction),
                    BoneNames.arm_rotate_ik(direction),
                ],
            )

            # 腕回転(ひじから垂直に出たIKターゲット位置)
            arm_rotate_ik_bf = dest_motion.bones[BoneNames.arm_rotate_ik(direction)][
                fno
            ]
            # 腕捩垂線が最終的に初期値になるよう、配置
            arm_rotate_ik_bf.position = after_matrixes[
                BoneNames.arm_rotate_ik(direction), fno
            ].global_matrix.inverse() * (
                after_matrixes[BoneNames.arm_twist(direction), fno].position
                + (
                    dest_initial_matrixes[
                        BoneNames.arm_twist_vertical(direction), fno
                    ].position
                    - dest_initial_matrixes[
                        BoneNames.arm_twist(direction), fno
                    ].position
                )
            )
            dest_motion.insert_bone_frame(arm_rotate_ik_bf)

            # ■ --------------
            VmdWriter(
                dest_motion,
                f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}腕回転_{fno:04d}.vmd",
                model_name="Test Model",
            ).save()
            # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.arm_rotate_ik(direction)],
                ik_qqs,
            )

            # ■ --------------
            arm_rotate_bf = dest_motion.bones[BoneNames.arm_rotate(direction)][fno]
            arm_rotate_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.arm_rotate(direction)].index
            ]
            arm_rotate_bf.register = True
            dest_motion.insert_bone_frame(arm_rotate_bf)
            # ■ --------------

            # 腕捩 解決結果を設定
            arm_twist_bf = dest_motion.bones[BoneNames.arm_twist(direction)][fno]
            arm_twist_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.arm_rotate(direction)].index
            ]
            dest_motion.insert_bone_frame(arm_twist_bf)

            # ひじの捩りを除いた角度を取得
            _, _, _, elbow_yz_qq = dest_initial_matrixes[
                BoneNames.elbow(direction), fno
            ].frame_fk_rotation.separate_by_axis(arm_twist_fixed_axis)

            # ひじのZは殺してY回転だけにして登録
            elbow_yz_axis = elbow_yz_qq.to_axis()
            elbow_yz_rad = elbow_yz_qq.to_radian()
            elbow_yz_sign = np.sign(elbow_y_axis.dot(elbow_yz_axis))
            if elbow_yz_sign < 0 and elbow_yz_rad > ELBOW_REVERSE_Y_RAD:
                # 逆ひじは一定角度以上は正ひじに直す
                elbow_yz_sign = 1

            # ひじY
            elbow_bf = dest_motion.bones[BoneNames.elbow(direction)][fno]
            elbow_bf.rotation = MQuaternion.from_axis_angles(
                elbow_y_axis, elbow_yz_rad * elbow_yz_sign
            )
            dest_motion.insert_bone_frame(elbow_bf)

            elbow_direction_bf = dest_motion.bones[
                BoneNames.elbow_direction(direction)
            ][fno]
            elbow_direction_bf.rotation = elbow_bf.rotation.copy()
            elbow_direction_bf.register = True
            dest_motion.insert_bone_frame(elbow_direction_bf)

            # ひじY 結果解決後の行列取得
            after_matrixes = dest_motion.animate_bone(
                [fno],
                dest_model,
                [
                    BoneNames.wrist(direction),
                    BoneNames.elbow_rotate_ik(direction),
                ],
            )

            # ひじ回転
            elbow_rotate_ik_bf = dest_motion.bones[
                BoneNames.elbow_rotate_ik(direction)
            ][fno]
            # 初期手首が最終的に初期値になるよう、配置
            elbow_rotate_ik_bf.position = (
                after_matrixes[
                    BoneNames.elbow_rotate_ik(direction), fno
                ].global_matrix.inverse()
                * after_matrixes[BoneNames.wrist(direction), fno].position
            )
            dest_motion.insert_bone_frame(elbow_rotate_ik_bf)

            # ■ --------------
            VmdWriter(
                dest_motion,
                f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}ひじ回転_{fno:04d}.vmd",
                model_name="Test Model",
            ).save()
            # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.elbow_rotate_ik(direction)],
                ik_qqs,
            )

            # ■ --------------
            elbow_rotate_bf = dest_motion.bones[BoneNames.elbow_rotate(direction)][fno]
            elbow_rotate_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.elbow_rotate(direction)].index
            ]
            elbow_rotate_bf.register = True
            dest_motion.insert_bone_frame(elbow_rotate_bf)
            # ■ --------------

            # 腕捩 解決結果を設定
            arm_twist_bf = dest_motion.bones[BoneNames.arm_twist(direction)][fno]

            # 腕回転
            arm_twist_qq = arm_twist_bf.rotation
            arm_twist_axis = arm_twist_qq.to_axis()
            arm_twist_rad = arm_twist_qq.to_radian()
            arm_twist_sign = np.sign(arm_twist_fixed_axis.dot(arm_twist_axis))

            # # ひじ回転
            # elbow_rotate_qq = ik_qqs[
            #     dest_model.bones[BoneNames.elbow_rotate(direction)].index
            # ]
            # elbow_rotate_axis = elbow_rotate_qq.to_axis()
            # elbow_rotate_rad = elbow_rotate_qq.to_radian()
            # elbow_rotate_sign = np.sign(arm_twist_fixed_axis.dot(elbow_rotate_axis))

            # 2つを合成
            arm_twist_bf.rotation = MQuaternion.from_axis_angles(
                arm_twist_fixed_axis,
                (arm_twist_rad * arm_twist_sign)
                # + (elbow_rotate_rad * elbow_rotate_sign),
            )
            dest_motion.insert_bone_frame(arm_twist_bf)

            # ひじ回転 結果解決後の行列取得
            after_matrixes = dest_motion.animate_bone(
                [fno],
                dest_model,
                [
                    BoneNames.wrist(direction),
                    BoneNames.wrist_direction_ik(direction),
                ],
            )

            # 手首の捩りを除いた角度を取得
            _, _, _, wrist_yz_qq = dest_initial_matrixes[
                BoneNames.wrist(direction), fno
            ].frame_fk_rotation.separate_by_axis(wrist_twist_fixed_axis)

            # 手首YZを手首に設定
            wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
            wrist_bf.rotation = wrist_yz_qq
            dest_motion.insert_bone_frame(wrist_bf)

            # 手首 結果解決後の行列取得
            after_matrixes = dest_motion.animate_bone(
                [fno],
                dest_model,
                [
                    BoneNames.wrist_vertical(direction),
                    BoneNames.wrist_rotate_ik(direction),
                ],
            )

            # 手首回転 手捩から垂直に出たIKターゲット位置
            wrist_rotate_ik_bf = dest_motion.bones[
                BoneNames.wrist_rotate_ik(direction)
            ][fno]
            # 手首垂線が最終的に初期値になるよう、配置
            wrist_rotate_ik_bf.position = after_matrixes[
                BoneNames.wrist_rotate_ik(direction), fno
            ].global_matrix.inverse() * (
                after_matrixes[BoneNames.wrist_twist(direction), fno].position
                + (
                    dest_initial_matrixes[
                        BoneNames.wrist_twist_vertical(direction), fno
                    ].position
                    - dest_initial_matrixes[
                        BoneNames.wrist_twist(direction), fno
                    ].position
                )
            )
            dest_motion.insert_bone_frame(wrist_rotate_ik_bf)

            # ■ --------------
            VmdWriter(
                dest_motion,
                f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}手首回転_{fno:04d}.vmd",
                model_name="Test Model",
            ).save()
            # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.wrist_rotate_ik(direction)],
                ik_qqs,
            )

            # ■ --------------
            wrist_rotate_bf = dest_motion.bones[BoneNames.wrist_rotate(direction)][fno]
            wrist_rotate_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.wrist_rotate(direction)].index
            ]
            wrist_rotate_bf.register = True
            dest_motion.insert_bone_frame(wrist_rotate_bf)
            # ■ --------------

            # 手首
            wrist_twist_bf = dest_motion.bones[BoneNames.wrist_twist(direction)][fno]
            wrist_twist_bf.rotation = ik_qqs[
                dest_model.bones[BoneNames.wrist_rotate(direction)].index
            ]
            dest_motion.insert_bone_frame(wrist_twist_bf)

            # ■ --------------
            VmdWriter(
                dest_motion,
                f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}手首回転結果_{fno:04d}.vmd",
                model_name="Test Model",
            ).save()
            # ■ --------------

        # 終わったらIKボーンのキーフレを削除
        del dest_motion.bones[BoneNames.arm_direction_ik(direction)]
        del dest_motion.bones[BoneNames.arm_rotate_ik(direction)]
        del dest_motion.bones[BoneNames.elbow_direction_ik(direction)]
        del dest_motion.bones[BoneNames.elbow_rotate_ik(direction)]
        del dest_motion.bones[BoneNames.wrist_direction_ik(direction)]
        del dest_motion.bones[BoneNames.wrist_rotate_ik(direction)]
        del dest_motion.bones[BoneNames.arm_direction(direction)]
        del dest_motion.bones[BoneNames.arm_rotate(direction)]
        del dest_motion.bones[BoneNames.elbow_direction(direction)]
        del dest_motion.bones[BoneNames.elbow_rotate(direction)]
        del dest_motion.bones[BoneNames.wrist_direction(direction)]
        del dest_motion.bones[BoneNames.wrist_rotate(direction)]

        return sizing_idx, direction, dest_motion

    def setup_model_ik(
        self,
        sizing_idx: int,
        is_src: bool,
        model: PmxModel,
    ) -> tuple[int, bool, PmxModel]:
        logger.info(
            "【No.{x}】捩り分散：追加IKセットアップ({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        ik_model = model.copy()
        sizing_display_slot = ik_model.display_slots[BoneNames.sizing_display_slot()]

        for direction in ("左", "右"):
            # 腕方向 ---------------
            arm_direction_bone = Bone(
                index=ik_model.bones[BoneNames.elbow(direction)].index + 1,
                name=BoneNames.arm_direction(direction),
            )
            arm_direction_bone.position = ik_model.bones[
                BoneNames.shoulder_c(direction)
            ].position.copy()

            arm_direction_bone.is_system = True
            arm_direction_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
            )

            ik_model.insert_bone(arm_direction_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_direction_bone.index)
            )

            # 腕方向先 ---------------
            arm_direction_tail_bone = Bone(
                index=arm_direction_bone.index + 1,
                name=BoneNames.arm_direction_tail(direction),
            )
            arm_direction_tail_bone.position = ik_model.bones[
                BoneNames.elbow(direction)
            ].position.copy()

            arm_direction_tail_bone.is_system = True
            arm_direction_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            ik_model.insert_bone(arm_direction_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_direction_tail_bone.index)
            )

            # 腕方向の表示先を腕方向先に変更
            arm_direction_bone.tail_index = arm_direction_tail_bone.index

            # 腕方向IK ---------------
            arm_direction_ik_bone = Bone(
                index=arm_direction_tail_bone.index + 1,
                name=BoneNames.arm_direction_ik(direction),
            )
            arm_direction_ik_bone.position = MVector3D()
            arm_direction_ik_bone.is_system = True
            arm_direction_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            # 腕方向IKの構成
            arm_direction_ik = Ik()
            arm_direction_ik.bone_index = arm_direction_tail_bone.index
            arm_direction_ik.loop_count = 8
            arm_direction_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # 腕方向
            arm_direction_ik_direction = IkLink()
            arm_direction_ik_direction.bone_index = arm_direction_bone.index
            arm_direction_ik.links.append(arm_direction_ik_direction)

            arm_direction_ik_bone.ik = arm_direction_ik

            ik_model.insert_bone(arm_direction_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_direction_ik_bone.index)
            )

            # 腕回転 ---------------
            arm_rotate_bone = Bone(
                index=arm_direction_ik_bone.index + 1,
                name=BoneNames.arm_rotate(direction),
            )
            arm_rotate_bone.position = ik_model.bones[
                BoneNames.arm_twist(direction)
            ].position.copy()

            arm_rotate_bone.is_system = True
            arm_rotate_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
                | BoneFlg.HAS_FIXED_AXIS
            )

            # 腕捩の軸制限をコピー
            arm_rotate_bone.fixed_axis = ik_model.bones[
                BoneNames.arm_twist(direction)
            ].fixed_axis.copy()

            ik_model.insert_bone(arm_rotate_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_rotate_bone.index)
            )

            # 腕回転先 ---------------
            arm_rotate_tail_bone = Bone(
                index=arm_rotate_bone.index + 1,
                name=BoneNames.arm_rotate_tail(direction),
            )
            arm_rotate_tail_bone.position = ik_model.bones[
                BoneNames.arm_twist_vertical(direction)
            ].position.copy()

            arm_rotate_tail_bone.is_system = True
            arm_rotate_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            ik_model.insert_bone(arm_rotate_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_rotate_tail_bone.index)
            )

            # 腕回転の表示先を腕回転先に変更
            arm_rotate_bone.tail_index = arm_rotate_tail_bone.index

            # 腕回転IK ---------------
            arm_rotate_ik_bone = Bone(
                index=arm_rotate_tail_bone.index + 1,
                name=BoneNames.arm_rotate_ik(direction),
            )
            arm_rotate_ik_bone.position = MVector3D()
            arm_rotate_ik_bone.is_system = True
            arm_rotate_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(arm_rotate_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_rotate_ik_bone.index)
            )

            # 腕回転IKの構成
            arm_rotate_ik = Ik()
            arm_rotate_ik.bone_index = arm_rotate_tail_bone.index
            arm_rotate_ik.loop_count = 8
            arm_rotate_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # 腕回転
            arm_rotate_ik_rotate = IkLink()
            arm_rotate_ik_rotate.bone_index = arm_rotate_bone.index
            arm_rotate_ik.links.append(arm_rotate_ik_rotate)

            arm_rotate_ik_bone.ik = arm_rotate_ik

            # ひじ方向 ---------------
            elbow_direction_bone = Bone(
                index=ik_model.bones[BoneNames.wrist(direction)].index + 1,
                name=BoneNames.elbow_direction(direction),
            )
            elbow_direction_bone.position = ik_model.bones[
                BoneNames.elbow(direction)
            ].position.copy()

            elbow_direction_bone.is_system = True
            elbow_direction_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
            )

            ik_model.insert_bone(elbow_direction_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_direction_bone.index)
            )

            # ひじ方向先 ---------------
            elbow_direction_tail_bone = Bone(
                index=elbow_direction_bone.index + 1,
                name=BoneNames.elbow_direction_tail(direction),
            )
            elbow_direction_tail_bone.position = ik_model.bones[
                BoneNames.wrist(direction)
            ].position.copy()

            elbow_direction_tail_bone.is_system = True
            elbow_direction_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            ik_model.insert_bone(elbow_direction_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_direction_tail_bone.index)
            )

            # ひじ方向の表示先をひじ方向先に変更
            elbow_direction_bone.tail_index = elbow_direction_tail_bone.index

            # ひじ方向IK ---------------
            elbow_direction_ik_bone = Bone(
                index=elbow_direction_tail_bone.index + 1,
                name=BoneNames.elbow_direction_ik(direction),
            )
            elbow_direction_ik_bone.position = MVector3D()
            elbow_direction_ik_bone.is_system = True
            elbow_direction_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            # ひじ方向IKの構成
            elbow_direction_ik = Ik()
            elbow_direction_ik.bone_index = elbow_direction_tail_bone.index
            elbow_direction_ik.loop_count = 8
            elbow_direction_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # ひじ方向
            elbow_direction_ik_direction = IkLink()
            elbow_direction_ik_direction.bone_index = elbow_direction_bone.index
            elbow_direction_ik_direction.local_angle_limit = True
            # ローカル軸での角度制限を行う
            elbow_direction_ik_direction.local_min_angle_limit.radians = MVector3D(
                0, -ELBOW_REVERSE_Y_RAD, 0
            )
            elbow_direction_ik_direction.local_max_angle_limit.degrees = MVector3D(
                0, 180, 0
            )
            elbow_direction_ik.links.append(elbow_direction_ik_direction)

            elbow_direction_ik_bone.ik = elbow_direction_ik

            ik_model.insert_bone(elbow_direction_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_direction_ik_bone.index)
            )

            # ひじ回転 ---------------
            elbow_rotate_bone = Bone(
                index=elbow_direction_ik_bone.index + 1,
                name=BoneNames.elbow_rotate(direction),
            )
            elbow_rotate_bone.position = ik_model.bones[
                BoneNames.elbow(direction)
            ].position.copy()

            elbow_rotate_bone.is_system = True
            elbow_rotate_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
                | BoneFlg.HAS_FIXED_AXIS
            )

            # 腕捩の軸制限をコピー
            elbow_rotate_bone.fixed_axis = ik_model.bones[
                BoneNames.arm_twist(direction)
            ].fixed_axis.copy()

            ik_model.insert_bone(elbow_rotate_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_rotate_bone.index)
            )

            # ひじ回転先 ---------------
            elbow_rotate_tail_bone = Bone(
                index=elbow_rotate_bone.index + 1,
                name=BoneNames.elbow_rotate_tail(direction),
            )
            elbow_rotate_tail_bone.position = ik_model.bones[
                BoneNames.wrist(direction)
            ].position.copy()

            elbow_rotate_tail_bone.is_system = True
            elbow_rotate_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            ik_model.insert_bone(elbow_rotate_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_rotate_tail_bone.index)
            )

            # ひじ回転の表示先をひじ回転先に変更
            elbow_rotate_bone.tail_index = elbow_rotate_tail_bone.index

            # ひじ回転IK ---------------
            elbow_rotate_ik_bone = Bone(
                index=elbow_rotate_tail_bone.index + 1,
                name=BoneNames.elbow_rotate_ik(direction),
            )
            elbow_rotate_ik_bone.position = MVector3D()
            elbow_rotate_ik_bone.is_system = True
            elbow_rotate_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(elbow_rotate_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_rotate_ik_bone.index)
            )

            # ひじ回転IKの構成
            elbow_rotate_ik = Ik()
            elbow_rotate_ik.bone_index = elbow_rotate_tail_bone.index
            elbow_rotate_ik.loop_count = 8
            elbow_rotate_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # ひじ回転
            elbow_rotate_ik_rotate = IkLink()
            elbow_rotate_ik_rotate.bone_index = elbow_rotate_bone.index
            elbow_rotate_ik.links.append(elbow_rotate_ik_rotate)

            elbow_rotate_ik_bone.ik = elbow_rotate_ik

            # 手首方向 ---------------
            wrist_direction_bone = Bone(
                index=ik_model.bones[BoneNames.wrist_tail(direction)].index + 1,
                name=BoneNames.wrist_direction(direction),
            )
            wrist_direction_bone.position = ik_model.bones[
                BoneNames.wrist(direction)
            ].position.copy()

            wrist_direction_bone.is_system = True
            wrist_direction_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
            )

            ik_model.insert_bone(wrist_direction_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_direction_bone.index)
            )

            # 手首方向先 ---------------
            wrist_direction_tail_bone = Bone(
                index=wrist_direction_bone.index + 1,
                name=BoneNames.wrist_direction_tail(direction),
            )
            wrist_direction_tail_bone.position = ik_model.bones[
                BoneNames.wrist_tail(direction)
            ].position.copy()

            wrist_direction_tail_bone.is_system = True
            wrist_direction_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            ik_model.insert_bone(wrist_direction_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_direction_tail_bone.index)
            )

            # 手首方向の表示先を手首方向先に変更
            wrist_direction_bone.tail_index = wrist_direction_tail_bone.index

            # 手首方向IK ---------------
            wrist_direction_ik_bone = Bone(
                index=wrist_direction_tail_bone.index + 1,
                name=BoneNames.wrist_direction_ik(direction),
            )
            wrist_direction_ik_bone.position = MVector3D()
            wrist_direction_ik_bone.is_system = True
            wrist_direction_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            # 手首方向IKの構成
            wrist_direction_ik = Ik()
            wrist_direction_ik.bone_index = wrist_direction_tail_bone.index
            wrist_direction_ik.loop_count = 8
            wrist_direction_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # 手首方向
            wrist_direction_ik_direction = IkLink()
            wrist_direction_ik_direction.bone_index = wrist_direction_bone.index
            wrist_direction_ik.links.append(wrist_direction_ik_direction)

            wrist_direction_ik_bone.ik = wrist_direction_ik

            ik_model.insert_bone(wrist_direction_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_direction_ik_bone.index)
            )

            # 手首回転 ---------------
            wrist_rotate_bone = Bone(
                index=wrist_direction_ik_bone.index + 1,
                name=BoneNames.wrist_rotate(direction),
            )
            wrist_rotate_bone.position = ik_model.bones[
                BoneNames.wrist_twist(direction)
            ].position.copy()

            wrist_rotate_bone.is_system = True
            wrist_rotate_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
                | BoneFlg.HAS_FIXED_AXIS
            )

            # 手捩の軸制限をコピー
            wrist_rotate_bone.fixed_axis = ik_model.bones[
                BoneNames.wrist_twist(direction)
            ].fixed_axis.copy()

            ik_model.insert_bone(wrist_rotate_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_rotate_bone.index)
            )

            # 手首回転先 ---------------
            wrist_rotate_tail_bone = Bone(
                index=wrist_rotate_bone.index + 1,
                name=BoneNames.wrist_rotate_tail(direction),
            )
            wrist_rotate_tail_bone.position = ik_model.bones[
                BoneNames.wrist_twist_vertical(direction)
            ].position.copy()

            wrist_rotate_tail_bone.is_system = True
            wrist_rotate_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            ik_model.insert_bone(wrist_rotate_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_rotate_tail_bone.index)
            )

            # 手首回転の表示先を手首回転先に変更
            wrist_rotate_bone.tail_index = wrist_rotate_tail_bone.index

            # 手首回転IK ---------------
            wrist_rotate_ik_bone = Bone(
                index=wrist_rotate_tail_bone.index + 1,
                name=BoneNames.wrist_rotate_ik(direction),
            )
            wrist_rotate_ik_bone.position = MVector3D()
            wrist_rotate_ik_bone.is_system = True
            wrist_rotate_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(wrist_rotate_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_rotate_ik_bone.index)
            )

            # 手首回転IKの構成
            wrist_rotate_ik = Ik()
            wrist_rotate_ik.bone_index = wrist_rotate_tail_bone.index
            wrist_rotate_ik.loop_count = 8
            wrist_rotate_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # 手首回転
            wrist_rotate_ik_rotate = IkLink()
            wrist_rotate_ik_rotate.bone_index = wrist_rotate_bone.index
            wrist_rotate_ik.links.append(wrist_rotate_ik_rotate)

            wrist_rotate_ik_bone.ik = wrist_rotate_ik

        for direction in ("左", "右"):
            # 腕方向
            ik_model.bones[
                BoneNames.arm_direction(direction)
            ].parent_index = ik_model.bones[BoneNames.shoulder_c(direction)].index

            ik_model.bones[
                BoneNames.arm_direction_tail(direction)
            ].parent_index = ik_model.bones[BoneNames.arm_direction(direction)].index

            ik_model.bones[
                BoneNames.arm_direction_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # 腕回転
            ik_model.bones[
                BoneNames.arm_rotate(direction)
            ].parent_index = ik_model.bones[BoneNames.arm_direction(direction)].index

            ik_model.bones[
                BoneNames.arm_rotate_tail(direction)
            ].parent_index = ik_model.bones[BoneNames.arm_rotate(direction)].index

            ik_model.bones[
                BoneNames.arm_rotate_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # ひじ方向
            ik_model.bones[
                BoneNames.elbow_direction(direction)
            ].parent_index = ik_model.bones[BoneNames.arm_rotate(direction)].index

            ik_model.bones[
                BoneNames.elbow_direction_tail(direction)
            ].parent_index = ik_model.bones[BoneNames.elbow_direction(direction)].index

            ik_model.bones[
                BoneNames.elbow_direction_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # ひじ回転
            ik_model.bones[
                BoneNames.elbow_rotate(direction)
            ].parent_index = ik_model.bones[BoneNames.elbow_direction(direction)].index

            ik_model.bones[
                BoneNames.elbow_rotate_tail(direction)
            ].parent_index = ik_model.bones[BoneNames.elbow_rotate(direction)].index

            ik_model.bones[
                BoneNames.elbow_rotate_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # 手首方向
            ik_model.bones[
                BoneNames.wrist_direction(direction)
            ].parent_index = ik_model.bones[BoneNames.elbow_rotate(direction)].index

            ik_model.bones[
                BoneNames.wrist_direction_tail(direction)
            ].parent_index = ik_model.bones[BoneNames.wrist_direction(direction)].index

            ik_model.bones[
                BoneNames.wrist_direction_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # 手首回転
            ik_model.bones[
                BoneNames.wrist_rotate(direction)
            ].parent_index = ik_model.bones[BoneNames.wrist_direction(direction)].index

            ik_model.bones[
                BoneNames.wrist_rotate_tail(direction)
            ].parent_index = ik_model.bones[BoneNames.wrist_rotate(direction)].index

            ik_model.bones[
                BoneNames.wrist_rotate_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # ひじの親は腕捩
            ik_model.bones[BoneNames.elbow(direction)].parent_index = ik_model.bones[
                BoneNames.arm_twist(direction)
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
                    f"twist_{os.path.basename(ik_model.path)}",
                ),
                include_system=True,
            ).save()

        return sizing_idx, is_src, ik_model

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
            "【No.{x}】【{d}】初期位置取得({m})",
            x=sizing_idx + 1,
            d=__(direction),
            m=model_type,
            decoration=MLogger.Decoration.LINE,
        )

        tail_bone_names: list[str] = [
            BoneNames.arm_direction_ik(direction),
            BoneNames.arm_rotate_ik(direction),
            BoneNames.elbow_direction_ik(direction),
            BoneNames.elbow_rotate_ik(direction),
            BoneNames.wrist_direction_ik(direction),
            BoneNames.arm_direction_tail(direction),
            BoneNames.arm_rotate_tail(direction),
            BoneNames.elbow_direction_tail(direction),
            BoneNames.elbow_rotate_tail(direction),
            BoneNames.wrist_direction_tail(direction),
            BoneNames.arm_twist_vertical(direction),
            BoneNames.elbow_vertical(direction),
            BoneNames.wrist_twist_vertical(direction),
            BoneNames.wrist_vertical(direction),
            BoneNames.wrist_tail(direction),
        ]
        fnos_set: set[int] = {0}

        for tail_bone_name in tail_bone_names:
            for tree_bone_name in model.bone_trees[tail_bone_name].names:
                fnos_set |= set(motion.bones[tree_bone_name].register_indexes)

        fnos: list[int] = sorted(fnos_set)

        initial_matrixes = motion.animate_bone(
            fnos,
            model,
            tail_bone_names,
            out_fno_log=True,
            is_calc_ik=False,
            description=f"{sizing_idx + 1}|{__(direction)}|{__('初期位置取得')}|{model_type}",
        )

        return (
            sizing_idx,
            is_src,
            direction,
            initial_matrixes,
        )
