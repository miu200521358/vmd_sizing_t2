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
ELBOW_REVERSE_Y_RAD = radians(7)


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
        dest_initial_matrixes: VmdBoneFrameTrees,
        direction: str,
    ) -> tuple[int, str, VmdMotion]:
        logger.info(
            "【No.{i}】【{d}】捩り分散",
            i=sizing_idx + 1,
            d=__(direction),
            decoration=MLogger.Decoration.LINE,
        )

        elbow_x_axis = dest_model.bones[
            BoneNames.elbow(direction)
        ].corrected_local_x_vector

        elbow_y_axis = dest_model.bones[
            BoneNames.elbow(direction)
        ].corrected_local_y_vector

        arm_twist_fixed_axis = dest_model.bones[
            BoneNames.arm_twist(direction)
        ].corrected_fixed_axis

        wrist_twist_fixed_axis = dest_model.bones[
            BoneNames.wrist_twist(direction)
        ].corrected_fixed_axis

        fnos = sorted(
            {0}
            | set(dest_motion.bones[BoneNames.arm(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.arm_twist(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.elbow(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.wrist_twist(direction)].register_indexes)
            | set(dest_motion.bones[BoneNames.wrist(direction)].register_indexes)
        )

        # 処理対象ボーン名取得
        target_bone_names = dest_motion.bones.get_animate_bone_names(
            dest_model,
            [
                BoneNames.elbow_vertical(direction),
                BoneNames.wrist_vertical(direction),
                BoneNames.wrist_tail(direction),
            ],
        )

        # 処理対象ボーンの行列取得
        (
            bone_dict,
            bone_offset_matrixes,
            bone_pos_matrixes,
        ) = dest_motion.bones.create_bone_matrixes(dest_model, target_bone_names)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】捩り分散 - 準備",
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

            arm_rotate_ik_bf = dest_motion.bones[BoneNames.arm_rotate_ik(direction)][
                fno
            ]
            arm_rotate_ik_bf.register = True
            dest_motion.insert_bone_frame(arm_rotate_ik_bf)

            wrist_rotate_ik_bf = dest_motion.bones[
                BoneNames.wrist_rotate_ik(direction)
            ][fno]
            wrist_rotate_ik_bf.register = True
            dest_motion.insert_bone_frame(wrist_rotate_ik_bf)

            wrist_direction_ik_bf = dest_motion.bones[
                BoneNames.wrist_direction_ik(direction)
            ][fno]
            wrist_direction_ik_bf.register = True
            dest_motion.insert_bone_frame(wrist_direction_ik_bf)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】捩り分散",
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

            # --------------

            # 腕
            arm_bf = dest_motion.bones[BoneNames.arm(direction)][fno]
            _, _, _, arm_yz_qq = arm_bf.rotation.separate_by_axis(arm_twist_fixed_axis)
            arm_bf.rotation = arm_yz_qq
            dest_motion.insert_bone_frame(arm_bf)

            arm_bone = dest_model.bones[BoneNames.arm(direction)]
            motion_bone_qqs[0, arm_bone.index] = arm_yz_qq.to_matrix4x4().vector

            # 腕捩(初期値)
            arm_twist_bf = dest_motion.bones[BoneNames.arm_twist(direction)][fno]
            arm_twist_bf.rotation = MQuaternion()
            dest_motion.insert_bone_frame(arm_twist_bf)

            arm_twist_bone = dest_model.bones[BoneNames.arm_twist(direction)]
            motion_bone_qqs[0, arm_twist_bone.index] = np.eye(4)

            # ひじの捩りを除いた角度を取
            elbow_bf = dest_motion.bones[BoneNames.elbow(direction)][fno]
            _, _, _, elbow_yz_qq = elbow_bf.rotation.separate_by_axis(elbow_x_axis)

            # ひじのZは殺してY回転だけにして登録
            elbow_yz_axis = elbow_yz_qq.to_axis()
            elbow_yz_rad = elbow_yz_qq.to_radian()
            elbow_yz_sign = np.sign(elbow_y_axis.dot(elbow_yz_axis))
            if elbow_yz_sign < 0 and elbow_yz_rad > ELBOW_REVERSE_Y_RAD:
                # 逆ひじは一定角度以上は正ひじに直す
                elbow_yz_sign = 1

            # ひじY
            elbow_bf.rotation = MQuaternion.from_axis_angles(
                elbow_y_axis, elbow_yz_rad * elbow_yz_sign
            )
            dest_motion.insert_bone_frame(elbow_bf)

            elbow_bone = dest_model.bones[BoneNames.elbow(direction)]
            motion_bone_qqs[
                0, elbow_bone.index
            ] = elbow_bf.rotation.to_matrix4x4().vector

            # 手捩(初期値)
            wrist_twist_bf = dest_motion.bones[BoneNames.wrist_twist(direction)][fno]
            wrist_twist_bf.rotation = MQuaternion()
            dest_motion.insert_bone_frame(wrist_twist_bf)

            wrist_twist_bone = dest_model.bones[BoneNames.wrist_twist(direction)]
            motion_bone_qqs[0, wrist_twist_bone.index] = np.eye(4)

            # 手首
            wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
            _, _, _, wrist_yz_qq = wrist_bf.rotation.separate_by_axis(
                wrist_twist_fixed_axis
            )
            wrist_bf.rotation = wrist_yz_qq
            dest_motion.insert_bone_frame(wrist_bf)

            wrist_bone = dest_model.bones[BoneNames.wrist(direction)]
            motion_bone_qqs[0, wrist_bone.index] = wrist_yz_qq.to_matrix4x4().vector

            # 捩り成分OFF解決後の行列取得
            twist_off_matrixes = dest_motion.bones.calc_bone_matrixes(
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

            # 腕回転
            arm_rotate_ik_bf = dest_motion.bones[BoneNames.arm_rotate_ik(direction)][
                fno
            ]
            # ひじ垂線はYひじの場合、手首から垂線までの角度が絶対に変わらない
            # そのため、手首からの垂線の相対位置を最初の手首位置に加算する
            # ひじ垂線が最終的に初期値になるよう、配置
            arm_rotate_ik_bf.position = (
                dest_initial_matrixes[BoneNames.elbow_tail(direction), fno].position
            ) + (
                twist_off_matrixes[BoneNames.elbow_vertical(direction), fno].position
                - twist_off_matrixes[BoneNames.elbow_tail(direction), fno].position
            )
            # arm_rotate_ik_bf.position = dest_initial_matrixes[
            #     BoneNames.wrist(direction), fno
            # ].global_matrix * (
            #     twist_off_matrixes[
            #         BoneNames.wrist(direction), fno
            #     ].global_matrix.inverse()
            #     * twist_off_matrixes[BoneNames.elbow_vertical(direction), fno].position
            # )
            dest_motion.insert_bone_frame(arm_rotate_ik_bf)

            # # ■ --------------
            # from datetime import datetime

            # from mlib.vmd.vmd_writer import VmdWriter

            # VmdWriter(
            #     dest_motion,
            #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}腕回転_{fno:04d}.vmd",
            #     model_name="Test Model",
            # ).save()
            # # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.arm_rotate_ik(direction)],
            )

            arm_twist_bf = dest_motion.bones[BoneNames.arm_twist(direction)][fno]
            arm_twist_bf.rotation = ik_qqs[arm_twist_bone.index]
            dest_motion.insert_bone_frame(arm_twist_bf)

            motion_bone_qqs[0, arm_twist_bone.index] = (
                ik_qqs[arm_twist_bone.index].to_matrix4x4().vector
            )

            # 腕捩り解決後の行列取得
            arm_twist_matrixes = dest_motion.bones.calc_bone_matrixes(
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

            # 手首回転
            wrist_rotate_ik_bf = dest_motion.bones[
                BoneNames.wrist_rotate_ik(direction)
            ][fno]
            # 手首垂線が最初の手首から手首垂線の相対位置地点に、現在の手首から置けるよう調整
            wrist_rotate_ik_bf.position = arm_twist_matrixes[
                BoneNames.wrist(direction), fno
            ].position + (
                dest_initial_matrixes[BoneNames.wrist_vertical(direction), fno].position
                - dest_initial_matrixes[BoneNames.wrist(direction), fno].position
            )
            # # 手首垂線が最終的に初期値になるよう、配置
            # wrist_rotate_ik_bf.position = dest_initial_matrixes[
            #     BoneNames.wrist_vertical(direction), fno
            # ].position
            # # 手首垂線が最初の手首から手首垂線の相対位置地点に、現在の手首から置けるよう調整
            # wrist_rotate_ik_bf.position = arm_twist_matrixes[
            #     BoneNames.wrist(direction), fno
            # ].global_matrix * (
            #     dest_initial_matrixes[
            #         BoneNames.wrist(direction), fno
            #     ].global_matrix.inverse()
            #     * dest_initial_matrixes[
            #         BoneNames.wrist_vertical(direction), fno
            #     ].position
            # )
            dest_motion.insert_bone_frame(wrist_rotate_ik_bf)

            # # ■ --------------
            # VmdWriter(
            #     dest_motion,
            #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}手首回転_{fno:04d}.vmd",
            #     model_name="Test Model",
            # ).save()
            # # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.wrist_rotate_ik(direction)],
            )

            # 手捩
            wrist_twist_bf = dest_motion.bones[BoneNames.wrist_twist(direction)][fno]
            wrist_twist_bf.rotation = ik_qqs[wrist_twist_bone.index]
            dest_motion.insert_bone_frame(wrist_twist_bf)

            # 手捩り解決後の行列取得
            wrist_twist_matrixes = dest_motion.bones.calc_bone_matrixes(
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

            # 手首方向
            wrist_direction_ik_bf = dest_motion.bones[
                BoneNames.wrist_direction_ik(direction)
            ][fno]
            # 手首先が最終的に初期値になるよう、配置
            wrist_direction_ik_bf.position = wrist_twist_matrixes[
                BoneNames.wrist(direction), fno
            ].position + (
                dest_initial_matrixes[BoneNames.wrist_tail(direction), fno].position
                - dest_initial_matrixes[BoneNames.wrist(direction), fno].position
            )
            dest_motion.insert_bone_frame(wrist_direction_ik_bf)

            # # ■ --------------
            # VmdWriter(
            #     dest_motion,
            #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}手首方向_{fno:04d}.vmd",
            #     model_name="Test Model",
            # ).save()
            # # ■ --------------

            # IK解決する
            _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                fidx,
                fno,
                dest_model,
                dest_model.bones[BoneNames.wrist_direction_ik(direction)],
            )

            # 手首
            wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
            wrist_bf.rotation = ik_qqs[wrist_bone.index]
            dest_motion.insert_bone_frame(wrist_bf)

            # # ■ --------------
            # VmdWriter(
            #     dest_motion,
            #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{direction}手首方向結果_{fno:04d}.vmd",
            #     model_name="Test Model",
            # ).save()
            # # ■ --------------

        # 終わったらIKボーンのキーフレを削除
        del dest_motion.bones[BoneNames.arm_rotate_ik(direction)]
        del dest_motion.bones[BoneNames.wrist_rotate_ik(direction)]
        del dest_motion.bones[BoneNames.wrist_direction_ik(direction)]

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
            # 腕回転IK ---------------
            arm_rotate_ik_bone = Bone(
                index=ik_model.bones[BoneNames.wrist(direction)].index + 1,
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
            arm_rotate_ik.bone_index = ik_model.bones[
                BoneNames.elbow_vertical(direction)
            ].index
            arm_rotate_ik.loop_count = 300
            arm_rotate_ik.unit_rotation.degrees = MVector3D(180, 0, 0)

            # # ひじ
            # arm_rotate_ik_elbow = IkLink()
            # arm_rotate_ik_elbow.bone_index = ik_model.bones[
            #     BoneNames.elbow(direction)
            # ].index
            # arm_rotate_ik_elbow.local_angle_limit = True
            # # ローカル軸での角度制限を行う
            # arm_rotate_ik_elbow.local_min_angle_limit.radians = MVector3D(
            #     0, -ELBOW_REVERSE_Y_RAD, 0
            # )
            # arm_rotate_ik_elbow.local_max_angle_limit.degrees = MVector3D(0, 180, 0)
            # arm_rotate_ik.links.append(arm_rotate_ik_elbow)

            # 腕捩
            arm_rotate_ik_arm_twist = IkLink()
            arm_rotate_ik_arm_twist.bone_index = ik_model.bones[
                BoneNames.arm_twist(direction)
            ].index
            arm_rotate_ik.links.append(arm_rotate_ik_arm_twist)

            arm_rotate_ik_bone.ik = arm_rotate_ik

            # 手首回転IK ---------------
            wrist_rotate_ik_bone = Bone(
                index=arm_rotate_ik_bone.index + 1,
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
            wrist_rotate_ik.bone_index = ik_model.bones[
                BoneNames.wrist_vertical(direction)
            ].index
            wrist_rotate_ik.loop_count = 200
            wrist_rotate_ik.unit_rotation.degrees = MVector3D(180, 0, 0)

            # 手捩
            wrist_rotate_ik_rotate = IkLink()
            wrist_rotate_ik_rotate.bone_index = ik_model.bones[
                BoneNames.wrist_twist(direction)
            ].index
            wrist_rotate_ik.links.append(wrist_rotate_ik_rotate)

            wrist_rotate_ik_bone.ik = wrist_rotate_ik

            # 手首方向IK ---------------
            wrist_direction_ik_bone = Bone(
                index=wrist_rotate_ik_bone.index + 1,
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
            ik_model.insert_bone(wrist_direction_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=wrist_direction_ik_bone.index)
            )

            # 手首方向IKの構成
            wrist_direction_ik = Ik()
            wrist_direction_ik.bone_index = ik_model.bones[
                BoneNames.wrist_tail(direction)
            ].index
            wrist_direction_ik.loop_count = 20
            wrist_direction_ik.unit_rotation.degrees = MVector3D(180, 0, 0)

            # 手首
            wrist_direction_ik_direction = IkLink()
            wrist_direction_ik_direction.bone_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index
            wrist_direction_ik.links.append(wrist_direction_ik_direction)

            wrist_direction_ik_bone.ik = wrist_direction_ik

        for direction in ("左", "右"):
            # 腕回転
            ik_model.bones[
                BoneNames.arm_rotate_ik(direction)
            ].parent_index = ik_model.bones[BoneNames.root()].index

            # 手首回転
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
            BoneNames.elbow_vertical(direction),
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
