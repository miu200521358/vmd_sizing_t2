import os
from math import radians
from typing import Iterable, Optional

import numpy as np
from service.usecase.bone_names import BoneNames

from mlib.core.logger import MLogger
from mlib.core.math import MQuaternion, MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlotReference, Ik, IkLink
from mlib.vmd.vmd_collection import VmdMotion
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

        arm_twist_fixed_axis = dest_model.bones[
            BoneNames.arm_twist(direction)
        ].fixed_axis
        elbow_y_axis = dest_model.bones[
            BoneNames.arm(direction)
        ].corrected_local_y_vector
        wrist_twist_fixed_axis = dest_model.bones[
            BoneNames.wrist_twist(direction)
        ].fixed_axis
        # 3度までは逆ひじを許可する
        elbow_reverse_y_rad = radians(3)

        for fidx, fno in enumerate(fnos):
            logger.count(
                "【No.{x}】【{d}】捩り分散",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(fnos),
                display_block=1000,
            )

            # IKターゲットボーンの位置をIKの位置として登録
            arm_ik_bf = dest_motion.bones[BoneNames.arm_ik(direction)][fno]
            arm_ik_bf.register = True
            arm_ik_bf.position = dest_initial_matrixes[
                BoneNames.elbow_center(direction), fno
            ].position
            dest_motion.insert_bone_frame(arm_ik_bf)

            elbow_ik_bf = dest_motion.bones[BoneNames.elbow_ik(direction)][fno]
            elbow_ik_bf.register = True
            elbow_ik_bf.position = dest_initial_matrixes[
                BoneNames.wrist_tail(direction), fno
            ].position
            dest_motion.insert_bone_frame(elbow_ik_bf)

            # 腕の捩りを取得
            arm_bf = dest_motion.bones[BoneNames.arm(direction)][fno]
            arm_x_qq, _, _, arm_yz_qq = arm_bf.rotation.separate_by_axis(
                dest_model.bones[BoneNames.arm(direction)].corrected_local_x_vector
            )

            # 捩りを除いた腕の角度を設定
            arm_bf.register = True
            arm_bf.rotation = arm_yz_qq
            dest_motion.insert_bone_frame(arm_bf)

            # 腕捩キーフレ
            arm_twist_bf = dest_motion.bones[BoneNames.arm_twist(direction)][fno]
            arm_twist_bf.register = True

            arm_twist_qq = arm_twist_bf.rotation * arm_x_qq
            arm_twist_axis = arm_twist_qq.to_axis()
            arm_twist_rad = arm_twist_qq.to_radian()
            arm_twist_sign = np.sign(arm_twist_fixed_axis.dot(arm_twist_axis))

            arm_twist_bf.rotation = MQuaternion.from_axis_angles(
                arm_twist_fixed_axis,
                arm_twist_rad * arm_twist_sign,
            )
            dest_motion.insert_bone_frame(arm_twist_bf)

            # ひじの捩りを取得
            elbow_bf = dest_motion.bones[BoneNames.elbow(direction)][fno]
            elbow_x_qq, _, _, elbow_yz_qq = elbow_bf.rotation.separate_by_axis(
                dest_model.bones[BoneNames.elbow(direction)].corrected_local_x_vector
            )

            # ひじのZは殺してY回転だけにして登録
            elbow_yz_axis = elbow_yz_qq.to_axis()
            elbow_yz_rad = elbow_yz_qq.to_radian()
            elbow_yz_sign = np.sign(elbow_y_axis.dot(elbow_yz_axis))
            if elbow_yz_sign < 0 and elbow_yz_rad > elbow_reverse_y_rad:
                # 逆ひじは一定角度以上は正ひじに直す
                elbow_yz_sign = 1

            elbow_bf.rotation = MQuaternion.from_axis_angles(
                elbow_y_axis, elbow_yz_rad * elbow_yz_sign
            )
            elbow_bf.register = True
            dest_motion.insert_bone_frame(elbow_bf)

            # 手首
            wrist_bf = dest_motion.bones[BoneNames.wrist(direction)][fno]
            wrist_x_qq, _, _, wrist_yz_qq = wrist_bf.rotation.separate_by_axis(
                dest_model.bones[BoneNames.wrist(direction)].corrected_local_x_vector
            )

            wrist_bf.rotation = wrist_yz_qq
            wrist_bf.register = True
            dest_motion.insert_bone_frame(wrist_bf)

            # 手捩
            wrist_twist_bf = dest_motion.bones[BoneNames.wrist_twist(direction)][fno]
            wrist_twist_bf.register = True

            wrist_twist_qq = wrist_x_qq * wrist_twist_bf.rotation * elbow_x_qq
            wrist_twist_axis = wrist_twist_qq.to_axis()
            wrist_twist_rad = wrist_twist_qq.to_radian()
            wrist_twist_sign = np.sign(wrist_twist_fixed_axis.dot(wrist_twist_axis))

            wrist_twist_bf.rotation = MQuaternion.from_axis_angles(
                wrist_twist_fixed_axis,
                wrist_twist_rad * wrist_twist_sign,
            )
            dest_motion.insert_bone_frame(wrist_twist_bf)

            # ■ --------------
            from datetime import datetime

            from mlib.vmd.vmd_writer import VmdWriter

            dest_ik_motion = VmdMotion()
            dest_ik_motion.append_bone_frame(arm_ik_bf)
            dest_ik_motion.append_bone_frame(elbow_ik_bf)

            dest_ik_motion.append_bone_frame(arm_bf)
            dest_ik_motion.append_bone_frame(arm_twist_bf)
            dest_ik_motion.append_bone_frame(elbow_bf)
            dest_ik_motion.append_bone_frame(wrist_twist_bf)
            dest_ik_motion.append_bone_frame(wrist_bf)

            VmdWriter(
                dest_ik_motion,
                f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_{BoneNames.arm_ik(direction)}_{fno:04d}.vmd",
                model_name="Test Model",
            ).save()
            # ■ --------------

            # IK解決する
            _, _, arm_ik_qqs = dest_motion.bones.get_ik_rotation(
                0, fno, dest_model, dest_model.bones[BoneNames.arm_ik(direction)]
            )
            _, _, elbow_ik_qqs = dest_motion.bones.get_ik_rotation(
                0, fno, dest_model, dest_model.bones[BoneNames.elbow_ik(direction)]
            )

            # 解決結果を保持
            arm_bf.rotation = arm_ik_qqs[
                dest_model.bones[BoneNames.arm(direction)].index
            ]
            dest_motion.insert_bone_frame(arm_bf)

            arm_twist_bf.rotation = arm_ik_qqs[
                dest_model.bones[BoneNames.arm_twist(direction)].index
            ]
            dest_motion.insert_bone_frame(arm_twist_bf)

            wrist_twist_bf.rotation = elbow_ik_qqs[
                dest_model.bones[BoneNames.wrist_twist(direction)].index
            ]
            dest_motion.insert_bone_frame(wrist_twist_bf)

            wrist_bf.rotation = elbow_ik_qqs[
                dest_model.bones[BoneNames.wrist(direction)].index
            ]
            dest_motion.insert_bone_frame(wrist_bf)

        # 終わったらIKボーンのキーフレを削除
        del dest_motion.bones[BoneNames.arm_ik(direction)]
        del dest_motion.bones[BoneNames.elbow_ik(direction)]

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
        sizing_display_slot = ik_model.display_slots["SIZING"]

        for direction in ("左", "右"):
            # 腕IK追加 ---------------
            arm_ik_bone = Bone(
                index=ik_model.bones[BoneNames.elbow_center(direction)].index,
                name=BoneNames.arm_ik(direction),
            )
            arm_ik_bone.parent_index = ik_model.bones[BoneNames.root()].index
            arm_ik_bone.position = MVector3D()
            arm_ik_bone.is_system = True
            arm_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            arm_ik = Ik()
            arm_ik.bone_index = ik_model.bones[BoneNames.elbow_center(direction)].index
            arm_ik.loop_count = 32
            arm_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # ひじ(動かさない)
            arm_ik_link_elbow = IkLink()
            arm_ik_link_elbow.bone_index = ik_model.bones[
                BoneNames.elbow(direction)
            ].index
            arm_ik_link_elbow.angle_limit = True
            arm_ik.links.append(arm_ik_link_elbow)

            # 腕捩
            arm_ik_link_arm_twist = IkLink()
            arm_ik_link_arm_twist.bone_index = ik_model.bones[
                BoneNames.arm_twist(direction)
            ].index
            arm_ik.links.append(arm_ik_link_arm_twist)

            for b in ik_model.bones:
                if (
                    BoneNames.arm_twist(direction) in b.name
                    and BoneNames.arm_twist(direction) != b.name
                ):
                    b.layer += 1

            # 腕(動かさない)
            arm_ik_link_arm = IkLink()
            arm_ik_link_arm.bone_index = ik_model.bones[BoneNames.arm(direction)].index
            arm_ik_link_arm.angle_limit = True
            arm_ik.links.append(arm_ik_link_arm)

            arm_ik_bone.ik = arm_ik
            ik_model.insert_bone(arm_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=arm_ik_bone.index)
            )

            # ひじIK追加 ---------------
            elbow_ik_bone = Bone(
                index=ik_model.bones[BoneNames.wrist_tail(direction)].index,
                name=BoneNames.elbow_ik(direction),
            )
            elbow_ik_bone.parent_index = ik_model.bones[BoneNames.root()].index
            elbow_ik_bone.position = MVector3D()
            elbow_ik_bone.is_system = True
            elbow_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )

            elbow_ik = Ik()
            elbow_ik.bone_index = ik_model.bones[BoneNames.wrist_tail(direction)].index
            elbow_ik.loop_count = 32
            elbow_ik.unit_rotation.radians = MVector3D(1, 0, 0)

            # 手首(動かさない)
            elbow_ik_link_wrist = IkLink()
            elbow_ik_link_wrist.bone_index = ik_model.bones[
                BoneNames.wrist(direction)
            ].index
            elbow_ik_link_wrist.angle_limit = True
            elbow_ik.links.append(elbow_ik_link_wrist)

            # 手捩
            elbow_ik_link_twist = IkLink()
            elbow_ik_link_twist.bone_index = ik_model.bones[
                BoneNames.wrist_twist(direction)
            ].index
            elbow_ik.links.append(elbow_ik_link_twist)

            for b in ik_model.bones:
                if (
                    BoneNames.wrist_twist(direction) in b.name
                    and BoneNames.wrist_twist(direction) != b.name
                ):
                    b.layer += 1

            elbow_ik_bone.ik = elbow_ik
            ik_model.insert_bone(elbow_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=elbow_ik_bone.index)
            )

        for direction in ("左", "右"):
            if "全ての親" in ik_model.bones:
                ik_model.bones["全ての親"].parent_index = ik_model.bones[
                    BoneNames.root()
                ].index

            if BoneNames.arm_ik(direction) in ik_model.bones:
                ik_model.bones[
                    BoneNames.arm_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

            if BoneNames.elbow_center(direction) in ik_model.bones:
                ik_model.bones[
                    BoneNames.elbow_center(direction)
                ].parent_index = ik_model.bones[BoneNames.elbow(direction)].index

            if BoneNames.elbow_ik(direction) in ik_model.bones:
                ik_model.bones[
                    BoneNames.elbow_ik(direction)
                ].parent_index = ik_model.bones[BoneNames.root()].index

            if BoneNames.wrist_tail(direction) in ik_model.bones:
                ik_model.bones[
                    BoneNames.wrist_tail(direction)
                ].parent_index = ik_model.bones[BoneNames.wrist(direction)].index

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

        tail_bone_names: Iterable[str] = (
            BoneNames.wrist_tail(direction),
            BoneNames.thumb_tail(direction),
            BoneNames.index_tail(direction),
            BoneNames.middle_tail(direction),
            BoneNames.ring_tail(direction),
            BoneNames.pinky_tail(direction),
        )
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
            description=f"{sizing_idx + 1}|{__(direction)}|{__('初期位置取得')}|{model_type}",
        )

        return (
            sizing_idx,
            is_src,
            direction,
            initial_matrixes,
        )
