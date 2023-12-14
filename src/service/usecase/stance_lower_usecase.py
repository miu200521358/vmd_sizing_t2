import os
from math import radians
from typing import Optional

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


class StanceLowerUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        is_stance_lower: bool,
        show_message: bool = False,
    ) -> bool:
        if not src_model or not dest_model or (not is_stance_lower):
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        """下半身"""
        if {"下半身"} - set(src_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{x}】モーション作成元モデルに下半身ボーンがないため、下半身補正をスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if {"下半身"} - set(dest_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{x}】サイジング先モデルに下半身ボーンがないため、下半身補正をスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_stance_lower(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        src_initial_matrixes: VmdBoneFrameTrees,
        dest_initial_matrixes: VmdBoneFrameTrees,
    ) -> tuple[int, str, VmdMotion]:
        logger.info(
            "【No.{i}】下半身補正",
            i=sizing_idx + 1,
            decoration=MLogger.Decoration.LINE,
        )

        lower_ratio = (
            dest_model.bones[BoneNames.lower()].position.distance(
                dest_model.bones[BoneNames.leg("右")].position
            )
            + dest_model.bones[BoneNames.lower()].position.distance(
                dest_model.bones[BoneNames.leg("左")].position
            )
        ) / (
            src_model.bones[BoneNames.lower()].position.distance(
                src_model.bones[BoneNames.leg("右")].position
            )
            + src_model.bones[BoneNames.lower()].position.distance(
                src_model.bones[BoneNames.leg("左")].position
            )
        )

        leg_ratio = (
            dest_model.bones[BoneNames.knee("右")].position.distance(
                dest_model.bones[BoneNames.leg("右")].position
            )
            + dest_model.bones[BoneNames.knee("左")].position.distance(
                dest_model.bones[BoneNames.leg("左")].position
            )
        ) / (
            src_model.bones[BoneNames.knee("右")].position.distance(
                src_model.bones[BoneNames.leg("右")].position
            )
            + src_model.bones[BoneNames.knee("左")].position.distance(
                src_model.bones[BoneNames.leg("左")].position
            )
        )

        # 足の長さ＋下半身から足中心までの長さの比率
        # lower_leg_ratio = lower_ratio * leg_ratio
        lower_leg_ratio = (lower_ratio + leg_ratio) / 2

        for fidx, fno in enumerate(dest_initial_matrixes.indexes):
            logger.count(
                "【No.{x}】下半身補正 - 準備",
                x=sizing_idx + 1,
                index=fidx,
                total_index_count=len(dest_initial_matrixes.indexes),
                display_block=1000,
            )

            # --------------

            # キーフレを登録しておく
            lower_bf = dest_motion.bones[BoneNames.lower()][fno]
            lower_bf.register = True
            dest_motion.insert_bone_frame(lower_bf)

            for direction in ("右", "左"):
                leg_bf = dest_motion.bones[BoneNames.leg(direction)][fno]
                leg_bf.register = True
                leg_bf.rotation = src_initial_matrixes[
                    BoneNames.leg(direction), fno
                ].frame_fk_rotation
                dest_motion.insert_bone_frame(leg_bf)

                lower_ik_bf = dest_motion.bones[BoneNames.lower_ik(direction)][fno]
                lower_ik_bf.register = True
                dest_motion.insert_bone_frame(lower_ik_bf)

                lower_tail_bf = dest_motion.bones[BoneNames.lower_tail(direction)][fno]
                lower_tail_bf.register = True
                dest_motion.insert_bone_frame(lower_tail_bf)

                leg_tail_bf = dest_motion.bones[BoneNames.leg_tail(direction)][fno]
                leg_tail_bf.register = True
                dest_motion.insert_bone_frame(leg_tail_bf)

        for fidx, fno in enumerate(dest_initial_matrixes.indexes):
            logger.count(
                "【No.{x}】下半身補正",
                x=sizing_idx + 1,
                index=fidx,
                total_index_count=len(dest_initial_matrixes.indexes),
                display_block=100,
            )

            # --------------

            lower_qqs: list[MQuaternion] = []

            for direction in ("右", "左"):
                # 下半身先
                lower_tail_bf = dest_motion.bones[BoneNames.lower_tail(direction)][fno]
                lower_tail_bf.rotation = dest_initial_matrixes[
                    BoneNames.lower(), fno
                ].frame_fk_rotation
                dest_motion.insert_bone_frame(lower_tail_bf)

                # 足先
                leg_tail_bf = dest_motion.bones[BoneNames.leg_tail(direction)][fno]
                leg_tail_bf.rotation = src_initial_matrixes[
                    BoneNames.leg(direction), fno
                ].frame_fk_rotation
                dest_motion.insert_bone_frame(leg_tail_bf)

                # 下半身IK
                lower_ik_bf = dest_motion.bones[BoneNames.lower_ik(direction)][fno]
                lower_ik_bf.position = dest_initial_matrixes[
                    BoneNames.lower(), fno
                ].position + (
                    (
                        src_initial_matrixes[BoneNames.knee(direction), fno].position
                        - src_initial_matrixes[BoneNames.lower(), fno].position
                    )
                    * lower_leg_ratio
                )
                dest_motion.insert_bone_frame(lower_ik_bf)

                # # ■ --------------
                # from datetime import datetime

                # from mlib.vmd.vmd_writer import VmdWriter

                # VmdWriter(
                #     dest_motion,
                #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_下半身{direction}_{fno:04d}.vmd",
                #     model_name="Test Model",
                # ).save()
                # # ■ --------------

                # IK解決する
                _, _, ik_qqs = dest_motion.bones.get_ik_rotation(
                    fidx,
                    fno,
                    dest_model,
                    dest_model.bones[BoneNames.lower_ik(direction)],
                )

                lower_qqs.append(
                    ik_qqs[dest_model.bones[BoneNames.lower_tail(direction)].index]
                )

                # # ■ --------------

                # # 下半身先
                # lower_tail_bf = dest_motion.bones[BoneNames.lower_tail(direction)][fno]
                # lower_tail_bf.rotation = ik_qqs[
                #     dest_model.bones[BoneNames.lower_tail(direction)].index
                # ]
                # dest_motion.insert_bone_frame(lower_tail_bf)

                # # 足先
                # leg_tail_bf = dest_motion.bones[BoneNames.leg_tail(direction)][fno]
                # leg_tail_bf.rotation = ik_qqs[
                #     dest_model.bones[BoneNames.leg_tail(direction)].index
                # ]
                # dest_motion.insert_bone_frame(leg_tail_bf)

                # VmdWriter(
                #     dest_motion,
                #     f"E:/MMD/サイジング/足IK/IK_step/{datetime.now():%Y%m%d_%H%M%S_%f}_下半身{direction}結果_{fno:04d}.vmd",
                #     model_name="Test Model",
                # ).save()
                # # ■ --------------

            # 両足の中間を採用する
            lower_bf = dest_motion.bones[BoneNames.lower()][fno]
            lower_bf.rotation = MQuaternion.slerp(lower_qqs[0], lower_qqs[1], 0.5)
            dest_motion.insert_bone_frame(lower_bf)

        # 終わったらIKボーンのキーフレを削除
        del dest_motion.bones[BoneNames.lower_ik("右")]
        del dest_motion.bones[BoneNames.lower_ik("左")]
        del dest_motion.bones[BoneNames.lower_tail("右")]
        del dest_motion.bones[BoneNames.lower_tail("左")]
        del dest_motion.bones[BoneNames.leg_tail("右")]
        del dest_motion.bones[BoneNames.leg_tail("左")]

        return sizing_idx, dest_motion

    def setup_model_ik(
        self,
        sizing_idx: int,
        is_src: bool,
        model: PmxModel,
    ) -> tuple[int, bool, PmxModel]:
        logger.info(
            "【No.{x}】下半身補正：追加IKセットアップ({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        ik_model = model.copy()
        sizing_display_slot = ik_model.display_slots[BoneNames.sizing_display_slot()]

        for direction in ("右", "左"):
            # 下半身先(位置は下半身と同じ) ---------------
            lower_tail_bone = Bone(
                index=ik_model.bones[BoneNames.waist()].index + 1,
                name=BoneNames.lower_tail(direction),
            )
            lower_tail_bone.position = model.bones[BoneNames.lower()].position.copy()
            lower_tail_bone.is_system = True
            lower_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
            )
            ik_model.insert_bone(lower_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=lower_tail_bone.index)
            )

            # 足先(位置は足と同じ) ---------------
            leg_tail_bone = Bone(
                index=lower_tail_bone.index + 1,
                name=BoneNames.leg_tail(direction),
            )
            leg_tail_bone.position = model.bones[
                BoneNames.leg(direction)
            ].position.copy()
            leg_tail_bone.is_system = True
            leg_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
                | BoneFlg.TAIL_IS_BONE
            )
            ik_model.insert_bone(leg_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=leg_tail_bone.index)
            )

            # ひざ先(位置はひざと同じ) ---------------
            knee_tail_bone = Bone(
                index=leg_tail_bone.index + 1,
                name=BoneNames.knee_tail(direction),
            )
            knee_tail_bone.position = model.bones[
                BoneNames.knee(direction)
            ].position.copy()
            knee_tail_bone.is_system = True
            knee_tail_bone.bone_flg |= (
                BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(knee_tail_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=knee_tail_bone.index)
            )

            # 下半身IK ---------------
            lower_ik_bone = Bone(
                index=knee_tail_bone.index + 1,
                name=BoneNames.lower_ik(direction),
            )
            lower_ik_bone.position = MVector3D()
            lower_ik_bone.is_system = True
            lower_ik_bone.bone_flg |= (
                BoneFlg.IS_IK
                | BoneFlg.CAN_TRANSLATE
                | BoneFlg.CAN_ROTATE
                | BoneFlg.CAN_MANIPULATE
                | BoneFlg.IS_VISIBLE
            )
            ik_model.insert_bone(lower_ik_bone)
            sizing_display_slot.references.append(
                DisplaySlotReference(display_index=lower_ik_bone.index)
            )

            # 下半身IKの構成
            lower_ik = Ik()
            lower_ik.bone_index = knee_tail_bone.index
            lower_ik.loop_count = 20
            lower_ik.unit_rotation.degrees = MVector3D(180, 0, 0)

            # 足先
            lower_ik_leg = IkLink()
            lower_ik_leg.bone_index = leg_tail_bone.index
            lower_ik.links.append(lower_ik_leg)

            # 下半身先
            lower_ik_lower = IkLink()
            lower_ik_lower.bone_index = lower_tail_bone.index
            lower_ik.links.append(lower_ik_lower)

            lower_ik_bone.ik = lower_ik

        ik_model.bones[BoneNames.root()].parent_index = -1

        for direction in ("右", "左"):
            lower_tail_bone = ik_model.bones[BoneNames.lower_tail(direction)]
            leg_tail_bone = ik_model.bones[BoneNames.leg_tail(direction)]
            knee_tail_bone = ik_model.bones[BoneNames.knee_tail(direction)]
            lower_ik_bone = ik_model.bones[BoneNames.lower_ik(direction)]

            lower_tail_bone.parent_index = ik_model.bones[BoneNames.waist()].index
            lower_tail_bone.tail_index = leg_tail_bone.index
            leg_tail_bone.parent_index = lower_tail_bone.index
            leg_tail_bone.tail_index = knee_tail_bone.index
            knee_tail_bone.parent_index = leg_tail_bone.index
            lower_ik_bone.parent_index = ik_model.bones[BoneNames.root()].index

        ik_model.setup()

        if 10 >= logger.total_level:
            # デバッグレベルの場合、モデルを出力する
            from mlib.pmx.pmx_writer import PmxWriter

            PmxWriter(
                ik_model,
                os.path.join(
                    os.path.dirname(ik_model.path),
                    f"lower_{os.path.basename(ik_model.path)}",
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
    ) -> tuple[int, bool, str, VmdBoneFrameTrees]:
        model_type = __("作成元モデル" if is_src else "サイジング先モデル")

        logger.info(
            "【No.{x}】初期位置取得({m})",
            x=sizing_idx + 1,
            m=model_type,
            decoration=MLogger.Decoration.LINE,
        )

        tail_bone_names: list[str] = [
            BoneNames.ankle("右"),
            BoneNames.ankle("左"),
        ]

        fnos_set = (
            {0}
            | set(motion.bones[BoneNames.lower()].register_indexes)
            | set(motion.bones[BoneNames.leg("右")].register_indexes)
            | set(motion.bones[BoneNames.leg("左")].register_indexes)
        )

        fnos = sorted(fnos_set)

        initial_matrixes = motion.animate_bone(
            fnos,
            model,
            tail_bone_names,
            out_fno_log=True,
            description=f"{sizing_idx + 1}|{__('初期位置取得')}|{model_type}",
        )

        return (
            sizing_idx,
            is_src,
            initial_matrixes,
        )
