import os

import numpy as np
from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import DisplaySlot
from mlib.pmx.pmx_reader import PmxReader
from mlib.utils.file_utils import get_path
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_reader import VmdReader
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from mlib.vmd.vmd_writer import VmdWriter
from service.usecase.align_arm_usecase import ARM_BONE_NAMES
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class IoUsecase:
    def load_motion(
        self, sizing_idx: int, motion_path: str, cache_motions: dict[str, VmdMotion]
    ) -> tuple[int, str, VmdMotion, VmdMotion]:
        """モーションの読み込み"""
        logger.info(
            "【No.{x}】モーション読み込み",
            x=sizing_idx + 1,
            decoration=MLogger.Decoration.LINE,
        )

        reader = VmdReader()
        digest = reader.read_hash_by_filepath(motion_path)
        original_motion = cache_motions.get(digest)

        if not original_motion:
            original_motion = reader.read_by_filepath(motion_path)

        return sizing_idx, digest, original_motion, original_motion.copy()

    def load_model(
        self,
        sizing_idx: int,
        model_path: str,
        cache_models: dict[str, PmxModel],
        is_src: bool,
    ) -> tuple[int, str, PmxModel, PmxModel]:
        """モデルの読み込み"""
        logger.info(
            "【No.{x}】モデル読み込み({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        reader = PmxReader()
        digest = reader.read_hash_by_filepath(model_path)
        original_model = cache_models.get(digest)

        if not original_model:
            original_model = reader.read_by_filepath(model_path)
            original_model.setup()

        original_matrixes = VmdMotion().animate_bone(
            [0], original_model, original_model.bones.names
        )

        # サイジング用モデルをベースに位置などを置き換えて作り直す
        model = reader.read_by_filepath(get_path("resources/bone.pmx"))
        model.path = original_model.path
        model.model_name = original_model.name

        for bone in model.bones:
            if bone.name in original_model.bones:
                original_bone = original_model.bones[bone.name]
                bone.position = original_matrixes[original_bone.name, 0].position
                tail_relative_position = (
                    original_model.bones.get_tail_relative_position(original_bone.index)
                )
                if bone.tail_index in model.bones:
                    # 表示先ボーンがある場合はそこに繋ぐ
                    bone.bone_flg |= BoneFlg.TAIL_IS_BONE
                    bone.tail_index = model.bones[
                        model.bones[bone.tail_index].name
                    ].index
                else:
                    # 表示先ボーンがない場合、相対位置に変更
                    bone.bone_flg &= ~BoneFlg.TAIL_IS_BONE
                    bone.tail_position = tail_relative_position

                if original_bone.has_fixed_axis:
                    # 軸制限
                    bone.fixed_axis = original_bone.fixed_axis.copy()
                    bone.bone_flg |= BoneFlg.HAS_FIXED_AXIS
                if original_bone.has_local_coordinate:
                    # ローカル軸
                    bone.local_x_vector = original_bone.local_x_vector.copy()
                    bone.local_z_vector = original_bone.local_z_vector.copy()
                    bone.bone_flg |= BoneFlg.HAS_LOCAL_COORDINATE
                if (
                    original_bone.is_external_rotation
                    or original_bone.is_external_translation
                ) and original_model.bones[
                    original_bone.effect_index
                ].name in model.bones:
                    # 付与親
                    bone.bone_flg |= (
                        BoneFlg.IS_EXTERNAL_ROTATION
                        if original_bone.is_external_rotation
                        else BoneFlg.IS_EXTERNAL_TRANSLATION
                    )
                    bone.effect_index = model.bones[
                        original_model.bones[original_bone.effect_index].name
                    ].index
                    bone.effect_factor = original_bone.effect_factor
            else:
                # ボーンが存在しなかった場合、フラグを立てておく
                bone.bone_flg |= BoneFlg.NOTHING

        if BoneNames.groove() not in original_model.bones:
            # グルーブがなかったらセンターから再計算
            model.bones[BoneNames.groove()].bone_flg |= BoneFlg.NOTHING
            model.bones[BoneNames.groove()].position = MVector3D(
                0, original_model.bones[BoneNames.center()].position.y * 1.05, 0
            )

        if BoneNames.waist() not in original_model.bones:
            # 腰がなかったら上半身と下半身の間
            model.bones[BoneNames.waist()].bone_flg |= BoneFlg.NOTHING
            model.bones[BoneNames.waist()].position = (
                model.bones[BoneNames.upper()].position
                + model.bones[BoneNames.lower()].position
            ) / 2

        if BoneNames.upper2() not in original_model.bones:
            # 上半身2がなかったら上半身と首の間
            model.bones[BoneNames.upper2()].bone_flg |= BoneFlg.NOTHING
            model.bones[BoneNames.upper2()].position = (
                model.bones[BoneNames.upper()].position
                + model.bones[BoneNames.neck()].position
            ) / 2

        if BoneNames.upper3() not in original_model.bones:
            # 上半身3がなかったら上半身2と首の間
            model.bones[BoneNames.upper3()].bone_flg |= BoneFlg.NOTHING
            model.bones[BoneNames.upper3()].position = (
                model.bones[BoneNames.upper2()].position
                + model.bones[BoneNames.neck()].position
            ) / 2

        for direction in ("右", "左"):
            # 肩Pは肩にあわせる
            model.bones[
                BoneNames.shoulder_p(direction)
            ].position = original_model.bones[
                BoneNames.shoulder(direction)
            ].position.copy()
            # 肩Cは腕にあわせる
            model.bones[
                BoneNames.shoulder_c(direction)
            ].position = original_model.bones[BoneNames.arm(direction)].position.copy()
            # 肩中点は肩根元と腕の間
            model.bones[BoneNames.shoulder_center(direction)].position = (
                model.bones[BoneNames.shoulder_root(direction)].position
                + model.bones[BoneNames.arm(direction)].position
            ) / 2

            # 捩りボーンがなかったら中間に配置
            if BoneNames.arm_twist(direction) not in original_model.bones:
                model.bones[BoneNames.arm_twist(direction)].bone_flg |= BoneFlg.NOTHING

                model.bones[BoneNames.arm_twist(direction)].position = (
                    model.bones[BoneNames.elbow(direction)].position
                    + model.bones[BoneNames.arm(direction)].position
                ) / 2
                model.bones[BoneNames.arm_twist(direction)].fixed_axis = (
                    model.bones[BoneNames.elbow(direction)].position
                    - model.bones[BoneNames.arm(direction)].position
                ).normalized()
            if BoneNames.wrist_twist(direction) not in original_model.bones:
                model.bones[
                    BoneNames.wrist_twist(direction)
                ].bone_flg |= BoneFlg.NOTHING

                model.bones[BoneNames.wrist_twist(direction)].position = (
                    model.bones[BoneNames.wrist(direction)].position
                    + model.bones[BoneNames.elbow(direction)].position
                ) / 2
                model.bones[BoneNames.wrist_twist(direction)].fixed_axis = (
                    model.bones[BoneNames.wrist(direction)].position
                    - model.bones[BoneNames.elbow(direction)].position
                ).normalized()

            if BoneNames.thumb0(direction) not in original_model.bones:
                model.bones[BoneNames.thumb0(direction)].bone_flg |= BoneFlg.NOTHING

                # 親指0がなかったら親指1と手首の間
                model.bones[BoneNames.thumb0(direction)].position = MVector3D(
                    *np.average(
                        [
                            model.bones[BoneNames.wrist(direction)].position.vector,
                            model.bones[BoneNames.thumb1(direction)].position.vector,
                        ],
                        weights=[0.2, 0.8],
                        axis=0,
                    )
                )

            # 手首先はひじと手首のベクトル
            model.bones[BoneNames.wrist_tail(direction)].position = (
                original_model.bones[BoneNames.wrist(direction)].position
                + original_model.bones[
                    BoneNames.elbow(direction)
                ].tail_relative_position.normalized()
            )
            # 指先は指末端ボーンの表示先からコピー
            model.bones[BoneNames.thumb_tail(direction)].position = (
                original_model.bones[BoneNames.thumb2(direction)].position
                + original_model.bones[
                    BoneNames.thumb2(direction)
                ].tail_relative_position
            )
            model.bones[BoneNames.index_tail(direction)].position = (
                original_model.bones[BoneNames.index3(direction)].position
                + original_model.bones[
                    BoneNames.index3(direction)
                ].tail_relative_position.copy()
            )
            model.bones[BoneNames.middle_tail(direction)].position = (
                original_model.bones[BoneNames.middle3(direction)].position
                + original_model.bones[
                    BoneNames.middle3(direction)
                ].tail_relative_position.copy()
            )
            model.bones[BoneNames.ring_tail(direction)].position = (
                original_model.bones[BoneNames.ring3(direction)].position
                + original_model.bones[
                    BoneNames.ring3(direction)
                ].tail_relative_position.copy()
            )
            model.bones[BoneNames.pinky_tail(direction)].position = (
                original_model.bones[BoneNames.pinky3(direction)].position
                + original_model.bones[
                    BoneNames.pinky3(direction)
                ].tail_relative_position.copy()
            )

            # 腰キャンセルボーンは足からコピー
            model.bones[
                BoneNames.waist_cancel(direction)
            ].position = original_model.bones[BoneNames.leg(direction)].position.copy()

            # Dボーンは付与親からコピー
            model.bones[BoneNames.leg_d(direction)].position = original_model.bones[
                BoneNames.leg(direction)
            ].position.copy()
            model.bones[BoneNames.knee_d(direction)].position = original_model.bones[
                BoneNames.knee(direction)
            ].position.copy()
            model.bones[BoneNames.ankle_d(direction)].position = original_model.bones[
                BoneNames.ankle(direction)
            ].position.copy()

            if BoneNames.leg_ik_parent(direction) not in original_model.bones:
                model.bones[
                    BoneNames.leg_ik_parent(direction)
                ].bone_flg |= BoneFlg.NOTHING

                # 足IK親がなかったら足IKから再計算
                model.bones[
                    BoneNames.leg_ik_parent(direction)
                ].position = original_model.bones[
                    BoneNames.leg_ik(direction)
                ].position * MVector3D(
                    1, 0, 1
                )
            if BoneNames.toe(direction) not in original_model.bones:
                # つま先がなかったらつま先IKのターゲットボーンの位置
                model.bones[BoneNames.toe(direction)].position = original_model.bones[
                    original_model.bones[BoneNames.toe_ik(direction)].ik.bone_index
                ].position.copy()
            if BoneNames.toe_ex(direction) not in original_model.bones:
                model.bones[BoneNames.toe_ex(direction)].bone_flg |= BoneFlg.NOTHING

                # 足先EXがなかったらつま先と足首の間
                model.bones[BoneNames.toe_ex(direction)].position = MVector3D(
                    *np.average(
                        [
                            model.bones[BoneNames.ankle(direction)].position.vector,
                            model.bones[BoneNames.toe(direction)].position.vector,
                        ],
                        weights=[0.2, 0.8],
                        axis=0,
                    )
                )

            # ひじ先
            model.bones[BoneNames.elbow_tail(direction)].position = model.bones[
                BoneNames.wrist(direction)
            ].position.copy()
            # ひじ垂線
            elbow_vertical_relative_position = (
                (
                    model.bones[BoneNames.elbow(direction)].position
                    - model.bones[BoneNames.arm(direction)].position
                )
                .cross(
                    MVector3D(
                        0,
                        -1
                        * np.sign(model.bones[BoneNames.wrist(direction)].position.x),
                        0,
                    )
                )
                .normalized()
            )
            model.bones[BoneNames.elbow_vertical(direction)].position = (
                model.bones[BoneNames.elbow_tail(direction)].position
                + elbow_vertical_relative_position
            )

            # 手首垂線
            wrist_vertical_relative_position = (
                (
                    model.bones[BoneNames.wrist(direction)].position
                    - model.bones[BoneNames.elbow(direction)].position
                )
                .cross(
                    MVector3D(
                        0,
                        1 * np.sign(model.bones[BoneNames.wrist(direction)].position.x),
                        0,
                    )
                )
                .normalized()
            )
            model.bones[BoneNames.wrist_vertical(direction)].position = (
                model.bones[BoneNames.wrist(direction)].position
                + wrist_vertical_relative_position
            )
            model.bones[
                BoneNames.wrist_vertical(direction)
            ].tail_position = -wrist_vertical_relative_position

        for bone in model.bones:
            if (
                bone.parent_index < 0
                and bone.index != model.bones[BoneNames.root()].index
            ):
                bone.parent_index = model.bones[BoneNames.root()].index

        sizing_display_slot = DisplaySlot(name=BoneNames.sizing_display_slot())
        sizing_display_slot.is_system = True
        model.display_slots.append(sizing_display_slot)

        # セットアップ
        model.setup()

        if 10 >= logger.total_level:
            # デバッグレベルの場合、モデルを出力する
            from mlib.pmx.pmx_writer import PmxWriter

            PmxWriter(
                model,
                os.path.join(
                    os.path.dirname(model.path), f"io_{os.path.basename(model.path)}"
                ),
                include_system=True,
            ).save()

        return sizing_idx, digest, original_model, model

    def load_model_no_copy(
        self, sizing_idx: int, model_path: str, cache_models: dict[str, PmxModel]
    ) -> tuple[int, str, PmxModel]:
        """モデルの読み込み"""
        reader = PmxReader()
        digest = reader.read_hash_by_filepath(model_path)
        original_model = cache_models.get(digest)

        if not original_model:
            original_model = reader.read_by_filepath(model_path)

        return sizing_idx, digest, original_model

    def save(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        motion: VmdMotion,
        output_path: str,
    ) -> None:
        """結果保存"""
        logger.info(
            "【No.{i}】結果保存", i=sizing_idx + 1, decoration=MLogger.Decoration.LINE
        )

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        VmdWriter(motion, output_path, dest_model.name).save()

        logger.info(
            "【No.{i}】結果保存成功\n{p}",
            i=sizing_idx + 1,
            p=output_path,
            decoration=MLogger.Decoration.BOX,
        )

    def get_src_matrixes(
        self,
        sizing_idx: int,
        model: PmxModel,
        motion: VmdMotion,
        is_align_arm_middle: bool,
    ) -> tuple[int, VmdBoneFrameTrees]:
        model_type = __("作成元モデル")

        logger.info(
            "【No.{x}】初期位置取得({m})",
            x=sizing_idx + 1,
            m=model_type,
            decoration=MLogger.Decoration.LINE,
        )

        tail_bone_names = (
            [
                BoneNames.toe_ik("右"),
                BoneNames.toe_ik("左"),
                BoneNames.wrist_tail("右"),
                BoneNames.wrist_tail("左"),
            ]
            + list(ARM_BONE_NAMES)
            + BoneNames.fingers("右")
            + BoneNames.fingers("左")
        )

        # 必要なフレーム取得
        if is_align_arm_middle:
            # 全フレーム取得
            fnos = list(range(motion.max_fno + 1))
        else:
            fnos_set = {0}
            for bone_name in tail_bone_names:
                fnos_set |= set(motion.bones[bone_name].register_indexes)
            fnos = sorted(fnos_set)

        # ボーン行列取得
        initial_matrixes = motion.animate_bone(
            fnos,
            model,
            tail_bone_names,
            out_fno_log=True,
            description=f"{sizing_idx + 1}|{__('初期位置取得')}|{model_type}",
        )

        return sizing_idx, initial_matrixes
