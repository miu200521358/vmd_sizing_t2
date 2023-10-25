import os

from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlot, DisplaySlotReference
from mlib.pmx.pmx_reader import PmxReader
from mlib.utils.file_utils import get_path
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_reader import VmdReader
from mlib.vmd.vmd_writer import VmdWriter
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class IoUsecase:
    def load_motion(self, sizing_idx: int, motion_path: str, cache_motions: dict[str, VmdMotion]) -> tuple[int, str, VmdMotion, VmdMotion]:
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
        self, sizing_idx: int, model_path: str, cache_models: dict[str, PmxModel], is_src: bool
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

        original_matrixes = VmdMotion().animate_bone([0], original_model, original_model.bones.names)

        # サイジング用モデルをベースに位置などを置き換えて作り直す
        model = reader.read_by_filepath(get_path("resources/bone.pmx"))
        model.path = original_model.path
        model.model_name = original_model.name

        for bone in model.bones:
            if bone.name in original_model.bones:
                original_bone = original_model.bones[bone.name]
                bone.position = original_matrixes[0, original_bone.name].position
                tail_relative_position = original_model.bones.get_tail_relative_position(original_bone.index)
                if original_bone.is_tail_bone:
                    # 表示先
                    bone.bone_flg &= ~BoneFlg.TAIL_IS_BONE
                    bone.tail_position = (
                        original_matrixes[0, original_bone.name].global_matrix * tail_relative_position
                        - original_matrixes[0, original_bone.name].position
                    )
                else:
                    bone.tail_position = MVector3D()
                if original_bone.has_fixed_axis:
                    # 軸制限
                    bone.fixed_axis = original_bone.fixed_axis.copy()
                    bone.bone_flg |= BoneFlg.HAS_FIXED_AXIS
                if original_bone.has_local_coordinate:
                    # ローカル軸
                    bone.local_x_vector = original_bone.local_x_vector.copy()
                    bone.local_z_vector = original_bone.local_z_vector.copy()
                    bone.bone_flg |= BoneFlg.HAS_LOCAL_COORDINATE
                if original_bone.is_external_rotation or original_bone.is_external_translation:
                    # 付与親
                    bone.bone_flg |= BoneFlg.IS_EXTERNAL_ROTATION if original_bone.is_external_rotation else BoneFlg.IS_EXTERNAL_TRANSLATION
                    bone.effect_index = model.bones[original_model.bones[original_bone.effect_index].name].index
                    bone.effect_factor = original_bone.effect_factor

        # セットアップ
        self.setup_model(sizing_idx, is_src, model)

        if 10 >= logger.total_level:
            # デバッグレベルの場合、モデルを出力する
            from mlib.pmx.pmx_writer import PmxWriter

            PmxWriter(model, os.path.join(os.path.dirname(model.path), f"io_{os.path.basename(model.path)}"), include_system=True).save()

        return sizing_idx, digest, original_model, model

    def setup_model(self, sizing_idx: int, is_src: bool, model: PmxModel) -> None:
        logger.info(
            "【No.{x}】サイジング追加セットアップ({m})",
            x=sizing_idx + 1,
            m=__("モーション作成元モデル" if is_src else "サイジング先モデル"),
            decoration=MLogger.Decoration.LINE,
        )

        sizing_display_slot = DisplaySlot(name="SIZING")
        sizing_display_slot.is_system = True
        model.display_slots.append(sizing_display_slot)

        root_bone = Bone(index=0, name=BoneNames.root())
        root_bone.parent_index = -1
        root_bone.is_system = True
        root_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

        model.insert_bone(root_bone)
        sizing_display_slot.references.append(DisplaySlotReference(display_index=root_bone.index))

        for bone in model.bones:
            if bone.parent_index < 0 and bone.index != root_bone.index:
                bone.parent_index = root_bone.index

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
                # 肩中点追加 ---------------

                shoulder_center_bone = Bone(
                    index=model.bones[BoneNames.shoulder(direction)].index + 1, name=BoneNames.shoulder_center(direction)
                )
                shoulder_center_bone.parent_index = model.bones[BoneNames.shoulder(direction)].index
                shoulder_center_bone.position = (
                    model.bones[BoneNames.shoulder_root(direction)].position + model.bones[BoneNames.arm(direction)].position
                ) / 2

                shoulder_center_bone.is_system = True
                shoulder_center_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(shoulder_center_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=shoulder_center_bone.index))

                # # ひじ中点追加 ---------------

                # elbow_center_bone = Bone(index=model.bones[BoneNames.elbow(direction)].index + 1, name=BoneNames.elbow_center(direction))
                # elbow_center_bone.parent_index = model.bones[BoneNames.elbow(direction)].index
                # elbow_center_bone.position = (
                #     model.bones[BoneNames.elbow(direction)].position + model.bones[BoneNames.wrist(direction)].position
                # ) / 2

                # elbow_center_bone.is_system = True
                # elbow_center_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                # model.insert_bone(elbow_center_bone)
                # sizing_display_slot.references.append(DisplaySlotReference(display_index=elbow_center_bone.index))

                # 手首先追加 ---------------

                wrist_vector_qq = (
                    (model.bones[BoneNames.wrist(direction)].position - model.bones[BoneNames.elbow(direction)].position)
                    .to_local_matrix4x4()
                    .to_quaternion()
                )

                wrist_tail_bone = Bone(index=model.bones[BoneNames.wrist(direction)].index + 1, name=BoneNames.wrist_tail(direction))
                wrist_tail_bone.parent_index = model.bones[BoneNames.wrist(direction)].index
                wrist_tail_bone.position = (
                    model.bones[BoneNames.wrist(direction)].position
                    + (wrist_vector_qq * MVector3D(1, 0, 0))
                    # + (wrist_vector_qq * MVector3D(0, 0, 1))
                )
                wrist_tail_bone.is_system = True
                wrist_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                model.insert_bone(wrist_tail_bone)
                sizing_display_slot.references.append(DisplaySlotReference(display_index=wrist_tail_bone.index))

                # 親指先 ---------------
                if BoneNames.thumb2(direction) in model.bones:
                    thumb_tail_bone = Bone(index=model.bones[BoneNames.thumb2(direction)].index + 1, name=BoneNames.thumb_tail(direction))
                    thumb_tail_bone.parent_index = model.bones[BoneNames.thumb2(direction)].index
                    thumb_tail_bone.position = model.bones[BoneNames.thumb2(direction)].position + model.bones.get_tail_relative_position(
                        model.bones[BoneNames.thumb2(direction)].index
                    )
                    thumb_tail_bone.is_system = True
                    thumb_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                    model.insert_bone(thumb_tail_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=thumb_tail_bone.index))

                # 人指先 ---------------
                if BoneNames.index3(direction) in model.bones:
                    index_tail_bone = Bone(index=model.bones[BoneNames.index3(direction)].index + 1, name=BoneNames.index_tail(direction))
                    index_tail_bone.parent_index = model.bones[BoneNames.index3(direction)].index
                    index_tail_bone.position = model.bones[BoneNames.index3(direction)].position + model.bones.get_tail_relative_position(
                        model.bones[BoneNames.index3(direction)].index
                    )
                    index_tail_bone.is_system = True
                    index_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                    model.insert_bone(index_tail_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=index_tail_bone.index))

                # 中指先 ---------------
                if BoneNames.middle3(direction) in model.bones:
                    middle_tail_bone = Bone(
                        index=model.bones[BoneNames.middle3(direction)].index + 1, name=BoneNames.middle_tail(direction)
                    )
                    middle_tail_bone.parent_index = model.bones[BoneNames.middle3(direction)].index
                    middle_tail_bone.position = model.bones[BoneNames.middle3(direction)].position + model.bones.get_tail_relative_position(
                        model.bones[BoneNames.middle3(direction)].index
                    )
                    middle_tail_bone.is_system = True
                    middle_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                    model.insert_bone(middle_tail_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=middle_tail_bone.index))

                # 薬指先 ---------------
                if BoneNames.ring3(direction) in model.bones:
                    ring_tail_bone = Bone(index=model.bones[BoneNames.ring3(direction)].index + 1, name=BoneNames.ring_tail(direction))
                    ring_tail_bone.parent_index = model.bones[BoneNames.ring3(direction)].index
                    ring_tail_bone.position = model.bones[BoneNames.ring3(direction)].position + model.bones.get_tail_relative_position(
                        model.bones[BoneNames.ring3(direction)].index
                    )
                    ring_tail_bone.is_system = True
                    ring_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                    model.insert_bone(ring_tail_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=ring_tail_bone.index))

                # 小指先 ---------------
                if BoneNames.pinky3(direction) in model.bones:
                    pinky_tail_bone = Bone(index=model.bones[BoneNames.pinky3(direction)].index + 1, name=BoneNames.pinky_tail(direction))
                    pinky_tail_bone.parent_index = model.bones[BoneNames.pinky3(direction)].index
                    pinky_tail_bone.position = model.bones[BoneNames.pinky3(direction)].position + model.bones.get_tail_relative_position(
                        model.bones[BoneNames.pinky3(direction)].index
                    )
                    pinky_tail_bone.is_system = True
                    pinky_tail_bone.bone_flg |= BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE

                    model.insert_bone(pinky_tail_bone)
                    sizing_display_slot.references.append(DisplaySlotReference(display_index=pinky_tail_bone.index))

        for direction in ("左", "右"):
            if BoneNames.shoulder_p(direction) in model.bones:
                model.bones[BoneNames.shoulder_p(direction)].parent_index = model.bones[BoneNames.shoulder_root(direction)].index
            if BoneNames.shoulder(direction) in model.bones:
                model.bones[BoneNames.shoulder(direction)].parent_index = model.bones[BoneNames.shoulder_root(direction)].index
            # if BoneNames.hand_twist(direction) in model.bones:
            #     model.bones[BoneNames.hand_twist(direction)].parent_index = model.bones[BoneNames.elbow_center(direction)].index
            # elif BoneNames.wrist(direction) in model.bones:
            #     model.bones[BoneNames.wrist(direction)].parent_index = model.bones[BoneNames.elbow_center(direction)].index
            if BoneNames.wrist_tail(direction) in model.bones:
                model.bones[BoneNames.wrist_tail(direction)].parent_index = model.bones[BoneNames.wrist(direction)].index
            if BoneNames.thumb_tail(direction) in model.bones:
                model.bones[BoneNames.thumb_tail(direction)].parent_index = model.bones[BoneNames.thumb2(direction)].index
            if BoneNames.index_tail(direction) in model.bones:
                model.bones[BoneNames.index_tail(direction)].parent_index = model.bones[BoneNames.index3(direction)].index
            if BoneNames.middle_tail(direction) in model.bones:
                model.bones[BoneNames.middle_tail(direction)].parent_index = model.bones[BoneNames.middle3(direction)].index
            if BoneNames.ring_tail(direction) in model.bones:
                model.bones[BoneNames.ring_tail(direction)].parent_index = model.bones[BoneNames.ring3(direction)].index
            if BoneNames.pinky_tail(direction) in model.bones:
                model.bones[BoneNames.pinky_tail(direction)].parent_index = model.bones[BoneNames.pinky3(direction)].index

        model.setup()

    def load_model_no_copy(self, sizing_idx: int, model_path: str, cache_models: dict[str, PmxModel]) -> tuple[int, str, PmxModel]:
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
        logger.info("【No.{i}】結果保存", i=sizing_idx + 1, decoration=MLogger.Decoration.LINE)

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        VmdWriter(motion, output_path, dest_model.name).save()

        logger.info("【No.{i}】結果保存成功\n{p}", i=sizing_idx + 1, p=output_path, decoration=MLogger.Decoration.BOX)
