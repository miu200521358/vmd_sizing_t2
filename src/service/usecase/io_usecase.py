import os

from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, Ik, IkLink
from mlib.pmx.pmx_reader import PmxReader
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_reader import VmdReader
from mlib.vmd.vmd_writer import VmdWriter

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class IoUsecase:
    def load_motion(self, sizing_idx: int, motion_path: str, cache_motions: dict[str, VmdMotion]) -> tuple[int, str, VmdMotion, VmdMotion]:
        """モーションの読み込み"""
        reader = VmdReader()
        digest = reader.read_hash_by_filepath(motion_path)
        original_motion = cache_motions.get(digest)

        if not original_motion:
            original_motion = reader.read_by_filepath(motion_path)

        return sizing_idx, digest, original_motion, original_motion.copy()

    def load_model(self, sizing_idx: int, model_path: str, cache_models: dict[str, PmxModel]) -> tuple[int, str, PmxModel, PmxModel]:
        """モデルの読み込み"""
        reader = PmxReader()
        digest = reader.read_hash_by_filepath(model_path)
        original_model = cache_models.get(digest)

        if not original_model:
            original_model = reader.read_by_filepath(model_path)

        logger.info("モデルセットアップ：サイジング用", decoration=MLogger.Decoration.LINE)

        model = original_model.copy()
        local_y_vector = MVector3D(0, -1, 0)

        for direction in ("左", "右"):
            if not ({f"{direction}肩根元", f"{direction}肩", f"{direction}腕", f"{direction}手首"} - set(model.bones.names)):
                shoulder_bone_parent_name = model.bones[model.bones[f"{direction}肩"].parent_index].name

                # 肩+
                shoulder_plus_bone = Bone(index=model.bones[f"{direction}肩"].index, name=f"【SIZING】{direction}肩+")
                shoulder_plus_bone.parent_index = (
                    model.bones[f"{direction}肩P"].index if f"{direction}肩P" in model.bones else f"{direction}肩根元"
                )
                shoulder_plus_bone.position = model.bones[f"{direction}肩"].position.copy()
                shoulder_plus_bone.is_system = True
                shoulder_plus_bone.bone_flg |= BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                model.insert_bone(shoulder_plus_bone)

                # 肩の付与親
                shoulder_bone = model.bones[f"{direction}肩"]
                shoulder_bone.bone_flg |= BoneFlg.IS_EXTERNAL_ROTATION
                shoulder_bone.effect_index = shoulder_plus_bone.index
                shoulder_bone.effect_factor = 1.0

                # 腕+
                arm_plus_bone = Bone(index=shoulder_plus_bone.index + 1, name=f"【SIZING】{direction}腕+")
                arm_plus_bone.parent_index = shoulder_plus_bone.index
                arm_plus_bone.position = model.bones[f"{direction}腕"].position.copy()
                arm_plus_bone.is_system = True
                arm_plus_bone.bone_flg |= BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                model.insert_bone(arm_plus_bone)

                # 肩+の表示先
                shoulder_plus_bone.bone_flg |= BoneFlg.TAIL_IS_BONE | BoneFlg.HAS_LOCAL_COORDINATE
                shoulder_plus_bone.tail_index = arm_plus_bone.index
                shoulder_plus_bone.local_x_vector = (arm_plus_bone.position - shoulder_plus_bone.position).normalized()
                shoulder_plus_bone.local_z_vector = local_y_vector.cross(shoulder_plus_bone.local_x_vector).normalized()

                # 腕の付与親
                arm_bone = model.bones[f"{direction}腕"]
                arm_bone.bone_flg |= BoneFlg.IS_EXTERNAL_ROTATION
                arm_bone.effect_index = arm_plus_bone.index
                arm_bone.effect_factor = 1.0

                # ひじ+
                elbow_plus_bone = Bone(index=arm_plus_bone.index + 1, name=f"【SIZING】{direction}ひじ+")
                elbow_plus_bone.parent_index = arm_plus_bone.index
                elbow_plus_bone.position = model.bones[f"{direction}ひじ"].position.copy()
                elbow_plus_bone.is_system = True
                elbow_plus_bone.bone_flg |= BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                model.insert_bone(elbow_plus_bone)

                # 腕+の表示先
                arm_plus_bone.bone_flg |= BoneFlg.TAIL_IS_BONE | BoneFlg.HAS_LOCAL_COORDINATE
                arm_plus_bone.tail_index = elbow_plus_bone.index
                arm_plus_bone.local_x_vector = (elbow_plus_bone.position - arm_plus_bone.position).normalized()
                arm_plus_bone.local_z_vector = local_y_vector.cross(arm_plus_bone.local_x_vector).normalized()

                # ひじの付与親
                elbow_bone = model.bones[f"{direction}ひじ"]
                elbow_bone.bone_flg |= BoneFlg.IS_EXTERNAL_ROTATION
                elbow_bone.effect_index = elbow_plus_bone.index
                elbow_bone.effect_factor = 1.0

                # 手首+
                wrist_plus_bone = Bone(index=elbow_plus_bone.index + 1, name=f"【SIZING】{direction}手首+")
                wrist_plus_bone.parent_index = elbow_plus_bone.index
                wrist_plus_bone.position = model.bones[f"{direction}手首"].position.copy()
                wrist_plus_bone.is_system = True
                wrist_plus_bone.bone_flg |= BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE | BoneFlg.HAS_LOCAL_COORDINATE
                wrist_plus_bone.tail_position = MVector3D(0, 0.001, 0)
                wrist_plus_bone.local_x_vector = model.bones[f"{direction}手首"].local_x_vector.copy()
                wrist_plus_bone.local_z_vector = model.bones[f"{direction}手首"].local_z_vector.copy()
                model.insert_bone(wrist_plus_bone)

                # ひじ+の表示先
                elbow_plus_bone.bone_flg |= BoneFlg.TAIL_IS_BONE | BoneFlg.HAS_LOCAL_COORDINATE
                elbow_plus_bone.tail_index = wrist_plus_bone.index
                elbow_plus_bone.local_x_vector = (wrist_plus_bone.position - elbow_plus_bone.position).normalized()
                elbow_plus_bone.local_z_vector = local_y_vector.cross(elbow_plus_bone.local_x_vector).normalized()

                # 腕IK追加
                shoulder_ik_bone = Bone(index=wrist_plus_bone.index + 1, name=f"【SIZING】{direction}腕IK")
                shoulder_ik_bone.parent_index = model.bones[f"{direction}肩根元"].index
                shoulder_ik_bone.position = wrist_plus_bone.position.copy()
                shoulder_ik_bone.is_system = True
                shoulder_ik_bone.bone_flg |= (
                    BoneFlg.IS_IK | BoneFlg.CAN_TRANSLATE | BoneFlg.CAN_ROTATE | BoneFlg.CAN_MANIPULATE | BoneFlg.IS_VISIBLE
                )

                shoulder_ik = Ik()
                shoulder_ik.bone_index = wrist_plus_bone.index
                shoulder_ik.loop_count = 20
                shoulder_ik.unit_rotation.radians = MVector3D(0.5, 0, 0)

                shoulder_ik_link_elbow = IkLink()
                shoulder_ik_link_elbow.bone_index = elbow_plus_bone.index
                shoulder_ik.links.append(shoulder_ik_link_elbow)

                shoulder_ik_link_arm = IkLink()
                shoulder_ik_link_arm.bone_index = arm_plus_bone.index
                shoulder_ik.links.append(shoulder_ik_link_arm)

                shoulder_ik_link_shoulder = IkLink()
                shoulder_ik_link_shoulder.bone_index = shoulder_plus_bone.index
                shoulder_ik.links.append(shoulder_ik_link_shoulder)

                shoulder_ik_bone.ik = shoulder_ik
                model.insert_bone(shoulder_ik_bone)

                shoulder_bone.parent_index = model.bones[shoulder_bone_parent_name].index

        model.setup()

        if 10 >= logger.total_level:
            # デバッグレベルの場合、モデルを出力する
            from mlib.pmx.pmx_writer import PmxWriter

            PmxWriter(
                model, os.path.join(os.path.dirname(model.path), f"sizing_{os.path.basename(model.path)}"), include_system=True
            ).save()

        return sizing_idx, digest, original_model, model

    def save(
        self,
        sizing_idx: int,
        dest_model: PmxModel,
        motion: VmdMotion,
        output_path: str,
    ) -> None:
        """サイジング結果保存"""
        logger.info("【No.{i}】サイジング結果保存", i=sizing_idx + 1, decoration=MLogger.Decoration.LINE)

        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        VmdWriter(motion, output_path, dest_model.name).save()

        logger.info("【No.{i}】サイジング結果保存成功\n{p}", i=sizing_idx + 1, p=output_path, decoration=MLogger.Decoration.BOX)
