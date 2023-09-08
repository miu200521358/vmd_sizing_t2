import os

from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone, DisplaySlot, DisplaySlotReference, Ik, IkLink
from mlib.pmx.pmx_reader import PmxReader
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_reader import VmdReader
from mlib.vmd.vmd_writer import VmdWriter

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text

SIZING_BONE_PREFIX = "[SZ]"


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
        sizing_display_slot = DisplaySlot(name="SIZING")
        sizing_display_slot.is_system = True
        model.display_slots.append(sizing_display_slot)

        for direction in ("左", "右"):
            if not ({f"{direction}肩根元", f"{direction}肩", f"{direction}腕", f"{direction}手首"} - set(model.bones.names)):
                # 肩IK追加 ---------------
                shoulder_ik_bone = Bone(index=model.bones[f"{direction}腕"].index, name=f"{SIZING_BONE_PREFIX}{direction}肩IK")
                shoulder_ik_bone.parent_index = model.bones[f"{direction}肩根元"].index
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

                # 腕IK追加 ---------------
                arm_ik_bone = Bone(index=model.bones[f"{direction}ひじ"].index, name=f"{SIZING_BONE_PREFIX}{direction}腕IK")
                arm_ik_bone.parent_index = model.bones[f"{direction}肩根元"].index
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

                # ひじIK追加 ---------------
                elbow_ik_bone = Bone(index=model.bones[f"{direction}手首"].index, name=f"{SIZING_BONE_PREFIX}{direction}ひじIK")
                elbow_ik_bone.parent_index = model.bones[f"{direction}肩根元"].index
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
