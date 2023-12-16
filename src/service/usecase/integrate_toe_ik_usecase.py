import os
from typing import Optional

from mlib.core.logger import MLogger
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text

TOE_BONE_NAMES = {
    BoneNames.leg("右"),
    BoneNames.knee("右"),
    BoneNames.ankle("右"),
    BoneNames.toe("右"),
    BoneNames.leg("左"),
    BoneNames.knee("左"),
    BoneNames.ankle("左"),
    BoneNames.toe("左"),
}


class IntegrateToeIkUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        show_message: bool = False,
    ) -> bool:
        if not src_model or not dest_model:
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        """つま先IK"""
        if TOE_BONE_NAMES - set(src_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{x}】モーション作成元モデルに下半身・足・ひざ・足首・つま先ボーンのいずれかがないため、つま先IK統合をスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        if TOE_BONE_NAMES - set(dest_model.bones.names):
            if show_message:
                logger.warning(
                    "【No.{x}】サイジング先モデルに下半身・足・ひざ・足首・つま先ボーンのいずれかがないため、つま先IK統合をスキップします",
                    x=sizing_idx + 1,
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def sizing_integrate_toe_ik(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        src_matrixes: VmdBoneFrameTrees,
    ) -> tuple[int, str, VmdMotion]:
        logger.info(
            "【No.{i}】つま先IK統合",
            i=sizing_idx + 1,
            decoration=MLogger.Decoration.LINE,
        )

        for direction in ("右", "左"):
            self.sizing_integrate_toe_ik_direction(
                sizing_idx,
                src_model,
                dest_model,
                dest_motion,
                src_matrixes,
                direction,
            )

    def sizing_integrate_toe_ik_direction(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        dest_motion: VmdMotion,
        src_matrixes: VmdBoneFrameTrees,
        direction: str,
    ) -> tuple[int, str, VmdMotion]:
        for fidx, fno in enumerate(
            dest_motion.bones[BoneNames.toe_ik(direction)].register_indexes
        ):
            logger.count(
                "【No.{x}】【{d}】つま先IK統合",
                x=sizing_idx + 1,
                d=__(direction),
                index=fidx,
                total_index_count=len(
                    dest_motion.bones[BoneNames.toe_ik(direction)].register_indexes
                ),
                display_block=1000,
            )

            # --------------

            leg_ik_bf = dest_motion.bones[BoneNames.leg_ik(direction)][fno]
            leg_ik_bf.rotation = src_matrixes[
                BoneNames.ankle(direction), fno
            ].local_matrix.to_quaternion()
            leg_ik_bf.register = True
            dest_motion.insert_bone_frame(leg_ik_bf)

        # 終わったらつま先IKボーンのキーフレを削除
        del dest_motion.bones[BoneNames.toe_ik(direction)]

        return sizing_idx, dest_motion
