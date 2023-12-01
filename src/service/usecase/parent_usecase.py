import os

from service.usecase.bone_names import BoneNames

from mlib.core.logger import MLogger
from mlib.core.math import MMatrix4x4, MVector3D
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


INTEGRATE_BONE_NAMES = {
    BoneNames.center(),
    BoneNames.leg_ik("右"),
    BoneNames.leg_ik("左"),
}


class ParentUsecase:
    def integrate_parent(
        self,
        sizing_idx: int,
        model: PmxModel,
        motion: VmdMotion,
    ) -> tuple[int, VmdMotion]:
        """全親統合"""
        logger.info(
            "【No.{i}】全親統合", i=sizing_idx + 1, decoration=MLogger.Decoration.LINE
        )

        if BoneNames.parent() not in motion.bones:
            logger.warning(
                "【No.{i}】モーションに全ての親のキーフレームがないため、全親統合をスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        parent_bone = model.bones[BoneNames.parent()]

        # 全親のキーフレ
        fnos_set: set[int] = {0} | set(
            motion.bones[BoneNames.parent()].register_indexes
        )
        bone_names: list[str] = [BoneNames.parent()]

        # 全親の子ボーンのキーフレ
        for child_bone_index in parent_bone.child_bone_indexes:
            child_bone_name = model.bones[child_bone_index].name
            bone_names.append(child_bone_name)
            fnos_set |= set(motion.bones[child_bone_name].register_indexes)

        fnos: dict[int, int] = dict(
            [(fno, fidx) for fidx, fno in enumerate(sorted(fnos_set))]
        )

        matrixes = motion.animate_bone(
            list(fnos.keys()),
            model,
            bone_names,
            out_fno_log=True,
            description="全親統合準備",
        )

        n = 0
        total_count = len(fnos) * len(bone_names)

        for child_bone_index in parent_bone.child_bone_indexes:
            child_bone_name = model.bones[child_bone_index].name

            for fno in sorted(
                set(motion.bones[child_bone_name].register_indexes)
                | set(motion.bones[BoneNames.parent()].register_indexes)
            ):
                logger.count(
                    "【No.{x}】全親統合",
                    x=sizing_idx + 1,
                    index=n,
                    total_index_count=total_count,
                    display_block=1000,
                )

                bf = motion.bones[child_bone_name][fno]
                bf.register = True
                # 親を加味した位置から自分のボーン位置を除外して再設定
                bf.position = (
                    MVector3D(
                        *matrixes._result_global_matrixes[
                            fnos[bf.index], child_bone_index, :3, 3
                        ]
                    )
                    - model.bones[child_bone_name].position
                )
                # 回転も同様
                bf.rotation = MMatrix4x4(
                    matrixes._result_matrixes[fnos[bf.index], child_bone_index]
                ).to_quaternion()
                motion.insert_bone_frame(bf)

                n += 1

        # 全親キーフレを削除
        del motion.bones[BoneNames.parent()]

        return sizing_idx, motion
