import os
from concurrent.futures import Future, ThreadPoolExecutor, as_completed

from mlib.core.logger import MLogger
from mlib.core.math import MQuaternion
from mlib.pmx.pmx_collection import PmxModel
from mlib.pmx.pmx_part import Bone
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_tree import VmdBoneFrameTrees

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BakeUsecase:
    def bake_ik(
        self,
        model: PmxModel,
        motion: VmdMotion,
        max_worker: int,
    ) -> VmdMotion:
        """IK焼き込み"""

        with ThreadPoolExecutor(
            thread_name_prefix="bake", max_workers=max_worker
        ) as executor:
            futures: list[Future] = []

            for bone in model.bones:
                if (
                    bone.is_ik
                    and bone.can_manipulate
                    and bone.is_visible
                    and bone.ik.bone_index in model.bones
                    and not [
                        bone_index
                        for bone_index in bone.child_bone_indexes
                        if model.bones[bone_index].is_ik
                    ]
                ):
                    # 自身がIKであること、IKのターゲットが存在している事、子ボーンにIKがいないこと（つま先ＩＫ対策）
                    futures.append(
                        executor.submit(self.bake_ik_bone, model, motion, bone)
                    )

            bake_motion = VmdMotion()
            bake_motion.path = motion.path

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()

                ik_bone_index, fnos, matrixes = future.result()
                self.set_ik_rotations(
                    model, motion, bake_motion, ik_bone_index, fnos, matrixes
                )

        # TODO サイジングに持っていった時に、足FKの向きからあるべきグルーブorセンターの位置を再計算して、配置し直す

        return bake_motion

    def set_ik_rotations(
        self,
        model: PmxModel,
        motion: VmdMotion,
        bake_motion: VmdMotion,
        ik_bone_index: int,
        fnos: list[int],
        matrixes: VmdBoneFrameTrees,
    ):
        for link in model.bones[ik_bone_index].ik.links:
            if link.bone_index not in model.bones:
                continue

            bone_name = model.bones[link.bone_index].name

            for bf in motion.bones[bone_name]:
                if not bf.register:
                    continue
                # 先に一旦焼き込み用のモーションを生成しておく
                bake_motion.append_bone_frame(bf.copy())

            logger.info("IK計算結果設定: {b}", b=bone_name)
            prev_qq = MQuaternion()
            for fidx, fno in enumerate(fnos):
                logger.count("IK計算結果設定", fidx, len(fnos), display_block=1000)

                qq = matrixes[fno, bone_name].frame_ik_rotation
                logger.debug(f"[{bone_name}][{fno}][now: {qq.to_degrees():.3f}]")

                bf = motion.bones[bone_name][fno]
                if fidx == 0 or bf.register or qq.dot(prev_qq) < 0.99:
                    # 最初のキーフレ・登録キーフレ・前回からある程度変形したキーフレの場合、登録
                    bf.rotation = qq
                    bf.register = True
                    bake_motion.insert_bone_frame(bf)

                    prev_qq = qq

        if model.bones[model.bones[ik_bone_index].parent_index].is_ik:
            # 親ボーンがIKである場合、親も辿る
            self.set_ik_rotations(
                model,
                motion,
                bake_motion,
                model.bones[ik_bone_index].parent_index,
                fnos,
                matrixes,
            )

    def bake_ik_bone(
        self,
        model: PmxModel,
        motion: VmdMotion,
        ik_bone: Bone,
    ) -> tuple[Bone, list[int], VmdBoneFrameTrees]:
        logger.info("IK焼き込み: {b}", b=ik_bone.name, decoration=MLogger.Decoration.LINE)

        bone_names: list[str] = [model.bones[ik_bone.ik.bone_index].name]

        fnos = set(motion.bones[ik_bone.name].register_indexes)
        for tree_bone in model.bone_trees[ik_bone.name]:
            fnos |= set(motion.bones[tree_bone.name].register_indexes)
            bone_names.append(tree_bone.name)

        fnos |= set(
            motion.bones[model.bones[ik_bone.ik.bone_index].name].register_indexes
        )
        for link in ik_bone.ik.links:
            if link.bone_index not in model.bones:
                continue
            bone_name = model.bones[link.bone_index].name
            fnos = fnos | set(motion.bones[bone_name].register_indexes)
            bone_names.append(bone_name)

            for tree_bone in model.bone_trees[bone_name]:
                fnos |= set(motion.bones[tree_bone.name].register_indexes)
                bone_names.append(tree_bone.name)

        sorted_fnos = sorted(fnos)

        # for fidx, fno in enumerate(fnos):
        #     logger.count("IK事前計算", fidx, len(fnos), display_block=10000)
        #     for bone_name in bone_names:
        #         bf = motion.bones[bone_name][fno]
        #         motion.insert_bone_frame(bf)

        return (
            ik_bone.index,
            sorted_fnos,
            motion.animate_bone(
                sorted_fnos,
                model,
                bone_names,
                out_fno_log=True,
                description=ik_bone.name,
            ),
        )
