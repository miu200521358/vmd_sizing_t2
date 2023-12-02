import os
from concurrent.futures import Future, ThreadPoolExecutor, as_completed, wait

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
        selected_bone_names: list[str],
        bake_interval: int,
        bake_grain: float,
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
                    and bone.ik.bone_index in model.bones
                    and [
                        ik_link.bone_index
                        for ik_link in bone.ik.links
                        if model.bones[ik_link.bone_index].name in selected_bone_names
                    ]
                    and not [
                        ik_link.bone_index
                        for child_bone_index in bone.child_bone_indexes
                        if model.bones[child_bone_index].is_ik
                        for ik_link in model.bones[child_bone_index].ik.links
                        if model.bones[ik_link.bone_index].name in selected_bone_names
                    ]
                ):
                    # 自身がIKであること、IKのターゲットが存在している事、IKリンクに処理対象ボーンがいること、子ボーンにIKがいないこと（つま先ＩＫ対策）
                    futures.append(
                        executor.submit(
                            self.bake_ik_bone, model, motion, bone, bake_interval
                        )
                    )

            wait(futures)

            bake_motion = VmdMotion()
            bake_motion.path = motion.path

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()

                ik_bone_index, fnos, matrixes = future.result()
                self.set_ik_rotations(
                    model,
                    motion,
                    bake_motion,
                    ik_bone_index,
                    fnos,
                    selected_bone_names,
                    bake_interval,
                    bake_grain,
                    matrixes,
                )

        return bake_motion

    def set_ik_rotations(
        self,
        model: PmxModel,
        motion: VmdMotion,
        bake_motion: VmdMotion,
        ik_bone_index: int,
        fnos: list[int],
        selected_bone_names: list[str],
        bake_interval: int,
        bake_grain: float,
        matrixes: VmdBoneFrameTrees,
    ):
        # # IKボーン
        # ik_bone = model.bones[ik_bone_index]
        # # Ikリンクルートボーン
        # ik_link_root_bone = model.bones[ik_bone.ik.links[-1].bone_index]
        # # IKターゲットボーン
        # effector_bone = model.bones[ik_bone.ik.bone_index]

        # ik_bone_indexes = list(
        #     reversed(
        #         [effector_bone.index]
        #         + [ik_link.bone_index for ik_link in ik_bone.ik.links]
        #     )
        # )
        # # IKボーンの距離
        # bone_distances = [
        #     model.bones[parent_bone_index].position.distance(
        #         model.bones[child_bone_index].position
        #     )
        #     for parent_bone_index, child_bone_index in zip(
        #         ik_bone_indexes, ik_bone_indexes[1:]
        #     )
        # ]

        # ik_over_fnos: set[int] = set([])
        link_fnos: set[int] = set([])
        for link in reversed(model.bones[ik_bone_index].ik.links):
            if (
                link.bone_index not in model.bones
                or model.bones[link.bone_index].name not in selected_bone_names
            ):
                continue

            bone = model.bones[link.bone_index]

            for bf in motion.bones[bone.name]:
                if not bf.register:
                    continue
                # 先に一旦焼き込み用のモーションを生成しておく
                bake_motion.append_bone_frame(bf.copy())

            logger.info("IK計算結果設定: {b}", b=bone.name)
            prev_qq = MQuaternion()
            for fidx, fno in enumerate(fnos):
                logger.count("IK計算結果設定", fidx, len(fnos), display_block=1000)

                qq = matrixes[bone.name, fno].frame_fk_rotation
                effective_bone_registers = []
                for effective_bone_index in bone.effective_target_indexes:
                    # 自身が付与親となっているボーンがある場合、そのボーンの回転量を除去しておく
                    effective_bone = model.bones[effective_bone_index]
                    effective_bf = motion.bones[effective_bone.name][fno]
                    qq = (
                        matrixes[effective_bone.name, fno].frame_fk_rotation.inverse()
                        * qq
                    )
                    if effective_bf.register:
                        # 付与親となってるボーンにキーフレが打たれていたら、それも登録対象とする
                        effective_bone_registers.append(True)

                bf = motion.bones[bone.name][fno]

                # ik_local_position = (
                #     matrixes[ik_bone.name, fno].position
                #     - matrixes[ik_link_root_bone.name, fno].position
                # )
                # if sum(bone_distances) * 1.05 < ik_local_position.length():
                #     # IKボーンの位置がIK関連ボーンの長さより長い場合、警告する
                #     ik_over_fnos |= {fno}

                # 固定間隔キーフレが選択されているか否か
                is_register_interval = (
                    True
                    if (
                        (bake_interval == 1 and fno % 5 == 0)
                        or (bake_interval == 2 and fno % 3 == 0)
                        or bake_interval == 3
                    )
                    else False
                )

                if (
                    0 == fidx
                    or bf.register
                    or effective_bone_registers
                    or fno in link_fnos
                    or qq.dot(prev_qq) < 0.97 + (bake_grain * 0.03)
                    or is_register_interval
                ):
                    logger.debug(f"[{bone.name}][{fno}][now: {qq.to_degree():.3f}]")

                    # 最初のキーフレ・登録キーフレ・リンクの親が登録されているキーフレ・前回からある程度変形したキーフレの場合、登録

                    # 前のキーフレが離れていて登録されてなければそれも登録
                    if (
                        0 < fidx
                        and fnos[fidx - 1] not in bake_motion.bones[bone.name]
                        and 3 < fno - fnos[fidx - 1]
                    ):
                        prev_fno = fnos[fidx - 1]
                        prev_bf = motion.bones[bone.name][prev_fno]

                        prev_qq = matrixes[bone.name, prev_fno].frame_fk_rotation
                        for effective_bone_index in bone.effective_target_indexes:
                            # 自身が付与親となっているボーンがある場合、そのボーンの回転量を除去しておく
                            effective_bone = model.bones[effective_bone_index]
                            prev_qq = (
                                matrixes[
                                    effective_bone.name, prev_fno
                                ].frame_fk_rotation.inverse()
                                * prev_qq
                            )

                        logger.debug(
                            f"[{bone.name}][{prev_fno}][prev: {prev_qq.to_degree():.3f}]"
                        )
                        prev_bf.rotation = prev_qq
                        prev_bf.register = True
                        bake_motion.insert_bone_frame(prev_bf)

                    bf.rotation = qq
                    bf.register = True
                    bake_motion.insert_bone_frame(bf)

                    prev_qq = qq
                    link_fnos |= {fno}

        # if ik_over_fnos:
        #     logger.warning(
        #         "[{b}]下記キーフレは、IKボーン({i})の位置がIKターゲットボーン({e})より遠くに配置されている為、焼き込み結果が安定してない可能性があります\n    {f}",
        #         b=", ".join(
        #             (
        #                 model.bones[link.bone_index].name
        #                 for link in reversed(model.bones[ik_bone_index].ik.links)
        #             )
        #         ),
        #         i=ik_bone.name,
        #         e=effector_bone.name,
        #         f=", ".join((str(fno) for fno in sorted(ik_over_fnos))),
        #         decoration=MLogger.Decoration.BOX,
        #     )

        if model.bones[model.bones[ik_bone_index].parent_index].is_ik:
            # 親ボーンがIKである場合、親も辿る
            self.set_ik_rotations(
                model,
                motion,
                bake_motion,
                model.bones[ik_bone_index].parent_index,
                fnos,
                selected_bone_names,
                bake_interval,
                bake_grain,
                matrixes,
            )

    def bake_ik_bone(
        self,
        model: PmxModel,
        motion: VmdMotion,
        ik_bone: Bone,
        bake_interval: int,
    ) -> tuple[Bone, list[int], VmdBoneFrameTrees]:
        logger.info("IK焼き込み: {b}", b=ik_bone.name, decoration=MLogger.Decoration.LINE)

        bone_names: list[str] = [ik_bone.name, model.bones[ik_bone.ik.bone_index].name]

        fnos = set(motion.bones[ik_bone.name].register_indexes)

        for effective_index in model.bones[ik_bone.name].effective_target_indexes:
            effective_bone = model.bones[effective_index]
            fnos |= set(motion.bones[effective_bone.name].register_indexes)
            bone_names.append(effective_bone.name)

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

            for effective_index in model.bones[bone_name].effective_target_indexes:
                effective_bone = model.bones[effective_index]
                fnos |= set(motion.bones[effective_bone.name].register_indexes)
                bone_names.append(effective_bone.name)

            for tree_bone in model.bone_trees[bone_name]:
                fnos |= set(motion.bones[tree_bone.name].register_indexes)
                bone_names.append(tree_bone.name)

        if bake_interval == 1:
            # 5Fごと
            fnos |= set(range(0, motion.max_fno + 1, 5))
        elif bake_interval == 2:
            # 3Fごと
            fnos |= set(range(0, motion.max_fno + 1, 3))
        elif bake_interval == 3:
            # 全打ち
            fnos |= set(range(motion.max_fno + 1))

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
