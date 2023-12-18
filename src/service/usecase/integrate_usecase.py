import os
from typing import Optional

from mlib.core.logger import MLogger
from mlib.core.math import MMatrix4x4, MVector3D
from mlib.pmx.bone_setting import BoneFlg
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion
from service.usecase.bone_names import BoneNames

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class IntegrateUsecase:
    def validate(
        self,
        sizing_idx: int,
        src_model: Optional[PmxModel],
        dest_model: Optional[PmxModel],
        bone_name: str,
        show_message: bool = False,
    ) -> bool:
        if not src_model or not dest_model:
            # モデルが揃ってない、チェックが入ってない場合、スルー
            return False

        if BoneFlg.NOTHING in dest_model.bones[bone_name].bone_flg:
            if show_message:
                logger.warning(
                    "【No.{x}】サイジング先モデルに{b}ボーンがないため、{b}統合をスキップします",
                    x=sizing_idx + 1,
                    b=__(bone_name),
                    decoration=MLogger.Decoration.BOX,
                )
            return False

        return True

    def integrate(
        self,
        sizing_idx: int,
        model: PmxModel,
        motion: VmdMotion,
        bone_name: str,
    ) -> tuple[int, VmdMotion]:
        """ボーン統合"""
        if bone_name not in motion.bones:
            logger.info(
                "【No.{x}】モーションに{b}のキーフレームがないため、{b}統合をスキップします",
                x=sizing_idx + 1,
                b=__(bone_name),
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        logger.info(
            "【No.{i}】{b}統合",
            i=sizing_idx + 1,
            b=__(bone_name),
            decoration=MLogger.Decoration.LINE,
        )

        target_bone = model.bones[bone_name]

        # 対象ボーンのキーフレ
        fnos_set: set[int] = {0} | set(motion.bones[bone_name].register_indexes)
        bone_names: list[str] = [bone_name]

        # 子ボーンのキーフレ
        for child_bone_index in target_bone.child_bone_indexes:
            child_bone_name = model.bones[child_bone_index].name
            bone_names.append(child_bone_name)
            fnos_set |= set(motion.bones[child_bone_name].register_indexes)

        if "腰" == bone_name:
            # センターも追加しておく
            fnos_set |= set(motion.bones["センター"].register_indexes)

        fnos: dict[int, int] = dict(
            [(fno, fidx) for fidx, fno in enumerate(sorted(fnos_set))]
        )

        matrixes = motion.animate_bone(
            list(fnos.keys()),
            model,
            bone_names,
            out_fno_log=True,
            description=f"{sizing_idx + 1}|{__(f'{bone_name}統合準備')}",
        )

        n = 0
        total_count = len(fnos) * len(bone_names)

        for child_bone_index in target_bone.child_bone_indexes:
            child_bone_name = model.bones[child_bone_index].name

            for fno in sorted(
                set(motion.bones[child_bone_name].register_indexes)
                | set(motion.bones[bone_name].register_indexes)
            ):
                logger.count(
                    "【No.{x}】{b}統合",
                    x=sizing_idx + 1,
                    b=__(bone_name),
                    index=n,
                    total_index_count=total_count,
                    display_block=1000,
                )

                bf = motion.bones[child_bone_name][fno]
                bf.register = True

                if model.bones[child_bone_index].can_translate:
                    # 親を加味した位置から自分のボーン位置を除外して再設定
                    bf.position = (
                        MVector3D(
                            *matrixes._result_global_matrixes[
                                fnos[bf.index], child_bone_index, :3, 3
                            ]
                        )
                        - model.bones[child_bone_name].position
                    )
                if model.bones[child_bone_index].can_rotate:
                    # 回転も同様
                    bf.rotation = MMatrix4x4(
                        matrixes._result_matrixes[fnos[bf.index], child_bone_index]
                    ).to_quaternion()
                motion.insert_bone_frame(bf)

                n += 1

        if bone_name == "腰":
            center_bone = model.bones["センター"]
            waist_bone = model.bones["腰"]
            upper_bone = model.bones["上半身"]
            lower_bone = model.bones["下半身"]

            # 上半身・下半身の中間と腰の差分
            trunk_waist_diff_vector = (
                (upper_bone.position + lower_bone.position) / 2
            ) - waist_bone.position
            # Yの差分は不要
            trunk_waist_diff_vector.y = 0

            for fno in sorted(
                set(motion.bones["センター"].register_indexes)
                | set(motion.bones["上半身"].register_indexes)
                | set(motion.bones["下半身"].register_indexes)
                | set(motion.bones[bone_name].register_indexes)
            ):
                logger.count(
                    "【No.{x}】{b}統合 - センター",
                    x=sizing_idx + 1,
                    b=__(bone_name),
                    index=n,
                    total_index_count=total_count,
                    display_block=1000,
                )

                center_bf = motion.bones[center_bone.name][fno]
                center_bf.register = True

                # センターに腰から回転したときの移動分を加算する
                center_bf.position += (
                    matrixes[waist_bone.name, fno].frame_fk_rotation
                    * trunk_waist_diff_vector
                ) - trunk_waist_diff_vector

                motion.insert_bone_frame(center_bf)

            for direction in ("右", "左"):
                for fno in sorted(
                    set(motion.bones[BoneNames.leg(direction)].register_indexes)
                    | set(motion.bones[bone_name].register_indexes)
                ):
                    logger.count(
                        "【No.{x}】{b}統合 - {l}",
                        x=sizing_idx + 1,
                        b=__(bone_name),
                        l=BoneNames.leg(direction),
                        index=n,
                        total_index_count=total_count,
                        display_block=1000,
                    )

                    waist_bf = motion.bones[BoneNames.waist()][fno]

                    leg_bf = motion.bones[BoneNames.leg(direction)][fno]
                    leg_bf.register = True

                    # 腰キャンセル分を除外する
                    leg_bf.rotation = waist_bf.rotation.inverse() * leg_bf.rotation

                    motion.insert_bone_frame(leg_bf)

        # 対象キーフレを削除
        del motion.bones[bone_name]

        return sizing_idx, motion
