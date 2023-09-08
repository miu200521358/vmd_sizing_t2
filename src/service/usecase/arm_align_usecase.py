from concurrent.futures import Future, ThreadPoolExecutor, as_completed
import os

from mlib.core.logger import MLogger
from mlib.pmx.pmx_collection import PmxModel
from mlib.vmd.vmd_collection import VmdMotion
from mlib.vmd.vmd_part import VmdBoneFrame
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.usecase.io_usecase import SIZING_BONE_PREFIX

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class ArmAlignUsecase:
    def sizing_arm_align(
        self,
        sizing_idx: int,
        src_model: PmxModel,
        dest_model: PmxModel,
        original_motion: VmdMotion,
        motion: VmdMotion,
        direction: str,
        max_worker: int,
    ) -> tuple[int, VmdMotion]:
        SHOULDER_BONE_NAMES = [f"{direction}肩", f"{direction}腕", f"{direction}ひじ", f"{direction}手首"]

        """肩腕位置合わせ"""
        if set(SHOULDER_BONE_NAMES) - set(src_model.bones.names):
            logger.warning(
                "【No.{i}】モーション作成元モデルに肩・腕・ひじ・手首の左右ボーンがないため、肩腕位置合わせをスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        if set(SHOULDER_BONE_NAMES) - set(dest_model.bones.names):
            logger.warning(
                "【No.{i}】サイジング先モデルに肩・腕・ひじ・手首の左右ボーンがないため、肩腕位置合わせをスキップします",
                i=sizing_idx + 1,
                decoration=MLogger.Decoration.BOX,
            )
            return sizing_idx, motion

        shoulder_fnos = sorted(set(motion.bones[f"{direction}肩"].indexes) | set(motion.bones[f"{direction}肩P"].indexes))

        arm_fnos = sorted(
            set(motion.bones[f"{direction}肩"].indexes)
            | set(motion.bones[f"{direction}肩P"].indexes)
            | set(motion.bones[f"{direction}腕"].indexes)
            | set(motion.bones[f"{direction}腕捩"].indexes)
            | set(motion.bones[f"{direction}ひじ"].indexes)
            | set(motion.bones[f"{direction}手捩"].indexes)
            | set(motion.bones[f"{direction}手首"].indexes)
        )

        if not shoulder_fnos and not arm_fnos:
            return sizing_idx, motion

        shoulder_ratio = dest_model.bones[f"{direction}肩根元"].position.distance(
            dest_model.bones[f"{direction}腕"].position
        ) / src_model.bones[f"{direction}肩根元"].position.distance(src_model.bones[f"{direction}腕"].position)

        logger.debug(f"shoulder_ratio[{shoulder_ratio:.3f}]")

        arm_ratio = (
            dest_model.bones[f"{direction}腕"].position.distance(dest_model.bones[f"{direction}ひじ"].position)
            + dest_model.bones[f"{direction}ひじ"].position.distance(dest_model.bones[f"{direction}手首"].position)
        ) / (
            src_model.bones[f"{direction}腕"].position.distance(src_model.bones[f"{direction}ひじ"].position)
            + src_model.bones[f"{direction}ひじ"].position.distance(src_model.bones[f"{direction}手首"].position)
        )

        logger.debug(f"arm_ratio[{arm_ratio:.3f}]")

        logger.info("【No.{i}】{d}肩位置合わせ", i=sizing_idx + 1, d=direction, decoration=MLogger.Decoration.LINE)

        original_motion.cache_clear()
        motion.cache_clear()

        with ThreadPoolExecutor(thread_name_prefix="arm_align_shoulder", max_workers=max_worker) as executor:
            futures: list[Future] = []
            futures.append(
                executor.submit(
                    original_motion.animate_bone,
                    arm_fnos,
                    src_model,
                    [f"{direction}肩", f"{direction}腕", f"{direction}手首"],
                    append_ik=False,
                    out_fno_log=True,
                )
            )

            futures.append(
                executor.submit(
                    motion.animate_bone,
                    arm_fnos,
                    dest_model,
                    [f"{direction}肩", f"{direction}腕", f"{SIZING_BONE_PREFIX}{direction}肩IK"],
                    append_ik=False,
                    out_fno_log=True,
                )
            )

            if as_completed(futures):
                src_matrixes: VmdBoneFrameTrees = futures[0].result()
                dest_shoulder_matrixes: VmdBoneFrameTrees = futures[1].result()

        for fidx, fno in enumerate(shoulder_fnos):
            logger.count(
                "【No.{x}】{d}肩位置合わせ", x=sizing_idx + 1, d=direction, index=fidx, total_index_count=len(shoulder_fnos), display_block=1000
            )

            # 肩IK --------------------
            src_arm_local_position = src_matrixes[fno, f"{direction}腕"].position - src_matrixes[fno, f"{direction}肩"].position
            dest_arm_global_position = dest_shoulder_matrixes[fno, f"{direction}肩"].position + (src_arm_local_position * shoulder_ratio)

            logger.debug(
                f"[{direction}肩][{fno}]"
                + f"元肩{src_matrixes[fno, f'{direction}肩'].position}, 元腕{src_matrixes[fno, f'{direction}腕'].position}, "
                + f"元腕ローカル {src_arm_local_position}, "
                + f"先肩{dest_shoulder_matrixes[fno, f'{direction}肩'].position}, 先腕{dest_shoulder_matrixes[fno, f'{direction}腕'].position}, "
                + f"先腕グローバル {dest_arm_global_position}, "
            )

            shoulder_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}肩IK")
            shoulder_ik_bf.position = (
                dest_shoulder_matrixes[fno, f"{SIZING_BONE_PREFIX}{direction}肩IK"].global_matrix_no_scale.inverse()
                * dest_arm_global_position
            )
            motion.append_bone_frame(shoulder_ik_bf)

        logger.info("【No.{i}】{d}腕位置合わせ", i=sizing_idx + 1, d=direction, decoration=MLogger.Decoration.LINE)

        motion.cache_clear()
        dest_arm_matrixes = motion.animate_bone(
            arm_fnos,
            dest_model,
            [f"{direction}腕", f"{direction}手首", f"{SIZING_BONE_PREFIX}{direction}腕IK"],
            out_fno_log=True,
        )

        for fidx, fno in enumerate(arm_fnos):
            logger.count(
                "【No.{x}】{d}腕位置合わせ", x=sizing_idx + 1, d=direction, index=fidx, total_index_count=len(arm_fnos), display_block=1000
            )

            # 腕IK --------------------
            src_wrist_local_position = src_matrixes[fno, f"{direction}手首"].position - src_matrixes[fno, f"{direction}腕"].position
            dest_wrist_global_position = dest_arm_matrixes[fno, f"{direction}腕"].position + (src_wrist_local_position * arm_ratio)

            logger.debug(
                f"[{direction}腕][{fno}]"
                + f"元腕{src_matrixes[fno, f'{direction}腕'].position}, 元手首{src_matrixes[fno, f'{direction}手首'].position}, "
                + f"元腕ローカル {src_wrist_local_position}, "
                + f"先腕{dest_arm_matrixes[fno, f'{direction}腕'].position}, 先手首{dest_arm_matrixes[fno, f'{direction}手首'].position}, "
                + f"先手首グローバル {dest_wrist_global_position}, "
            )

            arm_ik_bf = VmdBoneFrame(fno, f"{SIZING_BONE_PREFIX}{direction}腕IK")
            arm_ik_bf.position = (
                dest_arm_matrixes[fno, f"{SIZING_BONE_PREFIX}{direction}腕IK"].global_matrix_no_scale.inverse() * dest_wrist_global_position
            )
            motion.append_bone_frame(arm_ik_bf)

        return sizing_idx, motion
