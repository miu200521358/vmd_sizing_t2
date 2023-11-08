import os
from concurrent.futures import (
    FIRST_EXCEPTION,
    Future,
    ThreadPoolExecutor,
    as_completed,
    wait,
)

import wx

from mlib.core.exception import MApplicationException
from mlib.core.logger import MLogger
from mlib.core.math import MVector3D
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_frame import BaseFrame
from mlib.utils.file_utils import get_root_dir
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.form.widgets.bone_set import SizingBoneSet
from service.usecase.arm_twist_usecase import ArmTwistUsecase
from service.usecase.io_usecase import IoUsecase
from service.usecase.move_usecase import MoveUsecase
from service.usecase.arm_stance_usecase import ArmStanceUsecase
from service.usecase.arm_align_usecase import ArmAlignUsecase

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class BoneWorker(BaseWorker):
    def __init__(self, frame: BaseFrame, result_event: wx.Event) -> None:
        super().__init__(frame, result_event)
        self.max_worker = (
            1
            if frame.is_saving
            else max(1, int(min(32, (os.cpu_count() or 0) + 4) / 2))
        )

    def thread_execute(self):
        bone_panel = self.frame.bone_panel

        # まずは読み込み
        self.load()

        # 移動補正
        self.sizing_move()

        # 腕スタンス補正
        self.sizing_arm_stance()

        # 捩り分散・腕位置合わせがある場合、腕の回転の初期位置を取得
        if (
            bone_panel.align_check_ctrl.GetValue()
            or bone_panel.twist_check_ctrl.GetValue()
        ):
            initial_matrixes = self.get_initial_arm_matrixes()

            # 捩り分散
            if bone_panel.twist_check_ctrl.GetValue():
                self.sizing_arm_twist(initial_matrixes)

            # # 腕位置合わせ
            # if bone_panel.align_check_ctrl.GetValue():
            #     self.sizing_arm_align(initial_matrixes)

        # 保存
        self.save()

        self.result_data = []

    def get_initial_arm_matrixes(self) -> None:
        """腕初期位置取得"""
        logger.info("腕：初期位置取得", decoration=MLogger.Decoration.BOX)

        usecase = ArmTwistUsecase()
        bone_panel = self.frame.bone_panel
        initial_matrixes: dict[tuple[int, bool, str], VmdBoneFrameTrees] = {}

        # 先にIKが無い状態でモーション行列を取得する
        with ThreadPoolExecutor(
            thread_name_prefix="arm_initial", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in bone_panel.sizing_sets:
                for direction in ("右", "左"):
                    futures.append(
                        executor.submit(
                            usecase.get_initial_matrixes,
                            sizing_set.sizing_idx,
                            True,
                            sizing_set.src_model_ctrl.data,
                            sizing_set.motion_ctrl.data,
                            direction,
                        )
                    )

                    futures.append(
                        executor.submit(
                            usecase.get_initial_matrixes,
                            sizing_set.sizing_idx,
                            False,
                            sizing_set.dest_model_ctrl.data,
                            sizing_set.output_motion_ctrl.data,
                            direction,
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, is_src, direction, matrixes = future.result()
                initial_matrixes[(sizing_idx, is_src, direction)] = matrixes

        return initial_matrixes

    def sizing_arm_align(self) -> None:
        """腕位置合わせ"""
        logger.info("腕位置合わせ", decoration=MLogger.Decoration.BOX)

        usecase = ArmAlignUsecase()
        bone_panel = self.frame.bone_panel
        initial_matrixes: dict[tuple[int, bool, str], VmdBoneFrameTrees] = {}

        # 先にIKが無い状態でモーション行列を取得する
        with ThreadPoolExecutor(
            thread_name_prefix="arm_align_initial", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in bone_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.src_model_ctrl.data,
                    sizing_set.dest_model_ctrl.data,
                    bone_panel.align_check_ctrl.GetValue(),
                    bone_panel.align_finger_check_ctrl.GetValue(),
                    bone_panel.align_finger_tail_check_ctrl.GetValue(),
                    bone_panel.twist_check_ctrl.GetValue(),
                ):
                    for direction in ("右", "左"):
                        futures.append(
                            executor.submit(
                                usecase.get_initial_matrixes,
                                sizing_set.sizing_idx,
                                True,
                                sizing_set.src_model_ctrl.data,
                                sizing_set.motion_ctrl.data,
                                direction,
                            )
                        )

                        futures.append(
                            executor.submit(
                                usecase.get_initial_matrixes,
                                sizing_set.sizing_idx,
                                False,
                                sizing_set.dest_model_ctrl.data,
                                sizing_set.output_motion_ctrl.data,
                                direction,
                            )
                        )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, is_src, direction, matrixes = future.result()
                initial_matrixes[(sizing_idx, is_src, direction)] = matrixes

        # IKセットアップする
        for sizing_set in bone_panel.sizing_sets:
            if usecase.validate(
                sizing_set.sizing_idx,
                sizing_set.src_model_ctrl.data,
                sizing_set.dest_model_ctrl.data,
                bone_panel.align_check_ctrl.GetValue(),
                bone_panel.align_finger_check_ctrl.GetValue(),
                bone_panel.align_finger_tail_check_ctrl.GetValue(),
                bone_panel.twist_check_ctrl.GetValue(),
            ):
                usecase.setup_model_ik(
                    sizing_set.sizing_idx,
                    True,
                    sizing_set.src_model_ctrl.data,
                    bone_panel.align_finger_check_ctrl.GetValue(),
                    bone_panel.align_finger_tail_check_ctrl.GetValue(),
                    bone_panel.twist_check_ctrl.GetValue(),
                )
                usecase.setup_model_ik(
                    sizing_set.sizing_idx,
                    False,
                    sizing_set.dest_model_ctrl.data,
                    bone_panel.align_finger_check_ctrl.GetValue(),
                    bone_panel.align_finger_tail_check_ctrl.GetValue(),
                    bone_panel.twist_check_ctrl.GetValue(),
                )

        with ThreadPoolExecutor(
            thread_name_prefix="arm_align", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in bone_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.src_model_ctrl.data,
                    sizing_set.dest_model_ctrl.data,
                    bone_panel.align_check_ctrl.GetValue(),
                    bone_panel.align_finger_check_ctrl.GetValue(),
                    bone_panel.align_finger_tail_check_ctrl.GetValue(),
                    bone_panel.twist_check_ctrl.GetValue(),
                ):
                    for direction in ("右", "左"):
                        futures.append(
                            executor.submit(
                                usecase.sizing_arm_align,
                                sizing_set.sizing_idx,
                                sizing_set.src_model_ctrl.data,
                                sizing_set.dest_model_ctrl.data,
                                sizing_set.motion_ctrl.data,
                                sizing_set.output_motion_ctrl.data,
                                initial_matrixes[
                                    (sizing_set.sizing_idx, True, direction)
                                ],
                                initial_matrixes[
                                    (sizing_set.sizing_idx, False, direction)
                                ],
                                direction,
                                bone_panel.align_finger_check_ctrl.GetValue(),
                                bone_panel.align_finger_tail_check_ctrl.GetValue(),
                                bone_panel.twist_check_ctrl.GetValue(),
                            )
                        )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, sizing_motion = future.result()
                bone_panel.sizing_sets[
                    sizing_idx
                ].output_motion_ctrl.data = sizing_motion

    def sizing_arm_twist(
        self, initial_matrixes: dict[tuple[int, bool, str], VmdBoneFrameTrees]
    ) -> None:
        """捩り分散"""
        logger.info("捩り分散", decoration=MLogger.Decoration.BOX)

        usecase = ArmTwistUsecase()
        bone_panel = self.frame.bone_panel

        with ThreadPoolExecutor(
            thread_name_prefix="arm_twist", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in bone_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.src_model_ctrl.data,
                    sizing_set.dest_model_ctrl.data,
                    bone_panel.twist_check_ctrl.GetValue(),
                ):
                    for direction in ("右", "左"):
                        futures.append(
                            executor.submit(
                                usecase.sizing_arm_twist,
                                sizing_set.sizing_idx,
                                sizing_set.src_model_ctrl.data,
                                sizing_set.dest_model_ctrl.data,
                                sizing_set.motion_ctrl.data,
                                sizing_set.output_motion_ctrl.data,
                                initial_matrixes[
                                    (sizing_set.sizing_idx, True, direction)
                                ],
                                initial_matrixes[
                                    (sizing_set.sizing_idx, False, direction)
                                ],
                                direction,
                            )
                        )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, sizing_motion = future.result()
                bone_panel.sizing_sets[
                    sizing_idx
                ].output_motion_ctrl.data = sizing_motion

    def sizing_arm_stance(self):
        """腕スタンス補正"""
        logger.info("腕スタンス補正", decoration=MLogger.Decoration.BOX)

        usecase = ArmStanceUsecase()
        bone_panel = self.frame.bone_panel

        with ThreadPoolExecutor(
            thread_name_prefix="arm_stance", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in bone_panel.sizing_sets:
                futures.append(
                    executor.submit(
                        usecase.sizing_arm_stance,
                        sizing_set.sizing_idx,
                        sizing_set.src_model_ctrl.data,
                        sizing_set.dest_model_ctrl.data,
                        sizing_set.output_motion_ctrl.data,
                    )
                )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, sizing_motion = future.result()
                bone_panel.sizing_sets[
                    sizing_idx
                ].output_motion_ctrl.data = sizing_motion

    def sizing_move(self):
        """移動補正"""

        logger.info("移動補正", decoration=MLogger.Decoration.BOX)

        usecase = MoveUsecase()
        bone_panel = self.frame.bone_panel

        xz_leg_ratios: list[float] = []
        y_leg_ratios: list[float] = []
        center_offsets: list[MVector3D] = []

        for sizing_one_set in bone_panel.sizing_sets:
            sizing_set: SizingBoneSet = sizing_one_set

            # 個別の足XZ比率、足Y比率
            xz_leg_ratio, y_leg_ratio, center_offset = usecase.get_move_ratio(
                sizing_set.src_model_ctrl.data, sizing_set.dest_model_ctrl.data
            )

            xz_leg_ratios.append(xz_leg_ratio)
            y_leg_ratios.append(y_leg_ratio)
            center_offsets.append(center_offset)

        # 全体の足XZ比率
        all_xz_leg_ratio = usecase.get_all_leg_xz_ratio(xz_leg_ratios)

        with ThreadPoolExecutor(
            thread_name_prefix="move", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set, xz_leg_ratio, y_leg_ratio, center_offset in zip(
                bone_panel.sizing_sets, xz_leg_ratios, y_leg_ratios, center_offsets
            ):
                futures.append(
                    executor.submit(
                        usecase.sizing_move,
                        sizing_set.sizing_idx,
                        xz_leg_ratio,
                        MVector3D(all_xz_leg_ratio, y_leg_ratio, all_xz_leg_ratio),
                        center_offset,
                        sizing_set.src_model_ctrl.data,
                        sizing_set.dest_model_ctrl.data,
                        sizing_set.output_motion_ctrl.data,
                    )
                )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, sizing_motion = future.result()
                bone_panel.sizing_sets[
                    sizing_idx
                ].output_motion_ctrl.data = sizing_motion

    def save(self):
        """サイジング結果保存"""
        usecase = IoUsecase()
        bone_panel = self.frame.bone_panel

        with ThreadPoolExecutor(
            thread_name_prefix="save", max_workers=self.max_worker
        ) as executor:
            futures = [
                executor.submit(
                    usecase.save,
                    sizing_set.sizing_idx,
                    sizing_set.dest_model_ctrl.data,
                    sizing_set.output_motion_ctrl.data,
                    sizing_set.output_motion_ctrl.path,
                )
                for sizing_set in bone_panel.sizing_sets
            ]

        wait(futures, return_when=FIRST_EXCEPTION)

        for future in as_completed(futures):
            if future.exception():
                raise future.exception()

    def load(self):
        """データ読み込み"""
        usecase = IoUsecase()
        bone_panel = self.frame.bone_panel

        loadable_motion_paths: list[str] = []
        loadable_src_model_paths: list[str] = []
        loadable_dest_model_paths: list[str] = []
        can_load: bool = True

        for sizing_one_set in bone_panel.sizing_sets:
            sizing_set: SizingBoneSet = sizing_one_set
            (
                sizing_can_load,
                sizing_loadable_motion_paths,
                sizing_loadable_src_model_paths,
                sizing_loadable_dest_model_paths,
            ) = sizing_set.get_loadable_path()
            can_load &= sizing_can_load
            loadable_motion_paths.extend(sizing_loadable_motion_paths)
            loadable_src_model_paths.extend(sizing_loadable_src_model_paths)
            loadable_dest_model_paths.extend(sizing_loadable_dest_model_paths)

        if (
            not can_load
            or not loadable_motion_paths
            or not loadable_src_model_paths
            or not loadable_dest_model_paths
        ):
            raise MApplicationException(
                "サイジングできないファイルセットが含まれているため、処理を中断します\nファイルパスが正しいか確認してください"
            )

        with ThreadPoolExecutor(
            thread_name_prefix="load", max_workers=self.max_worker
        ) as executor:
            motion_futures = [
                executor.submit(
                    usecase.load_motion,
                    sizing_idx,
                    motion_path,
                    self.frame.cache_motions,
                )
                for sizing_idx, motion_path in enumerate(loadable_motion_paths)
            ]
            src_model_futures = [
                executor.submit(
                    usecase.load_model,
                    sizing_idx,
                    model_path,
                    self.frame.cache_models,
                    True,
                )
                for sizing_idx, model_path in enumerate(loadable_src_model_paths)
            ]
            dest_model_futures = [
                executor.submit(
                    usecase.load_model,
                    sizing_idx,
                    model_path,
                    self.frame.cache_models,
                    False,
                )
                for sizing_idx, model_path in enumerate(loadable_dest_model_paths)
            ]

        wait(motion_futures, return_when=FIRST_EXCEPTION)

        for future in as_completed(motion_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, digest, original_motion, motion = future.result()
            self.frame.cache_motions[digest] = original_motion
            bone_panel.sizing_sets[sizing_idx].motion_ctrl.data = original_motion
            bone_panel.sizing_sets[sizing_idx].output_motion_ctrl.data = motion

        wait(src_model_futures, return_when=FIRST_EXCEPTION)

        for future in as_completed(src_model_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, digest, original_model, model = future.result()
            self.frame.cache_models[digest] = original_model
            bone_panel.sizing_sets[sizing_idx].src_model_ctrl.data = model

        wait(dest_model_futures, return_when=FIRST_EXCEPTION)

        for future in as_completed(dest_model_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, digest, original_model, model = future.result()
            self.frame.cache_models[digest] = original_model
            bone_panel.sizing_sets[sizing_idx].dest_model_ctrl.data = model

    def output_log(self):
        bone_panel = self.frame.bone_panel
        output_log_path = os.path.join(
            get_root_dir(),
            f"{os.path.basename(bone_panel.sizing_sets[0].output_motion_ctrl.path)}.log",
        )
        # 出力されたメッセージを全部出力
        bone_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
