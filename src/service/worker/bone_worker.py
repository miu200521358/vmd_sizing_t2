import multiprocessing
import os
from concurrent.futures import Future, ProcessPoolExecutor, as_completed
from logging.handlers import QueueListener

import wx

from mlib.core.exception import MApplicationException
from mlib.core.logger import ConsoleQueueHandler, MLogger
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_frame import BaseFrame
from mlib.utils.file_utils import get_root_dir
from service.form.widgets.bone_set import SizingBoneSet
from service.usecase.io_usecase import IoUsecase
from service.usecase.move_usecase import MoveUsecase

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class BoneWorker(BaseWorker):
    def __init__(self, frame: BaseFrame, result_event: wx.Event) -> None:
        super().__init__(frame, result_event)
        self.max_worker = 1 if frame.is_saving else min(32, (os.cpu_count() or 0) + 4)

    def thread_execute(self):
        # まずは読み込み
        self.load()

        # 移動補正
        self.fit_move_sizing()

        # 保存
        self.save()

        self.result_data = []

    def fit_move_sizing(self):
        """基本のサイジング"""
        usecase = MoveUsecase()
        bone_panel = self.frame.bone_panel

        y_leg_ratios: list[float] = []
        xz_leg_ratios: list[float] = []

        for sizing_one_set in bone_panel.sizing_sets:
            sizing_set: SizingBoneSet = sizing_one_set

            # 個別の足XZ比率、足Y比率
            xz_leg_ratio, y_leg_ratio = usecase.get_leg_ratio(sizing_set.src_model_ctrl.data, sizing_set.dest_model_ctrl.data)

        # 足XZ比率
        all_xz_leg_ratio = usecase.get_all_leg_xz_ratio(xz_leg_ratios)

        with multiprocessing.Manager() as manager:
            log_queue = manager.Queue()

            console_handler = ConsoleQueueHandler(bone_panel.console_ctrl.text_ctrl, log_queue)
            listener = QueueListener(log_queue, console_handler)
            listener.start()

            with ProcessPoolExecutor(max_workers=self.max_worker) as executor:
                futures: list[Future] = []
                for sizing_set, xz_leg_ratio, y_leg_ratio in zip(bone_panel.sizing_sets, xz_leg_ratios, y_leg_ratios):
                    logger.info(
                        "【No.{i}】移動補正  縮尺: XZ[{x:.4f}]({ox:.4f}), Y[{y:.4f}]",
                        i=sizing_set.sizing_idx + 1,
                        x=all_xz_leg_ratio,
                        ox=xz_leg_ratio,
                        y=y_leg_ratio,
                        decoration=MLogger.Decoration.LINE,
                    )

                    futures.append(
                        executor.submit(
                            usecase.fit_move_sizing,
                            sizing_set.sizing_idx,
                            all_xz_leg_ratio,
                            y_leg_ratio,
                            sizing_set.src_model_ctrl.data,
                            sizing_set.dest_model_ctrl.data,
                            sizing_set.output_motion_ctrl.data,
                            log_queue,
                        )
                    )

                self.sub_process = executor._processes

            for future in as_completed(futures):
                if not future.exception():
                    sizing_idx, sizing_motion = future.result()
                    bone_panel.sizing_sets[sizing_idx].output_motion_ctrl.data = sizing_motion

            listener.stop()

    def create_sizing_morph2(self, usecase: IoUsecase):
        """サイジングモーフ追加"""
        bone_panel = self.frame.bone_panel

        y_leg_ratios: list[float] = []
        xz_leg_ratios: list[float] = []
        for sizing_one_set in bone_panel.sizing_sets:
            sizing_set: SizingBoneSet = sizing_one_set

            xz_leg_ratio, y_leg_ratio = usecase.get_leg_ratio(sizing_set.src_model_ctrl.data, sizing_set.dest_model_ctrl.data)

            # 足Y比率
            usecase.create_leg_y_ratio_morph(sizing_set.dest_model_ctrl.data, y_leg_ratio)

            xz_leg_ratios.append(xz_leg_ratio)
            y_leg_ratios.append(y_leg_ratio)

        # 足XZ比率
        all_xz_leg_ratio = usecase.get_all_leg_xz_ratio(xz_leg_ratios)
        usecase.create_leg_xz_ratio_morph(sizing_set.dest_model_ctrl.data, all_xz_leg_ratio)

        return xz_leg_ratios, y_leg_ratios, all_xz_leg_ratio

    def fit_sizing_morph_motion2(self, usecase: IoUsecase, xz_leg_ratios: list[float], y_leg_ratios: list[float], all_xz_leg_ratio: float):
        """サイジングモーフ適用"""
        bone_panel = self.frame.bone_panel

        with multiprocessing.Manager() as manager:
            log_queue = manager.Queue()

            console_handler = ConsoleQueueHandler(bone_panel.console_ctrl.text_ctrl, log_queue)
            listener = QueueListener(log_queue, console_handler)
            listener.start()

            with ProcessPoolExecutor(max_workers=self.max_worker) as executor:
                futures: list[Future] = []
                for sizing_set, xz_leg_ratio, y_leg_ratio in zip(bone_panel.sizing_sets, xz_leg_ratios, y_leg_ratios):
                    logger.info(
                        "【No.{i}】移動補正  縮尺: XZ[{x:.4f}]({ox:.4f}), Y[{y:.4f}]",
                        i=sizing_set.sizing_idx + 1,
                        x=all_xz_leg_ratio,
                        ox=xz_leg_ratio,
                        y=y_leg_ratio,
                        decoration=MLogger.Decoration.LINE,
                    )

                    futures.append(
                        executor.submit(
                            usecase.fit_move_sizing,
                            sizing_set.sizing_idx,
                            all_xz_leg_ratio,
                            y_leg_ratio,
                            sizing_set.src_model_ctrl.data,
                            sizing_set.dest_model_ctrl.data,
                            sizing_set.output_motion_ctrl.data,
                            log_queue,
                        )
                    )

                self.sub_process = executor._processes

            for future in as_completed(futures):
                if not future.exception():
                    sizing_idx, sizing_motion = future.result()
                    bone_panel.sizing_sets[sizing_idx].output_motion_ctrl.data = sizing_motion

            listener.stop()

    def save(self):
        """サイジング結果保存"""
        usecase = IoUsecase()
        bone_panel = self.frame.bone_panel

        with multiprocessing.Manager() as manager:
            log_queue = manager.Queue()

            console_handler = ConsoleQueueHandler(bone_panel.console_ctrl.text_ctrl, log_queue)
            listener = QueueListener(log_queue, console_handler)
            listener.start()

            with ProcessPoolExecutor(max_workers=self.max_worker) as executor:
                futures = [
                    executor.submit(
                        usecase.save,
                        sizing_set.sizing_idx,
                        sizing_set.dest_model_ctrl.data,
                        sizing_set.output_motion_ctrl.data,
                        sizing_set.output_motion_ctrl.path,
                        log_queue,
                    )
                    for sizing_set in bone_panel.sizing_sets
                ]

                self.sub_process = executor._processes

            as_completed(futures)
            listener.stop()

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

        if not can_load or not loadable_motion_paths or not loadable_src_model_paths or not loadable_dest_model_paths:
            raise MApplicationException("サイジングできないファイルセットが含まれているため、処理を中断します\nファイルパスが正しいか確認してください")

        with multiprocessing.Manager() as manager:
            log_queue = manager.Queue()

            console_handler = ConsoleQueueHandler(bone_panel.console_ctrl.text_ctrl, log_queue)
            listener = QueueListener(log_queue, console_handler)
            listener.start()

            with ProcessPoolExecutor(max_workers=self.max_worker) as executor:
                motion_futures = [
                    executor.submit(usecase.load_motion, motion_path, self.frame.cache_motions, log_queue)
                    for motion_path in set(loadable_motion_paths)
                ]
                src_model_futures = [
                    executor.submit(usecase.load_model, model_path, self.frame.cache_models, log_queue)
                    for model_path in set(loadable_src_model_paths)
                ]
                dest_model_futures = [
                    executor.submit(usecase.load_model, model_path, self.frame.cache_models, log_queue)
                    for model_path in set(loadable_dest_model_paths)
                ]
                self.sub_process = executor._processes

            for future in as_completed(motion_futures):
                if not future.exception():
                    sizing_idx, digest, original_motion, motion = future.result()
                    self.frame.cache_motions[digest] = original_motion
                    bone_panel.sizing_sets[sizing_idx].output_motion_ctrl.data = motion

            for future in as_completed(src_model_futures):
                if not future.exception():
                    sizing_idx, digest, original_model, model = future.result()
                    self.frame.cache_models[digest] = original_model
                    bone_panel.sizing_sets[sizing_idx].src_model_ctrl.data = model

            for future in as_completed(dest_model_futures):
                if not future.exception():
                    sizing_idx, digest, original_model, model = future.result()
                    self.frame.cache_models[digest] = original_model
                    bone_panel.sizing_sets[sizing_idx].dest_model_ctrl.data = model

            listener.stop()

    def output_log(self):
        bone_panel = self.frame.bone_panel
        output_log_path = os.path.join(get_root_dir(), f"{os.path.basename(bone_panel.sizing_sets[0].output_motion_ctrl.path)}.log")
        # 出力されたメッセージを全部出力
        bone_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
