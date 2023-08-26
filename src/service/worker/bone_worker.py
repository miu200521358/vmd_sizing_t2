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
from service.usecase.bone_usecase import BoneUsecase
from service.usecase.bone_usecase import MOVE_BONE_NAMES

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class BoneWorker(BaseWorker):
    def __init__(self, frame: BaseFrame, result_event: wx.Event) -> None:
        super().__init__(frame, result_event)
        self.max_worker = 1 if frame.is_saving else 6

    def thread_execute(self):
        usecase = BoneUsecase()

        # まずは読み込み
        self.load(usecase)

        # サイジングモーフを追加する
        xz_leg_ratios, y_leg_ratios, all_xz_leg_ratio = self.create_sizing_morph(usecase)

        # サイジングモーフを適用
        self.fit_sizing_morph_motion(usecase, xz_leg_ratios, y_leg_ratios, all_xz_leg_ratio)

        # 保存
        self.save(usecase)

        self.result_data = []

    def create_sizing_morph(self, usecase: BoneUsecase):
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

    def fit_sizing_morph_motion(self, usecase: BoneUsecase, xz_leg_ratios: list[float], y_leg_ratios: list[float], all_xz_leg_ratio: float):
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
                    for bone_name in MOVE_BONE_NAMES:
                        logger.info(
                            "【No.{i}】移動補正 [{n}]  縮尺: XZ[{x:.4f}]({ox:.4f}), Y[{y:.4f}]",
                            i=sizing_set.sizing_idx + 1,
                            n=bone_name,
                            x=all_xz_leg_ratio,
                            ox=xz_leg_ratio,
                            y=y_leg_ratio,
                            decoration=MLogger.Decoration.LINE,
                        )

                        futures.append(
                            executor.submit(
                                usecase.fit_move_sizing,
                                sizing_set.sizing_idx,
                                bone_name,
                                sizing_set.dest_model_ctrl.data,
                                sizing_set.output_motion_ctrl.data,
                                log_queue,
                            )
                        )

                self.sub_process = executor._processes

            for future in as_completed(futures):
                if not future.exception():
                    sizing_idx, sizing_name, sizing_motion_bones = future.result()
                    if sizing_motion_bones:
                        bone_panel.sizing_sets[sizing_idx].output_motion_ctrl.data.bones[sizing_name] = sizing_motion_bones

            listener.stop()

    def save(self, usecase: BoneUsecase):
        """サイジング結果保存"""
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

    def load(self, usecase: BoneUsecase):
        """データ読み込み"""
        bone_panel = self.frame.bone_panel

        loadable_motion_paths: list[str] = []
        loadable_model_paths: list[str] = []
        can_load: bool = True

        for sizing_one_set in bone_panel.sizing_sets:
            sizing_set: SizingBoneSet = sizing_one_set
            sizing_can_load, sizing_loadable_motion_paths, sizing_loadable_model_paths = sizing_set.get_loadable_path()
            can_load &= sizing_can_load
            loadable_motion_paths.extend(sizing_loadable_motion_paths)
            loadable_model_paths.extend(sizing_loadable_model_paths)

        if not can_load or not loadable_motion_paths or not loadable_model_paths:
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
                model_futures = [
                    executor.submit(usecase.load_model, model_path, self.frame.cache_models, log_queue)
                    for model_path in set(loadable_model_paths)
                ]
                self.sub_process = executor._processes

            for future in as_completed(motion_futures):
                if not future.exception():
                    digest, motion = future.result()
                    self.frame.cache_motions[digest] = motion

            for future in as_completed(model_futures):
                if not future.exception():
                    digest, model = future.result()
                    self.frame.cache_models[digest] = model

            listener.stop()

        for sizing_one_set in bone_panel.sizing_sets:
            sizing_set = sizing_one_set

            logger.info("【No.{i}】サイジングセット保持", i=sizing_set.sizing_idx + 1)

            sizing_set.output_motion_ctrl.data = self.frame.cache_motions[sizing_set.motion_ctrl.digest].copy()
            sizing_set.src_model_ctrl.data = self.frame.cache_models[sizing_set.src_model_ctrl.digest].copy()
            sizing_set.dest_model_ctrl.data = self.frame.cache_models[sizing_set.dest_model_ctrl.digest].copy()

    def output_log(self):
        bone_panel = self.frame.bone_panel
        output_log_path = os.path.join(get_root_dir(), f"{os.path.basename(bone_panel.sizing_sets[0].output_motion_ctrl.path)}.log")
        # 出力されたメッセージを全部出力
        bone_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
