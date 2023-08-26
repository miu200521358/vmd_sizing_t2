from concurrent.futures import ProcessPoolExecutor, as_completed
from logging.handlers import QueueListener
import multiprocessing
import os

import wx

from mlib.core.logger import MLogger
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_frame import BaseFrame
from mlib.utils.file_utils import get_root_dir
from mlib.core.logger import ConsoleQueueHandler
from service.usecase.bone_usecase import BoneUsecase
from mlib.core.exception import MApplicationException

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class BoneWorker(BaseWorker):
    def __init__(self, frame: BaseFrame, result_event: wx.Event) -> None:
        super().__init__(frame, result_event)

    def thread_execute(self):
        bone_panel = self.frame.bone_panel
        usecase = BoneUsecase()

        loadable_motion_paths: list[str] = []
        loadable_model_paths: list[str] = []
        can_load: bool = True

        for sizing_set in bone_panel.sizing_sets:
            sizing_can_load, sizing_loadable_motion_paths, sizing_loadable_model_paths = sizing_set.get_loadable_path()
            can_load &= sizing_can_load
            loadable_motion_paths.extend(sizing_loadable_motion_paths)
            loadable_model_paths.extend(sizing_loadable_model_paths)

        if not can_load or not loadable_motion_paths or not loadable_model_paths:
            raise MApplicationException("サイジングできないファイルセットが含まれているため、処理を中断します\nファイルパスが正しいか確認してください")

        log_queue = multiprocessing.Manager().Queue()

        console_handler = ConsoleQueueHandler(bone_panel.console_ctrl.text_ctrl, log_queue)
        listener = QueueListener(log_queue, console_handler)
        listener.start()

        with ProcessPoolExecutor(max_workers=3) as executor:
            motion_futures = [
                executor.submit(usecase.load_motion, motion_path, self.frame.cache_motions, log_queue)
                for motion_path in loadable_motion_paths
            ]
            model_futures = [
                executor.submit(usecase.load_model, model_path, self.frame.cache_models, log_queue) for model_path in loadable_model_paths
            ]

        for future in as_completed(motion_futures):
            digest, motion = future.result()
            self.frame.cache_motions[digest] = motion

        for future in as_completed(model_futures):
            digest, model = future.result()
            self.frame.cache_models[digest] = model

        listener.stop()

        self.result_data = []

    def output_log(self):
        bone_panel = self.frame.bone_panel
        output_log_path = os.path.join(get_root_dir(), f"{os.path.basename(bone_panel.sizing_sets[0].output_motion_ctrl.path)}.log")
        # 出力されたメッセージを全部出力
        bone_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
