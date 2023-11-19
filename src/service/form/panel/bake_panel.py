import os
from datetime import datetime

import wx
from service.worker.bake_worker import BakeWorker

from mlib.core.logger import ConsoleHandler, MLogger
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.widgets.console_ctrl import ConsoleCtrl
from mlib.service.form.widgets.exec_btn_ctrl import ExecButton
from mlib.service.form.widgets.file_ctrl import MPmxFilePickerCtrl, MVmdFilePickerCtrl
from mlib.utils.file_utils import save_histories

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class BakePanel(NotebookPanel):
    def __init__(self, frame: NotebookFrame, tab_idx: int, *args, **kw) -> None:
        super().__init__(frame, tab_idx, *args, **kw)

        self.bake_worker = BakeWorker(self, self.on_exec_result)

        self._initialize_ui()

        self.EnableExec(False)

    def _initialize_ui(self) -> None:
        # ヘッダー -----------------------------
        self.header_sizer = wx.BoxSizer(wx.VERTICAL)

        self.motion_ctrl = MVmdFilePickerCtrl(
            self,
            self.frame,
            self,
            key="vmd",
            title="焼き込み対象モーション",
            is_show_name=True,
            name_spacer=1,
            is_save=False,
            tooltip="IK焼き込みの対象となるVMDモーションデータを指定してください",
            file_change_event=self.on_change_motion,
        )
        self.motion_ctrl.set_parent_sizer(self.header_sizer)

        self.model_ctrl = MPmxFilePickerCtrl(
            self,
            self.frame,
            self,
            key="ik_pmx",
            title="焼き込み元モデル",
            is_show_name=True,
            name_spacer=4,
            is_save=False,
            tooltip="モーションを適用させたいモデルを指定してください\nこのモデルに合わせてモーションのIK部分をIK焼き込みします",
            file_change_event=self.on_change_model_pmx,
        )
        self.model_ctrl.set_parent_sizer(self.header_sizer)

        self.output_motion_ctrl = MVmdFilePickerCtrl(
            self,
            self.frame,
            self,
            title="IK焼き込みモーション出力先",
            is_show_name=False,
            is_save=True,
            tooltip="IK焼き込みモーションの出力ファイルパスです\n任意の値に変更可能です",
        )
        self.output_motion_ctrl.set_parent_sizer(self.header_sizer)

        # ボタン -------------------------
        self.btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.exec_btn_ctrl = ExecButton(
            self,
            self,
            __("IK焼き込み実行"),
            __("IK焼き込み実行停止"),
            self.exec,
            250,
            __("IK焼き込みを実行します\nモデルとモーションを設定後、クリックできるようになります"),
        )
        self.exec_btn_ctrl.exec_worker = self.bake_worker
        self.btn_sizer.Add(self.exec_btn_ctrl, 0, wx.ALL, 3)

        self.root_sizer.Add(self.header_sizer, 1, wx.EXPAND | wx.ALL, 3)
        self.root_sizer.Add(self.btn_sizer, 0, wx.ALIGN_CENTER | wx.SHAPED, 3)

        # コンソール -----------------
        self.console_ctrl = ConsoleCtrl(self, self.frame, self, rows=500)
        self.console_ctrl.set_parent_sizer(self.root_sizer)

    def on_change_model_pmx(self, event: wx.Event) -> None:
        self.model_ctrl.unwrap()
        if self.model_ctrl.read_name():
            self.model_ctrl.read_digest()
            self.create_output_path()
        self.EnableExec(True)

    def on_change_motion(self, event: wx.Event) -> None:
        self.motion_ctrl.unwrap()
        if self.motion_ctrl.read_name():
            self.motion_ctrl.read_digest()
            self.create_output_path()
        self.EnableExec(True)

    def create_output_path(self) -> None:
        if self.motion_ctrl.valid() and self.model_ctrl.valid():
            (
                motion_dir_path,
                motion_file_name,
                motion_file_ext,
            ) = self.motion_ctrl.separated_path
            (
                model_dir_path,
                model_file_name,
                model_file_ext,
            ) = self.model_ctrl.separated_path

            self.output_motion_ctrl.path = os.path.join(
                motion_dir_path,
                "".join(
                    [
                        f"{motion_file_name}_{model_file_name}_{datetime.now():%Y%m%d_%H%M%S}{motion_file_ext}",
                    ]
                ),
            )

    def save_histories(self) -> None:
        self.motion_ctrl.save_path()
        self.model_ctrl.save_path()

        save_histories(self.frame.histories)

    def exec(self, event: wx.Event) -> None:
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.frame.running_worker = True
        self.save_histories()

        self.Enable(False)
        self.exec_btn_ctrl.Enable(True)
        self.bake_worker.start()

    def on_exec_result(self, result: bool, data: list, elapsed_time: str):
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.console_ctrl.write(f"\n----------------\n{elapsed_time}")

        self.Enable(True)
        self.frame.running_worker = False
        self.frame.on_sound()

        logger.info("IK焼き込み完了", decoration=MLogger.Decoration.BOX)

    def Enable(self, enable: bool) -> None:
        self.motion_ctrl.Enable(enable)
        self.output_motion_ctrl.Enable(enable)
        self.model_ctrl.Enable(enable)
        self.EnableExec(enable)

    def EnableExec(self, enable: bool) -> None:
        self.exec_btn_ctrl.Enable(enable)
