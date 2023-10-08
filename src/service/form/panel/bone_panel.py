import os

import wx

from mlib.core.logger import ConsoleHandler, MLogger
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.widgets.console_ctrl import ConsoleCtrl
from mlib.service.form.widgets.exec_btn_ctrl import ExecButton
from mlib.utils.file_utils import save_histories
from service.form.widgets.bone_set import SizingBoneSet
from service.worker.bone_worker import BoneWorker

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class BonePanel(NotebookPanel):
    def __init__(self, frame: NotebookFrame, tab_idx: int, *args, **kw) -> None:
        super().__init__(frame, tab_idx, *args, **kw)
        self.sizing_sets: list[SizingBoneSet] = []

        self.bone_worker = BoneWorker(frame, self.on_exec_result)

        self._initialize_ui()

        self.on_add_set(wx.EVT_BUTTON)

        self.EnableExec(False)

    def _initialize_ui(self) -> None:
        # ヘッダー -----------------------------
        self.header_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.add_ctrl = wx.Button(self, wx.ID_ANY, __("サイジングセット追加"), wx.DefaultPosition, wx.Size(120, -1))
        self.add_ctrl.SetToolTip(__("サイジングセットを追加できます"))
        self.add_ctrl.Bind(wx.EVT_BUTTON, self.on_add_set)
        self.header_sizer.Add(self.add_ctrl, 0, wx.ALL, 3)

        self.clear_ctrl = wx.Button(self, wx.ID_ANY, __("サイジングセット全削除"), wx.DefaultPosition, wx.Size(120, -1))
        self.clear_ctrl.SetToolTip(__("全てのサイジングセットを削除できます"))
        self.clear_ctrl.Bind(wx.EVT_BUTTON, self.on_clear_set)
        self.header_sizer.Add(self.clear_ctrl, 0, wx.ALL, 3)

        self.root_sizer.Add(self.header_sizer, 0, wx.ALL | wx.ALIGN_RIGHT, 3)

        # ウィンドウ ------------------------------
        self.window = wx.ScrolledWindow(
            self,
            wx.ID_ANY,
            wx.DefaultPosition,
            wx.Size(-1, -1),
            wx.FULL_REPAINT_ON_RESIZE | wx.VSCROLL | wx.HSCROLL,
        )
        self.window.SetScrollRate(5, 5)

        self.window_sizer = wx.BoxSizer(wx.VERTICAL)
        self.set_sizer = wx.BoxSizer(wx.VERTICAL)

        self.window_sizer.Add(self.set_sizer, 0, wx.ALL | wx.EXPAND, 3)
        self.window.SetSizer(self.window_sizer)
        self.root_sizer.Add(self.window, 1, wx.GROW, 0)

        # ボタン -------------------------
        self.btn_sizer = wx.BoxSizer(wx.HORIZONTAL)

        self.exec_btn_ctrl = ExecButton(
            self,
            self,
            __("サイジング実行"),
            __("サイジング実行停止"),
            self.exec,
            250,
            __("サイジングを実行します\nサイジングセットを1件以上設定後、クリックできるようになります"),
        )
        self.exec_btn_ctrl.exec_worker = self.bone_worker
        self.btn_sizer.Add(self.exec_btn_ctrl, 0, wx.ALL, 3)

        self.root_sizer.Add(self.btn_sizer, 0, wx.ALIGN_CENTER | wx.SHAPED, 3)

        # コンソール -----------------
        self.console_ctrl = ConsoleCtrl(self, self.frame, self, rows=150)
        self.console_ctrl.set_parent_sizer(self.root_sizer)

    def on_add_set(self, event: wx.Event) -> None:
        sizing_idx = len(self.sizing_sets)
        sizing_set = SizingBoneSet(self.window, self.frame, self, sizing_idx)

        self.sizing_sets.append(sizing_set)
        self.set_sizer.Add(sizing_set.sizer, 1, wx.GROW, 0)

        self.Enable(True)
        self.fit_window()

    def on_clear_set(self, event: wx.Event) -> None:
        self.window_sizer.Hide(self.set_sizer, recursive=True)
        del self.sizing_sets
        self.sizing_sets = []

    def fit_window(self) -> None:
        self.window.Layout()
        self.window.Fit()
        self.Layout()

    def save_histories(self) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.motion_ctrl.save_path()
            sizing_set.src_model_ctrl.save_path()
            sizing_set.dest_model_ctrl.save_path()

        save_histories(self.frame.histories)

    def exec(self, event: wx.Event) -> None:
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.frame.running_worker = True
        self.save_histories()

        self.Enable(False)
        self.exec_btn_ctrl.Enable(True)
        self.bone_worker.start()

    def on_exec_result(self, result: bool, data: list, elapsed_time: str):
        MLogger.console_handler = ConsoleHandler(self.console_ctrl.text_ctrl)
        self.console_ctrl.write(f"\n----------------\n{elapsed_time}")

        self.Enable(True)
        self.frame.running_worker = False
        self.frame.on_sound()

        logger.info("サイジング完了", decoration=MLogger.Decoration.BOX)

    def Enable(self, enable: bool) -> None:
        for sizing_set in self.sizing_sets:
            sizing_set.Enable(enable)
        self.EnableExec(enable)

    def EnableExec(self, enable: bool) -> None:
        self.exec_btn_ctrl.Enable(enable)
