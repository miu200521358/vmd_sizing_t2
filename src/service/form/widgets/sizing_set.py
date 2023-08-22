import os
from datetime import datetime

import wx

from mlib.service.form.widgets.file_ctrl import MPmxFilePickerCtrl
from mlib.service.form.widgets.file_ctrl import MVmdFilePickerCtrl
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.core.logger import MLogger


logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class SizingSet:
    def __init__(self, window: wx.Window, frame: NotebookFrame, panel: NotebookPanel, sizing_idx: int) -> None:
        self.window = window
        self.frame = frame
        self.panel = panel
        self.sizing_idx = sizing_idx

        self._initialize_ui()

    def _initialize_ui(self) -> None:

        self.sizer = wx.BoxSizer(wx.VERTICAL)

        self.background_color = wx.Colour("LIGHT GREY") if 0 == self.sizing_idx % 2 else wx.Colour("LIGHT BLUE")
        self.box = wx.StaticBox(self.window, wx.ID_ANY, f"No.{self.sizing_idx + 1}")
        self.box.SetBackgroundColour(self.background_color)

        self.box_sizer = wx.StaticBoxSizer(self.box, orient=wx.HORIZONTAL)

        # ----------------------------------
        self.file_sizer = wx.BoxSizer(wx.VERTICAL)

        self.motion_ctrl = MVmdFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="vmd",
            title="サイジング対象モーション",
            is_show_name=True,
            name_spacer=1,
            is_save=False,
            tooltip="サイジングの対象となるVMDモーションデータを指定してください",
            file_change_event=self.on_change_motion,
        )
        self.motion_ctrl.set_parent_sizer(self.file_sizer)
        self.motion_ctrl.set_color(self.background_color)

        self.src_model_ctrl = MPmxFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="org_pmx",
            title="モーション作成元モデル",
            is_show_name=True,
            name_spacer=2,
            is_save=False,
            tooltip="モーションを作成した時に使用されたトレースモデル（或いは類似モデル）を指定してください",
            file_change_event=self.on_change_src_model_pmx,
        )
        self.src_model_ctrl.set_parent_sizer(self.file_sizer)
        self.src_model_ctrl.set_color(self.background_color)

        self.dest_model_ctrl = MPmxFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="rep_pmx",
            title="サイジング先モデル",
            is_show_name=True,
            name_spacer=4,
            is_save=False,
            tooltip="モーションを適用させたいモデルを指定してください\nこのモデルに合わせてモーションをサイジングします",
            file_change_event=self.on_change_dest_model_pmx,
        )
        self.dest_model_ctrl.set_parent_sizer(self.file_sizer)
        self.dest_model_ctrl.set_color(self.background_color)

        self.output_motion_ctrl = MVmdFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            title="サイジングモーション出力先",
            is_show_name=False,
            is_save=True,
            tooltip="サイジングモーションの出力ファイルパスです\n任意の値に変更可能です",
        )
        self.output_motion_ctrl.set_parent_sizer(self.file_sizer)
        self.output_motion_ctrl.set_color(self.background_color)

        self.camera_model_ctrl = MPmxFilePickerCtrl(
            self.window,
            self.frame,
            self.panel,
            key="camera_pmx",
            title="カメラモーション作成元モデル",
            is_show_name=True,
            name_spacer=0,
            is_save=False,
            tooltip="カメラモーションを作成した時に使用されたモデル（或いは類似モデル）を指定してください（任意）",
            file_change_event=self.on_change_camera_model_pmx,
        )
        self.camera_model_ctrl.set_parent_sizer(self.file_sizer)
        self.camera_model_ctrl.set_color(self.background_color)

        self.box_sizer.Add(self.file_sizer, 6, wx.ALL, 0)

        # ----------------------------------
        self.config_sizer = wx.BoxSizer(wx.VERTICAL)

        # 位置合わせ
        self.align_check_ctrl = wx.CheckBox(self.window, wx.ID_ANY, __("位置合わせ"), wx.DefaultPosition, wx.DefaultSize, 0)
        self.align_check_ctrl.SetBackgroundColour(self.background_color)
        self.config_sizer.Add(self.align_check_ctrl, 0, wx.ALL, 0)

        # 足

        # モーフ

        self.box_sizer.Add(self.config_sizer, 4, wx.ALL, 0)

        self.sizer.Add(self.box_sizer, 1, wx.EXPAND | wx.ALL, 0)

    def on_change_src_model_pmx(self, event: wx.Event) -> None:
        self.src_model_ctrl.unwrap()
        if self.src_model_ctrl.read_name():
            self.src_model_ctrl.read_digest()
            self.create_output_path()
        if not self.src_model_ctrl.data:
            self.frame.Enable(False)

    def on_change_dest_model_pmx(self, event: wx.Event) -> None:
        self.dest_model_ctrl.unwrap()
        if self.dest_model_ctrl.read_name():
            self.dest_model_ctrl.read_digest()
            self.create_output_path()
        if not self.dest_model_ctrl.data:
            self.frame.Enable(False)

    def on_change_camera_model_pmx(self, event: wx.Event) -> None:
        self.camera_model_ctrl.unwrap()
        if self.camera_model_ctrl.read_name():
            self.camera_model_ctrl.read_digest()
            self.create_output_path()
        if not self.camera_model_ctrl.data:
            self.frame.Enable(False)

    def on_change_motion(self, event: wx.Event) -> None:
        self.motion_ctrl.unwrap()
        if self.motion_ctrl.read_name():
            self.motion_ctrl.read_digest()
        if not self.motion_ctrl.data:
            self.frame.Enable(False)

    def create_output_path(self) -> None:
        if self.motion_ctrl.valid() and self.src_model_ctrl.valid() and self.dest_model_ctrl.valid():
            motion_dir_path, motion_file_name, motion_file_ext = self.motion_ctrl.separated_path
            # src_model_dir_path, src_model_file_name, src_model_file_ext = self.src_model_ctrl.separated_path
            dest_model_dir_path, dest_model_file_name, dest_model_file_ext = self.dest_model_ctrl.separated_path

            sizing_types: list[str] = []
            self.output_motion_ctrl.path = os.path.join(
                motion_dir_path,
                ''.join(
                    [
                        f"{motion_file_name}_{dest_model_file_name}_{''.join(sizing_types)}",
                        f"{('_' if sizing_types else '')}{datetime.now():%Y%m%d_%H%M%S}{motion_file_ext}"
                    ]
                )
            )
