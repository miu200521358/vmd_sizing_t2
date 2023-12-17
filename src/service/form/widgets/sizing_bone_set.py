import os
from datetime import datetime

import wx
from mlib.core.logger import MLogger
from mlib.service.form.notebook_frame import NotebookFrame
from mlib.service.form.notebook_panel import NotebookPanel
from mlib.service.form.widgets.file_ctrl import MPmxFilePickerCtrl, MVmdFilePickerCtrl

logger = MLogger(os.path.basename(__file__))
__ = logger.get_text


class SizingBoneSet:
    def __init__(
        self,
        window: wx.Window,
        frame: NotebookFrame,
        panel: NotebookPanel,
        sizing_idx: int,
    ) -> None:
        self.window = window
        self.frame = frame
        self.panel = panel
        self.sizing_idx = sizing_idx
        self.selected_stance_correct: list[int] = []

        self._initialize_ui()

    def _initialize_ui(self) -> None:
        self.sizer = wx.BoxSizer(wx.VERTICAL)

        self.background_color = (
            wx.Colour("LIGHT GREY")
            if 0 == self.sizing_idx % 2
            else wx.Colour("LIGHT BLUE")
        )
        self.box = wx.StaticBox(self.window, wx.ID_ANY, f"【No.{self.sizing_idx + 1}】")
        self.box.SetBackgroundColour(self.background_color)

        self.file_sizer = wx.StaticBoxSizer(self.box, orient=wx.VERTICAL)

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

        # # 足IKオフセット ------------------------
        # self.offset_sizer = wx.BoxSizer(wx.HORIZONTAL)

        # self.leg_ik_offset_label = wx.StaticText(self.window, wx.ID_ANY, __("足IKオフセット"))
        # self.leg_ik_offset_label.SetBackgroundColour(self.background_color)
        # self.offset_sizer.Add(self.leg_ik_offset_label, 0, wx.ALL, 2)

        # self.leg_ik_offset_slider = FloatSliderCtrl(
        #     parent=self.window,
        #     value=0,
        #     min_value=-5,
        #     max_value=5,
        #     increment=0.01,
        #     spin_increment=0.01,
        #     border=3,
        #     size=wx.Size(270, -1),
        #     change_event=None,
        #     tooltip=__("モーションで足が重なってしまう場合などで、足を少し開き気味にするなどのオフセットを追加できます"),
        # )
        # self.leg_ik_offset_slider._slider.SetBackgroundColour(self.background_color)
        # self.offset_sizer.Add(self.leg_ik_offset_slider.sizer, 0, wx.ALL, 2)

        # self.config_sizer.Add(self.offset_sizer, 0, wx.ALL, 2)

        # self.box_sizer.Add(self.config_sizer, 5, wx.ALL, 0)
        self.sizer.Add(self.file_sizer, 1, wx.EXPAND | wx.ALL, 0)

    def on_change_src_model_pmx(self, event: wx.Event) -> None:
        self.src_model_ctrl.unwrap()
        if self.src_model_ctrl.read_name():
            self.src_model_ctrl.read_digest()
            self.create_output_path()
        self.panel.EnableExec(True)

    def on_change_dest_model_pmx(self, event: wx.Event) -> None:
        self.dest_model_ctrl.unwrap()
        if self.dest_model_ctrl.read_name():
            self.dest_model_ctrl.read_digest()
            self.create_output_path()
        self.panel.EnableExec(True)

    def on_change_motion(self, event: wx.Event) -> None:
        self.motion_ctrl.unwrap()
        if self.motion_ctrl.read_name():
            self.motion_ctrl.read_digest()
            self.create_output_path()
        self.panel.EnableExec(True)

    def create_output_path(self) -> None:
        if (
            self.motion_ctrl.valid()
            and self.src_model_ctrl.valid()
            and self.dest_model_ctrl.valid()
        ):
            (
                motion_dir_path,
                motion_file_name,
                motion_file_ext,
            ) = self.motion_ctrl.separated_path
            # src_model_dir_path, src_model_file_name, src_model_file_ext = self.src_model_ctrl.separated_path
            (
                dest_model_dir_path,
                dest_model_file_name,
                dest_model_file_ext,
            ) = self.dest_model_ctrl.separated_path

            sizing_types: list[str] = []
            if self.panel.integrate_root_check_ctrl.GetValue():
                sizing_types.append("I")
            if self.panel.integrate_waist_check_ctrl.GetValue():
                sizing_types.append("W")
            # if self.panel.stance_lower_check_ctrl.GetValue():
            #     sizing_types.append("L")
            if self.panel.twist_check_ctrl.GetValue():
                sizing_types.append("T")
            if self.panel.align_arm_check_ctrl.GetValue():
                sizing_types.append("P")
            if self.panel.integrate_toe_ik_check_ctrl.GetValue():
                sizing_types.append("O")
            self.output_motion_ctrl.path = os.path.join(
                motion_dir_path,
                "".join(
                    [
                        f"{motion_file_name}_{dest_model_file_name}_{''.join(sizing_types)}",
                        f"{('_' if sizing_types else '')}{datetime.now():%Y%m%d_%H%M%S}{motion_file_ext}",
                    ]
                ),
            )

    def get_loadable_path(self) -> tuple[bool, list[str], list[str], list[str]]:
        logger.info(
            "【No.{i}】読み込み開始",
            i=self.sizing_idx + 1,
            decoration=MLogger.Decoration.LINE,
        )
        loadable_motion_paths: list[str] = []
        loadable_src_model_paths: list[str] = []
        loadable_dest_model_paths: list[str] = []
        can_load: bool = True
        is_check: bool = False

        if (
            self.motion_ctrl.path
            or self.src_model_ctrl.path
            or self.dest_model_ctrl.path
        ):
            is_check = True

        if is_check:
            if not self.motion_ctrl.path:
                logger.error("サイジング対象モーションパスが指定されていません。")
                can_load = False
            elif not self.motion_ctrl.valid():
                logger.error("有効なサイジング対象モーションが指定されていません。")
                can_load = False

            if not self.src_model_ctrl.path:
                logger.error("モーション作成元モデルが指定されていません。")
                can_load = False
            elif not self.src_model_ctrl.valid():
                logger.error("有効なモーション作成元モデルが指定されていません。")
                can_load = False

            if not self.dest_model_ctrl.path:
                logger.error("サイジング先モデルが指定されていません。")
                can_load = False
            elif not self.dest_model_ctrl.valid():
                logger.error("有効なサイジング先モデルが指定されていません。")
                can_load = False

        if can_load and (
            not self.output_motion_ctrl.path
            or (self.output_motion_ctrl.path and not self.output_motion_ctrl.valid())
        ):
            logger.warning("出力ファイルパスが有効なパスではないため、デフォルトの出力ファイルパスを再設定します。")
            self.create_output_path()

        if can_load:
            if self.motion_ctrl.path:
                loadable_motion_paths.append(self.motion_ctrl.path)
            if self.src_model_ctrl.path:
                loadable_src_model_paths.append(self.src_model_ctrl.path)
            if self.dest_model_ctrl.path:
                loadable_dest_model_paths.append(self.dest_model_ctrl.path)

        return (
            can_load,
            loadable_motion_paths,
            loadable_src_model_paths,
            loadable_dest_model_paths,
        )

    def Enable(self, enable: bool) -> None:
        self.motion_ctrl.Enable(enable)
        self.src_model_ctrl.Enable(enable)
        self.dest_model_ctrl.Enable(enable)
        self.output_motion_ctrl.Enable(enable)
