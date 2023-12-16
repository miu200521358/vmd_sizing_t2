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
from mlib.pmx.pmx_collection import PmxModel
from mlib.service.base_worker import BaseWorker
from mlib.service.form.base_panel import BasePanel
from mlib.utils.file_utils import get_root_dir
from mlib.vmd.vmd_tree import VmdBoneFrameTrees
from service.form.panel.sizing_panel import SizingPanel
from service.form.widgets.sizing_bone_set import SizingBoneSet
from service.usecase.arm_stance_usecase import ArmStanceUsecase
from service.usecase.arm_twist_usecase import ArmTwistUsecase
from service.usecase.bone_names import BoneNames
from service.usecase.integrate_toe_ik_usecase import IntegrateToeIkUsecase
from service.usecase.integrate_usecase import IntegrateUsecase
from service.usecase.io_usecase import IoUsecase
from service.usecase.move_usecase import MoveUsecase
from service.usecase.stance_lower_usecase import StanceLowerUsecase

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class SizingWorker(BaseWorker):
    def __init__(self, panel: BasePanel, result_event: wx.Event) -> None:
        super().__init__(panel, result_event)
        # ワーカー固定(中で細分化していくため)
        self.max_worker = (
            1
            if self.frame.is_saving
            else min(4, max(1, int(min(32, (os.cpu_count() or 0) + 4) / 4)))
        )

    def thread_execute(self):
        sizing_panel: SizingPanel = self.frame.sizing_panel

        # まずは読み込み
        all_src_matrixes = self.load()

        # 移動補正
        self.sizing_move()

        # 全親統合
        if sizing_panel.integrate_root_check_ctrl.GetValue():
            self.integrate_root()

        # 腰統合
        if sizing_panel.integrate_waist_check_ctrl.GetValue():
            self.integrate_waist()

        # つま先IK統合
        if sizing_panel.integrate_toe_ik_check_ctrl.GetValue():
            self.integrate_toe_ik(all_src_matrixes)

        # # 下半身補正
        # if sizing_panel.stance_lower_check_ctrl.GetValue():
        #     self.stance_lower()

        # 腕スタンス補正
        self.sizing_arm_stance()

        # # 腕位置合わせ
        # if sizing_panel.align_check_ctrl.GetValue():
        #     self.sizing_arm_align()

        # 捩り分散
        if sizing_panel.twist_check_ctrl.GetValue():
            self.sizing_arm_twist()

        # 保存
        self.save()

        self.result_data = []

    def stance_lower(self) -> None:
        """下半身補正"""
        logger.info("下半身補正", decoration=MLogger.Decoration.BOX)

        usecase = StanceLowerUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        # 初期位置を取得する
        initial_matrixes: dict[tuple[int, bool, str], VmdBoneFrameTrees] = {}

        with ThreadPoolExecutor(
            thread_name_prefix="lower_initial", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                # キーフレは元と変わっている可能性があるので、先モーションのキーフレを基準とする
                fnos = usecase.get_fnos(sizing_set.output_motion_ctrl.data)

                futures.append(
                    executor.submit(
                        usecase.get_initial_matrixes,
                        sizing_set.sizing_idx,
                        True,
                        sizing_set.src_model_ctrl.data,
                        sizing_set.motion_ctrl.data,
                        fnos,
                    )
                )

                futures.append(
                    executor.submit(
                        usecase.get_initial_matrixes,
                        sizing_set.sizing_idx,
                        False,
                        sizing_set.dest_model_ctrl.data,
                        sizing_set.output_motion_ctrl.data,
                        fnos,
                    )
                )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, is_src, matrixes = future.result()
                initial_matrixes[(sizing_idx, is_src)] = matrixes

        ik_models: dict[tuple[int, bool], PmxModel] = {}

        # IKセットアップする
        with ThreadPoolExecutor(
            thread_name_prefix="ik_setup", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []

            for sizing_set in sizing_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.src_model_ctrl.data,
                    sizing_set.dest_model_ctrl.data,
                    sizing_panel.stance_lower_check_ctrl.GetValue(),
                ):
                    futures.append(
                        executor.submit(
                            usecase.setup_model_ik,
                            sizing_set.sizing_idx,
                            False,
                            sizing_set.dest_model_ctrl.data,
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()

                sizing_idx, is_src, ik_model = future.result()
                ik_models[(sizing_idx, is_src)] = ik_model

        # 下半身補正を実行する
        with ThreadPoolExecutor(
            thread_name_prefix="sizing_stance_lower", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.src_model_ctrl.data,
                    sizing_set.dest_model_ctrl.data,
                    sizing_panel.stance_lower_check_ctrl.GetValue(),
                ):
                    futures.append(
                        executor.submit(
                            usecase.sizing_stance_lower,
                            sizing_set.sizing_idx,
                            sizing_set.src_model_ctrl.data,
                            ik_models[(sizing_idx, False)],
                            sizing_set.output_motion_ctrl.data,
                            initial_matrixes[(sizing_idx, True)],
                            initial_matrixes[(sizing_idx, False)],
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

    def sizing_arm_twist(self) -> None:
        logger.info("捩り分散", decoration=MLogger.Decoration.BOX)

        usecase = ArmTwistUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        # 初期位置を取得する
        dest_matrixes: dict[tuple[int, str], VmdBoneFrameTrees] = {}

        with ThreadPoolExecutor(
            thread_name_prefix="arm_initial", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                for direction in ("右", "左"):
                    futures.append(
                        executor.submit(
                            usecase.get_dest_matrixes,
                            sizing_set.sizing_idx,
                            sizing_set.dest_model_ctrl.data,
                            sizing_set.output_motion_ctrl.data,
                            direction,
                            sizing_panel.twist_middle_check_ctrl.GetValue(),
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()
                sizing_idx, direction, matrixes = future.result()
                dest_matrixes[(sizing_idx, direction)] = matrixes

        ik_dest_models: dict[int, PmxModel] = {}

        # IKセットアップする
        with ThreadPoolExecutor(
            thread_name_prefix="ik_setup", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []

            for sizing_set in sizing_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.dest_model_ctrl.data,
                    sizing_panel.twist_check_ctrl.GetValue(),
                ):
                    futures.append(
                        executor.submit(
                            usecase.setup_model_ik,
                            sizing_set.sizing_idx,
                            sizing_set.dest_model_ctrl.data,
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

            for future in as_completed(futures):
                if future.exception():
                    raise future.exception()

                sizing_idx, ik_model = future.result()
                ik_dest_models[sizing_idx] = ik_model

        # 捩り分散を実行する
        with ThreadPoolExecutor(
            thread_name_prefix="arm_twist", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.dest_model_ctrl.data,
                    sizing_panel.twist_check_ctrl.GetValue(),
                ):
                    futures.append(
                        executor.submit(
                            usecase.sizing_arm_twist,
                            sizing_set.sizing_idx,
                            ik_dest_models[sizing_idx],
                            sizing_set.output_motion_ctrl.data,
                            dest_matrixes,
                            sizing_panel.twist_middle_check_ctrl.GetValue(),
                            sizing_panel.twist_middle_threshold_slider.GetValue(),
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

    def sizing_arm_stance(self):
        """腕スタンス補正"""
        logger.info("腕スタンス補正", decoration=MLogger.Decoration.BOX)

        usecase = ArmStanceUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        with ThreadPoolExecutor(
            thread_name_prefix="arm_stance", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
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

    def integrate_root(self):
        """全ての親統合"""

        logger.info("全ての親統合", decoration=MLogger.Decoration.BOX)

        self.integrate(BoneNames.root())

    def integrate_waist(self):
        """腰統合"""

        logger.info("腰統合", decoration=MLogger.Decoration.BOX)

        self.integrate(BoneNames.waist())

    def integrate_toe_ik(self, all_src_matrixes: dict[int, VmdBoneFrameTrees]):
        """つま先IK統合"""

        logger.info("つま先IK統合", decoration=MLogger.Decoration.BOX)

        usecase = IntegrateToeIkUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        # つま先IK統合を実行する
        with ThreadPoolExecutor(
            thread_name_prefix="sizing_toe_ik", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                if usecase.validate(
                    sizing_set.sizing_idx,
                    sizing_set.src_model_ctrl.data,
                    sizing_set.dest_model_ctrl.data,
                ):
                    futures.append(
                        executor.submit(
                            usecase.sizing_integrate_toe_ik,
                            sizing_set.sizing_idx,
                            sizing_set.src_model_ctrl.data,
                            sizing_set.dest_model_ctrl.data,
                            sizing_set.output_motion_ctrl.data,
                            all_src_matrixes[sizing_set.sizing_idx],
                        )
                    )

            wait(futures, return_when=FIRST_EXCEPTION)

    def integrate(self, bone_name: str):
        """ボーン統合"""

        logger.info("{b}統合", b=bone_name, decoration=MLogger.Decoration.BOX)

        usecase = IntegrateUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        with ThreadPoolExecutor(
            thread_name_prefix="integrate", max_workers=self.max_worker
        ) as executor:
            futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                futures.append(
                    executor.submit(
                        usecase.integrate,
                        sizing_set.sizing_idx,
                        sizing_set.dest_model_ctrl.data,
                        sizing_set.output_motion_ctrl.data,
                        bone_name,
                    )
                )

            wait(futures, return_when=FIRST_EXCEPTION)

    def sizing_move(self):
        """移動補正"""

        logger.info("移動補正", decoration=MLogger.Decoration.BOX)

        usecase = MoveUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        xz_leg_ratios: list[float] = []
        y_leg_ratios: list[float] = []
        center_offsets: list[MVector3D] = []

        for sizing_one_set in sizing_panel.sizing_sets:
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
                sizing_panel.sizing_sets, xz_leg_ratios, y_leg_ratios, center_offsets
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

    def save(self):
        """サイジング結果保存"""
        usecase = IoUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

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
                for sizing_set in sizing_panel.sizing_sets
            ]

        for future in as_completed(futures):
            if future.exception():
                raise future.exception()

    def load(self) -> dict[int, VmdBoneFrameTrees]:
        """データ読み込み"""
        usecase = IoUsecase()
        sizing_panel: SizingPanel = self.frame.sizing_panel

        loadable_motion_paths: list[str] = []
        loadable_src_model_paths: list[str] = []
        loadable_dest_model_paths: list[str] = []
        can_load: bool = True

        for sizing_one_set in sizing_panel.sizing_sets:
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

        # まずは元モデルとモーションの取得を行う
        wait(src_model_futures, return_when=FIRST_EXCEPTION)
        wait(motion_futures, return_when=FIRST_EXCEPTION)

        for future in as_completed(motion_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, digest, original_motion, motion = future.result()
            self.frame.cache_motions[digest] = original_motion
            sizing_panel.sizing_sets[sizing_idx].motion_ctrl.data = original_motion
            sizing_panel.sizing_sets[sizing_idx].output_motion_ctrl.data = motion

        for future in as_completed(src_model_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, digest, original_model, model = future.result()
            self.frame.cache_models[digest] = original_model
            sizing_panel.sizing_sets[sizing_idx].src_model_ctrl.data = model

        with ThreadPoolExecutor(
            thread_name_prefix="src_initial", max_workers=self.max_worker
        ) as executor:
            matrix_futures: list[Future] = []
            for sizing_set in sizing_panel.sizing_sets:
                matrix_futures.append(
                    executor.submit(
                        usecase.get_src_matrixes,
                        sizing_set.sizing_idx,
                        sizing_set.src_model_ctrl.data,
                        sizing_set.motion_ctrl.data,
                    )
                )

        for future in as_completed(dest_model_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, digest, original_model, model = future.result()
            self.frame.cache_models[digest] = original_model
            sizing_panel.sizing_sets[sizing_idx].dest_model_ctrl.data = model

        all_src_matrixes: dict[int, VmdBoneFrameTrees] = {}

        for future in as_completed(matrix_futures):
            if future.exception():
                raise future.exception()
            sizing_idx, src_matrixes = future.result()
            all_src_matrixes[sizing_idx] = src_matrixes

        return all_src_matrixes

    def output_log(self):
        sizing_panel: SizingPanel = self.frame.sizing_panel
        output_log_path = os.path.join(
            get_root_dir(),
            f"{os.path.basename(sizing_panel.sizing_sets[0].output_motion_ctrl.path)}.log",
        )
        # 出力されたメッセージを全部出力
        sizing_panel.console_ctrl.text_ctrl.SaveFile(filename=output_log_path)
