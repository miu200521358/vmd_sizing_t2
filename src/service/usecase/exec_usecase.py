import os

from mlib.core.logger import MLogger

logger = MLogger(os.path.basename(__file__), level=1)
__ = logger.get_text


class ExecUsecase:
    pass
