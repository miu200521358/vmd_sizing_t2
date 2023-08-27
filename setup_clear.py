import os
import shutil
from glob import glob

if os.path.exists("build"):
    shutil.rmtree("build")

for source in glob("src/**/*.pyd", recursive=True):
    print(f"remove {source}")
    os.remove(source)
