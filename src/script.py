import os
import shutil
parentDir = 'D:\\PyModulesByCPP\\github\\MazeSolvingAlgos\\src\\'
folders = ('build', 'dist', 'MazeSolvingAlgos.egg-info')
for folder in folders:
    fullname = parentDir + folder
    if os.path.isdir(fullname):
        shutil.rmtree(fullname)
os.system('pip uninstall MazeSolvingAlgos ; y')
os.system('python setup.py build')
os.system('python setup.py install')