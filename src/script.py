'''
This file updates `MazeSolvingAlgos` module; You can place modified files 
in the same directory besides this file and update `parentDir` with path, 
then run this file as administrator and module will be updated by itself.
*Note*: This script runs fine in my Windows 10 machine system calls may differ 
for your own operating system.
'''
import os
import shutil
parentDir = 'Path\\To\\MazeSolvingAlgos\\src\\'
folders = ('build', 'dist', 'MazeSolvingAlgos.egg-info')
for folder in folders:
    fullname = parentDir + folder
    if os.path.isdir(fullname):
        shutil.rmtree(fullname)
os.system('pip uninstall MazeSolvingAlgos ; y')
os.system('python setup.py build')
os.system('python setup.py install')