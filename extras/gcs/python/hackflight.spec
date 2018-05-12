# -*- mode: python -*-

# hackflight.spec : Pyinstaller script for Hackflight
#
# Copyright (C) Simon D. Levy 2016
#
# This file is part of Hackflight.
#
# Hackflight is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as 
# published by the Free Software Foundation, either version 3 of the 
# License, or (at your option) any later version.
# This code is distributed in the hope that it will be useful,     
# but WITHOUT ANY WARRANTY without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License 
# along with this code.  If not, see <http:#www.gnu.org/licenses/>.


block_cipher = None

a = Analysis(['hackflight.py'],
             pathex=['C:\\Users\\levys\\Documents\\Arduino\\libraries\\Hackflight\\extras\\gcs\\python'],
             binaries=[],
             datas=[('media/'+name, 'media') for name in ['icon.xbm', 'splash.gif', 'motors.gif', 'motors1.gif', 'motors2.gif', 'motors3.gif', 'motors4.gif']],
             hiddenimports=[],
             hookspath=[],
             runtime_hooks=[],
             excludes=[],
             win_no_prefer_redirects=False,
             win_private_assemblies=False,
             cipher=block_cipher)
pyz = PYZ(a.pure, a.zipped_data,
             cipher=block_cipher)
exe = EXE(pyz,
          a.scripts,
          exclude_binaries=True,
          name='hackflight',
          debug=False,
          strip=False,
          upx=True,
          console=False )
coll = COLLECT(exe,
               a.binaries,
               a.zipfiles,
               a.datas,
               strip=False,
               upx=True,
               name='hackflight')

