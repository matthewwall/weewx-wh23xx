# installer for wh2300 driver
# Copyright 2016 Matthew Wall

from setup import ExtensionInstaller

def loader():
    return WH2300Installer()

class WH2300Installer(ExtensionInstaller):
    def __init__(self):
        super(WH2300Installer, self).__init__(
            version="0.3",
            name='wh2300',
            description='Collect data from wh2300 weather stations',
            author="Matthew Wall",
            author_email="mwall@users.sourceforge.net",
            files=[('bin/user', ['bin/user/wh2300.py'])]
            )
