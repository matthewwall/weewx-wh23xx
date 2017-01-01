# installer for wh23xx driver
# Copyright 2016 Matthew Wall

from setup import ExtensionInstaller

def loader():
    return WH23xxInstaller()

class WH23xxInstaller(ExtensionInstaller):
    def __init__(self):
        super(WH23xxInstaller, self).__init__(
            version="0.9",
            name='wh23xx',
            description='Collect data from wh23xx weather stations',
            author="Matthew Wall",
            author_email="mwall@users.sourceforge.net",
            files=[('bin/user', ['bin/user/wh23xx.py'])]
            )
