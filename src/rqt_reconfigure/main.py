#!/usr/bin/env python3

import sys

from rqt_gui.main import Main

def main():
    plugin = 'rqt_reconfigure.param_plugin.ParamPlugin'
    sys.exit(Main().main(sys.argv, standalone=plugin))

if __name__ == '__main__':
    main()
