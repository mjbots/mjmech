#!/usr/bin/python

import re
import os
import sys
import tempfile

filename = sys.argv[1]

IGNORE_RES = [
    re.compile('^<wire .* width="0.0762"'),
    re.compile('^<via .* drill="0.3048"'),
    ]


def ignore(line):
    for re in IGNORE_RES:
        if re.search(line):
            return True
    return False


with tempfile.NamedTemporaryFile(delete=False) as out:
    with open(filename) as infile:
        for line in infile.readlines():
            if ignore(line):
                continue
            out.write(line)
    out.close()

    os.rename(out.name, filename)
