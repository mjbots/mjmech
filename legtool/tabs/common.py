# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

class BoolContext(object):
    def __init__(self, ):
        self.value = False

    def __enter__(self):
        self.value = True

    def __exit__(self, type, value, traceback):
        self.value = False
