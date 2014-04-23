# Copyright 2014 Josh Pieper, jjp@pobox.com.  All rights reserved.

def restore_combo(config, group, combo, name):
    if not config.has_option(group, name):
        return

    value = config.get(group, name)
    for i in range(combo.count()):
        if combo.itemText(i) == value:
            combo.setCurrentIndex(i)
            return
