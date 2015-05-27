#!/usr/bin/python

import argparse
import itertools
import logging
import sys
import time

import trollius
import trollius as asyncio

sys.path.append('../../legtool/')

import PySide.QtCore as QtCore
import PySide.QtGui as QtGui

from legtool.async import asyncio_qt

import ui_manager_main_window


class Mech(object):
    def __init__(self, item):
        self.ident = 0
        self.name = 'unknown'
        self.hp = 0
        self.item = item

    def update(self):
        self.item.setText('%02X: %s: HP %d' % (self.ident, self.name, self.hp))


class HistoryItem(object):
    def __init__(self, ident, value):
        self.stamp = time.time()
        self.ident = ident
        self.value = value


class State(object):
    def __init__(self):
        self.mechs = []
        self.history = []

    def add_history(self, item):
        assert isinstance(item, HistoryItem)
        self.history.append(item)


class ManagerMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ManagerMainWindow, self).__init__(parent)

        self.ui = ui_manager_main_window.Ui_ManagerMainWindow()
        self.ui.setupUi(self)

        self.ui.mechAddButton.clicked.connect(
            self.handle_mech_add_button)
        self.ui.mechRemoveButton.clicked.connect(
            self.handle_mech_remove_button)

        self.ui.mechListWidget.currentRowChanged.connect(
            self.handle_mech_current_row)

        self.ui.propertiesIdEdit.editingFinished.connect(
            self.handle_mech_properties)
        self.ui.propertiesNameEdit.editingFinished.connect(
            self.handle_mech_properties)
        self.ui.propertiesHpEdit.editingFinished.connect(
            self.handle_mech_properties)

        self.ui.propertiesAddHpButton.clicked.connect(
            self.handle_add_hp_button)
        self.ui.propertiesRemoveHpButton.clicked.connect(
            self.handle_remove_hp_button)
        self.ui.propertiesSetHpButton.clicked.connect(
            self.handle_set_hp_button)

        self.state = State()

    def handle_mech_add_button(self):
        widget = self.ui.mechListWidget
        widget.addItem('')
        item = widget.item(widget.count() - 1)
        self.state.mechs.append(Mech(item))
        self.state.mechs[-1].update()

        widget.setCurrentRow(widget.count() - 1)

    def handle_mech_remove_button(self):
        mech = self._current_mech()
        if mech is None:
            return

        result = QtGui.QMessageBox.question(
            self, 'Remove Mech',
            'Confirm removing mech %02X (%s)' % (mech.ident, mech.name),
            QtGui.QMessageBox.Yes | QtGui.QMessageBox.No)
        if result == QtGui.QMessageBox.No:
            return

        index = self.state.mechs.index(mech)
        del self.state.mechs[index]
        self.ui.mechListWidget.takeItem(index)

        self.handle_mech_current_row()

    def handle_mech_current_row(self):
        row = self.ui.mechListWidget.currentRow()
        if row < 0 or row >= len(self.state.mechs):
            return
        mech = self.state.mechs[row]
        self.ui.propertiesIdEdit.setText('%02X' % mech.ident)
        self.ui.propertiesNameEdit.setText(mech.name)
        self.ui.propertiesHpEdit.setText('%d' % mech.hp)

    def _current_mech(self):
        row = self.ui.mechListWidget.currentRow()
        if row < 0:
            return
        if row >= len(self.state.mechs):
            return
        return self.state.mechs[row]

    def _add_history(self, item):
        self.state.add_history(item)
        self.update_history()

    def handle_mech_properties(self):
        mech = self._current_mech()
        if mech is None:
            return
        mech.ident = int(self.ui.propertiesIdEdit.text(), 16)
        mech.name = self.ui.propertiesNameEdit.text()
        if not self.ui.propertiesHpEdit.isReadOnly():
            newhp = int(self.ui.propertiesHpEdit.text())
            self._add_history(HistoryItem(
                    mech.ident,
                    '(%s) manual HP %d -> %d' % (
                        mech.name, mech.hp, newhp)))
            mech.hp = newhp
            self.ui.propertiesHpEdit.setReadOnly(True)

        mech.update()

    def handle_add_hp_button(self):
        self._change_hp(1)

    def handle_remove_hp_button(self):
        self._change_hp(-1)

    def _change_hp(self, delta):
        mech = self._current_mech()
        if mech is None:
            return
        newhp = mech.hp + delta

        self._add_history(
            HistoryItem(mech.ident,
                        '(%s) manual HP %d -> %d' % (
                    mech.name, mech.hp, newhp)))
        mech.hp = newhp

        mech.update()

        self.handle_mech_current_row()

    def handle_set_hp_button(self):
        mech = self._current_mech()
        if mech is None:
            return

        self.ui.propertiesHpEdit.setReadOnly(False)

    def update_history(self):
        text = ''
        for item in itertools.islice(reversed(self.state.history), 20):
            text += '%s: %02X: %s\n' % (
                time.asctime(time.localtime(item.stamp)),
                item.ident,
                item.value)

        self.ui.historyEdit.setPlainText(text)


def main():
    logging.basicConfig(level=logging.WARN, stream=sys.stdout)

    asyncio.set_event_loop_policy(asyncio_qt.QtEventLoopPolicy())

    app = QtGui.QApplication(sys.argv)
    app.setApplicationName('mjscore_manager')

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-s', '--serial',
                        help='serial port to use')

    args = parser.parse_args()

    manager = ManagerMainWindow()
    manager.show()

    asyncio.get_event_loop().run_forever()


if __name__ == '__main__':
    main()
