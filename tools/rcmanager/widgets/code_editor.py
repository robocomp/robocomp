#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  -----------------------
#  -----  rcmanager  -----
#  -----------------------
#  An ICE component manager.
#
#    Copyright (C) 2009-2015 by RoboLab - University of Extremadura
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#

#
# CODE BEGINS
#

from PyQt4.QtCore import QRegExp
from PyQt4.QtGui import QTextCursor, QTextDocument, QTextEdit, QColor, QFontMetrics, QFont

"""
# This will takes care about the code editing part of the software
"""
class CodeEditor(object):
    @staticmethod
    def get_code_editor(parent):
        try:
            from PyQt4 import Qsci
            class ScintillaCodeEditor(Qsci.QsciScintilla):  # For the dynamic code editing (Widget )
                def __init__(self, parent=None):
                    super(ScintillaCodeEditor, self).__init__(parent)

                    # Setting default font
                    self.font = QFont()
                    self.font.setFamily('Courier')
                    self.font.setFixedPitch(True)
                    self.font.setPointSize(10)
                    self.setFont(self.font)
                    self.setMarginsFont(self.font)

                    # Margin 0 is used for line numbers
                    fontmetrics = QFontMetrics(self.font)
                    self.setMarginsFont(self.font)
                    self.setMarginWidth(0, fontmetrics.width("0000") + 6)
                    self.setMarginLineNumbers(0, True)
                    self.setMarginsBackgroundColor(QColor("#cccccc"))

                    # BraceMatching
                    self.setBraceMatching(Qsci.QsciScintilla.SloppyBraceMatch)

                    # Current line visible with special background color
                    self.setCaretLineVisible(True)
                    self.setCaretLineBackgroundColor(QColor("#ffe4e4"))

                    # Setting xml lexer
                    lexer = Qsci.QsciLexerXML()
                    lexer.setDefaultFont(self.font)
                    self.setLexer(lexer)
                    self.SendScintilla(Qsci.QsciScintilla.SCI_STYLESETFONT, 1, 'Courier')

                def on_margin_clicked(self, nmargin, nline, modifiers):
                    # Toggle marker for the line the margin was clicked on
                    if self.markersAtLine(nline) != 0:
                        self.markerDelete(nline, self.ARROW_MARKER_NUM)
                    else:
                        self.markerAdd(nline, self.ARROW_MARKER_NUM)

            return ScintillaCodeEditor(parent)
        except:
            class TextEditCodeEditor(QTextEdit):

                def __init__(self, parent=None):
                    super(TextEditCodeEditor, self).__init__(parent)
                    # Setting default font
                    self.font = QFont()
                    self.font.setFamily('Courier')
                    self.font.setFixedPitch(True)
                    self.font.setPointSize(10)
                    self.setFont(self.font)

                def text(self):
                    return self.toPlainText()

                def getCursorPosition(self):
                    cursor = self.textCursor()
                    line = cursor.blockNumber()
                    column = cursor.columnNumber()
                    return (line, column)

                def findFirst(self, expr, regexp = False, case_sensitive = True, whole_word = True, wrap = True):
                    search_flags = 0
                    if case_sensitive:
                        search_flags |= QTextDocument.FindCaseSensitively
                    if whole_word:
                        search_flags |= QTextDocument.FindWholeWords
                    text_to_find = QRegExp(expr)
                    text_to_find.setPatternSyntax(QRegExp.Wildcard)
                    find_result = self.document().find(text_to_find, search_flags)
                    self.setTextCursor(find_result)

                def insertAt(self, string, line, column):
                    document = self.document()
                    cursor = self.textCursor()
                    line = document.findBlockByLineNumber(line)
                    cursor = QTextCursor(line)
                    cursor.movePosition(QTextCursor.NextCharacter, QTextCursor.MoveAnchor, column)
                    self.setTextCursor(cursor)
                    self.insertPlainText(string)

            return TextEditCodeEditor(parent)
