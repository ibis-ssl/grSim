/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "statuswidget.h"

CStatusWidget::CStatusWidget(CStatusPrinter* _statusPrinter)
{    
    statusPrinter = _statusPrinter;
    logTime.start();

    this->setAllowedAreas(Qt::BottomDockWidgetArea);
    this->setFeatures(QDockWidget::NoDockWidgetFeatures);

    statusText = new QTextEdit(this);
    statusText->setReadOnly(true);
    statusText->document()->setMaximumBlockCount(2000);
    titleLbl = new QLabel(tr("Messages"));


    this->setWidget(statusText);
    this->setTitleBarWidget(titleLbl);

}

void CStatusWidget::write(QString str, QColor color)
{
    QTextCursor cursor(statusText->document());
    cursor.movePosition(QTextCursor::End);
    QTextCharFormat format;
    format.setForeground(color);
    cursor.insertText(str + QLatin1Char('\n'), format);
    statusText->setTextCursor(cursor);
}

void CStatusWidget::flushPendingMessages()
{
    if (statusPrinter == nullptr || statusPrinter->textBuffer.isEmpty())
    {
        return;
    }

    constexpr int kMaxMessagesPerFlush = 64;
    int processed = 0;
    QTextCursor cursor(statusText->document());
    cursor.movePosition(QTextCursor::End);
    QTextCharFormat format;

    cursor.beginEditBlock();
    while (!statusPrinter->textBuffer.isEmpty() && processed < kMaxMessagesPerFlush)
    {
        const CStatusText text = statusPrinter->textBuffer.dequeue();
        format.setForeground(text.color);
        cursor.insertText(text.text + QLatin1Char('\n'), format);
        processed++;
    }
    cursor.endEditBlock();
    statusText->setTextCursor(cursor);
}
