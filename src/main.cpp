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
#include <QtWidgets/QApplication>
#include <QByteArray>
#include <cstdio>
#include "mainwindow.h"

int main(int argc, char *argv[])
{
    std::locale::global( std::locale( "" ) );

#ifdef HAVE_LINUX
    // QGLWidget is legacy and may behave poorly on native Wayland compositors.
    // Prefer XWayland automatically when available, unless user overrides it.
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")
        && qEnvironmentVariableIsEmpty("GRSIM_ALLOW_NATIVE_WAYLAND"))
    {
        const QByteArray sessionType = qgetenv("XDG_SESSION_TYPE").toLower();
        if (sessionType == QByteArrayLiteral("wayland")
            && !qEnvironmentVariableIsEmpty("DISPLAY"))
        {
            qputenv("QT_QPA_PLATFORM", QByteArrayLiteral("xcb"));
            std::fprintf(stderr,
                         "grSim: Wayland session detected. Using XWayland (QT_QPA_PLATFORM=xcb).\n");
        }
    }
#endif
    
    QCoreApplication::setOrganizationName("Parsian");
    QCoreApplication::setOrganizationDomain("parsian-robotics.com");
    QCoreApplication::setApplicationName("grSim");
    QApplication a(argc, argv);

    QCommandLineParser parser;
    parser.setApplicationDescription("RoboCup Small Size League Simulator");
    parser.addHelpOption();
    QCommandLineOption headlessOption(QStringList() << "H" << "headless", 
                                      QCoreApplication::translate("main", "Run without a UI"));
    parser.addOption(headlessOption);
    parser.process(a);

    MainWindow w;
    if (parser.isSet(headlessOption)) {
        // enable headless mode
        w.hide();
        w.setIsGlEnabled(false);
    } else {
        // Run normally
        w.show();
    }
    return a.exec();
}
