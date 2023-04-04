// http://harmattan-dev.nokia.com/docs/library/html/qt4/unix-signals.html
// http://qt-project.org/forums/viewthread/1227
// https://github.com/qknight/qt-fuse-example/blob/master/qt-fuse/QSignalHandler.hh

#pragma once

#include <QSocketNotifier>
#include <csignal>

namespace rqt_utils {

class QSignalHandler : public QObject
{
    Q_OBJECT

public:
    QSignalHandler(QObject *parent = 0);
    ~QSignalHandler();

    // Unix signal handlers.
    static void intSignalHandler(int unused);
    static void hupSignalHandler(int unused);
    static void termSignalHandler(int unused);
    static int setup_unix_signal_handlers();
public slots:
    // Qt signal handlers.
    void handleSigInt();
    void handleSigHup();
    void handleSigTerm();
signals:
    void sigTERM();
    void sigHUP();
    void sigINT();

private:
    static int sigintFd[2];
    static int sighupFd[2];
    static int sigtermFd[2];

    QSocketNotifier *snInt;
    QSocketNotifier *snHup;
    QSocketNotifier *snTerm;

};

}
