// http://harmattan-dev.nokia.com/docs/library/html/qt4/unix-signals.html
// http://qt-project.org/forums/viewthread/1227
// https://github.com/qknight/qt-fuse-example/blob/master/qt-fuse/

#include "rqt_utils/common/QSignalHandler.h"

#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <unistd.h>

namespace rqt_utils {

//needed to not get an undefined reference to static members
int QSignalHandler::sighupFd[2];
int QSignalHandler::sigtermFd[2];
int QSignalHandler::sigintFd[2];

QSignalHandler::QSignalHandler(QObject *parent)
    : QObject(parent)
{
    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sighupFd))
        qFatal("Couldn't create HUP socketpair");

    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigtermFd))
        qFatal("Couldn't create TERM socketpair");

    if (::socketpair(AF_UNIX, SOCK_STREAM, 0, sigintFd))
        qFatal("Couldn't create INT socketpair");

    snInt = new QSocketNotifier(sigintFd[1], QSocketNotifier::Read, this);
    connect(snInt, SIGNAL(activated(int)), this, SLOT(handleSigInt()));

    snHup = new QSocketNotifier(sighupFd[1], QSocketNotifier::Read, this);
    connect(snHup, SIGNAL(activated(int)), this, SLOT(handleSigHup()));

    snTerm = new QSocketNotifier(sigtermFd[1], QSocketNotifier::Read, this);
    connect(snTerm, SIGNAL(activated(int)), this, SLOT(handleSigTerm()));
}


QSignalHandler::~QSignalHandler() {
    delete snHup;
    delete snTerm;
    delete snInt;
}


void QSignalHandler::intSignalHandler(int)
{
    char a = 1;
    if (::write(sigintFd[0], &a, sizeof(a)) != 0) {}
}


void QSignalHandler::hupSignalHandler(int)
{
    char a = 1;
    if (::write(sighupFd[0], &a, sizeof(a))) {}
}


void QSignalHandler::termSignalHandler(int)
{
    char a = 1;
    if (::write(sigtermFd[0], &a, sizeof(a))) {}
}


void QSignalHandler::handleSigInt()
{
    snInt->setEnabled(false);
    char tmp;
    if (::read(sigintFd[1], &tmp, sizeof(tmp))) {}

    emit sigINT();
    snInt->setEnabled(true);
}


void QSignalHandler::handleSigTerm()
{
    snTerm->setEnabled(false);
    char tmp;
    if (::read(sigtermFd[1], &tmp, sizeof(tmp))) {}

    emit sigTERM();
    snTerm->setEnabled(true);
}


void QSignalHandler::handleSigHup()
{
    snHup->setEnabled(false);
    char tmp;
    if (::read(sighupFd[1], &tmp, sizeof(tmp))) {}

    emit sigHUP();

    snHup->setEnabled(true);
}

int QSignalHandler::setup_unix_signal_handlers() {
    struct sigaction sigHup, sigTerm, sigInt;

    sigHup.sa_handler = hupSignalHandler;
    sigemptyset(&sigHup.sa_mask);
    sigHup.sa_flags = 0;
    sigHup.sa_flags |= SA_RESTART;

    if (sigaction(SIGHUP, &sigHup, 0) > 0)
        return 1;

    sigTerm.sa_handler = termSignalHandler;
    sigemptyset(&sigTerm.sa_mask);
    sigTerm.sa_flags |= SA_RESTART;

    if (sigaction(SIGTERM, &sigTerm, 0) > 0)
        return 2;

    sigInt.sa_handler = intSignalHandler;
    sigemptyset(&sigInt.sa_mask);
    sigInt.sa_flags |= SA_RESTART;

    if (sigaction(SIGINT, &sigInt, 0) > 0)
        return 3;

    return 0;
}

}
