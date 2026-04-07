#ifndef PACKET_SENDER_THREAD_H
#define PACKET_SENDER_THREAD_H

#include <QThread>
#include <QQueue>
#include <QMutex>
#include <QWaitCondition>
#include <QHostAddress>
#include <QByteArray>

// UDP送信を非同期で処理するワーカースレッド。
// プロデューサー（メインスレッド）はenqueue()でパケットを積み、
// ワーカースレッドがまとめてwriteDatagramする。
class PacketSenderThread : public QThread {
    Q_OBJECT
public:
    struct Packet {
        QByteArray data;
        QHostAddress addr;
        quint16 port;
    };

    explicit PacketSenderThread(QObject* parent = nullptr);
    ~PacketSenderThread() override;

    void enqueue(QByteArray data, const QHostAddress& addr, quint16 port);
    void stop();

protected:
    void run() override;

private:
    QQueue<Packet> queue_;
    QMutex mutex_;
    QWaitCondition cond_;
    bool running_ = true;
};

#endif // PACKET_SENDER_THREAD_H
