// P0-1: libdatachannel (BoringSSL) <-> browser interop probe.
// Server is the offerer: GET /offer -> SDP, POST /answer, POST /report (browser stats), then exit.
#include <rtc/rtc.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <mutex>
#include <random>
#include <sstream>
#include <string>
#include <thread>

using namespace std::chrono_literals;

static std::string readFile(const char* p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

static void reply(int fd, const char* status, const char* type, const std::string& body) {
    std::string h = std::string("HTTP/1.1 ") + status + "\r\nContent-Type: " + type +
                    "\r\nContent-Length: " + std::to_string(body.size()) +
                    "\r\nConnection: close\r\n\r\n";
    (void)!write(fd, h.data(), h.size());
    (void)!write(fd, body.data(), body.size());
}

int main(int argc, char** argv) {
    int port = argc > 1 ? atoi(argv[1]) : 8081;
    rtc::InitLogger(rtc::LogLevel::Info);

    rtc::Configuration cfg;
    cfg.disableAutoNegotiation = true;
    auto pc = std::make_shared<rtc::PeerConnection>(cfg);

    std::mutex m;
    std::condition_variable cv;
    bool gathered = false;
    std::atomic<bool> connected{false}, done{false};
    std::atomic<int> dcRx{0};

    pc->onStateChange([&](rtc::PeerConnection::State s) {
        std::cout << "[pc] state " << s << std::endl;
        if (s == rtc::PeerConnection::State::Connected) connected = true;
    });
    pc->onGatheringStateChange([&](rtc::PeerConnection::GatheringState s) {
        if (s == rtc::PeerConnection::GatheringState::Complete) {
            std::lock_guard<std::mutex> l(m);
            gathered = true;
            cv.notify_all();
        }
    });

    // Video track, H.264 PT 96, with the media handler chain the real backend will use.
    rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
    media.addH264Codec(96);
    const uint32_t ssrc = 0x1234;
    media.addSSRC(ssrc, "video-send", "stream", "video");
    auto track = pc->addTrack(media);
    auto rtpCfg = std::make_shared<rtc::RtpPacketizationConfig>(
        ssrc, "video-send", 96, rtc::H264RtpPacketizer::ClockRate);
    auto packetizer = std::make_shared<rtc::H264RtpPacketizer>(
        rtc::NalUnit::Separator::LongStartSequence, rtpCfg);
    auto sr = std::make_shared<rtc::RtcpSrReporter>(rtpCfg);
    packetizer->addToChain(sr);
    packetizer->addToChain(std::make_shared<rtc::RtcpNackResponder>());
    std::atomic<int> plis{0};
    packetizer->addToChain(std::make_shared<rtc::PliHandler>([&] { plis++; }));
    track->setMediaHandler(packetizer);

    auto dc = pc->createDataChannel("input");
    dc->onOpen([&] {
        std::cout << "[dc] open" << std::endl;
        dc->send(std::string("ping-from-server"));
    });
    dc->onMessage([&](rtc::message_variant msg) {
        dcRx++;
        if (auto s = std::get_if<std::string>(&msg)) std::cout << "[dc] rx " << *s << std::endl;
    });

    pc->setLocalDescription();
    {
        std::unique_lock<std::mutex> l(m);
        cv.wait_for(l, 10s, [&] { return gathered; });
    }
    std::string offer = std::string(*pc->localDescription());
    for (auto& c : pc->localDescription()->extractCandidates()) std::cout << "[sdp] local " << std::string(c) << std::endl;

    // Synthetic H.264 access units: real start codes and NAL headers, random payload. Enough to
    // exercise packetizer + SRTP; the browser counts packets whether or not it can decode them.
    std::thread sender([&] {
        std::mt19937 rng(1);
        uint64_t frame = 0;
        while (!done) {
            if (connected && track->isOpen()) {
                rtc::binary au;
                auto nal = [&](uint8_t hdr, size_t n) {
                    const uint8_t sc[4] = {0, 0, 0, 1};
                    for (auto b : sc) au.push_back(std::byte(b));
                    au.push_back(std::byte(hdr));
                    for (size_t i = 0; i < n; i++) au.push_back(std::byte(rng() & 0xff));
                };
                bool idr = frame % 30 == 0;
                nal(idr ? 0x65 : 0x41, idr ? 6000 : 1500);  // > MTU: exercises FU-A
                rtpCfg->timestamp = rtpCfg->secondsToTimestamp(frame / 30.0);
                try {
                    track->send(au);
                } catch (const std::exception& e) {
                    std::cerr << "send: " << e.what() << std::endl;
                }
                frame++;
            }
            std::this_thread::sleep_for(33ms);
        }
        std::cout << "[video] sent " << frame << " frames, PLI " << plis << std::endl;
    });

    int ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(ls, (sockaddr*)&a, sizeof a) || listen(ls, 8)) {
        perror("bind");
        return 1;
    }
    std::cout << "[http] listening on " << port << std::endl;
    std::string page = readFile("p0_page.html");

    while (!done) {
        int fd = accept(ls, nullptr, nullptr);
        if (fd < 0) continue;
        std::string req;
        char buf[65536];
        size_t need = std::string::npos;
        while (true) {
            ssize_t n = read(fd, buf, sizeof buf);
            if (n <= 0) break;
            req.append(buf, n);
            auto he = req.find("\r\n\r\n");
            if (he != std::string::npos) {
                if (need == std::string::npos) {
                    size_t cl = 0;
                    auto p = req.find("Content-Length:");
                    if (p == std::string::npos) p = req.find("content-length:");
                    if (p != std::string::npos) cl = std::stoul(req.substr(p + 15));
                    need = he + 4 + cl;
                }
                if (req.size() >= need) break;
            }
        }
        auto body = req.substr(req.find("\r\n\r\n") + 4);
        if (req.rfind("GET / ", 0) == 0) {
            reply(fd, "200 OK", "text/html", page);
        } else if (req.rfind("GET /offer", 0) == 0) {
            reply(fd, "200 OK", "application/sdp", offer);
        } else if (req.rfind("POST /answer", 0) == 0) {
            rtc::Description answer(body, "answer");
            for (auto& c : answer.extractCandidates()) std::cout << "[sdp] remote " << std::string(c) << std::endl;
            pc->setRemoteDescription(rtc::Description(body, "answer"));
            reply(fd, "200 OK", "text/plain", "ok");
        } else if (req.rfind("POST /report", 0) == 0) {
            std::cout << "[report] " << body << std::endl;
            reply(fd, "200 OK", "text/plain", "ok");
            done = true;
        } else {
            reply(fd, "404 Not Found", "text/plain", "");
        }
        close(fd);
    }
    sender.join();
    std::cout << "[result] connected=" << connected << " dcRx=" << dcRx << std::endl;
    pc->close();
    return 0;
}
