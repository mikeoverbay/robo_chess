// umax_cli.cpp - micro-Max 4.8 (umax/engine, Robo Chess's own copy) as a console program, so the arm's self-play
// (tools/selfplay.py) gets exactly the moves its Mega 2560 would play.
//
// The Mega's clock is replaced by a node clock: micro-Max calls its hook every 64 nodes, and each call moves the
// clock on by 64 nodes at the Mega's speed (about 700 nodes a second, measured with umaxbench). So the time budget
// of a SKILL level buys the same depth here as on the Mega, in milliseconds instead of seconds. The engine's hash
// table and repetition locks carry from move to move as on the device: keep one process for a whole game.
//
// stdin, one command a line:
//   new                                        start position, empty hash          -> ok
//   think BUDGET_MS MAXDEPTH NPS w|b EP C0..C63  the position as the Mega loads it   -> move FROM TO DEPTH SCORE NODES MS
//                                                (C = micro-Max piece codes, a8..h8, a7..h1; EP 0x88 or 128)   or none
//   quit
#include "umax.h"
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>

static double clock_us = 0.0, us_per_poll = 0.0;

uint32_t millis() { return (uint32_t) (clock_us / 1000.0); }

static bool tick() { clock_us += us_per_poll; return false; }

int main() {
    umax_hook = tick;
    std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream in(line);
        std::string cmd;
        in >> cmd;
        if (cmd == "new") {
            umax_new_game();
            std::printf("ok\n");
        } else if (cmd == "think") {
            unsigned budget = 0, maxDepth = 0, ep = UMAX_NO_EP;
            double nps = 700.0;
            std::string side;
            in >> budget >> maxDepth >> nps >> side >> ep;
            umax_clear_board();
            for (int i = 0; i < 64; i++) {
                unsigned c = 0;
                in >> c;
                if (c) umax_put(UMAX_SQ(i / 8, i % 8), (uint8_t) c);
            }
            if (!in) { std::printf("error bad position\n"); std::fflush(stdout); continue; }
            umax_set_position(side == "w", (uint8_t) ep);
            clock_us = 0.0;
            us_per_poll = 64.0 * 1e6 / nps;
            UmaxResult r;
            if (umax_think((uint16_t) budget, (uint8_t) maxDepth, r))
                std::printf("move %u %u %u %d %lu %u\n", r.from, r.to, r.depth, r.score, (unsigned long) r.nodes, r.ms);
            else
                std::printf("none\n");
        } else if (cmd == "quit") {
            break;
        } else if (!cmd.empty()) {
            std::printf("error unknown %s\n", cmd.c_str());
        }
        std::fflush(stdout);
    }
    return 0;
}
