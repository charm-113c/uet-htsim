// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "2d_torus_switch.h"
#include <cstddef>
#include <vector>
#include "2d_torus_topology.h"
#include "eth_pause_packet.h"
#include "network.h"
#include "queue_lossless.h"
#include "route.h"
#include "switch.h"

TwoDimensionalTorusSwitch::TwoDimensionalTorusSwitch(EventList* eventlist,
                                                     TwoDimensionalTorusTopology* torus,
                                                     uint32_t id,
                                                     torus_routing_strategy routing_strategy)
    : Switch(*eventlist) {
    _torus = torus;
    _id = id;
    _routing_strategy = routing_strategy;
}

void TwoDimensionalTorusSwitch::receivePacket(Packet& pkt) {
    if (pkt.type() == ETH_PAUSE) {
        // In lossless mode
        EthPausePacket* eth_pkt = (EthPausePacket*)&pkt;
        // Identify egress queue to which pkt is adressed
        for (size_t i = 0; i < _ports.size(); i++) {
            LosslessQueue* egress_q = (LosslessQueue*)_ports.at(i);
            // NOTE: queues themselves don't have an ID, so packet's sender ID is always set to
            // sending switch's ID
            if (egress_q->getRemoteEndpoint() &&
                ((Switch*)egress_q->getRemoteEndpoint())->getID() == eth_pkt->senderID()) {
                egress_q->receivePacket(pkt);
                break;
            }
        }
        return;
    }

    if (_routing_strategy == DIMENSION_ORDER) {
        // Fully deterministic routing algorithm, so since pkt's route is fully written in pkt by
        // transport protocol using get_bidir_paths, switch only serves to forward it: no
        // modifications necessary.
    } else {
        Route* next_hop = getNextHop(pkt);
        pkt.set_route(next_hop);
    }
    pkt.sendOn();
}

Route* TwoDimensionalTorusSwitch::getNextHop(Packet& pkt) {
    Route* next_hop = new Route();
    if (_routing_strategy == MINIMAL_ADAPTIVE_ALG) {
        // Minimal adaptive algorithm, or *-channels.
        //
        // It is based on DOR, but gives one additional virtual channel (VC2) to all links. VC0 and
        // VC1 are called star channels, and this algorithm works exactly the same as DOR using the
        // star channels. The only difference is as follows: at any point, and especially when
        // meeting congestion, a packet may choose to use VC2 to go in any direction, provided that
        // that direction still ensures a minimal path.
        // The packet is allowed to enter a star channel on dimension Y only after it has finished
        // correcting its X dimension. When entering a star channel, it must follow DOR rules and
        // use VC0 if it hasn't wrapped around yet, otherwise VC1.
        //
        // NOTE: in order to determine which VC to use (so whether a packet has wrapped around a
        // given dimension or not), we establish a convention where if there are two minimal paths
        // on the X-dimension, we default to the westward one, and similarly default to the
        // northward one on Y.

        // Host coords format: [switch_y, switch_x, internal_host_number]
        vector<uint32_t> src_coords = _torus->get_coords_of_host[pkt.src()];
        vector<uint32_t> dst_coords = _torus->get_coords_of_host[pkt.dst()];
        // Switch coords: [switch_y, switch_x]
        vector<uint32_t> pkt_coords = _torus->get_coords_of_switch[_id];

        // We need to determine the next hop:
        // 1. Default to DOR:
        //    a. Determine if we need to and can travel across X-dimension, and if so, if we've
        //    already wrapped around:
        //        i. If we need to and can, set next_hop accordingly.
        //        ii. If we need to but can't, jump to point 2.
        //    b. If here, then we don't need to travel acros X; see if we've wrapped around Y
        //    already:
        //        i. If we can travel across Y, do so.
        //        ii. Else, go to point 2.
        // 2. Use VC2:
        //    a.
    }

    return next_hop;
}
