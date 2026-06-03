// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "2d_torus_switch.h"
#include <cstddef>
#include <cstdint>
#include <vector>
#include "2d_torus_topology.h"
#include "config.h"
#include "eth_pause_packet.h"
#include "network.h"
#include "queue.h"
#include "queue_lossless.h"
#include "route.h"
#include "switch.h"

#define VC2 2

torus_routing_strategy TwoDimensionalTorusSwitch::_routing_strategy = DIMENSION_ORDER;

TwoDimensionalTorusSwitch::TwoDimensionalTorusSwitch(EventList* eventlist,
                                                     TwoDimensionalTorusTopology* torus,
                                                     uint32_t id,
                                                     string name,
                                                     torus_routing_strategy routing_strategy)
    : Switch(*eventlist, name) {
    _torus = torus;
    _id = id;
    TwoDimensionalTorusSwitch::_routing_strategy = routing_strategy;
}

void TwoDimensionalTorusSwitch::receivePacket(Packet& pkt) {
    // cout << "-----------------------------------------------------" << endl;
    // vector<uint32_t> coords = _torus->get_coords_of_switch[_id];
    // cout << "x=" << coords[1] << ", y=" << coords[0] << endl;
    // cout << "Packet src_host=" << pkt.src() << ", dest_host=" << pkt.dst() << endl;
    // vector<uint32_t> pkt_coords = _torus->get_coords_of_host[pkt.src()];
    // vector<uint32_t> dest_coords = _torus->get_coords_of_host[pkt.dst()];
    // cout << "src_x=" << pkt_coords[1] << ", src_y=" << pkt_coords[0] << endl;
    // cout << "dst_x=" << dest_coords[1] << ", dst_y=" << dest_coords[0] << endl;
    //
    if (pkt.type() == ETH_PAUSE) {
        // In lossless mode
        EthPausePacket* eth_pkt = (EthPausePacket*)&pkt;
        // Identify egress queue to which pkt is adressed
        for (size_t i = 0; i < _ports.size(); i++) {
            LosslessQueue* egress_q = (LosslessQueue*)_ports.at(i);
            // NOTE: queues themselves don't have an ID but use their switch's, and packet's sender
            // ID is always set to sending switch's ID
            if (egress_q->getRemoteEndpoint() &&
                ((Switch*)egress_q->getRemoteEndpoint())->getID() == eth_pkt->senderID()) {
                egress_q->receivePacket(pkt);
                break;
            }
        }
        return;
    }

    if (TwoDimensionalTorusSwitch::_routing_strategy == DIMENSION_ORDER) {
        // Fully deterministic routing algorithm, so since pkt's route is fully written in pkt by
        // transport protocol using get_bidir_paths, switch only serves to forward it: no route
        // modifications necessary.
    } else {
        Route* next_hop = getNextHop(pkt);
        // WARNING: Possible memory leak as old route is dereferenced, set_route should delete old
        // route
        pkt.set_route(next_hop);
    }
    pkt.sendOn();
}

Route* TwoDimensionalTorusSwitch::getNextHop(Packet& pkt) {
    Route* next_hop = new Route();
    if (TwoDimensionalTorusSwitch::_routing_strategy == MINIMAL_ADAPTIVE_ALG) {
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

        // NOTE: in order to determine which VC to use (so whether a packet has wrapped around a
        // given dimension or not), we establish a convention where if there are two minimal paths
        // on the X-dimension, we default to the westward one, and similarly default to the
        // northward one on Y.

        // Host coords format: [switch_y, switch_x, internal_host_number]
        vector<uint32_t> src_coords = _torus->get_coords_of_host[pkt.src()];
        vector<uint32_t> dest_coords = _torus->get_coords_of_host[pkt.dst()];
        // Switch coords: [switch_y, switch_x]
        vector<uint32_t> pkt_coords = _torus->get_coords_of_switch[_id];

        uint32_t n = _torus->get_n();
        uint32_t m = _torus->get_m();
        uint32_t vc = 0;
        cout << "------------------------------------------------" << endl;
        cout << "Routing packet " << pkt.id() << " from src=" << pkt.src()
             << " to dest=" << pkt.dst() << endl;
        cout << "Current position: x=" << pkt_coords[1] << ", src_y=" << pkt_coords[0] << endl;

        // First, construct route out of host and into switch if still at src
        // Actually, transport should take care of that: it's responsible for sending a packet from
        // host to its first switch

        // Default to DOR, and if DOR routes are congested use VC2
        // Check if X dimension needs to be corrected
        if (pkt_coords[1] != dest_coords[1]) {
            // -----------------------
            // Determine minimal direction

            // Eastward case: (src_x < dest_x && wrapping around to the west is slower) ||
            // (src_x > dest_x && wrapping around to the east is faster)
            if (((src_coords[1] < dest_coords[1]) && ((dest_coords[1] - src_coords[1]) * 2 < m)) ||
                ((src_coords[1] > dest_coords[1]) && ((src_coords[1] - dest_coords[1]) * 2 > m))) {
                // Determine which VC is being used by determining whether a wrap-around has
                // occurred.
                // Wrapped around if (src_x > dest_x && x < dest_x)
                if (src_coords[1] > dest_coords[1] && pkt_coords[1] < dest_coords[1]) {
                    vc = 1;
                }
                Queue* east_q = _torus->east_egress_queues[pkt_coords[0]][pkt_coords[1]][vc];

                // Is east route congested?
                double fullness_ratio = ((double)east_q->queuesize()) / ((double)east_q->maxsize());
                // The fuller the queue is, the higher the probability to use VC2
                if ((fullness_ratio > 0.5) && (drand() <= fullness_ratio))
                    // Use VC2
                    vc = 2;
                // TODO: Check if eastward VC2 is congested. If so, attempt to go N or S
                // if (east_q_vc2->queuesize() < east_q_vc2->maxsize()) {
                next_hop->push_back(_torus->east_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->eastbound_pipes[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->east_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]
                                        ->getRemoteEndpoint());
            } else {
                // Westward case:
                // Check if pkt wrapped around
                if ((src_coords[1] < dest_coords[1]) && (pkt_coords[1] > dest_coords[1])) {
                    vc = 1;
                }

                // Determine whether west is congested
                Queue* west_q = _torus->west_egress_queues[pkt_coords[0]][pkt_coords[1]][vc];
                double fullness_ratio = ((double)west_q->queuesize()) / ((double)west_q->maxsize());
                if ((fullness_ratio > 0.5) && (drand() <= fullness_ratio))
                    vc = 2;
                next_hop->push_back(_torus->west_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->westbound_pipes[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->west_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]
                                        ->getRemoteEndpoint());
            }
            // -----------------------

        } else if (pkt_coords[0] != dest_coords[0]) {
            // -----------------------
            // Check if Y dimension needs correcting
            // Determine next minimal direction

            // Southward case: (y < y' && wrap-around to the north is slower) || (y > y' &&
            // wrap-around to the south is faster)
            if (((src_coords[0] < dest_coords[0]) && ((dest_coords[0] - src_coords[0]) * 2 < n)) ||
                ((src_coords[0] > dest_coords[0]) && ((src_coords[0] - dest_coords[0]) * 2 > n))) {
                // Check if pkt wrapped around
                if ((src_coords[0] > dest_coords[0]) && (pkt_coords[0] < dest_coords[0])) {
                    vc = 1;
                }
                // Check queue congestion
                Queue* south_q = _torus->south_egress_queues[pkt_coords[0]][pkt_coords[1]][vc];

                double fullness_ratio =
                    ((double)south_q->queuesize()) / ((double)south_q->maxsize());
                if ((fullness_ratio > 0.5) && (drand() <= fullness_ratio))
                    vc = 2;
                next_hop->push_back(_torus->south_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->southbound_pipes[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->south_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]
                                        ->getRemoteEndpoint());
            } else {
                // Northward case:
                // Check for wrap-around
                if ((src_coords[0] < dest_coords[0]) && (pkt_coords[0] > dest_coords[0])) {
                    vc = 1;
                }
                // Check queue congestion
                Queue* north_q = _torus->north_egress_queues[pkt_coords[0]][pkt_coords[1]][vc];
                double fullness_ratio =
                    ((double)north_q->queuesize()) / ((double)north_q->maxsize());
                if ((fullness_ratio > 0.5) && (drand() <= fullness_ratio))
                    vc = 2;
                next_hop->push_back(_torus->north_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->northbound_pipes[pkt_coords[0]][pkt_coords[1]][vc]);
                next_hop->push_back(_torus->north_egress_queues[pkt_coords[0]][pkt_coords[1]][vc]
                                        ->getRemoteEndpoint());
            }
            // -----------------------

        } else {
            // If at destination, add switch to host routing
            next_hop->push_back(
                _torus->switch_to_host_queues[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
            next_hop->push_back(
                _torus->switch_to_host_pipes[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
        }
    }

    return next_hop;
}
