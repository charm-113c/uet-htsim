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
        uint32_t VC = 0;

        // First, construct route out of host and into switch if still at src
        if (pkt_coords[1] == src_coords[1] && pkt_coords[0] == src_coords[0]) {
            next_hop->push_back(
                _torus->host_to_switch_queues[src_coords[0]][src_coords[1]][src_coords[2]]);
            next_hop->push_back(
                _torus->host_to_switch_pipes[src_coords[0]][src_coords[1]][src_coords[2]]);
            next_hop->push_back(_torus->switches[src_coords[0]][src_coords[1]]);
        }

        // Check if X dimension needs to be corrected
        if (pkt_coords[1] != dest_coords[1]) {
            // Determine next minimal direction

            // Eastward case: (x < dest_x && wrapping around to the west is slower) ||
            // (x > dest_x && wrapping around to the east is faster)
            if (((pkt_coords[1] < dest_coords[1]) && ((dest_coords[1] - pkt_coords[1]) * 2 < m)) ||
                ((pkt_coords[1] > dest_coords[1]) && ((pkt_coords[1] - dest_coords[1]) * 2 > m))) {
                // Determine which VC is being used by determining whether a wrap-around has
                // happened.
                // Wrapped around if (src_x > dest_x && x < dest_x)
                if (src_coords[1] > dest_coords[1] && pkt_coords[1] < dest_coords[1]) {
                    VC = 1;
                }
                Queue* east_q = _torus->east_egress_queues[pkt_coords[0]][pkt_coords[1]][VC];

                // Is east route congested?
                double fullness_ratio = ((double)east_q->queuesize()) / ((double)east_q->maxsize());
                // The fuller the queue is, the higher the probability to use VC2
                if ((fullness_ratio > 0.5) && (drand() < fullness_ratio)) {
                    // if (east_q->queuesize() == east_q->maxsize()) {
                    // 2.a. Use VC2
                    // Check if eastward VC2 is congested. If it is, attempt to go north or south
                    Queue* east_q_vc2 =
                        _torus->east_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2];
                    // if (east_q_vc2->queuesize() < east_q_vc2->maxsize()) {
                    next_hop->push_back(east_q_vc2);
                    next_hop->push_back(_torus->eastbound_pipes[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(east_q_vc2->getRemoteEndpoint());
                    // } else {
                    // TODO:Find if going north or south is a minimal path

                } else {
                    next_hop->push_back(east_q);
                    next_hop->push_back(_torus->eastbound_pipes[pkt_coords[0]][pkt_coords[1]][VC]);
                    next_hop->push_back(east_q->getRemoteEndpoint());
                }
            } else {
                // Westward case:
                // Check if pkt wrapped around
                if ((src_coords[1] < dest_coords[1]) && (pkt_coords[1] > dest_coords[1])) {
                    VC = 1;
                }

                // Determine whether west is congested
                Queue* west_q = _torus->west_egress_queues[pkt_coords[0]][pkt_coords[1]][VC];
                double fullness_ratio = ((double)west_q->queuesize()) / ((double)west_q->maxsize());
                if ((fullness_ratio > 0.5) && (drand() < fullness_ratio)) {
                    next_hop->push_back(
                        _torus->west_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(_torus->westbound_pipes[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(
                        _torus->west_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2]
                            ->getRemoteEndpoint());
                } else {
                    next_hop->push_back(west_q);
                    next_hop->push_back(_torus->westbound_pipes[pkt_coords[0]][pkt_coords[1]][VC]);
                    next_hop->push_back(west_q->getRemoteEndpoint());
                }
            }
            return next_hop;
        } else if (pkt_coords[0] != dest_coords[0]) {
            // Check if Y dimension needs correcting
            // Determine next minimal direction

            // Southward case: (y < y' && wrap-around to the north is slower) || (y > y' &&
            // wrap-around to the south is faster)
            if (((pkt_coords[0] < dest_coords[0]) && ((dest_coords[0] - pkt_coords[0]) * 2 < n)) ||
                ((pkt_coords[0] > dest_coords[0]) && ((pkt_coords[0] - dest_coords[0]) * 2 > n))) {
                // Check if pkt wrapped around
                if ((src_coords[0] > dest_coords[0]) && (pkt_coords[0] < dest_coords[0])) {
                    VC = 1;
                }
                // Check queue congestion
                Queue* south_q = _torus->south_egress_queues[pkt_coords[0]][pkt_coords[1]][VC];

                double fullness_ratio =
                    ((double)south_q->queuesize()) / ((double)south_q->maxsize());
                if ((fullness_ratio > 0.5) && (drand() < fullness_ratio)) {
                    next_hop->push_back(
                        _torus->south_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(
                        _torus->southbound_pipes[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(
                        _torus->south_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2]
                            ->getRemoteEndpoint());
                } else {
                    next_hop->push_back(south_q);
                    next_hop->push_back(_torus->southbound_pipes[pkt_coords[0]][pkt_coords[1]][VC]);
                    next_hop->push_back(south_q->getRemoteEndpoint());
                }
            } else {
                // Northward case:
                // Check for wrap-around
                if ((src_coords[0] < dest_coords[0]) && (pkt_coords[0] > dest_coords[0])) {
                    VC = 1;
                }
                // Check queue congestion
                Queue* north_q = _torus->north_egress_queues[pkt_coords[0]][pkt_coords[1]][VC];
                double fullness_ratio =
                    ((double)north_q->queuesize()) / ((double)north_q->maxsize());
                if ((fullness_ratio > 0.5) && (drand() < fullness_ratio)) {
                    next_hop->push_back(
                        _torus->north_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(
                        _torus->northbound_pipes[pkt_coords[0]][pkt_coords[1]][VC2]);
                    next_hop->push_back(
                        _torus->north_egress_queues[pkt_coords[0]][pkt_coords[1]][VC2]
                            ->getRemoteEndpoint());
                } else {
                    next_hop->push_back(north_q);
                    next_hop->push_back(_torus->northbound_pipes[pkt_coords[0]][pkt_coords[1]][VC]);
                    next_hop->push_back(north_q->getRemoteEndpoint());
                }
            }
            return next_hop;
        } else {
            // If at destination, add switch to host routing
            next_hop->push_back(
                _torus->switch_to_host_queues[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
            next_hop->push_back(
                _torus->switch_to_host_pipes[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
        }
    }
    // TODO: Check if we need to add the host-switch routing here, as that doesn't seem to be
    // the case in fat tree switch

    return next_hop;
}
