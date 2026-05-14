// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "2d_torus_topology.h"
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>
#include "2d_torus_switch.h"
#include "compositequeue.h"
#include "config.h"
#include "ecnqueue.h"
#include "eventlist.h"
#include "inter_satellite_link.h"
#include "logfile.h"
#include "loggers.h"
#include "loggertypes.h"
#include "main.h"
#include "prioqueue.h"
#include "queue.h"
#include "queue_lossless.h"
#include "queue_lossless_input.h"
#include "queue_lossless_output.h"
#include "randomqueue.h"
#include "switch.h"

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

//! Minimal constructor used to set default parameters
TwoDimensionalTorusTopology::TwoDimensionalTorusTopology(Logfile* logfile,
                                                         QueueLoggerFactory* q_log_factory,
                                                         uint32_t n,
                                                         uint32_t m,
                                                         bool is_constellation)
    : _eventlist(&EventList::getTheEventList()),
      _logfile(logfile),
      _q_logger_factory(q_log_factory),
      _n(n),
      _m(m),
      _n_virtual_channels(0),
      _queue_type(UNDEFINED),
      _queue_size_b(0),
      _composite_q_cfg(),
      _linkspeed_bps(0),
      _routing_strategy(MINIMAL_ADAPTIVE_ALG),
      _is_constellation(is_constellation),
      _latency(0),
      _altitude_in_m(0),
      _inclination_in_deg(0) {};

//! Full constructor
TwoDimensionalTorusTopology::TwoDimensionalTorusTopology(Logfile* logfile,
                                                         QueueLoggerFactory* q_log_factory,
                                                         uint32_t n,
                                                         uint32_t m,
                                                         uint32_t n_virtual_channels,
                                                         queue_type queue_type,
                                                         mem_b queue_size,
                                                         linkspeed_bps linkspeed,
                                                         bool disable_trim,
                                                         uint16_t trim_size,
                                                         mem_b ecn_high,
                                                         mem_b ecn_low,
                                                         torus_routing_strategy routing_strategy,
                                                         bool is_constellation,
                                                         simtime_picosec latency,
                                                         uint32_t altitude,
                                                         float_t inclination)
    : TwoDimensionalTorusTopology(logfile, q_log_factory, n, m, is_constellation) {
    // Set parameters
    set_routing_strategy(routing_strategy);
    set_n_virtual_channels(n_virtual_channels);
    set_queue_params(queue_type, queue_size, disable_trim, trim_size, ecn_high, ecn_low);
    set_linkspeed(linkspeed);
    if (is_constellation) {
        set_constellation_params(altitude, inclination);
    } else {
        set_switch_latency(latency);
    }

    // Allocating switches, queues, and pipes' vectors
    // NOTE: resize() initialises vectors with values' default type, in this case nullptr
    // If things don't go right, explicitly set values to nullptr with loops here
    _switches.resize(_n, vector<Switch*>(_m));

    _north_egress_queues.resize(_n,
                                vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));
    _east_egress_queues.resize(_n, vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));
    _south_egress_queues.resize(_n,
                                vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));
    _west_egress_queues.resize(_n, vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));

    _north_ingress_queues.resize(_n,
                                 vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));
    _east_ingress_queues.resize(_n,
                                vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));
    _south_ingress_queues.resize(_n,
                                 vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));
    _west_ingress_queues.resize(_n,
                                vector<vector<Queue*>>(_m, vector<Queue*>(_n_virtual_channels)));

    _northbound_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_n_virtual_channels)));
    _eastbound_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_n_virtual_channels)));
    _southbound_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_n_virtual_channels)));
    _westbound_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_n_virtual_channels)));

    // Create all switches first to avoid referencing un-instantiated switches in next step
    uint32_t id = 0;
    for (uint32_t i = 0; i < _n; i++) {
        for (uint32_t j = 0; j < _m; j++) {
            _switches[i][j] = new TwoDimensionalTorusSwitch(_eventlist, id++, _routing_strategy);
            _switches[i][j]->setName("Switch_(" + ntoa(i) + ", " + ntoa(j) + ")");
        }
    }

    // Create queues and pipes
    QueueLogger* n_queue_logger;
    QueueLogger* e_queue_logger;
    QueueLogger* s_queue_logger;
    QueueLogger* w_queue_logger;
    for (uint32_t i = 0; i < _n; i++) {
        for (uint32_t j = 0; j < _m; j++) {
            for (uint32_t vc = 0; vc < _n_virtual_channels; vc++) {
                // --- Create Pipes ---
                // TODO: Check correctness in Pipe creation
                if (is_constellation) {
                    _northbound_pipes[i][j][vc] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _eastbound_pipes[i][j][vc] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _southbound_pipes[i][j][vc] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _westbound_pipes[i][j][vc] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                } else {
                    _northbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                    _eastbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                    _southbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                    _westbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                }
                // Set names
                _northbound_pipes[i][j][vc]->setName("NorthboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                     ")");
                _eastbound_pipes[i][j][vc]->setName("EastboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                    ")");
                _southbound_pipes[i][j][vc]->setName("SouthboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                     ")");
                _westbound_pipes[i][j][vc]->setName("WestboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                    ")");
                // --------------------

                // --- Create Queues ---
                // Start with egress queues

                // Init queue loggers
                n_queue_logger =
                    (_q_logger_factory) ? _q_logger_factory->createQueueLogger() : nullptr;
                e_queue_logger =
                    (_q_logger_factory) ? _q_logger_factory->createQueueLogger() : nullptr;
                s_queue_logger =
                    (_q_logger_factory) ? _q_logger_factory->createQueueLogger() : nullptr;
                w_queue_logger =
                    (_q_logger_factory) ? _q_logger_factory->createQueueLogger() : nullptr;

                // We allocate a queue of given type to _egress_queues with alloc_queue,
                // and then populate _ingress_queues with LosslessInputQueue only
                // if such queue types are used. Otherwise, ingress queues are left as nullptr.
                // TODO: when allocating buffer size, divide by _n_virtual_channels
                _north_egress_queues[i][j][vc] =
                    alloc_queue(n_queue_logger, _linkspeed_bps / _n_virtual_channels,
                                _queue_size_b / _n_virtual_channels);
                _east_egress_queues[i][j][vc] =
                    alloc_queue(e_queue_logger, _linkspeed_bps / _n_virtual_channels,
                                _queue_size_b / _n_virtual_channels);
                _south_egress_queues[i][j][vc] =
                    alloc_queue(s_queue_logger, _linkspeed_bps / _n_virtual_channels,
                                _queue_size_b / _n_virtual_channels);
                _west_egress_queues[i][j][vc] =
                    alloc_queue(w_queue_logger, _linkspeed_bps / _n_virtual_channels,
                                _queue_size_b / _n_virtual_channels);

                _north_egress_queues[i][j][vc]->setName("NorthEgressQueue_(" + ntoa(i) + ", " +
                                                        ntoa(j) + ")");
                _east_egress_queues[i][j][vc]->setName("EastEgressQueue_(" + ntoa(i) + ", " +
                                                       ntoa(j) + ")");
                _south_egress_queues[i][j][vc]->setName("SouthEgressQueue_(" + ntoa(i) + ", " +
                                                        ntoa(j) + ")");
                _west_egress_queues[i][j][vc]->setName("WestEgressQueue_(" + ntoa(i) + ", " +
                                                       ntoa(j) + ")");
                // Connect switch to these queues
                _switches[i][j]->addPort(_north_egress_queues[i][j][vc]);
                _switches[i][j]->addPort(_east_egress_queues[i][j][vc]);
                _switches[i][j]->addPort(_south_egress_queues[i][j][vc]);
                _switches[i][j]->addPort(_west_egress_queues[i][j][vc]);

                if (_queue_type == LOSSLESS_INPUT || _queue_type == LOSSLESS_INPUT_ECN) {
                    // Create an ingress queue for the switch opposite to appropriate egress switch
                    // E.g.: to NorthEgressQueue associate opposite Switch's SouthIngressQueue
                    // So determine coords of opposite switch
                    uint32_t n_i, n_j, e_i, e_j, s_i, s_j, w_i, w_j;
                    // Simulate wrap-around when network edge is reached
                    n_i = (i == 0) ? _n - 1 : i - 1;
                    n_j = j;
                    _north_ingress_queues[n_i][n_j][vc] = new LosslessInputQueue(
                        *_eventlist, _south_egress_queues[i][j][vc], _switches[n_i][n_j], _latency);
                    _north_ingress_queues[n_i][n_j][vc]->setName("NorthIngressQueue_(" + ntoa(n_i) +
                                                                 ", " + ntoa(n_j) + ")");
                    e_i = i;
                    e_j = (j == _m - 1) ? 0 : j + 1;
                    _east_ingress_queues[e_i][e_j][vc] = new LosslessInputQueue(
                        *_eventlist, _west_egress_queues[i][j][vc], _switches[e_i][e_j], _latency);
                    _east_ingress_queues[e_i][e_j][vc]->setName("EastIngressQueue_(" + ntoa(e_i) +
                                                                ", " + ntoa(e_j) + ")");
                    s_i = (i == _n - 1) ? 0 : i + 1;
                    s_j = j;
                    _south_ingress_queues[s_i][s_j][vc] = new LosslessInputQueue(
                        *_eventlist, _north_egress_queues[i][j][vc], _switches[s_i][s_j], _latency);
                    _south_ingress_queues[s_i][s_j][vc]->setName("SouthIngressQueue_(" + ntoa(s_i) +
                                                                 ", " + ntoa(s_j) + ")");
                    w_i = i;
                    w_j = (j == 0) ? _m - 1 : j - 1;
                    _west_ingress_queues[w_i][w_j][vc] = new LosslessInputQueue(
                        *_eventlist, _east_egress_queues[i][j][vc], _switches[w_i][w_j], _latency);
                    _west_ingress_queues[w_i][w_j][vc]->setName("WestIngressQueue_(" + ntoa(w_i) +
                                                                ", " + ntoa(w_j) + ")");
                } else {
                    // Do nothing.
                    // Ingress queues are never used, except for the lossless input case.
                    // Egress queues do the heavy lifting during simulations.
                    // The only job of ingress queues is to store lossless input queues; something
                    // that wasn't done in the fat tree implementation, likely because the lossless
                    // input queue type was added after a first version of the fat tree was
                    // completed, and keeping track of the needed ingress queues would require
                    // doubling the number of queue vectors, which is already significant in fat
                    // tree. That would explain the blatant memory leak.
                    // In our case, keeping track of ingress queues is easy so we do it.
                }

                // --------------------
            }

            // TODO: Deal with logging, is it automatic for Switches and Pipes or is a manual setup
            // needed?
            // NOTE: Pipes are "connected" upon route creation, see get_bidir_paths of fat tree
        }
    }
}

// TODO: Create constructor that reads from config file

template <class P>
void delete_3d_vector(vector<vector<vector<P*>>>& vec3d) {
    for (auto& vec1 : vec3d) {
        for (auto& vec2 : vec1) {
            for (auto* pipe : vec2) {
                delete pipe;
            }
        }
    }
    vec3d.clear();
}

TwoDimensionalTorusTopology::~TwoDimensionalTorusTopology() {
    for (auto& switch_vec : _switches) {
        for (auto* sw : switch_vec) {
            delete sw;
        }
    }
    _switches.clear();

    delete_3d_vector(_north_egress_queues);
    delete_3d_vector(_east_egress_queues);
    delete_3d_vector(_south_egress_queues);
    delete_3d_vector(_west_egress_queues);

    delete_3d_vector(_north_ingress_queues);
    delete_3d_vector(_east_ingress_queues);
    delete_3d_vector(_south_ingress_queues);
    delete_3d_vector(_west_ingress_queues);

    delete_3d_vector(_northbound_pipes);
    delete_3d_vector(_westbound_pipes);
    delete_3d_vector(_southbound_pipes);
    delete_3d_vector(_eastbound_pipes);
}

void TwoDimensionalTorusTopology::set_routing_strategy(torus_routing_strategy rs) {
    _routing_strategy = rs;
}

void TwoDimensionalTorusTopology::set_n_virtual_channels(uint32_t n) {
    if (_routing_strategy == DIMENSION_ORDERED) {
        // In this case do not use virtual channels
        _n_virtual_channels = 1;
    } else {  // MINIMAL_ADAPTIVE_ALG needs at least 2 VCs
        _n_virtual_channels = (n < 2) ? 2 : n;
    }
}

void TwoDimensionalTorusTopology::set_queue_params(queue_type queue_type,
                                                   mem_b queue_size,
                                                   bool disable_trim,
                                                   uint16_t trim_size,
                                                   mem_b ecn_high,
                                                   mem_b ecn_low) {
    // Default to lossless
    _queue_type = (queue_type == UNDEFINED) ? LOSSLESS : queue_type;
    // Default to accomodating 1 packet per virtual channel
    _queue_size_b = (queue_size == 0) ? memFromPkt(_n_virtual_channels) : queue_size;

    if (_queue_type == COMPOSITE || _queue_type == COMPOSITE_ECN) {
        ecn_low = (ecn_low == 0) ? ecn_high : ecn_low;
        _composite_q_cfg = composite_q_cfg{disable_trim, trim_size, ecn_high, ecn_low};
    }
}

void TwoDimensionalTorusTopology::set_linkspeed(linkspeed_bps ls) {
    //! Default to 100Gbps
    _linkspeed_bps = (ls == 0) ? speedFromGbps(100) : ls;
}

void TwoDimensionalTorusTopology::set_constellation_params(uint32_t altitude, float_t inclination) {
    if (altitude <= 160000 || altitude > 2000000) {
        cout << "Set altitude is invalid, using default of 800km" << endl;
        _altitude_in_m = 800000;
    } else {
        _altitude_in_m = altitude;
    }

    if (inclination <= 0 || inclination >= 360) {
        cout << "Set orbital inclination is invalid, using default of 45 degrees" << endl;
        _inclination_in_deg = 45;
    } else {
        _inclination_in_deg = inclination;
    }
}

void TwoDimensionalTorusTopology::set_switch_latency(simtime_picosec latency) {
    _latency = latency;
}

Queue* TwoDimensionalTorusTopology::alloc_queue(QueueLogger* queue_logger,
                                                linkspeed_bps linkspeed_bps,
                                                mem_b queue_size) {
    switch (_queue_type) {
        case RANDOM:
            // Simple queue that randomly decides whether to drop packet when above a given
            // threshold (set through RANDOM_BUFFER), and drops incoming packet when full
            return new RandomQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger,
                                   memFromPkt(RANDOM_BUFFER));
        case ECN:
            // Explicit Congestion Notification, sends pause frames to signal when buffer is full,
            // but drops incoming packet
            return new ECNQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger,
                                memFromPkt(15));
        case LOSSLESS:
            // Sets a threshold below max size, and when reached sends a pause frame back to sender
            // until ready to receive again. If it receives a pause frame, it waits until sender is
            // ready to receive again
            return new LosslessQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger, nullptr);
        case LOSSLESS_INPUT:
            // We need to pair this queue to a LosslessInputQueue we'll create, so actually return
            // LosslessOutputQueue. Only input queues can send pause frame, output queues must wait
            return new LosslessOutputQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger);
        case LOSSLESS_INPUT_ECN:
            // Emulate (actually lossless) ECN by having virtually inexhaustible buffer size
            return new LosslessOutputQueue(linkspeed_bps, memFromPkt(10000), *_eventlist,
                                           queue_logger);
        case CTRL_PRIO:
            // Queue that sets packets as high or low priority, but I'm unsure of its workings, what
            // it's supposed to do uniquely
            return new CtrlPrioQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger);
        case COMPOSITE:
            // Complex queue type: contains high and low priority buffers, with weighted round
            // robin that heavily prioritises high priority packets. Can use ECN on low priority
            // queue. When that queue is full, can trim packets down to header and put them in high
            // priority queue, or drop them entirely if not trimming.

            //! _trim_size is size packet is trimmed down to, i.e. header's size
            //! _ecn_threshold is size starting which ECN is triggered
            //! _ecn_low and _ecn_high are respectively soft and hard thresholds: if queue_size
            //! exceeds _ecn_high, ECN is triggered. Else, if only exceedes low, then ECN is
            //! triggered only with a given probability proportional to fullness of queue. Setting
            //! single threshold means _ecn_high == _ecn_low
            return new CompositeQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger,
                                      _composite_q_cfg.trim_size, _composite_q_cfg.disable_trim);
        case COMPOSITE_ECN: {
            CompositeQueue* q =
                new CompositeQueue(linkspeed_bps, queue_size, *_eventlist, queue_logger,
                                   _composite_q_cfg.trim_size, _composite_q_cfg.disable_trim);
            q->set_ecn_thresholds(_composite_q_cfg.ecn_min_threshold,
                                  _composite_q_cfg.ecn_max_threshold);
            return q;
        }
        default:
            throw runtime_error(
                "Error allocating queue: queue type is invalid or hasn't been implemented");
    }
}
