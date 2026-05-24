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
#include "network.h"
#include "pipe.h"
#include "prioqueue.h"
#include "queue.h"
#include "queue_lossless.h"
#include "queue_lossless_input.h"
#include "queue_lossless_output.h"
#include "randomqueue.h"
#include "route.h"
#include "switch.h"

// Define virtual channels 0 and 1
// NOTE: constructor ensures those two must exist, although there may be more
#define VC0 0
#define VC1 1

#define PI 3.14159265358979323846
// Earth's radius in m, needed to compute latencies
#define R_EARTH_M 6378000
// Speed of light in mps
#define C_MPS 299792458

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
      _hosts_per_switch(0),
      _n_virtual_channels(0),
      _switch_queue_type(UNDEFINED),
      _queue_size_b(0),
      _composite_q_cfg(),
      _linkspeed_bps(0),
      _routing_strategy(DIMENSION_ORDERED),
      _is_constellation(is_constellation),
      _latency(0),
      _host_switch_latency(0),
      _altitude_in_m(0),
      _inclination_in_deg(0) {};

//! Full constructor
TwoDimensionalTorusTopology::TwoDimensionalTorusTopology(Logfile* logfile,
                                                         QueueLoggerFactory* q_log_factory,
                                                         uint32_t n,
                                                         uint32_t m,
                                                         uint32_t hosts_per_switch,
                                                         queue_type switch_queue_type,
                                                         queue_type host_queue_type,
                                                         mem_b queue_size,
                                                         linkspeed_bps linkspeed,
                                                         bool disable_trim,
                                                         uint16_t trim_size,
                                                         mem_b ecn_high,
                                                         mem_b ecn_low,
                                                         torus_routing_strategy routing_strategy,
                                                         bool is_constellation,
                                                         simtime_picosec latency,
                                                         simtime_picosec host_switch_latency,
                                                         uint32_t altitude,
                                                         float_t inclination)
    : TwoDimensionalTorusTopology(logfile, q_log_factory, n, m, is_constellation) {
    // Set parameters
    set_hosts_per_switch(hosts_per_switch);
    set_routing_strategy(routing_strategy);
    set_n_virtual_channels();
    set_queue_params(switch_queue_type, host_queue_type, queue_size, disable_trim, trim_size,
                     ecn_high, ecn_low);
    set_linkspeed(linkspeed);
    if (is_constellation) {
        set_constellation_params(altitude, inclination);
    } else {
        set_switch_latency(latency);
    }
    set_host_switch_latency(host_switch_latency);

    // Allocating switches, queues, and pipes' vectors
    // NOTE: resize() initialises vectors with values' default type, in this case nullptr
    // If things don't go right, explicitly set values to nullptr with loops here
    _switches.resize(_n, vector<Switch*>(_m));

    // Switch-host networking
    _host_to_switch_queues.resize(_n,
                                  vector<vector<Queue*>>(_m, vector<Queue*>(_hosts_per_switch)));
    _switch_to_host_queues.resize(_n,
                                  vector<vector<Queue*>>(_m, vector<Queue*>(_hosts_per_switch)));
    _host_to_switch_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_hosts_per_switch)));
    _switch_to_host_pipes.resize(_n, vector<vector<Pipe*>>(_m, vector<Pipe*>(_hosts_per_switch)));

    // Switch-switch networking
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
            // Save id mapping
            get_coords_of_switch[id] = {i, j};

            _switches[i][j] = new TwoDimensionalTorusSwitch(_eventlist, id++, _routing_strategy);
            _switches[i][j]->setName("Switch_(" + ntoa(i) + ", " + ntoa(j) + ")");
        }
    }

    // Create queues and pipes
    uint32_t host_id = 0;
    QueueLogger* hst_swi_q_logger;
    QueueLogger* swi_hst_q_logger;

    QueueLogger* n_queue_logger;
    QueueLogger* e_queue_logger;
    QueueLogger* s_queue_logger;
    QueueLogger* w_queue_logger;
    for (uint32_t i = 0; i < _n; i++) {
        for (uint32_t j = 0; j < _m; j++) {
            // Switch-host networking
            for (uint32_t host = 0; host < _hosts_per_switch; host++) {
                // Map id to position
                get_coords_of_host[host_id++] = {i, j, host};
                // Create Pipes
                // TODO: create a switch-host latency field
                _host_to_switch_pipes[i][j][host] = new Pipe(_host_switch_latency, *_eventlist);
                _switch_to_host_pipes[i][j][host] = new Pipe(_host_switch_latency, *_eventlist);
                _host_to_switch_pipes[i][j][host]->setName("Pipe_Host" + ntoa(host) + "->Switch(" +
                                                           ntoa(j) + ", " + ntoa(i) + ")");
                _switch_to_host_pipes[i][j][host]->setName("Pipe_Switch(" + ntoa(j) + ", " +
                                                           ntoa(i) + ")->Host" + ntoa(host));

                // Create queues
                hst_swi_q_logger =
                    (_q_logger_factory) ? _q_logger_factory->createQueueLogger() : nullptr;
                swi_hst_q_logger =
                    (_q_logger_factory) ? _q_logger_factory->createQueueLogger() : nullptr;

                _host_to_switch_queues[i][j][host] =
                    alloc_host_queue(hst_swi_q_logger, _linkspeed_bps);
                _switch_to_host_queues[i][j][host] =
                    alloc_queue(swi_hst_q_logger, _linkspeed_bps, _queue_size_b);
            }

            // Switch-switch networking
            for (uint32_t vc = 0; vc < _n_virtual_channels; vc++) {
                // --- Create Pipes ---
                if (is_constellation) {
                    _northbound_pipes[i][j][vc] =
                        new InterSatelliteLink(_n, _m, InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _eastbound_pipes[i][j][vc] =
                        new InterSatelliteLink(_n, _m, InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _southbound_pipes[i][j][vc] =
                        new InterSatelliteLink(_n, _m, InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _westbound_pipes[i][j][vc] =
                        new InterSatelliteLink(_n, _m, InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                } else {
                    _northbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                    _eastbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                    _southbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                    _westbound_pipes[i][j][vc] = new Pipe(_latency, *_eventlist);
                }
                // Set names
                _northbound_pipes[i][j][vc]->setName("NorthboundPipe_(" + ntoa(j) + ", " + ntoa(i) +
                                                     ")");
                _eastbound_pipes[i][j][vc]->setName("EastboundPipe_(" + ntoa(j) + ", " + ntoa(i) +
                                                    ")");
                _southbound_pipes[i][j][vc]->setName("SouthboundPipe_(" + ntoa(j) + ", " + ntoa(i) +
                                                     ")");
                _westbound_pipes[i][j][vc]->setName("WestboundPipe_(" + ntoa(j) + ", " + ntoa(i) +
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

                _north_egress_queues[i][j][vc]->setName("NorthEgressQueue_(" + ntoa(j) + ", " +
                                                        ntoa(i) + ")");
                _east_egress_queues[i][j][vc]->setName("EastEgressQueue_(" + ntoa(j) + ", " +
                                                       ntoa(i) + ")");
                _south_egress_queues[i][j][vc]->setName("SouthEgressQueue_(" + ntoa(j) + ", " +
                                                        ntoa(i) + ")");
                _west_egress_queues[i][j][vc]->setName("WestEgressQueue_(" + ntoa(j) + ", " +
                                                       ntoa(i) + ")");
                // Connect switch to these queues
                _switches[i][j]->addPort(_north_egress_queues[i][j][vc]);
                _switches[i][j]->addPort(_east_egress_queues[i][j][vc]);
                _switches[i][j]->addPort(_south_egress_queues[i][j][vc]);
                _switches[i][j]->addPort(_west_egress_queues[i][j][vc]);

                if (_switch_queue_type == LOSSLESS_INPUT ||
                    _switch_queue_type == LOSSLESS_INPUT_ECN) {
                    // Create an ingress queue for the switch opposite to appropriate egress switch
                    // E.g.: to NorthEgressQueue associate opposite Switch's SouthIngressQueue
                    // So determine coords of opposite switch
                    uint32_t n_i, n_j, e_i, e_j, s_i, s_j, w_i, w_j;

                    // Simulate wrap-around when network edge is reached
                    n_i = (i == 0) ? _n - 1 : i - 1;
                    n_j = j;
                    _south_ingress_queues[n_i][n_j][vc] = new LosslessInputQueue(
                        *_eventlist, _north_egress_queues[i][j][vc], _switches[n_i][n_j], _latency);
                    _south_ingress_queues[n_i][n_j][vc]->setName("SouthIngressQueue_(" + ntoa(n_i) +
                                                                 ", " + ntoa(n_j) + ")");
                    e_i = i;
                    e_j = (j == _m - 1) ? 0 : j + 1;
                    _west_ingress_queues[e_i][e_j][vc] = new LosslessInputQueue(
                        *_eventlist, _east_egress_queues[i][j][vc], _switches[e_i][e_j], _latency);
                    _west_ingress_queues[e_i][e_j][vc]->setName("WestIngressQueue_(" + ntoa(e_i) +
                                                                ", " + ntoa(e_j) + ")");
                    s_i = (i == _n - 1) ? 0 : i + 1;
                    s_j = j;
                    _north_ingress_queues[s_i][s_j][vc] = new LosslessInputQueue(
                        *_eventlist, _south_egress_queues[i][j][vc], _switches[s_i][s_j], _latency);
                    _north_ingress_queues[s_i][s_j][vc]->setName("NorthIngressQueue_(" + ntoa(s_i) +
                                                                 ", " + ntoa(s_j) + ")");
                    w_i = i;
                    w_j = (j == 0) ? _m - 1 : j - 1;
                    _east_ingress_queues[w_i][w_j][vc] = new LosslessInputQueue(
                        *_eventlist, _west_egress_queues[i][j][vc], _switches[w_i][w_j], _latency);
                    _east_ingress_queues[w_i][w_j][vc]->setName("EastIngressQueue_(" + ntoa(w_i) +
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

            // NOTE: Pipes are "connected" upon route creation, see get_bidir_paths
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

    delete_3d_vector(_host_to_switch_queues);
    delete_3d_vector(_host_to_switch_pipes);
    delete_3d_vector(_switch_to_host_queues);
    delete_3d_vector(_switch_to_host_pipes);

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

void TwoDimensionalTorusTopology::set_hosts_per_switch(uint32_t hosts_per_switch) {
    _hosts_per_switch = (hosts_per_switch == 0) ? 1 : hosts_per_switch;
}
void TwoDimensionalTorusTopology::set_routing_strategy(torus_routing_strategy rs) {
    _routing_strategy = rs;
}

void TwoDimensionalTorusTopology::set_n_virtual_channels() {
    // Two VCs for dimension-ordered, 3 for minimum adaptive
    _n_virtual_channels = (_routing_strategy == DIMENSION_ORDERED) ? 2 : 3;
}

void TwoDimensionalTorusTopology::set_queue_params(queue_type switch_queue_type,
                                                   queue_type host_queue_type,
                                                   mem_b queue_size,
                                                   bool disable_trim,
                                                   uint16_t trim_size,
                                                   mem_b ecn_high,
                                                   mem_b ecn_low) {
    _switch_queue_type = (switch_queue_type == UNDEFINED) ? LOSSLESS : switch_queue_type;
    _host_queue_type = (host_queue_type == UNDEFINED) ? FAIR_PRIO : host_queue_type;
    // Default to accomodating 10 packet per virtual channel
    _queue_size_b = (queue_size == 0) ? memFromPkt(10 * _n_virtual_channels) : queue_size;

    if (_switch_queue_type == COMPOSITE || _switch_queue_type == COMPOSITE_ECN) {
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

    // NOTE: since latency will still be used by lossless ingress queues' callback pipes, we cannot
    // leave it at 0. Instead, compute the fuxed intraplane latency and use that
    simtime_picosec latency = compute_average_interplane_latency_ps();
    set_switch_latency(latency);
}

void TwoDimensionalTorusTopology::set_switch_latency(simtime_picosec latency) {
    _latency = latency;
}

void TwoDimensionalTorusTopology::set_host_switch_latency(simtime_picosec latency) {
    _host_switch_latency = latency;
}

double square(double x) {
    return x * x;
}

/*
 * The distance formula used can be found in the paper "QoS-Aware Resource Placement For LEO
 * Satellite Edge Computing" by T. Pfandzelter and D. Bermbach.
 * Since it computes an average, it doesn't need additional input parameters.
 * It makes use of the complete elliptic integral of the second kind.
 */
simtime_picosec TwoDimensionalTorusTopology::compute_average_interplane_latency_ps() {
    double factor1 = sqrt(2 * (1 - cos(2 * PI / _n)));
    double factor2 = ellint_2l((1 - square(cos(_inclination_in_deg * PI / 180))), PI / 2);

    double avg_dist = (2 / PI) * (_altitude_in_m + R_EARTH_M) * factor1 * factor2;
    double latency_s = avg_dist / C_MPS;
    return timeFromSec(latency_s);
}

Queue* TwoDimensionalTorusTopology::alloc_host_queue(QueueLogger* queue_logger,
                                                     linkspeed_bps linkspeed_bps) {
    // NOTE: since these queues need to handle multiple senders at once, their size is considerably
    // larger than switch queues (at the time of writing, FEEDER_BUFFER=1000)
    switch (_host_queue_type) {
        case PRIORITY:
            // Priority queue with 3 levels, low, mid, and high. Heavily favours high priority
            // packets.
            return new PriorityQueue(linkspeed_bps, memFromPkt(FEEDER_BUFFER), *_eventlist,
                                     queue_logger);
        case FAIR_PRIO:
            // Similar to PriorityQueue, but handles packets more fairly
            return new FairPriorityQueue(linkspeed_bps, memFromPkt(FEEDER_BUFFER), *_eventlist,
                                         queue_logger);
        default:
            // Uncaught exception, will eventually lead to abort() but hopefully with error message
            // first
            throw runtime_error(
                "Error allocating host queue: queue type is invalid or hasn't been implemented");
    }
}

Queue* TwoDimensionalTorusTopology::alloc_queue(QueueLogger* queue_logger,
                                                linkspeed_bps linkspeed_bps,
                                                mem_b queue_size) {
    switch (_switch_queue_type) {
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
            // Queue that sets packets as high or low priority and serves them accordingly
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
            // Uncaught exception, will eventually lead to abort() but hopefully with error message
            // first
            throw runtime_error(
                "Error allocating switch queue: queue type is invalid or hasn't been implemented");
    }
}

/*
 * Since the minimal adaptive routing algorithm is based off dimension-ordered routing, we shall
 * output the DOR routes, going along X direction first and then on Y direction.
 * Furthermore, because the Pipes in the road represent virtual channels, the switches can choose
 * whether to use another virtual channel from the ones outputted here
 */
vector<const Route*>* TwoDimensionalTorusTopology::get_bidir_paths(uint32_t src,
                                                                   uint32_t dest,
                                                                   bool reverse) {
    vector<const Route*>* paths = new vector<const Route*>();
    Route *route_src_dest, *route_src_dest_2, *route_src_dest_3, *route_src_dest_4;
    // DOR admits one, two, or four minimal routes
    route_src_dest = new Route();
    route_src_dest_2 = nullptr;
    route_src_dest_3 = nullptr;
    route_src_dest_4 = nullptr;

    // Coords are saved in [i, j, host_position] == [y-dimension, x-dimension, host_position]
    vector<uint32_t> pkt_coords = get_coords_of_host[src];
    vector<uint32_t> dest_coords = get_coords_of_host[dest];

    // In dimension-ordered routing, we use two virtual channels per link.
    // Then, when routing a packet from (x,y) to (x', y'), we first route it over one dimension, in
    // our case over the X-dimension until the packet reaches coordinates (x', y) using VC0, and
    // only then we route it over the Y-dimension using VC1.
    // This ensures deadlock-free and minimal path routing.

    // X-dimension routing
    if (pkt_coords[1] != dest_coords[1]) {
        // Three possibilities:
        // 1. Packet needs to go east for min route (1 min route)
        // 2. Packet needs to go west for min route (1 min route)
        // 3. Packet can go east or west if routes are equidistant (2 min routes)

        // 1. Eastward case: (x < x' && wrapping around to the west is slower) || (x > x' &&
        // wrapping around to the east is faster)
        if (((pkt_coords[1] < dest_coords[1]) && ((dest_coords[1] - pkt_coords[1]) * 2 < _m)) ||
            ((pkt_coords[1] > dest_coords[1]) && ((pkt_coords[1] - dest_coords[1]) * 2 > _m))) {
            // Create single eastward route
            _add_eastward_route_to(route_src_dest, pkt_coords, dest_coords, reverse);
        }

        // 2. Westward case: (x > x' && wrapping around to the east is slower) || (x < x' &&
        // wrapping around to he west is faster)
        if (((pkt_coords[1] > dest_coords[1]) && ((pkt_coords[1] - dest_coords[1]) * 2 < _m)) ||
            ((pkt_coords[1] < dest_coords[1]) && ((dest_coords[1] - pkt_coords[1]) * 2 > _m))) {
            // Create single westward route
            _add_westward_route_to(route_src_dest, pkt_coords, dest_coords, reverse);
        }

        // 3. Two min routes: ( |x->x'| == |x'->x| == _m/2), can only happen when m is even
        if ((pkt_coords[1] - dest_coords[1] == _m / 2) ||
            (dest_coords[1] - pkt_coords[1] == _m / 2)) {
            // Add both east and west routes
            _add_x_routes_to(route_src_dest, route_src_dest_2, pkt_coords, dest_coords, reverse);
        }
    }

    // Y-dimension routing
    if (pkt_coords[0] != dest_coords[0]) {
        // Same logic as before, but with more subcases:
        // 1.1. Packet must go north, and we start with a single route (end with 1 min route)
        //   2. Packet must go north, and we start with two possible routes (end with 2 min routes)
        //
        // 2.1. Packet must go south, and we start with a single route (end with 1 min route)
        //   2. Packet must go south, and we start with two possible routes (end with 2 min routes)
        //
        // 3.1. Packet can go either way, and we start with a single route (end with 2 min routes)
        //   2. Packet can go either way, and we start with two possible routes (4 min routes)
        //
        // NOTE: we'll use route_src_dest_2's nullity as a proxy for if there's a second route

        // 1. Northward case: (y < y' && southward wrap-around is slower) || (y > y' &&
        // wrap-around north is faster)
        // Use ceiling as C++ truncates rather than round integers
        if (((pkt_coords[0] < dest_coords[0]) && ((dest_coords[0] - pkt_coords[0]) * 2 < _n)) ||
            ((pkt_coords[0] > dest_coords[0]) && ((pkt_coords[0] - dest_coords[0]) * 2 > _n))) {
            // 1. We start with a single route
            _add_northward_route_to(route_src_dest, pkt_coords, dest_coords, reverse);

            // 2. We start with two possible routes
            if (route_src_dest_2 != nullptr) {
                _add_northward_route_to(route_src_dest_2, pkt_coords, dest_coords, reverse);
            }
        }

        // 2. Southward case: (y > y' && wrap-around to the north is slower) || (y < y' &&
        // wrap-around to the south is faster)
        if (((pkt_coords[0] > dest_coords[0]) && ((pkt_coords[0] - dest_coords[0]) * 2 < _n)) ||
            ((pkt_coords[0] < dest_coords[0]) && ((dest_coords[0] - pkt_coords[0]) * 2 > _n))) {
            // 1. Single route
            _add_southward_route_to(route_src_dest, pkt_coords, dest_coords, reverse);
            if (route_src_dest_2 != nullptr) {
                // 2. Two routes
                _add_southward_route_to(route_src_dest_2, pkt_coords, dest_coords, reverse);
            }
        }

        // 3. Two min routes: (|y->y'| == |y-y'| == _n/2), can only happen when n is even
        if ((pkt_coords[0] - dest_coords[0] == _n / 2) ||
            (dest_coords[0] - pkt_coords[0] == _n / 2)) {
            // 1. Single route: copy route_src_dest into route3, so as to create one variant that
            // goes north and another that goes south
            route_src_dest_3 = route_src_dest->clone();
            // NOTE: clone() only makes a shallow copy of the reverse, meaning route and route3
            // would share the same reverse, which we don't want as each will go in a different
            // direction and have different reverse routes
            if (route_src_dest->reverse() != nullptr) {
                Route* rev_clone = route_src_dest->reverse()->clone();
                route_src_dest_3->set_reverse(rev_clone);
                rev_clone->set_reverse(route_src_dest_3);
            }
            _add_y_routes_to(route_src_dest, route_src_dest_3, pkt_coords, dest_coords, reverse);

            if (route_src_dest_2 != nullptr) {
                // 2. Two routes: copy route_src_dest_2 into route_src_dest_4 and do the same as
                // above
                route_src_dest_4 = route_src_dest_2->clone();
                if (route_src_dest_2->reverse() != nullptr) {
                    Route* rev_clone = route_src_dest_2->reverse()->clone();
                    route_src_dest_4->set_reverse(rev_clone);
                    rev_clone->set_reverse(route_src_dest_4);
                }
                _add_y_routes_to(route_src_dest_2, route_src_dest_4, pkt_coords, dest_coords,
                                 reverse);
            }
        }
    }

    paths->push_back(route_src_dest);
    if (route_src_dest_2 != nullptr) {
        paths->push_back(route_src_dest_2);
    }
    if (route_src_dest_3 != nullptr) {
        paths->push_back(route_src_dest_3);
        paths->push_back(route_src_dest_4);
    }

    return paths;
}

/*
 * Constructs eastward route on X-dimension from source's x to destination's x. This method
 * doesn't handle Y-dimension routing.
 * The construction follows dimension-ordered routing starting from the X-dimension, and so uses
 * VC0.
 */
void TwoDimensionalTorusTopology::_add_eastward_route_to(Route* route,
                                                         const vector<uint32_t>& src_coords,
                                                         const vector<uint32_t>& dest_coords,
                                                         bool reverse) {
    vector<uint32_t> pkt_coords = src_coords;
    vector<uint32_t> reverse_pkt_coords = dest_coords;

    Route* reverse_route = (reverse) ? new Route() : nullptr;

    // First push the host's egress queues into route to allow packets into switch.
    route->push_back(_host_to_switch_queues[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
    route->push_back(_host_to_switch_pipes[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
    if (reverse) {
        reverse_route->push_back(
            _host_to_switch_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                  [reverse_pkt_coords[2]]);
        reverse_route->push_back(_host_to_switch_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                                      [reverse_pkt_coords[2]]);
    }

    // Then move east
    while (pkt_coords[1] != dest_coords[1]) {
        // Keep moving east, using wrap-around if necessary
        route->push_back(_east_egress_queues[pkt_coords[0]][pkt_coords[1]][VC0]);
        route->push_back(_eastbound_pipes[pkt_coords[0]][pkt_coords[1]][VC0]);

        if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
            // Add matching lossless input queue (current queue's remote endpoint)
            route->push_back(
                _east_egress_queues[pkt_coords[0]][pkt_coords[1]][VC0]->getRemoteEndpoint());
        }
        // Increment x-coord with possible wrap-around
        pkt_coords[1] = (pkt_coords[1] == _m - 1) ? 0 : pkt_coords[1] + 1;

        if (reverse) {
            // Apply same logic but from dest to src, so reverse route
            reverse_route->push_back(
                _west_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC0]);
            reverse_route->push_back(
                _westbound_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC0]);
            if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
                reverse_route->push_back(
                    _west_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC0]
                        ->getRemoteEndpoint());
            }
            reverse_pkt_coords[1] =
                (reverse_pkt_coords[1] == 0) ? _m - 1 : reverse_pkt_coords[1] - 1;
        }
    }

    // If dest is reached, route packet from switch to host
    if (pkt_coords[0] == dest_coords[0]) {
        route->push_back(_switch_to_host_queues[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
        route->push_back(_switch_to_host_pipes[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
        if (reverse) {
            reverse_route->push_back(
                _switch_to_host_queues[src_coords[0]][src_coords[1]][src_coords[2]]);
            reverse_route->push_back(
                _switch_to_host_pipes[src_coords[0]][src_coords[1]][src_coords[2]]);
        }
    }

    if (reverse) {
        route->set_reverse(reverse_route);
        reverse_route->set_reverse(route);
    }
}

/*
 * Constructs westward route on X-dimension from source's x to destination's x. This method
 * doesn't handle Y-dimension routing.
 * The construction follows dimension-ordered routing starting from the X-dimension, and so uses
 * VC0.
 */
void TwoDimensionalTorusTopology::_add_westward_route_to(Route* route,
                                                         const vector<uint32_t>& src_coords,
                                                         const vector<uint32_t>& dest_coords,
                                                         bool reverse) {
    vector<uint32_t> pkt_coords = src_coords;
    vector<uint32_t> reverse_pkt_coords = dest_coords;

    Route* reverse_route = (reverse) ? new Route() : nullptr;

    // First push the host's egress queues into route to allow packets into switch.
    route->push_back(_host_to_switch_queues[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
    route->push_back(_host_to_switch_pipes[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
    if (reverse) {
        reverse_route->push_back(
            _host_to_switch_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                  [reverse_pkt_coords[2]]);
        reverse_route->push_back(_host_to_switch_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                                      [reverse_pkt_coords[2]]);
    }

    while (pkt_coords[1] != dest_coords[1]) {
        // Keep moving west, using wrap-around if necessary
        route->push_back(_west_egress_queues[pkt_coords[0]][pkt_coords[1]][VC0]);
        route->push_back(_westbound_pipes[pkt_coords[0]][pkt_coords[1]][VC0]);

        if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
            // Add matching lossless input queue (current queue's remote endpoint)
            route->push_back(
                _west_egress_queues[pkt_coords[0]][pkt_coords[1]][VC0]->getRemoteEndpoint());
        }
        // Decrement x-coord with possible wrap-around
        pkt_coords[1] = (pkt_coords[1] == 0) ? _m - 1 : pkt_coords[1] - 1;

        if (reverse) {
            // Apply same logic but from dest to src, so reverse route
            reverse_route->push_back(
                _east_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC0]);
            reverse_route->push_back(
                _eastbound_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC0]);
            if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
                reverse_route->push_back(
                    _east_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC0]
                        ->getRemoteEndpoint());
            }
            reverse_pkt_coords[1] =
                (reverse_pkt_coords[1] == _m - 1) ? 0 : reverse_pkt_coords[1] + 1;
        }
    }

    // If dest is reached, route packet from switch to host
    if (pkt_coords[0] == dest_coords[0]) {
        route->push_back(_switch_to_host_queues[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
        route->push_back(_switch_to_host_pipes[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
        if (reverse) {
            reverse_route->push_back(
                _switch_to_host_queues[src_coords[0]][src_coords[1]][src_coords[2]]);
            reverse_route->push_back(
                _switch_to_host_pipes[src_coords[0]][src_coords[1]][src_coords[2]]);
        }
    }

    if (reverse) {
        route->set_reverse(reverse_route);
        reverse_route->set_reverse(route);
    }
}

/*
 * This method is only used when source-destination routes are of the same length going east as
 * going west, so along the x-axis.
 */
void TwoDimensionalTorusTopology::_add_x_routes_to(Route* route1,
                                                   Route* route2,
                                                   const vector<uint32_t>& src_coords,
                                                   const vector<uint32_t>& dest_coords,
                                                   bool reverse) {
    _add_eastward_route_to(route1, src_coords, dest_coords, reverse);
    _add_westward_route_to(route2, src_coords, dest_coords, reverse);
}

void TwoDimensionalTorusTopology::_add_northward_route_to(Route* route,
                                                          const vector<uint32_t>& src_coords,
                                                          const vector<uint32_t>& dest_coords,
                                                          bool reverse) {
    vector<uint32_t> pkt_coords = src_coords;
    vector<uint32_t> reverse_pkt_coords = dest_coords;

    Route* reverse_route = nullptr;

    if (reverse) {
        // Reconstruct reverse route (can't use route->reverse() directly as it's a const)
        // NOTE: if src and dest have the same X coords, then route's reverse was never initialised
        reverse_route = (route->reverse() == nullptr) ? new Route() : route->reverse()->clone();
    }

    // If route wasn't initialised because x == x', do it here by pushing packets from host to
    // switch
    if (pkt_coords[1] == dest_coords[1]) {
        route->push_back(_host_to_switch_queues[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
        route->push_back(_host_to_switch_pipes[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
        if (reverse) {
            reverse_route->push_back(
                _host_to_switch_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                      [reverse_pkt_coords[2]]);
            reverse_route->push_back(
                _host_to_switch_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                     [reverse_pkt_coords[2]]);
        }
    }

    while (pkt_coords[0] != dest_coords[0]) {
        route->push_back(_north_egress_queues[pkt_coords[0]][pkt_coords[1]][VC1]);
        route->push_back(_northbound_pipes[pkt_coords[0]][pkt_coords[1]][VC1]);

        if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
            route->push_back(
                _north_egress_queues[pkt_coords[0]][pkt_coords[1]][VC1]->getRemoteEndpoint());
        }
        pkt_coords[0] = (pkt_coords[0] == 0) ? _n - 1 : pkt_coords[0] - 1;

        if (reverse) {
            reverse_route->push_back(
                _south_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC1]);
            reverse_route->push_back(
                _southbound_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC1]);
            if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
                reverse_route->push_back(
                    _south_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC1]
                        ->getRemoteEndpoint());
            }
            reverse_pkt_coords[0] =
                (reverse_pkt_coords[0] == _n - 1) ? 0 : reverse_pkt_coords[0] + 1;
        }
    }

    // Dest is reached, so route packet from switch to host
    route->push_back(_switch_to_host_queues[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
    route->push_back(_switch_to_host_pipes[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
    if (reverse) {
        reverse_route->push_back(
            _switch_to_host_queues[src_coords[0]][src_coords[1]][src_coords[2]]);
        reverse_route->push_back(
            _switch_to_host_pipes[src_coords[0]][src_coords[1]][src_coords[2]]);
    }

    // Overwrite reverse route to update
    if (reverse) {
        // Avoid memory leak, as we dereference the old reverse route and won't reference it again
        delete route->reverse();
        route->set_reverse(reverse_route);
        reverse_route->set_reverse(route);
    }
}

void TwoDimensionalTorusTopology::_add_southward_route_to(Route* route,
                                                          const vector<uint32_t>& src_coords,
                                                          const vector<uint32_t>& dest_coords,
                                                          bool reverse) {
    vector<uint32_t> pkt_coords = src_coords;
    vector<uint32_t> reverse_pkt_coords = dest_coords;

    Route* reverse_route = nullptr;

    if (reverse) {
        reverse_route = (route->reverse() == nullptr) ? new Route() : route->reverse()->clone();
    }

    // If route wasn't initialised because x == x', do it here by pushing packets from host to
    // switch
    if (pkt_coords[1] == dest_coords[1]) {
        route->push_back(_host_to_switch_queues[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
        route->push_back(_host_to_switch_pipes[pkt_coords[0]][pkt_coords[1]][pkt_coords[2]]);
        if (reverse) {
            reverse_route->push_back(
                _host_to_switch_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                      [reverse_pkt_coords[2]]);
            reverse_route->push_back(
                _host_to_switch_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]]
                                     [reverse_pkt_coords[2]]);
        }
    }

    while (pkt_coords[0] != dest_coords[0]) {
        route->push_back(_south_egress_queues[pkt_coords[0]][pkt_coords[1]][VC1]);
        route->push_back(_southbound_pipes[pkt_coords[0]][pkt_coords[1]][VC1]);

        if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
            route->push_back(
                _south_egress_queues[pkt_coords[0]][pkt_coords[1]][VC1]->getRemoteEndpoint());
        }
        pkt_coords[0] = (pkt_coords[0] == _n - 1) ? 0 : pkt_coords[0] + 1;

        if (reverse) {
            reverse_route->push_back(
                _north_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC1]);
            reverse_route->push_back(
                _northbound_pipes[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC1]);
            if (_switch_queue_type == LOSSLESS_INPUT || _switch_queue_type == LOSSLESS_INPUT_ECN) {
                reverse_route->push_back(
                    _north_egress_queues[reverse_pkt_coords[0]][reverse_pkt_coords[1]][VC1]
                        ->getRemoteEndpoint());
            }
            reverse_pkt_coords[0] =
                (reverse_pkt_coords[0] == 0) ? _n - 1 : reverse_pkt_coords[0] - 1;
        }
    }

    // Dest is reached, so route packet from switch to host
    route->push_back(_switch_to_host_queues[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
    route->push_back(_switch_to_host_pipes[dest_coords[0]][dest_coords[1]][dest_coords[2]]);
    if (reverse) {
        reverse_route->push_back(
            _switch_to_host_queues[src_coords[0]][src_coords[1]][src_coords[2]]);
        reverse_route->push_back(
            _switch_to_host_pipes[src_coords[0]][src_coords[1]][src_coords[2]]);
    }

    if (reverse) {
        delete route->reverse();
        route->set_reverse(reverse_route);
        reverse_route->set_reverse(route);
    }
}

void TwoDimensionalTorusTopology::_add_y_routes_to(Route* route1,
                                                   Route* route2,
                                                   const vector<uint32_t>& src_coords,
                                                   const vector<uint32_t>& dest_coords,
                                                   bool reverse) {
    _add_northward_route_to(route1, src_coords, dest_coords, reverse);
    _add_southward_route_to(route2, src_coords, dest_coords, reverse);
}

void TwoDimensionalTorusTopology::add_switch_loggers(Logfile& logfile,
                                                     simtime_picosec sample_period) {
    for (uint32_t i = 0; i < _n; i++) {
        for (uint32_t j = 0; j < _m; j++) {
            _switches[i][j]->add_logger(logfile, sample_period);
        }
    }
}

/*
 * Not implemented, as it is unclear what a host's neighbour is (linked to same switch, or hosts
 * linked to neighbouring switches?)
 */
vector<uint32_t>* TwoDimensionalTorusTopology::get_neighbours(uint32_t src) {
    return nullptr;
}
