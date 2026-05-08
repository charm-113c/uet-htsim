// -*- c-basic-offset: 4; indent-tabs-mode: nil -*-

// Shall we start with the constructor?
#include "2d_torus_topology.h"
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <vector>
#include "2d_torus_switch.h"
#include "config.h"
#include "eventlist.h"
#include "inter_satellite_link.h"
#include "logfile.h"
#include "loggers.h"
#include "loggertypes.h"
#include "queue.h"
#include "queue_lossless_input.h"
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
      _queue_type(queue_type::FAIR_PRIO),
      _queue_size_b(0),
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
                                                         torus_routing_strategy routing_strategy,
                                                         bool is_constellation,
                                                         simtime_picosec latency,
                                                         uint32_t altitude,
                                                         float_t inclination)
    : TwoDimensionalTorusTopology(logfile, q_log_factory, n, m, is_constellation) {
    // --- Set parameters
    set_n_virtual_channels(n_virtual_channels);
    set_queue_params(queue_type, queue_size);
    set_linkspeed(linkspeed);
    set_routing_strategy(routing_strategy);

    if (is_constellation) {
        set_constellation_params(altitude, inclination);
    } else {
        set_switch_latency(latency);
    }
    // ---

    // Allocating switches, queues, and pipes' vectors
    // NOTE: resize initialises vectors with values' default type, in this case nullptr
    // If things don't go right, explicitly set values to nullptr with loops here
    _switches.resize(_n, vector<Switch*>(_m));

    _egress_queues.resize(_n, vector<vector<Queue*>>(_m, vector<Queue*>(4)));
    _ingress_queues.resize(_n, vector<vector<Queue*>>(_m, vector<Queue*>(4)));

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
    QueueLogger* queue_logger;
    for (uint32_t i = 0; i < _n; i++) {
        for (uint32_t j = 0; j < _m; j++) {
            // Init queues and their loggers
            for (uint32_t dir = 0; dir < 4; dir++) {
                // Start from egress queue
                if (_q_logger_factory) {
                    queue_logger = _q_logger_factory->createQueueLogger();
                } else {
                    queue_logger = nullptr;
                }

                // TODO: REWORK NEEDED
                // Ingress queues are never used, except for the lossless input case. It's the
                // egress queues that do the lion share of the work in this simulation.
                // The only job of ingress queues is to store lossless input queues (something that
                // wasn't done in the fat tree implementation, likely because the lossless input
                // queue type was added after a first version of the fat tree was completed, and
                // keeping track of them would require doubling the number of queue vectors, which
                // is already significant).
                //
                // TODO: redo the part below, now that we actually know what we're doing.
                // Thus, in this topology, we allocate a queue type to _egress_queues
                // with alloc_queue, and then populate _ingress_queues with LosslessInputQueue only
                // if such queue types are used. Otherwise, they're left as nullptr.

                _egress_queues[i][j][dir] = alloc_queue(queue_logger);
                string dir_str, opp_dir_str;
                uint32_t opp_dir;
                switch (dir) {
                    case NORTH:
                        dir_str = "North";
                        opp_dir_str = "South";
                        opp_dir = SOUTH;
                        break;
                    case EAST:
                        dir_str = "East";
                        opp_dir_str = "West";
                        opp_dir = WEST;
                        break;
                    case SOUTH:
                        dir_str = "South";
                        opp_dir_str = "North";
                        opp_dir = NORTH;
                        break;
                    case WEST:
                        dir_str = "West";
                        opp_dir_str = "East";
                        opp_dir = EAST;
                        break;
                }
                _egress_queues[i][j][dir]->setName(dir_str + "EgressQueue_(" + ntoa(i) + ", " +
                                                   ntoa(j) + ")");
                // NOTE: assuming 4 ports per switch; could make #ports a variable in the future
                _switches[i][j]->addPort(_egress_queues[i][j][dir]);

                // To each _egress_queues create and associate corresponding ingress queue
                if (_q_logger_factory) {
                    queue_logger = _q_logger_factory->createQueueLogger();
                } else {
                    queue_logger = nullptr;
                }
                // Find opposite switch's coordinates
                uint32_t opp_i, opp_j;
                switch (dir) {
                    case NORTH:
                        opp_i = (i == 0) ? _n - 1 : i - 1;
                        opp_j = j;
                        break;
                    case EAST:
                        opp_i = i;
                        opp_j = (j == _m - 1) ? 0 : j + 1;
                        break;
                    case SOUTH:
                        opp_i = (i == _n - 1) ? 0 : i + 1;
                        opp_j = j;
                        break;
                    case WEST:
                        opp_i = i;
                        opp_j = (j == 0) ? _m - 1 : j - 1;
                        break;
                }

                // Create opposite switch's ingress queue
                if (_queue_type == LOSSLESS_INPUT || _queue_type == LOSSLESS_INPUT_ECN) {
                    // Overwrite ingress queue to be lossless queue
                    _ingress_queues[opp_i][opp_j][opp_dir] = new LosslessInputQueue(
                        *_eventlist, _egress_queues[i][j][dir], _switches[i][j], _latency);
                } else {
                    _ingress_queues[opp_i][opp_j][opp_dir] = alloc_queue(queue_logger);
                    // Set remote endpoint as switch
                    // NOTE: may actually not be used
                    _ingress_queues[opp_i][opp_j][opp_dir]->setRemoteEndpoint(_switches[i][j]);
                }
                _ingress_queues[opp_i][opp_j][opp_dir]->setName(
                    opp_dir_str + "IngressQueue_(" + ntoa(opp_i) + ", " + ntoa(opp_j) + ")");
                // TODO: check working of addPort, I doubt it takes both egress and ingress queues
                // at once
            }

            // TODO: Move Pipe creation in loop above to match their respective ports/queues,
            // consider cleaner way to handle virtual channels
            // NOTE: Actually, we may not need to do that, this might work as is. Consider this.
            // Create Pipes
            for (uint32_t k = 0; k < _n_virtual_channels; k++) {
                if (is_constellation) {
                    _northbound_pipes[i][j][k] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _eastbound_pipes[i][j][k] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _southbound_pipes[i][j][k] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTERPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                    _westbound_pipes[i][j][k] =
                        new InterSatelliteLink(InterSatelliteLink::ISL_Type::INTRAPLANE,
                                               _inclination_in_deg, _altitude_in_m);
                } else {
                    _northbound_pipes[i][j][k] = new Pipe(_latency, *_eventlist);
                    _eastbound_pipes[i][j][k] = new Pipe(_latency, *_eventlist);
                    _southbound_pipes[i][j][k] = new Pipe(_latency, *_eventlist);
                    _westbound_pipes[i][j][k] = new Pipe(_latency, *_eventlist);
                }
                // Set names
                _northbound_pipes[i][j][k]->setName("NorthboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                    ")");
                _eastbound_pipes[i][j][k]->setName("EastboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                   ")");
                _southbound_pipes[i][j][k]->setName("SouthboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                    ")");
                _westbound_pipes[i][j][k]->setName("WestboundPipe_(" + ntoa(i) + ", " + ntoa(j) +
                                                   ")");
            }

            // TODO: Deal with logging, is it automatic for Switches and Pipes or is a manual setup
            // needed?
            // TODO: Connect switches, queues and pipes
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

    delete_3d_vector(_egress_queues);
    delete_3d_vector(_ingress_queues);
    delete_3d_vector(_northbound_pipes);
    delete_3d_vector(_westbound_pipes);
    delete_3d_vector(_southbound_pipes);
    delete_3d_vector(_eastbound_pipes);
}

void TwoDimensionalTorusTopology::set_n_virtual_channels(uint32_t n) {
    n == 0 ? _n_virtual_channels = 2 : _n_virtual_channels = n;
}

void TwoDimensionalTorusTopology::set_queue_params(queue_type qt, mem_b qs) {
    qt ? _queue_type = qt : _queue_type = LOSSLESS;
    // TODO: find out more about queue size, since in here they set mysterious numbers like 8, or 7.
    // Furthermore, it appears that both in Infiniband and UEC, the focus is on a credit-based flow,
    // meaning buffer size is of low relevance, as senders must wait for buffer space to be
    // available
}

void TwoDimensionalTorusTopology::set_linkspeed(linkspeed_bps ls) {
    //! Default to 100Gbps
    ls == 0 ? _linkspeed_bps = 100000000000 : _linkspeed_bps = ls;
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

void TwoDimensionalTorusTopology::set_routing_strategy(torus_routing_strategy rs) {
    _routing_strategy = rs;
}

Queue* TwoDimensionalTorusTopology::alloc_queue(QueueLogger* queue_logger) {
    // TODO: figure out which queues fit our use case and implement only those
    return nullptr;
}
