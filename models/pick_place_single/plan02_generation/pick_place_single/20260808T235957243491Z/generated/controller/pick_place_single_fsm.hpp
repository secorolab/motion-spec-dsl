/*
 * This is an auto-generated file. Do not edit it directly.
 *
 * FSM: pick_place_single_fsm
 * FSM Description: Single-arm pick&place; path only home->pick_above, geometric setpoints elsewhere. Grasp-loss during transport (lift/place_above/place) routes to S_RECOVER, which captures and holds the current pose.
 *
 * -----------------------------------------------------
 * Usage example:
 * -----------------------------------------------------

#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "pick_place_single_fsm.hpp"

struct user_data {

};

void yyyy_behavior(struct user_data *userData, struct events *eventData) {
    // ... do something

    produce_event(eventData, pick_place_single_fsm::E_ZZZZ);
}

void fsm_behavior(struct events *eventData, struct user_data *userData) {
    if (consume_event(eventData, pick_place_single_fsm::E_XXXX)) {
        yyyy_behavior(userData, eventData);
    }
    ...
}

int main() {

    struct user_data userData = {};
    struct fsm_nbx *fsm = pick_place_single_fsm::create_fsm();
    if (!fsm) return 1;

    while (true) {
        produce_event(fsm->eventData, pick_place_single_fsm::E_STEP);

        // run state machine, event loop
        fsm_behavior(fsm->eventData, &userData);
        fsm_step_nbx(fsm);
        reconfig_event_buffers(fsm->eventData);
    }

    pick_place_single_fsm::destroy_fsm(fsm);
    return 0;
}

 * -----------------------------------------------------
 */

#ifndef PICK_PLACE_SINGLE_FSM_HPP
#define PICK_PLACE_SINGLE_FSM_HPP

#include "coord2b/types/fsm.h"
#include "coord2b/types/event_loop.h"
#include <new>


namespace pick_place_single_fsm {

struct fsm_nbx * create_fsm();
void destroy_fsm(struct fsm_nbx * fsm);

// sm states
enum e_states {
    S_START = 0,
    S_HOME,
    S_PICK_ABOVE,
    S_PICK,
    S_GRASP,
    S_LIFT,
    S_PLACE_ABOVE,
    S_PLACE,
    S_OPEN,
    S_RETREAT,
    S_DONE,
    S_RECOVER,
    NUM_STATES
};

/// This FSM's own IRI; the tables below name its parts.
static constexpr const char * FSM_URI = "https://secorolab.github.io/models/pick-place-single/fsm/pick_place_single_fsm";

static constexpr const char * STATE_URIS[NUM_STATES] = {
    "https://secorolab.github.io/models/pick-place-single/fsm/S_START",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_HOME",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_PICK_ABOVE",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_PICK",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_GRASP",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_LIFT",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_PLACE_ABOVE",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_PLACE",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_OPEN",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_RETREAT",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_DONE",
    "https://secorolab.github.io/models/pick-place-single/fsm/S_RECOVER",
};

// sm events
enum e_events {
    E_STEP = 0,
    E_PICK_ABOVE_READY,
    E_PICK_READY,
    E_GRASP_READY,
    E_LIFT_READY,
    E_PLACE_ABOVE_READY,
    E_PLACE_READY,
    E_OPEN_READY,
    E_RETREAT_READY,
    E_RETREAT_SETTLED,
    E_GRASP_LOST_LIFT,
    E_GRASP_LOST_PLACE_ABOVE,
    E_GRASP_LOST_PLACE,
    E_RECOVER,
    NUM_EVENTS
};

static constexpr const char * EVENT_URIS[NUM_EVENTS] = {
    "https://secorolab.github.io/models/pick-place-single/fsm/E_STEP",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_PICK_ABOVE_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_PICK_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_GRASP_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_LIFT_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_PLACE_ABOVE_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_PLACE_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_OPEN_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_RETREAT_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_RETREAT_SETTLED",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_GRASP_LOST_LIFT",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_GRASP_LOST_PLACE_ABOVE",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_GRASP_LOST_PLACE",
    "https://secorolab.github.io/models/pick-place-single/fsm/E_RECOVER",
};

// sm transitions
enum e_transitions {
    T_START_HOME = 0,
    T_HOME_PICK_ABOVE,
    T_PICK_ABOVE_PICK,
    T_PICK_GRASP,
    T_GRASP_LIFT,
    T_LIFT_PLACE_ABOVE,
    T_PLACE_ABOVE_PLACE,
    T_PLACE_OPEN,
    T_OPEN_RETREAT,
    T_RETREAT_DONE,
    T_LIFT_RECOVER,
    T_PLACE_ABOVE_RECOVER,
    T_PLACE_RECOVER,
    T_RECOVER_HOLD,
    NUM_TRANSITIONS
};

static constexpr const char * TRANSITION_URIS[NUM_TRANSITIONS] = {
    "https://secorolab.github.io/models/pick-place-single/fsm/T_START_HOME",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_HOME_PICK_ABOVE",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_PICK_ABOVE_PICK",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_PICK_GRASP",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_GRASP_LIFT",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_LIFT_PLACE_ABOVE",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_PLACE_ABOVE_PLACE",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_PLACE_OPEN",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_OPEN_RETREAT",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_RETREAT_DONE",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_LIFT_RECOVER",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_PLACE_ABOVE_RECOVER",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_PLACE_RECOVER",
    "https://secorolab.github.io/models/pick-place-single/fsm/T_RECOVER_HOLD",
};

// sm reactions
enum e_reactions {
    R_STEP_START = 0,
    R_PICK_ABOVE_READY,
    R_PICK_READY,
    R_GRASP_READY,
    R_LIFT_READY,
    R_PLACE_ABOVE_READY,
    R_PLACE_READY,
    R_OPEN_READY,
    R_RETREAT_READY,
    R_RETREAT_SETTLED,
    R_LIFT_GRASP_LOST,
    R_PLACE_ABOVE_GRASP_LOST,
    R_PLACE_GRASP_LOST,
    R_RECOVER_HOLD,
    NUM_REACTIONS
};

static constexpr const char * REACTION_URIS[NUM_REACTIONS] = {
    "https://secorolab.github.io/models/pick-place-single/fsm/R_STEP_START",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_PICK_ABOVE_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_PICK_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_GRASP_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_LIFT_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_PLACE_ABOVE_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_PLACE_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_OPEN_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_RETREAT_READY",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_RETREAT_SETTLED",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_LIFT_GRASP_LOST",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_PLACE_ABOVE_GRASP_LOST",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_PLACE_GRASP_LOST",
    "https://secorolab.github.io/models/pick-place-single/fsm/R_RECOVER_HOLD",
};

inline struct fsm_nbx * create_fsm() {

    struct fsm_nbx * fsm   = new (std::nothrow) fsm_nbx{
        .numReactions      = NUM_REACTIONS,
        .numTransitions    = NUM_TRANSITIONS,
        .numStates         = NUM_STATES,
        .states            = nullptr,
        .startStateIndex   = S_START,
        .endStateIndex     = S_DONE,
        .currentStateIndex = S_START,
        .eventData         = nullptr,
        .reactions         = nullptr,
        .transitions       = nullptr
    };
    if (!fsm) return nullptr;

    // sm states
    struct state * states = new (std::nothrow) state[NUM_STATES]{
        {.name = "S_start"},
        {.name = "S_home"},
        {.name = "S_pick_above"},
        {.name = "S_pick"},
        {.name = "S_grasp"},
        {.name = "S_lift"},
        {.name = "S_place_above"},
        {.name = "S_place"},
        {.name = "S_open"},
        {.name = "S_retreat"},
        {.name = "S_done"},
        {.name = "S_recover"}
    };

    // sm transition table
    struct transition * transitions = new (std::nothrow) transition[NUM_TRANSITIONS]{
        {
            .startStateIndex = S_START,
            .endStateIndex   = S_HOME,
        },
        {
            .startStateIndex = S_HOME,
            .endStateIndex   = S_PICK_ABOVE,
        },
        {
            .startStateIndex = S_PICK_ABOVE,
            .endStateIndex   = S_PICK,
        },
        {
            .startStateIndex = S_PICK,
            .endStateIndex   = S_GRASP,
        },
        {
            .startStateIndex = S_GRASP,
            .endStateIndex   = S_LIFT,
        },
        {
            .startStateIndex = S_LIFT,
            .endStateIndex   = S_PLACE_ABOVE,
        },
        {
            .startStateIndex = S_PLACE_ABOVE,
            .endStateIndex   = S_PLACE,
        },
        {
            .startStateIndex = S_PLACE,
            .endStateIndex   = S_OPEN,
        },
        {
            .startStateIndex = S_OPEN,
            .endStateIndex   = S_RETREAT,
        },
        {
            .startStateIndex = S_RETREAT,
            .endStateIndex   = S_DONE,
        },
        {
            .startStateIndex = S_LIFT,
            .endStateIndex   = S_RECOVER,
        },
        {
            .startStateIndex = S_PLACE_ABOVE,
            .endStateIndex   = S_RECOVER,
        },
        {
            .startStateIndex = S_PLACE,
            .endStateIndex   = S_RECOVER,
        },
        {
            .startStateIndex = S_RECOVER,
            .endStateIndex   = S_RECOVER,
        }
    };

    // sm reaction table
    struct event_reaction * reactions = new (std::nothrow) event_reaction[NUM_REACTIONS]{
        {
            .conditionEventIndex = E_STEP,
            .transitionIndex     = T_START_HOME,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_PICK_ABOVE_READY,
            .transitionIndex     = T_HOME_PICK_ABOVE,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_PICK_READY,
            .transitionIndex     = T_PICK_ABOVE_PICK,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_GRASP_READY,
            .transitionIndex     = T_PICK_GRASP,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_LIFT_READY,
            .transitionIndex     = T_GRASP_LIFT,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_PLACE_ABOVE_READY,
            .transitionIndex     = T_LIFT_PLACE_ABOVE,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_PLACE_READY,
            .transitionIndex     = T_PLACE_ABOVE_PLACE,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_OPEN_READY,
            .transitionIndex     = T_PLACE_OPEN,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_RETREAT_READY,
            .transitionIndex     = T_OPEN_RETREAT,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_RETREAT_SETTLED,
            .transitionIndex     = T_RETREAT_DONE,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_GRASP_LOST_LIFT,
            .transitionIndex     = T_LIFT_RECOVER,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_GRASP_LOST_PLACE_ABOVE,
            .transitionIndex     = T_PLACE_ABOVE_RECOVER,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_GRASP_LOST_PLACE,
            .transitionIndex     = T_PLACE_RECOVER,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        },
        {
            .conditionEventIndex = E_RECOVER,
            .transitionIndex     = T_RECOVER_HOLD,
            .numFiredEvents      = 0,
            .firedEventIndices   = nullptr,
        }};

    if (!states || !transitions || !reactions) {
        delete[] states;
        delete[] transitions;
        delete[] reactions;
        delete fsm;
        return nullptr;
    }

    for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
        if (reactions[i].numFiredEvents > 0 && !reactions[i].firedEventIndices) {
            for (unsigned int j = 0; j < NUM_REACTIONS; ++j) {
                delete[] reactions[j].firedEventIndices;
            }
            delete[] reactions;
            delete[] transitions;
            delete[] states;
            delete fsm;
            return nullptr;
        }
    }

    // sm event data
    struct events * eventData = new (std::nothrow) events{};
    _Bool * currentEvents = new (std::nothrow) _Bool[NUM_EVENTS]{false};
    _Bool * futureEvents = new (std::nothrow) _Bool[NUM_EVENTS]{false};
    if (!eventData || !currentEvents || !futureEvents) {
        delete[] states;
        delete[] transitions;
        if (reactions) {
            for (unsigned int i = 0; i < NUM_REACTIONS; ++i) {
                delete[] reactions[i].firedEventIndices;
            }
        }
        delete[] reactions;
        delete[] currentEvents;
        delete[] futureEvents;
        delete eventData;
        delete fsm;
        return nullptr;
    }
    eventData->numEvents     = NUM_EVENTS;
    eventData->currentEvents = currentEvents;
    eventData->futureEvents  = futureEvents;

    // sm fsm struct
    fsm->states      = states;
    fsm->eventData   = eventData;
    fsm->reactions   = reactions;
    fsm->transitions = transitions;

    return fsm;
}

inline void destroy_fsm(struct fsm_nbx * fsm) {
    if (!fsm) return;
    if (fsm->reactions) {
        for (unsigned int i = 0; i < fsm->numReactions; ++i) {
            delete[] fsm->reactions[i].firedEventIndices;
            fsm->reactions[i].firedEventIndices = nullptr;
            fsm->reactions[i].numFiredEvents = 0;
        }
    }
    if (fsm->eventData) {
        delete[] fsm->eventData->currentEvents;
        delete[] fsm->eventData->futureEvents;
        delete fsm->eventData;
        fsm->eventData = nullptr;
    }
    delete[] fsm->reactions;
    delete[] fsm->transitions;
    delete[] fsm->states;
    delete fsm;
}

} // namespace pick_place_single_fsm

#endif // PICK_PLACE_SINGLE_FSM_HPP