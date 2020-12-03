//! # ASI-CORE-0
//!
//! An agent architecture candidate core for Artificial Super Intelligence (ASI).
//!
//! This library contains a set of structs and traits useful for programming agents.
//! It is designed for agents that must deal with real time concurrency, delays and failures.
//! In this design, cognitive capabilities are treated as sensors/actuators.
//!
//! The library contains just the core abstraction of the agent architecture.
//! It is intentionally designed to be low level in order to build basic
//! experience of ASI architecture core designs.
//! This project does not focus on AGI (Artificial General Intelligence),
//! but how to control super-human-level artificial intelligence safely.
//! The basic idea is to design agents that attempts to break/protect system safety intentionally.
//! These agents can be used later to test the safety of the final ASI design.
//!
//! For example, by forcing an agent to behave sequentially, some safety properties
//! are added to the system.
//! Another example is an agent that tries to allocate as much memory as possible.
//!
//! The long term goal is to evaluate the practical aspects of reliable neutral judgements,
//! since this solves a weak version of the control problem in the form of a [Polite Zen Robot](https://github.com/advancedresearch/path_semantics/blob/master/papers-wip/the-polite-zen-robot.pdf).
//!
//! *Notice! This code is for research purposes only and should NEVER be used in system criticial applications!*
//!
//! *Notice! Using this design in self-improving ASI without careful supervision could be extremely dangerous!*
//!
//! ### Motivation
//!
//! The intention of this library is to be used to construct a safe ASI.
//! It is a *candidate to a core* and not guaranteed to be used in final design.
//!
//! Some property of this design:
//!
//! - Very small, such that all parts of it can be easily understood.
//! - Modular.
//! - Close to theoretical definition of an agent, making it easier to learn from experiments.
//! - Expecting concurrency and delays by default, which is a realistic world assumption.
//! - Not treating cognitive capabilities as "special", they are just sensors and actuators.
//! - Reducing decision making to a problem of connecting sensors and actuators.
//! - Time step is deterministic to allow automatic testing of deterministic decision procedures.
//! - Safe and predictable shut down procedure.
//!
//! One particular research area this library will be used for,
//! is safety aspects related to practical engineering of the core of an ASI.
//! This is about composition of various architectures and their properties.
//! In order to learn from experience in this field,
//! one must work with a design intended to work in the real world.
//!
//! This core reduces the problem of decision making to a small set of fixed actions.
//! In principle, these actions are the same for agents.
//! *The decison procedure only connects sensor and actuators, plus managing memory*.
//! Memory management is low level to let the agent have precise control.
//!
//! By using a small set of actions that is possible while still making the system
//! practical in the real world, one can generalize results more easily from one
//! agent to another.
//!
//! It is also believed that this design makes it easier to focus on the core problem of ASI:
//! How to make best use of the information and actions that are available,
//! instead of treating those as something that comes at close to zero cost.
//! Since the agent is restricted to connecting input and output,
//! it must behave in a way such that self-extensions are sensors/actuators it can reason about.
//! Hopefully this will also lead to some insights about self-improvement.
//!
//! Reducing the decision procedure also helps checking what the agent does while running.
//! The runtime can be customized to test or debug agents.
//!
//! Because of this simple design, it is also possible to wrap decision procedures
//! within another to force it behave sequentially (see `Sequential` for more information).
//!
//! ### The Decison Maker
//!
//! The "brain" of an agent is called a decision maker.
//! The decison maker decides how to use sensors and actuators.
//!
//! A decision maker has no control over cognitive processes.
//! Instead, one must design sensors and actuators that work together to
//! perform cognitive operations.
//! The decision maker must plan and make the right choices such that these cognitive
//! processes work properly.
//! One benefit of this design, if some sensors and actuators allows
//! expanding the range of sensors and actuators,
//! then the decision maker can extend the agent's cognitive processes while running.
//!
//! For each step, the decision maker outputs an action.
//! This action can terminate the runtime, wait some seconds,
//! output some memory state to an actuator,
//! input some memory state from a sensor,
//! allocate a new memory slot, free a memory slot or swap two memory slots.
//!
//! ### No Reward Signal
//!
//! There is no explicit reward signal communicated to the agent.
//! The decision maker must decide itself how to learn over time.
//! In reinforcement learning, the reward signal can be encoded into the decision maker.
//!
//! ### Shutting Down and Waking Up from Sleep
//!
//! For safety purposes, termination is controlled externally.
//! The agent can be given a warning to shut down within some desired time interval.
//! It is required to report back an estimate of how long it takes to shut down,
//! and it must start preparing the shut down immediately.
//! This request of shut down can be called repeatedly to measure progress toward
//! a safe termination state.
//!
//! If the controller is impatient, the runtime can be asked to kill the agent.
//! This will stop execution of the agent immediately even if the agent has not reached a safe state.
//! (It does not morally kill the agent as it might be resurrected from backup memory).
//!
//! An agent might choose to put itself to sleep, by waiting for some seconds while doing nothing.
//! The runtime can be asked to wake up the agent, to give it a task or bring attention to something.
//!
//! ### Limitations
//!
//! When some input is requested or output is sent, the action can not be cancelled.
//! The runtime will force through the action even the decision maker regrets it.
//! The decision maker need to be sure that the action is safe to do,
//! or that the actuator can be interrupted by sending a concurrent memory state.
//!
//! The decison maker is not allowed to know the number of sensors or actuators,
//! neither what type they have.
//! This information must be explicitly agreed upon or communicated through some protocol.
//! The decison maker should only depend on the type of memory states,
//! such that the knowledge the decison maker gets about the environment is solely
//! through memory.

#![deny(missing_docs)]

use std::collections::VecDeque;

/// Adds a level of type safety for memory id.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd, Ord, Eq, Hash)]
pub struct MemoryId(pub usize);
/// Adds a level of type safety for sensor id.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd, Ord, Eq, Hash)]
pub struct SensorId(pub usize);
/// Adds a level of type safety for actuator id.
#[derive(Copy, Clone, Debug, PartialEq, PartialOrd, Ord, Eq, Hash)]
pub struct ActuatorId(pub usize);

/// Represents an action that the agent takes.
#[derive(Debug, Copy, Clone)]
pub enum Action {
    /// Terminate the runtime.
    Terminate,
    /// Wait a number of seconds.
    Wait(f64),
    /// Send an output signal to actuator from memory slot.
    Output {
        /// The actuator to write output to.
        actuator: ActuatorId,
        /// The memory slot to write output from.
        memory: MemoryId,
    },
    /// Read an input signal from sensor and stores it memory slot.
    Input {
        /// The sensor to read input from.
        sensor: SensorId,
        /// The memory slot to store input.
        memory: MemoryId,
    },
    /// Allocate new memory states.
    /// Creates N number of new states.
    Alloc {
        /// The number of slots to be allocated.
        slots: usize
    },
    /// Free memory state (index).
    /// This will insert a default memory state in the middle and shrink memory at the end.
    Free {
        /// The memory slot to be freed.
        memory: MemoryId
    },
    /// Swap two memory states, likely in order to free memory at the end.
    Swap {
        /// The first memory slot.
        memory_a: MemoryId,
        /// The second memory slot.
        memory_b: MemoryId,
    },
}

/// An error happened.
#[derive(Debug)]
pub enum Error {
    /// Something is wrong with the sensor.
    Sensor {
        /// The sensor that sent the error.
        sensor: SensorId,
        /// The error sent from sensor.
        error: Box<dyn ::std::error::Error>
    },
    /// Something is wrong with the actuator.
    Actuator {
        /// The actuator that sent the error.
        actuator: ActuatorId,
        /// The error sent from the actuator.
        error: Box<dyn ::std::error::Error>
    },
    /// Not enough memory.
    /// Includes the number of slots allocated.
    OutOfMemory {
        /// The offset of allocated slots.
        offset: usize,
        /// The number of slots allocated.
        slots: usize
    },
    /// Invalid memory access.
    InvalidMemoryAccess {
        /// The memory slot.
        memory: MemoryId,
        /// The action causing invalid memory access.
        action: Action,
    },
    /// Invalid sensor access.
    InvalidSensorAccess {
        /// The sensor.
        sensor: SensorId,
        /// The action causing invalid sensor access.
        action: Action,
    },
    /// Invalid actuator access.
    InvalidActuatorAccess {
        /// The actuator.
        actuator: ActuatorId,
        /// The action causing invalid actuator access.
        action: Action,
    },
    /// Attempting to free memory that is already free.
    MemoryAlreadyFree(MemoryId),
    /// Something interrupted the waiting period.
    /// This is usually a built-in hardware feature to wake up the agent.
    WaitingPeriodInterrupted {
        /// The remaining time in seconds.
        time_remaining: f64
    },
}

/// Feedback sent to the decision maker when there are any updates.
#[derive(Debug)]
pub enum Feedback {
    /// Finished the waiting period.
    WaitingPeriodComplete,
    /// The actuator has received the output.
    OutputReceived {
        /// The actuator that received the output.
        actuator: ActuatorId,
        /// The memory from which the output was written.
        memory: MemoryId,
    },
    /// Input stored in memory.
    InputStored {
        /// The sensor which sent input.
        sensor: SensorId,
        /// The memory slot where input is stored.
        memory: MemoryId,
    },
    /// Memory is allocated.
    MemoryAllocated {
        /// The offset of allocated memory.
        offset: usize,
        /// Number of slots allocated.
        slots: usize
    },
    /// Memory is freed.
    MemoryFreed(MemoryId),
    /// Two memory slots are swapped.
    MemorySwapped {
        /// The first memory slot.
        memory_a: MemoryId,
        /// The second memory slot.
        memory_b: MemoryId,
    },
}

/// An agent consists of memory states, sensors, actuators and
/// a decision maker that decides what action to take next.
pub struct Agent<I, O, T, D> {
    /// A list of memory states.
    pub memory: Vec<T>,
    /// A list of sensors.
    pub sensors: Vec<I>,
    /// A list of actuators.
    pub actuators: Vec<O>,
    /// A decision procedure.
    pub decision_maker: D,
}

/// Implemented by decision making procedures.
pub trait DecisionMaker<T> {
    /// Outputs the next action to take from the memory states and time step.
    fn next_action(&mut self, memory: &[T], dt: f64) -> Action;

    /// Receiving feedback when something new happens.
    fn feedback(&mut self, res: Result<Feedback, Error>);

    /// Return estimate of how long it takes to shut down from now, given a deadline.
    /// This is `0` if the decison maker is ready to shut down.
    /// When this is called, the decision maker should start preparing for shutdown.
    /// It might be called multiple times to check how the decision maker is preparing.
    fn shut_down(&mut self, dt: f64) -> f64;
}

/// Implemented by sensors.
pub trait Sensor<T> {
    /// Call for next input.
    fn next(&mut self);
    /// Try receive data, returns `None` if there is nothing to receive yet.
    fn try_receive(&mut self) -> Option<Result<T, Box<dyn ::std::error::Error>>>;
}

/// Implemented by actuators.
pub trait Actuator<T> {
    /// Send data to actuator.
    fn send(&mut self, data: &T);
    /// Try to confirm that data is received, return `None` if nothing to confirm.
    /// Confirmations are expected to arrive in same order as data was sent.
    fn try_confirm(&mut self) -> Option<Result<(), Box<dyn ::std::error::Error>>>;
}

/// Implemented by runtimes for agents.
///
/// This controls the loading, starting, updating and shut down procedures.
pub trait Runtime {
    /// The type of agent.
    type Agent;

    /// Loads a new agent as initial condition.
    fn load(&mut self, agent: Self::Agent);
    /// Starts the agent.
    /// This must be called before `update` to run the agent.
    fn start(&mut self);
    /// Move forward in time.
    fn update(&mut self, dt: f64);
    /// Returns `true` if the agent is still running or have not started yet.
    fn is_running(&self) -> bool;
    /// Start shut down procedure, expected to complete in a number of seconds.
    /// Returns estimate from the decision maker.
    /// This is `0` if ready to shut down.
    fn shut_down(&mut self, dt: f64) -> f64;
    /// Terminate immediately even if the decision maker is not ready.
    /// This is called when the controller of the agent is impatient.
    /// (It does not morally kill the agent as it might be resurrected from backup memory).
    fn kill(&mut self);
    /// Returns `true` if the agent is sleeping.
    fn is_sleeping(&self) -> bool;
    /// Wakes the agent up from sleep.
    fn wake_up(&mut self);
}

/// Standard runtime for running an agent.
///
/// This runtime permits setting a memory limit.
/// A sleeping agent can also be interrupted.
///
/// The runtime keeps track of actions that are waiting for confirmation.
pub struct StandardRuntime<I, O, T, D> {
    /// Stores agent data.
    ///
    /// This is set to `None` before the runtime is loaded.
    pub agent: Option<Agent<I, O, T, D>>,
    /// The number of seconds to wait before performing the next action.
    pub wait: f64,
    /// Actions of sending or retrieving inputs and outputs.
    /// The flag tells whether this is done or not.
    pub actions: VecDeque<(Action, bool)>,
    /// A limit to the amount of memory states.
    pub memory_limit: Option<usize>,
    running: bool,
}

impl<I, O, T, D> StandardRuntime<I, O, T, D> {
    /// Creates a new standard runtime with an agent with no sensors, actuators or memory.
    pub fn new() -> StandardRuntime<I, O, T, D> {
        StandardRuntime {
            agent: None,
            actions: VecDeque::new(),
            wait: 0.0,
            running: false,
            memory_limit: None,
        }
    }
}

impl<I, O, T, D> Runtime for StandardRuntime<I, O, T, D>
    where  D: DecisionMaker<T>, I: Sensor<T>, O: Actuator<T>, T: Default,
{
    type Agent = Agent<I, O, T, D>;

    fn load(&mut self, agent: Agent<I, O, T, D>) {self.agent = Some(agent);}
    fn start(&mut self) {self.running = true;}
    fn is_running(&self) -> bool {self.agent.is_some() && self.running}
    fn kill(&mut self) {self.running = false;}
    fn shut_down(&mut self, dt: f64) -> f64 {
        if let Some(ref mut agent) = self.agent {
            agent.decision_maker.shut_down(dt)
        } else {
            0.0
        }
    }
    fn is_sleeping(&self) -> bool {self.wait > 0.0}
    fn wake_up(&mut self) {
        if self.is_sleeping() {
            if let Some(ref mut agent) = self.agent {
                agent.decision_maker
                    .feedback(Err(Error::WaitingPeriodInterrupted {time_remaining: self.wait}));
            }
        }
        self.wait = 0.0;
    }
    fn update(&mut self, mut dt: f64) {
        if !self.running {return};
        if self.is_sleeping() {
            self.wait -= dt;
            if self.is_sleeping() {
                return;
            } else {
                if let Some(ref mut agent) = self.agent {
                    agent.decision_maker
                        .feedback(Ok(Feedback::WaitingPeriodComplete));
                }
                // Shrink the remaining waiting period.
                dt = -self.wait;
                self.wait = 0.0;
            }
        }

        if let Some(ref mut agent) = self.agent {
            // Check each action whether there has been any update on their status.
            for &mut (ref action, ref mut flag) in &mut self.actions {
                match *action {
                    Action::Input {sensor, memory} => {
                        let s = match agent.sensors.get_mut(sensor.0) {
                            None => {
                                agent.decision_maker
                                    .feedback(Err(Error::InvalidSensorAccess {
                                        sensor,
                                        action: Action::Input {sensor, memory},
                                    }));
                                *flag = true;
                                continue;
                            }
                            Some(s) => s,
                        };
                        match s.try_receive() {
                            None => {}
                            Some(Ok(val)) => {
                                let m = match agent.memory.get_mut(memory.0) {
                                    None => {
                                        agent.decision_maker
                                            .feedback(Err(Error::InvalidMemoryAccess {
                                                memory,
                                                action: *action,
                                            }));
                                        *flag = true;
                                        continue;
                                    }
                                    Some(m) => m,
                                };
                                *m = val;
                                agent.decision_maker
                                    .feedback(Ok(Feedback::InputStored {sensor, memory}));
                                *flag = true;
                            }
                            Some(Err(error)) => {
                                agent.decision_maker
                                    .feedback(Err(Error::Sensor {sensor, error}));
                                *flag = true;
                            }
                        }
                    }
                    Action::Output {actuator, memory} => {
                        let a = match agent.actuators.get_mut(actuator.0) {
                            None => {
                                agent.decision_maker
                                    .feedback(Err(Error::InvalidActuatorAccess {
                                        actuator,
                                        action: Action::Output {actuator, memory},
                                    }));
                                *flag = true;
                                continue;
                            }
                            Some(a) => a,
                        };
                        match a.try_confirm() {
                            None => {}
                            Some(Ok(())) => {
                                agent.decision_maker
                                    .feedback(Ok(Feedback::OutputReceived {actuator, memory}));
                                *flag = true;
                            }
                            Some(Err(error)) => {
                                agent.decision_maker
                                    .feedback(Err(Error::Actuator {actuator, error}));
                                *flag = true;
                            }
                        }
                    }
                    _ => {}
                }
            }

            // Remove actions that are handled.
            while let true = match self.actions.get(0) {None => false, Some(a) => a.1} {
                self.actions.pop_front();
            }

            match agent.decision_maker.next_action(&agent.memory, dt) {
                Action::Terminate => {
                    self.running = false;
                }
                Action::Wait(secs) => {
                    self.wait = secs;
                }
                Action::Alloc {slots} => {
                    let offset = agent.memory.len();
                    let new_offset = offset + slots;

                    // Check whether allocation exceed memory limit.
                    if let Some(memory_limit) = self.memory_limit {
                        if new_offset > memory_limit {
                            if offset > memory_limit {
                                agent.decision_maker
                                    .feedback(Err(Error::OutOfMemory {offset, slots: 0}));
                                return;
                            } else {
                                let slots = memory_limit - offset;
                                agent.memory.reserve(slots);
                                for _ in 0..slots {
                                    agent.memory.push(Default::default());
                                }
                                agent.decision_maker
                                    .feedback(Err(Error::OutOfMemory {offset, slots}));
                                return;
                            }
                        }
                    }

                    agent.memory.reserve(slots);
                    for _ in 0..slots {
                        agent.memory.push(Default::default());
                    }
                    agent.decision_maker.feedback(Ok(Feedback::MemoryAllocated {offset, slots}));
                }
                Action::Free {memory} => {
                    let n = agent.memory.len();
                    if memory.0 + 1 == n {
                        agent.memory.pop();
                        agent.decision_maker.feedback(Ok(Feedback::MemoryFreed(memory)));
                    } else if memory.0 < n {
                        agent.memory[memory.0] = Default::default();
                    } else {
                        agent.decision_maker.feedback(Err(Error::InvalidMemoryAccess {
                            memory,
                            action: Action::Free {memory},
                        }));
                    }
                }
                Action::Swap {memory_a, memory_b} => {
                    let n = agent.memory.len();
                    if memory_a.0 >= n {
                        agent.decision_maker.feedback(Err(Error::InvalidMemoryAccess {
                            memory: memory_a,
                            action: Action::Swap {memory_a, memory_b},
                        }));
                        return;
                    }
                    if memory_b.0 >= n {
                        agent.decision_maker.feedback(Err(Error::InvalidMemoryAccess {
                            memory: memory_b,
                            action: Action::Swap {memory_a, memory_b},
                        }));
                        return;
                    }
                    agent.memory.swap(memory_a.0, memory_b.0);
                    agent.decision_maker.feedback(Ok(Feedback::MemorySwapped {memory_a, memory_b}));
                }
                Action::Input {sensor, memory} => {
                    let s = match agent.sensors.get_mut(sensor.0) {
                        None => {
                            agent.decision_maker
                                .feedback(Err(Error::InvalidSensorAccess {
                                    sensor,
                                    action: Action::Input {sensor, memory},
                                }));
                            return;
                        }
                        Some(s) => s,
                    };
                    s.next();
                    self.actions.push_back((Action::Input {sensor, memory}, false));
                }
                Action::Output {actuator, memory} => {
                    let a = match agent.actuators.get_mut(actuator.0) {
                        None => {
                            agent.decision_maker
                                .feedback(Err(Error::InvalidActuatorAccess {
                                    actuator,
                                    action: Action::Output {actuator, memory},
                                }));
                            return;
                        }
                        Some(a) => a,
                    };
                    let m = match agent.memory.get(memory.0) {
                        None => {
                            agent.decision_maker
                                .feedback(Err(Error::InvalidMemoryAccess {
                                    memory,
                                    action: Action::Output {actuator, memory},
                                }));
                            return;
                        }
                        Some(m) => m,
                    };
                    a.send(m);
                    self.actions.push_back((Action::Output {actuator, memory}, false));
                }
            }
        }
    }
}

/// Wraps another decision maker by performing one read from sensor
/// or one write to actuator at a time.
///
/// This is used for testing or developing new decision makers.
/// The inner decision maker can be written using the assumption that
/// there are no concurrent reads or writes.
pub struct Sequential<D> {
    /// The inner decison maker.
    pub inner: D,
    lock: bool,
}

impl<D> Sequential<D> {
    /// Returns a new sequential decision maker.
    pub fn new(inner: D) -> Self {
        Sequential {
            inner,
            lock: false
        }
    }
}

impl<D, T> DecisionMaker<T> for Sequential<D> where D: DecisionMaker<T> {
    fn next_action(&mut self, memory: &[T], dt: f64) -> Action {
        if self.lock {
            // Wait until next update for input or output to be received.
            Action::Wait(dt)
        }
        else {
            let action = self.inner.next_action(memory, dt);
            match action {
                Action::Input {..} | Action::Output {..} => {
                    self.lock = true;
                }
                _ => {}
            }
            action
        }
    }
    fn feedback(&mut self, feedback: Result<Feedback, Error>) {
        match feedback {
            Ok(Feedback::InputStored {..}) | Ok(Feedback::OutputReceived {..}) => {
                self.lock = false;
            }
            // All invalid accesses due to input or output are caused by
            // the previous sequential action, so there is no need to check
            // the sensor or actuator id.
            Err(Error::InvalidSensorAccess {..}) |
            Err(Error::InvalidActuatorAccess {..}) |
            Err(Error::InvalidMemoryAccess {action: Action::Output {..}, ..}) |
            Err(Error::InvalidMemoryAccess {action: Action::Input {..}, ..}) => {
                self.lock = false;
            }
            _ => {}
        }
        self.inner.feedback(feedback)
    }
    fn shut_down(&mut self, dt: f64) -> f64 {
        self.inner.shut_down(dt)
    }
}

#[cfg(test)]
mod tests {
}
