/*

# BOB SLEEPS

This example shows a very simple demonstration of the core.
A agent named "Bob" sleeps and wakes up once in a while, then goes back to sleep.
If you wake Bob, he explains he just needs five more minutes to sleep.
Then, he goes back to sleep again.

A couple things you should notice:

- The data structure of sensors, actuators and memory must be defined.
- Concurrency and delays are by default, so extra steps are taken
  to make sure Bob interacts in a sequential way with the world.

*/


extern crate asi_core0 as asi;

use asi::{
    ActuatorId,
    MemoryId,
};

/// Bob's sensors.
///
/// He does not need any in this example.
pub enum Sensor {}

impl asi::Sensor<()> for Sensor {
    fn next(&mut self) {}
    fn try_receive(&mut self) -> Option<Result<(), Box<dyn ::std::error::Error>>> {None}
}

/// Bob's actuators.
pub enum Actuator {
    /// Bob tells the user he just needs five more minutes to sleep.
    ///
    /// The flag is used to confirm the action is done.
    INeedJustFiveMinutes(bool),
    /// Bob looks around to see if anyone are there.
    ///
    /// The flag is used to confirm the action is done.
    LookAround(bool),
}

impl asi::Actuator<()> for Actuator {
    fn send(&mut self, _: &()) {
        match *self {
            Actuator::INeedJustFiveMinutes(ref mut state) => {
                println!("Bob: I need just five minutes more to sleep.");
                *state = true;
            }
            Actuator::LookAround(ref mut state) => {
                println!("Bob: Uh? Anyone there?");
                *state = true;
            }
        }
    }
    fn try_confirm(&mut self) -> Option<Result<(), Box<dyn ::std::error::Error>>> {
        match *self {
            Actuator::INeedJustFiveMinutes(ref mut state) => {
                if *state == true {
                    *state = false;
                    Some(Ok(()))
                } else {
                    None
                }
            }
            Actuator::LookAround(ref mut state) => {
                if *state == true {
                    *state = false;
                    Some(Ok(()))
                } else {
                    None
                }
            }
        }
    }
}

pub enum Bob {
    Sleep,
    Disturbed,
    LookAround,
}

const I_NEED_JUST_FIVE_MORE_MINUTES: ActuatorId = ActuatorId(0);
const LOOK_AROUND: ActuatorId = ActuatorId(1);
const DUMMY_MEMORY: MemoryId = MemoryId(0);

// Use `()` for memory, since the only input we need is waiting period to be interrupted.
impl asi::DecisionMaker<()> for Bob {
    fn next_action(&mut self, _: &[()], _dt: f64) -> asi::Action {
        match *self {
            Bob::Sleep => {
                // Need to print it here since waiting does not do anything.
                println!("Bob: ZZZZZZZZZZZZ...");
                asi::Action::Wait(5.0)
            }
            Bob::Disturbed => asi::Action::Output {
                actuator: I_NEED_JUST_FIVE_MORE_MINUTES,
                memory: DUMMY_MEMORY,
            },
            Bob::LookAround => asi::Action::Output {
                actuator: LOOK_AROUND,
                memory: DUMMY_MEMORY,
            }
        }
    }

    fn feedback(&mut self, feedback: Result<asi::Feedback, asi::Error>) {
        match feedback {
            Ok(asi::Feedback::WaitingPeriodComplete) => {
                *self = Bob::LookAround;
            }
            Err(asi::Error::WaitingPeriodInterrupted {..}) => {
                *self = Bob::Disturbed
            }
            Ok(asi::Feedback::OutputReceived {
                actuator: a, memory: DUMMY_MEMORY
            }) if a == I_NEED_JUST_FIVE_MORE_MINUTES => {
                *self = Bob::Sleep;
            }
            Ok(asi::Feedback::OutputReceived {
                actuator: a, memory: DUMMY_MEMORY
            }) if a == LOOK_AROUND => {
                *self = Bob::Sleep;
            }
            x => {
                // Some unexpected feedback.
                println!("{:?}", x);
            }
        }
    }
    fn shut_down(&mut self, dt: f64) -> f64 {dt}
}

fn main() {
    use asi::Runtime;

    println!("Bob is sleeping.");
    println!("Enter - One time step forward");
    println!("'bye' - Exit");
    println!("'wake up!' - Wake up Bob");
    println!("");

    let ref mut runtime = asi::StandardRuntime::new();
    runtime.load(asi::Agent {
        // Try remove the `()` to cause invalid memory access.
        memory: vec![()],
        // Don't need any sensors in this example.
        sensors: Vec::<Sensor>::new(),
        actuators: vec![
            // Try comment this to write to wrong actuator.
            Actuator::INeedJustFiveMinutes(false),
            // Try comment this to cause invalid actuator access.
            Actuator::LookAround(false),
        ],
        // Make sure Bob takes sequential decisions,
        // because we don't want to think about concurrency.
        decision_maker: asi::Sequential::new(Bob::Sleep),
    });
    runtime.start();

    let stdin = ::std::io::stdin();
    loop {
        runtime.update(1.0);

        let mut input = String::new();
        stdin.read_line(&mut input).unwrap();
        input = input.trim().into();

        if input == "bye" {
            println!("Bob: Bye!");
            break;
        } else if input == "wake up!" {
            runtime.wake_up();
        }
    }
}
